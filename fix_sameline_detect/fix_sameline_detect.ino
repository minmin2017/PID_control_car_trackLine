#include <Arduino.h>
#include <WiFi.h>
#include "Communication_order.h"

LanCommand net;
String  path_map = ""; 
//ex
// path += 'F';
// path += 'R';
// path += 'U';
// ---------------- WiFi ----------------
const char* ssid     = "MinMin2017";
const char* password = "minmin2017i";
const bool wifi_enable = true;

// ================== DRV8833 PIN MAP ==================
static const int AIN1 = 27;
static const int AIN2 = 14;
static const int EN   = 12;
static const int BIN1 = 26;
static const int BIN2 = 25;

// ================== IR SENSOR PINS ==================
static const int S1 = 34;
static const int S2 = 35;
static const int S3 = 32;
static const int S4 = 33;

// ================== PID GAINS ==================
float Kp = 30.0f;
float Ki = 0.0f;   // [FIX v1] ลดจาก 50 -> 0 ก่อน tune ใหม่
float Kd = 0.0f;

// ================== SPEED SETTINGS ==================
int baseSpeed = 120;
int maxSpeed  = 200;
const int TURN_PWM = 110;
// ================== SENSOR AUTO-CAL ==================
int sMin[4] = {4095,4095,4095,4095};
int sMax[4] = {0,0,0,0};

// ================== PID STATE ==================
float integral  = 0.0f;
float prevError = 0.0f;
unsigned long prevMs = 0;

// ================== STATE MACHINE ==================
enum RunState { FOLLOW, UTURN, STRAIGHT_BEFORE_RTURN, RTURN };
RunState runState = FOLLOW;

unsigned long actionStartMs = 0;


const unsigned long STRAIGHT_MS = 525;

// debounce เข้า state พิเศษ
int confirmLow = 0, confirmHigh = 0;
const int CONFIRM_N = 1;

// ================== [FIX v2] TURN STOP LOGIC ==================
// เปลี่ยนจาก count-based -> millis-based
// หุ่นต้องเห็น centerHigh ต่อเนื่องนาน STOP_HOLD_MS ms จึงหยุด
// กัน spike / กระพริบจาก sensor ขณะเลี้ยวผ่านเส้น

bool turnArmed          = false;  // true หลังหุ่นหลุดเส้นแล้ว
unsigned long centerHighStartMs = 0;  // จับเวลาเริ่มเห็น centerHigh
const unsigned long STOP_HOLD_MS = 1; // ต้องเห็นเส้นนิ่งกี่ ms จึงหยุด
                                       // ลองปรับ 30-80 ms ถ้ายังไม่ดี

// ================== UTILITY ==================
static inline int clampi(int v, int lo, int hi){
  if(v < lo) return lo;
  if(v > hi) return hi;
  return v;
}

// ================== RESET PID ==================
// [FIX v1] เรียกทุกครั้งกลับเข้า FOLLOW
void resetPID(){
  integral        = 0.0f;
  prevError       = 0.0f;
  prevMs          = millis();
}

// ================== RESET TURN STOP ==================
// [FIX v2] เรียกตอนเข้า UTURN / RTURN ใหม่
void resetTurnStop(){
  turnArmed         = false;
  centerHighStartMs = 0;
}
void addToPath(char c) {
    if (path_map.length() == 0 || path_map[path_map.length() - 1] != c) {
        path_map += c;
    }
}
// ================== MOTOR CONTROL ==================
void motorA(int speed){
  speed = clampi(speed, -255, 255);
  if(speed >= 0){ analogWrite(AIN1, speed); analogWrite(AIN2, 0); }
  else           { analogWrite(AIN1, 0);    analogWrite(AIN2, -speed); }
}

void motorB(int speed){
  speed = clampi(speed, -255, 255);
  if(speed >= 0){ analogWrite(BIN1, speed); analogWrite(BIN2, 0); }
  else           { analogWrite(BIN1, 0);    analogWrite(BIN2, -speed); }
}

// ================== SENSOR READ ==================
int readNorm(int pin, int idx){
  int raw = analogRead(pin);
  if(raw < sMin[idx]) sMin[idx] = raw;
  if(raw > sMax[idx]) sMax[idx] = raw;
  int den = (sMax[idx] - sMin[idx]);
  if(den < 10) den = 10;
  int norm = (raw - sMin[idx]) * 1000 / den;
  return clampi(norm, 0, 1000);
}

// ================== LINE ERROR ==================
float computeError(int n1, int n2, int n3, int n4){
  const float w1=-3, w2=-1, w3=1, w4=3;
  float sum = (float)n1+n2+n3+n4;
  if(sum < 1) sum = 1;
  return (w1*n1 + w2*n2 + w3*n3 + w4*n4) / sum;
}

// ================== CALIBRATE ==================
void calibrate(unsigned long ms = 2000){
  unsigned long t0 = millis();
  while(millis() - t0 < ms){
    (void)readNorm(S1,0); (void)readNorm(S2,1);
    (void)readNorm(S3,2); (void)readNorm(S4,3);
    delay(5);
  }
}

static inline void sendToClient(const char* msg){
  if(wifi_enable && net.hasClient()) net.clientRef().println(msg);
}

// ================== SEND DEBUG (all states) ==================
void sendDebug(int n1, int n2, int n3, int n4,
               int st, bool aL, bool aH, bool cH,
               unsigned long now){
  char dbg[120];
  snprintf(dbg, sizeof(dbg),
    "D:%d,%d,%d,%d,st=%d,aL=%d,aH=%d,cH=%d,arm=%d,hMs=%d",
    n1, n2, n3, n4, st, aL, aH, cH,
    turnArmed,
    (int)(centerHighStartMs > 0 ? (now - centerHighStartMs) : 0));
  Serial.println(dbg);
  sendToClient(dbg);
}

// ================== [FIX v2] TURN STOP HANDLER ==================
// ใช้ร่วมกันระหว่าง UTURN และ RTURN
// return true เมื่อควรหยุด
bool checkTurnStop(bool centerHigh, unsigned long now){
  // phase 1: รอให้หลุดเส้นก่อน
  if(!turnArmed){ // turnArmed == false
    if(!centerHigh) turnArmed = true; //center = ที่ว่าง 
    return false;
  }

  // phase 2: armed แล้ว เจอ centerHigh ปุ๊บ หยุดเลย
  return centerHigh;
}

// ================== SETUP ==================
void setup(){
  Serial.begin(115200);

  if(wifi_enable){
    bool ok = net.begin(ssid, password);
    if(!ok){ Serial.println("WiFi failed"); while(1) delay(1000); }
    Serial.print("IP = "); Serial.println(WiFi.localIP());
  }

  pinMode(EN, OUTPUT); digitalWrite(EN, HIGH);
  pinMode(AIN1,OUTPUT); pinMode(AIN2,OUTPUT);
  pinMode(BIN1,OUTPUT); pinMode(BIN2,OUTPUT);
  pinMode(S1,INPUT); pinMode(S2,INPUT);
  pinMode(S3,INPUT); pinMode(S4,INPUT);

  analogReadResolution(12);
  analogSetAttenuation(ADC_11db);

  motorA(0); motorB(0);

  Serial.println("Calibrating...");
  calibrate(2000);
  Serial.println("Done.");

  resetPID();
  motorA(baseSpeed); motorB(baseSpeed);
}

// ================== LOOP ==================
int first_time = 0;
void loop(){
  if(wifi_enable) net.update();
  if(first_time == 0){ sendToClient("F"); first_time = 1; }

  unsigned long now = millis(); // [FIX v1] ย้ายขึ้นต้น loop

  int n1 = readNorm(S1,0);
  int n2 = readNorm(S2,1);
  int n3 = readNorm(S3,2);
  int n4 = readNorm(S4,3);

  bool allLow  = (n1<=300 && n2<=300 && n3<=300 && n4<=300);
  bool allHigh = (n1>=800 && n2>=800 && n3>=800 && n4>=800);
  bool centerHigh = (n2>=800 ||  n3>=800);

  // ================== STATE MACHINE ==================
  switch(runState){

    case FOLLOW: {
      confirmLow  = allLow  ? (confirmLow+1)  : 0;
      confirmHigh = allHigh ? (confirmHigh+1) : 0;

      if(confirmHigh >= CONFIRM_N){ // R 
        resetTurnStop();
        runState = STRAIGHT_BEFORE_RTURN;
        confirmLow = confirmHigh = 0;
        actionStartMs = now;
        motorA(baseSpeed); motorB(baseSpeed);
        sendToClient("S");
        //addToPath('F');
        prevMs = now;
        return;
      }

      if(confirmLow >= CONFIRM_N){ // U 
        runState = UTURN;
        confirmLow = confirmHigh = 0;
        resetTurnStop(); // [FIX v2]
        motorA( TURN_PWM);
        motorB(-TURN_PWM);
        sendToClient("U");
        addToPath('U');
        prevMs = now;
        return;
      }
      break;
    }

    case STRAIGHT_BEFORE_RTURN: {
      motorA(baseSpeed); motorB(baseSpeed);
      if(now - actionStartMs >= STRAIGHT_MS){
        runState = RTURN;
        resetTurnStop(); // [FIX v2]
        motorA(TURN_PWM);
        motorB(0);
        sendToClient("R");
        addToPath('R');
      }
      sendDebug(n1,n2,n3,n4,(int)runState,allLow,allHigh,centerHigh,now);
      prevMs = now;
      return;
    }

    case UTURN: {
      motorA( TURN_PWM);
      motorB(-TURN_PWM);

      if(checkTurnStop(centerHigh, now)){ // [FIX v2]
        motorA(0); motorB(0);
        runState = FOLLOW;
        resetPID(); // [FIX v1]
      
      }
      sendDebug(n1,n2,n3,n4,(int)runState,allLow,allHigh,centerHigh,now);
      prevMs = now;
      return;
    }

    case RTURN: {
      motorA(TURN_PWM);
      motorB(0);

      if(checkTurnStop(centerHigh, now) && (now-actionStartMs) >= STRAIGHT_MS+500 ){ // [FIX v2]
        motorA(0); motorB(0);
        runState = FOLLOW;
        resetPID(); // [FIX v1]
        
      }else{
        Serial.println(now-actionStartMs);
      }
      sendDebug(n1,n2,n3,n4,(int)runState,allLow,allHigh,centerHigh,now);
      prevMs = now;
      return;
    }
  }

  // ================== PID FOLLOW ==================
  float error = computeError(n1, n2, n3, n4);

  float dt = (now - prevMs) / 1000.0f;
  if(dt <= 0) dt = 0.001f;
  prevMs = now;

  integral += error * dt;
  integral = constrain(integral, -1.0f, 1.0f);

  float derivative = (error - prevError) / dt;
  prevError = error;

  float corr = Kp*error + Ki*integral + Kd*derivative;

  int left  = clampi((int)(baseSpeed + corr), 0, maxSpeed);
  int right = clampi((int)(baseSpeed - corr), 0, maxSpeed);

  motorA(left);
  motorB(right);

  // debug (Serial + WiFi)
  char dbg[120];
  snprintf(dbg, sizeof(dbg),
    "D:%d,%d,%d,%d,st=%d,aL=%d,aH=%d,cH=%d,arm=%d,hMs=%d,e=%.3f,c=%.2f,L=%d,R=%d",
    n1, n2, n3, n4,
    (int)runState, allLow, allHigh, centerHigh,
    turnArmed,
    (int)(centerHighStartMs > 0 ? (now - centerHighStartMs) : 0),
    error, corr, left, right);
  Serial.println(dbg);
  if(wifi_enable){ char buf[122]; snprintf(buf, sizeof(buf), "%s\n", dbg); sendToClient(buf); }
}