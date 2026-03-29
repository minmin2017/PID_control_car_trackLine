#include <Arduino.h>
#include <WiFi.h>
#include "Communication_order.h"

LanCommand net;
String  path_map = ""; 

const char* ssid     = "MinMin2017";
const char* password = "minmin2017i";
const bool wifi_enable = false;

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
float Ki = 0.0f;
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
enum RunState { 
  FOLLOW, UTURN, STRAIGHT_BEFORE_RTURN, RTURN,
  // ========== [NEW] ==========
  STRAIGHT_CHECK,   // เดินตรงหลัง allLow เพื่อหา checkpoint
  REVERSE_BACK,     // ถอยหลังกลับ
  UTURNTHEN         // กลับรถหลังถอย (ก่อนเดินต่อ / หรือหยุดถ้า C)
};
RunState runState = FOLLOW;

unsigned long actionStartMs = 0;
const unsigned long STRAIGHT_MS = 525;

int confirmLow = 0, confirmHigh = 0;
const int CONFIRM_N = 1;

bool turnArmed          = false;
unsigned long centerHighStartMs = 0;
const unsigned long STOP_HOLD_MS = 1;

// ========== [NEW] CHECKPOINT VARIABLES ==========
const unsigned long STRAIGHT_CHECK_MS = 300; // เวลาเดินตรงหลัง allLow (ปรับได้)
const unsigned long UTURN_AFTER_REVERSE_MS = 300; // เวลากลับรถ (ปรับได้)

int  checkpointCount  = 0;        // 0=ยังไม่เจอ, 1=เจอ A, 2=เจอ B, 3=เจอ C
unsigned long straightCheckStart = 0;  // เวลาเริ่มเดินตรงใน STRAIGHT_CHECK
unsigned long reverseStartMs     = 0;  // เวลาเริ่มถอย
unsigned long reverseDuration    = 0;  // ถอยนานเท่าไหร่ (= เวลาที่เดินมา)
bool stopAfterUTurn = false;           // true ถ้าเจอ C → หยุดหลังกลับรถ

// ================== UTILITY ==================
static inline int clampi(int v, int lo, int hi){
  if(v < lo) return lo;
  if(v > hi) return hi;
  return v;
}

void resetPID(){
  integral        = 0.0f;
  prevError       = 0.0f;
  prevMs          = millis();
}

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

void sendDebug(int n1, int n2, int n3, int n4,
               int st, bool aL, bool aH, bool cH,
               unsigned long now){
  char dbg[140];
  snprintf(dbg, sizeof(dbg),
    "D:%d,%d,%d,%d,st=%d,aL=%d,aH=%d,cH=%d,arm=%d,hMs=%d,A=%d,B=%d,C=%d",
    n1, n2, n3, n4, st, aL, aH, cH,
    turnArmed,
    (int)(centerHighStartMs > 0 ? (now - centerHighStartMs) : 0),
    (checkpointCount >= 1), (checkpointCount >= 2), (checkpointCount >= 3));
  Serial.println(dbg);
  sendToClient(dbg);
}

bool checkTurnStop(bool centerHigh, unsigned long now){
  if(!turnArmed){
    if(!centerHigh) turnArmed = true;
    return false;
  }
  return centerHigh;
}

// ========== [NEW] CHECK CHECKPOINT PATTERN ==========
// L,L,L,H  → n1<300 && n2<300 && n3<300 && n4>=700
// H,L,L,L  → n1>=700 && n2<300 && n3<300 && n4<300
// H,L,L,H  → n1>=700 && n2<300 && n3<300 && n4>=700
bool isCheckpointPattern(int n1, int n2, int n3, int n4){
  bool lllh = (n1<300 && n2<300 && n3<300 && n4>=700);
  bool hlll = (n1>=700 && n2<300 && n3<300 && n4<300);
  bool hlLH = (n1>=700 && n2<300 && n3<300 && n4>=700);
  return lllh || hlll || hlLH;
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
  pinMode(15,OUTPUT);
  pinMode(2,OUTPUT);
  pinMode(5,OUTPUT);

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

  unsigned long now = millis();

  int n1 = readNorm(S1,0);
  int n2 = readNorm(S2,1);
  int n3 = readNorm(S3,2);
  int n4 = readNorm(S4,3);
  if(n1 ==0|| n2==0 || n3 ==0|| n4 ==0){
    digitalWrite(15,1);
    digitalWrite(5,1);
    digitalWrite(2,1);
    delay(100);
    digitalWrite(15,0);
    digitalWrite(5,0);
    digitalWrite(2,0);
    delay(100);
    return;
  }
  bool allLow  = (n1<=300 && n2<=300 && n3<=300 && n4<=300);
  bool allHigh = (n1>=800 && n2>=800 && n3>=800 && n4>=800);
  bool centerHigh = (n2>=800 ||  n3>=800);

  Serial.println(n1);
  Serial.println(n2);
  Serial.println(n3);
  Serial.println(n4);
  Serial.println("#########");
  // ================== STATE MACHINE ==================
  switch(runState){

    case FOLLOW: {
      confirmLow  = allLow  ? (confirmLow+1)  : 0;
      confirmHigh = allHigh ? (confirmHigh+1) : 0;

      if(confirmHigh >= CONFIRM_N){
        Serial.println(confirmHigh);
        resetTurnStop();
        runState = STRAIGHT_BEFORE_RTURN;
        confirmLow = confirmHigh = 0;
        actionStartMs = now;
        motorA(baseSpeed); motorB(baseSpeed);
        sendToClient("S");
        prevMs = now;
        return;
      }

      if(confirmLow >= 5){
        // ========== [NEW] เข้า STRAIGHT_CHECK แทน UTURN ==========
        runState = STRAIGHT_CHECK;
        confirmLow = confirmHigh = 0;
        straightCheckStart = now;       // จับเวลาเริ่มเดิน
        motorA(baseSpeed); motorB(baseSpeed);
        sendToClient("CHK");
        Serial.println(">> STRAIGHT_CHECK");
        prevMs = now;
        return;
      }
      break;
    }

    // ========== [NEW] STRAIGHT_CHECK ==========
    case STRAIGHT_CHECK: {
      motorA(baseSpeed); motorB(baseSpeed);

      // เช็ค pattern ระหว่างเดิน
      if(isCheckpointPattern(n1, n2, n3, n4)){
        checkpointCount++;
        unsigned long traveledMs = now - straightCheckStart; // เวลาที่เดินมา

        char buf[64];
        snprintf(buf, sizeof(buf), "CHECKPOINT_%c travelMs=%lu",
                 'A' + checkpointCount - 1, traveledMs);
        Serial.println(buf);
        sendToClient(buf);
        addToPath('A' + checkpointCount - 1); // เพิ่ม A / B / C ลง path

        // ตั้งค่าสำหรับถอยหลัง
        reverseDuration = traveledMs;
        reverseStartMs  = now;

        if(checkpointCount >= 3){
          // เจอ C → ถอยแล้วหยุด ไม่ต้อง U-Turn
          stopAfterUTurn = true;
        } else {
          stopAfterUTurn = false;
        }

        // เข้า REVERSE_BACK
        runState = REVERSE_BACK;
        motorA(-baseSpeed); motorB(-baseSpeed);
        sendToClient("REV");
        prevMs = now;
        return;
      }

      // หมดเวลา STRAIGHT_CHECK_MS แล้วยังไม่เจอ checkpoint → U-Turn เหมือนเดิม
      if(now - straightCheckStart >= STRAIGHT_CHECK_MS){
        // [แก้] ถอยก่อน แล้วค่อยกลับรถ
        unsigned long traveledMs = now - straightCheckStart;
        reverseDuration = traveledMs;
        reverseStartMs  = now;
        stopAfterUTurn  = false;   // ไม่หยุด → กลับรถแล้วเดินต่อ
        
        runState = REVERSE_BACK;
        motorA(-baseSpeed); motorB(-baseSpeed);
        sendToClient("REV_NOCHK");
        Serial.println(">> No checkpoint → REVERSE then UTURN");
        prevMs = now;
        return;
    }

      sendDebug(n1,n2,n3,n4,(int)runState,allLow,allHigh,centerHigh,now);
      prevMs = now;
      return;
    }

    // ========== [NEW] REVERSE_BACK ==========
    case REVERSE_BACK: {
      motorA(-baseSpeed); motorB(-baseSpeed);

      if(now - reverseStartMs >= reverseDuration){
        motorA(0); motorB(0);

        if(stopAfterUTurn){
          // เจอ C → หยุดสนิท
          Serial.println(">> STOP at C");
          sendToClient("STOP_C");
          motorA(0); motorB(0);
          while(1) delay(1000); // หยุดถาวร
        } else {
          // A หรือ B → กลับรถ แล้วเดินต่อ
          runState = UTURNTHEN;
          resetTurnStop();
          motorA( TURN_PWM);
          motorB(-TURN_PWM);
          actionStartMs = now;
          sendToClient("UTURN_THEN");
          Serial.println(">> UTURNTHEN");
          prevMs = now;
        }
      }
      sendDebug(n1,n2,n3,n4,(int)runState,allLow,allHigh,centerHigh,now);
      prevMs = now;
      return;
    }

    // ========== [NEW] UTURNTHEN ==========
    case UTURNTHEN: {
      motorA( TURN_PWM);
      motorB(-TURN_PWM);

      // ใช้ checkTurnStop เหมือน UTURN เดิม
      if(checkTurnStop(centerHigh, now)){
        motorA(0); motorB(0);
        runState = FOLLOW;
        resetPID();
        straightCheckStart = millis(); // reset timer สำหรับ STRAIGHT_CHECK ครั้งต่อไป
        sendToClient("RESUME");
        Serial.println(">> RESUME FOLLOW");
      }
      sendDebug(n1,n2,n3,n4,(int)runState,allLow,allHigh,centerHigh,now);
      prevMs = now;
      return;
    }

    case STRAIGHT_BEFORE_RTURN: {
      motorA(baseSpeed); motorB(baseSpeed);
      if(now - actionStartMs >= STRAIGHT_MS){
        runState = RTURN;
        resetTurnStop();
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

      if(checkTurnStop(centerHigh, now)){
        motorA(0); motorB(0);
        runState = FOLLOW;
        resetPID();
      }
      sendDebug(n1,n2,n3,n4,(int)runState,allLow,allHigh,centerHigh,now);
      prevMs = now;
      return;
    }

    case RTURN: {
      motorA(TURN_PWM);
      motorB(0);

      if(checkTurnStop(centerHigh, now) && (now-actionStartMs) >= STRAIGHT_MS+500){
        motorA(0); motorB(0);
        runState = FOLLOW;
        resetPID();
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
  if (checkpointCount == 1){
    digitalWrite(15,1);
  }else if(checkpointCount == 2){
    digitalWrite(15,1);
    digitalWrite(5,1);
  }else if(checkpointCount ==3){
    digitalWrite(15,1);
    digitalWrite(5,1);
    digitalWrite(2,1);
  }
  char dbg[160];
  snprintf(dbg, sizeof(dbg),
    "D:%d,%d,%d,%d,st=%d,aL=%d,aH=%d,cH=%d,arm=%d,hMs=%d,e=%.3f,c=%.2f,L=%d,R=%d,A=%d,B=%d,C=%d",
    n1, n2, n3, n4,
    (int)runState, allLow, allHigh, centerHigh,
    turnArmed,
    (int)(centerHighStartMs > 0 ? (now - centerHighStartMs) : 0),
    error, corr, left, right,
    (checkpointCount >= 1), (checkpointCount >= 2), (checkpointCount >= 3));
  //Serial.println(dbg);
  if(wifi_enable){ char buf[162]; snprintf(buf, sizeof(buf), "%s\n", dbg); sendToClient(buf); }
}
