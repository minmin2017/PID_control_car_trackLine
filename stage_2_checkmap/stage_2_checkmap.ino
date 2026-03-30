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
  STRAIGHT_CHECK, REVERSE_BACK, UTURNTHEN,
  // ========== เพิ่มสำหรับเลี้ยวซ้าย / ตรงผ่านแยก / กลับรถที่บ้าน ==========
  STRAIGHT_BEFORE_LTURN, LTURN,
  STRAIGHT_THROUGH,
  HOME_UTURN
};
RunState runState = FOLLOW;

// ================== MODE ==================
enum Mode { MODE_EXPLORE, MODE_RETURN, MODE_SECOND };
Mode currentMode = MODE_EXPLORE;

unsigned long actionStartMs = 0;
const unsigned long STRAIGHT_MS = 525;

int confirmLow = 0, confirmHigh = 0;
const int CONFIRM_N = 1;

bool turnArmed          = false;
unsigned long centerHighStartMs = 0;
const unsigned long STOP_HOLD_MS = 1;

// ========== CHECKPOINT VARIABLES ==========
const unsigned long STRAIGHT_CHECK_MS = 300;
const unsigned long UTURN_AFTER_REVERSE_MS = 300;

int  checkpointCount  = 0;
unsigned long straightCheckStart = 0;
unsigned long reverseStartMs     = 0;
unsigned long reverseDuration    = 0;
bool stopAfterUTurn = false;

// ========== RETURN & SECOND RUN VARIABLES ==========
String optJunctions = "";        // junction turns สำหรับรอบ 2 (ทางลัด)
int    optJunctionIndex = 0;
int    returnJunctionsTotal = 0; // จำนวน R ทั้งหมดตอนสำรวจ = จำนวน L ตอนกลับบ้าน
int    returnJunctionsDone  = 0;
bool   nearHome = false;

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
}

bool checkTurnStop(bool centerHigh, unsigned long now){
  if(!turnArmed){
    if(!centerHigh) turnArmed = true;
    return false;
  }
  return centerHigh;
}

// ========== CHECK CHECKPOINT PATTERN ==========
bool isCheckpointPattern(int n1, int n2, int n3, int n4){
  bool lllh = (n1<300 && n2<300 && n3<300 && n4>=700);
  bool hlll = (n1>=700 && n2<300 && n3<300 && n4<300);
  bool hlLH = (n1>=700 && n2<300 && n3<300 && n4>=700);
  return lllh || hlll || hlLH;
}

// ========== PATH SIMPLIFICATION (ยุบ path) ==========
// ลบทางตัน (U ที่ไม่ใช่ checkpoint) โดยรวม 3 เลี้ยวรอบ U เป็น 1 เลี้ยว
// ใช้มุม: R=90, L=270, S=0, U=180
int turnAngle(char c){
  if(c == 'R') return 90;
  if(c == 'L') return 270;
  if(c == 'S') return 0;
  if(c == 'U') return 180;
  return 0;
}

char angleToTurn(int a){
  a = ((a % 360) + 360) % 360;
  if(a == 0)   return 'S';
  if(a == 90)  return 'R';
  if(a == 180) return 'U';
  if(a == 270) return 'L';
  return 'S';
}

String simplifyPath(String path){
  bool changed = true;
  while(changed){
    changed = false;
    for(int i = 0; i < (int)path.length(); i++){
      if(path.charAt(i) != 'U') continue;
      // ข้าม checkpoint U (มี A/B/C นำหน้า)
      if(i > 0 && path.charAt(i-1) >= 'A' && path.charAt(i-1) <= 'C') continue;

      // หา turn ก่อน U
      int before = -1;
      for(int j = i-1; j >= 0; j--){
        char c = path.charAt(j);
        if(c == 'R' || c == 'L' || c == 'S'){ before = j; break; }
      }
      // หา turn หลัง U
      int after = -1;
      for(int j = i+1; j < (int)path.length(); j++){
        char c = path.charAt(j);
        if(c == 'R' || c == 'L' || c == 'S'){ after = j; break; }
      }

      if(before >= 0 && after >= 0){
        int a = turnAngle(path.charAt(before)) + 180 + turnAngle(path.charAt(after));
        char simplified = angleToTurn(a);
        String np = path.substring(0, before);
        np += simplified;
        np += path.substring(after + 1);
        path = np;
        changed = true;
        break;
      }
    }
  }
  return path;
}

// ดึงเฉพาะ junction turns (R/L/S) จาก path
String extractJunctions(String path){
  String result = "";
  for(int i = 0; i < (int)path.length(); i++){
    char c = path.charAt(i);
    if(c == 'R' || c == 'L' || c == 'S') result += c;
  }
  return result;
}

// นับจำนวนตัวอักษรที่ต้องการ
int countChar(String s, char target){
  int cnt = 0;
  for(int i = 0; i < (int)s.length(); i++){
    if(s.charAt(i) == target) cnt++;
  }
  return cnt;
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
  pinMode(4,OUTPUT);
  pinMode(5,OUTPUT);

  analogReadResolution(12);
  analogSetAttenuation(ADC_11db);

  motorA(0); motorB(0);

  Serial.println("Calibrating...");
  calibrate(2000);
  Serial.println("Done.");

  resetPID();
  int check_start_only_on_line = 0;
  while( (check_start_only_on_line == 0) ){
    int n1 = readNorm(S1,0);
    int n2 = readNorm(S2,1);
    int n3 = readNorm(S3,2);
    int n4 = readNorm(S4,3);
    if (n1 <300 && n2 >800 && n3>800 && n4 <300){
      check_start_only_on_line = 1;
    }
  }
  motorA(baseSpeed); motorB(baseSpeed);
}

// ================== LOOP ==================
int first_time = 0;
int glich_zero_count = 0;
void loop(){

  if(wifi_enable) net.update();
  if(first_time == 0){ sendToClient("F"); first_time = 1; }

  unsigned long now = millis();

  int n1 = readNorm(S1,0);
  int n2 = readNorm(S2,1);
  int n3 = readNorm(S3,2);
  int n4 = readNorm(S4,3);
  if((n1 ==0|| n2==0 || n3 ==0|| n4 ==0) && glich_zero_count == 0){
    digitalWrite(15,1);
    digitalWrite(5,1);
    digitalWrite(4,1);
    delay(100);
    digitalWrite(15,0);
    digitalWrite(5,0);
    digitalWrite(4,0);
    delay(100);
    return;
  }else if (n1 !=0 && n2!=0 && n3 !=0&& n4 !=0){
    glich_zero_count = 1;
    if (checkpointCount == 1){
      digitalWrite(15,1);
    }else if(checkpointCount == 2){
      digitalWrite(15,1);
      digitalWrite(4,1);
    }else if(checkpointCount ==3){
      digitalWrite(15,1);
      digitalWrite(5,1);
      digitalWrite(4,1);
    }
  }
  bool allLow  = (n1<=300 && n2<=300 && n3<=300 && n4<=300);
  bool allHigh = (n1>=800 && n2>=800 && n3>=800 && n4>=800);
  bool centerHigh = (n2>=800 ||  n3>=800);


  // ================== STATE MACHINE ==================
  switch(runState){

    // ==================== FOLLOW ====================
    case FOLLOW: {
      confirmLow  = allLow  ? (confirmLow+1)  : 0;
      confirmHigh = allHigh ? (confirmHigh+1) : 0;

      // ---------- allHigh = แยก ----------
      if(confirmHigh >= CONFIRM_N){
        confirmLow = confirmHigh = 0;
        actionStartMs = now;

        if(currentMode == MODE_EXPLORE){
          // สำรวจ: เลี้ยวขวาเสมอ
          resetTurnStop();
          runState = STRAIGHT_BEFORE_RTURN;
          motorA(baseSpeed); motorB(baseSpeed);
          sendToClient("S");
          path_map += 'R';
          Serial.println("R");

        } else if(currentMode == MODE_RETURN){
          // กลับบ้าน: เลี้ยวซ้ายเสมอ
          resetTurnStop();
          runState = STRAIGHT_BEFORE_LTURN;
          motorA(baseSpeed); motorB(baseSpeed);
          Serial.println("L (return)");
          returnJunctionsDone++;
          if(returnJunctionsDone >= returnJunctionsTotal) nearHome = true;

        } else { // MODE_SECOND
          // รอบ 2 ทางลัด: อ่านจาก optJunctions
          if(optJunctionIndex < (int)optJunctions.length()){
            char action = optJunctions.charAt(optJunctionIndex++);
            Serial.print("OPT: "); Serial.println(action);

            if(action == 'R'){
              resetTurnStop();
              runState = STRAIGHT_BEFORE_RTURN;
              motorA(baseSpeed); motorB(baseSpeed);
            } else if(action == 'L'){
              resetTurnStop();
              runState = STRAIGHT_BEFORE_LTURN;
              motorA(baseSpeed); motorB(baseSpeed);
            } else { // 'S' = ตรงผ่านแยก
              runState = STRAIGHT_THROUGH;
              motorA(baseSpeed); motorB(baseSpeed);
            }
          }
        }
        prevMs = now;
        return;
      }

      // ---------- allLow = ทางตัน ----------
      if(confirmLow >= 5){
        confirmLow = confirmHigh = 0;

        // กลับบ้าน + ผ่านแยกหมดแล้ว = ถึงบ้าน → กลับรถแล้วเริ่มรอบ 2
        if(currentMode == MODE_RETURN && nearHome){
          runState = HOME_UTURN;
          resetTurnStop();
          motorA(TURN_PWM); motorB(-TURN_PWM);
          Serial.println(">> HOME - U-TURN");
          sendToClient("HOME_UTURN");
          prevMs = now;
          return;
        }

        // ปกติ: เข้า STRAIGHT_CHECK
        runState = STRAIGHT_CHECK;
        straightCheckStart = now;
        motorA(baseSpeed); motorB(baseSpeed);
        sendToClient("CHK");
        Serial.println(">> STRAIGHT_CHECK");
        prevMs = now;
        return;
      }
      break;
    }

    // ==================== STRAIGHT_CHECK ====================
    case STRAIGHT_CHECK: {
      motorA(baseSpeed); motorB(baseSpeed);

      // เช็ค pattern ระหว่างเดิน
      if(isCheckpointPattern(n1, n2, n3, n4)){
        unsigned long traveledMs = now - straightCheckStart;

        if(currentMode == MODE_RETURN){
          // กลับบ้าน: ไม่นับ checkpoint แค่ถอย + U-Turn ออก
          reverseDuration = traveledMs;
          reverseStartMs  = now;
          runState = REVERSE_BACK;
          motorA(-baseSpeed); motorB(-baseSpeed);
          Serial.println(">> return: skip checkpoint, reverse");
          prevMs = now;
          return;
        }

        checkpointCount++;

        char buf[64];
        snprintf(buf, sizeof(buf), "CHECKPOINT_%c travelMs=%lu",
                 'A' + checkpointCount - 1, traveledMs);
        Serial.println(buf);
        sendToClient(buf);

        // รอบ 2 ทางลัด: เจอครบ 3 = จบ
        if(currentMode == MODE_SECOND && checkpointCount >= 3){
          motorA(0); motorB(0);
          Serial.println(">> MISSION COMPLETE!");
          sendToClient("MISSION_COMPLETE");
          // กระพริบ LED ฉลอง
          while(1){
            digitalWrite(15,1); digitalWrite(4,1); digitalWrite(5,1);
            delay(300);
            digitalWrite(15,0); digitalWrite(4,0); digitalWrite(5,0);
            delay(300);
          }
        }

        // บันทึก path (สำรวจ + รอบ 2)
        if(currentMode == MODE_EXPLORE){
          path_map += ('A' + checkpointCount - 1);
          path_map += 'U';
          Serial.println(path_map);
        }

        // ตั้งค่าถอยหลัง
        reverseDuration = traveledMs;
        reverseStartMs  = now;

        // สำรวจรอบแรก: เจอ C → flag หยุดหลัง U-Turn
        if(currentMode == MODE_EXPLORE) stopAfterUTurn = (checkpointCount >= 3);

        runState = REVERSE_BACK;
        motorA(-baseSpeed); motorB(-baseSpeed);
        sendToClient("REV");
        prevMs = now;
        return;
      }

      // หมดเวลา → ไม่เจอ checkpoint
      if(now - straightCheckStart >= STRAIGHT_CHECK_MS){
        if(currentMode == MODE_EXPLORE) path_map += 'U';
        unsigned long traveledMs = now - straightCheckStart;
        reverseDuration = traveledMs;
        reverseStartMs  = now;
        runState = REVERSE_BACK;
        motorA(-baseSpeed); motorB(-baseSpeed);
        sendToClient("REV_NOCHK");
        Serial.println(">> No checkpoint -> REVERSE then UTURN");
        prevMs = now;
        return;
      }

      prevMs = now;
      return;
    }

    // ==================== REVERSE_BACK ====================
    case REVERSE_BACK: {
      motorA(-baseSpeed); motorB(-baseSpeed);

      if(now - reverseStartMs >= reverseDuration){
        motorA(0); motorB(0);

        runState = UTURNTHEN;
        resetTurnStop();
        motorA( TURN_PWM);
        motorB(-TURN_PWM);
        actionStartMs = now;
        sendToClient("UTURN_THEN");
        Serial.println(">> UTURNTHEN");
        prevMs = now;
      }
      prevMs = now;
      return;
    }

    // ==================== UTURNTHEN ====================
    case UTURNTHEN: {
      motorA( TURN_PWM);
      motorB(-TURN_PWM);

      if(checkTurnStop(centerHigh, now)){
        motorA(0); motorB(0);

        if(stopAfterUTurn){
          // ===== เจอ C รอบสำรวจ: รอ 5 วิ + กระพริบ 5 วิ + คำนวณ path + กลับบ้าน =====
          Serial.println(">> STOP at C (after U-Turn)");
          sendToClient("STOP_C");

          // รอ 5 วินาที
          delay(5000);

          // กระพริบ LED 5 วินาที
          unsigned long blinkStart = millis();
          while(millis() - blinkStart < 5000){
            digitalWrite(15, 1); digitalWrite(4, 1); digitalWrite(5, 1);
            delay(200);
            digitalWrite(15, 0); digitalWrite(4, 0); digitalWrite(5, 0);
            delay(200);
          }

          // ===== คำนวณ path ทางลัดสำหรับรอบ 2 =====
          Serial.print("Raw path: "); Serial.println(path_map);
          String simplified = simplifyPath(path_map);
          Serial.print("Simplified: "); Serial.println(simplified);
          optJunctions = extractJunctions(simplified);
          Serial.print("Opt junctions: "); Serial.println(optJunctions);

          // นับ R ใน path_map เดิม = จำนวนแยกที่ต้องเลี้ยวซ้ายตอนกลับ
          returnJunctionsTotal = countChar(path_map, 'R');
          Serial.print("Return L-turns needed: "); Serial.println(returnJunctionsTotal);

          // เริ่มกลับบ้าน
          currentMode = MODE_RETURN;
          returnJunctionsDone = 0;
          nearHome = false;
          checkpointCount = 0;
          stopAfterUTurn = false;
          confirmLow = 0;
          confirmHigh = 0;
          runState = FOLLOW;
          resetPID();
          motorA(baseSpeed); motorB(baseSpeed);
          Serial.println(">> RETURN HOME");
          sendToClient("RETURN");
          prevMs = millis();
          return;
        }

        runState = FOLLOW;
        resetPID();
        straightCheckStart = millis();
        sendToClient("RESUME");
        Serial.println(">> RESUME FOLLOW");
      }
      prevMs = now;
      return;
    }

    // ==================== HOME_UTURN (กลับรถที่บ้าน → เริ่มรอบ 2) ====================
    case HOME_UTURN: {
      motorA(TURN_PWM); motorB(-TURN_PWM);

      if(checkTurnStop(centerHigh, now)){
        motorA(0); motorB(0);

        // เริ่มรอบ 2 ทางลัด
        currentMode = MODE_SECOND;
        optJunctionIndex = 0;
        checkpointCount = 0;
        confirmLow = 0;
        confirmHigh = 0;
        runState = FOLLOW;
        resetPID();
        motorA(baseSpeed); motorB(baseSpeed);
        Serial.println(">> SECOND RUN (optimized)");
        Serial.print("Junctions: "); Serial.println(optJunctions);
        sendToClient("SECOND_RUN");
      }
      prevMs = now;
      return;
    }

    // ==================== STRAIGHT_BEFORE_RTURN ====================
    case STRAIGHT_BEFORE_RTURN: {
      motorA(baseSpeed); motorB(baseSpeed);
      if(now - actionStartMs >= STRAIGHT_MS){
        runState = RTURN;
        resetTurnStop();
        motorA(TURN_PWM);
        motorB(0);
        sendToClient("R");
      }
      prevMs = now;
      return;
    }

    // ==================== RTURN ====================
    case RTURN: {
      motorA(TURN_PWM);
      motorB(0);

      if(checkTurnStop(centerHigh, now) && (now-actionStartMs) >= STRAIGHT_MS+500){
        motorA(0); motorB(0);
        runState = FOLLOW;
        confirmLow = 0;
        confirmHigh = 0;
        resetPID();
      }
      prevMs = now;
      return;
    }

    // ==================== STRAIGHT_BEFORE_LTURN ====================
    case STRAIGHT_BEFORE_LTURN: {
      motorA(baseSpeed); motorB(baseSpeed);
      if(now - actionStartMs >= STRAIGHT_MS){
        runState = LTURN;
        resetTurnStop();
        motorA(0);
        motorB(TURN_PWM);
        sendToClient("L");
        Serial.println(">> LTURN");
      }
      prevMs = now;
      return;
    }

    // ==================== LTURN ====================
    case LTURN: {
      motorA(0);
      motorB(TURN_PWM);

      if(checkTurnStop(centerHigh, now) && (now-actionStartMs) >= STRAIGHT_MS+500){
        motorA(0); motorB(0);
        runState = FOLLOW;
        confirmLow = 0;
        confirmHigh = 0;
        resetPID();
      }
      prevMs = now;
      return;
    }

    // ==================== STRAIGHT_THROUGH (ตรงผ่านแยก) ====================
    case STRAIGHT_THROUGH: {
      motorA(baseSpeed); motorB(baseSpeed);
      // วิ่งตรงจนผ่านแยก (allHigh หาย)
      if(!allHigh && (now - actionStartMs) > 100){
        runState = FOLLOW;
        confirmLow = 0;
        confirmHigh = 0;
        resetPID();
        Serial.println(">> STRAIGHT through");
      }
      prevMs = now;
      return;
    }

    // ==================== UTURN (legacy) ====================
    case UTURN: {
      motorA( TURN_PWM);
      motorB(-TURN_PWM);

      if(checkTurnStop(centerHigh, now)){
        motorA(0); motorB(0);
        runState = FOLLOW;
        resetPID();
      }
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

  char dbg[160];
  snprintf(dbg, sizeof(dbg),
    "D:%d,%d,%d,%d,st=%d,aL=%d,aH=%d,cH=%d,arm=%d,hMs=%d,e=%.3f,c=%.2f,L=%d,R=%d,A=%d,B=%d,C=%d",
    n1, n2, n3, n4,
    (int)runState, allLow, allHigh, centerHigh,
    turnArmed,
    (int)(centerHighStartMs > 0 ? (now - centerHighStartMs) : 0),
    error, corr, left, right,
    (checkpointCount >= 1), (checkpointCount >= 2), (checkpointCount >= 3));
  if(wifi_enable){ char buf[162]; snprintf(buf, sizeof(buf), "%s\n", dbg); sendToClient(buf); }
}
