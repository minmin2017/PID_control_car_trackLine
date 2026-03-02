#include <Arduino.h>
#include <WiFi.h>
#include "Communication_order.h"

LanCommand net;

// ---------------- WiFi ----------------
const char* ssid     = "MinMin2017";
const char* password = "minmin2017i";
const bool wifi_enable = true;

// ================== DRV8833 PIN MAP ==================
static const int AIN1 = 27;
static const int AIN2 = 14;
static const int EN   = 12;   // HIGH always
static const int BIN1 = 26;
static const int BIN2 = 25;

// ================== IR SENSOR PINS (4 inputs) ==================
static const int S1 = 34; // leftmost
static const int S2 = 35;
static const int S3 = 32;
static const int S4 = 33; // rightmost

// ================== PID GAINS ==================
float Kp = 30.0f;
float Ki = 50.0f;
float Kd = 0.0f;

// ================== SPEED SETTINGS ==================
int baseSpeed = 110;     // 0..255
int maxSpeed  = 200;     // clamp 0..255

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

const int TURN_PWM = 150;

// “เจอแยก (4 ตัวสูง) -> ตรงไปต่ออีกกี่ ms -> ค่อยเลี้ยวขวา”
const unsigned long STRAIGHT_MS = 250;  // ปรับเป็น 400, 600, 800 ได้

// กันสัญญาณแกว่ง (เข้าโหมดพิเศษ)
int confirmLow = 0, confirmHigh = 0;
const int CONFIRM_N = 2;

// หยุดเลี้ยวด้วย “2 เส้นกลาง” (n2,n3) ตามสนามคุณ
bool turnArmed = false;         // ต้องหลุด centerHigh ก่อน ถึงจะเริ่มนับหยุด
int  confirmStop = 0;           // นับยืนยันตอน centerHigh เพื่อหยุด
const int STOP_CONFIRM_N = 2;

// ---------- utility ----------
static inline int clampi(int v, int lo, int hi){
  if(v < lo) return lo;
  if(v > hi) return hi;
  return v;
}

// ================== MOTOR CONTROL ==================
void motorA(int speed){
  speed = clampi(speed, -255, 255);
  if(speed >= 0){
    analogWrite(AIN1, speed);
    analogWrite(AIN2, 0);
  } else {
    analogWrite(AIN1, 0);
    analogWrite(AIN2, -speed);
  }
}

void motorB(int speed){
  speed = clampi(speed, -255, 255);
  if(speed >= 0){
    analogWrite(BIN1, speed);
    analogWrite(BIN2, 0);
  } else {
    analogWrite(BIN1, 0);
    analogWrite(BIN2, -speed);
  }
}

// ================== SENSOR READ + NORMALIZE 0..1000 ==================
int readNorm(int pin, int idx){
  int raw = analogRead(pin); // 0..4095

  if(raw < sMin[idx]) sMin[idx] = raw;
  if(raw > sMax[idx]) sMax[idx] = raw;

  int den = (sMax[idx] - sMin[idx]);
  if(den < 10) den = 10;

  int norm = (raw - sMin[idx]) * 1000 / den;
  norm = clampi(norm, 0, 1000);
  return norm;
}

// ================== LINE POSITION ERROR (weighted) ==================
float computeError(int n1, int n2, int n3, int n4){
  const float w1 = -3, w2 = -1, w3 = 1, w4 = 3;

  float sum = (float)n1 + n2 + n3 + n4;
  if(sum < 1) sum = 1;

  float pos = (w1*n1 + w2*n2 + w3*n3 + w4*n4) / sum; // approx -3..+3
  return pos;
}

// ================== CALIBRATE ==================
void calibrate(unsigned long ms = 2000){
  unsigned long t0 = millis();
  while(millis() - t0 < ms){
    (void)readNorm(S1, 0);
    (void)readNorm(S2, 1);
    (void)readNorm(S3, 2);
    (void)readNorm(S4, 3);
    delay(5);
  }
}

static inline void sendToClient(const char* msg){
  if (wifi_enable && net.hasClient()){
    net.clientRef().println(msg);
  }
}

// ================== SETUP ==================
void setup(){
  Serial.begin(115200);

  if (wifi_enable) {
    bool ok = net.begin(ssid, password);
    if (!ok) {
      Serial.println("WiFi failed (net.begin timeout)");
      while (1) { delay(1000); }
    }
    Serial.print("WiFi connected, IP = ");
    Serial.println(WiFi.localIP());
  }

  pinMode(EN, OUTPUT);
  digitalWrite(EN, HIGH);

  pinMode(AIN1, OUTPUT);
  pinMode(AIN2, OUTPUT);
  pinMode(BIN1, OUTPUT);
  pinMode(BIN2, OUTPUT);

  pinMode(S1, INPUT);
  pinMode(S2, INPUT);
  pinMode(S3, INPUT);
  pinMode(S4, INPUT);

  analogReadResolution(12);
  analogSetAttenuation(ADC_11db);

  motorA(0);
  motorB(0);

  Serial.println("Calibrating IR... (move over line and floor)");
  calibrate(2000);
  Serial.println("Calibration done.");

  prevMs = millis();
}

// ================== LOOP ==================
int first_time = 0;
void loop(){
  if (wifi_enable) net.update();
  if (first_time==0){
    sendToClient("F");
    first_time = 1;
  }

  int n1 = readNorm(S1, 0);
  int n2 = readNorm(S2, 1);
  int n3 = readNorm(S3, 2);
  int n4 = readNorm(S4, 3);

  // ใช้ detect “ทางตัน/หลุดเส้น” (คงเดิม)
  bool allLow  = (n1>=0   && n1<=100) &&
                 (n2>=0   && n2<=100) &&
                 (n3>=0   && n3<=100) &&
                 (n4>=0   && n4<=100);

  // ใช้ detect “แยก/จุดสั่งเลี้ยวขวา” (คงเดิม: 4 ตัวสูง)
  bool allHigh = (n1>=800 && n1<=1000) &&
                 (n2>=800 && n2<=1000) &&
                 (n3>=800 && n3<=1000) &&
                 (n4>=800 && n4<=1000);

  // เกณฑ์ “เจอเส้นตอนกำลังเลี้ยว” ตามที่คุณต้องการ: ใช้แค่กลาง 2 ตัว
  bool centerHigh = (n2 >= 900 && n3 >= 900);

  unsigned long now = millis();

  // ================== NON-BLOCKING STATE MACHINE ==================
  switch(runState){

    case FOLLOW: {
      confirmLow  = allLow  ? (confirmLow  + 1) : 0;
      confirmHigh = allHigh ? (confirmHigh + 1) : 0;

      // เจอ allLow ต่อเนื่อง -> UTURN (หมุนจน centerHigh กลับมา)
      if(confirmLow >= CONFIRM_N){
        runState = UTURN;
        confirmLow = confirmHigh = 0;

        turnArmed = false;
        confirmStop = 0;

        motorA( TURN_PWM);
        motorB(-TURN_PWM);
        sendToClient("U");
        return;
      }

      // เจอ allHigh ต่อเนื่อง -> ตรงไปก่อน STRAIGHT_MS แล้วค่อย RTURN
      if(confirmHigh >= CONFIRM_N){
        runState = STRAIGHT_BEFORE_RTURN;
        confirmLow = confirmHigh = 0;

        actionStartMs = now;

        motorA(baseSpeed);
        motorB(baseSpeed);

        sendToClient("S"); // Straight before right turn
        return;
      }

      break; // ไป PID
    }

    case STRAIGHT_BEFORE_RTURN: {
      // ตรงไปต่อก่อนเลี้ยวขวา
      motorA(baseSpeed);
      motorB(baseSpeed);

      if(now - actionStartMs >= STRAIGHT_MS){
        runState = RTURN;

        turnArmed = false;
        confirmStop = 0;

        motorA(TURN_PWM);
        motorB(0);
        sendToClient("R");
      }
      return;
    }

    case UTURN: {
      motorA( TURN_PWM);
      motorB(-TURN_PWM);

      // ต้องหลุด centerHigh ก่อน (กันเข้าแล้วหยุดทันที)
      if(!centerHigh) turnArmed = true;

      if(turnArmed && centerHigh){
        confirmStop++;
      } else {
        confirmStop = 0;
      }

      if(confirmStop >= STOP_CONFIRM_N){
        motorA(0);
        motorB(0);
        runState = FOLLOW;
        sendToClient("F");
      }
      return;
    }

    case RTURN: {
      motorA( TURN_PWM);
      motorB(0);

      if(!centerHigh) turnArmed = true;

      if(turnArmed && centerHigh){
        confirmStop++;
      } else {
        confirmStop = 0;
      }

      if(confirmStop >= STOP_CONFIRM_N){
        motorA(0);
        motorB(0);
        runState = FOLLOW;
        sendToClient("F");
      }
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

  int left  = (int)(baseSpeed + corr);
  int right = (int)(baseSpeed - corr);

  left  = clampi(left,  0, maxSpeed);
  right = clampi(right, 0, maxSpeed);

  motorA(left);
  motorB(right);

  // debug
  Serial.print("N:");
  Serial.print(n1); Serial.print(",");
  Serial.print(n2); Serial.print(",");
  Serial.print(n3); Serial.print(",");
  Serial.print(n4);
  Serial.print("  allLow="); Serial.print(allLow);
  Serial.print("  allHigh="); Serial.print(allHigh);
  Serial.print("  centerHigh="); Serial.print(centerHigh);
  Serial.print("  state="); Serial.print((int)runState);
  Serial.print("  armed="); Serial.print(turnArmed);
  Serial.print("  stopCnt="); Serial.print(confirmStop);

  Serial.print("  e="); Serial.print(error, 3);
  Serial.print("  corr="); Serial.print(corr, 2);
  Serial.print("  L="); Serial.print(left);
  Serial.print("  R="); Serial.println(right);
}