#include <Arduino.h>

// ================== DRV8833 PIN MAP ==================
static const int AIN1 = 25;
static const int AIN2 = 26;
static const int EN   = 27;   // HIGH always
static const int BIN1 = 33;
static const int BIN2 = 32;

// ================== IR SENSOR PINS (4 inputs) ==================
static const int S1 = 34; // leftmost
static const int S2 = 35
;
static const int S3 = 36;
static const int S4 = 39; // rightmost

// ================== PID GAINS (TUNE) ==================
float Kp = 25.0f;
float Ki = 0.0f;
float Kd = 12.0f;

// ================== SPEED SETTINGS ==================
int baseSpeed = 140;     // 0..255
int maxSpeed  = 220;     // clamp 0..255

// ================== SENSOR AUTO-CAL ==================
int sMin[4] = {4095,4095,4095,4095};
int sMax[4] = {0,0,0,0};

// ================== PID STATE ==================
float integral  = 0.0f;
float prevError = 0.0f;
unsigned long prevMs = 0;

// ---------- utility ----------
static inline int clampi(int v, int lo, int hi){
  if(v < lo) return lo;
  if(v > hi) return hi;
  return v;
}

// ================== MOTOR CONTROL (no ledcSetup) ==================
// speed: -255..255 (negative = reverse)
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
// weights: -3, -1, +1, +3  (left -> right)
// assume "line gives higher value" on sensor.
// if your black line gives LOWER value, invert: n = 1000 - n;
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

// ================== SETUP ==================
void setup(){
  Serial.begin(115200);

  pinMode(EN, OUTPUT);
  digitalWrite(EN, HIGH); // enable driver always

  // Motor pins as output
  pinMode(AIN1, OUTPUT);
  pinMode(AIN2, OUTPUT);
  pinMode(BIN1, OUTPUT);
  pinMode(BIN2, OUTPUT);

  // ADC config
  analogReadResolution(12);       // 0..4095
  analogSetAttenuation(ADC_11db); // ~0..3.3V

  // PWM config (still "ธรรมดา" คือใช้ analogWrite API)
  //(8);       // 0..255
  // ถ้าคุณอยากให้เงียบขึ้น ลอง 20kHz (บาง core รองรับ)
  // analogWriteFrequency(20000);

  motorA(0);
  motorB(0);

  Serial.println("Calibrating IR... (move over line and floor)");
  calibrate(2000);
  Serial.println("Calibration done.");

  prevMs = millis();
}

// ================== LOOP ==================
void loop(){
  int n1 = readNorm(S1, 0);
  int n2 = readNorm(S2, 1);
  int n3 = readNorm(S3, 2);
  int n4 = readNorm(S4, 3);

  // ถ้า “เส้นดำ” ทำให้ค่าน้อยลง ให้ปลดคอมเมนต์ 4 บรรทัดนี้:
  // n1 = 1000 - n1; n2 = 1000 - n2; n3 = 1000 - n3; n4 = 1000 - n4;

  float error = computeError(n1, n2, n3, n4);

  unsigned long now = millis();
  float dt = (now - prevMs) / 1000.0f;
  if(dt <= 0) dt = 0.001f;
  prevMs = now;

  // PID
  integral += error * dt;
  integral = constrain(integral, -1.0f, 1.0f); // anti-windup แบบง่าย

  float derivative = (error - prevError) / dt;
  prevError = error;

  float corr = Kp*error + Ki*integral + Kd*derivative;

  int left  = (int)(baseSpeed + corr);
  int right = (int)(baseSpeed - corr);

  left  = clampi(left,  0, maxSpeed);
  right = clampi(right, 0, maxSpeed);

  // สมมติ A=ซ้าย, B=ขวา (ถ้ากลับด้านให้สลับ motorA/motorB)
  motorA(left);
  motorB(right);

  // debug
  Serial.print("N:");
  Serial.print(n1); Serial.print(",");
  Serial.print(n2); Serial.print(",");
  Serial.print(n3); Serial.print(",");
  Serial.print(n4);
  Serial.print("  e="); Serial.print(error, 3);
  Serial.print("  corr="); Serial.print(corr, 2);
  Serial.print("  L="); Serial.print(left);
  Serial.print("  R="); Serial.println(right);

  delay(5);
}