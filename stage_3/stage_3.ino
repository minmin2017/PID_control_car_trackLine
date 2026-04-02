#include <Arduino.h>
#include <WiFi.h>
#include "Communication_order.h"

LanCommand net;
String path_map = "";

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
int baseSpeed = 150;
int maxSpeed  = 200;
const int TURN_PWM = 150;

// ================== SENSOR AUTO-CAL ==================
int sMin[4] = {4095, 4095, 4095, 4095};
int sMax[4] = {0, 0, 0, 0};

// ================== PID STATE ==================
float integral  = 0.0f;
float prevError = 0.0f;
unsigned long prevMs = 0;

// ================== STATE MACHINE ==================
enum RunState {
  FOLLOW, UTURN, STRAIGHT_BEFORE_RTURN, RTURN,
  STRAIGHT_CHECK, REVERSE_BACK, UTURNTHEN,
  STRAIGHT_BEFORE_LTURN, LTURN,
  STRAIGHT_THROUGH,
  HOME_UTURN,
  FINAL_UTURN
};
RunState runState = FOLLOW;

// ================== MODE ==================
enum Mode { MODE_EXPLORE, MODE_RETURN, MODE_SECOND };
Mode currentMode = MODE_EXPLORE;

unsigned long actionStartMs = 0;
const unsigned long STRAIGHT_MS = 300;

int confirmLow = 0, confirmHigh = 0;
const int CONFIRM_N = 1;

bool turnArmed = false;
unsigned long centerHighStartMs = 0;

// ========== CHECKPOINT VARIABLES ==========
const unsigned long STRAIGHT_CHECK_MS = 400; // 400

int checkpointCount = 0;
unsigned long straightCheckStart = 0;
unsigned long reverseStartMs     = 0;
unsigned long reverseDuration    = 0;
bool stopAfterUTurn = false;

// ========== CHECKPOINT CONFIRMATION ==========
char pendingCP = '\0';
int cpConfirmCount = 0;
const int CP_CONFIRM_N = 50; 

// ========== CHECKPOINT TRACKING ==========
bool foundA = false, foundB = false, foundC = false;
String rawPathToA = "", rawPathToB = "", rawPathToC = "";

// ========== RETURN & SECOND RUN VARIABLES ==========
String optJunctions = "";
int    optJunctionIndex = 0;
int    returnJunctionsTotal = 0;
int    returnJunctionsDone  = 0;
bool   nearHome = false;

// ========== RETURN PATH VARIABLES ==========
String returnJunctionsStr = "";
int    returnJunctionIndex = 0;

// ================== UTILITY ==================
static inline int clampi(int v, int lo, int hi) {
  if (v < lo) return lo;
  if (v > hi) return hi;
  return v;
}

void resetPID() {
  integral  = 0.0f;
  prevError = 0.0f;
  prevMs    = millis();
}

void resetTurnStop() {
  turnArmed = false;
  centerHighStartMs = 0;
}

// ================== MOTOR CONTROL ==================
void motorA(int speed) {
  speed = clampi(speed, -255, 255);
  if (speed >= 0) { analogWrite(AIN1, speed); analogWrite(AIN2, 0); }
  else            { analogWrite(AIN1, 0);     analogWrite(AIN2, -speed); }
}

void motorB(int speed) {
  speed = clampi(speed, -255, 255);
  if (speed >= 0) { analogWrite(BIN1, speed); analogWrite(BIN2, 0); }
  else            { analogWrite(BIN1, 0);     analogWrite(BIN2, -speed); }
}

// ================== SENSOR READ ==================
int readNorm(int pin, int idx) {
  int raw = analogRead(pin);
  if (raw < sMin[idx]) sMin[idx] = raw;
  if (raw > sMax[idx]) sMax[idx] = raw;
  int den = (sMax[idx] - sMin[idx]);
  if (den < 10) den = 10;
  int norm = (raw - sMin[idx]) * 1000 / den;
  return clampi(norm, 0, 1000);
}

// ================== LINE ERROR ==================
float computeError(int n1, int n2, int n3, int n4) {
  const float w1 = -3, w2 = -1, w3 = 1, w4 = 3;
  float sum = (float)n1 + n2 + n3 + n4;
  if (sum < 1) sum = 1;
  return (w1 * n1 + w2 * n2 + w3 * n3 + w4 * n4) / sum;
}

// ================== CALIBRATE ==================
void calibrate(unsigned long ms = 2000) {
  unsigned long t0 = millis();
  while (millis() - t0 < ms) {
    (void)readNorm(S1, 0); (void)readNorm(S2, 1);
    (void)readNorm(S3, 2); (void)readNorm(S4, 3);
    delay(5);
  }
}

static inline void sendToClient(const char* msg) {
  if (wifi_enable && net.hasClient()) net.clientRef().println(msg);
}

bool checkTurnStop(bool centerHigh, unsigned long now) {
  if (!turnArmed) {
    if (!centerHigh) turnArmed = true;
    return false;
  }
  return centerHigh;
}

// ========== IDENTIFY CHECKPOINT BY PATTERN ==========
char identifyCheckpoint(int n1, int n2, int n3, int n4) {
  bool h1 = (n1 >= 700), l1 = (n1 < 300);
  bool h2 = (n2 >= 700), l2 = (n2 < 300);
  bool h3 = (n3 >= 700), l3 = (n3 < 300);
  bool h4 = (n4 >= 700), l4 = (n4 < 300);

  if ((h1 && l2 && l3 && h4) || (h1 && h2 && l3 && h4) || (h1 && l2 && h3 && h4)) return 'C';
  if (l1 && l2 && l3 && h4){ return 'B' ;}
  if ((h1 && h2 && l3 && l4) || (h1 && h2 && h3 && l4)) return 'A';
  return '\0';
}

// ========== PATH COMPUTATION HELPERS ==========
String stripCheckpoints(String s) {
  String r = "";
  for (int i = 0; i < (int)s.length(); i++) {
    char c = s.charAt(i);
    if (c != 'A' && c != 'B' && c != 'C') r += c;
  }
  return r;
}

String reversePath(String path) {
  String rev = "";
  for (int i = path.length() - 1; i >= 0; i--) {
    char c = path.charAt(i);
    if (c == 'R') rev += 'L';
    else if (c == 'L') rev += 'R';
    else rev += c;
  }
  return rev;
}

int commonPrefixLen(String a, String b) {
  int len = min((int)a.length(), (int)b.length());
  for (int i = 0; i < len; i++) {
    if (a.charAt(i) != b.charAt(i)) return i;
  }
  return len;
}

String simplifyPath(String path) {
  bool changed = true;
  while (changed) {
    changed = false;
    int lenBefore = path.length();
    path.replace("RUR", "S");
    path.replace("LUL", "S");
    path.replace("RUL", "U");
    path.replace("LUR", "U");
    path.replace("SUL", "R");
    path.replace("LUS", "R");
    path.replace("SUR", "L");
    path.replace("RUS", "L");
    path.replace("SUS", "U");
    if (path.length() != lenBefore) changed = true;
  }
  return path;
}

String extractJunctions(String path) {
  String result = "";
  for (int i = 0; i < (int)path.length(); i++) {
    char c = path.charAt(i);
    if (c == 'R' || c == 'L' || c == 'S') result += c;
  }
  return result;
}

// ================== RELATIVE TURN CALCULATOR ==================
// ฟังก์ชันนี้จะแปลงทิศทางให้ถูกต้องเมื่อรถกลับออกมาจากสาขาอื่น
char getRelativeTurn(char fromDir, char toDir) {
  int hin = (fromDir == 'L') ? 1 : (fromDir == 'S') ? 2 : 3;
  int hout = (toDir == 'L') ? 3 : (toDir == 'S') ? 0 : 1;
  int diff = (hout - hin + 4) % 4;
  if (diff == 0) return 'S';
  if (diff == 1) return 'R';
  if (diff == 3) return 'L';
  return 'U';
}

String buildSegment(String p1, String p2) {
  int p = commonPrefixLen(p1, p2);
  if (p == (int)p1.length() && p == (int)p2.length()) return "";
  
  if (p == (int)p1.length()) return "U" + p2.substring(p);
  if (p == (int)p2.length()) return reversePath(p1.substring(p));

  String back = reversePath(p1.substring(p + 1));
  char turn = getRelativeTurn(p1.charAt(p), p2.charAt(p));
  String forward = p2.substring(p + 1);
  return back + turn + forward;
}
// =============================================================

// ================== SETUP ==================
void setup() {
  Serial.begin(115200);

  if (wifi_enable) {
    bool ok = net.begin(ssid, password);
    if (!ok) { Serial.println("WiFi failed"); while (1) delay(1000); }
  }

  pinMode(EN, OUTPUT); digitalWrite(EN, HIGH);
  pinMode(AIN1, OUTPUT); pinMode(AIN2, OUTPUT);
  pinMode(BIN1, OUTPUT); pinMode(BIN2, OUTPUT);
  pinMode(S1, INPUT); pinMode(S2, INPUT);
  pinMode(S3, INPUT); pinMode(S4, INPUT);
  pinMode(15, OUTPUT); pinMode(4, OUTPUT); pinMode(5, OUTPUT);

  analogReadResolution(12);
  analogSetAttenuation(ADC_11db);

  motorA(0); motorB(0);

  Serial.println("Calibrating...");
  calibrate(2000);
  Serial.println("Done.");

  resetPID();
  int check_start_only_on_line = 0;
  while (check_start_only_on_line == 0) {
    int n1 = readNorm(S1, 0); int n2 = readNorm(S2, 1);
    int n3 = readNorm(S3, 2); int n4 = readNorm(S4, 3);
    if (n1 < 300 && n2 > 800 && n3 > 800 && n4 < 300) {
      check_start_only_on_line = 1;
    }
  }
  motorA(baseSpeed); motorB(baseSpeed);
}

// ================== LOOP ==================
int first_time = 0;
int glich_zero_count = 0;

void loop() {
  if (wifi_enable) net.update();
  if (first_time == 0) { sendToClient("F"); first_time = 1; }

  unsigned long now = millis();

  int n1 = readNorm(S1, 0); int n2 = readNorm(S2, 1);
  int n3 = readNorm(S3, 2); int n4 = readNorm(S4, 3);
  
  
  if ((n1 == 0 || n2 == 0 || n3 == 0 || n4 == 0) && glich_zero_count == 0) {
    digitalWrite(15, 1); digitalWrite(5, 1); digitalWrite(4, 1);
    delay(100);
    digitalWrite(15, 0); digitalWrite(5, 0); digitalWrite(4, 0);
    delay(100);
    return;
  } else if (n1 != 0 && n2 != 0 && n3 != 0 && n4 != 0) {
    glich_zero_count = 1;
    if (checkpointCount == 1) digitalWrite(15, 1);
    else if (checkpointCount == 2) { digitalWrite(15, 1); digitalWrite(4, 1); }
    else if (checkpointCount == 3) { digitalWrite(15, 1); digitalWrite(5, 1); digitalWrite(4, 1); }
  }

  bool allLow  = (n1 <= 300 && n2 <= 300 && n3 <= 300 && n4 <= 300);
  bool allHigh = (n1 >= 800 && n2 >= 800 && n3 >= 800 && n4 >= 800);
  bool centerHigh = (n2 >= 800 || n3 >= 800);

  // ================== STATE MACHINE ==================
  switch (runState) {

    // ==================== FOLLOW ====================
    case FOLLOW: {
      confirmLow  = allLow  ? (confirmLow + 1)  : 0;
      confirmHigh = allHigh ? (confirmHigh + 1) : 0;
      Serial.print(n1); Serial.print(" ");
      Serial.print(n2);Serial.print(" ");
      Serial.print(n3);Serial.print(" ");
      Serial.println(n4);
      if (confirmHigh >= CONFIRM_N) {
        confirmLow = confirmHigh = 0;
        actionStartMs = now;

        if (currentMode == MODE_EXPLORE) {
          resetTurnStop();
          runState = STRAIGHT_BEFORE_LTURN;
          motorA(baseSpeed); motorB(baseSpeed);
          path_map += 'L';
          Serial.println("L");
        }
        else if (currentMode == MODE_RETURN) {
          if (returnJunctionIndex < (int)returnJunctionsStr.length()) {
            char action = returnJunctionsStr.charAt(returnJunctionIndex++);
            Serial.print("RET: "); Serial.println(action);
            if (action == 'R') {
              resetTurnStop(); runState = STRAIGHT_BEFORE_RTURN;
            } else if (action == 'L') {
              resetTurnStop(); runState = STRAIGHT_BEFORE_LTURN;
            } else if (action == 'U') {
              resetTurnStop(); runState = UTURN;
            } else {
              runState = STRAIGHT_THROUGH;
            }
            motorA(baseSpeed); motorB(baseSpeed);
            if (returnJunctionIndex >= (int)returnJunctionsStr.length()) nearHome = true;
          }
        }
        else { // MODE_SECOND
          if (optJunctionIndex < (int)optJunctions.length()) {
            char action = optJunctions.charAt(optJunctionIndex++);
            Serial.print("OPT: "); Serial.println(action);
            if (action == 'R') {
              resetTurnStop(); runState = STRAIGHT_BEFORE_RTURN;
            } else if (action == 'L') {
              resetTurnStop(); runState = STRAIGHT_BEFORE_LTURN;
            } else if (action == 'U') {
              resetTurnStop(); runState = UTURN;
            } else {
              runState = STRAIGHT_THROUGH;
            }
            motorA(baseSpeed); motorB(baseSpeed);
          }
        }
        prevMs = now; return;
      }

      if (confirmLow >= 30) {
        
        confirmLow = confirmHigh = 0;

        if (currentMode == MODE_RETURN && nearHome) {
          runState = HOME_UTURN;
          resetTurnStop();
          motorA(TURN_PWM); motorB(-TURN_PWM);
          actionStartMs = now;
          Serial.println(">> HOME - U-TURN");
          prevMs = now; return;
        }

        if (currentMode == MODE_SECOND && optJunctionIndex >= (int)optJunctions.length()) {
          runState = FINAL_UTURN; 
          resetTurnStop();
          motorA(TURN_PWM); motorB(-TURN_PWM);
          Serial.println(">> FINAL MISSION COMPLETE! DOING U-TURN");
          actionStartMs = now;
          prevMs = now; return;
        }

        runState = STRAIGHT_CHECK;
        straightCheckStart = now;
        motorA(baseSpeed - 30); motorB(baseSpeed - 30);
        Serial.println(">> STRAIGHT_CHECK");
        prevMs = now; return;
      }
      break;
    }

    // ==================== STRAIGHT_CHECK ====================
    case STRAIGHT_CHECK: {
      motorA(baseSpeed - 30); motorB(baseSpeed - 30);

      char cp = identifyCheckpoint(n1, n2, n3, n4);

      if (cp != '\0') {
        if (cp == pendingCP) {
          cpConfirmCount++;
        } else {
          pendingCP = cp;
          cpConfirmCount = 1;
        }
      }

      if (cpConfirmCount >= CP_CONFIRM_N) {
        char confirmedCP = pendingCP;
        cpConfirmCount = 0;
        pendingCP = '\0';
        unsigned long traveledMs = now - straightCheckStart;

        if (currentMode == MODE_RETURN) {
          reverseDuration = traveledMs; reverseStartMs = now;
          runState = REVERSE_BACK;
          motorA(-baseSpeed); motorB(-baseSpeed);
          prevMs = now; return;
        }

        Serial.print("CHECKPOINT_CONFIRMED: "); 
        Serial.println(confirmedCP);

        if (checkpointCount == 0) { 
          foundA = true; rawPathToA = path_map; checkpointCount = 1; 
        }
        else if (checkpointCount == 1) { 
          foundB = true; rawPathToB = path_map; checkpointCount = 2; 
        }
        else if (checkpointCount == 2) { 
          foundC = true; rawPathToC = path_map; checkpointCount = 3; 
        }

        if (currentMode == MODE_EXPLORE) {
          path_map += confirmedCP; 
          path_map += 'U';
          if (checkpointCount >= 3) {
             stopAfterUTurn = true; 
          }
        }

        reverseDuration = traveledMs; reverseStartMs = now;
        runState = REVERSE_BACK;
        motorA(-baseSpeed); motorB(-baseSpeed);
        prevMs = now; return;
      }

      if (now - straightCheckStart >= STRAIGHT_CHECK_MS) {
        cpConfirmCount = 0; pendingCP = '\0'; 
        if (currentMode == MODE_EXPLORE) path_map += 'U';
        unsigned long traveledMs = now - straightCheckStart;
        reverseDuration = traveledMs; reverseStartMs = now;
        runState = REVERSE_BACK;
        motorA(-baseSpeed); motorB(-baseSpeed);
        prevMs = now; return;
      }
      prevMs = now; return;
    }

    // ==================== REVERSE_BACK ====================
    case REVERSE_BACK: {
      motorA(-baseSpeed); motorB(-baseSpeed);
      if (now - reverseStartMs >= reverseDuration) {
        motorA(0); motorB(0);
        runState = UTURNTHEN;
        resetTurnStop();
        motorA(TURN_PWM); motorB(-TURN_PWM);
        actionStartMs = now;
      }
      prevMs = now; return;
    }

    // ==================== UTURNTHEN ====================
    case UTURNTHEN: {
      motorA(TURN_PWM); motorB(-TURN_PWM);
      if (stopAfterUTurn) {
        if (now - actionStartMs >= 800) {
          motorA(0); motorB(0);
          motorA(0); motorB(0);
          Serial.println(">> STOP (all checkpoints found, after 1s U-Turn)");

          unsigned long blinkStart = millis();
          while (millis() - blinkStart < 3000) {
            motorA(0); motorB(0);
            digitalWrite(15, 1); digitalWrite(4, 1); digitalWrite(5, 1); delay(200);
            digitalWrite(15, 0); digitalWrite(4, 0); digitalWrite(5, 0); delay(200);
          }

          Serial.print("Raw Path: "); Serial.println(path_map);

          // แก้ไขวิธีต่อเส้นทางของ รอบ 2 ให้คำนวณทิศทางหัวรถจริงๆ
          String simA = extractJunctions(simplifyPath(stripCheckpoints(rawPathToA)));
          String simB = extractJunctions(simplifyPath(stripCheckpoints(rawPathToB)));
          String simC = extractJunctions(simplifyPath(stripCheckpoints(rawPathToC)));

          String seg1 = simA;
          String seg2 = buildSegment(simA, simB);
          String seg3 = buildSegment(simB, simC);
          String seg4 = reversePath(simC);

          optJunctions = seg1 + seg2 + seg3 + seg4;
          Serial.print("Round2 Junctions: "); Serial.println(optJunctions);

          String simHome = simplifyPath(stripCheckpoints(path_map));
          returnJunctionsStr = extractJunctions(reversePath(simHome));
          returnJunctionIndex = 0;
          Serial.print("Return Path (Optimized): "); Serial.println(returnJunctionsStr);

          currentMode = MODE_RETURN;
          nearHome = (returnJunctionsStr.length() == 0);
          checkpointCount = 0; stopAfterUTurn = false;
          foundA = false; foundB = false; foundC = false;
          confirmLow = 0; confirmHigh = 0;
          
          runState = FOLLOW;
          resetPID();
          motorA(baseSpeed); motorB(baseSpeed);
          Serial.println(">> START RETURN HOME");
          prevMs = millis(); return;
        }
      } 
      else {
        motorA(TURN_PWM); motorB(-TURN_PWM);
        if (checkTurnStop(centerHigh, now) && (now-actionStartMs) >= 700) {
          motorA(0); motorB(0); 
          runState = FOLLOW; 
          resetPID();
        }
      }
      prevMs = now; return;
    }

    // ==================== HOME_UTURN ====================
    case HOME_UTURN: {
      motorA(TURN_PWM); motorB(-TURN_PWM);
      if (checkTurnStop(centerHigh, now) && (now-actionStartMs) >= 700 ) {
        motorA(0); motorB(0);

        Serial.println(">> ARRIVED HOME, WAIT 3 SECONDS BEFORE ROUND 2");
        unsigned long blinkStart = millis();
        while (millis() - blinkStart < 3000) {
          motorA(0); motorB(0); 
          digitalWrite(15, 1); digitalWrite(5, 1); digitalWrite(4, 1); delay(200);
          digitalWrite(15, 0); digitalWrite(5, 0); digitalWrite(4, 0); delay(200);
        }

        currentMode = MODE_SECOND;
        optJunctionIndex = 0; checkpointCount = 0;
        foundA = false; foundB = false; foundC = false;
        confirmLow = 0; confirmHigh = 0;
        runState = FOLLOW;
        resetPID();
        Serial.println(">> START SECOND RUN (Optimized)");
      }
      prevMs = now; return;
    }

    // ==================== FINAL_UTURN ====================
    case FINAL_UTURN: {
      motorA(TURN_PWM); motorB(-TURN_PWM);
      if (checkTurnStop(centerHigh, now) && (now-actionStartMs) >= 700) {
        motorA(0); motorB(0);
        Serial.println(">> FINAL U-TURN DONE! BLINKING SEQUENTIALLY FOREVER");
        
        while (1) {
          motorA(0); motorB(0);
          digitalWrite(15, 1); delay(200); digitalWrite(15, 0); delay(100);
          digitalWrite(5, 1);  delay(200); digitalWrite(5, 0);  delay(100);
          digitalWrite(4, 1);  delay(200); digitalWrite(4, 0);  delay(100);
        }
      }
      prevMs = now; return;
    }

    // ==================== STRAIGHT_BEFORE_RTURN ====================
    case STRAIGHT_BEFORE_RTURN: {
      motorA(baseSpeed); motorB(baseSpeed);
      if (now - actionStartMs >= STRAIGHT_MS) {
        runState = RTURN;
        resetTurnStop();
        motorA(TURN_PWM); motorB(0);
      }
      prevMs = now; return;
    }

    // ==================== RTURN ====================
    case RTURN: {
      motorA(TURN_PWM); motorB(0);
      if (checkTurnStop(centerHigh, now) && (now - actionStartMs) >= STRAIGHT_MS + 700) {
        motorA(0); motorB(0);
        runState = FOLLOW;
        confirmLow = 0; confirmHigh = 0; resetPID();
      }
      prevMs = now; return;
    }

    // ==================== STRAIGHT_BEFORE_LTURN ====================
    case STRAIGHT_BEFORE_LTURN: {
      motorA(baseSpeed); motorB(baseSpeed);
      if (now - actionStartMs >= STRAIGHT_MS) {
        runState = LTURN;
        resetTurnStop();
        motorA(0); motorB(TURN_PWM);
      }
      prevMs = now; return;
    }

    // ==================== LTURN ====================
    case LTURN: {
      motorA(0); motorB(TURN_PWM);
      if (checkTurnStop(centerHigh, now) && (now - actionStartMs) >= STRAIGHT_MS + 700) {
        motorA(0); motorB(0);
        runState = FOLLOW;
        confirmLow = 0; confirmHigh = 0; resetPID();
      }
      prevMs = now; return;
    }

    // ==================== STRAIGHT_THROUGH ====================
    case STRAIGHT_THROUGH: {
      motorA(baseSpeed); motorB(baseSpeed);
      if (now - actionStartMs >= STRAIGHT_MS) {
        runState = FOLLOW;
        confirmLow = 0; confirmHigh = 0; resetPID();
        Serial.println(">> Straight through Junction Done");
      }
      prevMs = now; return;
    }

    case UTURN: {
      motorA(TURN_PWM); motorB(-TURN_PWM);
      if (checkTurnStop(centerHigh, now)) {
        motorA(0); motorB(0); runState = FOLLOW; resetPID();
      }else{
        Serial.println(checkTurnStop( centerHigh, now) );
      }
      prevMs = now; return;
    }
  }

  // ================== PID FOLLOW ==================
  float error = computeError(n1, n2, n3, n4);
  float dt = (now - prevMs) / 1000.0f;
  if (dt <= 0) dt = 0.001f;
  prevMs = now;

  integral += error * dt;
  integral = constrain(integral, -1.0f, 1.0f);

  float derivative = (error - prevError) / dt;
  prevError = error;

  float corr = Kp * error + Ki * integral + Kd * derivative;

  int left  = clampi((int)(baseSpeed + corr), 0, maxSpeed);
  int right = clampi((int)(baseSpeed - corr), 0, maxSpeed);

  motorA(left); motorB(right);
}