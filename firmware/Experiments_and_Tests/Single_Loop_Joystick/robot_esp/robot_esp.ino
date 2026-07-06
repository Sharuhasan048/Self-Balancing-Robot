/*
 * ============================================================
 *  ROBOT ESP32 — Self-Balancing + ESP-NOW Receiver
 * ============================================================
 *
 *  Based on: Self_PID_Dir_deg_serial
 *  Changes from base:
 *    REMOVED: Serial commands w/s/a/d/x/h/j/t
 *    REMOVED: moveDuration auto-decay logic
 *    KEPT:    Serial commands p** i** o** (PID bench tuning via USB)
 *    ADDED:   Serial command a** (set max lean degree, default 5.0°)
 *    ADDED:   ESP-NOW receiver (gets pitchOffset/yawOffset/armed from controller)
 *    ADDED:   300ms failsafe (no packet → zero all offsets, robot stands still)
 *    ADDED:   Joystick arm/disarm via packet.armed flag
 *
 *  State Machine:
 *    DISARMED  → [Physical BUTTON]  → CALIBRATING (1 sec still) → READY
 *    READY     → [Joystick SW press / Physical BUTTON] → ARMED
 *    ARMED     → [Joystick SW press / Physical BUTTON] → DISARMED
 *
 *  Serial Commands (USB bench tuning only):
 *    p18.5  → Set Kp
 *    i230.0 → Set Ki (resets integral to prevent jerk)
 *    o0.55  → Set Kd
 *    a5.0   → Set max lean degree (default 5.0°)
 *
 *  STEP 1: Open Serial, read "MAC: XX:XX:XX:XX:XX:XX", give to controller.
 *  STEP 2: Flash controller with that MAC, then test.
 * ============================================================
 */

#include <Wire.h>
#include <esp_now.h>
#include <WiFi.h>
#include "Kalman.h"

// ==================== HARDWARE PINS ====================
const int START_BUTTON = 4;

// Motor A (Left)
const int PWMA = 14;
const int AIN1 = 26;
const int AIN2 = 25;

// Motor B (Right)
const int PWMB = 13;
const int BIN1 = 27;
const int BIN2 = 19;

// I2C (MPU6050)
const int SDA_PIN = 22;
const int SCL_PIN = 23;

// ==================== STATE MACHINE ====================
enum SystemState { DISARMED, CALIBRATING, READY, ARMED };
SystemState currentState = DISARMED;
unsigned long lastButtonPress = 0;

// Calibration
unsigned long calibStartTime = 0;
float angleSum   = 0;
int   calibCount = 0;

// ==================== IMU / KALMAN ====================
const uint8_t IMUAddress = 0x68;
#define RESTRICT_PITCH
Kalman kalmanX;
Kalman kalmanY;

double accX, accY, accZ;
double gyroX, gyroY, gyroZ;
int16_t tempRaw;

double gyroXangle, gyroYangle;
double compAngleX, compAngleY;
double kalAngleX, kalAngleY;

uint32_t timer;
uint8_t i2cData[14];

// ==================== PID ====================
float refAngle = 0.0;

float Kp = 18.0;
float Ki = 230.0;
float Kd = 0.55;

float err_integral = 0.0;
const float integral_limit = 800.0;
float pid_output = 0;

// Motor constraints
int max_speed       = 180;
int min_speed       = 20;
int motor_rest_limit = 40;

// ==================== MOVEMENT (set by joystick via ESP-NOW) ====================
volatile float pitchOffset = 0.0;
volatile float yawOffset   = 0.0;
volatile bool  joystickArmed = false;
volatile bool  newPacketFlag = false;

float maxLeanDegree = 5.0;  // Adjustable via 'a**' serial command

// Failsafe: if no packet received in this time → zero offsets
const unsigned long FAILSAFE_MS = 300;
unsigned long lastPacketTime = 0;

// Speed-based drift correction (slope handling)
float dynamic_ref_angle = 0.0;
float current_speed_est = 0.0;
float speed_Kp = 0.0001; // Adjustable via 's**'

// ==================== ESP-NOW PACKET ====================
typedef struct {
  float pitchOffset;
  float yawOffset;
  bool  button_down;
} JoystickPacket;

JoystickPacket rxPacket;

// --- ESP-NOW Receive Callback (runs on WiFi task, keep short) ---
#if ESP_ARDUINO_VERSION >= ESP_ARDUINO_VERSION_VAL(3, 0, 0)
void onDataReceive(const esp_now_recv_info_t *info, const uint8_t *data, int len) {
#else
void onDataReceive(const uint8_t *mac, const uint8_t *data, int len) {
#endif
  if (len == sizeof(JoystickPacket)) {
    memcpy(&rxPacket, data, sizeof(JoystickPacket));
    lastPacketTime = millis();
    newPacketFlag  = true;
  }
}

// ==================== IMU HELPERS ====================
bool imuWrite(uint8_t reg, uint8_t data, bool stop) { return imuWrite(reg, &data, 1, stop); }
bool imuWrite(uint8_t reg, uint8_t *data, uint8_t len, bool stop) {
  Wire.beginTransmission(IMUAddress);
  Wire.write(reg);
  Wire.write(data, len);
  uint8_t rcode = Wire.endTransmission(stop);
  if (rcode) { Serial.print(F("imuWrite err: ")); Serial.println(rcode); return true; }
  return false;
}
uint8_t imuRead(uint8_t reg, uint8_t *data, uint8_t nbytes) {
  uint32_t t;
  Wire.beginTransmission(IMUAddress);
  Wire.write(reg);
  uint8_t rcode = Wire.endTransmission(false);
  if (rcode) { Serial.print(F("imuRead err: ")); Serial.println(rcode); return rcode; }
  Wire.requestFrom(IMUAddress, nbytes, (uint8_t)true);
  for (uint8_t i = 0; i < nbytes; i++) {
    if (Wire.available()) {
      data[i] = Wire.read();
    } else {
      t = micros();
      while (((micros() - t) < 1000) && !Wire.available());
      if (Wire.available()) data[i] = Wire.read();
      else return 5;
    }
  }
  return 0;
}

// ==================== MOTOR CONTROL ====================
void setMotorLeft(float output) {
  int speed = abs(output);
  if (speed < motor_rest_limit) speed = 0;
  else if (speed < min_speed)   speed = min_speed;
  if (speed > max_speed)        speed = max_speed;

  if (output > 0)      { digitalWrite(AIN1, LOW);  digitalWrite(AIN2, HIGH); }
  else if (output < 0) { digitalWrite(AIN1, HIGH); digitalWrite(AIN2, LOW);  }
  else                 { digitalWrite(AIN1, LOW);  digitalWrite(AIN2, LOW); speed = 0; }
  analogWrite(PWMA, speed);
}

void setMotorRight(float output) {
  int speed = abs(output);
  if (speed < motor_rest_limit) speed = 0;
  else if (speed < min_speed)   speed = min_speed;
  if (speed > max_speed)        speed = max_speed;

  if (output > 0)      { digitalWrite(BIN1, HIGH); digitalWrite(BIN2, LOW);  }
  else if (output < 0) { digitalWrite(BIN1, LOW);  digitalWrite(BIN2, HIGH); }
  else                 { digitalWrite(BIN1, LOW);  digitalWrite(BIN2, LOW); speed = 0; }
  analogWrite(PWMB, speed);
}

// ==================== SETUP ====================
void setup() {
  Serial.begin(115200);
  delay(500);

  Serial.println("\n====================================");
  Serial.println("  Robot ESP32 — Balancer + ESP-NOW  ");
  Serial.println("====================================");

  // Print this ESP's MAC — give this to the controller code
  WiFi.mode(WIFI_STA);
  Serial.print("Robot MAC: ");
  Serial.println(WiFi.macAddress());
  Serial.println(">>> Copy this MAC into controller_esp.ino robotMAC[]");
  Serial.println();

  // PID serial commands
  Serial.println("Serial Tuning Commands (USB only):");
  Serial.println("  p18.0  → Set Kp");
  Serial.println("  i230.0 → Set Ki");
  Serial.println("  o0.55  → Set Kd");
  Serial.println("  a5.0   → Set Max Lean Degree (default 5.0 deg)");
  Serial.println("  s0.0001→ Set Speed Correction Kp (for slopes)");
  Serial.println("====================================\n");

  // Hardware init
  pinMode(START_BUTTON, INPUT_PULLUP);
  Wire.begin(SDA_PIN, SCL_PIN);
  Wire.setClock(400000UL);

  pinMode(PWMA, OUTPUT); pinMode(AIN1, OUTPUT); pinMode(AIN2, OUTPUT);
  pinMode(PWMB, OUTPUT); pinMode(BIN1, OUTPUT); pinMode(BIN2, OUTPUT);
  analogWrite(PWMA, 0); analogWrite(PWMB, 0);

  // --- ESP-NOW init (receiver) ---
  if (esp_now_init() != ESP_OK) {
    Serial.println("ERROR: ESP-NOW init failed!");
  } else {
    esp_now_register_recv_cb(onDataReceive);
    Serial.println("ESP-NOW receiver ready.");
  }

  // --- IMU init ---
  Serial.println("Initializing MPU6050...");
  i2cData[0] = 7;    // Sample rate = 1kHz
  i2cData[1] = 0x03; // DLPF ~42Hz
  i2cData[2] = 0x00; // Gyro ±250 deg/s
  i2cData[3] = 0x00; // Accel ±2g
  while (imuWrite(0x19, i2cData, 4, false));
  while (imuWrite(0x6B, 0x01, true));
  while (imuRead(0x75, i2cData, 1));
  delay(100);
  while (imuRead(0x3B, i2cData, 6));

  int16_t raw_accX = (int16_t)((i2cData[0] << 8) | i2cData[1]);
  int16_t raw_accY = (int16_t)((i2cData[2] << 8) | i2cData[3]);
  int16_t raw_accZ = (int16_t)((i2cData[4] << 8) | i2cData[5]);

  accZ = raw_accX; accY = raw_accY; accX = raw_accZ;

#ifdef RESTRICT_PITCH
  double roll  = atan2(accY, accZ) * RAD_TO_DEG;
  double pitch = atan(-accX / sqrt(accY*accY + accZ*accZ)) * RAD_TO_DEG;
#else
  double roll  = atan(accY / sqrt(accX*accX + accZ*accZ)) * RAD_TO_DEG;
  double pitch = atan2(-accX, accZ) * RAD_TO_DEG;
#endif

  kalmanX.setAngle(roll);
  kalmanY.setAngle(pitch);
  gyroXangle = roll;  gyroYangle = pitch;
  compAngleX = roll;  compAngleY = pitch;

  timer = micros();
  Serial.println("IMU ready.");
  Serial.println("Hold robot upright and PRESS BUTTON to calibrate.");
  Serial.println("====================================\n");
}

// ==================== LOOP ====================
void loop() {

  // =============================================
  // 1. SERIAL COMMAND PARSER (p/i/o/a only)
  // =============================================
  if (Serial.available() > 0) {
    String input = Serial.readStringUntil('\n');
    input.trim();
    if (input.length() > 0) {
      char cmd = input.charAt(0);

      if (cmd == 'p' || cmd == 'P') {
        Kp = input.substring(1).toFloat();
        Serial.print(">>> Kp = "); Serial.println(Kp, 3);

      } else if (cmd == 'i' || cmd == 'I') {
        Ki = input.substring(1).toFloat();
        err_integral = 0;
        Serial.print(">>> Ki = "); Serial.println(Ki, 4);

      } else if (cmd == 'o' || cmd == 'O') {
        Kd = input.substring(1).toFloat();
        Serial.print(">>> Kd = "); Serial.println(Kd, 3);

      } else if (cmd == 'a' || cmd == 'A') {
        maxLeanDegree = input.substring(1).toFloat();
        maxLeanDegree = constrain(maxLeanDegree, 0.5f, 10.0f);
        Serial.print(">>> Max Lean = "); Serial.print(maxLeanDegree, 2); Serial.println(" deg");

      } else if (cmd == 's' || cmd == 'S') {
        speed_Kp = input.substring(1).toFloat();
        Serial.print(">>> Speed Correction Kp = "); Serial.println(speed_Kp, 6);

      } else {
        Serial.println("Unknown command. Use: p** i** o** a** s**");
      }
    }
  }

  // =============================================
  // 2. PROCESS INCOMING ESP-NOW PACKET
  // =============================================
  bool joyButtonDown = false;
  if (newPacketFlag) {
    newPacketFlag = false;

    // Clamp incoming offsets to the configured max lean
    pitchOffset = constrain(rxPacket.pitchOffset, -maxLeanDegree, maxLeanDegree);
    yawOffset   = rxPacket.yawOffset;
    joyButtonDown = rxPacket.button_down;
  }

  // =============================================
  // 3. FAILSAFE — no packet for >300ms → stop
  // =============================================
  if (currentState == ARMED && (millis() - lastPacketTime > FAILSAFE_MS)) {
    pitchOffset = 0.0;
    yawOffset   = 0.0;
    // (Does NOT disarm — robot still tries to balance, just holds still)
  }

  // =============================================
  // 4. BUTTONS — state machine (Physical + Joystick)
  // =============================================
  bool physicalButtonDown = (digitalRead(START_BUTTON) == LOW);
  
  static bool lastPhysBtn = false;
  static bool lastJoyBtn  = false;
  
  bool trigger = false;
  if (physicalButtonDown && !lastPhysBtn) trigger = true;
  if (joyButtonDown && !lastJoyBtn) trigger = true;
  
  lastPhysBtn = physicalButtonDown;
  if (millis() - lastPacketTime < FAILSAFE_MS) {
    lastJoyBtn = joyButtonDown; // Only update joy btn history if connection alive
  }

  if (trigger && (millis() - lastButtonPress) > 500) {
    lastButtonPress = millis();

    if (currentState == DISARMED) {
      currentState    = CALIBRATING;
      calibStartTime  = millis();
      angleSum        = 0;
      calibCount      = 0;
      Serial.println(">>> CALIBRATING... keep robot still for 1 second!");

    } else if (currentState == CALIBRATING) {
      // Do nothing, waiting to finish
    } else if (currentState == READY) {
      currentState = ARMED;
      err_integral = 0;
      dynamic_ref_angle = refAngle; // Reset speed correction
      current_speed_est = 0;
      Serial.println(">>> ARMED!");

    } else if (currentState == ARMED) {
      currentState = DISARMED;
      setMotorLeft(0); setMotorRight(0);
      pitchOffset = 0; yawOffset = 0;
      Serial.println(">>> DISARMED!");
    }
  }

  // =============================================
  // 5. IMU READ + KALMAN FILTER
  // =============================================
  while (imuRead(0x3B, i2cData, 14));

  int16_t raw_accX  = (int16_t)((i2cData[0]  << 8) | i2cData[1]);
  int16_t raw_accY  = (int16_t)((i2cData[2]  << 8) | i2cData[3]);
  int16_t raw_accZ  = (int16_t)((i2cData[4]  << 8) | i2cData[5]);
  tempRaw           = (int16_t)((i2cData[6]  << 8) | i2cData[7]);
  int16_t raw_gyroX = (int16_t)((i2cData[8]  << 8) | i2cData[9]);
  int16_t raw_gyroY = (int16_t)((i2cData[10] << 8) | i2cData[11]);
  int16_t raw_gyroZ = (int16_t)((i2cData[12] << 8) | i2cData[13]);

  // Axis remap (same as base code)
  accZ  =  raw_accX;
  accY  =  raw_accY;
  accX  =  raw_accZ;
  gyroX = -raw_gyroZ;
  gyroY =  raw_gyroY;
  gyroZ =  raw_gyroX;

  double dt = (double)(micros() - timer) / 1000000.0;
  timer = micros();

#ifdef RESTRICT_PITCH
  double roll  = atan2(accY, accZ) * RAD_TO_DEG;
  double pitch = atan(-accX / sqrt(accY*accY + accZ*accZ)) * RAD_TO_DEG;
#else
  double roll  = atan(accY / sqrt(accX*accX + accZ*accZ)) * RAD_TO_DEG;
  double pitch = atan2(-accX, accZ) * RAD_TO_DEG;
#endif

  double gyroXrate = gyroX / 131.0;
  double gyroYrate = gyroY / 131.0;

#ifdef RESTRICT_PITCH
  if ((roll < -90 && kalAngleX > 90) || (roll > 90 && kalAngleX < -90)) {
    kalmanX.setAngle(roll); compAngleX = roll; kalAngleX = roll; gyroXangle = roll;
  } else {
    kalAngleX = kalmanX.getAngle(roll, gyroXrate, dt);
  }
  if (abs(kalAngleX) > 90) gyroYrate = -gyroYrate;
  kalAngleY = kalmanY.getAngle(pitch, gyroYrate, dt);
#else
  if ((pitch < -90 && kalAngleY > 90) || (pitch > 90 && kalAngleY < -90)) {
    kalmanY.setAngle(pitch); compAngleY = pitch; kalAngleY = pitch; gyroYangle = pitch;
  } else {
    kalAngleY = kalmanY.getAngle(pitch, gyroYrate, dt);
  }
  if (abs(kalAngleY) > 90) gyroXrate = -gyroXrate;
  kalAngleX = kalmanX.getAngle(roll, gyroXrate, dt);
#endif

  // =============================================
  // 6. AUTO CALIBRATION (averages balance angle)
  // =============================================
  if (currentState == CALIBRATING) {
    if (millis() - calibStartTime < 1000) {
      angleSum += kalAngleX;
      calibCount++;
    } else {
      refAngle = angleSum / calibCount;
      currentState = READY;
      Serial.println("====================================");
      Serial.print("Calibration done! Zero angle: ");
      Serial.print(refAngle, 3); Serial.println(" deg");
      Serial.println("Press BUTTON or joystick SW to ARM.");
      Serial.println("====================================");
    }
  }

  // =============================================
  // 7. PID COMPUTATION + MOTOR OUTPUT
  // =============================================
  if (currentState == ARMED) {
    // A. Estimate current speed (low pass filter of motor output)
    current_speed_est = (0.95 * current_speed_est) + (0.05 * pid_output);

    // B. Target speed from joystick (-150 to +150 based on pitchOffset)
    float target_speed = (pitchOffset / maxLeanDegree) * 150.0f;

    // C. Speed error -> adjust dynamic reference angle
    float speed_err = current_speed_est - target_speed;
    dynamic_ref_angle -= speed_err * speed_Kp;
    
    // D. Clamp dynamic angle for safety (max 10 deg away from calibrated zero)
    dynamic_ref_angle = constrain(dynamic_ref_angle, refAngle - 10.0f, refAngle + 10.0f);
  } else {
    dynamic_ref_angle = refAngle;
  }

  float targetAngle = dynamic_ref_angle;
  float raw_err     = (float)kalAngleX - targetAngle;
  float err_angle   = raw_err; // No angular deadband
  float err_angvel  = (float)gyroXrate;

  if (currentState == ARMED) {
    // Integral with anti-windup (only accumulate within ±15 deg)
    if (abs(err_angle) < 15.0f) {
      err_integral += err_angle * dt;
      err_integral  = constrain(err_integral, -integral_limit, integral_limit);
    } else {
      err_integral = 0; // Reset if tipped over
    }

    pid_output = (Kp * err_angle) + (Ki * err_integral) + (Kd * err_angvel);

    if (abs(raw_err) > 60.0f) {
      // Safety cutoff — too far tilted
      setMotorLeft(0); setMotorRight(0);
    } else {
      // Differential steering: yawOffset biases left vs right
      setMotorLeft (pid_output + yawOffset);
      setMotorRight(pid_output - yawOffset);
    }
  } else {
    setMotorLeft(0);
    setMotorRight(0);
  }
}
