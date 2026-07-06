#include <Wire.h>
#include "Kalman.h"
/******************* HARDWARE PIN DEFINITIONS *********************/
const int START_BUTTON = 4; 
// Motor A (Left)
const int PWMA = 14;
const int AIN1 = 26;
const int AIN2 = 25;
// Motor B (Right)
const int PWMB = 13;
const int BIN1 = 27;
const int BIN2 = 19;
// ESP32 I2C Pins 
const int SDA_PIN = 22; 
const int SCL_PIN = 23;
/******************* SYSTEM STATE VARIABLES *********************/
enum SystemState { DISARMED, CALIBRATING, READY, ARMED };
volatile SystemState currentState = DISARMED;
unsigned long lastButtonPress = 0; 
// Calibration Variables
unsigned long calibStartTime = 0;
float angleSum = 0;
int calibCount = 0;
/******************* MPU6050 KALMAN VARIABLES *********************/
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
/******************* PID CONTROLLER VARIABLES *********************/
float refAngle = 0.0; // Overwritten by Auto-Calibration
// Starting values based on your successful LQR tuning
volatile float Kp = 18; 
volatile float Ki = 230;  // High integral to fight drift
volatile float Kd = 0.55;  
float err_integral = 0.0;
const float integral_limit = 800.0; // Anti-windup cap
float pid_output = 0;
// Motor Constraints
int max_speed = 180; 
int min_speed = 20;         // Minimum PWM to break static friction
int motor_rest_limit = 40;  // Deadband to kill micro-vibration when perfectly balanced
/******************* MOVEMENT VARIABLES *********************/
float pitchOffset = 0.0;    // Current lean applied to the robot
float yawOffset = 0.0;      // Current differential steering applied
float leanDegree = 3.0;     // The angle the robot leans when commanded (adjustable via j**)
float angleTolerance = 0.0; // Angular Deadband where the robot sleeps (adjustable via h**)
unsigned long lastCmdTime = 0;
unsigned long moveDuration = 500; // Default movement time (500 ms)
// Telemetry loop timing tracking variables
unsigned long lastLoopTime = 0;
unsigned long loopCounter = 0;
float measuredHz = 1000.0;
// Include Telemetry library here (so it has access to declared enums/variables)
#include "Telemetry.h"
void setup() {
  Serial.begin(115200);
  delay(1000); 
  
  // Start WiFi/WebSockets on Core 0
  initTelemetry();
  
  Serial.println("\n=========================================");
  Serial.println("Starting Advanced PID Movement Setup...");
  Serial.println("=========================================");
  
  // --- TUNING INSTRUCTIONS ---
  Serial.println(">>> LIVE TUNING COMMANDS:");
  Serial.println("  p** : Updates Proportional (Kp)");
  Serial.println("  i** : Updates Integral (Ki)");
  Serial.println("  o** : Updates Derivative (Kd)");
  Serial.println(">>> MOVEMENT COMMANDS:");
  Serial.println("  w, s : Move Forward / Backward");
  Serial.println("  a, d : Spin Left / Right");
  Serial.println("  x    : Emergency Stop");
  Serial.println(">>> MOVEMENT & TOLERANCE SETTINGS:");
  Serial.println("  h**  : Set Angle Tolerance/Deadband (e.g., h1.5 ignores errors < 1.5 deg)");
  Serial.println("  j**  : Set Lean Degree (e.g., j3.5 leans 3.5 degrees)");
  Serial.println("  t**  : Set Move Time (e.g., t1 = 100ms, t5 = 500ms)");
  Serial.println("=========================================\n");
  pinMode(START_BUTTON, INPUT_PULLUP); 
  
  Wire.begin(SDA_PIN, SCL_PIN);
  Wire.setClock(400000UL); // Maximum I2C Speed
  
  pinMode(PWMA, OUTPUT); pinMode(AIN1, OUTPUT); pinMode(AIN2, OUTPUT);
  pinMode(PWMB, OUTPUT); pinMode(BIN1, OUTPUT); pinMode(BIN2, OUTPUT);
  analogWrite(PWMA, 0); analogWrite(PWMB, 0);
  Serial.println("Initializing MPU6050 with Advanced DLPF...");
  
  i2cData[0] = 7;    // Sample Rate = 1kHz
  i2cData[1] = 0x03; // ADVANCED: Enable Digital Low Pass Filter (~42Hz bandwidth) 
  i2cData[2] = 0x00; // Gyro Range ±250 deg/s
  i2cData[3] = 0x00; // Accelerometer Range ±2g
  
  while (imuWrite(0x19, i2cData, 4, false)); 
  while (imuWrite(0x6B, 0x01, true)); 
  while (imuRead(0x75, i2cData, 1));
  delay(100); 
  while (imuRead(0x3B, i2cData, 6));
  
  int16_t raw_accX = (int16_t)((i2cData[0] << 8) | i2cData[1]);
  int16_t raw_accY = (int16_t)((i2cData[2] << 8) | i2cData[3]);
  int16_t raw_accZ = (int16_t)((i2cData[4] << 8) | i2cData[5]);
  
  accZ = raw_accX;  
  accY = raw_accY;  
  accX = raw_accZ;  
#ifdef RESTRICT_PITCH
  double roll  = atan2(accY, accZ) * RAD_TO_DEG;
  double pitch = atan(-accX / sqrt(accY * accY + accZ * accZ)) * RAD_TO_DEG;
#else
  double roll  = atan(accY / sqrt(accX * accX + accZ * accZ)) * RAD_TO_DEG;
  double pitch = atan2(-accX, accZ) * RAD_TO_DEG;
#endif
  kalmanX.setAngle(roll);
  kalmanY.setAngle(pitch);
  gyroXangle = roll; gyroYangle = pitch;
  compAngleX = roll; compAngleY = pitch;
  timer = micros();
  lastLoopTime = micros();
  Serial.println("SETUP COMPLETE.");
  Serial.println("Hold robot upright and PRESS BUTTON to Calibrate.");
  Serial.println("=========================================");
}
void loop() {
  // ==========================================
  // MASTER SERIAL COMMAND PARSER
  // ==========================================
  if (Serial.available() > 0) {
    String input = Serial.readStringUntil('\n'); 
    input.trim(); 
    
    if (input.length() > 0) {
      char cmd = input.charAt(0);
      
      // 1. PID Tuning
      if (cmd == 'p' || cmd == 'P') {
        Kp = input.substring(1).toFloat();
        Serial.print(">>> INJECTED: Proportional (Kp) = "); Serial.println(Kp, 3); 
        broadcastConfig(); // Update web dashboard
      } 
      else if (cmd == 'i' || cmd == 'I') {
        Ki = input.substring(1).toFloat();
        err_integral = 0; // Prevent jerks when tuning
        Serial.print(">>> INJECTED: Integral (Ki) = "); Serial.println(Ki, 4);
        broadcastConfig(); // Update web dashboard
      }
      else if (cmd == 'o' || cmd == 'O') {
        Kd = input.substring(1).toFloat();
        Serial.print(">>> INJECTED: Derivative (Kd) = "); Serial.println(Kd, 3);
        broadcastConfig(); // Update web dashboard
      }
      // 2. Movement Parameters
      else if (cmd == 'j' || cmd == 'J') {
        leanDegree = input.substring(1).toFloat();
        Serial.print(">>> INJECTED: Lean Degree = "); Serial.print(leanDegree, 2); Serial.println(" deg");
      }
      else if (cmd == 'h' || cmd == 'H') {
        angleTolerance = input.substring(1).toFloat();
        Serial.print(">>> INJECTED: Angle Tolerance (Deadband) = "); Serial.print(angleTolerance, 2); Serial.println(" deg");
      }
      else if (cmd == 't' || cmd == 'T') {
        float val = input.substring(1).toFloat();
        moveDuration = (unsigned long)(val * 100); 
        Serial.print(">>> INJECTED: Move Time = "); Serial.print(moveDuration); Serial.println(" ms");
      }
      // 3. Directional Control
      else if (cmd == 'w' || cmd == 'W') { 
        pitchOffset = leanDegree; yawOffset = 0; lastCmdTime = millis(); 
      } 
      else if (cmd == 's' || cmd == 'S') { 
        pitchOffset = -leanDegree; yawOffset = 0; lastCmdTime = millis(); 
      }
      else if (cmd == 'a' || cmd == 'A') { 
        pitchOffset = leanDegree * 0.4; // A slight forward lean helps it spin gracefully
        yawOffset = -40; 
        lastCmdTime = millis(); 
      }
      else if (cmd == 'd' || cmd == 'D') { 
        pitchOffset = leanDegree * 0.4; 
        yawOffset = 40; 
        lastCmdTime = millis(); 
      }
      else if (cmd == 'x' || cmd == 'X') { 
        pitchOffset = 0; yawOffset = 0; 
      }
    }
  }
  // Auto-Decay: Stop moving if the set duration has elapsed
  if (millis() - lastCmdTime > moveDuration && pitchOffset != 0.0) {
    pitchOffset = pitchOffset * 0.9; // Smoothly ease back to balance point
    yawOffset = 0;
    if (abs(pitchOffset) < 0.05) pitchOffset = 0.0; // Snap to zero
  }
  // ==========================================
  // STATE MACHINE & BUTTON LOGIC
  // ==========================================
  if (digitalRead(START_BUTTON) == LOW && (millis() - lastButtonPress) > 500) {
    lastButtonPress = millis(); 
    if (currentState == DISARMED) {
      currentState = CALIBRATING;
      calibStartTime = millis();
      angleSum = 0;
      calibCount = 0;
      Serial.println(">>> CALIBRATING... Keep Robot perfectly still for 1 second! <<<");
      sendState(); // Update web dashboard
    } 
    else if (currentState == READY) {
      currentState = ARMED;
      err_integral = 0; // Fresh start for integral on arming
      Serial.println(">>> ARMED: Balancing Started! <<<");
      sendState(); // Update web dashboard
    } 
    else if (currentState == ARMED) {
      currentState = DISARMED;
      setMotorLeft(0); setMotorRight(0);
      Serial.println(">>> DISARMED: Motors Stopped. Press button to recalibrate. <<<");
      sendState(); // Update web dashboard
    }
  }
  // ==========================================
  // IMU READING & KALMAN FILTER
  // ==========================================
  while (imuRead(0x3B, i2cData, 14));
  
  int16_t raw_accX = (int16_t)((i2cData[0] << 8) | i2cData[1]);
  int16_t raw_accY = (int16_t)((i2cData[2] << 8) | i2cData[3]);
  int16_t raw_accZ = (int16_t)((i2cData[4] << 8) | i2cData[5]);
  tempRaw = (int16_t)((i2cData[6] << 8) | i2cData[7]);
  int16_t raw_gyroX = (int16_t)((i2cData[8] << 8) | i2cData[9]);
  int16_t raw_gyroY = (int16_t)((i2cData[10] << 8) | i2cData[11]);
  int16_t raw_gyroZ = (int16_t)((i2cData[12] << 8) | i2cData[13]);
  // Vertical Remap & Inverted Gyro
  accZ = raw_accX;  
  accY = raw_accY;  
  accX = raw_accZ;  
  gyroX = -raw_gyroZ; 
  gyroY = raw_gyroY; 
  gyroZ = raw_gyroX;
  double dt = (double)(micros() - timer) / 1000000;
  timer = micros();
#ifdef RESTRICT_PITCH
  double roll  = atan2(accY, accZ) * RAD_TO_DEG;
  double pitch = atan(-accX / sqrt(accY * accY + accZ * accZ)) * RAD_TO_DEG;
#else
  double roll  = atan(accY / sqrt(accX * accX + accZ * accZ)) * RAD_TO_DEG;
  double pitch = atan2(-accX, accZ) * RAD_TO_DEG;
#endif
  double gyroXrate = gyroX / 131.0; 
  double gyroYrate = gyroY / 131.0; 
#ifdef RESTRICT_PITCH
  if ((roll < -90 && kalAngleX > 90) || (roll > 90 && kalAngleX < -90)) {
    kalmanX.setAngle(roll); compAngleX = roll; kalAngleX = roll; gyroXangle = roll;
  } else
    kalAngleX = kalmanX.getAngle(roll, gyroXrate, dt); 
  if (abs(kalAngleX) > 90) gyroYrate = -gyroYrate; 
  kalAngleY = kalmanY.getAngle(pitch, gyroYrate, dt);
#else
  if ((pitch < -90 && kalAngleY > 90) || (pitch > 90 && kalAngleY < -90)) {
    kalmanY.setAngle(pitch); compAngleY = pitch; kalAngleY = pitch; gyroYangle = pitch;
  } else
    kalAngleY = kalmanY.getAngle(pitch, gyroYrate, dt); 
  if (abs(kalAngleY) > 90) gyroXrate = -gyroXrate; 
  kalAngleX = kalmanX.getAngle(roll, gyroXrate, dt); 
#endif
  // ==========================================
  // AUTO CALIBRATION LOGIC
  // ==========================================
  if (currentState == CALIBRATING) {
    if (millis() - calibStartTime < 1000) {
      angleSum += kalAngleX;
      calibCount++;
    } else {
      refAngle = angleSum / calibCount;
      currentState = READY;
      Serial.println("=========================================");
      Serial.print("Calibration Complete! Zero: ");
      Serial.print(refAngle);
      Serial.println(" degrees.");
      Serial.println("Press button to START BALANCING.");
      Serial.println("=========================================");
      sendState(); // Update web dashboard
    }
  }
  // ==========================================
  // PID COMPUTATION LOGIC
  // ==========================================
  // Target angle incorporates the user's lean command for movement
  float targetAngle = refAngle + pitchOffset; 
  
  float raw_err_angle = kalAngleX - targetAngle; 
  float err_angle = 0.0;
  float err_angvel = gyroXrate; 
  // Apply the Angular Deadband (h**)
  if (abs(raw_err_angle) > angleTolerance) {
    err_angle = raw_err_angle; // Pass the error through if it exceeds the tolerance limit
  }
  if (currentState == ARMED) {
    if (abs(err_angle) < 15 && err_angle != 0.0) {
      err_integral += err_angle * dt;
      if (err_integral > integral_limit) err_integral = integral_limit;
      if (err_integral < -integral_limit) err_integral = -integral_limit;
    } else if (err_angle == 0.0) {
      // Do nothing to the integral if we are perfectly resting in the deadband
    } else {
      err_integral = 0; // Reset if the robot tips over completely
    }
    pid_output = (Kp * err_angle) + (Ki * err_integral) + (Kd * err_angvel);
    if (abs(raw_err_angle) > 60) { 
      setMotorLeft(0); setMotorRight(0); // Safety Cutoff based on raw angle
    } else {
      // Differential steering applied to the final PID output
      setMotorLeft(pid_output + yawOffset); 
      setMotorRight(pid_output - yawOffset); 
    }
  } else {
    setMotorLeft(0); setMotorRight(0); 
    pid_output = 0;
  }
  // ==========================================
  // TELEMETRY LOGGING
  // ==========================================
  unsigned long now = micros();
  float actualDt = (now - lastLoopTime) / 1000000.0;
  lastLoopTime = now;
  loopCounter++;
  measuredHz = 0.99 * measuredHz + 0.01 * (1.0 / actualDt);
  float timeS = millis() / 1000.0;
  handleTelemetry(timeS, kalAngleX, gyroXrate, targetAngle, err_angle, pid_output, measuredHz);
}
// ================================================================
// ===               DIFFERENTIAL MOTOR CONTROL                 ===
// ================================================================
void setMotorLeft(float output) {
  int speed = abs(output);
  
  if (speed < motor_rest_limit) {
    speed = 0; 
  } else if (speed < min_speed) {
    speed = min_speed; 
  }
  
  if (speed > max_speed) speed = max_speed;
  if (output > 0) { 
    digitalWrite(AIN1, LOW);  digitalWrite(AIN2, HIGH);
  } else if (output < 0) { 
    digitalWrite(AIN1, HIGH); digitalWrite(AIN2, LOW);
  } else { 
    digitalWrite(AIN1, LOW);  digitalWrite(AIN2, LOW);
    speed = 0;
  }
  analogWrite(PWMA, speed);
}
void setMotorRight(float output) {
  int speed = abs(output);
  
  if (speed < motor_rest_limit) {
    speed = 0; 
  } else if (speed < min_speed) {
    speed = min_speed; 
  }
  
  if (speed > max_speed) speed = max_speed;
  if (output > 0) { 
    digitalWrite(BIN1, HIGH); digitalWrite(BIN2, LOW); 
  } else if (output < 0) { 
    digitalWrite(BIN1, LOW);  digitalWrite(BIN2, HIGH); 
  } else { 
    digitalWrite(BIN1, LOW);  digitalWrite(BIN2, LOW);
    speed = 0;
  }
  analogWrite(PWMB, speed);
}
// ================================================================
// ===               IMU HELPER FUNCTIONS                       ===
// ================================================================
bool imuWrite(uint8_t registerAddress, uint8_t data, bool sendStop) { return imuWrite(registerAddress, &data, 1, sendStop); }
bool imuWrite(uint8_t registerAddress, uint8_t *data, uint8_t length, bool sendStop) {
  Wire.beginTransmission(IMUAddress); Wire.write(registerAddress); Wire.write(data, length);
  uint8_t rcode = Wire.endTransmission(sendStop);
  if (rcode) { Serial.print(F("imuWrite failed: ")); Serial.println(rcode); return true; }
  return false; 
}
uint8_t imuRead(uint8_t registerAddress, uint8_t *data, uint8_t nbytes) {
  uint32_t timeOutTimer; Wire.beginTransmission(IMUAddress); Wire.write(registerAddress);
  uint8_t rcode = Wire.endTransmission(false); 
  if (rcode) { Serial.print(F("imuRead failed: ")); Serial.println(rcode); return rcode; }
  Wire.requestFrom(IMUAddress, nbytes, (uint8_t)true); 
  for (uint8_t i = 0; i < nbytes; i++) {
    if (Wire.available()) data[i] = Wire.read();
    else {
      timeOutTimer = micros(); while (((micros() - timeOutTimer) < 1000) && !Wire.available());
      if (Wire.available()) data[i] = Wire.read(); else return 5; 
    }
  }
  return 0; 
}