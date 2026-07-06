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
SystemState currentState = DISARMED;
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

/******************* 3-STATE LQI VARIABLES *********************/
float refAngle = 0.0; // Overwritten by Auto-Calibration

// LQI Gain Matrix
float k_angle = 21.0;     // LQR Muscle (Proportional)
float k_angvel = 0.55;    // LQR Shock Absorber (Derivative)
float k_integral = 2.5;   // LQI Drift Corrector (Integral)

float err_integral = 0.0;
const float integral_limit = 50.0; // Strict clamp to prevent windup/hunting

// Motor Output Filter
float lqr_base_output = 0;
const float filter_alpha = 0.3; // EMA Filter constant (0.0 to 1.0)

int max_speed = 150; 
int min_speed = 20; // Increased to overcome static friction
int motor_rest_limit = 20; // Deadband to stop micro-vibrations

/******************* MOVEMENT VARIABLES *********************/
float pitchOffset = 0.0; // Drives forward/backward (degrees)
float yawOffset = 0.0;   // Drives left/right (PWM offset)
unsigned long lastCmdTime = 0;
unsigned long moveDuration = 500; // Default movement time (500 ms)

void setup() {
  Serial.begin(115200);
  delay(1000); 
  Serial.println("\n=========================================");
  Serial.println("KALM-RA-01 LQI Master Control Setup...");
  Serial.println("=========================================");
  Serial.println(">>> TUNING COMMANDS:");
  Serial.println("  p** : Set k_angle    (e.g., p21.5)");
  Serial.println("  i** : Set k_integral (e.g., i2.5)");
  Serial.println("  d** : Set k_angvel   (e.g., d0.55)");
  Serial.println(">>> MOVEMENT COMMANDS:");
  Serial.println("  w, a, s, d : Directional Control");
  Serial.println("  x          : Emergency Stop");
  Serial.println("  t**        : Set move time (e.g., t1 = 100ms, t5 = 500ms)");
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
      
      // 1. Tuning Commands (p, i, d)
      if (cmd == 'p' || cmd == 'P') {
        k_angle = input.substring(1).toFloat();
        Serial.print(">>> k_angle = "); Serial.println(k_angle, 3);
      } 
      else if (cmd == 'i' || cmd == 'I') {
        k_integral = input.substring(1).toFloat();
        err_integral = 0; // Prevent jerking when tuning the integral gain
        Serial.print(">>> k_integral = "); Serial.println(k_integral, 3);
      }
      else if (cmd == 'd' || cmd == 'D') {
        k_angvel = input.substring(1).toFloat();
        Serial.print(">>> k_angvel = "); Serial.println(k_angvel, 3);
      }
      // 2. Duration Command (t)
      else if (cmd == 't' || cmd == 'T') {
        float val = input.substring(1).toFloat();
        moveDuration = (unsigned long)(val * 100); 
        Serial.print(">>> MOVE TIME = "); Serial.print(moveDuration); Serial.println(" ms");
      }
      // 3. Directional Commands (w, a, s, d, x)
      else if (cmd == 'w' || cmd == 'W') { 
        pitchOffset = 2.0; yawOffset = 0; lastCmdTime = millis(); 
      } 
      else if (cmd == 's' || cmd == 'S') { 
        pitchOffset = -2.0; yawOffset = 0; lastCmdTime = millis(); 
      }
      else if (cmd == 'a' || cmd == 'A') { 
        pitchOffset = 0.8; yawOffset = -35; lastCmdTime = millis(); 
      }
      else if (cmd == 'd' || cmd == 'D') { 
        pitchOffset = 0.8; yawOffset = 35; lastCmdTime = millis(); 
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
    } 
    else if (currentState == READY) {
      currentState = ARMED;
      err_integral = 0; // Clean slate for the memory
      Serial.println(">>> ARMED: Balancing Started! <<<");
    } 
    else if (currentState == ARMED) {
      currentState = DISARMED;
      setMotorLeft(0); setMotorRight(0);
      Serial.println(">>> DISARMED: Motors Stopped. Press button to recalibrate. <<<");
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
    }
  }

  // ==========================================
  // 3-STATE LQI & MOVEMENT COMPUTATION
  // ==========================================
  float targetAngle = refAngle + pitchOffset;
  
  float err_angle = kalAngleX - targetAngle; 
  float err_angvel = gyroXrate; 

  if (currentState == ARMED) {
    // 1. Compute Safe Integral (Anti-Windup)
    if (abs(err_angle) < 15) { 
      err_integral += err_angle * dt;
      if (err_integral > integral_limit) err_integral = integral_limit;
      if (err_integral < -integral_limit) err_integral = -integral_limit;
    } else {
      err_integral = 0; // Wipe history if it falls
    }

    // 2. Compute Raw LQI Output
    float raw_output = (k_angle * err_angle) + (k_angvel * err_angvel) + (k_integral * err_integral);

    // 3. Apply EMA Low-Pass Filter
    lqr_base_output = (filter_alpha * raw_output) + ((1.0 - filter_alpha) * lqr_base_output);

    // 4. Actuate Motors with Yaw Differential Steering
    if (abs(err_angle) > 60) { 
      setMotorLeft(0); setMotorRight(0); // Safety Cutoff
    } else {
      setMotorLeft(lqr_base_output + yawOffset); 
      setMotorRight(lqr_base_output - yawOffset); 
    }
  } else {
    setMotorLeft(0); setMotorRight(0);   
  }
}

// ================================================================
// ===               DIFFERENTIAL MOTOR CONTROL                 ===
// ================================================================
void setMotorLeft(float output) {
  int speed = abs(output);
  
  // UPDATED DEADBAND & MIN SPEED
  if (speed < motor_rest_limit) speed = 0; 
  else if (speed < min_speed) speed = min_speed; 
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
  
  // UPDATED DEADBAND & MIN SPEED
  if (speed < motor_rest_limit) speed = 0; 
  else if (speed < min_speed) speed = min_speed; 
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