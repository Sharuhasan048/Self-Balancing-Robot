#include <Wire.h>
#include "Kalman.h"

/******************* HARDWARE PIN DEFINITIONS *********************/
const int START_BUTTON = 4; 

// Motor A (Left)
const int PWMA = 14;
const int AIN1 = 26;
const int AIN2 = 25;
const int ENCA_LEFT = 18;
const int ENCB_LEFT = 5;

// Motor B (Right) - Updated Pins
const int PWMB = 13;
const int BIN1 = 27;
const int BIN2 = 19;
const int ENCA_RIGHT = 33;
const int ENCB_RIGHT = 32;

// ESP32 I2C Pins 
const int SDA_PIN = 22; 
const int SCL_PIN = 23;

/******************* SYSTEM STATE VARIABLES *********************/
enum SystemState { DISARMED, CALIBRATING, READY, ARMED };
SystemState currentState = DISARMED;
unsigned long lastButtonPress = 0; 

unsigned long calibStartTime = 0;
float angleSum = 0;
int calibCount = 0;

/******************* ENCODER VARIABLES & MATH *********************/
volatile long encLeft = 0;
volatile long encRight = 0;

// Hardware Interrupts (2X Decoding via CHANGE on Phase A)
void IRAM_ATTR leftEncoderISR() {
  if (digitalRead(ENCA_LEFT) == digitalRead(ENCB_LEFT)) { encLeft++; } 
  else { encLeft--; }
}
void IRAM_ATTR rightEncoderISR() {
  if (digitalRead(ENCA_RIGHT) == digitalRead(ENCB_RIGHT)) { encRight++; } 
  else { encRight--; }
}

// MOTOR SPECS: 7 Poles * 2 (CHANGE Interrupt) * 100:1 Gearbox = 1400
const float TICKS_PER_REV = 1400.0; 
const float TICKS_TO_DEG = 360.0 / TICKS_PER_REV;

/******************* MPU6050 KALMAN VARIABLES *********************/
const uint8_t IMUAddress = 0x68; 
#define RESTRICT_PITCH 
Kalman kalmanX; 
Kalman kalmanY;

double accX, accY, accZ, gyroX, gyroY, gyroZ;
double kalAngleX, kalAngleY;   
uint32_t timer;
uint8_t i2cData[14]; 

/******************* CASCADED PID VARIABLES *********************/
float refAngle = 0.0; // Mechanical zero

// 1. OUTER LOOP (Position Control) - Tuned via a, s, d
float Kp_pos = 0.0; 
float Ki_pos = 0.0; 
float Kd_pos = 0.0; 

float pos_err_integral = 0.0;
float last_position_deg = 0.0;
float targetAngle = 0.0; // Output of the Outer Loop

// 2. INNER LOOP (Angle Control) - Tuned via p, i, o
float Kp_ang = 18.0; 
float Ki_ang = 230.0;  
float Kd_ang = 0.55;  

float ang_err_integral = 0.0;
const float integral_limit = 800.0; 
float pid_output = 0;

// Motor Constraints (8V Supply -> 6V Motor Limit)
int max_speed = 175;        // Capped at 75% duty cycle to output max 6V
int min_speed = 20;         // Minimum PWM to break static friction
int motor_rest_limit = 40;  // Deadband to kill micro-vibration

void setup() {
  Serial.begin(115200);
  delay(1000); 
  Serial.println("\n=========================================");
  Serial.println("Starting Cascaded Dual-Loop PID...");
  Serial.println("Encoder Spec: 1400 Ticks/Rev");
  Serial.println(">>> INNER LOOP (Angle): p**, i**, o**");
  Serial.println(">>> OUTER LOOP (Position): a**, s**, d**");
  Serial.println("=========================================\n");

  pinMode(START_BUTTON, INPUT_PULLUP); 
  
  pinMode(ENCA_LEFT, INPUT_PULLUP); pinMode(ENCB_LEFT, INPUT_PULLUP);
  pinMode(ENCA_RIGHT, INPUT_PULLUP); pinMode(ENCB_RIGHT, INPUT_PULLUP);
  
  // Attach 2X Decoding Interrupts
  attachInterrupt(digitalPinToInterrupt(ENCA_LEFT), leftEncoderISR, CHANGE);
  attachInterrupt(digitalPinToInterrupt(ENCA_RIGHT), rightEncoderISR, CHANGE);
  
  Wire.begin(SDA_PIN, SCL_PIN);
  Wire.setClock(400000UL); 
  
  pinMode(PWMA, OUTPUT); pinMode(AIN1, OUTPUT); pinMode(AIN2, OUTPUT);
  pinMode(PWMB, OUTPUT); pinMode(BIN1, OUTPUT); pinMode(BIN2, OUTPUT);
  analogWrite(PWMA, 0); analogWrite(PWMB, 0);

  i2cData[0] = 7; i2cData[1] = 0x03; i2cData[2] = 0x00; i2cData[3] = 0x00; 
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
  double pitch = atan(-accX / sqrt(accY * accY + accZ * accZ)) * RAD_TO_DEG;
#else
  double roll  = atan(accY / sqrt(accX * accX + accZ * accZ)) * RAD_TO_DEG;
  double pitch = atan2(-accX, accZ) * RAD_TO_DEG;
#endif

  kalmanX.setAngle(roll); kalmanY.setAngle(pitch);
  timer = micros();
  Serial.println("SETUP COMPLETE. Hold still & Press Button to Calibrate.");
}

void loop() {
  // ==========================================
  // SERIAL COMMAND PARSER (DUAL LOOP)
  // ==========================================
  if (Serial.available() > 0) {
    String input = Serial.readStringUntil('\n'); 
    input.trim(); 
    
    if (input.length() > 0) {
      char cmd = input.charAt(0);
      float val = input.substring(1).toFloat();
      
      // Inner Loop (Angle)
      if (cmd == 'p' || cmd == 'P') { Kp_ang = val; Serial.print("Kp_ang = "); Serial.println(Kp_ang); } 
      else if (cmd == 'i' || cmd == 'I') { Ki_ang = val; ang_err_integral = 0; Serial.print("Ki_ang = "); Serial.println(Ki_ang); }
      else if (cmd == 'o' || cmd == 'O') { Kd_ang = val; Serial.print("Kd_ang = "); Serial.println(Kd_ang); }
      
      // Outer Loop (Position)
      else if (cmd == 'a' || cmd == 'A') { Kp_pos = val; Serial.print("Kp_pos = "); Serial.println(Kp_pos); }
      else if (cmd == 's' || cmd == 'S') { Ki_pos = val; pos_err_integral = 0; Serial.print("Ki_pos = "); Serial.println(Ki_pos); }
      else if (cmd == 'd' || cmd == 'D') { Kd_pos = val; Serial.print("Kd_pos = "); Serial.println(Kd_pos); }
    }
  }

  // ==========================================
  // STATE MACHINE
  // ==========================================
  if (digitalRead(START_BUTTON) == LOW && (millis() - lastButtonPress) > 500) {
    lastButtonPress = millis(); 
    if (currentState == DISARMED) {
      currentState = CALIBRATING; calibStartTime = millis(); angleSum = 0; calibCount = 0;
      Serial.println("CALIBRATING...");
    } 
    else if (currentState == READY) {
      currentState = ARMED; 
      ang_err_integral = 0; pos_err_integral = 0;
      encLeft = 0; encRight = 0; // Lock absolute position 0 here
      Serial.println("ARMED! Position Locked.");
    } 
    else if (currentState == ARMED) {
      currentState = DISARMED; setMotorLeft(0); setMotorRight(0);
      Serial.println("DISARMED.");
    }
  }

  // ==========================================
  // IMU READING & KALMAN FILTER
  // ==========================================
  while (imuRead(0x3B, i2cData, 14));
  int16_t raw_accX = (int16_t)((i2cData[0] << 8) | i2cData[1]);
  int16_t raw_accY = (int16_t)((i2cData[2] << 8) | i2cData[3]);
  int16_t raw_accZ = (int16_t)((i2cData[4] << 8) | i2cData[5]);
  int16_t raw_gyroX = (int16_t)((i2cData[8] << 8) | i2cData[9]);

  accZ = raw_accX; accY = raw_accY; accX = raw_accZ;  
  gyroX = -raw_gyroX; 

  double dt = (double)(micros() - timer) / 1000000;
  timer = micros();

#ifdef RESTRICT_PITCH
  double roll  = atan2(accY, accZ) * RAD_TO_DEG;
#else
  double roll  = atan(accY / sqrt(accX * accX + accZ * accZ)) * RAD_TO_DEG;
#endif

  double gyroXrate = gyroX / 131.0; 

#ifdef RESTRICT_PITCH
  if ((roll < -90 && kalAngleX > 90) || (roll > 90 && kalAngleX < -90)) {
    kalmanX.setAngle(roll); kalAngleX = roll; 
  } else kalAngleX = kalmanX.getAngle(roll, gyroXrate, dt); 
#else
  kalAngleX = kalmanX.getAngle(roll, gyroXrate, dt); 
#endif

  if (currentState == CALIBRATING) {
    if (millis() - calibStartTime < 1000) {
      angleSum += kalAngleX; calibCount++;
    } else {
      refAngle = angleSum / calibCount; currentState = READY;
      Serial.print("Mechanical Zero: "); Serial.println(refAngle);
    }
  }

  // ==========================================
  // CASCADED PID COMPUTATION
  // ==========================================
  if (currentState == ARMED) {
    
    // --- 1. OUTER LOOP (Position -> Target Angle) ---
    // Convert raw ticks to average degrees of wheel rotation
    float current_position_deg = ((encLeft + encRight) / 2.0) * TICKS_TO_DEG;
    float pos_error = 0.0 - current_position_deg; 
    
    pos_err_integral += pos_error * dt;
    if (pos_err_integral > 1000) pos_err_integral = 1000;
    if (pos_err_integral < -1000) pos_err_integral = -1000;
    
    float pos_derivative = (current_position_deg - last_position_deg) / dt;
    last_position_deg = current_position_deg;

    float lean_command = (Kp_pos * pos_error) + (Ki_pos * pos_err_integral) + (Kd_pos * pos_derivative);
    
    // Outer Loop Safety Clamp
    if (lean_command > 8.0) lean_command = 8.0;
    if (lean_command < -8.0) lean_command = -8.0;

    targetAngle = refAngle + lean_command;

    // --- 2. INNER LOOP (Angle -> Motor PWM) ---
    float err_angle = kalAngleX - targetAngle; 
    float err_angvel = gyroXrate; 

    if (abs(err_angle) < 15) {
      ang_err_integral += err_angle * dt;
      if (ang_err_integral > integral_limit) ang_err_integral = integral_limit;
      if (ang_err_integral < -integral_limit) ang_err_integral = -integral_limit;
    } else {
      ang_err_integral = 0; 
    }

    pid_output = (Kp_ang * err_angle) + (Ki_ang * ang_err_integral) + (Kd_ang * err_angvel);

    // 70-degree physical safety cutoff
    if (abs(kalAngleX - refAngle) > 70) { 
      setMotorLeft(0); setMotorRight(0);
    } else {
      setMotorLeft(pid_output); 
      setMotorRight(pid_output); 
    }
  } else {
    setMotorLeft(0); setMotorRight(0); 
  }
}

// ================================================================
// ===               MOTOR CONTROL & IMU HELPERS                ===
// ================================================================
void setMotorLeft(float output) {
  int speed = abs(output);
  if (speed < motor_rest_limit) speed = 0; 
  else if (speed < min_speed) speed = min_speed; 
  if (speed > max_speed) speed = max_speed;

  if (output > 0) { digitalWrite(AIN1, LOW); digitalWrite(AIN2, HIGH); } 
  else if (output < 0) { digitalWrite(AIN1, HIGH); digitalWrite(AIN2, LOW); } 
  else { digitalWrite(AIN1, LOW); digitalWrite(AIN2, LOW); speed = 0; }
  analogWrite(PWMA, speed);
}

void setMotorRight(float output) {
  int speed = abs(output);
  if (speed < motor_rest_limit) speed = 0; 
  else if (speed < min_speed) speed = min_speed; 
  if (speed > max_speed) speed = max_speed;

  if (output > 0) { digitalWrite(BIN1, HIGH); digitalWrite(BIN2, LOW); } 
  else if (output < 0) { digitalWrite(BIN1, LOW); digitalWrite(BIN2, HIGH); } 
  else { digitalWrite(BIN1, LOW); digitalWrite(BIN2, LOW); speed = 0; }
  analogWrite(PWMB, speed);
}

bool imuWrite(uint8_t registerAddress, uint8_t data, bool sendStop) { return imuWrite(registerAddress, &data, 1, sendStop); }
bool imuWrite(uint8_t registerAddress, uint8_t *data, uint8_t length, bool sendStop) {
  Wire.beginTransmission(IMUAddress); Wire.write(registerAddress); Wire.write(data, length);
  uint8_t rcode = Wire.endTransmission(sendStop);
  if (rcode) return true; return false; 
}
uint8_t imuRead(uint8_t registerAddress, uint8_t *data, uint8_t nbytes) {
  uint32_t timeOutTimer; Wire.beginTransmission(IMUAddress); Wire.write(registerAddress);
  uint8_t rcode = Wire.endTransmission(false); 
  if (rcode) return rcode; 
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