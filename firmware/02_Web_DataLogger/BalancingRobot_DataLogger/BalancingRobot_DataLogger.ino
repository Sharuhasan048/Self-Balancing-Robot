/*
  BalancingRobot_DataLogger.ino
  Self-Balancing Robot - Phase 1: Data Logger with WiFi Dashboard
  
  Fixes applied vs original Cascaded_PID_encoder_1:
  1. Fixed 200Hz (5ms) control loop
  2. Filtered derivative (1st-order low-pass, stepper-code style)
  3. Derivative-on-measurement (avoids derivative kick)
  4. Reduced motor deadband (10 vs 40)
  5. Proper integral anti-windup with conditional integration
  6. WiFi AP + WebSocket dashboard for tuning & CSV logging

  Required Libraries (install via Arduino Library Manager):
  - WebSockets by Markus Sattler (v2.x)
*/

#include <WiFi.h>
#include <WebServer.h>
#include <WebSocketsServer.h>
#include <Wire.h>
#include "Kalman.h"
#include "webpage.h"

/******************* HARDWARE PIN DEFINITIONS *********************/
const int START_BUTTON = 4;

// Motor A (Left)
const int PWMA = 14;
const int AIN1 = 26;
const int AIN2 = 25;
const int ENCA_LEFT = 18;
const int ENCB_LEFT = 5;

// Motor B (Right)
const int PWMB = 13;
const int BIN1 = 27;
const int BIN2 = 19;
const int ENCA_RIGHT = 33;
const int ENCB_RIGHT = 32;

// ESP32 I2C Pins
const int SDA_PIN = 22;
const int SCL_PIN = 23;

/******************* WiFi & WebSocket *********************/
const char* WIFI_SSID = "Redmi Note 13"; // <-- CHANGE THIS
const char* WIFI_PASS = "123456789"; // <-- CHANGE THIS
WebServer httpServer(80);
WebSocketsServer wsServer(81);
uint8_t wsClientNum = 255; // Track connected client

/******************* SYSTEM STATE *********************/
enum SystemState { DISARMED, CALIBRATING, READY, ARMED };
volatile SystemState currentState = DISARMED;
unsigned long lastButtonPress = 0;
unsigned long calibStartTime = 0;
float angleSum = 0;
int calibCount = 0;

/******************* ENCODER *********************/
volatile long encLeft = 0;
volatile long encRight = 0;

void IRAM_ATTR leftEncoderISR() {
  if (digitalRead(ENCA_LEFT) == digitalRead(ENCB_LEFT)) encLeft++;
  else encLeft--;
}
void IRAM_ATTR rightEncoderISR() {
  if (digitalRead(ENCA_RIGHT) == digitalRead(ENCB_RIGHT)) encRight++;
  else encRight--;
}

const float TICKS_PER_REV = 1400.0;
const float TICKS_TO_DEG = 360.0 / TICKS_PER_REV;

/******************* IMU / KALMAN *********************/
const uint8_t IMUAddress = 0x68;
#define RESTRICT_PITCH
Kalman kalmanX;
Kalman kalmanY;

double accX, accY, accZ, gyroX, gyroY, gyroZ;
double kalAngleX, kalAngleY;
uint32_t imuTimer;
uint8_t i2cData[14];
double gyroXrate;

/******************* FIXED TIMING *********************/
// FIX #1: Enforce a fixed 1ms (1000Hz) control loop
#define LOOP_INTERVAL_US 1000
const float dT = LOOP_INTERVAL_US / 1000000.0; // 0.001s
unsigned long lastLoopTime = 0;
unsigned long loopCounter = 0;
float measuredHz = 1000.0;

/******************* PID VARIABLES *********************/
float refAngle = 0.0; // Mechanical zero (set during calibration)

// --- Inner Loop (Angle -> Motor PWM) ---
float Kp_ang = 12.0;
float Ki_ang = 80.0;
float Kd_ang = 0.6;

// FIX #3: Filtered derivative parameters (from stepper code's PID library)
// D_filtered = lastD * df1 + (input - lastInput) * df2
// df1 = Td/(Td + N*dT),  df2 = Td*N/(Td + N*dT)
float Td_ang = 0.8;   // Derivative time constant
float N_ang = 15.0;    // Derivative filter coefficient (higher = less filtering)
float df1_ang, df2_ang; // Pre-computed coefficients

float ang_err_integral = 0.0;
float ang_lastInput = 0.0;   // FIX #4: Derivative on measurement
float ang_lastDterm = 0.0;
float ang_Dterm = 0.0;

// FIX #5: Tighter integral limit (was 800, PWM max is 175)
const float integral_limit = 80.0;
float pid_output = 0;

// --- Outer Loop (Position -> Target Angle) ---
float Kp_pos = 0.3;
float Ki_pos = 0.01;
float Kd_pos = 0.1;

volatile float pos_err_integral = 0.0;
volatile float last_position_deg = 0.0;
volatile float targetAngle = 0.0;
volatile float leanCommand = 0.0;
volatile float current_position_deg = 0.0;
volatile float pos_error = 0.0;
volatile float encSpeed = 0.0;

// --- Motor Constraints ---
// FIX #6: Reduced deadband (was 40!)
int max_speed = 175;
int min_speed = 15;
int motor_rest_limit = 10;

// --- Data streaming ---
uint8_t streamPrescaler = 0;
#define STREAM_EVERY 50  // Send data every 50th loop = 20Hz (since loop rate is now 1000Hz)

// --- FreeRTOS WiFi & Outer Loop Task (runs on Core 0, away from PID loop on Core 1) ---
unsigned long lastOuterLoopTime = 0;
const unsigned long OUTER_LOOP_INTERVAL_MS = 20; // 50Hz outer loop rate
const float dT_outer = 0.020; // 20ms

void wifiTask(void * parameter) {
  for (;;) {
    httpServer.handleClient();
    wsServer.loop();

    unsigned long nowMs = millis();
    if (nowMs - lastOuterLoopTime >= OUTER_LOOP_INTERVAL_MS) {
      lastOuterLoopTime = nowMs;

      // Calculate outer loop PID (Position -> Target Angle)
      if (currentState == ARMED) {
        current_position_deg = ((encLeft + encRight) / 2.0) * TICKS_TO_DEG;
        pos_error = 0.0 - current_position_deg;

        pos_err_integral += pos_error * dT_outer;
        pos_err_integral = constrain(pos_err_integral, -500.0, 500.0);

        // Derivative on measurement to avoid spikes
        encSpeed = (current_position_deg - last_position_deg) / dT_outer;
        last_position_deg = current_position_deg;

        leanCommand = (Kp_pos * pos_error) + (Ki_pos * pos_err_integral) - (Kd_pos * encSpeed);
        leanCommand = constrain(leanCommand, -8.0, 8.0);
        targetAngle = refAngle + leanCommand;
      } else {
        pos_err_integral = 0;
        last_position_deg = 0;
        leanCommand = 0;
        targetAngle = refAngle;
      }
    }

    delay(2); // Yield to other Core 0 tasks, but never blocks Core 1
  }
}

/******************* FUNCTION PROTOTYPES *********************/
void readIMU();
bool imuWrite(uint8_t reg, uint8_t data, bool sendStop);
bool imuWrite(uint8_t reg, uint8_t *data, uint8_t len, bool sendStop);
uint8_t imuRead(uint8_t reg, uint8_t *data, uint8_t nbytes);
void setMotorLeft(float output);
void setMotorRight(float output);
void updateDerivativeCoeffs();
void webSocketEvent(uint8_t num, WStype_t type, uint8_t *payload, size_t length);
void sendConfigToClient(uint8_t num);
void sendState();

/******************* SETUP *********************/
void setup() {
  Serial.begin(115200);
  delay(500);
  Serial.println("\n=== BalanceBot DataLogger v1.0 ===");

  // --- Pin Setup ---
  pinMode(START_BUTTON, INPUT_PULLUP);
  pinMode(ENCA_LEFT, INPUT_PULLUP); pinMode(ENCB_LEFT, INPUT_PULLUP);
  pinMode(ENCA_RIGHT, INPUT_PULLUP); pinMode(ENCB_RIGHT, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(ENCA_LEFT), leftEncoderISR, CHANGE);
  attachInterrupt(digitalPinToInterrupt(ENCA_RIGHT), rightEncoderISR, CHANGE);

  pinMode(PWMA, OUTPUT); pinMode(AIN1, OUTPUT); pinMode(AIN2, OUTPUT);
  pinMode(PWMB, OUTPUT); pinMode(BIN1, OUTPUT); pinMode(BIN2, OUTPUT);
  analogWrite(PWMA, 0); analogWrite(PWMB, 0);

  // --- IMU Init ---
  Wire.begin(SDA_PIN, SCL_PIN);
  Wire.setClock(400000UL);

  i2cData[0] = 7; i2cData[1] = 0x03; i2cData[2] = 0x00; i2cData[3] = 0x00;
  while (imuWrite(0x19, i2cData, 4, false));
  while (imuWrite(0x6B, 0x01, true));
  while (imuRead(0x75, i2cData, 1));
  delay(100);
  while (imuRead(0x3B, i2cData, 6));

  // Axis remapping for your specific IMU orientation
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

  kalmanX.setAngle(roll);
  kalmanY.setAngle(pitch);
  imuTimer = micros();

  // --- Pre-compute derivative filter coefficients ---
  updateDerivativeCoeffs();

  // --- WiFi Station Mode ---
  WiFi.mode(WIFI_STA);
  WiFi.begin(WIFI_SSID, WIFI_PASS);
  Serial.print("Connecting to WiFi: ");
  Serial.println(WIFI_SSID);
  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print(".");
  }
  Serial.println("\nWiFi connected!");
  Serial.print("IP address: ");
  Serial.println(WiFi.localIP());

  // --- HTTP Server ---
  httpServer.on("/", []() {
    httpServer.send_P(200, "text/html", INDEX_HTML);
  });
  httpServer.begin();

  // --- WebSocket ---
  wsServer.begin();
  wsServer.onEvent(webSocketEvent);

  // --- Start WiFi task pinned to Core 0 (PID loop stays on Core 1) ---
  xTaskCreatePinnedToCore(
    wifiTask,    // Function
    "wifiTask",  // Name
    10000,       // Stack size
    NULL,        // Parameter
    1,           // Priority
    NULL,        // Task handle
    0);          // Core 0 ← WiFi lives here

  Serial.print("Dashboard: http://");
  Serial.println(WiFi.localIP());
  Serial.println("Hold still & press button to calibrate.");
}

/******************* MAIN LOOP *********************/
void loop() {
  // NOTE: WiFi is handled on Core 0 via wifiTask().
  // This loop runs purely on Core 1 — no WiFi interruptions!

  unsigned long now = micros();

  // FIX #1: Fixed-rate control loop
  if (now - lastLoopTime >= LOOP_INTERVAL_US) {
    // Measure actual loop rate
    float actualDt = (now - lastLoopTime) / 1000000.0;
    lastLoopTime = now;
    loopCounter++;
    
    // Calculate Hz (smoothed)
    measuredHz = 0.99 * measuredHz + 0.01 * (1.0 / actualDt);

    // ==========================================
    // IMU READING & KALMAN FILTER
    // ==========================================
    while (imuRead(0x3B, i2cData, 14));
    int16_t raw_accX = (int16_t)((i2cData[0] << 8) | i2cData[1]);
    int16_t raw_accY = (int16_t)((i2cData[2] << 8) | i2cData[3]);
    int16_t raw_accZ = (int16_t)((i2cData[4] << 8) | i2cData[5]);
    int16_t raw_gyroX = (int16_t)((i2cData[8] << 8) | i2cData[9]);

    // Your specific axis remapping
    accZ = raw_accX; accY = raw_accY; accX = raw_accZ;
    gyroX = -raw_gyroX;

    #ifdef RESTRICT_PITCH
      double roll = atan2(accY, accZ) * RAD_TO_DEG;
    #else
      double roll = atan(accY / sqrt(accX * accX + accZ * accZ)) * RAD_TO_DEG;
    #endif

    gyroXrate = gyroX / 131.0;

    #ifdef RESTRICT_PITCH
      if ((roll < -90 && kalAngleX > 90) || (roll > 90 && kalAngleX < -90)) {
        kalmanX.setAngle(roll); kalAngleX = roll;
      } else {
        kalAngleX = kalmanX.getAngle(roll, gyroXrate, dT);
      }
    #else
      kalAngleX = kalmanX.getAngle(roll, gyroXrate, dT);
    #endif

    // ==========================================
    // BUTTON STATE MACHINE
    // ==========================================
    if (digitalRead(START_BUTTON) == LOW && (millis() - lastButtonPress) > 500) {
      lastButtonPress = millis();
      if (currentState == DISARMED) {
        currentState = CALIBRATING;
        calibStartTime = millis(); angleSum = 0; calibCount = 0;
        Serial.println("CALIBRATING...");
      } else if (currentState == READY) {
        currentState = ARMED;
        ang_err_integral = 0; pos_err_integral = 0;
        ang_lastDterm = 0; ang_lastInput = kalAngleX;
        encLeft = 0; encRight = 0;
        last_position_deg = 0;
        Serial.println("ARMED!");
      } else if (currentState == ARMED) {
        currentState = DISARMED;
        setMotorLeft(0); setMotorRight(0);
        Serial.println("DISARMED.");
      }
      sendState();
    }

    // Calibration: average angle over 1 second
    if (currentState == CALIBRATING) {
      if (millis() - calibStartTime < 1000) {
        angleSum += kalAngleX; calibCount++;
      } else {
        refAngle = angleSum / calibCount;
        currentState = READY;
        Serial.print("Mechanical Zero: "); Serial.println(refAngle);
        sendState();
      }
    }

    // ==========================================
    // CASCADED PID COMPUTATION
    // ==========================================
    float err_angle = 0;

    if (currentState == ARMED) {

      // --- INNER LOOP (Angle -> Motor PWM) ---
      // Note: Outer loop targetAngle calculation is handled on Core 0
      err_angle = kalAngleX - targetAngle;

      // P term
      float P_term = Kp_ang * err_angle;

      // I term: FIX #5 - conditional integration (only when close to balance)
      if (abs(err_angle) < 20) {
        ang_err_integral += Ki_ang * err_angle * dT;
        ang_err_integral = constrain(ang_err_integral, -integral_limit, integral_limit);
      } else {
        ang_err_integral = 0;
      }

      // D term: FIX #3 & #4 - Filtered derivative on measurement
      // This is the key technique from the stepper code's PID library:
      //   D = lastD * df1 - (input - lastInput) * df2
      // The filter smooths out noise, derivative on measurement avoids kick
      ang_Dterm = ang_lastDterm * df1_ang - (kalAngleX - ang_lastInput) * df2_ang;
      ang_lastDterm = ang_Dterm;
      ang_lastInput = kalAngleX;

      pid_output = P_term + ang_err_integral + ang_Dterm;

      // Safety cutoff
      if (abs(kalAngleX - refAngle) > 45) {
        setMotorLeft(0); setMotorRight(0);
        currentState = DISARMED;
        Serial.println("FALL DETECTED - DISARMED");
        sendState();
      } else {
        setMotorLeft(pid_output);
        setMotorRight(pid_output);
      }

    } else {
      setMotorLeft(0); setMotorRight(0);
    }

    // ==========================================
    // DATA STREAMING VIA WEBSOCKET
    // ==========================================
    streamPrescaler++;
    if (streamPrescaler >= STREAM_EVERY) {
      streamPrescaler = 0;
      if (wsClientNum != 255) {
        // Format: D,time,angle,gyroRate,targetAngle,errAngle,pidOutput,position,posError,encSpeed,loopHz
        char buf[160];
        float timeS = loopCounter * dT;
        snprintf(buf, sizeof(buf), "D,%.3f,%.3f,%.2f,%.3f,%.3f,%.1f,%.1f,%.1f,%.1f,%.0f",
          timeS, kalAngleX, gyroXrate, targetAngle, err_angle,
          pid_output, current_position_deg, pos_error, encSpeed, measuredHz);
        wsServer.sendTXT(wsClientNum, buf);
      }
    }

    // Serial debug (occasional)
    if (loopCounter % 200 == 0 && currentState == ARMED) {
      Serial.printf("Angle:%.2f Target:%.2f PID:%.1f Pos:%.1f Hz:%.0f\n",
        kalAngleX, targetAngle, pid_output, current_position_deg, measuredHz);
    }
  }
}

/******************* DERIVATIVE COEFFICIENT UPDATE *********************/
void updateDerivativeCoeffs() {
  // From stepper code PID library: PID.cpp lines 123-124
  df1_ang = Td_ang / (Td_ang + N_ang * dT);
  df2_ang = Td_ang * N_ang / (Td_ang + N_ang * dT);
  Serial.printf("Derivative filter: df1=%.4f df2=%.4f\n", df1_ang, df2_ang);
}

/******************* MOTOR CONTROL *********************/
void setMotorLeft(float output) {
  int speed = abs((int)output);
  // FIX #6: Reduced deadband with smooth transition
  if (speed < motor_rest_limit) speed = 0;
  else if (speed < min_speed) speed = min_speed;
  if (speed > max_speed) speed = max_speed;

  if (output > 0) { digitalWrite(AIN1, LOW); digitalWrite(AIN2, HIGH); }
  else if (output < 0) { digitalWrite(AIN1, HIGH); digitalWrite(AIN2, LOW); }
  else { digitalWrite(AIN1, LOW); digitalWrite(AIN2, LOW); speed = 0; }
  analogWrite(PWMA, speed);
}

void setMotorRight(float output) {
  int speed = abs((int)output);
  if (speed < motor_rest_limit) speed = 0;
  else if (speed < min_speed) speed = min_speed;
  if (speed > max_speed) speed = max_speed;

  if (output > 0) { digitalWrite(BIN1, HIGH); digitalWrite(BIN2, LOW); }
  else if (output < 0) { digitalWrite(BIN1, LOW); digitalWrite(BIN2, HIGH); }
  else { digitalWrite(BIN1, LOW); digitalWrite(BIN2, LOW); speed = 0; }
  analogWrite(PWMB, speed);
}

/******************* WEBSOCKET EVENT HANDLER *********************/
void webSocketEvent(uint8_t num, WStype_t type, uint8_t *payload, size_t length) {
  switch (type) {
    case WStype_CONNECTED:
      wsClientNum = num;
      Serial.printf("[WS] Client %u connected\n", num);
      sendConfigToClient(num);
      sendState();
      break;

    case WStype_DISCONNECTED:
      Serial.printf("[WS] Client %u disconnected\n", num);
      if (wsClientNum == num) wsClientNum = 255;
      break;

    case WStype_TEXT: {
      char *data = (char*)payload;
      Serial.printf("[WS] Received: %s\n", data);

      // Robot control commands
      if (strcmp(data, "CAL") == 0) {
        if (currentState == DISARMED) {
          currentState = CALIBRATING;
          calibStartTime = millis(); angleSum = 0; calibCount = 0;
          sendState();
        }
      }
      else if (strcmp(data, "ARM") == 0) {
        if (currentState == READY) {
          currentState = ARMED;
          ang_err_integral = 0; pos_err_integral = 0;
          ang_lastDterm = 0; ang_lastInput = kalAngleX;
          encLeft = 0; encRight = 0;
          last_position_deg = 0;
          sendState();
        }
      }
      else if (strcmp(data, "DIS") == 0) {
        currentState = DISARMED;
        setMotorLeft(0); setMotorRight(0);
        sendState();
      }
      // PID tuning commands (same letters as original serial interface)
      else if (length >= 2) {
        char cmd = data[0];
        float val = atof(data + 1);
        switch (cmd) {
          case 'p': Kp_ang = val; Serial.printf("Kp_ang = %.3f\n", Kp_ang); break;
          case 'i': Ki_ang = val; ang_err_integral = 0; Serial.printf("Ki_ang = %.3f\n", Ki_ang); break;
          case 'o': Kd_ang = val; Td_ang = val; updateDerivativeCoeffs(); Serial.printf("Kd_ang/Td = %.3f\n", Kd_ang); break;
          case 'n': N_ang = val; updateDerivativeCoeffs(); Serial.printf("N_ang = %.1f\n", N_ang); break;
          case 'a': Kp_pos = val; Serial.printf("Kp_pos = %.3f\n", Kp_pos); break;
          case 's': Ki_pos = val; pos_err_integral = 0; Serial.printf("Ki_pos = %.3f\n", Ki_pos); break;
          case 'd': Kd_pos = val; Serial.printf("Kd_pos = %.3f\n", Kd_pos); break;
        }
        sendConfigToClient(num);
      }
      break;
    }
    default: break;
  }
}

void sendConfigToClient(uint8_t num) {
  char buf[100];
  snprintf(buf, sizeof(buf), "C,%.3f,%.3f,%.3f,%.1f,%.3f,%.3f,%.3f",
    Kp_ang, Ki_ang, Kd_ang, N_ang, Kp_pos, Ki_pos, Kd_pos);
  wsServer.sendTXT(num, buf);
}

void sendState() {
  if (wsClientNum != 255) {
    char buf[10];
    snprintf(buf, sizeof(buf), "S,%d", (int)currentState);
    wsServer.sendTXT(wsClientNum, buf);
  }
}

/******************* IMU I2C HELPERS *********************/
bool imuWrite(uint8_t registerAddress, uint8_t data, bool sendStop) {
  return imuWrite(registerAddress, &data, 1, sendStop);
}
bool imuWrite(uint8_t registerAddress, uint8_t *data, uint8_t length, bool sendStop) {
  Wire.beginTransmission(IMUAddress);
  Wire.write(registerAddress);
  Wire.write(data, length);
  uint8_t rcode = Wire.endTransmission(sendStop);
  return (rcode != 0);
}
uint8_t imuRead(uint8_t registerAddress, uint8_t *data, uint8_t nbytes) {
  Wire.beginTransmission(IMUAddress);
  Wire.write(registerAddress);
  uint8_t rcode = Wire.endTransmission(false);
  if (rcode) return rcode;
  Wire.requestFrom(IMUAddress, nbytes, (uint8_t)true);
  for (uint8_t i = 0; i < nbytes; i++) {
    if (Wire.available()) data[i] = Wire.read();
    else {
      uint32_t timeOutTimer = micros();
      while (((micros() - timeOutTimer) < 1000) && !Wire.available());
      if (Wire.available()) data[i] = Wire.read();
      else return 5;
    }
  }
  return 0;
}
