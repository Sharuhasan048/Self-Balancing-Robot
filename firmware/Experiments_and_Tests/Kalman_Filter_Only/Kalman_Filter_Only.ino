/*
 * ESP32 + MPU6050 - Standard Kalman Filter Only
 * Optimized for maximum sampling rate
 * 
 * Connections:
 * MPU6050 VCC -> 3.3V
 * MPU6050 GND -> GND
 * MPU6050 SCL -> GPIO 22
 * MPU6050 SDA -> GPIO 21
 */

#include <Wire.h>

#define MPU6050_ADDR 0x68
#define PWR_MGMT_1 0x6B
#define ACCEL_XOUT_H 0x3B
#define GYRO_XOUT_H 0x43

// Kalman filter variables
float angle = 0;
float bias = 0;
float P[2][2] = {{1, 0}, {0, 1}};

// Filter tuning parameters
const float Q_angle = 0.001;
const float Q_bias = 0.003;
const float R_measure = 0.03;
const float dt = 0.005;  // 5ms = 200Hz sampling

unsigned long lastTime = 0;

void setup() {
  Serial.begin(115200);
  Wire.begin();
  Wire.setClock(400000);  // 400kHz I2C
  
  // Initialize MPU6050
  Wire.beginTransmission(MPU6050_ADDR);
  Wire.write(PWR_MGMT_1);
  Wire.write(0x00);
  Wire.endTransmission(true);
  
  delay(100);
  
  // Get initial angle
  float ax, ay, az, gx, gy, gz;
  readIMU(ax, ay, az, gx, gy, gz);
  angle = atan2(ay, sqrt(ax * ax + az * az)) * 180.0 / PI;
  
  Serial.println("Raw_Angle,Kalman_Angle");
  lastTime = micros();
}

void loop() {
  unsigned long currentTime = micros();
  float deltaT = (currentTime - lastTime) / 1000000.0;
  
  if (deltaT >= dt) {
    float ax, ay, az, gx, gy, gz;
    readIMU(ax, ay, az, gx, gy, gz);
    
    float rawAngle = atan2(ay, sqrt(ax * ax + az * az)) * 180.0 / PI;
    float gyroRate = gx;
    
    // Kalman filter update
    float kalmanAngle = updateKalman(rawAngle, gyroRate, deltaT);
    
    Serial.print(rawAngle);
    Serial.print(",");
    Serial.println(kalmanAngle);
    
    lastTime = currentTime;
  }
}

void readIMU(float &ax, float &ay, float &az, float &gx, float &gy, float &gz) {
  Wire.beginTransmission(MPU6050_ADDR);
  Wire.write(ACCEL_XOUT_H);
  Wire.endTransmission(false);
  Wire.requestFrom(MPU6050_ADDR, 14, true);
  
  int16_t axRaw = Wire.read() << 8 | Wire.read();
  int16_t ayRaw = Wire.read() << 8 | Wire.read();
  int16_t azRaw = Wire.read() << 8 | Wire.read();
  Wire.read(); Wire.read();  // Skip temperature
  int16_t gxRaw = Wire.read() << 8 | Wire.read();
  int16_t gyRaw = Wire.read() << 8 | Wire.read();
  int16_t gzRaw = Wire.read() << 8 | Wire.read();
  
  ax = axRaw / 16384.0;
  ay = ayRaw / 16384.0;
  az = azRaw / 16384.0;
  gx = gxRaw / 131.0;
  gy = gyRaw / 131.0;
  gz = gzRaw / 131.0;
}

float updateKalman(float newAngle, float gyroRate, float dt) {
  // Predict
  float rate = gyroRate - bias;
  angle += dt * rate;
  
  P[0][0] += dt * (dt * P[1][1] - P[0][1] - P[1][0] + Q_angle);
  P[0][1] -= dt * P[1][1];
  P[1][0] -= dt * P[1][1];
  P[1][1] += Q_bias * dt;
  
  // Update
  float S = P[0][0] + R_measure;
  float K[2];
  K[0] = P[0][0] / S;
  K[1] = P[1][0] / S;
  
  float innovation = newAngle - angle;
  angle += K[0] * innovation;
  bias += K[1] * innovation;
  
  float P00 = P[0][0];
  float P01 = P[0][1];
  
  P[0][0] -= K[0] * P00;
  P[0][1] -= K[0] * P01;
  P[1][0] -= K[1] * P00;
  P[1][1] -= K[1] * P01;
  
  return angle;
}
