#include <Wire.h>

// I2C Address of the MPU6050
const int MPU_ADDR = 0x68; 

// The ESP32 pins we decided on earlier
const int SDA_PIN = 23;
const int SCL_PIN = 22;

// Variables to hold the raw data
int16_t AcX, AcY, AcZ, Tmp, GyX, GyY, GyZ;

void setup() {
  Serial.begin(115200);
  
  // Start the I2C communication on our specific pins
  Wire.begin(SDA_PIN, SCL_PIN);
  
  // Wake up the MPU6050 (it starts in sleep mode)
  Wire.beginTransmission(MPU_ADDR);
  Wire.write(0x6B);  // PWR_MGMT_1 register
  Wire.write(0);     // Write 0 to wake it up
  Wire.endTransmission(true);
  
  Serial.println("MPU6050 Connection Successful! Reading Data...");
  delay(1000);
}

void loop() {
  // Tell the MPU6050 we want to start reading from register 0x3B (Accelerometer X)
  Wire.beginTransmission(MPU_ADDR);
  Wire.write(0x3B);  
  Wire.endTransmission(false);
  
  // Request 14 bytes of data (Accel, Temp, and Gyro)
  Wire.requestFrom(MPU_ADDR, 14, true);  
  
  // Read and combine the bytes into 16-bit integers
  AcX = Wire.read()<<8 | Wire.read();     
  AcY = Wire.read()<<8 | Wire.read(); 
  AcZ = Wire.read()<<8 | Wire.read(); 
  Tmp = Wire.read()<<8 | Wire.read(); 
  GyX = Wire.read()<<8 | Wire.read(); 
  GyY = Wire.read()<<8 | Wire.read(); 
  GyZ = Wire.read()<<8 | Wire.read(); 
  
  // Print the raw values to the Serial Monitor
  Serial.print("Accel X: "); Serial.print(AcX);
  Serial.print(" | Y: "); Serial.print(AcY);
  Serial.print(" | Z: "); Serial.print(AcZ);
  
  Serial.print("    ||    Gyro X: "); Serial.print(GyX);
  Serial.print(" | Y: "); Serial.print(GyY);
  Serial.print(" | Z: "); Serial.println(GyZ);
  
  // Delay so the Serial Monitor isn't scrolling too fast to read
  delay(250);
}