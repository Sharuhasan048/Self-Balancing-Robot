// --- MOTOR DRIVER PINS ---
// Motor A (Left)
const int PWMA = 13; 
const int AIN1 = 25; 
const int AIN2 = 26; 

// Motor B (Right)
const int PWMB = 14; 
const int BIN1 = 27; 
const int BIN2 = 12; 

// Set a safe speed (0 to 255)
const int motorSpeed = 150; 

void setup() {
  // Set all motor control pins as outputs
  pinMode(PWMA, OUTPUT);
  pinMode(AIN1, OUTPUT);
  pinMode(AIN2, OUTPUT);
  
  pinMode(PWMB, OUTPUT);
  pinMode(BIN1, OUTPUT);
  pinMode(BIN2, OUTPUT);
  
  // Ensure motors are stopped when the ESP32 turns on
  stopMotors();
}

void loop() {
  // 1. FORWARD for 2 seconds
  moveForward(motorSpeed);
  delay(2000);
  stopMotors();
  delay(500); // 0.5s pause to protect hardware from voltage spikes

  // 2. REVERSE for 2 seconds
  moveReverse(motorSpeed);
  delay(2000);
  stopMotors();
  delay(500);

  // 3. DIFFERENTIAL (Spin in place) for 2 seconds
  spinDifferential(motorSpeed);
  delay(2000);
  stopMotors();
  
  // Wait 3 seconds before repeating the whole cycle
  delay(3000); 
}

// ==========================================
// --- HELPER FUNCTIONS FOR MOTOR CONTROL ---
// ==========================================

void moveForward(int speed) {
  // Both motors forward
  digitalWrite(AIN1, HIGH);
  digitalWrite(AIN2, LOW);
  analogWrite(PWMA, speed); 

  digitalWrite(BIN1, HIGH);
  digitalWrite(BIN2, LOW);
  analogWrite(PWMB, speed); 
}

void moveReverse(int speed) {
  // Both motors reverse
  digitalWrite(AIN1, LOW);
  digitalWrite(AIN2, HIGH);
  analogWrite(PWMA, speed); 

  digitalWrite(BIN1, LOW);
  digitalWrite(BIN2, HIGH);
  analogWrite(PWMB, speed); 
}

void spinDifferential(int speed) {
  // Left motor goes forward
  digitalWrite(AIN1, HIGH);
  digitalWrite(AIN2, LOW);
  analogWrite(PWMA, speed); 

  // Right motor goes backward
  digitalWrite(BIN1, LOW);
  digitalWrite(BIN2, HIGH);
  analogWrite(PWMB, speed); 
}

void stopMotors() {
  // Cut power to both motors
  digitalWrite(AIN1, LOW);
  digitalWrite(AIN2, LOW);
  analogWrite(PWMA, 0);

  digitalWrite(BIN1, LOW);
  digitalWrite(BIN2, LOW);
  analogWrite(PWMB, 0);
}