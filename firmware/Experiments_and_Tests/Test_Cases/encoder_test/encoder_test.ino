// KALM-RA-01 Encoder Calibration Tool

const int ENCA_LEFT = 33;
const int ENCB_LEFT = 32;

volatile long encLeft = 0;

// Hardware Interrupt Function
void IRAM_ATTR leftEncoderISR() {
  if (digitalRead(ENCA_LEFT) == digitalRead(ENCB_LEFT)) { 
    encLeft++; 
  } else { 
    encLeft--; 
  }
}

void setup() {
  Serial.begin(115200);
  
  pinMode(ENCA_LEFT, INPUT_PULLUP);
  pinMode(ENCB_LEFT, INPUT_PULLUP);
  
  attachInterrupt(digitalPinToInterrupt(ENCA_LEFT), leftEncoderISR, CHANGE);
  
  Serial.println("Encoder Calibration Ready.");
  Serial.println("Spin the LEFT wheel exactly ONE full rotation.");
}

void loop() {
  Serial.print("Encoder Ticks: ");
  Serial.println(encLeft);
  delay(100); 
}