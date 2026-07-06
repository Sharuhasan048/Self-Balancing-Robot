// HW-504 Joystick Module with ESP32
// Plots X and Y values in Arduino Serial Plotter

#define VRX 34   // X-axis pin
#define VRY 35   // Y-axis pin
#define SW  32   // Switch pin

void setup() 
{
  Serial.begin(115200);

  pinMode(SW, INPUT_PULLUP);
}

void loop() 
{
  int xValue = analogRead(VRX);
  int yValue = analogRead(VRY);
  int button = digitalRead(SW);

  // Send values for Serial Plotter
  Serial.print(xValue);
  Serial.print(" ");
  Serial.print(yValue);
  Serial.print(" ");
  Serial.println(button * 4095);  
  // makes button visible in plotter

  delay(20);
}