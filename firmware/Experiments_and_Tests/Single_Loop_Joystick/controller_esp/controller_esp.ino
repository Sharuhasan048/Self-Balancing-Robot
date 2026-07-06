/*
 * ============================================================
 *  CONTROLLER ESP32 — HW-504 Joystick → ESP-NOW → Robot
 * ============================================================
 *
 *  HW-504 Wiring:
 *    VCC  →  3.3V  (MUST use 3.3V, not 5V — ESP32 ADC is 3.3V max)
 *    GND  →  GND
 *    VRX  →  GPIO34  (ADC1 — X-axis, controls LEFT/RIGHT turning)
 *    VRY  →  GPIO35  (ADC1 — Y-axis, controls FORWARD/BACKWARD lean)
 *    SW   →  GPIO32  (Push-to-ground button; needs INPUT_PULLUP)
 *
 *  HW-504 SW pin:
 *    - Internally floating. We enable ESP32's internal pull-up → reads HIGH at rest.
 *    - When joystick stick is pressed DOWN physically, SW goes LOW.
 *    - Used here as: press to toggle ARM/DISARM on the robot.
 *
 *  ADC Note:
 *    - ESP32 ADC is 12-bit: 0–4095.
 *    - Center at rest ≈ 1800–2200 (not always exactly 2048 due to nonlinearity).
 *    - A self-calibration on boot reads the true center automatically.
 *    - A software deadzone (±DEADZONE counts) prevents drift at rest.
 *
 *  AXIS DIRECTION:
 *    - Push joystick FORWARD → VRY goes DOWN from center → pitchOffset POSITIVE
 *    - If robot goes BACKWARD when pushed forward, flip INVERT_PITCH to true.
 *    - Push joystick RIGHT  → VRX goes UP from center   → yawOffset POSITIVE
 *    - If robot turns wrong way, flip INVERT_YAW to true.
 *    - If pushing forward moves robot left/right instead, swap VRX/VRY pins.
 *
 *  STEP 1: Flash a MAC sketch to robot first, get its MAC, fill robotMAC[] below.
 *  STEP 2: Flash this to controller ESP. Open Serial @ 115200 to debug.
 * ============================================================
 */

#include <esp_now.h>
#include <WiFi.h>

// ==================== JOYSTICK PINS ====================
#define PIN_VRX   35   // X-axis → YAW  (swapped)
#define PIN_VRY   34   // Y-axis → PITCH (swapped)
#define PIN_SW    32   // Push switch (active LOW, pull-up enabled)

// ==================== DIRECTION INVERSION ====================
// Set to true if the direction is physically reversed on your robot
const bool INVERT_PITCH = false;  // flip if forward→robot goes back
const bool INVERT_YAW   = false;  // flip if right →robot turns left

// ==================== ROBOT MAC ADDRESS ====================
// REPLACE with your robot ESP32's actual MAC address.
// To get it: flash the MAC_Print example to robot and open Serial.
uint8_t robotMAC[] = {0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF};

// ==================== PACKET STRUCTURE ====================
// Must exactly match the struct in robot_esp.ino
typedef struct {
  float pitchOffset; // Forward/backward lean: -maxLean to +maxLean (degrees)
  float yawOffset;   // Turn speed differential: -40 to +40
  bool  button_down; // true = joystick switch is physically held down
} JoystickPacket;

JoystickPacket txPacket;

// ==================== CONFIG ====================
const float MAX_LEAN    = 5.0f;  // Max degrees of forward/backward lean
const float MAX_YAW     = 40.0f; // Max yaw differential speed
const int   DEADZONE    = 150;   // ADC counts deadzone around center (0–4095 scale)
const int   SEND_HZ     = 50;    // Packet send rate (Hz)
const unsigned long SEND_INTERVAL = 1000UL / SEND_HZ;

// ==================== RUNTIME VARS ====================
int adcCenterX = 2048; // Auto-calibrated on boot
int adcCenterY = 2048;

// Removed local toggle state - now we just send the raw button state

unsigned long lastSendTime = 0;
esp_now_peer_info_t peerInfo;

// ==================== HELPERS ====================
// Maps raw ADC to output range, applying deadzone around the auto-calibrated center
float joyAxisMap(int raw, int center, float maxOut, bool invert) {
  int delta = raw - center;
  if (abs(delta) < DEADZONE) return 0.0f;

  int sign = (delta > 0) ? 1 : -1;
  // Normalize beyond deadzone: 0.0 at deadzone edge, 1.0 at ADC extreme
  float norm = (float)(abs(delta) - DEADZONE) / (float)(2048 - DEADZONE);
  norm = constrain(norm, 0.0f, 1.0f);

  float out = sign * norm * maxOut;
  return invert ? -out : out;
}

#if ESP_ARDUINO_VERSION >= ESP_ARDUINO_VERSION_VAL(3, 0, 0)
void onDataSent(const esp_now_send_info_t *tx_info, esp_now_send_status_t status) {
#else
void onDataSent(const uint8_t *mac, esp_now_send_status_t status) {
#endif
  // Uncomment for send debug:
  // Serial.println(status == ESP_NOW_SEND_SUCCESS ? "SENT OK" : "SEND FAIL");
}

// ==================== SETUP ====================
void setup() {
  Serial.begin(115200);
  delay(500);
  Serial.println("\n====================================");
  Serial.println("  Controller ESP32 — Joystick Node  ");
  Serial.println("====================================");

  // SW pin: pull-up, active LOW
  pinMode(PIN_SW, INPUT_PULLUP);

  // --- Auto-calibrate joystick center at boot (keep stick centered!) ---
  Serial.println("Auto-calibrating joystick center... keep stick centered.");
  long sumX = 0, sumY = 0;
  const int CAL_SAMPLES = 200;
  for (int i = 0; i < CAL_SAMPLES; i++) {
    sumX += analogRead(PIN_VRX);
    sumY += analogRead(PIN_VRY);
    delay(5);
  }
  adcCenterX = (int)(sumX / CAL_SAMPLES);
  adcCenterY = (int)(sumY / CAL_SAMPLES);
  Serial.printf("Center calibrated: VRX=%d  VRY=%d\n", adcCenterX, adcCenterY);

  // --- ESP-NOW Init ---
  WiFi.mode(WIFI_STA);
  WiFi.disconnect();

  if (esp_now_init() != ESP_OK) {
    Serial.println("ERROR: ESP-NOW init failed!");
    while (true) delay(1000);
  }
  esp_now_register_send_cb(onDataSent);

  // Register robot as peer
  memset(&peerInfo, 0, sizeof(peerInfo));
  memcpy(peerInfo.peer_addr, robotMAC, 6);
  peerInfo.channel = 0;
  peerInfo.encrypt = false;

  if (esp_now_add_peer(&peerInfo) != ESP_OK) {
    Serial.println("ERROR: Failed to add robot peer. Check MAC address.");
    while (true) delay(1000);
  }

  Serial.println("ESP-NOW ready. Robot peer registered.");
  Serial.println("\nSerial debug format:");
  Serial.println("  VRX:<raw>  VRY:<raw> | Pitch:<deg>  Yaw:<val> | Armed:<0/1>");
  Serial.println("  (Move joystick — check which axis controls which direction)");
  Serial.println("  (SW press = toggle arm/disarm)");
  Serial.println("====================================\n");
}

// ==================== LOOP ====================
void loop() {
  // --- Read raw ADC ---
  int rawX = analogRead(PIN_VRX);
  int rawY = analogRead(PIN_VRY);

  // --- Map to control values ---
  // VRY → pitchOffset (forward/backward)
  // Note: pushing joystick FORWARD typically lowers VRY (below center)
  // We negate so that forward push = POSITIVE pitchOffset (lean forward)
  float pitchOffset = -joyAxisMap(rawY, adcCenterY, MAX_LEAN,  INVERT_PITCH);
  float yawOffset   =  joyAxisMap(rawX, adcCenterX, MAX_YAW,   INVERT_YAW);

  // --- SW Button: send raw state (active LOW) ---
  bool isDown = (digitalRead(PIN_SW) == LOW);

  // --- Build & send packet at 50Hz ---
  if (millis() - lastSendTime >= SEND_INTERVAL) {
    lastSendTime = millis();

    txPacket.pitchOffset = pitchOffset;
    txPacket.yawOffset   = yawOffset;
    txPacket.button_down = isDown;

    esp_now_send(robotMAC, (uint8_t*)&txPacket, sizeof(txPacket));

    // Serial debug
    Serial.printf("VRX:%4d VRY:%4d | Pitch:%+5.2f  Yaw:%+5.1f | BtnDown:%d\n",
                   rawX, rawY, pitchOffset, yawOffset, (int)isDown);
  }
}
