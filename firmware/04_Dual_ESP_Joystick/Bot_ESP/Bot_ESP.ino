/**
 * ============================================================
 *  Bot_ESP.ino  —  ROBOT BRAIN (transmitter)
 *  Folder: Self-Balancing-Robot\Two_ESP_Run\Bot_ESP\
 * ============================================================
 *
 *  WHY THIS SPLIT EXISTS:
 *  ─────────────────────────────────────────────────────────
 *  The original firmware had WiFi + WebSocket running on Core 0
 *  of the same ESP32 that drives the motors. The WiFi stack on
 *  ESP32 is notorious for causing microsecond-level interrupt
 *  stalls on Core 1 (the PID loop core) because:
 *    1. WiFi uses the hardware timer interrupt shared with all cores.
 *    2. WiFi TX/RX DMA briefly locks the shared AHB bus.
 *    3. WebSocket frame parsing can spike 300–800µs on Core 0,
 *       which steals CPU from Core 1 during task switches.
 *  Result: PID loop jitters, IMU reads get delayed, integral
 *  accumulates wrong values → robot falls, goes one-directional.
 *
 *  THIS FILE:
 *  ─────────────────────────────────────────────────────────
 *  • NO WiFi, NO HTTP, NO WebSocket — zero interference.
 *  • ESP-NOW is used INSTEAD. It is a lightweight 2.4GHz
 *    protocol that bypasses the full WiFi stack:
 *      - Runs in the radio driver layer only.
 *      - TX callback is <10µs and non-blocking.
 *      - Does NOT hold any bus locks during PID execution.
 *  • Core 0: ESP-NOW packet sender (FreeRTOS queue-based).
 *  • Core 1: Full PID loop (IMU → Kalman → GainScheduler → Motors).
 *
 *  PARTNER FILE:
 *    Self-Balancing-Robot\Two_ESP_Run\Receiver_ESP\Receiver_ESP.ino
 *    (receives packets, hosts full web dashboard on its own WiFi)
 *
 *  BEFORE UPLOADING:
 *  1. Get the MAC address of your Receiver ESP by uploading
 *     Receiver_ESP.ino first and reading Serial at 115200.
 *  2. Paste that MAC address into RECEIVER_MAC below.
 *  3. Upload this file to the Robot ESP.
 * ============================================================
 */

#include <Wire.h>
#include <esp_now.h>
#include <WiFi.h>          // needed for esp_now channel setup only
#include <esp_wifi.h>      // << ADDED THIS
#include "Kalman.h"
#include "GainScheduler.h" // copy from GainScheduler_PID folder

// ─────────────────────────────────────────────────────────────
//  !! SET THIS TO YOUR RECEIVER ESP32 MAC ADDRESS !!
//  Run Receiver_ESP.ino and read the MAC from Serial monitor.
// ─────────────────────────────────────────────────────────────
uint8_t RECEIVER_MAC[] = {0xE0, 0x8C, 0xFE, 0x5C, 0x66, 0xB4};  // << REPLACE THIS WITH RECEIVER MAC (format {0xAA, 0xBB, ...})

// ─────────────────────────────────────────────────────────────
// ESP-NOW WiFi channel — must match Receiver ESP AP channel (1)
// ─────────────────────────────────────────────────────────────
#define ESPNOW_CHANNEL  1

// ─────────────────────────────────────────────────────────────
// HARDWARE PINS (same as original)
// ─────────────────────────────────────────────────────────────
const int START_BUTTON = 4;
const int PWMA = 14; const int AIN1 = 26; const int AIN2 = 25;
const int PWMB = 13; const int BIN1 = 27; const int BIN2 = 19;
const int ENC_L_A = 18; const int ENC_L_B = 5;
const int ENC_R_A = 33; const int ENC_R_B = 32;
const int SDA_PIN = 22; const int SCL_PIN = 23;

// ─────────────────────────────────────────────────────────────
// SYSTEM STATE
// ─────────────────────────────────────────────────────────────
enum SystemState : uint8_t { DISARMED, CALIBRATING, READY, ARMED };
volatile SystemState currentState = DISARMED;
unsigned long lastButtonPress = 0;
unsigned long calibStartTime  = 0;
float angleSum  = 0; int calibCount = 0;

// ─────────────────────────────────────────────────────────────
// ENCODERS
// ─────────────────────────────────────────────────────────────
volatile long encLeft = 0, encRight = 0;
float current_position_deg = 0, pos_error_deg = 0;
float encoder_speed_degs   = 0, prev_position_deg = 0;

void IRAM_ATTR isr_encL() { if (digitalRead(ENC_L_B)) encLeft++;  else encLeft--;  }
void IRAM_ATTR isr_encR() { if (digitalRead(ENC_R_B)) encRight--; else encRight++; }

// ─────────────────────────────────────────────────────────────
// IMU / KALMAN
// ─────────────────────────────────────────────────────────────
const uint8_t IMU_ADDR = 0x68;
#define RESTRICT_PITCH
Kalman kalmanX, kalmanY;
double accX, accY, accZ, gyroX, gyroY, gyroZ;
double kalAngleX, kalAngleY;
int16_t tempRaw;
uint8_t i2cData[14];
uint32_t imuTimer;

// ─────────────────────────────────────────────────────────────
// PID VARIABLES
// ─────────────────────────────────────────────────────────────
float refAngle      = 0.0f;
float err_integral  = 0.0f;
float pid_output    = 0.0f;

// Expose for GainScheduler.h compatibility
volatile float Kp = 18.0f, Ki = 230.0f, Kd = 0.55f;
volatile float Kp_pos = GS_KP_POS, Ki_pos = GS_KI_POS, Kd_pos = GS_KD_POS;

float pos_integral    = 0.0f;
float prev_pos_err    = 0.0f;
const float pos_integral_limit = GS_POS_INT_LIMIT;  // 15.0

int max_speed        = 180;
int min_speed        = 20;
int motor_rest_limit = 40;

float pitchOffset = 0.0f, yawOffset = 0.0f, angleTolerance = 0.0f;
unsigned long lastOuterLoopTime = 0;
const unsigned long outerLoopInterval = 20; // 50Hz

// ─────────────────────────────────────────────────────────────
// ESP-NOW TELEMETRY PACKET
// ─────────────────────────────────────────────────────────────
// This struct is shared between Bot and Receiver — keep identical in both files.
typedef struct __attribute__((packed)) {
    float    timeS;
    float    angle;
    float    gyroRate;
    float    targetAngle;
    float    errAngle;
    float    pidOutput;
    float    position;
    float    posError;
    float    encSpeed;
    float    loopHz;
    uint8_t  zone;        // GainZone enum value
    uint8_t  state;       // SystemState enum value
    float    Kp_live;
    float    Ki_live;
    float    Kd_live;
} TelemetryPacket;  // 15 fields × ~4 bytes = ~60 bytes (well under 250B limit)

// FreeRTOS queue — Bot_ESP puts packets here, Core 0 task sends them
QueueHandle_t espnowQueue;
#define QUEUE_SIZE 1  // MUST be 1 to use xQueueOverwrite()

// ESP-NOW send result callback (runs in ISR context — keep it tiny)
#if ESP_IDF_VERSION >= ESP_IDF_VERSION_VAL(5, 0, 0)
void IRAM_ATTR onDataSent(const esp_now_send_info_t *info, esp_now_send_status_t status) {
#else
void IRAM_ATTR onDataSent(const uint8_t *mac, esp_now_send_status_t status) {
#endif
    // Nothing to do — fire-and-forget
    (void)status;
}

// ─────────────────────────────────────────────────────────────
// COMMAND PACKET (Receiver → Bot via ESP-NOW reply)
// ─────────────────────────────────────────────────────────────
typedef struct __attribute__((packed)) {
    char    cmdType;
    float   value;
} CommandPacket;

#if ESP_IDF_VERSION >= ESP_IDF_VERSION_VAL(5, 0, 0)
void onDataReceived(const esp_now_recv_info *info, const uint8_t *data, int len) {
#else
void onDataReceived(const uint8_t *mac, const uint8_t *data, int len) {
#endif
    if (len == sizeof(CommandPacket)) {
        CommandPacket cmd;
        memcpy(&cmd, data, sizeof(cmd));
        if (cmd.cmdType == 'C') {
            if (currentState == DISARMED) {
                currentState = CALIBRATING;
                calibStartTime = millis(); angleSum = 0; calibCount = 0;
            }
        } else if (cmd.cmdType == 'A') {
            if (currentState == READY) {
                currentState = ARMED;
                err_integral = 0; pos_integral = 0;
                encLeft = 0; encRight = 0;
                GS_init();
            }
        } else if (cmd.cmdType == 'D') {
            currentState = DISARMED;
            setMotorLeft(0); setMotorRight(0);
        } else {
            switch (cmd.cmdType) {
                case 'p': GS_TABLE[GS.zone].Kp = cmd.value; break;
                case 'i': GS_TABLE[GS.zone].Ki = cmd.value; err_integral = 0; break;
                case 'o': GS_TABLE[GS.zone].Kd = cmd.value; break;
                case 'a': Kp_pos = cmd.value; break;
                case 's': Ki_pos = cmd.value; pos_integral = 0; break;
                case 'd': Kd_pos = cmd.value; break;
            }
        }
    }
}

// Core 0 task: drains the queue and sends via ESP-NOW
void espnowSenderTask(void* param) {
    TelemetryPacket pkt;
    for (;;) {
        // Block until a packet arrives (max 50ms wait, then loop anyway)
        if (xQueueReceive(espnowQueue, &pkt, pdMS_TO_TICKS(50)) == pdTRUE) {
            esp_now_send(RECEIVER_MAC, (uint8_t*)&pkt, sizeof(pkt));
        }
        // Tiny yield so WiFi/BT stack on Core 0 can breathe
        taskYIELD();
    }
}

// Push a snapshot to the queue (called from Core 1 / main loop)
void pushTelemetry(float timeS, float angle, float gyroRate,
                   float targetAngle, float errAngle, float pidOutput,
                   float position, float posError, float encSpeed, float loopHz) {
    TelemetryPacket pkt;
    pkt.timeS       = timeS;
    pkt.angle       = angle;
    pkt.gyroRate    = gyroRate;
    pkt.targetAngle = targetAngle;
    pkt.errAngle    = errAngle;
    pkt.pidOutput   = pidOutput;
    pkt.position    = position;
    pkt.posError    = posError;
    pkt.encSpeed    = encSpeed;
    pkt.loopHz      = loopHz;
    pkt.zone        = (uint8_t)GS.zone;
    pkt.state       = (uint8_t)currentState;
    pkt.Kp_live     = GS.Kp;
    pkt.Ki_live     = GS.Ki;
    pkt.Kd_live     = GS.Kd;
    // Overwrite oldest if queue full (don't block PID loop)
    xQueueOverwrite(espnowQueue, &pkt);
}

// ─────────────────────────────────────────────────────────────
// IMU HELPERS
// ─────────────────────────────────────────────────────────────
bool imuWrite(uint8_t reg, uint8_t data, bool stop)   { return imuWrite(reg, &data, 1, stop); }
bool imuWrite(uint8_t reg, uint8_t *d, uint8_t n, bool stop) {
    Wire.beginTransmission(IMU_ADDR); Wire.write(reg); Wire.write(d, n);
    uint8_t rc = Wire.endTransmission(stop);
    if (rc) { Serial.print("imuW fail:"); Serial.println(rc); return true; }
    return false;
}
uint8_t imuRead(uint8_t reg, uint8_t *d, uint8_t n) {
    Wire.beginTransmission(IMU_ADDR); Wire.write(reg);
    uint8_t rc = Wire.endTransmission(false); if (rc) return rc;
    Wire.requestFrom(IMU_ADDR, n, (uint8_t)true);
    for (uint8_t i = 0; i < n; i++) {
        if (Wire.available()) d[i] = Wire.read();
        else { uint32_t t = micros(); while ((micros()-t)<1000 && !Wire.available()); if (Wire.available()) d[i]=Wire.read(); else return 5; }
    }
    return 0;
}

// ─────────────────────────────────────────────────────────────
// MOTOR CONTROL
// ─────────────────────────────────────────────────────────────
void setMotorLeft(float output) {
    int s = abs(output);
    if (s < motor_rest_limit) s = 0; else if (s < min_speed) s = min_speed;
    if (s > max_speed) s = max_speed;
    if (output > 0)      { digitalWrite(AIN1, LOW);  digitalWrite(AIN2, HIGH); }
    else if (output < 0) { digitalWrite(AIN1, HIGH); digitalWrite(AIN2, LOW);  }
    else                 { digitalWrite(AIN1, LOW);  digitalWrite(AIN2, LOW);  s = 0; }
    analogWrite(PWMA, s);
}
void setMotorRight(float output) {
    int s = abs(output);
    if (s < motor_rest_limit) s = 0; else if (s < min_speed) s = min_speed;
    if (s > max_speed) s = max_speed;
    if (output > 0)      { digitalWrite(BIN1, HIGH); digitalWrite(BIN2, LOW);  }
    else if (output < 0) { digitalWrite(BIN1, LOW);  digitalWrite(BIN2, HIGH); }
    else                 { digitalWrite(BIN1, LOW);  digitalWrite(BIN2, LOW);  s = 0; }
    analogWrite(PWMB, s);
}

// ─────────────────────────────────────────────────────────────
// SETUP
// ─────────────────────────────────────────────────────────────
void setup() {
    Serial.begin(115200);
    delay(500);
    Serial.println("\n========== BOT ESP — NO-WIFI PID + ESP-NOW ==========");

    // ── Gain Scheduler init ──
    GS_init();

    // ── Pins ──
    pinMode(START_BUTTON, INPUT_PULLUP);
    pinMode(PWMA,OUTPUT); pinMode(AIN1,OUTPUT); pinMode(AIN2,OUTPUT);
    pinMode(PWMB,OUTPUT); pinMode(BIN1,OUTPUT); pinMode(BIN2,OUTPUT);
    analogWrite(PWMA,0); analogWrite(PWMB,0);
    pinMode(ENC_L_A,INPUT_PULLUP); pinMode(ENC_L_B,INPUT_PULLUP);
    pinMode(ENC_R_A,INPUT_PULLUP); pinMode(ENC_R_B,INPUT_PULLUP);
    attachInterrupt(digitalPinToInterrupt(ENC_L_A), isr_encL, RISING);
    attachInterrupt(digitalPinToInterrupt(ENC_R_A), isr_encR, RISING);

    // ── IMU ──
    Serial.println("Initializing I2C...");
    Wire.begin(SDA_PIN, SCL_PIN);
    Wire.setClock(400000UL);
    i2cData[0]=7; i2cData[1]=0x03; i2cData[2]=0x00; i2cData[3]=0x00;
    
    Serial.println("Configuring IMU registers...");
    while (imuWrite(0x19, i2cData, 4, false));
    while (imuWrite(0x6B, 0x01, true));
    while (imuRead(0x75, i2cData, 1));
    delay(100);
    
    Serial.println("Reading initial IMU data...");
    while (imuRead(0x3B, i2cData, 6));
    int16_t ax=(int16_t)((i2cData[0]<<8)|i2cData[1]);
    int16_t ay=(int16_t)((i2cData[2]<<8)|i2cData[3]);
    int16_t az=(int16_t)((i2cData[4]<<8)|i2cData[5]);
    accZ=ax; accY=ay; accX=az;
#ifdef RESTRICT_PITCH
    double roll  = atan2(accY,accZ)*RAD_TO_DEG;
    double pitch = atan(-accX/sqrt(accY*accY+accZ*accZ))*RAD_TO_DEG;
#else
    double roll  = atan(accY/sqrt(accX*accX+accZ*accZ))*RAD_TO_DEG;
    double pitch = atan2(-accX,accZ)*RAD_TO_DEG;
#endif
    kalmanX.setAngle(roll); kalmanY.setAngle(pitch);
    imuTimer = micros();

    Serial.println("IMU initialized.");
    
    // ── ESP-NOW init (STA mode but NOT connected to any router) ──
    Serial.println("Starting WiFi STA...");
    WiFi.mode(WIFI_STA);
    WiFi.disconnect();     // ensure not connected
    
    Serial.println("Setting WiFi channel...");
    // Force channel 1 to match Receiver ESP AP
    esp_wifi_set_promiscuous(true);
    esp_wifi_set_channel(ESPNOW_CHANNEL, WIFI_SECOND_CHAN_NONE);
    esp_wifi_set_promiscuous(false);

    Serial.print("Bot MAC: "); Serial.println(WiFi.macAddress());

    Serial.println("Initializing ESP-NOW...");
    if (esp_now_init() != ESP_OK) {
        Serial.println("ESP-NOW init FAILED! Check MAC/power.");
        while(1);
    }
    Serial.println("Registering ESP-NOW callbacks...");
    esp_now_register_send_cb(onDataSent);
    esp_now_register_recv_cb(onDataReceived);

    // Register the Receiver ESP as peer
    Serial.println("Adding ESP-NOW peer...");
    esp_now_peer_info_t peer = {};
    memcpy(peer.peer_addr, RECEIVER_MAC, 6);
    peer.channel = ESPNOW_CHANNEL;
    peer.encrypt = false;
    if (esp_now_add_peer(&peer) != ESP_OK) {
        Serial.println("Failed to add receiver peer! Check RECEIVER_MAC.");
    } else {
        Serial.println("ESP-NOW peer added. Ready to transmit.");
    }

    // ── FreeRTOS queue for ESP-NOW packets ──
    espnowQueue = xQueueCreate(QUEUE_SIZE, sizeof(TelemetryPacket));

    // ── Core 0 task: sends ESP-NOW packets ──
    xTaskCreatePinnedToCore(
        espnowSenderTask,   // function
        "espnowSender",     // name
        4096,               // stack (small — just sending bytes)
        NULL,               // param
        1,                  // priority
        NULL,               // handle
        0                   // Core 0
    );

    Serial.println("SETUP DONE. Press button to calibrate.");
    Serial.println("======================================================\n");
}

// ─────────────────────────────────────────────────────────────
// MAIN LOOP  (runs on Core 1 — 100% dedicated to PID)
// ─────────────────────────────────────────────────────────────
unsigned long lastTelemetryPush = 0;
GainZone lastZonePrinted = ZONE_STABLE;

void loop() {

    // ── BUTTON ────────────────────────────────────────────────
    if (digitalRead(START_BUTTON) == LOW && (millis() - lastButtonPress) > 500) {
        lastButtonPress = millis();
        if (currentState == DISARMED) {
            currentState = CALIBRATING;
            calibStartTime = millis(); angleSum = 0; calibCount = 0;
        } else if (currentState == READY) {
            currentState = ARMED;
            err_integral = 0; pos_integral = 0;
            encLeft = 0; encRight = 0;
            GS_init();
        } else if (currentState == ARMED) {
            currentState = DISARMED;
            setMotorLeft(0); setMotorRight(0);
        }
    }

    // ── IMU READ ──────────────────────────────────────────────
    while (imuRead(0x3B, i2cData, 14));
    int16_t rax=(int16_t)((i2cData[0]<<8)|i2cData[1]);
    int16_t ray=(int16_t)((i2cData[2]<<8)|i2cData[3]);
    int16_t raz=(int16_t)((i2cData[4]<<8)|i2cData[5]);
    tempRaw    =(int16_t)((i2cData[6]<<8)|i2cData[7]);
    int16_t rgx=(int16_t)((i2cData[8]<<8)|i2cData[9]);
    int16_t rgy=(int16_t)((i2cData[10]<<8)|i2cData[11]);
    int16_t rgz=(int16_t)((i2cData[12]<<8)|i2cData[13]);

    accZ=rax; accY=ray; accX=raz;
    gyroX=-rgz; gyroY=rgy; gyroZ=rgx;

    double dt     = (double)(micros()-imuTimer)/1000000.0;
    imuTimer      = micros();
    double loopHz = 1.0/dt;

#ifdef RESTRICT_PITCH
    double roll  = atan2(accY,accZ)*RAD_TO_DEG;
    double pitch = atan(-accX/sqrt(accY*accY+accZ*accZ))*RAD_TO_DEG;
#else
    double roll  = atan(accY/sqrt(accX*accX+accZ*accZ))*RAD_TO_DEG;
    double pitch = atan2(-accX,accZ)*RAD_TO_DEG;
#endif
    double gXr = gyroX/131.0, gYr = gyroY/131.0;

#ifdef RESTRICT_PITCH
    if ((roll<-90&&kalAngleX>90)||(roll>90&&kalAngleX<-90)) { kalmanX.setAngle(roll); kalAngleX=roll; }
    else kalAngleX = kalmanX.getAngle(roll, gXr, dt);
    if (abs(kalAngleX)>90) gYr=-gYr;
    kalAngleY = kalmanY.getAngle(pitch, gYr, dt);
#else
    if ((pitch<-90&&kalAngleY>90)||(pitch>90&&kalAngleY<-90)) { kalmanY.setAngle(pitch); kalAngleY=pitch; }
    else kalAngleY = kalmanY.getAngle(pitch, gYr, dt);
    if (abs(kalAngleY)>90) gXr=-gXr;
    kalAngleX = kalmanX.getAngle(roll, gXr, dt);
#endif

    // ── CALIBRATION ───────────────────────────────────────────
    if (currentState == CALIBRATING) {
        if (millis()-calibStartTime < 1000) { angleSum+=kalAngleX; calibCount++; }
        else { refAngle=angleSum/calibCount; currentState=READY; Serial.printf("[CALIB] refAngle=%.3f\n",refAngle); }
    }

    // ── GAIN SCHEDULER ────────────────────────────────────────
    bool zoneChanged = GS_update((float)kalAngleX);
    Kp = GS.Kp; Ki = GS.Ki; Kd = GS.Kd;
    if (zoneChanged && GS.zone != lastZonePrinted) {
        lastZonePrinted = GS.zone;
        Serial.printf("[ZONE] -> %s | Kp=%.1f Ki=%.0f Kd=%.2f Clamp=%.0f Freeze=%d\n",
            GS_ZONE_NAMES[GS.zone], GS.Kp, GS.Ki, GS.Kd, GS.integralLimit, GS.freezeIntegral);
    }

    // ── OUTER LOOP @ 50Hz ─────────────────────────────────────
    unsigned long now = millis();
    if (now - lastOuterLoopTime >= outerLoopInterval) {
        float outer_dt = (now - lastOuterLoopTime) / 1000.0f;
        lastOuterLoopTime = now;
        current_position_deg = ((encLeft+encRight)/2.0f)*(360.0f/350.0f);
        pos_error_deg        = 0.0f - current_position_deg;
        encoder_speed_degs   = (current_position_deg - prev_position_deg)/outer_dt;
        prev_position_deg    = current_position_deg;

        if (currentState == ARMED) {
            pos_integral += pos_error_deg * outer_dt;
            if (pos_integral >  pos_integral_limit) pos_integral =  pos_integral_limit;
            if (pos_integral < -pos_integral_limit) pos_integral = -pos_integral_limit;
            float pos_deriv  = (pos_error_deg - prev_pos_err)/outer_dt;
            prev_pos_err     = pos_error_deg;
            pitchOffset      = -(Kp_pos*pos_error_deg + Ki_pos*pos_integral + Kd_pos*pos_deriv);
            if (pitchOffset >  GS_PITCH_OFFSET_MAX) pitchOffset =  GS_PITCH_OFFSET_MAX;
            if (pitchOffset < -GS_PITCH_OFFSET_MAX) pitchOffset = -GS_PITCH_OFFSET_MAX;
        } else {
            pitchOffset = 0; pos_integral = 0;
        }
    }

    // ── INNER LOOP (Angle PID) ────────────────────────────────
    float targetAngle   = refAngle + pitchOffset;
    float raw_err_angle = (float)kalAngleX - targetAngle;
    float err_angle     = (fabsf(raw_err_angle) > angleTolerance) ? raw_err_angle : 0.0f;
    float err_angvel    = (float)gXr;

    if (currentState == ARMED) {
        if (GS.motorsOff || fabsf(raw_err_angle) > GS_CUTOFF_ANGLE) {
            setMotorLeft(0); setMotorRight(0);
            err_integral = 0;
        } else {
            GS_accumulateIntegral(err_integral, err_angle, (float)dt);
            pid_output = (GS.Kp*err_angle) + (GS.Ki*err_integral) + (GS.Kd*err_angvel);
            setMotorLeft(pid_output  + yawOffset);
            setMotorRight(pid_output - yawOffset);
        }
    } else {
        setMotorLeft(0); setMotorRight(0);
    }

    // ── ESP-NOW TELEMETRY @ 20Hz ──────────────────────────────
    if (now - lastTelemetryPush >= 50) {
        lastTelemetryPush = now;
        pushTelemetry(
            now/1000.0f, (float)kalAngleX, (float)gXr,
            targetAngle, err_angle, pid_output,
            current_position_deg, pos_error_deg,
            encoder_speed_degs, (float)loopHz
        );
    }
}
