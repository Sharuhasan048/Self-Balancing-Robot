/**
 * ==========================================================================
 *  GainScheduler_PID.ino
 *  Adaptive Cascaded PID Balancing Robot - With Gain Scheduling
 * --------------------------------------------------------------------------
 *  Based on: Balancing_Cascad_PID.ino (Self-Balancing-Robot\Balancing_Cascad_PID)
 *  Changes:
 *    1. Inner loop gains are now ADAPTIVE via GainScheduler.h
 *    2. Outer position loop gains are ENABLED and tuned from data
 *    3. Position loop integral clamp raised to 15 (was 10)
 *    4. Telemetry extended with current zone name for debugging
 *
 *  DO NOT EDIT the original Balancing_Cascad_PID.ino.
 *  This is a SEPARATE experiment file in Self-Balancing-Robot\GainScheduler_PID\
 *
 *  Hardware: ESP32 (same pin config as original)
 *  Libraries needed: Wire, Kalman.h (copy from original), Telemetry.h (copy)
 * ==========================================================================
 *
 *  HOW GAIN SCHEDULING WORKS IN THIS FILE:
 *  ─────────────────────────────────────────────────────────────────────────
 *  Every inner loop tick, GS_update(kalAngleX) is called.
 *  It looks at the robot's tilt angle and picks a zone:
 *
 *  |angle| < 1.5°  → STABLE    → Kp=18, Ki=230, Kd=0.55, Clamp=200
 *  1.5° – 4°       → DISTURBED → Kp=20, Ki=260, Kd=0.65, Clamp=400
 *  4°  – 8°        → RECOVERY  → Kp=22, Ki=50,  Kd=0.80, Clamp=50  (integral frozen)
 *  > 60°           → CUTOFF    → motors off
 *
 *  ±0.3° hysteresis prevents chattering at zone boundaries.
 *
 *  HOW OUTER LOOP IS TUNED:
 *  ─────────────────────────────────────────────────────────────────────────
 *  From data analysis of 3 datasets (Ki=230/260/300):
 *   - All 3 showed continuous position drift because Kp_pos/Ki_pos/Kd_pos=0
 *   - Position error reached 400–700° of motor shaft with no correction
 *   - The robot falls from integral windup caused by drift, not from angle
 *
 *  Suggested outer loop gains:
 *    Kp_pos = 0.020  →  2° lean per 100° shaft error (gentle correction)
 *    Ki_pos = 0.008  →  slow steady-state drift cancellation
 *    Kd_pos = 0.012  →  damps position oscillation when returning to origin
 *    pos_integral_limit = 15.0  (raised from 10 for better correction range)
 * ==========================================================================
 */

#include <Wire.h>
#include "Kalman.h"
#include "Telemetry.h"
#include "GainScheduler.h"   // << NEW: adaptive gain scheduler

// ─────────────────────────────────────────────────────────────────────────
// HARDWARE PIN DEFINITIONS (unchanged from original)
// ─────────────────────────────────────────────────────────────────────────
const int START_BUTTON = 4;

// Motor A (Left)
const int PWMA = 14;
const int AIN1 = 26;
const int AIN2 = 25;

// Motor B (Right)
const int PWMB = 13;
const int BIN1 = 27;
const int BIN2 = 19;

// Encoders
const int ENC_L_A = 18;
const int ENC_L_B = 5;
const int ENC_R_A = 33;
const int ENC_R_B = 32;

// I2C
const int SDA_PIN = 22;
const int SCL_PIN = 23;

// ─────────────────────────────────────────────────────────────────────────
// SYSTEM STATE
// ─────────────────────────────────────────────────────────────────────────
volatile SystemState currentState = DISARMED;
unsigned long lastButtonPress = 0;

unsigned long calibStartTime = 0;
float angleSum   = 0;
int   calibCount = 0;

// ─────────────────────────────────────────────────────────────────────────
// ENCODERS
// ─────────────────────────────────────────────────────────────────────────
volatile long encLeft  = 0;
volatile long encRight = 0;

float current_position_deg  = 0.0f;
float pos_error_deg         = 0.0f;
float encoder_speed_degs    = 0.0f;
float prev_position_deg     = 0.0f;

volatile TelemetryData tData = {0};
portMUX_TYPE tDataMux = portMUX_INITIALIZER_UNLOCKED;

void IRAM_ATTR isr_encL() { if (digitalRead(ENC_L_B)) encLeft++;  else encLeft--;  }
void IRAM_ATTR isr_encR() { if (digitalRead(ENC_R_B)) encRight--; else encRight++; }

// ─────────────────────────────────────────────────────────────────────────
// IMU / KALMAN
// ─────────────────────────────────────────────────────────────────────────
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

// ─────────────────────────────────────────────────────────────────────────
// PID VARIABLES
// ─────────────────────────────────────────────────────────────────────────
float refAngle = 0.0f;  // Set by auto-calibration

// --- Inner Loop (Angle) ---
// NOTE: Kp, Ki, Kd are now read from GS.Kp, GS.Ki, GS.Kd at runtime.
//       The float variables below are kept for telemetry compatibility.
volatile float Kp = 18.0f;   // Updated each tick from scheduler
volatile float Ki = 230.0f;  // Updated each tick from scheduler
volatile float Kd = 0.55f;   // Updated each tick from scheduler

float err_integral = 0.0f;
// integral_limit is now per-zone: GS.integralLimit

float pid_output = 0.0f;

// --- Outer Loop (Position) ---
// DATA-DERIVED GAINS: (all were 0.0 in original – now enabled)
// ─────────────────────────────────────────────────────────────────────────
//  Kp_pos = 0.020  : 100° shaft error → 2.0° lean command
//  Ki_pos = 0.008  : steady drift removed slowly (integral_limit now 15)
//  Kd_pos = 0.012  : damps return oscillation
// ─────────────────────────────────────────────────────────────────────────
volatile float Kp_pos = GS_KP_POS;   // 0.020
volatile float Ki_pos = GS_KI_POS;   // 0.008
volatile float Kd_pos = GS_KD_POS;   // 0.012

float pos_integral     = 0.0f;
float prev_pos_err     = 0.0f;
const float pos_integral_limit = GS_POS_INT_LIMIT;  // 15.0 (raised from 10)

// Motor constraints (unchanged)
int max_speed       = 180;
int min_speed       = 20;
int motor_rest_limit = 40;

// ─────────────────────────────────────────────────────────────────────────
// MOVEMENT VARIABLES
// ─────────────────────────────────────────────────────────────────────────
float pitchOffset    = 0.0f;
float yawOffset      = 0.0f;
float angleTolerance = 0.0f;

unsigned long lastOuterLoopTime = 0;
const unsigned long outerLoopInterval = 20;  // 50Hz

// Debug: print zone changes to Serial
GainZone lastPrintedZone = ZONE_STABLE;

// ─────────────────────────────────────────────────────────────────────────
// SETUP
// ─────────────────────────────────────────────────────────────────────────
void setup() {
    Serial.begin(115200);
    delay(1000);

    // Init gain scheduler to defaults (STABLE zone)
    GS_init();
    Serial.println("[GainScheduler] Initialized. Default zone: STABLE");
    Serial.printf("[GainScheduler] Kp=%.1f Ki=%.0f Kd=%.2f | Clamp=%.0f\n",
                  GS.Kp, GS.Ki, GS.Kd, GS.integralLimit);
    Serial.printf("[OuterLoop] Kp_pos=%.3f Ki_pos=%.3f Kd_pos=%.3f | Clamp=%.1f\n",
                  Kp_pos, Ki_pos, Kd_pos, pos_integral_limit);

    pinMode(START_BUTTON, INPUT_PULLUP);

    pinMode(PWMA, OUTPUT); pinMode(AIN1, OUTPUT); pinMode(AIN2, OUTPUT);
    pinMode(PWMB, OUTPUT); pinMode(BIN1, OUTPUT); pinMode(BIN2, OUTPUT);
    analogWrite(PWMA, 0); analogWrite(PWMB, 0);

    pinMode(ENC_L_A, INPUT_PULLUP); pinMode(ENC_L_B, INPUT_PULLUP);
    pinMode(ENC_R_A, INPUT_PULLUP); pinMode(ENC_R_B, INPUT_PULLUP);
    attachInterrupt(digitalPinToInterrupt(ENC_L_A), isr_encL, RISING);
    attachInterrupt(digitalPinToInterrupt(ENC_R_A), isr_encR, RISING);

    Wire.begin(SDA_PIN, SCL_PIN);
    Wire.setClock(400000UL);

    i2cData[0] = 7;     // Sample Rate = 1kHz
    i2cData[1] = 0x03;  // DLPF ~42Hz bandwidth
    i2cData[2] = 0x00;  // Gyro ±250 deg/s
    i2cData[3] = 0x00;  // Accel ±2g
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
    double pitch = atan(-accX / sqrt(accY*accY + accZ*accZ)) * RAD_TO_DEG;
#else
    double roll  = atan(accY / sqrt(accX*accX + accZ*accZ)) * RAD_TO_DEG;
    double pitch = atan2(-accX, accZ) * RAD_TO_DEG;
#endif

    kalmanX.setAngle(roll);
    kalmanY.setAngle(pitch);
    gyroXangle = roll;  gyroYangle = pitch;
    compAngleX = roll;  compAngleY = pitch;
    timer = micros();

    initTelemetry();
    Serial.println("[SETUP] Complete. Connect via Web Dashboard.");
}

// ─────────────────────────────────────────────────────────────────────────
// MAIN LOOP
// ─────────────────────────────────────────────────────────────────────────
void loop() {

    // ── BUTTON STATE MACHINE (unchanged) ──────────────────────────────────
    if (digitalRead(START_BUTTON) == LOW && (millis() - lastButtonPress) > 500) {
        lastButtonPress = millis();
        if (currentState == DISARMED) {
            currentState = CALIBRATING;
            calibStartTime = millis();
            angleSum = 0; calibCount = 0;
            sendState();
        } else if (currentState == READY) {
            currentState = ARMED;
            err_integral = 0;
            pos_integral = 0;
            encLeft = 0; encRight = 0;
            GS_init();  // << Reset scheduler when arming
            sendState();
        } else if (currentState == ARMED) {
            currentState = DISARMED;
            setMotorLeft(0); setMotorRight(0);
            sendState();
        }
    }

    // ── IMU READ & KALMAN FILTER (unchanged) ──────────────────────────────
    while (imuRead(0x3B, i2cData, 14));

    int16_t raw_accX  = (int16_t)((i2cData[0]  << 8) | i2cData[1]);
    int16_t raw_accY  = (int16_t)((i2cData[2]  << 8) | i2cData[3]);
    int16_t raw_accZ  = (int16_t)((i2cData[4]  << 8) | i2cData[5]);
    tempRaw           = (int16_t)((i2cData[6]  << 8) | i2cData[7]);
    int16_t raw_gyroX = (int16_t)((i2cData[8]  << 8) | i2cData[9]);
    int16_t raw_gyroY = (int16_t)((i2cData[10] << 8) | i2cData[11]);
    int16_t raw_gyroZ = (int16_t)((i2cData[12] << 8) | i2cData[13]);

    accZ = raw_accX; accY = raw_accY; accX = raw_accZ;
    gyroX = -raw_gyroZ; gyroY = raw_gyroY; gyroZ = raw_gyroX;

    double dt     = (double)(micros() - timer) / 1000000.0;
    timer         = micros();
    double loopHz = 1.0 / dt;

#ifdef RESTRICT_PITCH
    double roll  = atan2(accY, accZ) * RAD_TO_DEG;
    double pitch = atan(-accX / sqrt(accY*accY + accZ*accZ)) * RAD_TO_DEG;
#else
    double roll  = atan(accY / sqrt(accX*accX + accZ*accZ)) * RAD_TO_DEG;
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

    // ── AUTO CALIBRATION (unchanged) ──────────────────────────────────────
    if (currentState == CALIBRATING) {
        if (millis() - calibStartTime < 1000) {
            angleSum += kalAngleX;
            calibCount++;
        } else {
            refAngle = angleSum / calibCount;
            currentState = READY;
            sendState();
            Serial.printf("[CALIB] refAngle = %.3f deg\n", refAngle);
        }
    }

    // ── GAIN SCHEDULER UPDATE ─────────────────────────────────────────────
    //   Call before any PID computation so GS.Kp/Ki/Kd are current.
    bool zoneChanged = GS_update((float)kalAngleX);

    // Sync readable variables for telemetry
    Kp = GS.Kp;
    Ki = GS.Ki;
    Kd = GS.Kd;

    // Print zone transitions to Serial for debugging
    if (zoneChanged || GS.zone != lastPrintedZone) {
        lastPrintedZone = GS.zone;
        Serial.printf("[ZONE] -> %-10s | Kp=%.1f Ki=%.0f Kd=%.2f Clamp=%.0f Freeze=%d\n",
                      GS_ZONE_NAMES[GS.zone],
                      GS.Kp, GS.Ki, GS.Kd,
                      GS.integralLimit, GS.freezeIntegral);
    }

    // ── OUTER LOOP: Position PID @ 50Hz ───────────────────────────────────
    unsigned long currentMillis = millis();

    if (currentMillis - lastOuterLoopTime >= outerLoopInterval) {
        float outer_dt = (currentMillis - lastOuterLoopTime) / 1000.0f;
        lastOuterLoopTime = currentMillis;

        // Convert encoder ticks to motor shaft degrees
        // 350 ticks/rev, 1x RISING resolution
        current_position_deg = ((encLeft + encRight) / 2.0f) * (360.0f / 350.0f);
        pos_error_deg        = 0.0f - current_position_deg;  // target = stay put

        // Position speed for derivative term
        encoder_speed_degs = (current_position_deg - prev_position_deg) / outer_dt;
        prev_position_deg  = current_position_deg;

        if (currentState == ARMED) {
            // ── Position Integral with anti-windup ──────────────────────
            pos_integral += pos_error_deg * outer_dt;
            if (pos_integral >  pos_integral_limit) pos_integral =  pos_integral_limit;
            if (pos_integral < -pos_integral_limit) pos_integral = -pos_integral_limit;

            float pos_deriv = (pos_error_deg - prev_pos_err) / outer_dt;
            prev_pos_err = pos_error_deg;

            // ── Outer loop output = target lean angle ────────────────────
            // Negative sign: positive position error (drifted forward)
            // means robot should lean backward (negative pitchOffset)
            pitchOffset = -(Kp_pos * pos_error_deg
                          + Ki_pos * pos_integral
                          + Kd_pos * pos_deriv);

            // Safety clamp: outer loop cannot command more than ±5° lean
            if (pitchOffset >  GS_PITCH_OFFSET_MAX) pitchOffset =  GS_PITCH_OFFSET_MAX;
            if (pitchOffset < -GS_PITCH_OFFSET_MAX) pitchOffset = -GS_PITCH_OFFSET_MAX;

        } else {
            pitchOffset  = 0.0f;
            pos_integral = 0.0f;
        }
    }

    // ── INNER LOOP: Angle PID @ ~500Hz ────────────────────────────────────
    float targetAngle   = refAngle + pitchOffset;
    float raw_err_angle = (float)kalAngleX - targetAngle;
    float err_angle     = 0.0f;
    float err_angvel    = (float)gyroXrate;

    if (fabsf(raw_err_angle) > angleTolerance) {
        err_angle = raw_err_angle;
    }

    if (currentState == ARMED) {
        // ── Safety cutoff (motors off if tipped too far) ─────────────────
        if (GS.motorsOff || fabsf(raw_err_angle) > GS_CUTOFF_ANGLE) {
            setMotorLeft(0); setMotorRight(0);

            // Reset integrator on hard cutoff
            err_integral = 0.0f;
            return;
        }

        // ── Gain-scheduled integral accumulation ─────────────────────────
        //   Replaces the original  err_integral += err_angle * dt  block.
        GS_accumulateIntegral(err_integral, err_angle, (float)dt);

        // ── PID output with scheduled gains ──────────────────────────────
        pid_output = (GS.Kp * err_angle)
                   + (GS.Ki * err_integral)
                   + (GS.Kd * err_angvel);

        setMotorLeft(pid_output  + yawOffset);
        setMotorRight(pid_output - yawOffset);

    } else {
        setMotorLeft(0); setMotorRight(0);
    }

    // ── TELEMETRY ─────────────────────────────────────────────────────────
    handleTelemetry(
        currentMillis / 1000.0f,
        (float)kalAngleX,
        (float)gyroXrate,
        targetAngle,
        err_angle,
        pid_output,
        current_position_deg,
        pos_error_deg,
        encoder_speed_degs,
        loopHz
    );
}

// ─────────────────────────────────────────────────────────────────────────
// MOTOR CONTROL (unchanged from original)
// ─────────────────────────────────────────────────────────────────────────
void setMotorLeft(float output) {
    int speed = abs(output);
    if (speed < motor_rest_limit) speed = 0;
    else if (speed < min_speed)   speed = min_speed;
    if (speed > max_speed)        speed = max_speed;

    if (output > 0)      { digitalWrite(AIN1, LOW);  digitalWrite(AIN2, HIGH); }
    else if (output < 0) { digitalWrite(AIN1, HIGH); digitalWrite(AIN2, LOW);  }
    else                 { digitalWrite(AIN1, LOW);  digitalWrite(AIN2, LOW); speed = 0; }
    analogWrite(PWMA, speed);
}

void setMotorRight(float output) {
    int speed = abs(output);
    if (speed < motor_rest_limit) speed = 0;
    else if (speed < min_speed)   speed = min_speed;
    if (speed > max_speed)        speed = max_speed;

    if (output > 0)      { digitalWrite(BIN1, HIGH); digitalWrite(BIN2, LOW);  }
    else if (output < 0) { digitalWrite(BIN1, LOW);  digitalWrite(BIN2, HIGH); }
    else                 { digitalWrite(BIN1, LOW);  digitalWrite(BIN2, LOW); speed = 0; }
    analogWrite(PWMB, speed);
}

// ─────────────────────────────────────────────────────────────────────────
// IMU HELPERS (unchanged from original)
// ─────────────────────────────────────────────────────────────────────────
bool imuWrite(uint8_t reg, uint8_t data, bool sendStop) { return imuWrite(reg, &data, 1, sendStop); }
bool imuWrite(uint8_t reg, uint8_t *data, uint8_t len, bool sendStop) {
    Wire.beginTransmission(IMUAddress);
    Wire.write(reg); Wire.write(data, len);
    uint8_t rcode = Wire.endTransmission(sendStop);
    if (rcode) { Serial.print(F("imuWrite failed: ")); Serial.println(rcode); return true; }
    return false;
}
uint8_t imuRead(uint8_t reg, uint8_t *data, uint8_t nbytes) {
    uint32_t timeOutTimer;
    Wire.beginTransmission(IMUAddress); Wire.write(reg);
    uint8_t rcode = Wire.endTransmission(false);
    if (rcode) return rcode;
    Wire.requestFrom(IMUAddress, nbytes, (uint8_t)true);
    for (uint8_t i = 0; i < nbytes; i++) {
        if (Wire.available()) data[i] = Wire.read();
        else {
            timeOutTimer = micros();
            while (((micros() - timeOutTimer) < 1000) && !Wire.available());
            if (Wire.available()) data[i] = Wire.read(); else return 5;
        }
    }
    return 0;
}
