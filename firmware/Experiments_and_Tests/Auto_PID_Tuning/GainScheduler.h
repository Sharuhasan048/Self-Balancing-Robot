/**
 * ==========================================================================
 *  GainScheduler.h
 *  Data-driven Adaptive PID Gain Scheduler for Self-Balancing Robot
 * --------------------------------------------------------------------------
 *  HOW IT WORKS:
 *    Based on 3 datasets (Ki=230/260/300, Kp=18, Kd=0.55) logged from the
 *    real robot, the angle magnitude at every moment maps to one of three
 *    operating zones. Each zone uses a different PID gain set tuned to
 *    what the robot actually needs at that level of tilt.
 *
 *  ZONES (Inner Angle Loop):
 *    ┌──────────────┬───────────┬──────┬─────┬──────┬──────────────────────┐
 *    │ Zone         │ |angle|   │  Kp  │  Ki │  Kd  │ Ki Integral Clamp    │
 *    ├──────────────┼───────────┼──────┼─────┼──────┼──────────────────────┤
 *    │ STABLE       │ 0 – 1.5°  │ 18.0 │ 230 │ 0.55 │ 200  (tight)         │
 *    │ DISTURBED    │ 1.5 – 4°  │ 20.0 │ 260 │ 0.65 │ 400  (medium)        │
 *    │ RECOVERY     │ 4 – 8°    │ 22.0 │  50 │ 0.80 │  50  (freeze integ)  │
 *    │ FALL-CUTOFF  │ > 60°     │  —   │  —  │  —   │ motors off           │
 *    └──────────────┴───────────┴──────┴─────┴──────┴──────────────────────┘
 *
 *  HYSTERESIS: Each zone boundary has ±0.3° hysteresis to prevent
 *  chattering/rapid gain switching at the thresholds.
 *
 *  INTEGRAL BEHAVIOUR PER ZONE:
 *    STABLE:    Integral accumulates normally within tight clamp.
 *    DISTURBED: Integral accumulates with wider clamp.
 *    RECOVERY:  Integral is FROZEN (held at current value, not accumulated).
 *               This prevents wind-up during large swings.
 *
 *  OUTER LOOP (Position PID):
 *    Runs at 50Hz. Outputs pitchOffset → added to refAngle.
 *    Gains tuned from observed position drift in data.
 *    ┌──────────┬────────────────────────────────────────────────┐
 *    │ Kp_pos   │ 0.020  (lean 2° per 100° of shaft drift)       │
 *    │ Ki_pos   │ 0.008  (slow, steady-state drift correction)    │
 *    │ Kd_pos   │ 0.012  (damp oscillation from position speed)   │
 *    │ Clamp    │ ±15.0  (pos_integral_limit, tighter than drift) │
 *    │ Output   │ ±5.0°  pitchOffset clamp (firmware safety)      │
 *    └──────────┴────────────────────────────────────────────────┘
 *
 *  USAGE:
 *    1. Include this header in your .ino file.
 *    2. Call GS_init() once in setup().
 *    3. Call GS_update(currentAngle) every inner loop tick.
 *    4. Use GS.Kp, GS.Ki, GS.Kd, GS.integralLimit in your PID.
 * ==========================================================================
 */

#pragma once

// ── Zone thresholds (degrees) ─────────────────────────────────────────────
#define GS_THRESH_STABLE     1.5f   // |angle| below this = STABLE zone
#define GS_THRESH_DISTURBED  4.0f   // |angle| below this = DISTURBED zone
#define GS_THRESH_RECOVERY   8.0f   // |angle| below this = RECOVERY zone
#define GS_THRESH_HYSTERESIS 0.3f   // ± hysteresis around each boundary

// ── Safety cutoff ────────────────────────────────────────────────────────
#define GS_CUTOFF_ANGLE     60.0f   // Motors off if |angle| exceeds this

// ── Zone IDs ─────────────────────────────────────────────────────────────
enum GainZone : uint8_t {
    ZONE_STABLE    = 0,
    ZONE_DISTURBED = 1,
    ZONE_RECOVERY  = 2,
    ZONE_CUTOFF    = 3
};

// ── Per-zone gain table ───────────────────────────────────────────────────
struct GainSet {
    float Kp;
    float Ki;
    float Kd;
    float integralLimit;  // Anti-windup clamp for this zone
    bool  freezeIntegral; // If true: hold integral, do NOT accumulate
};

// Gain table indexed by GainZone
static const GainSet GS_TABLE[4] = {
    // Zone STABLE
    { .Kp = 18.0f, .Ki = 230.0f, .Kd = 0.55f, .integralLimit = 200.0f, .freezeIntegral = false },
    // Zone DISTURBED
    { .Kp = 20.0f, .Ki = 260.0f, .Kd = 0.65f, .integralLimit = 400.0f, .freezeIntegral = false },
    // Zone RECOVERY
    { .Kp = 22.0f, .Ki =  50.0f, .Kd = 0.80f, .integralLimit =  50.0f, .freezeIntegral = true  },
    // Zone CUTOFF (all zero – motors off handled in main loop)
    { .Kp =  0.0f, .Ki =   0.0f, .Kd = 0.00f, .integralLimit =   0.0f, .freezeIntegral = true  }
};

// Zone display names (for telemetry / Serial debug)
static const char* const GS_ZONE_NAMES[4] = {
    "STABLE", "DISTURBED", "RECOVERY", "CUTOFF"
};

// ─────────────────────────────────────────────────────────────────────────
// Outer Position Loop Recommended Gains
// (All zero in original firmware – these are the data-derived suggestions)
// ─────────────────────────────────────────────────────────────────────────
#define GS_KP_POS          0.020f   // Lean 2° per 100° of shaft error
#define GS_KI_POS          0.008f   // Slow drift correction
#define GS_KD_POS          0.012f   // Damp position speed oscillation
#define GS_POS_INT_LIMIT   15.0f    // Integral clamp for outer loop
#define GS_PITCH_OFFSET_MAX 5.0f    // Max lean angle outer loop commands (deg)

// ── Scheduler State (read-only after GS_update) ───────────────────────────
struct GainSchedulerState {
    float     Kp;
    float     Ki;
    float     Kd;
    float     integralLimit;
    bool      freezeIntegral;
    GainZone  zone;
    GainZone  prevZone;
    uint32_t  zoneEnterTime_ms;  // millis() when we entered current zone
    uint32_t  zoneSamples;       // how many samples in current zone
    bool      motorsOff;         // safety cutoff active
};

static GainSchedulerState GS;  // Global scheduler state

// ─────────────────────────────────────────────────────────────────────────
/**
 * GS_init()
 * Call once in setup(). Initialises scheduler to STABLE zone.
 */
inline void GS_init() {
    GS.zone            = ZONE_STABLE;
    GS.prevZone        = ZONE_STABLE;
    GS.zoneEnterTime_ms = millis();
    GS.zoneSamples     = 0;
    GS.motorsOff       = false;
    // Load STABLE gains as default
    GS.Kp             = GS_TABLE[ZONE_STABLE].Kp;
    GS.Ki             = GS_TABLE[ZONE_STABLE].Ki;
    GS.Kd             = GS_TABLE[ZONE_STABLE].Kd;
    GS.integralLimit  = GS_TABLE[ZONE_STABLE].integralLimit;
    GS.freezeIntegral = GS_TABLE[ZONE_STABLE].freezeIntegral;
}

// ─────────────────────────────────────────────────────────────────────────
/**
 * GS_resolveZone(absAngle)
 * Determines the target zone from angle magnitude, applying hysteresis
 * relative to the CURRENT zone to prevent chattering at thresholds.
 */
inline GainZone GS_resolveZone(float absAngle) {
    // Safety cutoff always wins
    if (absAngle >= GS_CUTOFF_ANGLE) return ZONE_CUTOFF;

    // Hysteresis: to leave a zone upward, angle must exceed threshold + hyst.
    //             to leave a zone downward, angle must be below threshold - hyst.
    float hyst = GS_THRESH_HYSTERESIS;

    switch (GS.zone) {
        case ZONE_STABLE:
            if      (absAngle > GS_THRESH_DISTURBED + hyst) return ZONE_RECOVERY;
            else if (absAngle > GS_THRESH_STABLE    + hyst) return ZONE_DISTURBED;
            else                                             return ZONE_STABLE;

        case ZONE_DISTURBED:
            if      (absAngle > GS_THRESH_DISTURBED + hyst)  return ZONE_RECOVERY;
            else if (absAngle < GS_THRESH_STABLE    - hyst)  return ZONE_STABLE;
            else                                              return ZONE_DISTURBED;

        case ZONE_RECOVERY:
            if      (absAngle > GS_THRESH_RECOVERY  + hyst)  return ZONE_CUTOFF;  // extreme
            else if (absAngle < GS_THRESH_DISTURBED - hyst)  return ZONE_DISTURBED;
            else                                              return ZONE_RECOVERY;

        case ZONE_CUTOFF:
            // Only leave CUTOFF when angle drops well below recovery threshold
            if (absAngle < GS_THRESH_RECOVERY - hyst) return ZONE_RECOVERY;
            else                                       return ZONE_CUTOFF;

        default: return ZONE_STABLE;
    }
}

// ─────────────────────────────────────────────────────────────────────────
/**
 * GS_update(currentAngle)
 * Call every inner loop tick (before PID computation).
 * Updates GS.Kp, GS.Ki, GS.Kd, GS.integralLimit, GS.freezeIntegral.
 *
 * Returns: true if zone changed this tick (useful for resetting integrator)
 */
inline bool GS_update(float currentAngle) {
    float absAngle  = fabsf(currentAngle);
    GainZone newZone = GS_resolveZone(absAngle);

    GS.zoneSamples++;
    GS.motorsOff = (newZone == ZONE_CUTOFF);

    bool zoneChanged = (newZone != GS.zone);
    if (zoneChanged) {
        GS.prevZone         = GS.zone;
        GS.zone             = newZone;
        GS.zoneEnterTime_ms = millis();
        GS.zoneSamples      = 0;
    }

    // Load gains from table
    const GainSet& g    = GS_TABLE[GS.zone];
    GS.Kp             = g.Kp;
    GS.Ki             = g.Ki;
    GS.Kd             = g.Kd;
    GS.integralLimit  = g.integralLimit;
    GS.freezeIntegral = g.freezeIntegral;

    return zoneChanged;
}

// ─────────────────────────────────────────────────────────────────────────
/**
 * GS_accumulateIntegral(integral, error, dt)
 * Call INSTEAD of your manual  integral += err * dt  line.
 * Handles:
 *   - freeze in RECOVERY/CUTOFF zone
 *   - zone-specific clamp
 *   - firmware-compatible angle gate (|err| < 15 before accumulating)
 */
inline void GS_accumulateIntegral(float &integral, float error, float dt) {
    if (GS.freezeIntegral) {
        // In RECOVERY: keep integral at its current value (don't grow)
        // Clamp it to current zone's tighter limit to prevent carry-over
        if (integral >  GS.integralLimit) integral =  GS.integralLimit;
        if (integral < -GS.integralLimit) integral = -GS.integralLimit;
        return;
    }

    // Firmware gate: only accumulate if error is small enough
    if (fabsf(error) < 15.0f) {
        integral += error * dt;
        // Zone-aware anti-windup clamp
        if (integral >  GS.integralLimit) integral =  GS.integralLimit;
        if (integral < -GS.integralLimit) integral = -GS.integralLimit;
    } else {
        // Robot tipping: reset integral (same as original firmware)
        integral = 0.0f;
    }
}
