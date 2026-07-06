# GainScheduler_PID – Quick Reference
**Folder:** `Self-Balancing-Robot\GainScheduler_PID\`

---

## Files

| File | Purpose |
|------|---------|
| `GainScheduler_PID.ino` | Main sketch — drop-in replacement for original |
| `GainScheduler.h` | All zone logic, gain tables, anti-windup functions |
| `Kalman.h / .cpp` | Copied from original (unchanged) |
| `Telemetry.h` | Copied from original (unchanged) |

---

## Inner Loop Gain Schedule

| Zone | Condition | Kp | Ki | Kd | Int. Clamp | Integral |
|------|-----------|----|----|-----|------------|---------|
| STABLE | `\|angle\|` < 1.5° | 18 | 230 | 0.55 | 200 | Accumulates |
| DISTURBED | 1.5° – 4° | 20 | 260 | 0.65 | 400 | Accumulates |
| RECOVERY | 4° – 8° | 22 | 50 | 0.80 | 50 | **FROZEN** |
| CUTOFF | > 60° | — | — | — | — | Motors OFF |

Zone boundaries have **±0.3° hysteresis** to stop chattering.

---

## Outer Position Loop (NOW ENABLED)

Was **all zeros** in original firmware. These values come from data analysis:

```cpp
Kp_pos = 0.020   // 100° shaft error → 2° lean command
Ki_pos = 0.008   // slow steady-state drift removal
Kd_pos = 0.012   // damps oscillation when returning to home
pos_integral_limit = 15.0  // raised from 10 (more correction range)
```

**Why these values?**
- All 3 logged datasets showed position drifting 400–700° of motor shaft with zero correction.
- At 350 ticks/rev (1x RISING) and 100:1 gear ratio, each encoder degree = 1/100 wheel revolution.
- `Kp_pos × 100° error = 2°` lean is gentle — enough to pull back without destabilising.
- Ki is small so integral saturates slowly and doesn't cause windup.
- pitchOffset is still **hard clamped to ±5°** for safety.

---

## Why Ki=300 Falls (Root Cause from Data)

1. Outer loop was disabled → position drifts freely.
2. Inner integral fights the drift by building up (winding up).
3. At Ki=300, the integral × Ki component fills the PWM budget faster.
4. At ~62s, integral hits the 800-cap → `pid_output` saturates → robot can't correct angle → falls.

**Ki=230 is the winner** — lowest ITAE, survives longest.

---

## How to Upload

1. Open `GainScheduler_PID.ino` in Arduino IDE.
2. All 5 files are in same folder — Arduino IDE auto-includes them.
3. Board: `ESP32 Dev Module` | Upload Speed: 921600 | Partition: Default 4MB.
4. Monitor Serial at 115200 to see zone transitions printed live.

---

## Tuning the Outer Loop Further

If the robot still drifts, increase `Kp_pos` in small steps:
- `0.020` → `0.025` → `0.030` (max recommended without oscillation)

If the robot oscillates left-right while standing:
- Reduce `Kd_pos`: `0.012` → `0.008`
- Or reduce `Kp_pos` first

If steady-state drift is still slow:
- Increase `Ki_pos`: `0.008` → `0.012` → `0.015`
- Do NOT exceed `0.02` — outer integral windup risk.

---

## Serial Output During Run

```
[GainScheduler] Initialized. Default zone: STABLE
[GainScheduler] Kp=18.0 Ki=230 Kd=0.55 | Clamp=200
[OuterLoop] Kp_pos=0.020 Ki_pos=0.008 Kd_pos=0.012 | Clamp=15.0
[CALIB] refAngle = 0.143 deg
[ZONE] -> STABLE     | Kp=18.0 Ki=230 Kd=0.55 Clamp=200 Freeze=0
[ZONE] -> DISTURBED  | Kp=20.0 Ki=260 Kd=0.65 Clamp=400 Freeze=0   ← push!
[ZONE] -> RECOVERY   | Kp=22.0 Ki=50  Kd=0.80 Clamp=50  Freeze=1   ← big push
[ZONE] -> DISTURBED  | Kp=20.0 Ki=260 Kd=0.65 Clamp=400 Freeze=0   ← recovering
[ZONE] -> STABLE     | Kp=18.0 Ki=230 Kd=0.55 Clamp=200 Freeze=0   ← stable again
```
