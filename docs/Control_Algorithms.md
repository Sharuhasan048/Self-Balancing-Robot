# Control Algorithms

This project serves as an exploration of several different control methodologies applied to the classic inverted pendulum problem.

## 1. Single-Loop PID
The simplest form of balancing.
- **Input:** Pitch angle (from MPU6050 + Kalman Filter)
- **Output:** Motor PWM
- **Result:** Capable of keeping the robot upright, but entirely ignorant of its physical location. The robot will slowly drift across the floor due to sensor biases or uneven weight distribution.

## 2. Cascaded PID (Dual-Loop)
To solve the drift problem, a second PID loop is wrapped around the first one.
- **Inner Loop (Angle Control - 1000Hz):** Uses the IMU to keep the robot balanced at a specific `targetAngle`.
- **Outer Loop (Position Control - 50Hz):** Reads the wheel encoders to determine if the robot has drifted from its starting position. If it drifts forward, it outputs a small negative lean angle to the inner loop, causing the robot to reverse.
- **Result:** Highly stable position-holding and the ability to accept precise remote-control movement commands.

## 3. Adaptive PID (Gain Scheduling) - *Experimental*
An attempt to handle large disturbances differently than micro-vibrations at the balance point.
- The software measures the current tilt and places the robot in a zone (`STABLE`, `DISTURBED`, `RECOVERY`).
- It applies different `Kp`, `Ki`, and `Kd` values depending on the zone.
- **Result:** *Failed / Discarded.* The discontinuous jumps in motor commands caused jerkiness. 

## 4. Linear Quadratic Integrator (LQI) - *Experimental*
An attempt at modern state-space control.
- Utilizes LQR-like gains (`k_angle`, `k_angvel`, `k_integral`).
- **Limitations:** True LQR requires a highly accurate mathematical model of the robot (measuring the exact center of mass, wheel inertia, pendulum inertia, and motor torque constants) to solve the algebraic Riccati equation. Because this project used manually tuned LQR gains rather than a rigorous physical model, the performance did not exceed the Cascaded PID.

## Integral Anti-Windup Strategies
Across all algorithms, "integral windup" was a persistent threat. We implemented several strategies:
1. **Clamping:** Hard limits on the maximum value the integral can accumulate.
2. **Conditional Integration:** Only allowing the integral to accumulate when the robot is near the balance point (e.g., `< 15 degrees`). If the robot falls, the integral is wiped to prevent violent spinning upon recovery.
