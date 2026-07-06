# Project Timeline

This document outlines the chronological engineering progression of the self-balancing robot.

### 1. Hardware Validation
The first phase involved basic unit testing of the individual hardware components.
- **IMU Test:** Verifying MPU6050 raw values and applying a Kalman filter to extract stable pitch and roll angles.
- **Motor & Encoder Tests:** Characterizing the DC motors to find the minimum PWM signal required to overcome static friction (deadband mapping) and verifying quadrature encoder ticks.

### 2. Initial Balancing
- Implemented a standard single-loop PID controller aiming to hold the robot at a 0-degree tilt.
- **Outcome:** The robot could balance temporarily but suffered from inevitable positional drift over time.

### 3. Adaptive PID Failure
- Attempted to implement a "Gain Scheduler" that dynamically altered the `Kp`, `Ki`, and `Kd` values based on the severity of the robot's tilt (Stable, Disturbed, Recovery zones).
- **Outcome:** This experiment largely failed to provide smooth performance, as sudden jumps in PID gains caused jerky reactions.

### 4. LQR and LQI Experiments
- Explored modern state-space control theory by implementing an experimental Linear Quadratic Integrator (LQI).
- **Outcome:** The implementation relied on tuned "LQR-like" gains rather than a rigorous physical model (mass, inertia, etc.). While educational, it did not outperform traditional cascaded PID for this specific hardware setup.

### 5. Serial Monitor Tuning
- Added basic serial parsing commands (e.g., `p12.5`, `i80`, `d0.6`) to allow tuning the PID gains in real-time without recompiling the firmware.

### 6. Data Logging & Tuning via Website
- Migrated tuning to a WebSockets-based dashboard.
- Utilized ESP32's dual-core architecture (FreeRTOS) to pin the Web Server to Core 0 while strictly dedicating Core 1 to the 1000Hz PID loop.
- **Outcome:** Enabled smooth real-time CSV logging and wireless tuning without crashing the balance loop via WiFi interrupts.

### 7. Cascaded PID & Data Logging
- Implemented a dual-loop (cascaded) control architecture:
  - **Inner Loop (1000Hz):** Fast angle control using the IMU.
  - **Outer Loop (50Hz):** Slower position control using encoders.
- **Outcome:** Successfully eliminated positional drift. The outer loop calculates positional errors and outputs a target lean angle to the inner loop.

### 8. Remote Control via ESP32 Joystick
- Integrated a secondary ESP32 microcontroller with a physical joystick to act as a remote control.
- Wireless communication (likely ESP-NOW or WebSocket) allows overriding the outer loop's target position to drive the robot forward, backward, and steer differentially.
