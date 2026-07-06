# Software Architecture

The codebase is built entirely in C++ using the Arduino IDE framework for the ESP32.

## Key Libraries
- `Wire.h` - For I2C communication with the MPU6050 at 400kHz.
- `WiFi.h` & `WebServer.h` - Native ESP32 libraries for creating the local tuning network.
- `WebSocketsServer.h` - (By Markus Sattler) Used for real-time 20Hz telemetry streaming and instantaneous PID tuning.
- `Kalman.h` - A lightweight implementation of a 1D Kalman filter for sensor fusion.

## RTOS Core Pinning
A major milestone in this project was the transition to FreeRTOS dual-core tasking.
- **Core 0 (The Network Core):** Handles all WiFi background tasks, the HTTP Web Server, and the WebSocket connections. It also calculates the slow Outer PID Loop (Position) at 50Hz.
- **Core 1 (The Control Core):** Strictly dedicated to the 1000Hz (1ms) Inner PID Loop (Angle). It reads the MPU6050 via I2C, computes the Kalman filter, calculates the PID output, and updates the motor PWM.

*Failure to isolate the network stack to Core 0 results in the WiFi library triggering hardware interrupts on Core 1, delaying the PID loop by 10-50ms, causing the robot to violently fall.*

## Derivative Filter
Standard PID derivative calculation (`Kd * (error - last_error)`) results in violent motor stuttering due to sensor noise. The software implements a low-pass filter on the derivative term, heavily inspired by CNC stepper motor libraries, to provide smooth dampening without the "kick."
