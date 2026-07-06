# Lessons Learned

Building a self-balancing robot provides extreme insight into control theory and embedded systems. Here are the primary engineering lessons learned throughout this project.

### 1. The "Derivative Kick" Problem
During early PID tuning, applying a large derivative (`Kd`) term caused the motors to violently stutter. 
- **Cause:** When the target angle changes abruptly, the derivative of the error spikes infinitely, sending a massive shock to the motors.
- **Solution:** Implemented a "filtered derivative on measurement." By taking the derivative of the actual sensor reading (Kalman angle) rather than the error, and applying a first-order low-pass filter, the control loop smoothed out significantly.

### 2. Dual-Core RTOS is Mandatory for WiFi Control
Initially, attempting to serve a web page while balancing caused the robot to fall over.
- **Cause:** The ESP32's WiFi stack generates hardware interrupts that pause the main loop. Since balancing requires a strict, high-speed loop (e.g., 1000Hz), missing even 5 milliseconds causes catastrophic failure.
- **Solution:** Used FreeRTOS to pin the `wifiTask` and WebSockets to **Core 0**, while dedicating the critical IMU reading and PID computation to **Core 1**.

### 3. Static Friction (Deadband) Kills Balance
When the robot is almost perfectly balanced, the PID controller outputs very small PWM values (e.g., 5-15). 
- **Cause:** Due to static friction in the gearboxes, the motors won't actually turn at low PWM values. They just whine. By the time the robot leans far enough for the PID output to overcome static friction, the robot is already falling fast, resulting in a violent oscillation back and forth.
- **Solution:** Implemented a software deadband compensator (`min_speed`). If the PID output is non-zero, it is instantly boosted above the friction threshold.

### 4. Integral Windup and Drift
In a single-loop PID controller, any slight imbalance (wiring weight, sensor drift) causes the integral term to wind up indefinitely, eventually driving the robot across the room.
- **Solution:** Transitioned to a **Cascaded PID** system using wheel encoders. The outer position loop detects the drift and commands the robot to lean slightly in the opposite direction to hold its ground.

### 5. Adaptive PID (Gain Scheduling) Limitations
Attempting to dynamically switch PID gains based on tilt zones resulted in unpredictable behavior. 
- **Lesson:** Control loops rely heavily on mathematical continuity. Abruptly changing the `Kp` multiplier halfway through a fall creates a discontinuous jump in motor output, often worsening the instability. Standard Cascaded PID proved far more reliable.
