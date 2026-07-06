# Future Work

While the Cascaded PID implementation currently provides a highly stable platform, there are several avenues for future engineering exploration:

### 1. True State-Space LQR Implementation
The current LQI experiment relies on manually tuned gains. To implement a mathematically pure Linear Quadratic Regulator, the physical constants of the robot must be measured:
- Mass of the chassis and wheels.
- Distance from the wheel axle to the center of mass.
- Moment of inertia of the pendulum body.
- Motor torque constants.
Once these are derived, a MATLAB or Python script can solve the algebraic Riccati equation to generate mathematically optimal `K` matrix gains for the ESP32.

### 2. Upgraded Motor Drivers
Transitioning from standard brushed DC motors with generic H-bridges to Field Oriented Control (FOC) brushless motors or precision Stepper motors. FOC would completely eliminate the static friction (deadband) issues that currently plague the PID balance point, allowing the robot to stand perfectly still without micro-vibrations.

### 3. Trajectory Planning
Currently, remote control is achieved by directly manipulating the outer loop's target position. Integrating a trajectory planner (e.g., generating a smooth velocity curve for the outer loop to follow) would result in much smoother acceleration and braking when being driven by a joystick.

### 4. Automated System Identification
Creating a routine where the ESP32 outputs a sweep of frequencies (chirp signal) to the motors while logging the IMU response, allowing for automated Bode plot generation and mathematical system identification directly on the microcontroller.
