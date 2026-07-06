# Hardware Architecture

The physical build of the robot heavily dictates the software constraints. Here is the hardware stack used in this project.

## Core Components
- **Microcontroller:** ESP32 NodeMCU
  - *Why ESP32?* It provides a dual-core processor, which is absolutely critical for running a 1000Hz PID loop on Core 1 without being interrupted by the WiFi/WebSocket server running on Core 0.
- **Inertial Measurement Unit (IMU):** MPU6050
  - Communicates via I2C at 400kHz.
  - The raw accelerometer and gyroscope data is fused using a Kalman filter to prevent gyro drift and filter out accelerometer noise caused by motor vibrations.
- **Motor Driver:** Dual H-Bridge (e.g., L298N or TB6612FNG)
  - Driven via High-Frequency PWM from the ESP32.
- **Motors:** 12V DC Gear Motors with Quadrature Encoders
  - The encoders provide high-resolution feedback on wheel rotation, which is fed into the outer PID loop to prevent the robot from drifting across the room.
- **Power Supply:** 3S LiPo Battery (11.1V)
  - Provides sufficient torque to overcome the pendulum's inertia during a fall.

## Physical Considerations
- **Center of Mass (CoM):** The battery and heavier components should be mounted as high as possible. A higher CoM increases the pendulum's moment of inertia, effectively making the robot fall *slower*, giving the PID loop more time to react.
- **Static Friction:** The gearboxes in the motors require a minimum PWM signal just to overcome friction. A software deadband compensator is required to instantly bypass this friction threshold when balancing.
