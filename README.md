# Self-Balancing Robot: Complete Digital Twin & Hardware Implementation

A comprehensive engineering project covering the design, physics simulation, and real-world C++ firmware development of a two-wheeled self-balancing robot using an ESP32.

> **🧭 Repository Navigation & Context**
> This repository is a "Monorepo" containing the entire lifecycle of the robot. 
> - **Just want it to balance?** Head to `firmware/01_Basic_Serial_Tuning` or `03_Final_Cascaded_PID` to upload the working code.
> - **Want to study the control math?** Check out the simulations in the `simulation/` directory.
> - **Curious about failures?** Look into `simulation/Archive_Failed_Models` or `firmware/Experiments_and_Tests` to see what didn't work and why.

## 🚀 Getting Started (Firmware)

If you just built your robot and want to get it standing, jump straight into the `firmware/` directory and use the numbered codes in order:

1. **`01_Basic_Serial_Tuning`**: Start here. The perfect, lightweight code to tune your PID via the Arduino Serial Monitor without complex WiFi setups.
2. **`02_Web_DataLogger`**: Serve a webpage from the ESP32, tune your PID wirelessly, and download live CSV data to analyze your robot's reaction.
3. **`03_Final_Cascaded_PID`**: The ultimate working code. Uses wheel encoders in an outer loop to stop the robot from drifting across the room.
4. **`04_Dual_ESP_Joystick`**: Add a second ESP32 to drive the balanced robot around like an RC car.

*Note: All experimental code (like adaptive PID and unit tests) has been moved to the `firmware/Experiments_and_Tests/` folder so it doesn't clutter your workspace.*

## 💻 Simulation & Digital Twins (MATLAB/Simulink)

We don't just guess PID values. This repository contains high-fidelity Simscape Multibody simulations to test controllers before risking physical hardware.

- **`simulation/Matlab_PID_Tuning/self_bal_3_pid_gui.slx`**: The highly stable, working simulation model used to derive the baseline PID values.
- **`simulation/Matlab_PID_Tuning/self_bal_4.slx`**: An advanced, in-development digital twin incorporating real-world CAD geometries, sensor noise, and a discrete Kalman filter. *(Note: Still undergoing optimization).*
- **`simulation/Archive_Failed_Models/bal_bot2.slx`**: An early simulation iteration that failed due to sensor axis mismatches. Kept for educational and debugging reference.

## 🖨️ Hardware & CAD

- **`hardware/URDF_Model`**: Contains the direct exports from Fusion 360, preserving the exact mechanical geometry, mass distribution, and inertial matrices used by the Simscape simulation and future ROS integrations.

## 📚 Engineering Lessons Learned
For those interested in control theory, this project evolved through several stages:
1. **Initial Balancing:** Classic single-loop PID.
2. **Adaptive PID Experiment:** Attempted a Gain Scheduler based on tilt zones (failed to provide smoothness).
3. **LQR / LQI Experiments:** Attempted modern state-space control (LQI), highlighting the difficulty of tuning LQR without a perfect mathematical model.
4. **Cascaded PID:** The final breakthrough, adding an outer 50Hz position loop to eliminate drift.

Check the `docs/` folder for detailed write-ups on the specific engineering challenges (like Derivative Kick and Integral Windup)!

## License
MIT License.