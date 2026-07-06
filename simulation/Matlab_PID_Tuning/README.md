> [!NOTE]
> **Repository Context:** This folder contains the successful and actively-in-development MATLAB simulations. For the actual C++ firmware that runs on the ESP32, see the `firmware/` directory at the root of the repository. For failed simulation models (and why they failed), see `simulation/Archive_Failed_Models/`.

# MATLAB Multibody PID Simulation

This directory contains the Simscape multibody simulations of the self-balancing robot. Using simulation allows us to safely tune the PID control loops (`Kp`, `Ki`, `Kd`) before testing them on physical hardware, avoiding violent crashes and broken 3D printed parts.

## 1. Stable Baseline Model (`self_bal_3_pid_gui.slx`)
This represents the working, highly stable iteration of the physical model used to derive our initial PID values.

**How to Run:**
To run this simulation correctly, you **must** load the workspace variables first, otherwise Simulink will throw an error about missing parameters.
1. Open MATLAB.
2. Double-click and load the `self_bot_3_workspace_1.mat` file to populate the physical parameters (mass, inertia, etc.).
3. Open `self_bal_3_pid_gui.slx` in Simulink and click **Run**.
4. You can now tweak the PID values in the GUI to see how they affect stability in real-time.

---

## 2. High-Fidelity Digital Twin (`self_bal_4.slx`)
![Status](https://img.shields.io/badge/Status-Ongoing_Development-orange)

A comprehensive, advanced Simscape Multibody simulation. This model bridges the gap between CAD design and real-world hardware deployment by serving as a highly realistic digital twin for testing advanced non-linear control algorithms. *Note: This model is not fully optimized yet, but you are welcome to experiment with it.*

### 🚀 Advanced Features
Unlike standard simplified inverted pendulum models, this simulation incorporates real-world physics and sensor imperfections:
*   **Direct CAD-to-Simulation Pipeline:** Uses direct URDF imports from Fusion 360 (located in `hardware/`), preserving exact mechanical geometry and mass distribution without manual approximation.
*   **Realistic Contact Mechanics:** Replaces idealized joint constraints with **Spatial Contact Force** blocks. It utilizes smooth spring-damper and stick-slip friction models to simulate genuine wheel slip and drop impacts.
*   **Sensor Noise & State Estimation:** Injects band-limited white noise to simulate cheap, real-world IMU sensors. The noisy data is fused using a custom **Discrete Kalman Filter** (built in a MATLAB Function block) to recover accurate pitch.
*   **Advanced Cascaded Control Architecture:**
    *   **Inner Loop:** An aggressive PID controller for pitch angle stabilization.
    *   **Outer Loop:** A Position PID controller equipped with back-calculation anti-windup to eliminate steady-state drift.
    *   **Velocity Brake:** A direct velocity feedback path acting as an active dynamic brake, preventing exponential runaway loops caused by saturation.

### ⚙️ Robot Specifications (Modeled)
*   **Total Mass:** 500g
*   **Chassis:** 440g (115mm height, 120mm width, 50mm length)
*   **Wheels (x2):** 30g each, 70mm diameter, 25mm width
*   **Sensor Placement:** IMU located 110mm from the ground.

### 🛠️ Future Roadmap (Ongoing Work)
- [ ] **LQR Implementation:** Transitioning from PID to a Linear Quadratic Regulator for optimal state-space control.
- [ ] **Yaw Control (Steering):** Implementing differential drive mechanics to follow complex 3D trajectories.
- [ ] **Disturbance Rejection:** Adding environmental disturbance models (wind gusts, uneven terrain) to test robustness.
