> [!NOTE]
> **Repository Context:** This folder contains an archived, failed iteration of the simulation model, kept solely for educational debugging purposes. For the working MATLAB simulations, see simulation/Matlab_PID_Tuning/. For the actual C++ firmware, see irmware/ at the root of the repository.

# Failed Simulation Archive (bal_bot2)

This directory contains `bal_bot2.slx`, an early iteration of the Simscape Multibody model that ultimately failed to balance in simulation. 

This document summarizes the diagnostic investigation into *why* it failed, serving as a learning resource for debugging complex physical simulations.

## 1. Overview
Despite tuning the PID controller and adjusting control loop sign conventions, `bal_bot2` failed to achieve a stable upright position. The primary symptom was that the robot's tilt angle (as measured by the Kalman filter) hovered artificially around ~1.6 degrees, while the PID controller saturated at its maximum output (±12) without affecting the angle.

## 2. System Architecture
The simulated physics and control loops were structured as follows:

- **Control Loop:** A Kalman Filter (implemented via a MATLAB Function block) processes angle and angular velocity data. The filtered angle is compared against a setpoint (0 degrees). A PID Controller computes the required corrective torque.
- **Actuation:** The PID output feeds into two `Simulink-PS Converter` blocks. The right wheel receives direct PID output; the left wheel receives output passed through a Gain block. Both joints are configured with `InputTorque` (actuation) and `ComputedMotion`.
- **Physics & Sensing:** A Planar Joint connects the World to the chassis (`base_link`). A Transform Sensor measures angular velocity. Ground interaction relies on Infinite Plane and Spatial Contact Force blocks.

## 3. Diagnostic Investigation (Eliminated Causes)
To determine why maximum control effort was not translating into physical correction, several tests were run:

1. **Control Loop Sign Convention:** 
   - *Test:* Simulated all permutations of Sum block inputs (+- vs -+) and Gain blocks (1 vs -1).
   - *Conclusion:* The physical model failed to respond regardless of the direction the torque was applied. The control sign is not the root cause.
2. **Planar Joint Constraints:**
   - *Test:* Inspected Planar Joint actuation modes (Px, Py, Rz).
   - *Conclusion:* Set to `ComputedMotion`, meaning the chassis was free to rotate and was not artificially locked by the joint solver.

## 4. Root Cause Hypotheses (Unconfirmed)
Since the PID generated maximum torque but the angle remained static, the failure lies in the physics engine setup or sensor coordinate frames. The following remain the most likely theories to explore when debugging similar models:

### A. Sensor Axis Mismatch (Highly Likely)
The Transform Sensor was configured to output `OmegaY` (angular velocity around Y), while the Kalman filter expected a `YZX` rotation sequence to extract pitch. However, the Planar Joint's primary tilt axis was `Rz` (rotation around Z). 
*Impact:* If the Rigid Transform block does not align these coordinate frames, the sensor measures near-zero rotation on Y while the robot falls around Z. The 1.6-degree reading may be mathematical noise rather than the true chassis tilt.

### B. Contact Force / Friction Failure
The model relies on Spatial Contact Force blocks for traction against the Infinite Plane. 
*Impact:* If contact stiffness or friction coefficients are too low, the wheels slip continuously or spin in the air. Without physical traction, the wheel torque cannot exert a reactionary moment to pull the chassis upright.

### C. Actuation Unit Discrepancy
The `Simulink-PS Converter` blocks were configured with `Unit = 1` (dimensionless).
*Impact:* If Simscape expects torque in standard physical units (e.g., N·m) and the PID output (±12) is being interpreted on a mismatched scale, the physical force applied may simply be orders of magnitude too small to overcome gravity and chassis inertia.

