# KiCad PCB Design — Self-Balancing Robot

Branch: `pcb-design`  
Firmware reference: `firmware/01_Basic_Serial_Tuning/Self_PID_Dir_deg_serial/Self_PID_Dir_deg_serial.ino` (with encoder pins from `Bot_ESP.ino` / `Auto_PID_Tuning.ino`)

## Circuit Diagram

```text
+---------------------------------------------------------------------------------------------------+
|                           3S Li-Ion Battery Pack (11.1V nom / 12.6V max)                        |
+---------------------------------------------------------------------------------------------------+
      | (+) VBAT_RAW                                                                 | (-) GND
      v                                                                              v
+---------------------------------------------------------------------------------------------------+
| Reverse-Polarity Protection: Q1 (IRF9540N P-MOSFET, TO-220)                                       |
|  - Drain (Pin 2) -> VBAT_PROT / VMOTOR rail                                                       |
|  - Source (Pin 3) -> VBAT_RAW (from 3S +)                                                         |
|  - Gate (Pin 1)   -> Pull-down / Zener clamp to GND (turns ON when VBAT_RAW is positive)        |
+---------------------------------------------------------------------------------------------------+
      | VMOTOR                                                                         | GND
      +----------------------------------------+                                     |
      |                                        |                                     |
      v                                        v                                     |
+-----------------------------+      +----------------------------------------------+      |
| TB6612FNG Motor Driver      |      | 5V Linear Regulator Stage (U4: LM7805 TO-220)|      |
| (Socketed on Female Header) |      |  - C_IN_7805 (100uF) across VMOTOR <-> GND   |      |
|  - VM      <-- VMOTOR       |      |  - Pin 1 (IN)  <-- VMOTOR                    |      |
|  - VCC     <-- +3V3 (or +5V)|      |  - Pin 2 (GND) <-- GND                       |      |
|  - GND     <-- GND          |      |  - Pin 3 (OUT) --> +5V rail                  |      |
|  - PWMA    <-- GPIO14       |      |  - C_OUT_7805 (100uF) across +5V <-> GND    |      |
|  - AIN1    <-- GPIO26       |      +----------------------------------------------+      |
|  - AIN2    <-- GPIO25       |                 | +5V                                       |
|  - PWMB    <-- GPIO13       |                 v                                           |
|  - BIN1    <-- GPIO27       |      +----------------------------------------------+      |
|  - BIN2    <-- GPIO19       |      | DOIT ESP32 DevKit V1 (30-Pin, ~28mm wide)  |      |
|  - STBY    <-- GPIO15       |      | (Socketed on 2x15 Female Headers)            |      |
|  - AO1     --> Motor A Pin 1|      |  - VIN / 5V <-- +5V (from LM7805)            |      |
|  - AO2     --> Motor A Pin 2|      |  - 3V3      --> +3V3 logic rail (IMU/Enc/PU) |      |
|  - BO1     --> Motor B Pin 1|      |  - GND      <-- GND                          |<-----+
|  - BO2     --> Motor B Pin 2|      |  - GPIO22   <--> SDA (MPU6050)               |
+-----------------------------+      |  - GPIO23   ---> SCL (MPU6050)               |
                                     |  - GPIO14   ---> PWMA (TB6612FNG)            |
+-----------------------------+      |  - GPIO26   ---> AIN1 (TB6612FNG)            |
| GY-521 MPU6050 (PCB Center) |      |  - GPIO25   ---> AIN2 (TB6612FNG)            |
| (Socketed on Female Header) |      |  - GPIO13   ---> PWMB (TB6612FNG)            |
|  - VCC     <-- +3V3         |      |  - GPIO27   ---> BIN1 (TB6612FNG)            |
|  - GND     <-- GND          |      |  - GPIO19   ---> BIN2 (TB6612FNG)            |
|  - SDA     <--> GPIO22      |      |  - GPIO15   ---> STBY (TB6612FNG)            |
|  - SCL     <-- GPIO23       |      |  - GPIO18   <--- ENC_L_A (Motor A Pin 5)     |
+-----------------------------+      |  - GPIO5    <--- ENC_L_B (Motor A Pin 6)     |
                                     |  - GPIO33   <--- ENC_R_A (Motor B Pin 5)     |
+-----------------------------+      |  - GPIO32   <--- ENC_R_B (Motor B Pin 6)     |
| Start Button + RC Debounce  |      |  - GPIO4    <--- Debounced Button Node       |
|  - R_PU (10k) : +3V3 -> RAW |      +----------------------------------------------+
|  - J_BTN Pin 1: RAW -> GND  |
|    (when button pressed)    |
|  - R_SER (1k) : RAW -> GPIO4|
|  - C_DEB(100nF): GPIO4->GND |
+-----------------------------+

+---------------------------------------------------------------------------------------------------+
| 6-Pin RMC Motor Connectors (Left = Motor A, Right = Motor B)                                      |
|  Pin 1: M+ (AO1 / BO1)   | Pin 2: M- (AO2 / BO2)   | Pin 3: GND                                    |
|  Pin 4: +3V3 (Enc VCC)   | Pin 5: ENC_A (18 / 33)  | Pin 6: ENC_B (5 / 32)                           |
+---------------------------------------------------------------------------------------------------+
```

## Board Summary (`70 mm × 70 mm` Compact THT Socketed Design)

* **Board Size & Mounting:** Compact `70 mm × 70 mm` outline (`Edge.Cuts`) with four `M3` mounting holes (`H1`–`H4`, `4 mm` inset).
* **100% Through-Hole (THT) Socketed Modules:** All pluggable modules use 2.54 mm female headers (`PinSocket`) so they can be unplugged cleanly:
  * **30-Pin DOIT ESP32 DevKit V1 (`J_ESP_L` & `J_ESP_R`):** Dual 15-pin 2.54 mm female headers (~28 mm DevKit width, 22.86 mm / 0.9 in standard row spacing); USB port remains on the DevKit.
  * **GY-521 MPU6050 (`J_MPU6050`):** 8-pin 2.54 mm female header centered at `(35 mm, 35 mm)` on the `70 × 70 mm` board (`SDA = GPIO22`, `SCL = GPIO23`).
  * **TB6612FNG Driver (`J_DRV_L` & `J_DRV_R`):** Dual 8-pin 2.54 mm female headers (`PWMA = 14`, `AIN1 = 26`, `AIN2 = 25`, `PWMB = 13`, `BIN1 = 27`, `BIN2 = 19`, `STBY = 15`).
* **6-Pin RMC Motor Headers (`J_MOTA_6P_RMC` & `J_MOTB_6P_RMC`):** Two 6-pin 2.54 mm headers (`1: M+`, `2: M-`, `3: GND`, `4: +3V3`, `5: ENC_A`, `6: ENC_B`), with encoders mapped to `ENC_L_A = 18`, `ENC_L_B = 5`, `ENC_R_A = 33`, `ENC_R_B = 32`.
* **3S Li-Ion Power & Protection:** `J_BAT` -> `Q1` (`IRF9540N` TO-220 reverse-polarity protection) -> `VMOTOR` + `U4` (`LM7805` TO-220 5V linear regulator with `C_IN_7805` and `C_OUT_7805`) -> ESP32 `VIN/+5V`.
* **Start Button (`GPIO4`) Hardware Debounce:** `J_BTN` + `R_PU` (`10 kΩ`) + `R_SER` (`1 kΩ`) + `C_DEB` (`100 nF` THT disc cap to `GND`) to suppress button bounce/jerk noise.

## Repository Structure

```text
hardware/
└── kicad/
    ├── README.md
    ├── .gitignore
    ├── self_balancing_bot.kicad_pro
    ├── self_balancing_bot.kicad_sch
    └── self_balancing_bot.kicad_pcb
```
