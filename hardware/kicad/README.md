# KiCad PCB Design — Self-Balancing Robot

Branch: `pcb-design`  
Firmware reference: `firmware/01_Basic_Serial_Tuning/Self_PID_Dir_deg_serial/Self_PID_Dir_deg_serial.ino` (with encoder pins from `Bot_ESP.ino` / `Auto_PID_Tuning.ino`)

## Board Summary (`70 mm × 70 mm` Compact THT Socketed Design)

* **Board Size & Mounting:** Compact `70 mm × 70 mm` outline (`Edge.Cuts`) with four `M3` mounting holes (`H1`–`H4`, `4 mm` inset).
* **100% Through-Hole (THT) Socketed Modules:** All pluggable modules use 2.54 mm female headers (`PinSocket`) so they can be unplugged cleanly:
  * **30-Pin DOIT ESP32 DevKit V1 (`J_ESP_L` & `J_ESP_R`):** Dual 15-pin 2.54 mm female headers (~28 mm DevKit width, 22.86 mm / 0.9 in standard row spacing); USB port remains on the DevKit.
  * **GY-521 MPU6050 (`J_MPU6050`):** 8-pin 2.54 mm female header centered at `(35 mm, 35 mm)` on the `70 × 70 mm` board (`SDA = GPIO22`, `SCL = GPIO23`).
  * **TB6612FNG Driver (`J_DRV_L` & `J_DRV_R`):** Dual 8-pin 2.54 mm female headers (`PWMA = 14`, `AIN1 = 26`, `AIN2 = 25`, `PWMB = 13`, `BIN1 = 27`, `BIN2 = 19`, `STBY = 15`).
* **6-Pin RMC Motor Headers (`J_MOTA_6P_RMC` & `J_MOTB_6P_RMC`):** Two 6-pin 2.54 mm headers (`1: M+`, `2: M-`, `3: GND`, `4: +3V3`, `5: ENC_A`, `6: ENC_B`), with encoders mapped to `ENC_L_A = 18`, `ENC_L_B = 5`, `ENC_R_A = 33`, `ENC_R_B = 32`.
* **3S Li-Ion Power & Protection:** `J_BAT` -> `Q1` (`IRF9540N` TO-220 reverse-polarity protection) -> `VMOTOR` + `U4` (`LM7805` TO-220 5V linear regulator with `C_IN_7805` and `C_OUT_7805`) -> ESP32 `VIN/+5V`.
* **Start Button (`GPIO4`) Hardware Debounce:** `J_BTN` + `R_PU` (`10 kΩ`) + `R_SER` (`1 kΩ`) + `C_DEB` (`100 nF` THT disc cap to `GND`) to suppress button bounce/jerk noise.
