# KiCad PCB Design — Self-Balancing Robot

Branch: `pcb-design`  
Firmware reference: `firmware/01_Basic_Serial_Tuning/Self_PID_Dir_deg_serial/Self_PID_Dir_deg_serial.ino` (with encoder pins from `Bot_ESP.ino` / `Auto_PID_Tuning.ino`)

## Default Design Proposals (Ready for Your Edits)

* **Assembly & Sockets:** 100% Through-Hole (THT) sockets via 2.54 mm female headers so the ESP32-WROOM-32 DevKit (assumed 30-pin, 0.9 in / 22.86 mm row spacing), GY-521 MPU6050, and TB6612FNG can be unplugged cleanly.
* **Encoder Pins (from repo):** `ENC_L_A = GPIO18`, `ENC_L_B = GPIO5`, `ENC_R_A = GPIO33`, `ENC_R_B = GPIO32`.
* **TB6612FNG Standby (`STBY`):** Suggested `GPIO15` (set high in firmware to enable driver; can be changed to any free non-strapping GPIO).
* **6-Pin RMC Motor Connectors (Left & Right):** Suggested pinout `1: M+`, `2: M-`, `3: GND`, `4: ENC_VCC (+3.3V default)`, `5: ENC_A`, `6: ENC_B`.
* **Power & Reverse Protection:** 3S Li-Ion input (`11.1 V` nom / `12.6 V` max) via 2-pin connector -> P-channel MOSFET (`IRF9540N` TO-220 + gate Zener/pull-up) reverse-polarity protection -> `VMOTOR` + TO-220 `7805` 5V linear regulator (with heatsink allowance and input/output capacitors) -> ESP32 `VIN/5V`.
* **Start Button (`GPIO4`):** External button header with hardware RC debounce (`10 kΩ` pull-up, `1 kΩ` series resistor, `100 nF` capacitor to `GND`).
* **Board Dimensions & Mounting:** Suggested `90 mm × 90 mm` square PCB with four `M3` mounting holes (`5 mm` inset from corners) and GY-521 MPU6050 positioned at the `(45 mm, 45 mm)` center.

## Repository Structure

```text
hardware/
└── kicad/
    ├── README.md
    ├── .gitignore
    ├── self_balancing_bot.kicad_pro
    ├── self_balancing_bot.kicad_sch
    ├── self_balancing_bot.kicad_pcb
    ├── footprints/
    ├── symbols/
    └── manufacturing/
```
