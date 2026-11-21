# KiCad PCB Design — Self-Balancing Robot

Branch: `pcb-design`  
Firmware reference: `firmware/01_Basic_Serial_Tuning/Self_PID_Dir_deg_serial/Self_PID_Dir_deg_serial.ino`

## Hardware Architecture Summary (Confirmed)

* **Assembly Style:** 100% Through-Hole (THT). All pluggable modules (ESP32-WROOM-32 DevKit, GY-521 MPU6050 breakout, TB6612FNG breakout) mount into **2.54 mm female headers** soldered to the PCB so modules can be unplugged/replaced easily.
* **MCU:** ESP32-WROOM-32 DevKit (USB stays on the DevKit; powered via `+5V` on `VIN`/5V pin).
* **IMU Placement:** GY-521 MPU6050 breakout placed at the **geometric center** of the rectangular/square PCB (`SDA = GPIO22`, `SCL = GPIO23`).
* **Motor Driver:** TB6612FNG module (`PWMA = 14`, `AIN1 = 26`, `AIN2 = 25`, `PWMB = 13`, `BIN1 = 27`, `BIN2 = 19`, `STBY` controlled via dedicated ESP32 GPIO).
* **Motors & Encoders:** Two **6-pin RMC connectors** (Motor A Left & Motor B Right) carrying motor power (`M+`, `M-`) and quadrature encoder signals (`GND`, `VCC`, `CH_A`, `CH_B`).
* **Power Supply:** 3S Li-Ion battery input (`11.1 V` nominal / `12.6 V` max) -> **Reverse-polarity protection** -> direct `VMOTOR` rail for TB6612FNG + **5V linear voltage regulator** (TO-220, e.g., LM7805 with heatsink space and decoupling capacitors) feeding ESP32 `5V/VIN`.
* **Start Button:** External push button connected to `GPIO4` (`START_BUTTON`) with **hardware RC debounce / jerk-canceling filter** (pull-up resistor, series resistor, and filter capacitor to `GND`).

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
