# KiCad PCB Design — Self-Balancing Robot

Branch: `pcb-design`  
Firmware reference: `firmware/01_Basic_Serial_Tuning/Self_PID_Dir_deg_serial/Self_PID_Dir_deg_serial.ino`

## Repository Structure

```text
hardware/
└── kicad/
    ├── README.md
    ├── self_balancing_bot.kicad_pro
    ├── self_balancing_bot.kicad_sch
    ├── self_balancing_bot.kicad_pcb
    ├── footprints/
    ├── symbols/
    └── manufacturing/
```

## Extracted Pinout (`Self_PID_Dir_deg_serial.ino`)

* **MCU / Bus:** ESP32 (`Wire.begin(SDA_PIN, SCL_PIN)` at `400000UL` / 400 kHz I2C, `Serial.begin(115200)`)
* **IMU:** MPU6050 with DLPF + Kalman filter (`Kalman.cpp` / `Kalman.h`)
* **Start Button:** `START_BUTTON = 4` (`INPUT_PULLUP`)
* **Motor A (Left):** `PWMA = 14`, `AIN1 = 26`, `AIN2 = 25`
* **Motor B (Right):** `PWMB = 13`, `BIN1 = 27`, `BIN2 = 19`
* **I2C:** `SDA_PIN = 22`, `SCL_PIN = 23`

## Planned Next Steps

1. Confirm MCU carrier/module vs. bare IC, motor driver IC/module (and whether `STBY` is tied high or on a GPIO), and IMU breakout vs. bare IC.
2. Confirm power rail architecture (battery voltage, buck converter, reverse-polarity protection, connectors) and physical PCB outline/mounting holes.
3. Add schematic symbols and footprints into `symbols/` and `footprints/`, wire `self_balancing_bot.kicad_sch`, and run ERC.
4. Route `self_balancing_bot.kicad_pcb` with `MotorPower` net class separation, run DRC, and export manufacturing files into `manufacturing/`.
