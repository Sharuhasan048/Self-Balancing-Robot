# KiCad PCB Design — Self-Balancing Robot

Branch: `pcb-design`  
Firmware reference: `firmware/01_Basic_Serial_Tuning/Self_PID_Dir_deg_serial`

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

## Firmware Signal & Hardware Notes

* Uses `Wire` (`SDA` / `SCL`) for IMU attitude sensing combined with `Kalman` filtering (`Kalman.cpp` / `Kalman.h`) and PID serial tuning.
* Drives two motors with directional/PWM control and serial command telemetry/tuning.
* Exact GPIO pin assignments, motor driver IC/module, IMU part/module, and power rail specs are being confirmed before populating final schematic symbols and PCB footprints.

## Planned Next Steps

1. Confirm MCU, IMU module/sensor, motor driver, exact GPIO pins, and power architecture.
2. Add schematic symbols and footprints (or module headers) into `symbols/` and `footprints/`.
3. Wire `self_balancing_bot.kicad_sch` and run ERC.
4. Define board outline, mounting holes, and `MotorPower` vs. logic routing in `self_balancing_bot.kicad_pcb`, run DRC, and export manufacturing files into `manufacturing/`.
