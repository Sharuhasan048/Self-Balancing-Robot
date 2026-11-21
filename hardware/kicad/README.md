# KiCad PCB Design — Self-Balancing Robot (Hardware Architecture Spec)

Branch: `pcb-design`  
Primary Firmware: `firmware/01_Basic_Serial_Tuning/Self_PID_Dir_deg_serial/Self_PID_Dir_deg_serial.ino`  
Encoder-Enabled Firmware Reference: `firmware/03_Final_Cascaded_PID/Cascaded_PID_encoder_1/Cascaded_PID_encoder_1.ino`

## Confirmed Hardware Architecture

1. **Socketed Through-Hole Module Design:**
   - All major modules mount into **2.54 mm female pin headers** so every module can be unplugged/replaced easily.
   - **ESP32 DevKit WROOM** (30-pin or 38-pin DevKit — pin count & row spacing to be confirmed).
   - **MPU6050 GY-521 Module** (8-pin 2.54 mm header) placed at the **geometric center** of the PCB to minimize rotational/centripetal acceleration artifacts.
   - **TB6612FNG Dual Motor Driver Breakout Module** (2x 8-pin 2.54 mm female headers).

2. **Power & Protection (3S Li-ion ~11.1V Nominal / 12.6V Full):**
   - **Battery Input:** 3S Li-ion connector (`VBAT`).
   - **Reverse-Polarity Protection:** Series Schottky diode (e.g., 1N5822 / SS34) or P-MOSFET reverse-polarity protection on `VBAT` -> `VMOTOR`.
   - **5V Regulation (`+5V` for ESP32 VIN):** **LM7805** (TO-220, through-hole, with heatsink recommendation due to `(12.6V - 5V)` thermal drop) + input/output electrolytic & 100 nF ceramic decoupling capacitors.
   - **3.3V Logic (`+3V3`):** Sourced from ESP32 onboard 3.3V regulator (or dedicated AMS1117-3.3 module) for MPU6050 `VCC`, TB6612FNG `VCC`, and encoder pull-ups.

3. **Motor & Encoder Interfaces (6-Pin RMC Connectors):**
   - **Motor A (Left) — 6-pin 2.54 mm RMC Connector (`J_MOTA`):**
     1. `M_A1` (Motor Power +)
     2. `M_A2` (Motor Power -)
     3. `ENC_VCC` (`+3V3` / `+5V` encoder power)
     4. `ENC_GND` (`GND`)
     5. `ENCA_LEFT` -> **GPIO 18** (from `Cascaded_PID_encoder_1.ino`)
     6. `ENCB_LEFT` -> **GPIO 5** (from `Cascaded_PID_encoder_1.ino`)
   - **Motor B (Right) — 6-pin 2.54 mm RMC Connector (`J_MOTB`):**
     1. `M_B1` (Motor Power +)
     2. `M_B2` (Motor Power -)
     3. `ENC_VCC` (`+3V3` / `+5V` encoder power)
     4. `ENC_GND` (`GND`)
     5. `ENCA_RIGHT` -> **GPIO 33** (from `Cascaded_PID_encoder_1.ino`)
     6. `ENCB_RIGHT` -> **GPIO 32** (from `Cascaded_PID_encoder_1.ino`)

4. **Control & Debounced Start Button:**
   - **Start Button (`GPIO 4`):** External 2-pin RMC/header push button with hardware RC + Schmitt/diode debounce filter (`10 kΩ` pull-up to `+3V3`, `1 kΩ` series resistor, `100 nF` capacitor to `GND`) to eliminate switch bounce and motor EMI false triggers.
   - **TB6612FNG Control Pins:**
     - `PWMA` = `GPIO 14`, `AIN1` = `GPIO 26`, `AIN2` = `GPIO 25`
     - `PWMB` = `GPIO 13`, `BIN1` = `GPIO 27`, `BIN2` = `GPIO 19`
     - `STBY` = GPIO-controlled (exact GPIO pin number pending confirmation, e.g., `GPIO 16` / `GPIO 17` / `GPIO 21` / `GPIO 33` if encoders unused)
   - **MPU6050 I2C Pins:**
     - `SDA` = `GPIO 22`, `SCL` = `GPIO 23`

## Remaining Clarifications Needed Before Routing

1. **ESP32 DevKit Variant:** 30-pin (25.4 mm row spacing) or 38-pin (25.4 mm / 22.86 mm row spacing)?
2. **TB6612FNG `STBY` GPIO Pin:** Which ESP32 GPIO do you want assigned to `STBY`?
3. **6-Pin RMC Pinout Order & Encoder Voltage:** Confirm the exact 1-to-6 wire order of your 6-pin motor RMC cable and whether the motor encoders are included (using `GPIO 18, 5, 33, 32` from `Cascaded_PID_encoder_1.ino`) and whether they run on `3.3V` or `5V`.
4. **Board Dimensions & Mounting Holes:** Target width x height (mm) and corner mounting hole diameter (e.g., M3).
