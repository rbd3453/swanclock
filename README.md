# LOST Swan Station Split-Flap Clock - ESP32 Firmware

This project provides the firmware for an ESP32-powered split-flap countdown timer modeled after the iconic Swan Station terminal from LOST (108-minute countdown clock).

## Features
- **Homing & Flap Alignment**: Auto-indexes flap position 8 using a Hall-effect sensor on GPIO 26.
- **Precision Stepper Control**: Driven by `AccelStepper` (4096 steps/rev half-step 4-wire configuration on GPIOs 13, 12, 14, 27).
- **Embedded Web Diagnostics**: Retro green-phosphor web terminal at `http://<ESP32-IP>/` with controls for:
  - **EXECUTE (Reset to 108:00)**: Resets timer with dramatic flap rotation animations.
  - **Pause / Resume Timer**
  - **Live Motor Speed Adjustment** (300 to 1000 steps/sec)
  - **Manual Flap Positioning & Rotations Override**

## CLI Commands

### 1. Build Firmware
```bash
python -m platformio run
```

### 2. Upload to ESP32 Board
```bash
python -m platformio run --target upload
```

### 3. Open Serial Monitor (115200 Baud)
```bash
python -m platformio device monitor -b 115200
```
