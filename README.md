# Autonomous Follow-Me Rover

A 4-wheel independently-steerable rover with **Follow-Me** (person following) and **RC Control** modes.

## Hardware

| Component | Spec | Qty |
|---|---|---|
| Drive Motors | Yalu 24V 250W 360RPM Brush Motor | 4 |
| Steering | Linear Actuator 12V 100mm 7mm/s | 4 |
| Motor Drivers | SmartElex 15D (Dual Motor, PWM & RC) | 4 |
| Controller | Raspberry Pi 4B | 1 |
| Microcontroller | Arduino Mega 2560 | 1 |
| Camera | USB Webcam | 1 |
| RC System | FlySky Transmitter + Receiver (iBus) | 1 |

### Driver Assignment (Mixed Voltage Solution)
- **Driver #1 + #2** (24V supply): 4× drive motors (2 per driver)
- **Driver #3 + #4** (12V supply): 4× steering actuators (2 per driver)

## Setup

### 1. Wire Everything
See [docs/wiring_diagram.md](docs/wiring_diagram.md) for complete pin connections.

### 2. Flash Arduino Firmware
1. Install the **IBusBM** library via Arduino IDE Library Manager
2. Open `arduino/rover_firmware/rover_firmware.ino`
3. Select Board: **Arduino Mega 2560**
4. Upload

### 3. Setup Raspberry Pi
```bash
# Install Python dependencies
pip install -r requirements.txt

# Download the MobileNet SSD model
cd models
bash download_models.sh
cd ..
```

### 4. Run
```bash
# Full mode (camera + RC)
cd rpi
python main.py

# RC only (no camera)
python main.py --no-camera

# Custom serial port
python main.py --port /dev/ttyUSB0

# Debug logging
python main.py --debug
```

## Operating Modes

| Mode | FlySky CH5 | Behavior |
|---|---|---|
| **RC Mode** | Switch DOWN (low) | FlySky sticks control the rover |
| **Follow-Me** | Switch UP (high) | Camera detects person and follows |
| **Failsafe** | Signal lost | All motors stop immediately |

### FlySky Channel Assignments
- **CH1**: Steering (left/right)
- **CH2**: Throttle (forward/reverse)
- **CH4**: Spin in place
- **CH5**: Mode switch (2-position)
- **CH6**: Speed limiter

## Architecture

```
RPi 4B (Python)                    Arduino Mega
┌─────────────────────┐           ┌─────────────────────┐
│ USB Webcam → OpenCV  │           │ FlySky iBus → CH1-6 │
│ MobileNet SSD detect │  USB     │                     │
│ PID Follow Control   │◄────────►│ Mode Switch Logic   │
│ Serial Commands      │  Serial  │ Motor PWM Output    │
└─────────────────────┘           │ Soft Start + Safety │
                                  └──────┬──────────────┘
                                         │ PWM + DIR
                            ┌────────────┼────────────────┐
                            ▼            ▼                ▼
                       SmartElex     SmartElex        SmartElex
                       15D ×2        15D ×2
                       (24V motors)  (12V actuators)
```

## Project Structure

```
rover/
├── rpi/                    # Raspberry Pi Python code
│   ├── main.py             # Entry point
│   ├── config.py           # All configuration constants
│   ├── person_detector.py  # OpenCV + MobileNet SSD
│   ├── follow_controller.py# PID follow logic
│   └── serial_comm.py      # Arduino communication
├── arduino/
│   └── rover_firmware/
│       └── rover_firmware.ino  # Arduino Mega firmware
├── models/
│   └── download_models.sh  # Download ML model files
├── docs/
│   └── wiring_diagram.md   # Complete wiring guide
├── requirements.txt        # Python dependencies
└── README.md               # This file
```

## Tuning

### PID Gains (in `rpi/config.py`)
- `KP_STEER`, `KI_STEER`, `KD_STEER` — Steering responsiveness
- `KP_SPEED`, `KI_SPEED`, `KD_SPEED` — Speed/distance control
- `TARGET_AREA_RATIO` — How close the rover follows (higher = closer)

### Motor Limits (in Arduino firmware)
- `MAX_MOTOR_PWM` — Maximum drive motor speed (start low: 100, increase to 200)
- `SOFT_START_STEP` — Acceleration ramp rate
- `RC_DEADZONE` — Stick deadzone size

## Safety Notes
- **Always test with `MAX_MOTOR_PWM` set LOW** (e.g., 50) first
- The **E-Stop switch** should cut main battery power
- FlySky signal loss triggers automatic motor stop
- RPi communication loss triggers automatic motor stop in Follow mode
- Soft-start prevents sudden current spikes
