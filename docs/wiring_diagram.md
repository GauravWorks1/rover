# Rover Wiring Diagram

## Driver Assignment (Mixed Voltage Fix)

```
  24V BATTERY                              12V BATTERY (or 24V→12V Buck)
      │                                          │
      ├──────────────────┐                        ├──────────────────┐
      │                  │                        │                  │
┌─────▼─────┐    ┌───────▼───┐            ┌───────▼───┐    ┌────────▼──┐
│ Driver #1  │    │ Driver #2 │            │ Driver #3 │    │ Driver #4 │
│ SmartElex  │    │ SmartElex │            │ SmartElex │    │ SmartElex │
│   15D      │    │   15D     │            │   15D     │    │   15D     │
│ (24V VIN)  │    │ (24V VIN) │            │ (12V VIN) │    │ (12V VIN) │
├────────────┤    ├───────────┤            ├───────────┤    ├───────────┤
│CH-A: FL Mtr│    │CH-A:RL Mtr│            │CH-A:FL Act│    │CH-A:RL Act│
│CH-B: FR Mtr│    │CH-B:RR Mtr│            │CH-B:FR Act│    │CH-B:RR Act│
└────────────┘    └───────────┘            └───────────┘    └───────────┘
```

## Wheel Layout (Top View)

```
        FRONT (Camera here)
    ┌───────────────────────┐
    │                       │
 FL │                       │ FR
 ●──┤                       ├──●
    │       ROVER BODY      │
    │     [RPi + Arduino]   │
 RL │                       │ RR
 ●──┤                       ├──●
    │                       │
    └───────────────────────┘
         REAR
```

## Arduino Mega Pin Mapping

### Drive Motors → SmartElex 15D Drivers #1 & #2 (24V)

| Arduino Pin | Type | Function           | Connected To              |
|-------------|------|--------------------|---------------------------|
| D2          | PWM  | FL Motor Speed     | Driver #1, CH-A PWM Input |
| D22         | DIG  | FL Motor Direction | Driver #1, CH-A DIR Input |
| D3          | PWM  | FR Motor Speed     | Driver #1, CH-B PWM Input |
| D23         | DIG  | FR Motor Direction | Driver #1, CH-B DIR Input |
| D4          | PWM  | RL Motor Speed     | Driver #2, CH-A PWM Input |
| D24         | DIG  | RL Motor Direction | Driver #2, CH-A DIR Input |
| D5          | PWM  | RR Motor Speed     | Driver #2, CH-B PWM Input |
| D25         | DIG  | RR Motor Direction | Driver #2, CH-B DIR Input |

### Steering Actuators → SmartElex 15D Drivers #3 & #4 (12V)

| Arduino Pin | Type | Function           | Connected To              |
|-------------|------|--------------------|---------------------------|
| D6          | PWM  | FL Actuator Speed  | Driver #3, CH-A PWM Input |
| D26         | DIG  | FL Actuator Dir    | Driver #3, CH-A DIR Input |
| D7          | PWM  | FR Actuator Speed  | Driver #3, CH-B PWM Input |
| D27         | DIG  | FR Actuator Dir    | Driver #3, CH-B DIR Input |
| D8          | PWM  | RL Actuator Speed  | Driver #4, CH-A PWM Input |
| D28         | DIG  | RL Actuator Dir    | Driver #4, CH-A DIR Input |
| D9          | PWM  | RR Actuator Speed  | Driver #4, CH-B PWM Input |
| D29         | DIG  | RR Actuator Dir    | Driver #4, CH-B DIR Input |

### Communication

| Arduino Pin | Function              | Connected To                   |
|-------------|-----------------------|--------------------------------|
| Serial0 USB | RPi Communication     | RPi 4B USB Port (via USB cable)|
| Pin 19 (RX1)| FlySky iBus Input    | FlySky Receiver iBus/Servo Pin |

### Grounds (CRITICAL)

| Connection              | Notes                                   |
|-------------------------|-----------------------------------------|
| Arduino GND → RPi GND  | Common ground via USB cable              |
| Arduino GND → Driver #1 GND | Shared signal ground              |
| Arduino GND → Driver #2 GND | Shared signal ground              |
| Arduino GND → Driver #3 GND | Shared signal ground              |
| Arduino GND → Driver #4 GND | Shared signal ground              |
| 24V Battery GND → Driver #1 GND | Power ground                  |
| 24V Battery GND → Driver #2 GND | Power ground                  |
| 12V Battery GND → Driver #3 GND | Power ground                  |
| 12V Battery GND → Driver #4 GND | Power ground                  |
| All GNDs tied together  | SINGLE COMMON GROUND POINT              |

## SmartElex 15D DIP Switch Settings

Set ALL 4 drivers to **PWM Input Mode**:

| Switch | Position | Function          |
|--------|----------|-------------------|
| SW1    | Refer to manual | Mode select |
| SW2    | Refer to manual | Mode select |
| SW3    | OFF      | 5V logic (Arduino)|
| SW4    | Refer to manual | Baud rate   |

> **IMPORTANT**: Set the I/P Logic Select jumper to **5V** on all drivers
> (since Arduino Mega is 5V logic).

## FlySky Receiver Wiring

```
FlySky Receiver (e.g., FS-iA6B)
    ├── iBus/Servo Pin ──────── Arduino Mega Pin 19 (RX1)
    ├── VCC ─────────────────── 5V (from BEC or Arduino 5V)
    └── GND ─────────────────── Arduino GND
```

## Power Distribution

```
24V Battery ─┬── Driver #1 VIN (FL + FR Motors)
             ├── Driver #2 VIN (RL + RR Motors)
             ├── 24V→12V Buck Converter ─┬── Driver #3 VIN (FL + FR Actuators)
             │                           └── Driver #4 VIN (RL + RR Actuators)
             └── 24V→5V Buck Converter ──── RPi 4B (5V 3A via USB-C)

Arduino Mega powered via USB from RPi (5V from RPi USB port)
```

## Safety Components

| Component               | Location                    | Rating    |
|--------------------------|-----------------------------|-----------|
| Main Fuse               | 24V Battery positive lead   | 60A       |
| Motor Fuse (×4)         | Each motor positive lead    | 15A each  |
| Actuator Fuse (×4)      | Each actuator positive lead | 5A each   |
| E-Stop Switch           | Accessible on rover body    | Main power|
| 100µF Capacitor (×4)    | Across each driver VIN/GND  | 25V+      |
