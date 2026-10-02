# Complete Rover Wiring Schematic & Pin-by-Pin Reference

This document details **every single physical wire** in the rover, including all `+` / `-` power wires, motor/actuator polarity wires, 6-pin white JST driver connectors, FlySky FS-iA6 receiver pins, Arduino Mega 2560 pins, and Raspberry Pi 4B connections.

---

## 1. Master System Block Schematic

```
                          ┌────────────────────────────────────┐
                          │     Zebronics 4K USB Webcam        │
                          └─────────────────┬──────────────────┘
                                            │ USB 3.0 Cable
                                            ▼
┌────────────────────────┐        ┌────────────────────────────┐
│  FlySky FS-i6 Remote   │        │      Raspberry Pi 4B       │
│  • Sticks: CH1,2,3,4   │        │  • Runs Vision + Follow PID│
│  • Switch: SwB (CH5/6) │        │  • Web UI (:5123)          │
└───────────┬────────────┘        └─────────────┬──────────────┘
            │ 2.4GHz RF                         │ USB A-to-B Serial Cable
            ▼                                   │ (/dev/ttyACM0 @ 115200)
┌────────────────────────┐                      ▼
│  FlySky FS-iA6 Rx      │        ┌────────────────────────────┐
│  • CH1 (S) ────────────┼───────►│ Pin 2  (INT0)              │
│  • CH2 (S) ────────────┼───────►│ Pin 3  (INT1)              │
│  • CH3 (S) ────────────┼───────►│ Pin 18 (INT5)              │
│  • CH5 (S) ────────────┼───────►│ Pin 19 (INT4) [SwB Mode]   │
│  • CH6 (S) ────────────┼───────►│ Pin 20 (INT3) [SwB Backup] │
│  • VCC (+) ────────────┼───────►│ 5V                         │
│  • GND (-) ────────────┼───────►│ GND                        │
└────────────────────────┘        │                            │
                                  │     ARDUINO MEGA 2560      │
                                  │                            │
 ┌────────────────────────────────┤                            ├────────────────────────────────┐
 │ 24V DRIVE MOTORS (Drivers 1&2) │                            │ 12V ACTUATORS (Drivers 3&4)    │
 │ • Pin 4  (PWM) & Pin 22 (DIR)  │                            │ • Pin 8  (PWM) & Pin 13 (DIR)  │
 │ • Pin 5  (PWM) & Pin 23 (DIR)  │                            │ • Pin 9  (PWM) & Pin 12 (DIR)  │
 │ • Pin 6  (PWM) & Pin 24 (DIR)  │                            │ • Pin 10 (PWM) & Pin 14 (DIR)  │
 │ • Pin 7  (PWM) & Pin 25 (DIR)  │                            │ • Pin 11 (PWM) & Pin 15 (DIR)  │
 └──────────────┬─────────────────┘                            └───────────────┬────────────────┘
                │                                                              │
                ▼                                                              ▼
┌───────────────────────────────┐                              ┌───────────────────────────────┐
│ SmartElex 15D #1 & #2 (24V)   │                              │ SmartElex 15D #3 & #4 (12V)   │
│ • Powered by 24V Battery      │                              │ • Powered by 12V Battery      │
│ • Drives 4x Yalu 24V Motors   │                              │ • Drives 4x 12V Actuators     │
└───────────────────────────────┘                              └───────────────────────────────┘
```

---

## 2. SmartElex 15D (SKU: 55900) Board Anatomy

Each of your **4 SmartElex 15D** boards has two sides:
1. **6-Pin White JST Control Header** (Low-voltage 5V logic from Arduino Mega)
2. **6 Screw Terminals** (High-current battery input + Motor 1 & Motor 2 outputs)

```
       SMARTELEX 15D — TOP VIEW (SKU: 55900)
  ┌──────────────────────────────────────────────┐
  │  [VIN/+]  [GND/-]  [M1A] [M1B]  [M2A] [M2B]  │  ◄── 6 Heavy Screw Terminals
  │   Bat +    Bat -    Motor 1      Motor 2     │
  │                                              │
  │                                              │
  │  ┌───────┐                      ┌─────────┐  │
  │  │ 5V    │ (Leave OPEN for 5V)  │ 0 0 1 1 │  │  ◄── 4-Pin DIP Switch
  │  │ Logic │                      │SW1 2 3 4│  │      (SW1=OFF, SW2=OFF, SW3=ON, SW4=ON)
  │  └───────┘                      └─────────┘  │
  │                                              │
  │     [GND]  [5V]  [PWM1] [DIR1] [PWM2] [DIR2] │  ◄── 6-Pin White JST Connector
  └───────┬──────┬──────┬──────┬──────┬──────┬───┘
          │      │      │      │      │      │
         Pin1   Pin2   Pin3   Pin4   Pin5   Pin6
```

### Mandatory Settings on ALL 4 SmartElex 15D Drivers:
* **DIP Switches:** `SW1 = OFF (0)`, `SW2 = OFF (0)`, `SW3 = ON (1)`, `SW4 = ON (1)` $\rightarrow$ **`0 0 1 1` (PWM Independent Mode)**
* **Logic Select Jumper:** **OPEN (No jumper cap)** $\rightarrow$ Selects **5V Logic** for Arduino Mega.
* **Pin 2 (`5V`) on White Connector:** Leave **UNCONNECTED** (Arduino already has its own 5V power via USB).
* **Pin 1 (`GND`) on White Connector:** **MUST** connect to **Arduino Mega `GND`**.

---

## 3. Steering Linear Actuators (12V) — Drivers #3 & #4

### A. Driver #3 (Front-Left & Front-Right Actuators)

#### 1) White 6-Pin Control Connector (Driver #3 ➔ Arduino Mega)
| Driver #3 White Header Pin | Label | Connect To (Arduino Mega) | Function |
|---|---|---|---|
| **Pin 1** | `GND` | **Arduino `GND`** | Common Signal Ground (REQUIRED) |
| **Pin 2** | `5V` | *Leave Disconnected* | Not needed |
| **Pin 3** | `PWM1` | **Arduino `Pin 8`** | **FL Actuator Speed** (PWM) |
| **Pin 4** | `DIR1` | **Arduino `Pin 13`** | **FL Actuator Direction** (`HIGH`=Extend, `LOW`=Retract, Onboard LED) |
| **Pin 5** | `PWM2` | **Arduino `Pin 9`** | **FR Actuator Speed** (PWM) |
| **Pin 6** | `DIR2` | **Arduino `Pin 12`** | **FR Actuator Direction** (`HIGH`=Extend, `LOW`=Retract) |

#### 2) Heavy Screw Terminals (Driver #3 ➔ 12V Battery & Front Actuators)
| Driver #3 Screw Terminal | Connect To | Wire Polarity / Color | Action When `DIR = HIGH` (Turn Right) |
|---|---|---|---|
| **`VIN` (or `B+`)** | **12V Battery Positive (`+`)** | **Red (`+12V`)** | Supplies 12V power |
| **`GND` (or `B-`)** | **12V Battery Negative (`-`)** | **Black (`GND`)** | Power return ground |
| **`M1A` (Motor 1+)** | **FL Actuator Wire 1** | **Red (`+`)** | Outputs `+12V` $\rightarrow$ **FL Actuator EXTENDS** |
| **`M1B` (Motor 1-)** | **FL Actuator Wire 2** | **Black (`-`)** | Outputs `0V` (When `DIR=LOW`, reverses to `+12V` $\rightarrow$ **RETRACTS**) |
| **`M2A` (Motor 2+)** | **FR Actuator Wire 1** | **Red (`+`)** | Outputs `+12V` $\rightarrow$ **FR Actuator EXTENDS** |
| **`M2B` (Motor 2-)** | **FR Actuator Wire 2** | **Black (`-`)** | Outputs `0V` (When `DIR=LOW`, reverses to `+12V` $\rightarrow$ **RETRACTS**) |

> 💡 **Tip:** If an actuator retracts when it should extend, simply **swap its Red and Black wires on `M1A` / `M1B`** (or `M2A` / `M2B`).

---

### B. Driver #4 (Rear-Left & Rear-Right Actuators)

#### 1) White 6-Pin Control Connector (Driver #4 ➔ Arduino Mega)
| Driver #4 White Header Pin | Label | Connect To (Arduino Mega) | Function |
|---|---|---|---|
| **Pin 1** | `GND` | **Arduino `GND`** | Common Signal Ground (REQUIRED) |
| **Pin 2** | `5V` | *Leave Disconnected* | Not needed |
| **Pin 3** | `PWM1` | **Arduino `Pin 10`** | **RL Actuator Speed** (PWM) |
| **Pin 4** | `DIR1` | **Arduino `Pin 14`** | **RL Actuator Direction** |
| **Pin 5** | `PWM2` | **Arduino `Pin 11`** | **RR Actuator Speed** (PWM) |
| **Pin 6** | `DIR2` | **Arduino `Pin 15`** | **RR Actuator Direction** |

#### 2) Heavy Screw Terminals (Driver #4 ➔ 12V Battery & Rear Actuators)
| Driver #4 Screw Terminal | Connect To | Wire Polarity / Color |
|---|---|---|
| **`VIN` (or `B+`)** | **12V Battery Positive (`+`)** | **Red (`+12V`)** |
| **`GND` (or `B-`)** | **12V Battery Negative (`-`)** | **Black (`GND`)** |
| **`M1A` (Motor 1+)** | **RL Actuator Wire 1** | **Red (`+`)** |
| **`M1B` (Motor 1-)** | **RL Actuator Wire 2** | **Black (`-`)** |
| **`M2A` (Motor 2+)** | **RR Actuator Wire 1** | **Red (`+`)** |
| **`M2B` (Motor 2-)** | **RR Actuator Wire 2** | **Black (`-`)** |

---

## 4. Drive Motors (24V 250W Yalu Geared Motors) — Drivers #1 & #2

### A. Driver #1 (Front-Left & Front-Right 24V Drive Motors)

#### 1) White 6-Pin Control Connector (Driver #1 ➔ Arduino Mega)
| Driver #1 White Header Pin | Label | Connect To (Arduino Mega) | Function |
|---|---|---|---|
| **Pin 1** | `GND` | **Arduino `GND`** | Common Signal Ground (REQUIRED) |
| **Pin 2** | `5V` | *Leave Disconnected* | Not needed |
| **Pin 3** | `PWM1` | **Arduino `Pin 4`** | **FL Drive Motor Speed** (PWM) |
| **Pin 4** | `DIR1` | **Arduino `Pin 22`** | **FL Drive Motor Direction** (`HIGH`=Forward, `LOW`=Reverse) |
| **Pin 5** | `PWM2` | **Arduino `Pin 5`** | **FR Drive Motor Speed** (PWM) |
| **Pin 6** | `DIR2` | **Arduino `Pin 23`** | **FR Drive Motor Direction** (`HIGH`=Forward, `LOW`=Reverse) |

#### 2) Heavy Screw Terminals (Driver #1 ➔ 24V Battery & Front Drive Motors)
| Driver #1 Screw Terminal | Connect To | Wire Polarity / Color | Notes |
|---|---|---|---|
| **`VIN` (or `B+`)** | **24V Battery Positive (`+`)** | **Thick Red (`+24V`)** | Use heavy 12–14 AWG wire |
| **`GND` (or `B-`)** | **24V Battery Negative (`-`)** | **Thick Black (`GND`)** | Use heavy 12–14 AWG wire |
| **`M1A` (Motor 1+)** | **FL Motor Wire 1** | **Red (`+`)** | Spins FL wheel **Forward** when `DIR1 = HIGH` |
| **`M1B` (Motor 1-)** | **FL Motor Wire 2** | **Black (`-`)** | Return wire for FL motor |
| **`M2A` (Motor 2+)** | **FR Motor Wire 2** | **Black (`-`)* | *Note: Right-side motors are physically mounted mirrored to Left-side motors! If FR spins backward when FL spins forward, swap FR's Red & Black wires on `M2A`/`M2B`!* |
| **`M2B` (Motor 2-)** | **FR Motor Wire 1** | **Red (`+`)* | |

---

### B. Driver #2 (Rear-Left & Rear-Right 24V Drive Motors)

#### 1) White 6-Pin Control Connector (Driver #2 ➔ Arduino Mega)
| Driver #2 White Header Pin | Label | Connect To (Arduino Mega) | Function |
|---|---|---|---|
| **Pin 1** | `GND` | **Arduino `GND`** | Common Signal Ground (REQUIRED) |
| **Pin 2** | `5V` | *Leave Disconnected* | Not needed |
| **Pin 3** | `PWM1` | **Arduino `Pin 6`** | **RL Drive Motor Speed** (PWM) |
| **Pin 4** | `DIR1` | **Arduino `Pin 24`** | **RL Drive Motor Direction** (`HIGH`=Forward, `LOW`=Reverse) |
| **Pin 5** | `PWM2` | **Arduino `Pin 7`** | **RR Drive Motor Speed** (PWM) |
| **Pin 6** | `DIR2` | **Arduino `Pin 25`** | **RR Drive Motor Direction** (`HIGH`=Forward, `LOW`=Reverse) |

#### 2) Heavy Screw Terminals (Driver #2 ➔ 24V Battery & Rear Drive Motors)
| Driver #2 Screw Terminal | Connect To | Wire Polarity / Color | Notes |
|---|---|---|---|
| **`VIN` (or `B+`)** | **24V Battery Positive (`+`)** | **Thick Red (`+24V`)** | Use heavy 12–14 AWG wire |
| **`GND` (or `B-`)** | **24V Battery Negative (`-`)** | **Thick Black (`GND`)** | Use heavy 12–14 AWG wire |
| **`M1A` (Motor 1+)** | **RL Motor Wire 1** | **Red (`+`)** | Spins RL wheel **Forward** when `DIR1 = HIGH` |
| **`M1B` (Motor 1-)** | **RL Motor Wire 2** | **Black (`-`)** | |
| **`M2A` (Motor 2+)** | **RR Motor Wire 2** | **Black (`-`)* | *Swap Red/Black on `M2A`/`M2B` if RR spins backward when `DIR2 = HIGH`* |
| **`M2B` (Motor 2-)** | **RR Motor Wire 1** | **Red (`+`)* | |

---

## 5. FlySky FS-iA6 Receiver ➔ Arduino Mega 2560

```
  FLYSKY FS-iA6 RECEIVER HEADER (Looking directly at the pins):
  ┌─────────┬─────────┬─────────┬─────────┬─────────┬─────────┬─────────┐
  │   CH1   │   CH2   │   CH3   │   CH4   │   CH5   │   CH6   │  B/VCC  │
  ├─────────┼─────────┼─────────┼─────────┼─────────┼─────────┼─────────┤
  │ [S] Pin2│ [S] Pin3│ [S]Pin18│   (x)   │ [S]Pin19│ [S]Pin20│   (x)   │ ◄── TOP ROW: Signal (S)
  │ [+] 5V  │   (x)   │   (x)   │   (x)   │   (x)   │   (x)   │   (x)   │ ◄── MID ROW: +5V Power (+)
  │ [-] GND │   (x)   │   (x)   │   (x)   │   (x)   │   (x)   │   (x)   │ ◄── BOT ROW: Ground (-)
  └─────────┴─────────┴─────────┴─────────┴─────────┴─────────┴─────────┘
```

| FS-iA6 Receiver Pin | Row | Connect To (Arduino Mega) | Function |
|---|---|---|---|
| **CH1** | **Top Row (`S`)** | **Arduino `Pin 2`** | Steering Stick (Left / Right) |
| **CH2** | **Top Row (`S`)** | **Arduino `Pin 3`** | Throttle Stick (Forward / Reverse) |
| **CH3** | **Top Row (`S`)** | **Arduino `Pin 18`** | Aux Stick |
| **CH5** | **Top Row (`S`)** | **Arduino `Pin 19`** | **`SwB` Mode Switch** (`UP` = RC Mode, `DOWN` = Follow Mode) |
| **CH6** | **Top Row (`S`)** | **Arduino `Pin 20`** | Optional `SwB` Backup Channel |
| **CH1 (or any channel)** | **Middle Row (`+`)** | **Arduino `5V`** | Powers the receiver (`+5V`) |
| **CH1 (or any channel)** | **Bottom Row (`-`)** | **Arduino `GND`** | Receiver Ground (`GND`) |

---

## 6. Power & Common Ground Distribution

```
 ┌──────────────────────┐                      ┌──────────────────────┐
 │   24V BATTERY PACK   │                      │   12V BATTERY PACK   │
 │   (+)            (-) │                      │   (+)            (-) │
 └───┬───────────────┬──┘                      └───┬───────────────┬──┘
     │ +24V          │ 24V GND                     │ +12V          │ 12V GND
     ├──► Driver #1  ├──► Driver #1 B-             ├──► Driver #3  ├──► Driver #3 B-
     │    VIN (B+)   │                             │    VIN (B+)   │
     └──► Driver #2  └──► Driver #2 B-             └──► Driver #4  └──► Driver #4 B-
          VIN (B+)                                      VIN (B+)

               ┌─────────────────────────────────────────┐
               │   COMMON SIGNAL GROUND BUS (Arduino GND)│
               └─────┬───────┬───────┬───────┬───────┬───┘
                     │       │       │       │       │
                     ▼       ▼       ▼       ▼       ▼
                   Drv#1   Drv#2   Drv#3   Drv#4   FS-iA6
                   Pin 1   Pin 1   Pin 1   Pin 1   Bot(-)
                   (GND)   (GND)   (GND)   (GND)   (GND)
```

> ⚠️ **CRITICAL RULE:** Never connect `+24V` or `+12V` to the Arduino or to the 6-pin white JST connector! Only the **`GND` (Pin 1)** of each driver's white JST connector goes to **Arduino `GND`**.
