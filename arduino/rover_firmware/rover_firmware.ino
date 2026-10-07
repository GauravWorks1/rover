/*
 * ============================================================================
 *  ROVER FIRMWARE — Arduino Mega
 * ============================================================================
 *
 *  Controls 4 drive motors + 4 steering actuators via SmartElex 15D drivers.
 *  Reads FlySky FS-iA6 receiver via INDIVIDUAL PWM channels (no iBus needed).
 *  Receives autonomous commands from Raspberry Pi via USB Serial.
 *
 *  RECEIVER: FlySky FS-iA6 (PWM output on each channel pin)
 *
 *  DRIVER ASSIGNMENT (Mixed Voltage Fix):
 *    Driver #1 (24V): CH-A = FL Motor,    CH-B = FR Motor
 *    Driver #2 (24V): CH-A = RL Motor,    CH-B = RR Motor
 *    Driver #3 (12V): CH-A = FL Actuator, CH-B = FR Actuator
 *    Driver #4 (12V): CH-A = RL Actuator, CH-B = RR Actuator
 *
 *  MODES:
 *    RC Mode (FlySky CH5 low):     Motors driven by FlySky stick inputs
 *    Follow Mode (FlySky CH5 high): Motors driven by RPi commands
 *    Failsafe (signal lost):        All motors stopped
 *
 *  SERIAL PROTOCOL (RPi <-> Arduino):
 *    RPi -> Arduino (8 bytes):
 *      [0xAA] [CMD] [SPEED_H] [SPEED_L] [STEER_H] [STEER_L] [CHKSUM] [0x55]
 *    Arduino -> RPi (12 bytes):
 *      [0xBB] [MODE] [CH1_H] [CH1_L] [CH2_H] [CH2_L]
 *      [CH3_H] [CH3_L] [CH5_H] [CH5_L] [CHKSUM] [0x55]
 *
 * ============================================================================
 */

// NO iBus library needed! We read PWM directly.

// ============================================================================
//  PIN DEFINITIONS
// ============================================================================

// Drive Motors (SmartElex 15D drivers #1 and #2 — 24V)
// Each motor: PWM pin (speed) + DIR pin (direction)
#define FL_MOTOR_PWM   4
#define FL_MOTOR_DIR   16   // Moved from 22 to reliable single-row header (TX2)
#define FR_MOTOR_PWM   5
#define FR_MOTOR_DIR   17   // Moved from 23 to reliable single-row header (RX2)
#define RL_MOTOR_PWM   6
#define RL_MOTOR_DIR   A0   // Moved from 24 to reliable Analog header (A0)
#define RR_MOTOR_PWM   7
#define RR_MOTOR_DIR   A1   // Moved from 25 to reliable Analog header (A1)

// Steering Actuators (SmartElex 15D drivers #3 and #4 — 12V)
#define FL_ACTU_PWM    8
#define FL_ACTU_DIR    13   // Verified Working with LED!
#define FR_ACTU_PWM    9
#define FR_ACTU_DIR    12
#define RL_ACTU_PWM    10
#define RL_ACTU_DIR    14
#define RR_ACTU_PWM    11
#define RR_ACTU_DIR    15

// -------------------------------------------------------
//  RC RECEIVER PINS (FS-iA6 PWM outputs)
//  These MUST be interrupt-capable pins on the Mega!
//  Mega interrupt pins: 2, 3, 18, 19, 20, 21
// -------------------------------------------------------
#define RC_PIN_CH1     2    // CH1 Steering    (INT0)
#define RC_PIN_CH2     3    // CH2 Throttle    (INT1)
#define RC_PIN_CH3     18   // CH3 Aux         (INT5)
#define RC_PIN_CH5     19   // CH5 SwB Mode Sw (INT4)
#define RC_PIN_CH6     20   // CH6 SwB / Aux   (INT3)

// ============================================================================
//  CONSTANTS
// ============================================================================

// Serial protocol
#define START_BYTE      0xAA
#define END_BYTE        0x55
#define STATUS_START    0xBB
#define CMD_DRIVE       0x01
#define CMD_STOP        0x02
#define CMD_QUERY       0x03
#define CMD_WEB_DRIVE   0x04   // Direct Left/Right tank motor override from Web UI
#define CMD_PACKET_SIZE    8
#define STATUS_PACKET_SIZE 12

// Operating modes
#define MODE_RC         0
#define MODE_FOLLOW     1
#define MODE_FAILSAFE   2

// RC values
#define RC_CENTER       1500
#define RC_MIN          1000
#define RC_MAX          2000
#define RC_DEADZONE     50    // ±50 around center = deadzone
#define MODE_SWITCH_HIGH 1450  // SwB UP   (1500 to 2000) = RC mode
#define MODE_SWITCH_LOW  1350  // SwB DOWN (~1000-1100)   = Follow mode

// Safety
#define SERIAL_WATCHDOG_MS  1500  // Stop if no RPi command for 1.5s (prevents mode flapping)
#define WEB_WATCHDOG_MS     600   // Auto-release Web override if no web packet for 600ms
#define RC_TIMEOUT_MS       800   // Stop if no RC signal for this long
#define SOFT_START_STEP     5     // Max PWM change per loop iteration
#define STATUS_SEND_INTERVAL_MS 100  // Send status to RPi every 100ms

// Motor
#define MAX_MOTOR_PWM   200   // Limit max PWM (0-255) for safety during testing
#define ACTUATOR_PWM    200   // Actuator speed (fixed, they're either on or off)

// ============================================================================
//  RC PWM READING (Interrupt-driven)
// ============================================================================

// Volatile variables shared between ISR and main loop
volatile uint16_t rc_ch1_raw = 1500;
volatile uint16_t rc_ch2_raw = 1500;
volatile uint16_t rc_ch3_raw = 1500;
volatile uint16_t rc_ch5_raw = 1900;
volatile uint16_t rc_ch6_raw = 1900;

volatile unsigned long rc_ch1_rise = 0;
volatile unsigned long rc_ch2_rise = 0;
volatile unsigned long rc_ch3_rise = 0;
volatile unsigned long rc_ch5_rise = 0;
volatile unsigned long rc_ch6_rise = 0;

volatile unsigned long rc_last_update = 0;  // Timestamp of last valid pulse
volatile unsigned long rc_ch5_last_ms = 0;  // Timestamp of last valid CH5 pulse
volatile unsigned long rc_ch6_last_ms = 0;  // Timestamp of last valid CH6 pulse

// ISR for each channel: measure pulse width (HIGH time)
void isr_ch1() {
    if (digitalRead(RC_PIN_CH1) == HIGH) {
        rc_ch1_rise = micros();
    } else {
        if (rc_ch1_rise > 0) {
            uint16_t pw = (uint16_t)(micros() - rc_ch1_rise);
            if (pw >= 900 && pw <= 2100) {
                rc_ch1_raw = pw;
                rc_last_update = millis();
            }
        }
    }
}

void isr_ch2() {
    if (digitalRead(RC_PIN_CH2) == HIGH) {
        rc_ch2_rise = micros();
    } else {
        if (rc_ch2_rise > 0) {
            uint16_t pw = (uint16_t)(micros() - rc_ch2_rise);
            if (pw >= 900 && pw <= 2100) {
                rc_ch2_raw = pw;
                rc_last_update = millis();
            }
        }
    }
}

void isr_ch3() {
    if (digitalRead(RC_PIN_CH3) == HIGH) {
        rc_ch3_rise = micros();
    } else {
        if (rc_ch3_rise > 0) {
            uint16_t pw = (uint16_t)(micros() - rc_ch3_rise);
            if (pw >= 900 && pw <= 2100) {
                rc_ch3_raw = pw;
                rc_last_update = millis();
            }
        }
    }
}

void isr_ch5() {
    if (digitalRead(RC_PIN_CH5) == HIGH) {
        rc_ch5_rise = micros();
    } else {
        if (rc_ch5_rise > 0) {
            uint16_t pw = (uint16_t)(micros() - rc_ch5_rise);
            if (pw >= 900 && pw <= 2100) {
                rc_ch5_raw = pw;
                unsigned long nowMs = millis();
                rc_last_update = nowMs;
                rc_ch5_last_ms = nowMs;
            }
        }
    }
}

void isr_ch6() {
    if (digitalRead(RC_PIN_CH6) == HIGH) {
        rc_ch6_rise = micros();
    } else {
        if (rc_ch6_rise > 0) {
            uint16_t pw = (uint16_t)(micros() - rc_ch6_rise);
            if (pw >= 900 && pw <= 2100) {
                rc_ch6_raw = pw;
                unsigned long nowMs = millis();
                rc_last_update = nowMs;
                rc_ch6_last_ms = nowMs;
            }
        }
    }
}

// ============================================================================
//  GLOBAL VARIABLES
// ============================================================================

// Current state
uint8_t currentMode = MODE_RC;
bool rcConnected = false;
unsigned long lastRpiCmdTime = 0;
unsigned long lastStatusSendTime = 0;

// RC channel values (copied from ISR safely)
int16_t rcChannels[6] = {1500, 1500, 1500, 1500, 1900, 1500};

// RPi command values
int16_t rpiSpeed = 0;     // -255 to 255
int16_t rpiSteer = 0;     // -100 to 100

// Web manual override values (independent of RC and Follow modes)
bool webOverrideActive = false;
int16_t webLeftSpeed = 0;   // -255 to 255 (FL & RL motors)
int16_t webRightSpeed = 0;  // -255 to 255 (FR & RR motors)
unsigned long lastWebCmdTime = 0;

// Current motor outputs (for soft start ramping)
int16_t currentMotorFL = 0;
int16_t currentMotorFR = 0;
int16_t currentMotorRL = 0;
int16_t currentMotorRR = 0;

// Serial receive buffer
uint8_t rxBuffer[CMD_PACKET_SIZE];
uint8_t rxIndex = 0;

// ============================================================================
//  SETUP
// ============================================================================

void setup() {
    // USB Serial to RPi
    Serial.begin(115200);

    // Configure RC receiver input pins
    pinMode(RC_PIN_CH1, INPUT);
    pinMode(RC_PIN_CH2, INPUT);
    pinMode(RC_PIN_CH3, INPUT);
    pinMode(RC_PIN_CH5, INPUT);
    pinMode(RC_PIN_CH6, INPUT);

    // Attach interrupts (CHANGE = fires on both rising and falling edges)
    attachInterrupt(digitalPinToInterrupt(RC_PIN_CH1), isr_ch1, CHANGE);
    attachInterrupt(digitalPinToInterrupt(RC_PIN_CH2), isr_ch2, CHANGE);
    attachInterrupt(digitalPinToInterrupt(RC_PIN_CH3), isr_ch3, CHANGE);
    attachInterrupt(digitalPinToInterrupt(RC_PIN_CH5), isr_ch5, CHANGE);
    attachInterrupt(digitalPinToInterrupt(RC_PIN_CH6), isr_ch6, CHANGE);

    // Configure motor pins
    pinMode(FL_MOTOR_PWM, OUTPUT);  pinMode(FL_MOTOR_DIR, OUTPUT);
    pinMode(FR_MOTOR_PWM, OUTPUT);  pinMode(FR_MOTOR_DIR, OUTPUT);
    pinMode(RL_MOTOR_PWM, OUTPUT);  pinMode(RL_MOTOR_DIR, OUTPUT);
    pinMode(RR_MOTOR_PWM, OUTPUT);  pinMode(RR_MOTOR_DIR, OUTPUT);

    // Configure actuator pins
    pinMode(FL_ACTU_PWM, OUTPUT);   pinMode(FL_ACTU_DIR, OUTPUT);
    pinMode(FR_ACTU_PWM, OUTPUT);   pinMode(FR_ACTU_DIR, OUTPUT);
    pinMode(RL_ACTU_PWM, OUTPUT);   pinMode(RL_ACTU_DIR, OUTPUT);
    pinMode(RR_ACTU_PWM, OUTPUT);   pinMode(RR_ACTU_DIR, OUTPUT);

    // All motors off initially
    stopAllMotors();

    delay(1000);
    Serial.println("ROVER FIRMWARE READY (PWM RC Mode)");
}

// ============================================================================
//  MAIN LOOP
// ============================================================================

void loop() {
    // 1. Read RC channels from interrupt data
    readRC();

    // 2. Read serial commands from RPi
    readSerialCommands();

    // 3. Determine operating mode
    updateMode();

    // 4. Execute motor control (Web Manual Override takes priority while active)
    if (webOverrideActive && (millis() - lastWebCmdTime <= WEB_WATCHDOG_MS)) {
        // Keep steering actuators still and drive Left/Right motors directly
        setSteeringAngle(0);
        driveMotorsSmooth(webLeftSpeed, webRightSpeed, webLeftSpeed, webRightSpeed);
    } else {
        if (webOverrideActive) {
            // Web watchdog expired -> cleanly stop before returning to normal mode
            webOverrideActive = false;
            webLeftSpeed = 0;
            webRightSpeed = 0;
            stopAllMotors();
        }

        if (currentMode == MODE_FOLLOW) {
            // Follow Mode: executed via RPi vision commands (guarded by SERIAL_WATCHDOG_MS)
            executeFollowMode();
        } else if (currentMode == MODE_RC) {
            if (rcConnected) {
                executeRCMode();
            } else {
                stopAllMotors();
            }
        } else {
            stopAllMotors();
        }
    }

    // 5. Send status to RPi periodically
    if (millis() - lastStatusSendTime >= STATUS_SEND_INTERVAL_MS) {
        sendStatusToRPi();
        lastStatusSendTime = millis();
    }

    // Small delay to prevent overwhelming the loop
    delay(10);  // ~100 Hz loop rate
}

// ============================================================================
//  RC READING (from interrupts)
// ============================================================================

void readRC() {
    static uint8_t activeModePin = 5;  // 5 = Pin 19 (CH5), 6 = Pin 20 (CH6)
    static int16_t prevCh5 = -1;
    static int16_t prevCh6 = -1;

    // Safely copy volatile ISR values with interrupts disabled
    noInterrupts();
    int16_t ch1 = rc_ch1_raw;
    int16_t ch2 = rc_ch2_raw;
    int16_t ch3 = rc_ch3_raw;
    int16_t ch5 = rc_ch5_raw;
    int16_t ch6 = rc_ch6_raw;
    unsigned long lastUpdate = rc_last_update;
    unsigned long ch5Last = rc_ch5_last_ms;
    unsigned long ch6Last = rc_ch6_last_ms;
    interrupts();

    unsigned long nowMs = millis();
    bool ch5Active = (ch5Last > 0) && (nowMs - ch5Last < RC_TIMEOUT_MS);
    bool ch6Active = (ch6Last > 0) && (nowMs - ch6Last < RC_TIMEOUT_MS);

    // Auto-detect SwB switch on either Pin 19 (CH5) or Pin 20 (CH6)
    if (ch6Active && !ch5Active) {
        activeModePin = 6;
    } else if (ch5Active && !ch6Active) {
        activeModePin = 5;
    } else if (ch5Active && ch6Active) {
        if (prevCh6 >= 0 && abs(ch6 - prevCh6) > 250) {
            activeModePin = 6;
        } else if (prevCh5 >= 0 && abs(ch5 - prevCh5) > 250) {
            activeModePin = 5;
        }
    }
    if (ch5Active) prevCh5 = ch5;
    if (ch6Active) prevCh6 = ch6;

    rcChannels[0] = ch1;  // CH1 Steer (Pin 2)
    rcChannels[1] = ch2;  // CH2 Throttle (Pin 3)
    rcChannels[2] = ch3;  // CH3 Aux (Pin 18)
    rcChannels[3] = 1500; // CH4 default center
    rcChannels[4] = (activeModePin == 6) ? ch6 : ch5;  // SwB Mode Switch
    rcChannels[5] = ch6;

    // Check for RC signal timeout
    if (lastUpdate > 0 && (nowMs - lastUpdate < RC_TIMEOUT_MS)) {
        rcConnected = true;
    } else {
        rcConnected = false;
    }
}

// ============================================================================
//  SERIAL COMMAND PARSING (from RPi)
// ============================================================================

void readSerialCommands() {
    while (Serial.available() > 0) {
        uint8_t b = Serial.read();

        if (rxIndex == 0) {
            // Looking for start byte
            if (b == START_BYTE) {
                rxBuffer[0] = b;
                rxIndex = 1;
            }
        } else {
            rxBuffer[rxIndex++] = b;

            // Check if we have a complete packet
            if (rxIndex >= CMD_PACKET_SIZE) {
                parseCommand();
                rxIndex = 0;
            }
        }

        // Safety: reset if buffer gets corrupted
        if (rxIndex >= CMD_PACKET_SIZE) {
            rxIndex = 0;
        }
    }
}

void parseCommand() {
    // Verify end byte
    if (rxBuffer[CMD_PACKET_SIZE - 1] != END_BYTE) {
        rxIndex = 0;
        return;
    }

    // Verify checksum (XOR of bytes 1 through 5)
    uint8_t checksum = 0;
    for (int i = 1; i <= 5; i++) {
        checksum ^= rxBuffer[i];
    }
    if (checksum != rxBuffer[6]) {
        return;  // checksum mismatch, discard
    }

    // Parse command type
    uint8_t cmdType = rxBuffer[1];

    // Parse speed (signed int16, big-endian)
    int16_t speed = (int16_t)((rxBuffer[2] << 8) | rxBuffer[3]);
    // Parse steer (signed int16, big-endian)
    int16_t steer = (int16_t)((rxBuffer[4] << 8) | rxBuffer[5]);

    switch (cmdType) {
        case CMD_DRIVE:
            rpiSpeed = constrain(speed, -255, 255);
            rpiSteer = constrain(steer, -100, 100);
            lastRpiCmdTime = millis();
            break;

        case CMD_STOP:
            rpiSpeed = 0;
            rpiSteer = 0;
            webOverrideActive = false;
            webLeftSpeed = 0;
            webRightSpeed = 0;
            stopAllMotors();
            lastRpiCmdTime = millis();
            break;

        case CMD_QUERY:
            // Just update the timestamp (keeps watchdog alive)
            lastRpiCmdTime = millis();
            sendStatusToRPi();
            break;

        case CMD_WEB_DRIVE:
            // speed = left motors (-255..255), steer = right motors (-255..255)
            webLeftSpeed  = constrain(speed, -MAX_MOTOR_PWM, MAX_MOTOR_PWM);
            webRightSpeed = constrain(steer, -MAX_MOTOR_PWM, MAX_MOTOR_PWM);
            if (webLeftSpeed == 0 && webRightSpeed == 0) {
                webOverrideActive = false;
                stopAllMotors();
            } else {
                webOverrideActive = true;
                lastWebCmdTime = millis();
            }
            lastRpiCmdTime = millis();
            break;
    }
}

// ============================================================================
//  MODE MANAGEMENT
// ============================================================================

void updateMode() {
    // Mode switching via SwB with a 5-cycle debounce filter (50ms)
    // SwB DOWN (~1000) = FOLLOW MODE
    // SwB UP   (~2000) = RC MODE
    static uint8_t followCount = 0;
    static uint8_t rcCount = 0;

    if (rcChannels[4] < MODE_SWITCH_LOW) {
        // SwB DOWN -> FOLLOW MODE
        followCount++;
        rcCount = 0;
        if (followCount >= 5) {
            currentMode = MODE_FOLLOW;
            followCount = 5;  // clamp
        }
    } else if (rcChannels[4] > MODE_SWITCH_HIGH) {
        // SwB UP -> RC MODE
        rcCount++;
        followCount = 0;
        if (rcCount >= 5) {
            currentMode = MODE_RC;
            rcCount = 5;  // clamp
        }
    } else {
        // In deadband: reset counters, keep current mode
        followCount = 0;
        rcCount = 0;
    }
}

// ============================================================================
//  RC MODE
// ============================================================================

void executeRCMode() {
    // Map RC sticks to motor commands
    // CH2 (throttle): 1000=full reverse, 1500=stop, 2000=full forward
    // CH1 (steering): 1000=full left, 1500=center, 2000=full right

    int16_t throttle = rcChannels[1];  // CH2
    int16_t steering = rcChannels[0];  // CH1

    // Apply deadzone
    int16_t throttleCmd = applyDeadzone(throttle, RC_CENTER, RC_DEADZONE);
    int16_t steeringCmd = applyDeadzone(steering, RC_CENTER, RC_DEADZONE);

    // Map to motor range (-MAX_MOTOR_PWM to +MAX_MOTOR_PWM)
    int16_t fwdSpeed = map(throttleCmd, -500, 500, -MAX_MOTOR_PWM, MAX_MOTOR_PWM);
    fwdSpeed = constrain(fwdSpeed, -MAX_MOTOR_PWM, MAX_MOTOR_PWM);

    // Map steering to actuator command (-100 to 100)
    int16_t steerAngle = map(steeringCmd, -500, 500, -100, 100);

    // Calculate individual motor speeds
    int16_t targetFL, targetFR, targetRL, targetRR;

    // Normal driving: differential steering
    targetFL = fwdSpeed;
    targetFR = fwdSpeed;
    targetRL = fwdSpeed;
    targetRR = fwdSpeed;

    // Apply tank-style differential for turning while driving
    // Positive diffMix = stick right = turn right:
    //   Left wheels speed UP, Right wheels slow DOWN
    int16_t diffMix = map(steeringCmd, -500, 500, -50, 50);
    targetFL += diffMix;
    targetFR -= diffMix;
    targetRL += diffMix;
    targetRR -= diffMix;

    // Set steering actuators
    setSteeringAngle(steerAngle);

    // Apply soft start and drive motors
    driveMotorsSmooth(targetFL, targetFR, targetRL, targetRR);
}

// ============================================================================
//  FOLLOW MODE
// ============================================================================

void executeFollowMode() {
    // Check watchdog: if no command from RPi, stop
    if (millis() - lastRpiCmdTime > SERIAL_WATCHDOG_MS) {
        stopAllMotors();
        return;
    }

    // RPi sends overall speed and steering angle
    int16_t speed = rpiSpeed;
    int16_t steer = rpiSteer;  // -100 to 100

    // Calculate motor speeds with differential for steering assist
    int16_t targetFL = speed;
    int16_t targetFR = speed;
    int16_t targetRL = speed;
    int16_t targetRR = speed;

    // Add differential speed for tighter following turns
    int16_t diffMix = map(steer, -100, 100, -40, 40);
    targetFL += diffMix;
    targetFR -= diffMix;
    targetRL += diffMix;
    targetRR -= diffMix;

    // Set steering actuators
    setSteeringAngle(steer);

    // Apply soft start and drive motors
    driveMotorsSmooth(targetFL, targetFR, targetRL, targetRR);
}

// ============================================================================
//  FAILSAFE
// ============================================================================

void executeFailsafe() {
    stopAllMotors();
    rpiSpeed = 0;
    rpiSteer = 0;
}

// ============================================================================
//  MOTOR CONTROL
// ============================================================================

/**
 * Drive a single motor with PWM + Direction.
 * speed: -255 to 255 (negative = reverse)
 */
void driveMotor(uint8_t pwmPin, uint8_t dirPin, int16_t speed) {
    if (speed >= 0) {
        digitalWrite(dirPin, HIGH);  // Forward
        analogWrite(pwmPin, constrain(speed, 0, 255));
    } else {
        digitalWrite(dirPin, LOW);   // Reverse
        analogWrite(pwmPin, constrain(-speed, 0, 255));
    }
}

/**
 * Smoothly ramp all 4 drive motors toward target speeds.
 * Applies SOFT_START_STEP limit per iteration to prevent current spikes.
 */
void driveMotorsSmooth(int16_t targetFL, int16_t targetFR,
                       int16_t targetRL, int16_t targetRR) {
    // Clamp targets
    targetFL = constrain(targetFL, -MAX_MOTOR_PWM, MAX_MOTOR_PWM);
    targetFR = constrain(targetFR, -MAX_MOTOR_PWM, MAX_MOTOR_PWM);
    targetRL = constrain(targetRL, -MAX_MOTOR_PWM, MAX_MOTOR_PWM);
    targetRR = constrain(targetRR, -MAX_MOTOR_PWM, MAX_MOTOR_PWM);

    // Ramp toward targets
    currentMotorFL = rampValue(currentMotorFL, targetFL, SOFT_START_STEP);
    currentMotorFR = rampValue(currentMotorFR, targetFR, SOFT_START_STEP);
    currentMotorRL = rampValue(currentMotorRL, targetRL, SOFT_START_STEP);
    currentMotorRR = rampValue(currentMotorRR, targetRR, SOFT_START_STEP);

    // Apply to hardware
    // NOTE: Right-side motors (FR, RR) are mounted mirrored to Left-side (FL, RL),
    // so we invert (-currentMotorFR, -currentMotorRR) so all 4 wheels roll the same way!
    driveMotor(FL_MOTOR_PWM, FL_MOTOR_DIR,  currentMotorFL);
    driveMotor(FR_MOTOR_PWM, FR_MOTOR_DIR, -currentMotorFR);
    driveMotor(RL_MOTOR_PWM, RL_MOTOR_DIR,  currentMotorRL);
    driveMotor(RR_MOTOR_PWM, RR_MOTOR_DIR, -currentMotorRR);
}

/**
 * Set all 4 steering actuators to a target angle.
 * angle: -100 (full left) to +100 (full right)
 */
void setSteeringAngle(int16_t angle) {
    angle = constrain(angle, -100, 100);

    int16_t actuSpeed = 0;

    if (abs(angle) < 8) {
        // Near center: stop actuators (hold straight)
        actuSpeed = 0;
    } else if (angle > 0) {
        // Turn RIGHT -> Full torque (PWM = 200)
        actuSpeed = ACTUATOR_PWM;
    } else {
        // Turn LEFT -> Full torque reverse (PWM = -200)
        actuSpeed = -ACTUATOR_PWM;
    }

    // Right-side actuators (FR, RR) are mounted mirrored to Left-side (FL, RL),
    // so we invert (-actuSpeed) on FR and RR so left & right steer in the same direction!
    driveMotor(FL_ACTU_PWM, FL_ACTU_DIR,  actuSpeed);
    driveMotor(FR_ACTU_PWM, FR_ACTU_DIR, -actuSpeed);
    driveMotor(RL_ACTU_PWM, RL_ACTU_DIR,  actuSpeed);
    driveMotor(RR_ACTU_PWM, RR_ACTU_DIR, -actuSpeed);
}

/**
 * Emergency stop: all motors and actuators off immediately.
 */
void stopAllMotors() {
    analogWrite(FL_MOTOR_PWM, 0);
    analogWrite(FR_MOTOR_PWM, 0);
    analogWrite(RL_MOTOR_PWM, 0);
    analogWrite(RR_MOTOR_PWM, 0);

    analogWrite(FL_ACTU_PWM, 0);
    analogWrite(FR_ACTU_PWM, 0);
    analogWrite(RL_ACTU_PWM, 0);
    analogWrite(RR_ACTU_PWM, 0);

    currentMotorFL = 0;
    currentMotorFR = 0;
    currentMotorRL = 0;
    currentMotorRR = 0;
}

// ============================================================================
//  STATUS REPORTING (Arduino -> RPi)
// ============================================================================

void sendStatusToRPi() {
    uint8_t packet[STATUS_PACKET_SIZE];

    packet[0] = STATUS_START;  // 0xBB
    packet[1] = currentMode;

    // Pack RC channels as signed int16 big-endian
    packet[2] = (rcChannels[0] >> 8) & 0xFF;  // CH1 high
    packet[3] = rcChannels[0] & 0xFF;          // CH1 low
    packet[4] = (rcChannels[1] >> 8) & 0xFF;  // CH2 high
    packet[5] = rcChannels[1] & 0xFF;          // CH2 low
    packet[6] = (rcChannels[2] >> 8) & 0xFF;  // CH3 high
    packet[7] = rcChannels[2] & 0xFF;          // CH3 low
    packet[8] = (rcChannels[4] >> 8) & 0xFF;  // CH5 high (mode switch)
    packet[9] = rcChannels[4] & 0xFF;          // CH5 low

    // Checksum: XOR of bytes 1 through 9
    uint8_t checksum = 0;
    for (int i = 1; i <= 9; i++) {
        checksum ^= packet[i];
    }
    packet[10] = checksum;
    packet[11] = END_BYTE;  // 0x55

    Serial.write(packet, STATUS_PACKET_SIZE);
}

// ============================================================================
//  UTILITY FUNCTIONS
// ============================================================================

/**
 * Apply deadzone around center value.
 */
int16_t applyDeadzone(int16_t value, int16_t center, int16_t deadzone) {
    int16_t offset = value - center;
    if (abs(offset) < deadzone) {
        return 0;
    }
    if (offset > 0) {
        return map(offset, deadzone, 500, 0, 500);
    } else {
        return map(offset, -500, -deadzone, -500, 0);
    }
}

/**
 * Ramp a value toward a target with a maximum step size.
 */
int16_t rampValue(int16_t current, int16_t target, int16_t maxStep) {
    int16_t diff = target - current;
    if (abs(diff) <= maxStep) {
        return target;
    }
    return current + (diff > 0 ? maxStep : -maxStep);
}

/**
 * Float version of map() for speed limiter.
 */
float mapFloat(long x, long inMin, long inMax, float outMin, float outMax) {
    return (float)(x - inMin) * (outMax - outMin) / (float)(inMax - inMin) + outMin;
}
