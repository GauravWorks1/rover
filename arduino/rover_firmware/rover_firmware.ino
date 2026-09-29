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
#define FL_MOTOR_DIR   22
#define FR_MOTOR_PWM   5
#define FR_MOTOR_DIR   23
#define RL_MOTOR_PWM   6
#define RL_MOTOR_DIR   24
#define RR_MOTOR_PWM   7
#define RR_MOTOR_DIR   25

// Steering Actuators (SmartElex 15D drivers #3 and #4 — 12V)
#define FL_ACTU_PWM    8
#define FL_ACTU_DIR    26
#define FR_ACTU_PWM    9
#define FR_ACTU_DIR    27
#define RL_ACTU_PWM    10
#define RL_ACTU_DIR    28
#define RR_ACTU_PWM    11
#define RR_ACTU_DIR    29

// -------------------------------------------------------
//  RC RECEIVER PINS (FS-iA6 PWM outputs)
//  These MUST be interrupt-capable pins on the Mega!
//  Mega interrupt pins: 2, 3, 18, 19, 20, 21
// -------------------------------------------------------
#define RC_PIN_CH1     2    // CH1 Steering   (INT0)
#define RC_PIN_CH2     3    // CH2 Throttle   (INT1)
#define RC_PIN_CH3     18   // CH3 Aux        (INT5)
#define RC_PIN_CH5     19   // CH5 Mode switch(INT4)
#define RC_PIN_CH6     20   // CH6 Speed limit(INT3)

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
#define MODE_SWITCH_HIGH 1600  // CH5 > 1600 = Follow mode
#define MODE_SWITCH_LOW  1400  // CH5 < 1400 = RC mode

// Safety
#define SERIAL_WATCHDOG_MS  1500  // Stop if no RPi command for 1.5s (prevents mode flapping)
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
volatile uint16_t rc_ch5_raw = 1000;
volatile uint16_t rc_ch6_raw = 1500;

volatile unsigned long rc_ch1_rise = 0;
volatile unsigned long rc_ch2_rise = 0;
volatile unsigned long rc_ch3_rise = 0;
volatile unsigned long rc_ch5_rise = 0;
volatile unsigned long rc_ch6_rise = 0;

volatile unsigned long rc_last_update = 0;  // Timestamp of last valid pulse

// ISR for each channel: measure pulse width (HIGH time)
void isr_ch1() {
    if (digitalRead(RC_PIN_CH1) == HIGH) {
        rc_ch1_rise = micros();
    } else {
        uint16_t pw = (uint16_t)(micros() - rc_ch1_rise);
        if (pw >= 800 && pw <= 2200) {
            rc_ch1_raw = pw;
            rc_last_update = millis();
        }
    }
}

void isr_ch2() {
    if (digitalRead(RC_PIN_CH2) == HIGH) {
        rc_ch2_rise = micros();
    } else {
        uint16_t pw = (uint16_t)(micros() - rc_ch2_rise);
        if (pw >= 800 && pw <= 2200) {
            rc_ch2_raw = pw;
            rc_last_update = millis();
        }
    }
}

void isr_ch3() {
    if (digitalRead(RC_PIN_CH3) == HIGH) {
        rc_ch3_rise = micros();
    } else {
        uint16_t pw = (uint16_t)(micros() - rc_ch3_rise);
        if (pw >= 800 && pw <= 2200) {
            rc_ch3_raw = pw;
            rc_last_update = millis();
        }
    }
}

void isr_ch5() {
    if (digitalRead(RC_PIN_CH5) == HIGH) {
        rc_ch5_rise = micros();
    } else {
        uint16_t pw = (uint16_t)(micros() - rc_ch5_rise);
        if (pw >= 800 && pw <= 2200) {
            rc_ch5_raw = pw;
            rc_last_update = millis();
        }
    }
}

void isr_ch6() {
    if (digitalRead(RC_PIN_CH6) == HIGH) {
        rc_ch6_rise = micros();
    } else {
        uint16_t pw = (uint16_t)(micros() - rc_ch6_rise);
        if (pw >= 800 && pw <= 2200) {
            rc_ch6_raw = pw;
            rc_last_update = millis();
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
int16_t rcChannels[6] = {1500, 1500, 1500, 1500, 1000, 1500};

// RPi command values
int16_t rpiSpeed = 0;     // -255 to 255
int16_t rpiSteer = 0;     // -100 to 100

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

    // 4. Execute mode-specific motor control
    switch (currentMode) {
        case MODE_RC:
            executeRCMode();
            break;
        case MODE_FOLLOW:
            executeFollowMode();
            break;
        case MODE_FAILSAFE:
        default:
            executeFailsafe();
            break;
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
    // Safely copy volatile ISR values with interrupts disabled
    noInterrupts();
    rcChannels[0] = rc_ch1_raw;  // CH1 Steer
    rcChannels[1] = rc_ch2_raw;  // CH2 Throttle
    rcChannels[2] = rc_ch3_raw;  // CH3 Aux
    rcChannels[3] = 1500;        // CH4 not wired (no pin left), default center
    rcChannels[4] = rc_ch5_raw;  // CH5 Mode
    rcChannels[5] = rc_ch6_raw;  // CH6 Speed limit
    unsigned long lastUpdate = rc_last_update;
    interrupts();

    // Check for RC signal timeout
    if (millis() - lastUpdate < RC_TIMEOUT_MS) {
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
            lastRpiCmdTime = millis();
            break;

        case CMD_QUERY:
            // Just update the timestamp (keeps watchdog alive)
            lastRpiCmdTime = millis();
            sendStatusToRPi();
            break;
    }
}

// ============================================================================
//  MODE MANAGEMENT
// ============================================================================

void updateMode() {
    uint8_t targetMode = currentMode;

    // Priority 1: RC signal lost -> FAILSAFE
    if (!rcConnected) {
        targetMode = MODE_FAILSAFE;
    } else {
        // Priority 2: CH5 switch with hysteresis
        if (rcChannels[4] > MODE_SWITCH_HIGH) {
            // CH5 HIGH = Follow mode (if RPi watchdog is alive)
            if (millis() - lastRpiCmdTime < SERIAL_WATCHDOG_MS) {
                targetMode = MODE_FOLLOW;
            } else {
                targetMode = MODE_FAILSAFE;
            }
        } else if (rcChannels[4] < MODE_SWITCH_LOW) {
            // CH5 LOW = RC mode
            targetMode = MODE_RC;
        }
        // Between MODE_SWITCH_LOW and MODE_SWITCH_HIGH: keep targetMode as currentMode
    }

    // Debounce filter: require 5 consecutive cycles (50ms) to switch mode
    static uint8_t candidateMode = MODE_RC;
    static uint8_t debounceCount = 0;

    if (targetMode != currentMode) {
        if (targetMode == candidateMode) {
            debounceCount++;
            if (debounceCount >= 5) {
                currentMode = targetMode;
                debounceCount = 0;
            }
        } else {
            candidateMode = targetMode;
            debounceCount = 1;
        }
    } else {
        debounceCount = 0;
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

    // Apply speed limiter from CH6
    float speedLimit = mapFloat(rcChannels[5], RC_MIN, RC_MAX, 0.3, 1.0);

    // Apply deadzone
    int16_t throttleCmd = applyDeadzone(throttle, RC_CENTER, RC_DEADZONE);
    int16_t steeringCmd = applyDeadzone(steering, RC_CENTER, RC_DEADZONE);

    // Map to motor range (-MAX_MOTOR_PWM to +MAX_MOTOR_PWM)
    int16_t fwdSpeed = map(throttleCmd, -500, 500, -MAX_MOTOR_PWM, MAX_MOTOR_PWM);
    fwdSpeed = constrain(fwdSpeed * speedLimit, -MAX_MOTOR_PWM, MAX_MOTOR_PWM);

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
    driveMotor(FL_MOTOR_PWM, FL_MOTOR_DIR, currentMotorFL);
    driveMotor(FR_MOTOR_PWM, FR_MOTOR_DIR, currentMotorFR);
    driveMotor(RL_MOTOR_PWM, RL_MOTOR_DIR, currentMotorRL);
    driveMotor(RR_MOTOR_PWM, RR_MOTOR_DIR, currentMotorRR);
}

/**
 * Set all 4 steering actuators to a target angle.
 * angle: -100 (full left) to +100 (full right)
 */
void setSteeringAngle(int16_t angle) {
    angle = constrain(angle, -100, 100);

    int16_t frontActuSpeed;

    if (abs(angle) < 5) {
        // Near center: stop actuators (hold position)
        frontActuSpeed = 0;
    } else {
        // Drive actuators proportional to steering angle
        frontActuSpeed = map(angle, -100, 100, -ACTUATOR_PWM, ACTUATOR_PWM);
    }

    // Front actuators steer in the same direction
    driveMotor(FL_ACTU_PWM, FL_ACTU_DIR, frontActuSpeed);
    driveMotor(FR_ACTU_PWM, FR_ACTU_DIR, frontActuSpeed);

    // Rear actuators: keep centered (0) for normal driving
    driveMotor(RL_ACTU_PWM, RL_ACTU_DIR, 0);
    driveMotor(RR_ACTU_PWM, RR_ACTU_DIR, 0);
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
