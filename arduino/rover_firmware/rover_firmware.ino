/*
 * ============================================================================
 *  ROVER FIRMWARE — Arduino Mega
 * ============================================================================
 *
 *  Controls 4 drive motors + 4 steering actuators via SmartElex 15D drivers.
 *  Reads FlySky iBus receiver for RC control.
 *  Receives autonomous commands from Raspberry Pi via USB Serial.
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

#include <IBusBM.h>

// ============================================================================
//  PIN DEFINITIONS
// ============================================================================

// Drive Motors (SmartElex 15D drivers #1 and #2 — 24V)
// Each motor: PWM pin (speed) + DIR pin (direction)
#define FL_MOTOR_PWM   2
#define FL_MOTOR_DIR   22
#define FR_MOTOR_PWM   3
#define FR_MOTOR_DIR   23
#define RL_MOTOR_PWM   4
#define RL_MOTOR_DIR   24
#define RR_MOTOR_PWM   5
#define RR_MOTOR_DIR   25

// Steering Actuators (SmartElex 15D drivers #3 and #4 — 12V)
#define FL_ACTU_PWM    6
#define FL_ACTU_DIR    26
#define FR_ACTU_PWM    7
#define FR_ACTU_DIR    27
#define RL_ACTU_PWM    8
#define RL_ACTU_DIR    28
#define RR_ACTU_PWM    9
#define RR_ACTU_DIR    29

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

// RC channel mapping (iBus channel indices, 0-based)
#define RC_CH_STEER     0   // CH1: Left/Right stick
#define RC_CH_THROTTLE  1   // CH2: Forward/Reverse stick
#define RC_CH_AUX       2   // CH3: Aux / speed trim
#define RC_CH_ROTATE    3   // CH4: Rotation (spin in place)
#define RC_CH_MODE      4   // CH5: Mode switch (2-pos)
#define RC_CH_LIMIT     5   // CH6: Speed limiter

// RC values
#define RC_CENTER       1500
#define RC_MIN          1000
#define RC_MAX          2000
#define RC_DEADZONE     50    // ±50 around center = deadzone
#define MODE_SWITCH_THR 1500  // CH5 > 1500 = Follow mode

// Safety
#define SERIAL_WATCHDOG_MS  500   // Stop if no RPi command for this long
#define IBUS_TIMEOUT_MS     500   // Stop if no iBus signal for this long
#define SOFT_START_STEP     5     // Max PWM change per loop iteration
#define STATUS_SEND_INTERVAL_MS 100  // Send status to RPi every 100ms

// Motor
#define MAX_MOTOR_PWM   200   // Limit max PWM (0-255) for safety during testing
#define ACTUATOR_PWM    200   // Actuator speed (fixed, they're either on or off)

// ============================================================================
//  GLOBAL VARIABLES
// ============================================================================

IBusBM ibus;

// Current state
uint8_t currentMode = MODE_RC;
bool ibusConnected = false;
unsigned long lastIbusTime = 0;
unsigned long lastRpiCmdTime = 0;
unsigned long lastStatusSendTime = 0;

// RC channel values (raw, 1000-2000)
int16_t rcChannels[6] = {1500, 1500, 1500, 1500, 1000, 1500};

// RPi command values
int16_t rpiSpeed = 0;     // -255 to 255
int16_t rpiSteer = 0;     // -100 to 100

// Current motor outputs (for soft start ramping)
int16_t currentMotorFL = 0;
int16_t currentMotorFR = 0;
int16_t currentMotorRL = 0;
int16_t currentMotorRR = 0;

// Current actuator outputs (-255 to 255, positive = extend, negative = retract)
int16_t currentActuFL = 0;
int16_t currentActuFR = 0;
int16_t currentActuRL = 0;
int16_t currentActuRR = 0;

// Serial receive buffer
uint8_t rxBuffer[CMD_PACKET_SIZE];
uint8_t rxIndex = 0;

// ============================================================================
//  SETUP
// ============================================================================

void setup() {
    // USB Serial to RPi
    Serial.begin(115200);

    // iBus from FlySky receiver on Serial1
    ibus.begin(Serial1);

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
    Serial.println("ROVER FIRMWARE READY");
}

// ============================================================================
//  MAIN LOOP
// ============================================================================

void loop() {
    // 1. Read iBus channels from FlySky receiver
    readIBus();

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
//  iBus READING
// ============================================================================

void readIBus() {
    // IBusBM library handles reading internally via loop()
    // We just need to call readChannel()

    // Check if we're getting valid data
    int ch1 = ibus.readChannel(RC_CH_STEER);

    if (ch1 > 0) {
        // Valid iBus data received
        rcChannels[0] = ch1;
        rcChannels[1] = ibus.readChannel(RC_CH_THROTTLE);
        rcChannels[2] = ibus.readChannel(RC_CH_AUX);
        rcChannels[3] = ibus.readChannel(RC_CH_ROTATE);
        rcChannels[4] = ibus.readChannel(RC_CH_MODE);
        rcChannels[5] = ibus.readChannel(RC_CH_LIMIT);

        lastIbusTime = millis();
        ibusConnected = true;
    } else {
        // Check for iBus timeout
        if (millis() - lastIbusTime > IBUS_TIMEOUT_MS) {
            ibusConnected = false;
        }
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
    // Priority 1: iBus signal lost -> FAILSAFE
    if (!ibusConnected) {
        currentMode = MODE_FAILSAFE;
        return;
    }

    // Priority 2: CH5 switch determines RC vs Follow
    if (rcChannels[4] > MODE_SWITCH_THR) {
        // CH5 HIGH = Follow mode
        // But only if RPi is connected (receiving commands)
        if (millis() - lastRpiCmdTime < SERIAL_WATCHDOG_MS) {
            currentMode = MODE_FOLLOW;
        } else {
            // CH5 says Follow, but RPi isn't sending commands -> Failsafe
            currentMode = MODE_FAILSAFE;
        }
    } else {
        // CH5 LOW = RC mode
        currentMode = MODE_RC;
    }
}

// ============================================================================
//  RC MODE
// ============================================================================

void executeRCMode() {
    // Map RC sticks to motor commands
    // CH2 (throttle): 1000=full reverse, 1500=stop, 2000=full forward
    // CH1 (steering): 1000=full left, 1500=center, 2000=full right
    // CH4 (rotation): 1000=spin left, 1500=stop, 2000=spin right

    int16_t throttle = rcChannels[1];  // CH2
    int16_t steering = rcChannels[0];  // CH1
    int16_t rotation = rcChannels[3];  // CH4

    // Apply speed limiter from CH6
    float speedLimit = mapFloat(rcChannels[5], RC_MIN, RC_MAX, 0.3, 1.0);

    // Apply deadzone
    int16_t throttleCmd = applyDeadzone(throttle, RC_CENTER, RC_DEADZONE);
    int16_t steeringCmd = applyDeadzone(steering, RC_CENTER, RC_DEADZONE);
    int16_t rotationCmd = applyDeadzone(rotation, RC_CENTER, RC_DEADZONE);

    // Map to motor range (-MAX_MOTOR_PWM to +MAX_MOTOR_PWM)
    int16_t fwdSpeed = map(throttleCmd, -500, 500, -MAX_MOTOR_PWM, MAX_MOTOR_PWM);
    fwdSpeed = constrain(fwdSpeed * speedLimit, -MAX_MOTOR_PWM, MAX_MOTOR_PWM);

    // Map steering to actuator command (-100 to 100)
    int16_t steerAngle = map(steeringCmd, -500, 500, -100, 100);

    // Map rotation for spin-in-place
    int16_t spinSpeed = map(rotationCmd, -500, 500, -MAX_MOTOR_PWM, MAX_MOTOR_PWM);
    spinSpeed = constrain(spinSpeed * speedLimit, -MAX_MOTOR_PWM, MAX_MOTOR_PWM);

    // Calculate individual motor speeds
    int16_t targetFL, targetFR, targetRL, targetRR;

    if (abs(spinSpeed) > 10) {
        // Spin mode: left wheels forward, right wheels reverse (or vice versa)
        targetFL = -spinSpeed;
        targetFR = spinSpeed;
        targetRL = -spinSpeed;
        targetRR = spinSpeed;
        // Steering actuators: point wheels for rotation
        setSteeringForSpin();
    } else {
        // Normal driving: differential steering
        // Mix throttle and steering
        targetFL = fwdSpeed;
        targetFR = fwdSpeed;
        targetRL = fwdSpeed;
        targetRR = fwdSpeed;

        // Apply tank-style differential for turning while driving
        // (This supplements the actuator steering for tighter turns)
        int16_t diffMix = map(steeringCmd, -500, 500, -50, 50);
        targetFL -= diffMix;
        targetFR += diffMix;
        targetRL -= diffMix;
        targetRR += diffMix;

        // Set steering actuators
        setSteeringAngle(steerAngle);
    }

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
    // Convert to individual motor speeds and actuator positions

    int16_t speed = rpiSpeed;
    int16_t steer = rpiSteer;  // -100 to 100

    // Calculate motor speeds with differential for steering assist
    int16_t targetFL = speed;
    int16_t targetFR = speed;
    int16_t targetRL = speed;
    int16_t targetRR = speed;

    // Add differential speed for tighter following turns
    int16_t diffMix = map(steer, -100, 100, -40, 40);
    targetFL -= diffMix;
    targetFR += diffMix;
    targetRL -= diffMix;
    targetRR += diffMix;

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
 *
 * For basic steering (front wheels only), rear actuators stay centered.
 * Actuators are driven at full speed (ACTUATOR_PWM) in the desired direction
 * until they reach the target position.
 *
 * NOTE: Without position feedback, this is open-loop timed control.
 * The actuator PWM is set proportional to the error from center.
 */
void setSteeringAngle(int16_t angle) {
    angle = constrain(angle, -100, 100);

    // Front wheels: steer in the commanded direction
    // Map angle to actuator speed/direction
    // Positive angle = turn right = extend right actuator, retract left actuator
    // (actual direction depends on your mechanical linkage — adjust signs as needed)

    int16_t frontActuSpeed;

    if (abs(angle) < 5) {
        // Near center: stop actuators (hold position)
        frontActuSpeed = 0;
    } else {
        // Drive actuators proportional to steering angle
        frontActuSpeed = map(angle, -100, 100, -ACTUATOR_PWM, ACTUATOR_PWM);
    }

    // Front actuators steer in the same direction
    // (for parallel/same-direction steering, both push the same way)
    driveMotor(FL_ACTU_PWM, FL_ACTU_DIR, frontActuSpeed);
    driveMotor(FR_ACTU_PWM, FR_ACTU_DIR, frontActuSpeed);

    // Rear actuators: keep centered (0) for normal driving
    // Set to 0 to stop them / hold position
    driveMotor(RL_ACTU_PWM, RL_ACTU_DIR, 0);
    driveMotor(RR_ACTU_PWM, RR_ACTU_DIR, 0);
}

/**
 * Set steering for spin-in-place mode.
 * Front wheels angle left, rear wheels angle right (or vice versa)
 * to create a rotation pivot at the center of the rover.
 */
void setSteeringForSpin() {
    // Front actuators: push one way
    driveMotor(FL_ACTU_PWM, FL_ACTU_DIR, ACTUATOR_PWM);
    driveMotor(FR_ACTU_PWM, FR_ACTU_DIR, ACTUATOR_PWM);

    // Rear actuators: push opposite way
    driveMotor(RL_ACTU_PWM, RL_ACTU_DIR, -ACTUATOR_PWM);
    driveMotor(RR_ACTU_PWM, RR_ACTU_DIR, -ACTUATOR_PWM);
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
 * Returns value relative to center (e.g., -500 to +500 for RC).
 */
int16_t applyDeadzone(int16_t value, int16_t center, int16_t deadzone) {
    int16_t offset = value - center;
    if (abs(offset) < deadzone) {
        return 0;
    }
    // Scale remaining range to remove the deadzone gap
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
