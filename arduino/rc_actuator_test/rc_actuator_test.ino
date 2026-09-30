/*
 * ============================================================================
 *  RC + FL ACTUATOR TEST — Arduino Mega
 * ============================================================================
 *
 *  Reads FlySky receiver PWM on:
 *    - Pin 2  (CH1 - Steering Stick)
 *    - Pin 19 (CH5 - SwA Switch)
 *
 *  Drives Front-Left (FL) Actuator on:
 *    - Pin 8  (PWM Speed)
 *    - Pin 26 (DIR Direction)
 *
 *  BEHAVIOR:
 *    - Push Stick RIGHT (> 1550 us) -> Actuator EXTENDS  (DIR HIGH, PWM 200)
 *    - Push Stick LEFT  (< 1450 us) -> Actuator RETRACTS (DIR LOW,  PWM 200)
 *    - Stick CENTERED   (1450-1550) -> Actuator STOPS    (PWM 0)
 *
 *  Prints real-time pulse width & action to Serial Monitor at 115200 baud.
 * ============================================================================
 */

// Arduino Mega Receiver Input Pins (Interrupts)
#define RC_PIN_CH1   2    // INT0 (CH1 Steering Stick)
#define RC_PIN_CH5   19   // INT4 (CH5 SwA Switch)

// SmartElex 15D Driver #3 Pins
#define FL_ACTU_PWM  8    // Speed (PWM)
#define FL_ACTU_DIR  26   // Direction (DIR)

// RC Constants
#define RC_CENTER    1500
#define DEADZONE     60    // 1440 to 1560 is center deadzone
#define ACTU_SPEED   200   // Speed from 0 to 255

volatile uint16_t ch1_raw = 1500;
volatile uint16_t ch5_raw = 1000;
volatile unsigned long ch1_rise = 0;
volatile unsigned long ch5_rise = 0;
volatile unsigned long last_signal_time = 0;

void isr_ch1() {
    if (digitalRead(RC_PIN_CH1) == HIGH) {
        ch1_rise = micros();
    } else if (ch1_rise > 0) {
        uint16_t pw = (uint16_t)(micros() - ch1_rise);
        if (pw >= 800 && pw <= 2200) {
            ch1_raw = pw;
            last_signal_time = millis();
        }
    }
}

void isr_ch5() {
    if (digitalRead(RC_PIN_CH5) == HIGH) {
        ch5_rise = micros();
    } else if (ch5_rise > 0) {
        uint16_t pw = (uint16_t)(micros() - ch5_rise);
        if (pw >= 800 && pw <= 2200) {
            ch5_raw = pw;
            last_signal_time = millis();
        }
    }
}

void setup() {
    Serial.begin(115200);

    // Motor driver pins
    pinMode(FL_ACTU_PWM, OUTPUT);
    pinMode(FL_ACTU_DIR, OUTPUT);
    analogWrite(FL_ACTU_PWM, 0);
    digitalWrite(FL_ACTU_DIR, LOW);

    // Receiver interrupt pins
    pinMode(RC_PIN_CH1, INPUT);
    pinMode(RC_PIN_CH5, INPUT);
    attachInterrupt(digitalPinToInterrupt(RC_PIN_CH1), isr_ch1, CHANGE);
    attachInterrupt(digitalPinToInterrupt(RC_PIN_CH5), isr_ch5, CHANGE);

    Serial.println("=================================================");
    Serial.println("  RC + FL ACTUATOR TEST READY (115200 BAUD)");
    Serial.println("=================================================");
    Serial.println("Move Right Stick Left/Right to drive the Actuator!");
    Serial.println();
}

void loop() {
    noInterrupts();
    uint16_t steer = ch1_raw;
    uint16_t sw = ch5_raw;
    unsigned long lastSig = last_signal_time;
    interrupts();

    bool hasSignal = (millis() - lastSig < 600);

    Serial.print("CH1: ");
    Serial.print(steer);
    Serial.print(" us | CH5: ");
    Serial.print(sw);
    Serial.print(" us -> ");

    if (!hasSignal) {
        analogWrite(FL_ACTU_PWM, 0);
        Serial.println("NO RECEIVER SIGNAL (Check power/bind/wiring)");
    } 
    else if (steer > (RC_CENTER + DEADZONE)) {
        // Stick pushed RIGHT / UP (> 1560 us)
        digitalWrite(FL_ACTU_DIR, HIGH);
        analogWrite(FL_ACTU_PWM, ACTU_SPEED);
        Serial.println(">> EXTENDING (DIR=HIGH, PWM=200)");
    } 
    else if (steer < (RC_CENTER - DEADZONE)) {
        // Stick pushed LEFT / DOWN (< 1440 us)
        digitalWrite(FL_ACTU_DIR, LOW);
        analogWrite(FL_ACTU_PWM, ACTU_SPEED);
        Serial.println("<< RETRACTING (DIR=LOW, PWM=200)");
    } 
    else {
        // Stick in CENTER (1440 - 1560 us)
        analogWrite(FL_ACTU_PWM, 0);
        Serial.println("STOPPED [CENTER]");
    }

    delay(100);
}