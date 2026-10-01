/*
 * ============================================================================
 *  DIRECT RC ACTUATOR CONTROL (Pin 13 LED Visual DIR) — Arduino Mega
 * ============================================================================
 *
 *  Hardware Connections:
 *    - SmartElex PWM1 (Speed)     -> Arduino Pin 8
 *    - SmartElex DIR1 (Direction) -> Arduino Pin 13  (With Onboard LED!)
 *    - SmartElex GND              -> Arduino GND
 *
 *    - Receiver CH3 (Left Stick Up/Down)    -> Arduino Pin 2 (Top pin 'S')
 *    - Receiver CH4 (Left Stick Left/Right) -> Arduino Pin 3 (Top pin 'S')
 *    - Receiver VCC & GND                   -> Arduino 5V & GND
 *
 *  Visual Verification:
 *    - Push Stick UP   -> Arduino 'L' LED turns ON (5V)  -> EXTENDS
 *    - Push Stick DOWN -> Arduino 'L' LED turns OFF (0V) -> RETRACTS
 *    - Stick CENTER    -> Motor STOPS
 * ============================================================================
 */

#define ACTU_PWM     8    // SmartElex PWM1 (Speed)
#define ACTU_DIR     13   // SmartElex DIR1 (Direction + Built-in LED on Pin 13!)

#define RC_PIN_CH3   2    // Receiver CH3 (Left Stick Up/Down)
#define RC_PIN_CH4   3    // Receiver CH4 (Left Stick Left/Right)

#define RC_CENTER    1500
#define DEADZONE     60    // 1440 - 1560 is center deadzone
#define SPEED_VAL    200   // Speed (0 - 255)

volatile uint16_t ch3_pw = 1500;
volatile uint16_t ch4_pw = 1500;
volatile unsigned long ch3_rise = 0, ch4_rise = 0;
volatile unsigned long last_sig = 0;

void isr_ch3() {
    if (digitalRead(RC_PIN_CH3) == HIGH) {
        ch3_rise = micros();
    } else if (ch3_rise > 0) {
        uint16_t pw = (uint16_t)(micros() - ch3_rise);
        if (pw >= 800 && pw <= 2200) {
            ch3_pw = pw;
            last_sig = millis();
        }
    }
}

void isr_ch4() {
    if (digitalRead(RC_PIN_CH4) == HIGH) {
        ch4_rise = micros();
    } else if (ch4_rise > 0) {
        uint16_t pw = (uint16_t)(micros() - ch4_rise);
        if (pw >= 800 && pw <= 2200) {
            ch4_pw = pw;
            last_sig = millis();
        }
    }
}

void setup() {
    Serial.begin(115200);

    // Motor driver control pins
    pinMode(ACTU_PWM, OUTPUT);
    pinMode(ACTU_DIR, OUTPUT);
    analogWrite(ACTU_PWM, 0);
    digitalWrite(ACTU_DIR, LOW);

    // Receiver interrupt pins
    pinMode(RC_PIN_CH3, INPUT);
    pinMode(RC_PIN_CH4, INPUT);
    attachInterrupt(digitalPinToInterrupt(RC_PIN_CH3), isr_ch3, CHANGE);
    attachInterrupt(digitalPinToInterrupt(RC_PIN_CH4), isr_ch4, CHANGE);

    Serial.println("==================================================");
    Serial.println("  ACTUATOR CONTROL READY (DIR ON PIN 13 LED)      ");
    Serial.println("==================================================");
    Serial.println("PWM = Pin 8 | DIR = Pin 13 | CH3 = Pin 2 | CH4 = Pin 3");
    Serial.println("Push UP -> LED ON (5V) -> EXTEND");
    Serial.println("Push DOWN -> LED OFF (0V) -> RETRACT");
    Serial.println();
}

void loop() {
    noInterrupts();
    uint16_t ch3 = ch3_pw;
    uint16_t ch4 = ch4_pw;
    unsigned long lastTime = last_sig;
    interrupts();

    bool hasSignal = (millis() - lastTime < 600);

    // Pick active command from either CH3 or CH4
    int16_t command = 1500;
    if (abs((int16_t)ch3 - RC_CENTER) > DEADZONE) {
        command = ch3;
    } else if (abs((int16_t)ch4 - RC_CENTER) > DEADZONE) {
        command = ch4;
    }

    Serial.print("CH3: "); Serial.print(ch3);
    Serial.print(" us | CH4: "); Serial.print(ch4);
    Serial.print(" us -> ");

    if (!hasSignal) {
        analogWrite(ACTU_PWM, 0);
        digitalWrite(ACTU_DIR, LOW);
        Serial.println("NO SIGNAL");
    } 
    else if (command > (RC_CENTER + DEADZONE)) {
        // Push UP (> 1560 us) -> EXTEND (5V on Pin 13, LED ON)
        digitalWrite(ACTU_DIR, HIGH);
        analogWrite(ACTU_PWM, SPEED_VAL);
        Serial.println(">> EXTENDING (Pin 13 = HIGH / 5V, LED ON)");
    } 
    else if (command < (RC_CENTER - DEADZONE)) {
        // Push DOWN (< 1440 us) -> RETRACT (0V on Pin 13, LED OFF)
        digitalWrite(ACTU_DIR, LOW);
        analogWrite(ACTU_PWM, SPEED_VAL);
        Serial.println("<< RETRACTING (Pin 13 = LOW / 0V, LED OFF)");
    } 
    else {
        // Stick in CENTER -> STOP
        analogWrite(ACTU_PWM, 0);
        Serial.println("STOPPED (Center)");
    }

    delay(100);
}