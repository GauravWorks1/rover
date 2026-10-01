/*
 * Direct Hardware Test for Front-Left Actuator
 * Tests Arduino Pin 8 (PWM) and Pin 26 (DIR) connected to SmartElex 15D Driver #3
 */

#define FL_ACTU_PWM  8
#define FL_ACTU_DIR  26

void setup() {
    pinMode(FL_ACTU_PWM, OUTPUT);
    pinMode(FL_ACTU_DIR, OUTPUT);
}

void loop() {
    // 1. Extend actuator for 2.5 seconds
    digitalWrite(FL_ACTU_DIR, HIGH);
    analogWrite(FL_ACTU_PWM, 200);
    delay(2500);

    // 2. Stop for 1 second
    analogWrite(FL_ACTU_PWM, 0);
    delay(1000);

    // 3. Retract actuator for 2.5 seconds
    digitalWrite(FL_ACTU_DIR, LOW);
    analogWrite(FL_ACTU_PWM, 200);
    delay(2500);

    // 4. Stop for 1 second
    analogWrite(FL_ACTU_PWM, 0);
    delay(1000);
}
