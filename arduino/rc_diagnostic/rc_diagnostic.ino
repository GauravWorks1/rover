/*
 * ============================================================================
 *  RECEIVER MULTI-PIN DIAGNOSTIC SCANNER (Arduino Mega)
 * ============================================================================
 *  Directly scans all RC input pins (Pins 2, 3, 18, 19, 20) using pulseIn.
 *  Shows live pulse width in microseconds on Serial Monitor (115200 baud).
 * ============================================================================
 */

void setup() {
    Serial.begin(115200);

    pinMode(2, INPUT);
    pinMode(3, INPUT);
    pinMode(18, INPUT);
    pinMode(19, INPUT);
    pinMode(20, INPUT);

    Serial.println("==================================================");
    Serial.println("     FLY-SKY RECEIVER PIN SCANNER (115200 BAUD)   ");
    Serial.println("==================================================");
    Serial.println("Move sticks and switches on your remote to see which pin changes!");
    Serial.println();
}

void loop() {
    // Read raw pulse width on each pin (timeout 25ms = 1 full PWM cycle)
    unsigned long p2  = pulseIn(2, HIGH, 25000);
    unsigned long p3  = pulseIn(3, HIGH, 25000);
    unsigned long p18 = pulseIn(18, HIGH, 25000);
    unsigned long p19 = pulseIn(19, HIGH, 25000);
    unsigned long p20 = pulseIn(20, HIGH, 25000);

    Serial.print("Pin 2: ");   Serial.print(p2);   Serial.print(" us  |  ");
    Serial.print("Pin 3: ");   Serial.print(p3);   Serial.print(" us  |  ");
    Serial.print("Pin 18: ");  Serial.print(p18);  Serial.print(" us  |  ");
    Serial.print("Pin 19: ");  Serial.print(p19);  Serial.print(" us  |  ");
    Serial.print("Pin 20: ");  Serial.print(p20);  Serial.println(" us");

    delay(100);
}
