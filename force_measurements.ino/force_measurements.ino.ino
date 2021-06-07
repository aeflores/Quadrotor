
// ---------------------------------------------------------------------------
#include <Servo.h>
// ---------------------------------------------------------------------------
// Customize here pulse lengths as needed
#define MIN_PULSE_LENGTH 1000 // Minimum pulse length in µs
#define MAX_PULSE_LENGTH 2000 // Maximum pulse length in µs
// ---------------------------------------------------------------------------
Servo motA;
String data;
int power;
// ---------------------------------------------------------------------------

/**
 * Initialisation routine
 */
void setup() {
    Serial.begin(115200);
    motA.attach(3, MIN_PULSE_LENGTH, MAX_PULSE_LENGTH);
    motA.writeMicroseconds(MIN_PULSE_LENGTH);
}

/**
 * Main function
 */
void loop() {
    if (Serial.available()) {
        data = Serial.readString();
        power = data.toInt();
    }
  motA.writeMicroseconds(power + MIN_PULSE_LENGTH);
  Serial.print("Throttle = ");
  Serial.print("\t");
  Serial.println(power);
}
