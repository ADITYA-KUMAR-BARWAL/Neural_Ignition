#include <Arduino.h>
#include <Servo.h>

// ── 6-DOF Stewart Platform Servo Controller ───────────────────────
// Protocol: receive "<a0,a1,a2,a3,a4,a5>\n" where each angle is 0-180
// Pins: 3, 5, 6, 9, 10, 11 (PWM capable on Arduino Uno)

#define NUM_SERVOS 6

Servo servos[NUM_SERVOS];
const int servoPins[NUM_SERVOS] = {3, 5, 6, 9, 10, 11};
float currentAngles[NUM_SERVOS] = {90, 90, 90, 90, 90, 90};

// Serial parsing
const int BUF_SIZE = 64;
char buffer[BUF_SIZE];
int bufIdx = 0;
bool receiving = false;

void setup() {
    Serial.begin(9600);

    for (int i = 0; i < NUM_SERVOS; i++) {
        servos[i].attach(servoPins[i]);
        servos[i].write(90);  // Start at neutral
    }

    Serial.println("Stewart Platform Ready");
}

void parseAndApply() {
    // Parse comma-separated angles from buffer
    float angles[NUM_SERVOS];
    int count = 0;
    char* token = strtok(buffer, ",");

    while (token != NULL && count < NUM_SERVOS) {
        angles[count] = atof(token);
        // Clamp to safe range
        if (angles[count] < 0) angles[count] = 0;
        if (angles[count] > 180) angles[count] = 180;
        count++;
        token = strtok(NULL, ",");
    }

    if (count == NUM_SERVOS) {
        for (int i = 0; i < NUM_SERVOS; i++) {
            currentAngles[i] = angles[i];
            servos[i].write((int)angles[i]);
        }
    }
}

void loop() {
    while (Serial.available() > 0) {
        char c = Serial.read();

        if (c == '<') {
            // Start of message
            receiving = true;
            bufIdx = 0;
            memset(buffer, 0, BUF_SIZE);
        } else if (c == '>') {
            // End of message
            if (receiving) {
                buffer[bufIdx] = '\0';
                parseAndApply();
                receiving = false;
            }
        } else if (receiving && bufIdx < BUF_SIZE - 1) {
            buffer[bufIdx++] = c;
        }
    }
}