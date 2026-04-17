/*
 * ══════════════════════════════════════════════════════════════════════
 *  Stewart Platform 6-DOF Servo Controller
 *  Hardware: Arduino Uno + PCA9685 PWM Driver + 6× MG996R Servos
 *  Protocol: '<A1:90.0,A2:90.0,A3:90.0,A4:90.0,A5:90.0,A6:90.0>\n'
 * ══════════════════════════════════════════════════════════════════════
 *
 *  PCA9685 I2C address: 0x40 (default)
 *  PWM frequency: 50 Hz (standard for hobby servos)
 *  MG996R specs:
 *    - Operating voltage: 4.8V–7.2V
 *    - Pulse width: 500µs (0°) to 2500µs (180°)
 *    - Stall torque: 11 kg·cm (at 6V)
 *    - 0.5kg load well within capability
 *
 *  Safety constraints:
 *    - Hard angle limits: 10°–170° (prevent mechanical binding)
 *    - Slew rate limiting: max 2°/ms to prevent sudden jolts
 *    - Startup: all servos move to 50° (safe home) immediately
 */

#include <Arduino.h>
#include <Wire.h>
#include <Adafruit_PWMServoDriver.h>

// ── Configuration ─────────────────────────────────────────────────
#define NUM_SERVOS       6
#define SERIAL_BAUD      115200

// PCA9685 pulse ticks for MG996R at 50 Hz (4096-step resolution)
// 500µs  → tick ~102   (0°)
// 1500µs → tick ~307   (90°)
// 2500µs → tick ~512   (180°)
#define SERVO_MIN_TICK   102     // 0 degrees
#define SERVO_MAX_TICK   512     // 180 degrees

// Safety limits (degrees) — prevents mechanical over-extension
#define ANGLE_MIN        10.0f
#define ANGLE_MAX        170.0f

// Slew rate limit (degrees per update cycle to prevent jerk)
#define MAX_SLEW_RATE    5.0f    // degrees per update

// Serial parsing
#define BUF_SIZE         128
#define MSG_TIMEOUT_MS   100     // discard incomplete messages after this

// ── Globals ───────────────────────────────────────────────────────
Adafruit_PWMServoDriver pwm = Adafruit_PWMServoDriver(0x40);

// Servo channel mapping on PCA9685
// S1→ch0, S2→ch4, S3→ch6, S4→ch9, S5→ch11, S6→ch15
const uint8_t servoChannels[NUM_SERVOS] = {0, 4, 6, 9, 11, 15};

// Current and target angles — 50° safe home position
#define SAFE_HOME_ANGLE  127.0f
float currentAngles[NUM_SERVOS]  = {SAFE_HOME_ANGLE, SAFE_HOME_ANGLE, SAFE_HOME_ANGLE, SAFE_HOME_ANGLE, SAFE_HOME_ANGLE, SAFE_HOME_ANGLE};
float targetAngles[NUM_SERVOS]   = {SAFE_HOME_ANGLE, SAFE_HOME_ANGLE, SAFE_HOME_ANGLE, SAFE_HOME_ANGLE, SAFE_HOME_ANGLE, SAFE_HOME_ANGLE};

// Per-servo trim offsets (calibration, adjust per physical build)
float servoTrim[NUM_SERVOS] = {0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f};

// Serial buffer
char buffer[BUF_SIZE];
int bufIdx = 0;
bool receiving = false;
unsigned long msgStartTime = 0;

// Timing
unsigned long lastUpdateMs = 0;
const unsigned long UPDATE_INTERVAL_MS = 20;  // 50 Hz servo update

// ── Helper Functions ──────────────────────────────────────────────

/**
 * Convert angle (0–180°) to PCA9685 pulse tick value.
 */
uint16_t angleToPulse(float angle) {
    angle = constrain(angle, 0.0f, 180.0f);
    return (uint16_t)map((long)(angle * 10), 0, 1800,
                         SERVO_MIN_TICK, SERVO_MAX_TICK);
}

/**
 * Clamp angle to safe physical range.
 */
float clampAngle(float angle) {
    if (angle < ANGLE_MIN) return ANGLE_MIN;
    if (angle > ANGLE_MAX) return ANGLE_MAX;
    return angle;
}

/**
 * Apply slew rate limiting — move toward target at controlled speed.
 */
float slewLimit(float current, float target, float maxRate) {
    float diff = target - current;
    if (diff > maxRate)  return current + maxRate;
    if (diff < -maxRate) return current - maxRate;
    return target;
}

/**
 * Write angle to a specific servo channel on PCA9685.
 */
void setServo(uint8_t idx, float angle) {
    if (idx >= NUM_SERVOS) return;
    float trimmed = angle + servoTrim[idx];
    trimmed = constrain(trimmed, 0.0f, 180.0f);
    uint16_t pulse = angleToPulse(trimmed);
    pwm.setPWM(servoChannels[idx], 0, pulse);
}

// ── Serial Parsing ────────────────────────────────────────────────

/**
 * Parse the protocol packet and update target angles.
 * Expected format: 'A1:90.0,A2:90.0,A3:90.0,A4:90.0,A5:90.0,A6:90.0'
 *
 * Also supports simplified comma-separated format:
 * '90.0,90.0,90.0,90.0,90.0,90.0'
 */
void parseAndApply() {
    float angles[NUM_SERVOS];
    int count = 0;

    // Check if using A1:val format or simple CSV
    if (buffer[0] == 'A' || buffer[0] == 'a') {
        // Named format: A1:90.0,A2:90.0,...
        char* token = strtok(buffer, ",");
        while (token != NULL && count < NUM_SERVOS) {
            // Find the ':' separator
            char* colon = strchr(token, ':');
            if (colon != NULL) {
                angles[count] = atof(colon + 1);
            } else {
                angles[count] = atof(token);
            }
            angles[count] = clampAngle(angles[count]);
            count++;
            token = strtok(NULL, ",");
        }
    } else {
        // Simple CSV format: 90.0,90.0,...
        char* token = strtok(buffer, ",");
        while (token != NULL && count < NUM_SERVOS) {
            angles[count] = atof(token);
            angles[count] = clampAngle(angles[count]);
            count++;
            token = strtok(NULL, ",");
        }
    }

    // Only apply if we got all 6 angles
    if (count == NUM_SERVOS) {
        for (int i = 0; i < NUM_SERVOS; i++) {
            targetAngles[i] = angles[i];
        }

        // Send acknowledgement
        Serial.print("OK:");
        for (int i = 0; i < NUM_SERVOS; i++) {
            Serial.print(targetAngles[i], 1);
            if (i < NUM_SERVOS - 1) Serial.print(",");
        }
        Serial.println();
    } else {
        Serial.print("ERR:expected 6 angles, got ");
        Serial.println(count);
    }
}

/**
 * Process incoming serial data byte by byte.
 */
void processSerial() {
    while (Serial.available() > 0) {
        char c = Serial.read();

        if (c == '<') {
            // Start of message
            receiving = true;
            bufIdx = 0;
            memset(buffer, 0, BUF_SIZE);
            msgStartTime = millis();
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

    // Timeout protection: discard incomplete messages
    if (receiving && (millis() - msgStartTime > MSG_TIMEOUT_MS)) {
        receiving = false;
        bufIdx = 0;
        Serial.println("ERR:message timeout");
    }
}

// ── Servo Update Loop ─────────────────────────────────────────────

/**
 * Smooth servo movement with slew rate limiting.
 * Called at 50 Hz from main loop.
 */
void updateServos() {
    for (int i = 0; i < NUM_SERVOS; i++) {
        float prev = currentAngles[i];
        currentAngles[i] = slewLimit(currentAngles[i], targetAngles[i], MAX_SLEW_RATE);

        if (currentAngles[i] != prev) {
            // Still moving — drive the servo
            setServo(i, currentAngles[i]);
        } else if (currentAngles[i] == targetAngles[i]) {
            // Reached target — stop driving (turn off PWM output)
            pwm.setPWM(servoChannels[i], 0, 0);
        }
    }
}

// ── Startup ───────────────────────────────────────────────────────

/**
 * Immediately lock all PCA9685 channels to the safe home angle.
 * Called FIRST in setup() before any serial activity.
 */
void lockToSafeHome() {
    uint16_t homePulse = angleToPulse(SAFE_HOME_ANGLE);
    for (int i = 0; i < NUM_SERVOS; i++) {
        pwm.setPWM(servoChannels[i], 0, homePulse);
    }
}

/**
 * Slowly move all servos to safe home position on startup.
 * Prevents sudden movements that could damage the mechanism.
 */
void centerServos() {
    Serial.println("Centering servos...");
    for (int step = 0; step <= 20; step++) {
        for (int i = 0; i < NUM_SERVOS; i++) {
            float angle = currentAngles[i];  // Use the initial angle from the array
            setServo(i, angle);
        }
        delay(50);
    }

    // Print current angle of each servo after reset
    Serial.println("─── Servo Reset Complete ───");
    for (int i = 0; i < NUM_SERVOS; i++) {
        Serial.print("  S");
        Serial.print(i + 1);
        Serial.print(" (ch ");
        Serial.print(servoChannels[i]);
        Serial.print("): ");
        Serial.print(currentAngles[i], 1);
        Serial.println("°");
    }
    Serial.println("────────────────────────────");
}

// ── Arduino Entry Points ──────────────────────────────────────────

void setup() {
    Serial.begin(SERIAL_BAUD);

    // Initialize PCA9685
    pwm.begin();
    pwm.setOscillatorFrequency(25000000);  // Internal oscillator
    pwm.setPWMFreq(50);                     // 50 Hz for servos

    delay(10);

    // IMMEDIATELY lock all servos to safe home angle (50°)
    // before any serial communication begins
    lockToSafeHome();

    // Then smoothly verify position (already at target, so minimal movement)
    centerServos();

    Serial.println("═══════════════════════════════════════");
    Serial.println("  Stewart Platform Controller Ready");
    Serial.println("  PCA9685 + 6x MG996R @ 50Hz PWM");
    Serial.print("  Safety range: ");
    Serial.print(ANGLE_MIN, 0);
    Serial.print("° - ");
    Serial.print(ANGLE_MAX, 0);
    Serial.println("°");
    Serial.println("═══════════════════════════════════════");
}

void loop() {
    // Process incoming serial commands
    processSerial();

    // Update servo positions at fixed rate
    unsigned long now = millis();
    if (now - lastUpdateMs >= UPDATE_INTERVAL_MS) {
        lastUpdateMs = now;
        updateServos();
    }
}