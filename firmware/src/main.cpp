/*
 * ══════════════════════════════════════════════════════════════════════
 *  Stewart Platform 6-DOF Servo Controller
 *  Open-Loop Dead Reckoning for 360° Continuous Rotation Servos
 *  Hardware: Arduino Uno + PCA9685 PWM Driver + 6× Continuous Servos
 *  Protocol: '<A1:90.0,A2:90.0,A3:90.0,A4:90.0,A5:90.0,A6:90.0>\n'
 * ══════════════════════════════════════════════════════════════════════
 *
 *  Control Strategy — "Spin and Kill":
 *    1. Receive absolute target angles from Python IK engine
 *    2. Calculate delta = targetAngle - currentAngle
 *    3. Spin servo in the correct direction for (|delta| * TIME_PER_DEGREE_MS) ms
 *    4. Cut power (setPWM = 0) to stop rotation
 *    5. Update currentAngle = targetAngle (dead reckoning)
 *
 *  All 6 servos move simultaneously using non-blocking millis() timers.
 *
 *  PCA9685 I2C address: 0x40 (default)
 *  PWM frequency: 50 Hz (standard for hobby servos)
 */

#include <Arduino.h>
#include <Wire.h>
#include <Adafruit_PWMServoDriver.h>

// ── Configuration ─────────────────────────────────────────────────
#define NUM_SERVOS       6
#define SERIAL_BAUD      115200

// ── Open-Loop Dead Reckoning Tuning ──────────────────────────────
// TIME_PER_DEGREE_MS: How many milliseconds of spin equals 1° of rotation.
// This is THE critical tuning constant. Measure your servo's speed and adjust.
// Example: if servo does 60 RPM → 360°/sec → 1°/2.78ms → set to ~3.
#define TIME_PER_DEGREE_MS   5

// PCA9685 pulse tick values for continuous rotation direction control.
// At 50 Hz (20ms period), 4096 ticks per period:
//   ~307 ticks = 1500µs = stop (for most continuous servos)
//   >307 = forward spin,  <307 = backward spin
// Adjust these for your specific servos.
#define PULSE_FORWARD    400    // CW spin pulse tick
#define PULSE_BACKWARD   200    // CCW spin pulse tick

// Ignore angle deltas smaller than this (prevents micro-jitter)
#define DEAD_ZONE_DEG    0.5f

// Serial parsing
#define BUF_SIZE         128
#define MSG_TIMEOUT_MS   100     // discard incomplete messages after this

// ── Globals ───────────────────────────────────────────────────────
Adafruit_PWMServoDriver pwm = Adafruit_PWMServoDriver(0x40);

// Servo channel mapping on PCA9685
// S1→ch0, S2→ch4, S3→ch6, S4→ch9, S5→ch11, S6→ch15
const uint8_t servoChannels[NUM_SERVOS] = {0, 4, 6, 9, 11, 15};

// Dead-reckoned position tracking — 127° safe home position
#define SAFE_HOME_ANGLE  127.0f
float currentAngles[NUM_SERVOS]  = {SAFE_HOME_ANGLE, SAFE_HOME_ANGLE, SAFE_HOME_ANGLE, SAFE_HOME_ANGLE, SAFE_HOME_ANGLE, SAFE_HOME_ANGLE};
float targetAngles[NUM_SERVOS]   = {SAFE_HOME_ANGLE, SAFE_HOME_ANGLE, SAFE_HOME_ANGLE, SAFE_HOME_ANGLE, SAFE_HOME_ANGLE, SAFE_HOME_ANGLE};

// Per-servo movement state (non-blocking timers)
bool          servoMoving[NUM_SERVOS]    = {false, false, false, false, false, false};
unsigned long moveStartMs[NUM_SERVOS]    = {0, 0, 0, 0, 0, 0};
unsigned long moveDurationMs[NUM_SERVOS] = {0, 0, 0, 0, 0, 0};
bool          moveProcessed[NUM_SERVOS]  = {true, true, true, true, true, true};

// Serial buffer
char buffer[BUF_SIZE];
int bufIdx = 0;
bool receiving = false;
unsigned long msgStartTime = 0;

// Timing
unsigned long lastUpdateMs = 0;
const unsigned long UPDATE_INTERVAL_MS = 10;  // 100 Hz update check for precise timing

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
            count++;
            token = strtok(NULL, ",");
        }
    } else {
        // Simple CSV format: 90.0,90.0,...
        char* token = strtok(buffer, ",");
        while (token != NULL && count < NUM_SERVOS) {
            angles[count] = atof(token);
            count++;
            token = strtok(NULL, ",");
        }
    }

    // Only apply if we got all 6 angles
    if (count == NUM_SERVOS) {
        for (int i = 0; i < NUM_SERVOS; i++) {
            targetAngles[i] = angles[i];
            moveProcessed[i] = false;  // Flag: new target needs processing
        }

        // Send acknowledgement with delta info
        Serial.print("OK:");
        for (int i = 0; i < NUM_SERVOS; i++) {
            float delta = targetAngles[i] - currentAngles[i];
            Serial.print(targetAngles[i], 1);
            Serial.print("(d");
            Serial.print(delta, 1);
            Serial.print(")");
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

// ── Servo Update Loop — "Spin and Kill" ───────────────────────────

/**
 * Non-blocking servo movement using dead reckoning.
 * Called at high frequency from main loop.
 *
 * For each servo independently:
 *   1. If a new target arrived → calculate delta, start spinning
 *   2. If currently spinning → check if duration elapsed
 *   3. If duration elapsed → KILL power, update position
 */
void updateServos() {
    unsigned long now = millis();

    for (int i = 0; i < NUM_SERVOS; i++) {

        // ── Step 1: Process new target (if not yet processed) ──
        if (!moveProcessed[i] && !servoMoving[i]) {
            float delta = targetAngles[i] - currentAngles[i];

            // Dead zone: skip tiny deltas (prevent jitter)
            if (fabs(delta) < DEAD_ZONE_DEG) {
                currentAngles[i] = targetAngles[i];
                moveProcessed[i] = true;
                continue;
            }

            // Calculate spin duration from delta magnitude
            moveDurationMs[i] = (unsigned long)(fabs(delta) * TIME_PER_DEGREE_MS);
            moveStartMs[i] = now;
            servoMoving[i] = true;
            moveProcessed[i] = true;

            // Choose direction and START spinning
            uint16_t pulse = (delta > 0) ? PULSE_FORWARD : PULSE_BACKWARD;
            pwm.setPWM(servoChannels[i], 0, pulse);

            // Debug output
            Serial.print("  S");
            Serial.print(i + 1);
            Serial.print(": ");
            Serial.print(currentAngles[i], 1);
            Serial.print("° → ");
            Serial.print(targetAngles[i], 1);
            Serial.print("° (Δ");
            Serial.print(delta, 1);
            Serial.print("° = ");
            Serial.print(moveDurationMs[i]);
            Serial.println("ms)");
        }

        // ── Step 2: Check if spinning servo has reached its duration ──
        if (servoMoving[i]) {
            if (now - moveStartMs[i] >= moveDurationMs[i]) {
                // ═══ THE KILL SWITCH ═══
                // Cut power immediately to stop rotation
                pwm.setPWM(servoChannels[i], 0, 0);

                // Update dead-reckoned position
                currentAngles[i] = targetAngles[i];
                servoMoving[i] = false;

                Serial.print("  S");
                Serial.print(i + 1);
                Serial.print(": STOPPED at ");
                Serial.print(currentAngles[i], 1);
                Serial.println("°");
            }
        }
    }
}

// ── Startup ───────────────────────────────────────────────────────

/**
 * Initialize all servos to stopped (no power).
 * Continuous rotation servos cannot be "positioned" —
 * we just ensure they're not spinning on boot.
 */
void initServos() {
    for (int i = 0; i < NUM_SERVOS; i++) {
        // Kill all channels — no spinning on startup
        pwm.setPWM(servoChannels[i], 0, 0);
    }

    Serial.println("─── Servo Init (Dead Reckoning) ───");
    for (int i = 0; i < NUM_SERVOS; i++) {
        Serial.print("  S");
        Serial.print(i + 1);
        Serial.print(" (ch ");
        Serial.print(servoChannels[i]);
        Serial.print("): assumed ");
        Serial.print(currentAngles[i], 1);
        Serial.println("°");
    }
    Serial.println("────────────────────────────────────");
}

// ── Arduino Entry Points ──────────────────────────────────────────

void setup() {
    Serial.begin(SERIAL_BAUD);

    // Initialize PCA9685
    pwm.begin();
    pwm.setOscillatorFrequency(25000000);  // Internal oscillator
    pwm.setPWMFreq(50);                     // 50 Hz for servos

    delay(10);

    // Ensure all servos are STOPPED (no power) on boot
    initServos();

    Serial.println("═══════════════════════════════════════");
    Serial.println("  Stewart Platform Controller Ready");
    Serial.println("  MODE: Open-Loop Dead Reckoning");
    Serial.println("  PCA9685 + 6x Continuous Rotation");
    Serial.print("  Time/degree: ");
    Serial.print(TIME_PER_DEGREE_MS);
    Serial.println(" ms");
    Serial.print("  Fwd pulse: ");
    Serial.print(PULSE_FORWARD);
    Serial.print("  Rev pulse: ");
    Serial.println(PULSE_BACKWARD);
    Serial.println("═══════════════════════════════════════");
}

void loop() {
    // Process incoming serial commands
    processSerial();

    // Update servo movements at fixed rate
    unsigned long now = millis();
    if (now - lastUpdateMs >= UPDATE_INTERVAL_MS) {
        lastUpdateMs = now;
        updateServos();
    }
}