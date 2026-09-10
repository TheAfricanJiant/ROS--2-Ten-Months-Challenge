/*
 * Phase 1 - encoder test for the XRP controller board.
 *
 * Counts quadrature encoder edges on all four motor channels and prints them,
 * so you can turn a wheel by hand and confirm the wiring, the direction and
 * the counts-per-revolution before any of it matters.
 *
 * There is also an optional drive test: it runs each motor briefly and checks
 * the encoder moves the way the motor was told to. That catches swapped
 * encoder channels, which are invisible until you close a control loop and
 * the robot runs away.
 *
 * No micro-ROS, no ROS 2, no network. Serial output only.
 *
 * Pin assignments come from the SparkFun XRP documentation. Set XRP_BOARD
 * below to match yours - the two revisions are wired completely differently.
 */

#include <Arduino.h>

/* ------------------------------------------------------------------ config */

/* 1 = XRP Beta (Raspberry Pi Pico W, RP2040)
 * 2 = XRP Controller (RP2350B, GPIO numbers above 29)            */
#define XRP_BOARD 1

/* Set to 1 to drive each motor briefly and check the encoder follows.
 * PUT THE ROBOT ON A STAND FIRST - the wheels will turn.          */
#define RUN_DRIVE_TEST 0

/* Motor power for the drive test, 0-255. Low enough to be gentle. */
#define DRIVE_TEST_PWM 90
#define DRIVE_TEST_MS 600

/* Counts per output-shaft revolution. The XRP kit motors are a 48:1 gearbox
 * on a 12-count-per-revolution encoder, quadrature-decoded on one channel
 * here, so this is a starting point - measure it with the wheel test and
 * correct it. */
#define COUNTS_PER_REV 585.0f

/* ----------------------------------------------------------------- pinouts */

struct MotorPins {
    const char *name;
    uint8_t encoderA;
    uint8_t encoderB;
    uint8_t phase;      // direction
    uint8_t enable;     // PWM
};

#if XRP_BOARD == 1
/* XRP Beta - Pico W / RP2040. Qwiic + LSM6DSO IMU live on GP18/GP19. */
static const MotorPins MOTORS[] = {
    {"Motor L", 4, 5, 6, 7},
    {"Motor R", 12, 13, 14, 15},
    {"Motor 3", 0, 1, 2, 3},
    {"Motor 4", 8, 9, 10, 11},
};
static const char *BOARD_NAME = "XRP Beta (Pico W / RP2040)";
#elif XRP_BOARD == 2
/* XRP Controller - RP2350B. */
static const MotorPins MOTORS[] = {
    {"Motor L", 30, 31, 34, 35},
    {"Motor R", 24, 25, 32, 33},
    {"Motor 3", 22, 23, 20, 21},
    {"Motor 4", 2, 3, 10, 11},
};
static const char *BOARD_NAME = "XRP Controller (RP2350B)";
#else
#error "Set XRP_BOARD to 1 (Beta/Pico W) or 2 (Controller/RP2350B)"
#endif

static const size_t MOTOR_COUNT = sizeof(MOTORS) / sizeof(MOTORS[0]);

/* ---------------------------------------------------------------- counters */

/* Written from interrupt context, read from loop(). volatile keeps the
 * compiler from caching them; the reads are 32-bit and so are atomic enough
 * on this core for a diagnostic. */
static volatile long counts[MOTOR_COUNT];
static volatile unsigned long edges[MOTOR_COUNT];

/* One handler per channel. Reading B at the moment A changes gives the
 * direction: if they differ, the shaft is going one way; if they match, the
 * other. */
template <size_t INDEX>
static void onEncoderEdge() {
    const bool a = digitalRead(MOTORS[INDEX].encoderA);
    const bool b = digitalRead(MOTORS[INDEX].encoderB);
    counts[INDEX] += (a == b) ? 1 : -1;
    edges[INDEX]++;
}

static void (*const HANDLERS[])() = {
    onEncoderEdge<0>, onEncoderEdge<1>, onEncoderEdge<2>, onEncoderEdge<3>,
};

/* ------------------------------------------------------------------ motors */

static void motorStop(const MotorPins &motor) {
    analogWrite(motor.enable, 0);
}

static void motorDrive(const MotorPins &motor, bool forward, uint8_t power) {
    digitalWrite(motor.phase, forward ? HIGH : LOW);
    analogWrite(motor.enable, power);
}

static long readCount(size_t index) {
    noInterrupts();
    const long value = counts[index];
    interrupts();
    return value;
}

static void resetCounts() {
    noInterrupts();
    for (size_t i = 0; i < MOTOR_COUNT; i++) {
        counts[i] = 0;
        edges[i] = 0;
    }
    interrupts();
}

/* --------------------------------------------------------------------- app */

static void driveTest() {
    Serial.println("\n=== drive test ===");
    Serial.println("Each motor runs briefly in both directions. The encoder");
    Serial.println("count should follow the commanded direction.\n");

    for (size_t i = 0; i < MOTOR_COUNT; i++) {
        for (int direction = 0; direction < 2; direction++) {
            const bool forward = (direction == 0);

            noInterrupts();
            counts[i] = 0;
            interrupts();

            motorDrive(MOTORS[i], forward, DRIVE_TEST_PWM);
            delay(DRIVE_TEST_MS);
            motorStop(MOTORS[i]);
            delay(250);              // let it coast to a stop before reading

            const long moved = readCount(i);
            const char *label = forward ? "forward" : "reverse";

            Serial.printf("  %-8s %-7s -> %+6ld counts   ", MOTORS[i].name, label, moved);

            if (moved == 0) {
                Serial.println("NO MOVEMENT - motor not driven, or encoder not wired");
            } else if ((forward && moved > 0) || (!forward && moved < 0)) {
                Serial.println("ok");
            } else {
                Serial.println("WRONG SIGN - swap the encoder A/B pins");
            }
        }
    }

    Serial.println("\nDrive test finished.\n");
    resetCounts();
}

void setup() {
    Serial.begin(115200);
    unsigned long start = millis();
    while (!Serial && (millis() - start) < 4000) {
        delay(10);
    }

    for (size_t i = 0; i < MOTOR_COUNT; i++) {
        pinMode(MOTORS[i].encoderA, INPUT_PULLUP);
        pinMode(MOTORS[i].encoderB, INPUT_PULLUP);
        pinMode(MOTORS[i].phase, OUTPUT);
        pinMode(MOTORS[i].enable, OUTPUT);
        motorStop(MOTORS[i]);

        /* CHANGE, not RISING: both edges of channel A doubles the resolution
         * and makes direction detection more responsive. */
        attachInterrupt(digitalPinToInterrupt(MOTORS[i].encoderA), HANDLERS[i], CHANGE);
    }

    resetCounts();

    Serial.println();
    Serial.println("=====================================================");
    Serial.println(" XRP encoder test - phase 1");
    Serial.println("=====================================================");
    Serial.printf(" Board: %s\n", BOARD_NAME);
    Serial.println();
    for (size_t i = 0; i < MOTOR_COUNT; i++) {
        Serial.printf("  %-8s encoder A=GP%-2u B=GP%-2u   phase=GP%-2u enable=GP%u\n",
                      MOTORS[i].name, MOTORS[i].encoderA, MOTORS[i].encoderB,
                      MOTORS[i].phase, MOTORS[i].enable);
    }
    Serial.println();
    Serial.println(" Turn a wheel by hand and watch the counts move.");
    Serial.println(" Forward should count up. If a wheel counts down when");
    Serial.println(" driven forward, swap that motor's encoder A and B.");
    Serial.println();
    Serial.println(" Send 'r' to reset counts, 'd' to run the drive test.");
    Serial.println("=====================================================");

#if RUN_DRIVE_TEST
    delay(1500);
    driveTest();
#endif
}

void loop() {
    if (Serial.available()) {
        const char command = (char)Serial.read();
        if (command == 'r' || command == 'R') {
            resetCounts();
            Serial.println("\n-- counts reset --\n");
        } else if (command == 'd' || command == 'D') {
            driveTest();
        }
    }

    Serial.print("  ");
    for (size_t i = 0; i < MOTOR_COUNT; i++) {
        noInterrupts();
        const long count = counts[i];
        const unsigned long edgeCount = edges[i];
        interrupts();

        Serial.printf("%s %+7ld (%.2f rev, %lu edges)   ",
                      MOTORS[i].name, count, count / COUNTS_PER_REV, edgeCount);
    }
    Serial.println();

    delay(250);
}
