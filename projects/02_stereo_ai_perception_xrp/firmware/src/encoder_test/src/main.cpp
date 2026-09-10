/*
 * Phase 1 - encoder test & serial drive control for XRP board.
 */

#include <Arduino.h>

/* ------------------------------------------------------------------ config */

#define XRP_BOARD 1
#define RUN_DRIVE_TEST 0

/* Motor power levels (0-255) */
#define DRIVE_TEST_PWM 180
#define DRIVE_TEST_MS 600

#define COUNTS_PER_REV 585.0f

/* ----------------------------------------------------------------- pinouts */

struct MotorPins {
    const char *name;
    uint8_t encoderA;
    uint8_t encoderB;
    uint8_t phase;
    uint8_t enable;
    bool reversed;
};

#if XRP_BOARD == 1
static const MotorPins MOTORS[] = {
    {"Motor L", 4,  5,  6,  7, false},
    {"Motor R", 13, 12, 14, 15, false}, // Physical Left Motor
    {"Motor 3", 0,  1,  2,  3, false},
    {"Motor 4", 8,  9, 10, 11, true},  // Physical Right Motor
};
static const char *XRP_BOARD_NAME = "XRP Beta (Pico W / RP2040)";
#elif XRP_BOARD == 2
static const MotorPins MOTORS[] = {
    {"Motor L", 30, 31, 34, 35, false},
    {"Motor R", 25, 24, 32, 33, false}, // Physical Left Motor
    {"Motor 3", 22, 23, 20, 21, false},
    {"Motor 4", 2,  3, 10, 11, true},  // Physical Right Motor
};
static const char *XRP_BOARD_NAME = "XRP Controller (RP2350B)";
#else
#error "Set XRP_BOARD to 1 (Beta/Pico W) or 2 (Controller/RP2350B)"
#endif

static const size_t MOTOR_COUNT = sizeof(MOTORS) / sizeof(MOTORS[0]);

/* Differential Drive Mapping */
static const size_t LEFT_MOTOR  = 1; // Motor R
static const size_t RIGHT_MOTOR = 3; // Motor 4

static int current_pwm = 180;

/* Movement State Tracking */
enum DriveState {
    STATE_STOPPED,
    STATE_FORWARD,
    STATE_BACKWARD,
    STATE_FAST_TURN_LEFT,
    STATE_FAST_TURN_RIGHT,
    STATE_SOFT_TURN_LEFT,
    STATE_SOFT_TURN_RIGHT
};
static DriveState current_state = STATE_STOPPED;

/* ---------------------------------------------------------------- counters */

static volatile long counts[MOTOR_COUNT];
static volatile unsigned long edges[MOTOR_COUNT];

template <size_t INDEX>
static void onEncoderEdge()
{
    const bool a = digitalRead(MOTORS[INDEX].encoderA);
    const bool b = digitalRead(MOTORS[INDEX].encoderB);

    long step = (a == b) ? 1 : -1;
    if (MOTORS[INDEX].reversed) {
        step = -step;
    }
    
    counts[INDEX] += step;
    edges[INDEX]++;
}

static void (*const HANDLERS[])() = {
    onEncoderEdge<0>,
    onEncoderEdge<1>,
    onEncoderEdge<2>,
    onEncoderEdge<3>
};

/* ------------------------------------------------------------------ motors */

static void motorStop(const MotorPins &motor)
{
    analogWrite(motor.enable, 0);
}

static void motorDrive(const MotorPins &motor, bool forward, uint8_t power)
{
    bool actualDirection = motor.reversed ? !forward : forward;
    digitalWrite(motor.phase, actualDirection ? HIGH : LOW);
    analogWrite(motor.enable, power);
}

static void stopAllMotors()
{
    for (size_t i = 0; i < MOTOR_COUNT; i++) {
        motorStop(MOTORS[i]);
    }
}

/* Applies the active movement state using current_pwm (enables live speed changes) */
static void applyCurrentState()
{
    switch (current_state) {
        case STATE_FORWARD:
            motorDrive(MOTORS[LEFT_MOTOR],  true,  current_pwm);
            motorDrive(MOTORS[RIGHT_MOTOR], true,  current_pwm);
            break;
        case STATE_BACKWARD:
            motorDrive(MOTORS[LEFT_MOTOR],  false, current_pwm);
            motorDrive(MOTORS[RIGHT_MOTOR], false, current_pwm);
            break;
        case STATE_FAST_TURN_LEFT:
            motorDrive(MOTORS[LEFT_MOTOR],  false, current_pwm);
            motorDrive(MOTORS[RIGHT_MOTOR], true,  current_pwm);
            break;
        case STATE_FAST_TURN_RIGHT:
            motorDrive(MOTORS[LEFT_MOTOR],  true,  current_pwm);
            motorDrive(MOTORS[RIGHT_MOTOR], false, current_pwm);
            break;
        case STATE_SOFT_TURN_LEFT:
            motorStop(MOTORS[LEFT_MOTOR]);
            motorDrive(MOTORS[RIGHT_MOTOR], true,  current_pwm);
            break;
        case STATE_SOFT_TURN_RIGHT:
            motorDrive(MOTORS[LEFT_MOTOR],  true,  current_pwm);
            motorStop(MOTORS[RIGHT_MOTOR]);
            break;
        case STATE_STOPPED:
        default:
            stopAllMotors();
            break;
    }
}

/* ---------------------------------------------------------------- counters */

static long readCount(size_t index)
{
    noInterrupts();
    const long value = counts[index];
    interrupts();
    return value;
}

static unsigned long readEdges(size_t index)
{
    noInterrupts();
    const unsigned long value = edges[index];
    interrupts();
    return value;
}

static void resetCounts()
{
    noInterrupts();
    for (size_t i = 0; i < MOTOR_COUNT; i++) {
        counts[i] = 0;
        edges[i] = 0;
    }
    interrupts();
}

/* --------------------------------------------------------------- drive test */

static void driveTest()
{
    Serial.println();
    Serial.println("=====================================================");
    Serial.println(" DRIVE TEST");
    Serial.println("=====================================================");
    Serial.println("Each motor will run briefly in both directions.");
    Serial.println("WARNING: wheels should be OFF THE GROUND.\n");

    delay(1000);

    for (size_t i = 0; i < MOTOR_COUNT; i++) {
        for (int direction = 0; direction < 2; direction++) {
            const bool forward = (direction == 0);

            noInterrupts();
            counts[i] = 0;
            edges[i] = 0;
            interrupts();

            motorDrive(MOTORS[i], forward, DRIVE_TEST_PWM);
            delay(DRIVE_TEST_MS);
            motorStop(MOTORS[i]);
            delay(250);

            const long moved = readCount(i);
            const char *label = forward ? "forward" : "reverse";

            Serial.printf("  %-8s %-7s -> %+6ld counts   ", MOTORS[i].name, label, moved);

            if (moved == 0) {
                Serial.println("NO MOVEMENT - check motor/encoder wiring");
            } else {
                if ((forward && moved > 0) || (!forward && moved < 0)) {
                    Serial.println("OK");
                } else {
                    Serial.println("WRONG SIGN - encoder A/B reversed");
                }
            }
            delay(250);
        }
        Serial.println();
    }
    Serial.println("Drive test finished.\n");
    resetCounts();
}

/* ------------------------------------------------------------------- setup */

void setup()
{
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

        attachInterrupt(
            digitalPinToInterrupt(MOTORS[i].encoderA),
            HANDLERS[i],
            CHANGE
        );
    }

    resetCounts();

    Serial.println("\n=====================================================");
    Serial.println(" XRP DIFFERENTIAL DRIVE CONTROL (LIVE SPEED)");
    Serial.println("=====================================================");
    Serial.printf(" Board: %s\n\n", XRP_BOARD_NAME);
    
    Serial.println("Controls:");
    Serial.println("  w = Move Forward");
    Serial.println("  s = Move Backward");
    Serial.println("  q = Fast Turn Left");
    Serial.println("  e = Fast Turn Right");
    Serial.println("  a = Soft Turn Left");
    Serial.println("  d = Soft Turn Right");
    Serial.println("  + = Increase Speed (Live)");
    Serial.println("  - = Decrease Speed (Live)");
    Serial.println("  x = Stop Motors");
    Serial.println("  r = Reset Encoder Counts");
    Serial.println("  t = Run Drive Test\n");
    Serial.println("=====================================================");
}

/* -------------------------------------------------------------------- loop */

void loop()
{
    if (Serial.available()) {
        const char command = (char)Serial.read();

        if (command == 'r' || command == 'R') {
            resetCounts();
            Serial.println("\n-- encoder counts reset --\n");
        } 
        else if (command == 't' || command == 'T') {
            driveTest();
        } 
        else if (command == 'w' || command == 'W') {
            current_state = STATE_FORWARD;
            applyCurrentState();
            Serial.printf("\n-- MOVING FORWARD (Speed: %d) --\n", current_pwm);
        }
        else if (command == 's' || command == 'S') {
            current_state = STATE_BACKWARD;
            applyCurrentState();
            Serial.printf("\n-- MOVING BACKWARD (Speed: %d) --\n", current_pwm);
        }
        else if (command == 'q' || command == 'Q') {
            current_state = STATE_FAST_TURN_LEFT;
            applyCurrentState();
            Serial.println("\n-- FAST TURN LEFT --\n");
        }
        else if (command == 'e' || command == 'E') {
            current_state = STATE_FAST_TURN_RIGHT;
            applyCurrentState();
            Serial.println("\n-- FAST TURN RIGHT --\n");
        }
        else if (command == 'a' || command == 'A') {
            current_state = STATE_SOFT_TURN_LEFT;
            applyCurrentState();
            Serial.println("\n-- SOFT TURN LEFT --\n");
        }
        else if (command == 'd' || command == 'D') {
            current_state = STATE_SOFT_TURN_RIGHT;
            applyCurrentState();
            Serial.println("\n-- SOFT TURN RIGHT --\n");
        }
        else if (command == '+' || command == '=') {
            current_pwm = min(255, current_pwm + 25);
            applyCurrentState(); // Takes effect live immediately if moving!
            Serial.printf("\n-- SPEED INCREASED: %d --\n", current_pwm);
        }
        else if (command == '-' || command == '_') {
            current_pwm = max(50, current_pwm - 25);
            applyCurrentState(); // Takes effect live immediately if moving!
            Serial.printf("\n-- SPEED DECREASED: %d --\n", current_pwm);
        }
        else if (command == 'x' || command == 'X') {
            current_state = STATE_STOPPED;
            applyCurrentState();
            Serial.println("\n-- MOTORS STOPPED --\n");
        }
    }

    Serial.print("  ");
    for (size_t i = 0; i < MOTOR_COUNT; i++) {
        const long count = readCount(i);
        const unsigned long edgeCount = readEdges(i);
        const float revolutions = count / COUNTS_PER_REV;

        Serial.printf("%s %+7ld (%.2f rev, %lu edges)   ", 
            MOTORS[i].name, count, revolutions, edgeCount);
    }
    Serial.println();

    delay(250);
}