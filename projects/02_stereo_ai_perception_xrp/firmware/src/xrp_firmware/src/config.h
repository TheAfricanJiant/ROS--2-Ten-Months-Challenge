/*
 * XRP firmware - board configuration.
 *
 * Every pin and constant here was confirmed by the phase 1 test projects
 * (i2c_scanner, encoder_test, imu_test), not read off a datasheet. Where the
 * hardware disagreed with the documentation, the hardware won - see the
 * motor table below.
 */

#pragma once

#include <Arduino.h>

/* 1 = XRP Beta (Raspberry Pi Pico W, RP2040)
 * 2 = XRP Controller (RP2350B) */
#define XRP_BOARD 1

/* Set to 0 if the board runs out of RAM. Drops the ROS 2 parameter server,
 * which costs three services, and leaves the topics working. */
#define USE_PARAMETER_SERVER 0

/* --------------------------------------------------------------- motors */

struct MotorPins {
    const char *name;
    uint8_t encoderA;
    uint8_t encoderB;
    uint8_t phase;      // direction
    uint8_t enable;     // PWM
    bool reversed;      // flip both the drive and the encoder sign
};

#if XRP_BOARD == 1
/* XRP Beta - Pico W / RP2040.
 *
 * Two things here are not what the documentation says, and both were found
 * by running encoder_test on the actual robot:
 *   - Motor R's encoder A/B are swapped relative to the published pinout.
 *   - Motor 4 drives backwards, so it carries `reversed`.
 * Leave them as they are; they make the wheels agree with the maths. */
static const MotorPins MOTORS[] = {
    {"Motor L", 4,  5,  6,  7,  false},
    {"Motor R", 13, 12, 14, 15, false},   // physical LEFT wheel
    {"Motor 3", 0,  1,  2,  3,  false},
    {"Motor 4", 8,  9,  10, 11, true},    // physical RIGHT wheel
};
static const char *XRP_BOARD_NAME = "XRP Beta (Pico W / RP2040)";

/* IMU lives on Wire1. */
#define IMU_SDA_PIN 18
#define IMU_SCL_PIN 19

#elif XRP_BOARD == 2
static const MotorPins MOTORS[] = {
    {"Motor L", 30, 31, 34, 35, false},
    {"Motor R", 25, 24, 32, 33, false},   // physical LEFT wheel
    {"Motor 3", 22, 23, 20, 21, false},
    {"Motor 4", 2,  3,  10, 11, true},    // physical RIGHT wheel
};
static const char *XRP_BOARD_NAME = "XRP Controller (RP2350B)";

#define IMU_SDA_PIN 38
#define IMU_SCL_PIN 39

#else
#error "Set XRP_BOARD to 1 (Beta/Pico W) or 2 (Controller/RP2350B)"
#endif

static const size_t MOTOR_COUNT = sizeof(MOTORS) / sizeof(MOTORS[0]);

/* Which entries in MOTORS are the driven wheels. Confirmed on the robot. */
static const size_t LEFT_MOTOR = 1;    // "Motor R"
static const size_t RIGHT_MOTOR = 3;   // "Motor 4"

/* ------------------------------------------------------- default tunables */

/* All of these are exposed as ROS 2 parameters at runtime; these are only
 * the values used before the agent connects. */

/* Encoder counts per revolution of the OUTPUT shaft, both edges of channel A
 * counted. Measured with encoder_test by turning a wheel exactly once. */
#define DEFAULT_COUNTS_PER_REV 585.0f

/* XRP kit wheel: 60 mm diameter. Measure yours - odometry scales directly
 * with this. */
#define DEFAULT_WHEEL_RADIUS_M 0.030f

/* Distance between the two wheel contact patches. */
#define DEFAULT_WHEEL_SEPARATION_M 0.155f

/* Speed limits. The robot will not be commanded past these however large a
 * cmd_vel arrives. */
#define DEFAULT_MAX_LINEAR_MPS 0.60f
#define DEFAULT_MAX_ANGULAR_RPS 4.0f

/* PWM below MIN_PWM does not overcome friction; the motor just buzzes. Any
 * non-zero command is lifted to at least this. */
#define DEFAULT_MIN_PWM 50
#define DEFAULT_MAX_PWM 255

/* Stop if no /cmd_vel arrives within this long. A robot that keeps driving
 * after its controller dies is a robot that hits a wall. */
#define DEFAULT_CMD_TIMEOUT_MS 500

/* Closed loop uses the encoders to hit the commanded wheel speed. Open loop
 * just maps speed to PWM and hopes. Start open, tune, then close it. */
#define DEFAULT_CLOSED_LOOP false
#define DEFAULT_KP 120.0f
#define DEFAULT_KI 400.0f

/* Rates */
#define CONTROL_HZ 50
#define ODOM_PUBLISH_HZ 25
#define IMU_PUBLISH_HZ 50
#define JOINT_PUBLISH_HZ 25

/* Frames */
#define ODOM_FRAME "odom"
#define BASE_FRAME "base_link"
#define IMU_FRAME "imu_link"

/* ------------------------------------------------------------------- IMU */

#define IMU_ADDR 0x6B
#define IMU_WHO_AM_I_EXPECTED 0x6C

/* Full-scale settings written in imu_test: 104 Hz, +/-2 g, 250 dps. */
#define IMU_ACCEL_SCALE_G 2.0f
#define IMU_GYRO_SCALE_DPS 250.0f
