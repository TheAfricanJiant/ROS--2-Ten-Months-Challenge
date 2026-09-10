/*
 * Differential drive: motors, encoders and odometry.
 *
 * Grown out of the encoder_test project. The encoder ISRs, the `reversed`
 * handling and the motor-to-wheel mapping are unchanged from the version that
 * was verified on the robot - only the speed control and odometry are new.
 *
 * Header-only so there is one translation unit to reason about.
 */

#pragma once

#include <Arduino.h>

#include "config.h"

/* --------------------------------------------------------------- encoders */

/* Written from interrupt context, read from the control loop. 32-bit reads
 * are atomic enough on this core, but the sign flip below still needs the
 * pair read together, so callers use snapshot(). */
static volatile long g_counts[MOTOR_COUNT];
static volatile unsigned long g_edges[MOTOR_COUNT];

/* One handler per channel, exactly as encoder_test proved it. Reading B at
 * the moment A changes gives the direction. */
template <size_t INDEX>
static void onEncoderEdge() {
    const bool a = digitalRead(MOTORS[INDEX].encoderA);
    const bool b = digitalRead(MOTORS[INDEX].encoderB);

    long step = (a == b) ? 1 : -1;
    if (MOTORS[INDEX].reversed) {
        step = -step;
    }
    g_counts[INDEX] += step;
    g_edges[INDEX]++;
}

static void (*const ENCODER_HANDLERS[])() = {
    onEncoderEdge<0>, onEncoderEdge<1>, onEncoderEdge<2>, onEncoderEdge<3>,
};

/* ------------------------------------------------------------- drivetrain */

struct WheelState {
    float position_rad = 0.0f;
    float velocity_rad_s = 0.0f;
    long counts = 0;
};

struct Odometry {
    float x = 0.0f;
    float y = 0.0f;
    float theta = 0.0f;
    float linear = 0.0f;      // m/s
    float angular = 0.0f;     // rad/s
};

class Drivetrain {
public:
    /* Runtime-tunable, all exposed as ROS 2 parameters. */
    float counts_per_rev = DEFAULT_COUNTS_PER_REV;
    float wheel_radius = DEFAULT_WHEEL_RADIUS_M;
    float wheel_separation = DEFAULT_WHEEL_SEPARATION_M;
    float max_linear = DEFAULT_MAX_LINEAR_MPS;
    float max_angular = DEFAULT_MAX_ANGULAR_RPS;
    int min_pwm = DEFAULT_MIN_PWM;
    int max_pwm = DEFAULT_MAX_PWM;
    bool closed_loop = DEFAULT_CLOSED_LOOP;
    float kp = DEFAULT_KP;
    float ki = DEFAULT_KI;
    bool invert_left = false;
    bool invert_right = false;
    bool enabled = true;

    void begin() {
        for (size_t i = 0; i < MOTOR_COUNT; i++) {
            pinMode(MOTORS[i].encoderA, INPUT_PULLUP);
            pinMode(MOTORS[i].encoderB, INPUT_PULLUP);
            pinMode(MOTORS[i].phase, OUTPUT);
            pinMode(MOTORS[i].enable, OUTPUT);
            stopMotor(i);
            attachInterrupt(digitalPinToInterrupt(MOTORS[i].encoderA),
                            ENCODER_HANDLERS[i], CHANGE);
        }
        resetCounts();
        _last_update_us = micros();
    }

    /* -- commands ---------------------------------------------------- */

    /* A velocity command, in the units ROS 2 speaks. */
    void setTwist(float linear_mps, float angular_rps) {
        _cmd_linear = constrain(linear_mps, -max_linear, max_linear);
        _cmd_angular = constrain(angular_rps, -max_angular, max_angular);
        _direct_mode = false;
        _last_cmd_ms = millis();
    }

    /* Raw per-wheel duty, -1..+1. For bench testing and for behaviours that
     * want the wheels directly rather than through the kinematics. */
    void setDirect(float left_duty, float right_duty) {
        _direct_left = constrain(left_duty, -1.0f, 1.0f);
        _direct_right = constrain(right_duty, -1.0f, 1.0f);
        _direct_mode = true;
        _last_cmd_ms = millis();
    }

    void stop() {
        _cmd_linear = _cmd_angular = 0.0f;
        _direct_left = _direct_right = 0.0f;
        _integral_left = _integral_right = 0.0f;
        for (size_t i = 0; i < MOTOR_COUNT; i++) {
            stopMotor(i);
        }
    }

    bool commandStale(unsigned long timeout_ms) const {
        return (millis() - _last_cmd_ms) > timeout_ms;
    }

    /* -- the loop ---------------------------------------------------- */

    void update() {
        const unsigned long now_us = micros();
        float dt = (now_us - _last_update_us) * 1e-6f;
        if (dt <= 0.0f || dt > 0.5f) {       // first call, or a long stall
            dt = 1.0f / CONTROL_HZ;
        }
        _last_update_us = now_us;

        updateWheels(dt);
        updateOdometry(dt);

        if (!enabled) {
            for (size_t i = 0; i < MOTOR_COUNT; i++) {
                stopMotor(i);
            }
            return;
        }

        if (_direct_mode) {
            applyDuty(LEFT_MOTOR, _direct_left, invert_left);
            applyDuty(RIGHT_MOTOR, _direct_right, invert_right);
            return;
        }

        /* Differential drive: the wheels differ by half the turn rate times
         * the track width. */
        const float half_track = wheel_separation * 0.5f;
        const float target_left_mps = _cmd_linear - _cmd_angular * half_track;
        const float target_right_mps = _cmd_linear + _cmd_angular * half_track;

        applyDuty(LEFT_MOTOR, wheelDuty(target_left_mps, left, _integral_left, dt),
                  invert_left);
        applyDuty(RIGHT_MOTOR, wheelDuty(target_right_mps, right, _integral_right, dt),
                  invert_right);
    }

    /* -- state ------------------------------------------------------- */

    WheelState left;
    WheelState right;
    Odometry odom;

    long rawCounts(size_t index) const {
        noInterrupts();
        const long value = g_counts[index];
        interrupts();
        return value;
    }

    void resetCounts() {
        noInterrupts();
        for (size_t i = 0; i < MOTOR_COUNT; i++) {
            g_counts[i] = 0;
            g_edges[i] = 0;
        }
        interrupts();
        _last_left_counts = _last_right_counts = 0;
        left = WheelState();
        right = WheelState();
    }

    void resetOdometry() {
        odom = Odometry();
        resetCounts();
    }

private:
    float _cmd_linear = 0.0f;
    float _cmd_angular = 0.0f;
    float _direct_left = 0.0f;
    float _direct_right = 0.0f;
    bool _direct_mode = false;
    unsigned long _last_cmd_ms = 0;
    unsigned long _last_update_us = 0;
    long _last_left_counts = 0;
    long _last_right_counts = 0;
    float _integral_left = 0.0f;
    float _integral_right = 0.0f;

    static void stopMotor(size_t index) {
        analogWrite(MOTORS[index].enable, 0);
    }

    /* Duty is -1..+1. Sign picks the direction, magnitude the PWM. */
    void applyDuty(size_t index, float duty, bool invert) {
        if (invert) {
            duty = -duty;
        }
        duty = constrain(duty, -1.0f, 1.0f);

        if (fabsf(duty) < 1e-3f) {
            stopMotor(index);
            return;
        }

        const bool forward = duty > 0.0f;
        /* Lift small commands over the friction floor, so "slow" still turns
         * the wheel instead of just buzzing. */
        const int span = max_pwm - min_pwm;
        int pwm = min_pwm + (int)(fabsf(duty) * span);
        pwm = constrain(pwm, 0, 255);

        const bool actual = MOTORS[index].reversed ? !forward : forward;
        digitalWrite(MOTORS[index].phase, actual ? HIGH : LOW);
        analogWrite(MOTORS[index].enable, pwm);
    }

    /* Convert a wheel speed in m/s to a duty, open or closed loop. */
    float wheelDuty(float target_mps, const WheelState &state,
                    float &integral, float dt) {
        const float max_wheel_mps = max_linear > 0.0f ? max_linear : 1.0f;

        if (!closed_loop) {
            return target_mps / max_wheel_mps;
        }

        const float measured_mps = state.velocity_rad_s * wheel_radius;
        const float error = target_mps - measured_mps;

        integral += error * dt;
        /* Clamp the integrator, or a stalled wheel winds it up and the robot
         * lurches the moment it comes free. */
        integral = constrain(integral, -1.0f, 1.0f);

        const float duty = (kp * error + ki * integral) / 1000.0f
                           + target_mps / max_wheel_mps;
        return constrain(duty, -1.0f, 1.0f);
    }

    void updateWheels(float dt) {
        const long left_counts = rawCounts(LEFT_MOTOR);
        const long right_counts = rawCounts(RIGHT_MOTOR);

        const long dl = left_counts - _last_left_counts;
        const long dr = right_counts - _last_right_counts;
        _last_left_counts = left_counts;
        _last_right_counts = right_counts;

        const float rad_per_count = counts_per_rev > 0.0f
                                        ? (2.0f * PI / counts_per_rev)
                                        : 0.0f;

        left.counts = left_counts;
        right.counts = right_counts;
        left.position_rad = left_counts * rad_per_count;
        right.position_rad = right_counts * rad_per_count;

        /* Light smoothing: encoder velocity at 50 Hz on a low-count encoder
         * is quantised enough to be jumpy. */
        const float alpha = 0.4f;
        const float left_raw = (dl * rad_per_count) / dt;
        const float right_raw = (dr * rad_per_count) / dt;
        left.velocity_rad_s += alpha * (left_raw - left.velocity_rad_s);
        right.velocity_rad_s += alpha * (right_raw - right.velocity_rad_s);
    }

    void updateOdometry(float dt) {
        const float left_mps = left.velocity_rad_s * wheel_radius;
        const float right_mps = right.velocity_rad_s * wheel_radius;

        odom.linear = (right_mps + left_mps) * 0.5f;
        odom.angular = wheel_separation > 0.0f
                           ? (right_mps - left_mps) / wheel_separation
                           : 0.0f;

        /* Midpoint integration: use the heading halfway through the step,
         * which is noticeably better than the start-of-step heading when
         * turning. */
        const float half_turn = odom.theta + odom.angular * dt * 0.5f;
        odom.x += odom.linear * cosf(half_turn) * dt;
        odom.y += odom.linear * sinf(half_turn) * dt;
        odom.theta += odom.angular * dt;

        while (odom.theta > PI) odom.theta -= 2.0f * PI;
        while (odom.theta < -PI) odom.theta += 2.0f * PI;
    }
};
