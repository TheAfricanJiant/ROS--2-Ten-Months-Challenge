/*
 * LSM6DSO IMU on Wire1.
 *
 * The register work here is lifted straight from the imu_test project, which
 * confirmed WHO_AM_I reads 0x6C and that 104 Hz / +/-2 g / 250 dps is a
 * sensible starting configuration. Only the unit conversion and the bias
 * estimate are new - the raw counts a ROS 2 sensor_msgs/Imu wants are in
 * m/s^2 and rad/s, not LSB.
 */

#pragma once

#include <Arduino.h>
#include <Wire.h>

#include "config.h"

/* LSM6DSO registers */
#define LSM6DSO_WHO_AM_I 0x0F
#define LSM6DSO_CTRL1_XL 0x10
#define LSM6DSO_CTRL2_G  0x11
#define LSM6DSO_OUTX_L_G 0x22

class Imu {
public:
    bool present = false;

    /* SI units, ready for sensor_msgs/Imu. */
    float ax = 0, ay = 0, az = 0;      // m/s^2
    float gx = 0, gy = 0, gz = 0;      // rad/s

    /* Raw counts, kept for debugging against imu_test's output. */
    int16_t raw_ax = 0, raw_ay = 0, raw_az = 0;
    int16_t raw_gx = 0, raw_gy = 0, raw_gz = 0;

    bool begin() {
        Wire1.setSDA(IMU_SDA_PIN);
        Wire1.setSCL(IMU_SCL_PIN);
        Wire1.begin();
        Wire1.setClock(400000);        // fast mode, as in imu_test
        delay(100);

        uint8_t whoami = 0;
        readRegisters(LSM6DSO_WHO_AM_I, &whoami, 1);
        present = (whoami == IMU_WHO_AM_I_EXPECTED);

        if (present) {
            writeRegister(LSM6DSO_CTRL1_XL, 0x40);   // 104 Hz, +/- 2 g
            writeRegister(LSM6DSO_CTRL2_G, 0x40);    // 104 Hz, 250 dps
            delay(50);
        }
        return present;
    }

    /* Average the gyro while the robot is still, and subtract that from
     * every later reading. Without it the heading drifts steadily even
     * sitting on a desk. Keep the robot still while this runs. */
    void calibrateGyro(uint16_t samples = 200) {
        if (!present) {
            return;
        }
        float sx = 0, sy = 0, sz = 0;
        for (uint16_t i = 0; i < samples; i++) {
            read();
            sx += raw_gx;
            sy += raw_gy;
            sz += raw_gz;
            delay(5);
        }
        _bias_gx = sx / samples;
        _bias_gy = sy / samples;
        _bias_gz = sz / samples;
    }

    void read() {
        if (!present) {
            return;
        }

        uint8_t data[12];
        /* One burst from 0x22: gyro XYZ then accel XYZ, little-endian. */
        readRegisters(LSM6DSO_OUTX_L_G, data, 12);

        raw_gx = (int16_t)(data[1] << 8 | data[0]);
        raw_gy = (int16_t)(data[3] << 8 | data[2]);
        raw_gz = (int16_t)(data[5] << 8 | data[4]);
        raw_ax = (int16_t)(data[7] << 8 | data[6]);
        raw_ay = (int16_t)(data[9] << 8 | data[8]);
        raw_az = (int16_t)(data[11] << 8 | data[10]);

        /* LSB -> SI. Full scale spans +/-32768 counts. */
        const float accel_lsb = (IMU_ACCEL_SCALE_G * 9.80665f) / 32768.0f;
        const float gyro_lsb = (IMU_GYRO_SCALE_DPS * (float)PI / 180.0f) / 32768.0f;

        ax = raw_ax * accel_lsb;
        ay = raw_ay * accel_lsb;
        az = raw_az * accel_lsb;
        gx = (raw_gx - _bias_gx) * gyro_lsb;
        gy = (raw_gy - _bias_gy) * gyro_lsb;
        gz = (raw_gz - _bias_gz) * gyro_lsb;
    }

private:
    float _bias_gx = 0, _bias_gy = 0, _bias_gz = 0;

    static void writeRegister(uint8_t reg, uint8_t value) {
        Wire1.beginTransmission(IMU_ADDR);
        Wire1.write(reg);
        Wire1.write(value);
        Wire1.endTransmission();
    }

    static void readRegisters(uint8_t reg, uint8_t *buffer, uint8_t length) {
        Wire1.beginTransmission(IMU_ADDR);
        Wire1.write(reg);
        Wire1.endTransmission(false);          // repeated start
        Wire1.requestFrom((uint8_t)IMU_ADDR, length);
        for (uint8_t i = 0; i < length && Wire1.available(); i++) {
            buffer[i] = Wire1.read();
        }
    }
};
