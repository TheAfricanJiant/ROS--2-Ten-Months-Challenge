/*
 * Phase 2 - LSM6DSO IMU Reader for XRP Board
 * Reads raw accelerometer and gyroscope data from the built-in IMU at 0x6B on Wire1 (GP18/GP19).
 */

#include <Arduino.h>
#include <Wire.h>

/* LSM6DSO I2C Address (default is 0x6B) */
#define IMU_ADDR 0x6B

/* LSM6DSO Registers */
#define LSM6DSO_WHO_AM_I   0x0F
#define LSM6DSO_CTRL1_XL   0x10
#define LSM6DSO_CTRL2_G    0x11
#define LSM6DSO_OUTX_L_G   0x22

void writeRegister(uint8_t reg, uint8_t val) {
    Wire1.beginTransmission(IMU_ADDR);
    Wire1.write(reg);
    Wire1.write(val);
    Wire1.endTransmission();
}

void readRegisters(uint8_t reg, uint8_t *buffer, uint8_t length) {
    Wire1.beginTransmission(IMU_ADDR);
    Wire1.write(reg);
    Wire1.endTransmission(false); // Restart
    Wire1.requestFrom((uint8_t)IMU_ADDR, length);
    
    for (uint8_t i = 0; i < length && Wire1.available(); i++) {
        buffer[i] = Wire1.read();
    }
}

void setup() {
    Serial.begin(115200);
    while (!Serial && millis() < 4000) { delay(10); }

    // Initialize Wire1 on GP18 (SDA) and GP19 (SCL)
    Wire1.setSDA(18);
    Wire1.setSCL(19);
    Wire1.begin();
    Wire1.setClock(400000); // 400kHz Fast Mode

    delay(100);

    // Check WHO_AM_I register (should return 0x6C for LSM6DSO)
    uint8_t whoami = 0;
    readRegisters(LSM6DSO_WHO_AM_I, &whoami, 1);

    Serial.println("\n=====================================================");
    Serial.println(" XRP IMU INITIALIZATION");
    Serial.println("=====================================================");
    Serial.printf(" IMU WHO_AM_I Response: 0x%02X ", whoami);
    
    if (whoami == 0x6C) {
        Serial.println("(Verified: LSM6DSO found!)");
    } else {
        Serial.println("(Warning: Unexpected ID, check wiring/power)");
    }

    // Configure IMU: Accel = 104 Hz, 2g scale; Gyro = 104 Hz, 250 dps
    writeRegister(LSM6DSO_CTRL1_XL, 0x40); // ODR 104 Hz, +/- 2g
    writeRegister(LSM6DSO_CTRL2_G,  0x40); // ODR 104 Hz, 250 dps
    
    Serial.println(" IMU configured. Streaming live sensor data...\n");
}

void loop() {
    uint8_t data[12];
    // Read 12 bytes starting from Gyro Xout Low (0x22 through Accel Zout High)
    readRegisters(LSM6DSO_OUTX_L_G, data, 12);

    int16_t gyroX  = (int16_t)(data[1] << 8 | data[0]);
    int16_t gyroY  = (int16_t)(data[3] << 8 | data[2]);
    int16_t gyroZ  = (int16_t)(data[5] << 8 | data[4]);
    
    int16_t accelX = (int16_t)(data[7] << 8 | data[6]);
    int16_t accelY = (int16_t)(data[9] << 8 | data[8]);
    int16_t accelZ = (int16_t)(data[11] << 8 | data[10]);

    Serial.printf("Accel [X:%+6d  Y:%+6d  Z:%+6d]   |   Gyro [X:%+6d  Y:%+6d  Z:%+6d]\n",
                  accelX, accelY, accelZ, gyroX, gyroY, gyroZ);

    delay(200);
}