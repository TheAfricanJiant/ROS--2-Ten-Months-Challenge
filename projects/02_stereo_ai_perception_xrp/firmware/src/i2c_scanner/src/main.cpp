/*
 * Phase 1 - I2C scanner for the XRP controller board.
 *
 * Finds every I2C device that responds, and names the ones we expect. It
 * scans BOTH I2C peripherals across several pin pairs rather than trusting a
 * single wiring guess, because the two XRP board revisions put the bus in
 * different places:
 *
 *   XRP Beta (Pico W / RP2040)   Qwiic + IMU on GP18 / GP19   -> Wire1
 *   XRP Controller (RP2350B)     Qwiic0 GP4 / GP5, Qwiic1 GP38 / GP39
 *
 * Rather than ask you which board you have, it tries them all and reports
 * what actually answered. That is the whole point of a scanner.
 *
 * No micro-ROS, no ROS 2, no network. Serial output only.
 */

#include <Arduino.h>
#include <Wire.h>

struct BusOption {
    const char *label;
    TwoWire *wire;
    uint8_t sda;
    uint8_t scl;
};

/* Every plausible place the bus could be, on either board revision. A pin
 * pair that does not exist on this chip simply finds nothing. */
static const BusOption BUS_OPTIONS[] = {
    {"Wire1  GP18/GP19  (XRP Beta: Qwiic + IMU)", &Wire1, 18, 19},
    {"Wire   GP4/GP5    (XRP Controller: Qwiic0)", &Wire, 4, 5},
    {"Wire   GP0/GP1    (Pico default)", &Wire, 0, 1},
    {"Wire1  GP6/GP7", &Wire1, 6, 7},
    {"Wire   GP20/GP21", &Wire, 20, 21},
    {"Wire1  GP26/GP27", &Wire1, 26, 27},
};
static const size_t BUS_COUNT = sizeof(BUS_OPTIONS) / sizeof(BUS_OPTIONS[0]);

/* Addresses worth calling out by name when we meet them. */
struct KnownDevice {
    uint8_t address;
    const char *name;
};

static const KnownDevice KNOWN[] = {
    {0x0C, "AK8963 magnetometer"},
    {0x18, "LIS3DH / LSM303 accelerometer"},
    {0x19, "LIS3DH / LSM303 accelerometer (alt)"},
    {0x1E, "HMC5883L / LSM303 magnetometer"},
    {0x29, "VL53L0X / VL53L1X range sensor"},
    {0x3C, "SSD1306 OLED"},
    {0x3D, "SSD1306 OLED (alt)"},
    {0x40, "INA219 / HTU21D"},
    {0x48, "ADS1115 / TMP102"},
    {0x50, "EEPROM"},
    {0x53, "ADXL345 accelerometer"},
    {0x57, "MAX30105 / EEPROM"},
    {0x62, "SCD4x CO2 sensor"},
    {0x68, "MPU6050 / DS3231 RTC"},
    {0x69, "MPU6050 (alt)"},
    {0x6A, "LSM6DSO IMU  <- XRP on-board IMU, ADR jumper closed"},
    {0x6B, "LSM6DSO IMU  <- XRP on-board IMU, default address"},
    {0x76, "BMP280 / BME280"},
    {0x77, "BMP280 / BME280 (alt)"},
};
static const size_t KNOWN_COUNT = sizeof(KNOWN) / sizeof(KNOWN[0]);

static const char *describe(uint8_t address) {
    for (size_t i = 0; i < KNOWN_COUNT; i++) {
        if (KNOWN[i].address == address) {
            return KNOWN[i].name;
        }
    }
    return "unknown device";
}

/* Scan one bus. Returns how many devices answered. */
static int scanBus(const BusOption &option) {
    Serial.printf("\n%s\n", option.label);

    option.wire->setSDA(option.sda);
    option.wire->setSCL(option.scl);
    option.wire->begin();
    option.wire->setClock(100000);   // 100 kHz: slow, and tolerant of long cables

    int found = 0;
    /* 0x00-0x07 and 0x78-0x7F are reserved, so only 0x08-0x77 is worth trying. */
    for (uint8_t address = 0x08; address <= 0x77; address++) {
        option.wire->beginTransmission(address);
        uint8_t error = option.wire->endTransmission();

        if (error == 0) {
            Serial.printf("    0x%02X  %s\n", address, describe(address));
            found++;
        } else if (error == 4) {
            Serial.printf("    0x%02X  bus error while probing\n", address);
        }
    }

    if (found == 0) {
        Serial.println("    (nothing responded)");
    }

    option.wire->end();
    return found;
}

void setup() {
    Serial.begin(115200);

    /* Wait for the USB serial console, but not forever - the board should
     * still run when powered from a battery with nothing attached. */
    unsigned long start = millis();
    while (!Serial && (millis() - start) < 4000) {
        delay(10);
    }

    Serial.println();
    Serial.println("=====================================================");
    Serial.println(" XRP I2C scanner - phase 1");
    Serial.println("=====================================================");
    Serial.println("Scanning every plausible bus and pin pair. Whatever");
    Serial.println("answers tells you how this board is actually wired.");
}

void loop() {
    int total = 0;
    int busesWithDevices = 0;

    for (size_t i = 0; i < BUS_COUNT; i++) {
        int found = scanBus(BUS_OPTIONS[i]);
        total += found;
        if (found > 0) {
            busesWithDevices++;
        }
        delay(50);
    }

    Serial.println("\n-----------------------------------------------------");
    Serial.printf(" %d device(s) across %d bus configuration(s)\n",
                  total, busesWithDevices);

    if (total == 0) {
        Serial.println();
        Serial.println(" Nothing found anywhere. Usually one of:");
        Serial.println("   - the board is not actually powered (USB alone may");
        Serial.println("     not power the motor/sensor rail - check the");
        Serial.println("     battery switch)");
        Serial.println("   - nothing is plugged into the Qwiic connector, and");
        Serial.println("     this board revision has no on-board IMU");
        Serial.println("   - SDA and SCL are swapped");
        Serial.println("   - missing pull-up resistors on a hand-wired device");
    } else if (busesWithDevices > 1) {
        Serial.println();
        Serial.println(" The same device can appear on more than one entry");
        Serial.println(" above when two configurations share a pin. Trust the");
        Serial.println(" one matching your board revision.");
    }

    Serial.println("\n Rescanning in 5 seconds. Plug something in to watch it");
    Serial.println(" appear.");
    Serial.println("-----------------------------------------------------");
    delay(5000);
}
