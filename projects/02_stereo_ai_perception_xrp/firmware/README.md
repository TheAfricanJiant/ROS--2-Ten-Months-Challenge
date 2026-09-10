# XRP firmware

[← Back to Project 02](../README.md)

Three PlatformIO projects. **Phase 1 is the two test projects** — prove the
hardware before writing anything that depends on it. `xrp_firmware` is the
phase 2 home for micro-ROS and is still the PlatformIO template.

| Project | Phase | What it does |
|---------|-------|--------------|
| [`src/i2c_scanner/`](src/i2c_scanner) | **1** | Finds every I2C device, on every plausible bus and pin pair. |
| [`src/encoder_test/`](src/encoder_test) | **1** | Counts encoder edges on all four motors; optionally drives them to check direction. |
| [`src/xrp_firmware/`](src/xrp_firmware) | 2 | micro-ROS `/cmd_vel` subscriber. Not started. |

Each is a separate PlatformIO project on purpose: a test that shares build
flags with the firmware is not really testing the hardware.

---

## Which board do you have?

The two revisions are wired **completely differently**, and the GPIO numbers do
not overlap:

| | XRP Beta | XRP Controller |
|---|---|---|
| MCU | Raspberry Pi Pico W (RP2040) | RP2350B |
| I2C / Qwiic | SDA **GP18**, SCL **GP19** | Qwiic0 GP4/GP5, Qwiic1 GP38/GP39 |
| IMU | LSM6DSO @ `0x6B` (`0x6A` with the jumper) | LSM6DSO @ `0x6B` |
| Motor L | enc GP4/GP5, phase GP6, enable GP7 | enc GP30/GP31, phase GP34, enable GP35 |
| Motor R | enc GP12/GP13, phase GP14, enable GP15 | enc GP24/GP25, phase GP32, enable GP33 |
| PlatformIO `board` | `pico` | `rpipico2` |

GPIO numbers above 29 do not exist on RP2040, so if you see `GP34` anywhere in
a pinout you are reading the RP2350B page.

Run the I2C scanner first — it tells you which one you have without you
having to decide.

---

## Both projects use the earlephilhower core

```ini
platform = https://github.com/maxgerhardt/platform-raspberrypi.git
board = pico
framework = arduino
board_build.core = earlephilhower
```

Not the default mbed core, which lacks `Wire.setSDA()`/`setSCL()`, the second
`Wire1` bus, and `Serial.printf()` — all of which these sketches use.
PlatformIO downloads it on first build.

---

## 1. I2C scanner

```bash
cd src/i2c_scanner
pio run --target upload
pio device monitor
```

Scans **six bus/pin combinations** across both I2C peripherals and reports
what answers, naming the addresses it recognises:

```
Wire1  GP18/GP19  (XRP Beta: Qwiic + IMU)
    0x6B  LSM6DSO IMU  <- XRP on-board IMU, default address
```

It rescans every 5 seconds, so you can plug a Qwiic device in and watch it
appear. That is the quickest way to confirm a sensor is alive and correctly
addressed before any driver code exists.

Nothing found anywhere usually means: the board is not really powered (USB
alone may not power the sensor rail — check the battery switch), SDA and SCL
are swapped, or a hand-wired device is missing its pull-ups.

## 2. Encoder test

```bash
cd src/encoder_test
```

**Set `XRP_BOARD` in `src/main.cpp` first** — `1` for the Beta, `2` for the
Controller. Then:

```bash
pio run --target upload
pio device monitor
```

Prints live counts for all four motors:

```
Motor L    +1170 (2.00 rev, 2340 edges)   Motor R     -585 (-1.00 rev, 1170 edges)
```

Turn a wheel by hand and watch the numbers move. Forward should count **up**.

- `r` resets the counts
- `d` runs the drive test

The drive test runs each motor briefly in both directions and checks the
encoder followed:

```
Motor L  forward ->  +412 counts   ok
Motor L  reverse ->  -398 counts   ok
Motor R  forward ->  -405 counts   WRONG SIGN - swap the encoder A/B pins
```

**Put the robot on a stand first** — the wheels turn. It is off by default
(`RUN_DRIVE_TEST 0`); press `d` when you are ready.

Both edges of channel A are counted, so resolution is double what a
rising-edge-only counter gives. `COUNTS_PER_REV` is a starting figure — turn a
wheel exactly once by hand, read the count, and correct it. Odometry later
depends on that number being right.

---

## Phase 2 — micro-ROS

Not started. `src/xrp_firmware/` is still the PlatformIO template. It will
subscribe to `/cmd_vel` and drive the motors, with the encoders feeding
odometry back.

The split stays deliberate: motor control on the Pico, perception and
decisions on the Pi 5.

---

## Status

**Written, not yet compiled or flashed.** The pin tables come from the
SparkFun XRP documentation rather than from a board on a bench. The I2C
scanner is written to be robust to that — it tries every plausible wiring
rather than trusting one — but the encoder test needs `XRP_BOARD` set
correctly, and the drive test's direction check is only as good as the
pinout it was given.
