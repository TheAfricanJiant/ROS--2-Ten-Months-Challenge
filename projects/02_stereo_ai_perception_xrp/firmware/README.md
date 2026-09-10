# 🔌 XRP firmware

[← Back to Project 02](../README.md) · [ROS 2 workspace](../ros2_ws/README.md)

Four PlatformIO projects. The three test projects came first and **all pass on
the robot**; the real firmware is built directly out of them.

| Project | Status | What it does |
|---------|--------|--------------|
| [`src/i2c_scanner/`](src/i2c_scanner) | ✅ **working** | Finds every I2C device, on every plausible bus and pin pair. |
| [`src/encoder_test/`](src/encoder_test) | ✅ **working** | Live drive control from the keyboard, encoder counts for all four motors. |
| [`src/imu_test/`](src/imu_test) | ✅ **working** | Reads the LSM6DSO accelerometer and gyro. |
| [`src/xrp_firmware/`](src/xrp_firmware) | ⚙️ **compiles, not yet run on the robot** | micro-ROS bridge: `/cmd_vel` in, IMU/odom/encoders out. |

---

## Contents

- [Flashing an XRP board](#flashing-an-xrp-board)  ← no build needed
- [Building it yourself](#building-it-yourself)
- [What the tests established](#what-the-tests-established)
- [The real firmware](#the-real-firmware)
- [ROS 2 interface](#ros-2-interface)
- [Bringing it up](#bringing-it-up)
- [Troubleshooting](#troubleshooting)
- [Status](#status)

---

## Flashing an XRP board

**You do not need to build anything.** `firmware.uf2` is committed, so the
normal path is drag-and-drop:

1. **Unplug** the USB cable.
2. **Hold the BOOT button down** and keep holding it.
3. **Plug the cable back in**, then release BOOT.
4. A disk appears - `RPI-RP2` (RP2040) or `RP2350` (RP2350B).
5. Copy [`src/xrp_firmware/firmware.uf2`](src/xrp_firmware) onto that disk.
6. **The disk disappears.** That is the success signal - the board reset itself
   and is running the firmware.

> If the disk never appears, BOOT was released too early. It has to be held
> down *as the cable goes in*.

Watch the output with `pio device monitor`, or any serial terminal at 115200.

The three test projects work the same way; build them with `pio run` (they
build natively on Windows) and copy their `.pio/build/pico/firmware.uf2`.

---

## Building it yourself

Only needed if you change the firmware. Full instructions are in
**[`src/xrp_firmware/build process.md`](src/xrp_firmware/build%20process.md)**.

### Windows needs WSL

`micro_ros_platformio` compiles the micro-ROS library from source before it
touches your code, using shell scripts that need a POSIX shell. A native
Windows build gets as far as:

```
Building micro-ROS dev dependencies
Build dev micro-ROS environment failed:
 '.' is not recognized as an internal or external command
```

That is the Unix `.` (source) builtin hitting `cmd.exe`. There is no Windows
support in that library, so on Windows the firmware builds **inside WSL**:

```bash
wsl --install          # PowerShell as Administrator, once
wsl
cd ~ && git clone <repo> && cd <repo>/projects/02_stereo_ai_perception_xrp/firmware/src/xrp_firmware
python3 -m pip install --user platformio
pio run
```

Open it from WSL with `code .` so VS Code attaches to the WSL environment -
**not** as a normal Windows PlatformIO project.

> **How the committed `.uf2` was produced:** built on a Linux machine and
> pushed to GitHub, rather than fighting the Windows toolchain. That was
> simply the quickest route, and it is why you can flash without building.

### Linux / macOS

Native, no workaround:

```bash
cd projects/02_stereo_ai_perception_xrp/firmware/src/xrp_firmware
pio run
```

| OS | Build environment |
|----|-------------------|
| Windows | **WSL / Ubuntu** |
| Linux | native |
| macOS | native |

`pio run -t clean` then `pio run` if a build goes strange.

---

## What the tests established

These are not assumptions. Each came out of running the test projects on the
robot, and each is now baked into the real firmware.

### I2C — where the bus actually is

`i2c_scanner` tries six bus/pin combinations rather than trusting a pinout:

```cpp
static const BusOption BUS_OPTIONS[] = {
    {"Wire1  GP18/GP19  (XRP Beta: Qwiic + IMU)", &Wire1, 18, 19},
    {"Wire   GP4/GP5    (XRP Controller: Qwiic0)", &Wire, 4, 5},
    {"Wire   GP0/GP1    (Pico default)", &Wire, 0, 1},
    ...
};
```

**Result:** the LSM6DSO answers at `0x6B` on **Wire1, GP18/GP19**.

### IMU — the register sequence

`imu_test` confirmed `WHO_AM_I` returns `0x6C`, then configured it:

```cpp
Wire1.setSDA(18);
Wire1.setSCL(19);
Wire1.begin();
Wire1.setClock(400000);                  // fast mode

writeRegister(LSM6DSO_CTRL1_XL, 0x40);   // accel 104 Hz, +/- 2 g
writeRegister(LSM6DSO_CTRL2_G,  0x40);   // gyro  104 Hz, 250 dps
```

One burst read from `0x22` gives gyro then accel, little-endian:

```cpp
readRegisters(LSM6DSO_OUTX_L_G, data, 12);
int16_t gyroX  = (int16_t)(data[1] << 8 | data[0]);
...
int16_t accelZ = (int16_t)(data[11] << 8 | data[10]);
```

### Motors — two things the documentation got wrong

`encoder_test` found the published pinout does not match the board:

```cpp
static const MotorPins MOTORS[] = {
    {"Motor L", 4,  5,  6,  7,  false},
    {"Motor R", 13, 12, 14, 15, false},   // encoder A/B SWAPPED
    {"Motor 3", 0,  1,  2,  3,  false},
    {"Motor 4", 8,  9,  10, 11, true},    // drives BACKWARDS
};

static const size_t LEFT_MOTOR  = 1;   // "Motor R" is the physical left wheel
static const size_t RIGHT_MOTOR = 3;   // "Motor 4" is the physical right wheel
```

Motor R's encoder channels are reversed relative to the docs, and Motor 4
turns the wrong way — so it carries a `reversed` flag that flips both its
drive direction and its encoder sign:

```cpp
long step = (a == b) ? 1 : -1;
if (MOTORS[INDEX].reversed) {
    step = -step;
}
counts[INDEX] += step;
```

The driven wheels are **Motor R and Motor 4**, not "Motor L and Motor R" as
the silkscreen suggests.

### Encoders — both edges of channel A

```cpp
attachInterrupt(digitalPinToInterrupt(MOTORS[i].encoderA), HANDLERS[i], CHANGE);
```

`CHANGE` rather than `RISING` doubles the resolution. `COUNTS_PER_REV` is
`585` — measure yours by turning a wheel exactly once; odometry scales
directly with it.

### Live speed control

`encoder_test` re-applies the current state whenever the speed changes, so
`+`/`-` take effect while moving instead of at the next command. The real
firmware keeps that idea — `cmd_vel` updates take effect immediately.

```cpp
current_pwm = min(255, current_pwm + 25);
applyCurrentState();   // takes effect live
```

---

## The real firmware

`src/xrp_firmware/` is split so the tested parts stay recognisable:

| File | From | What it holds |
|------|------|---------------|
| `config.h` | all three tests | Pins, board select, tunable defaults. |
| `drivetrain.h` | `encoder_test` | Motors, encoder ISRs, wheel speeds, odometry. |
| `imu.h` | `imu_test` | LSM6DSO driver, plus SI conversion and gyro bias. |
| `main.cpp` | new | micro-ROS publishers, subscribers, parameters, control loop. |

Two things are genuinely new on top of the tests:

**Gyro bias.** Averaged at startup while the robot is still, then subtracted.
Without it the heading drifts noticeably sitting on a desk.

**Odometry with midpoint integration** — the heading halfway through each step
rather than at the start, which is meaningfully better while turning:

```cpp
const float half_turn = odom.theta + odom.angular * dt * 0.5f;
odom.x += odom.linear * cosf(half_turn) * dt;
odom.y += odom.linear * sinf(half_turn) * dt;
```

---

## ROS 2 interface

### Published

| Topic | Type | Rate | Notes |
|-------|------|------|-------|
| `/xrp/imu` | `sensor_msgs/Imu` | 50 Hz | Accel m/s², gyro rad/s. No orientation — covariance `[0] = -1` says so per REP-145. |
| `/xrp/odom` | `nav_msgs/Odometry` | 25 Hz | Pose and twist from the wheels. |
| `/xrp/joint_states` | `sensor_msgs/JointState` | 25 Hz | Wheel angle and speed. |
| `/xrp/encoders` | `std_msgs/Int32MultiArray` | 25 Hz | Raw counts, all four motors. |

### Subscribed

| Topic | Type | Notes |
|-------|------|-------|
| `/cmd_vel` | `geometry_msgs/Twist` | The normal way to drive. `linear.x`, `angular.z`. |
| `/xrp/motor_cmd` | `std_msgs/Float32MultiArray` | Direct wheel duty, `[left, right]`, −1…+1. Bypasses the kinematics. |
| `/xrp/enable` | `std_msgs/Bool` | `false` cuts the motors and holds them off. |
| `/xrp/reset_odom` | `std_msgs/Bool` | Zeroes pose and encoder counts. |

### Parameters — everything worth tuning, live

```bash
ros2 param list /xrp_firmware
ros2 param set /xrp_firmware max_linear_mps 0.35
ros2 param set /xrp_firmware closed_loop true
```

| Parameter | Default | What it changes |
|-----------|---------|-----------------|
| `max_linear_mps` | `0.60` | Speed ceiling; `cmd_vel` is clamped to it. |
| `max_angular_rps` | `4.0` | Turn-rate ceiling. |
| `wheel_radius_m` | `0.030` | Odometry scale. Measure it. |
| `wheel_separation_m` | `0.155` | Turn-rate scale. |
| `counts_per_rev` | `585` | Encoder scale. |
| `min_pwm` | `50` | Friction floor — below this the motor buzzes without turning. |
| `max_pwm` | `255` | Upper PWM limit. |
| `cmd_timeout_ms` | `500` | Stop if `/cmd_vel` goes quiet. |
| `closed_loop` | `false` | Use the encoders to hit the commanded speed. |
| `kp` / `ki` | `120` / `400` | Velocity gains, used when `closed_loop`. |
| `invert_left` / `invert_right` | `false` | Flip a wheel without reflashing. |
| `enabled` | `true` | Software kill switch. |

**Start open-loop.** Get the geometry right first; closed-loop on a
badly-scaled `counts_per_rev` just chases a wrong number faster.

If the board runs out of RAM, set `USE_PARAMETER_SERVER 0` in `config.h`. That
drops three services and leaves every topic working.

---

## Bringing it up

On the Pi:

```bash
sudo apt install ros-$ROS_DISTRO-micro-ros-agent   # or build from source
ros2 run micro_ros_agent micro_ros_agent serial --dev /dev/ttyACM0 -b 115200
```

Then, from another terminal:

```bash
ros2 topic list                  # /xrp/imu, /xrp/odom, /cmd_vel ...
ros2 topic echo /xrp/imu
ros2 topic echo /xrp/odom
```

**Put the robot on a stand before the first `cmd_vel`.**

```bash
ros2 topic pub --once /cmd_vel geometry_msgs/msg/Twist \
  '{linear: {x: 0.1}, angular: {z: 0.0}}'
```

Both wheels should turn forward and `/xrp/odom` should count up. If one turns
the wrong way, flip it live rather than reflashing:

```bash
ros2 param set /xrp_firmware invert_right true
```

Kill switch, at any time:

```bash
ros2 topic pub --once /xrp/enable std_msgs/msg/Bool '{data: false}'
```

The firmware also stops itself if `/cmd_vel` goes quiet for `cmd_timeout_ms`,
so a crashed controller does not leave the robot driving.

---

## Troubleshooting

### `'.' is not recognized as an internal or external command`

A native Windows build. See [Building it yourself](#building-it-yourself) -
use WSL, or just flash the committed `.uf2`.

### `Filename too long` during the micro-ROS download

Windows' 260-character path limit, hit by git while it clones micro-ROS's
dependencies. Inside WSL this cannot happen. If you are on native Windows for
some other reason:

```powershell
git config --global core.longpaths true
```

### `board_microros_distro` mismatch

The firmware and the agent must be built against the same ROS 2 distro. Check
with `echo $ROS_DISTRO` on the Pi and set `board_microros_distro` to match. A
mismatch usually shows up as the agent connecting but no topics appearing.

### No `RPI-RP2` drive appears

BOOT was released too early. It has to be held down *as the cable goes in*.
See [Flashing an XRP board](#flashing-an-xrp-board).

### Agent connects but nothing publishes

Check the transport matches: `board_microros_transport = serial` in
`platformio.ini`, and the agent started with `serial --dev /dev/ttyACM0`.
Confirm the port with `ls /dev/ttyACM*` on the Pi.

### Out of memory, or a hard fault on startup

RP2040 has 256 KB of RAM and micro-ROS uses a fair share. Set
`USE_PARAMETER_SERVER 0` in `config.h` - that drops three services and leaves
every topic working.

---

## Status

**The three test projects work on the robot** — I2C scan, encoder counting and
drive control, and IMU streaming all verified by running them.

**The micro-ROS firmware compiles**, built on Linux; `firmware.uf2` is
committed so it can be flashed without a toolchain. Its drivers are the tested
ones, so the remaining risk is the ROS 2 plumbing rather than the hardware.

**Not yet run on the robot.** Topic behaviour, odometry scaling and the
closed-loop gains are all unverified against hardware.

Not yet done: closed-loop tuning (`kp`/`ki` are starting values, not measured
ones), and `wheel_radius_m` / `wheel_separation_m` still need measuring on the
actual chassis before odometry means anything absolute.
