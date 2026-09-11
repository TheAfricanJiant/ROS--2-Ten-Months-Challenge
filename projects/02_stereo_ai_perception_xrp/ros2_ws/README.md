# ROS 2 workspace — tracking and perception

[← Back to Project 02](../README.md)

Two packages. `tracker` holds the nodes; `tracker_launch` holds the launch
files that wire them together.

**One camera tracks and drives the robot. The other does image processing.**
Stereo depth comes later — for now the tracking camera estimates range from
apparent size, which is enough to hold a following distance.

```
camera 1  ──►  /image1            ──►  Foxglove
   │           /image1_annotated  ──►  Foxglove
   └──────►  /camera1/detections  ──►  tracker_node  ──►  /cmd_vel  ──►  micro-ROS ──► XRP

camera 2  ──►  /image2            ──►  Foxglove
               /camera2/blob           (colour effect only)
```

---

## Prerequisites & Build

On the Pi:

```bash
# 1. System packages (OpenCV, vision_msgs, foxglove_bridge)
sudo apt update
sudo apt install -y python3-opencv ros-$ROS_DISTRO-vision-msgs ros-$ROS_DISTRO-foxglove-bridge

# 2. Camera driver and SSCMA library
pip install -e ~/ROS--2-Ten-Months-Challenge/test_camera --break-system-packages

# 3. USB permissions for serial ports
sudo chmod 666 /dev/ttyACM*

# 4. Build workspace
cd ~/ROS--2-Ten-Months-Challenge/projects/02_stereo_ai_perception_xrp/ros2_ws
colcon build --symlink-install
source install/setup.bash
```

`--symlink-install` means edits to the Python files take effect without
rebuilding.

---

## Run

### 1. Start micro-ROS Agent (Motor Control)
In a dedicated terminal:
```bash
source /opt/ros/$ROS_DISTRO/setup.bash
source ~/microros_ws/install/setup.bash
ros2 run micro_ros_agent micro_ros_agent serial --dev /dev/ttyACM0 -b 115200
```

### 2. Start Cameras & Foxglove Bridge
In a second terminal:

```bash
cd ~/ROS--2-Ten-Months-Challenge/projects/02_stereo_ai_perception_xrp/ros2_ws
source install/setup.bash

# Single camera + Foxglove bridge (tracker off, /cmd_vel free for manual teleop)
ros2 launch tracker_launch bringup.launch.py use_camera2:=false use_tracker:=false camera1_port:=/dev/ttyACM1

# Both cameras + AI tracker + Foxglove bridge
ros2 launch tracker_launch bringup.launch.py

# Cameras only (nothing moves, good for checking the feeds)
ros2 launch tracker_launch cameras.launch.py
```

**Start with the robot on a stand, or with `enabled:=false`.** That runs the
whole pipeline and publishes zero velocity, so you can watch the numbers
before anything turns a wheel.

```bash
ros2 launch tracker_launch bringup.launch.py enabled:=false
ros2 topic echo /tracker/status
```

### Launch arguments

| Argument | Default | Meaning |
|----------|---------|---------|
| `camera1_port` | `/dev/ttyACM0` | Tracking camera. |
| `camera2_port` | `/dev/ttyACM1` | Effects camera. |
| `camera1_key` | `ir-3.6mm` | Lens of the tracking camera — picks the projection model. |
| `camera1_focal_px` | `0.0` | Measured focal length. `0` uses the nominal one. |
| `target` | `face` | Class to follow; must be one the flashed model knows. |
| `object_height_m` | `0.22` | Real height of the target, for the range estimate. |
| `desired_distance_m` | `0.8` | Following distance. |
| `enabled` | `true` | `false` computes everything but publishes zero. |
| `drive_forward` | `true` | `false` turns to face the target without driving at it. |
| `search_on_lost` | `false` | `true` spins slowly when the target disappears. |
| `effect` | `ir_heat` | Camera 2: `raw`, `ir_heat`, `night_vision`, `enhanced`, `colour`, `edges`. |
| `colour` | `red` | Colour to isolate when `effect:=colour`. |
| `use_camera2` / `use_tracker` / `foxglove` | `true` | Turn pieces off. |
| `width` / `height` | `240` | Frame size. |
| `foxglove_port` | `8765` | Bridge port. |

---

## Viewing in Foxglove from another machine

1. On the Pi, launch with `foxglove:=true` (the default). The bridge binds to
   `0.0.0.0`, so it is reachable from the LAN.
2. On Windows, open **Foxglove Studio** → *Open connection* → **Foxglove
   WebSocket** → `ws://<pi-address>:8765`.
3. Add two **Image** panels, one on `/image1` and one on `/image2`.

Both are `sensor_msgs/CompressedImage` — the boards already produce JPEG, so
re-encoding to raw just to push it over Wi-Fi would waste bandwidth. Foxglove's
Image panel reads CompressedImage directly.

Useful extra panels: **Raw Messages** on `/tracker/status` for a plain-English
line about what the tracker is doing, and `/cmd_vel` to watch the velocity it
is asking for.

If nothing appears, check the Pi's firewall allows 8765 and that
`ros2 topic hz /image1` shows frames arriving on the Pi itself.

---

## Nodes

### `camera_node` — the tracking camera

Streams one board and republishes its frames and detections.

| Topic | Type | Notes |
|-------|------|-------|
| `image` → `/image1` | `sensor_msgs/CompressedImage` | Plain frames. |
| `image_annotated` → `/image1_annotated` | `sensor_msgs/CompressedImage` | Boxes drawn on. |
| `camera_info` | `sensor_msgs/CameraInfo` | Intrinsics; distortion is zero until calibrated. |
| `detections` | `vision_msgs/Detection2DArray` | What the model found. |

Class names come from the model's own metadata, so no labels file is needed.

### `effects_node` — the second camera

Same board type, no model, different job. Pick with `effect:=`:

| Effect | What it does |
|--------|--------------|
| `raw` | Straight through. |
| `ir_heat` | IR intensity through a heat-style palette. |
| `night_vision` | Green phosphor, gamma lift, vignette. |
| `enhanced` | CLAHE — pulls detail out of the shadows. |
| `colour` | Isolates one colour and tracks the largest blob, publishing its centre and area on `blob`. |
| `edges` | Canny edges over a dimmed original. |

> **`ir_heat` is not thermal.** These modules see near-infrared *reflected off
> things* — the IR LEDs are a torch you cannot see. It measures brightness,
> never temperature: a cold white wall under the LEDs reads "hot". Real
> temperature needs a thermal sensor such as an MLX90640.

### `tracker_node` — detections to motion

Subscribes to `/camera1/detections`, publishes `/cmd_vel` and
`/tracker/status`.

**Turning** comes from the horizontal angle to the target, computed with that
camera's own lens model. **Driving** comes from apparent size: one camera
cannot measure depth, so range is inferred from how tall the target looks given
an assumed real height. Wrong assumption means wrong absolute distance — but
consistently wrong, which still holds a gap.

Safety: no detection for `lost_timeout` (0.7 s) and it publishes zero, and it
publishes zero on shutdown.

---

## Why each camera gets its own maths

`lenses.py` carries two projection models, because the two cameras genuinely
obey different ones.

**Rectilinear** — the 3.6 mm module and any ordinary lens. Straight lines stay
straight: `tan(theta) = (u - cx) / f`.

**Equidistant** — the 1.7 mm fisheye. Fisheyes map angle *linearly* onto
radius, which is what lets them fit 90-plus degrees onto a sensor at all:
`theta = r / f`.

This is not pedantry. Measured at the frame edge on this rig:

| | Angle to a target at the frame edge |
|---|---|
| Fisheye, correct (equidistant) model | **61.3°** |
| Fisheye, wrong (rectilinear) model | 46.9° |

A **14.4° error**, and it grows toward the edges — exactly where a target is
when the robot most needs to turn. Near the centre the two agree to within a
degree, which is why the mistake is easy to miss until the robot starts
oversteering.

`normalised_x()` converts either model to the same −1…+1 scale, so the steering
gains behave the same whichever camera is doing the tracking.

---

## Checks before letting it drive

```bash
ros2 topic hz /image1                 # frames arriving?
ros2 topic echo /camera1/detections   # is it seeing the target?
ros2 topic echo /tracker/status       # what does the tracker think?
ros2 topic echo /cmd_vel              # what is it asking for?
```

Then run with `enabled:=false` and watch `/cmd_vel` while you move in front of
the camera. It should turn toward you, and drive forward or back to hold
`desired_distance_m`. Only once that looks right is it worth putting wheels on
the ground.

---

## Status

**Written and syntax-checked, not yet run on the Pi.** The lens models are
covered by a test harness (projection formulas, radial fisheye conversion,
field-of-view figures, steering sign, and the degenerate cases), but the nodes
themselves have not been built with colcon or run against hardware. Expect the
usual first-run friction: missing `vision_msgs`, the wrong `/dev/ttyACM*`
number, or a port held by another process.
