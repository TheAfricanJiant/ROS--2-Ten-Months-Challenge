# 👁️ Project 02 — Stereo AI Perception Robot (XRP)

[← Back to repository index](../../README.md)

A Raspberry Pi 5 running ROS 2 with **two night-vision cameras** for stereo depth and **two Grove AI V2 modules** for on-module object detection, driving an **XRP** robot through **micro-ROS on an RP2040/Pico**.

The split is deliberate: AI, stereo, and decision-making stay on the Pi 5; low-level motor control stays on the Pico.

```
[ Cam L ] [ Cam R ]        [ Grove AI V2 ×2 ]
     \        /                    |
      \      /                     |
   ┌───────────────────────────────────────┐
   │  Raspberry Pi 5 — ROS 2 (Jazzy)       │
   │  stereo depth · detections · logic    │
   └───────────────┬───────────────────────┘
                   │ /cmd_vel  (micro-ROS)
           ┌───────▼────────┐
           │  Pico / RP2040 │  motor control
           └───────┬────────┘
                   │
              ┌────▼────┐
              │   XRP   │
              └─────────┘
```

---

## 📁 Contents

```
02_stereo_ai_perception_xrp/
├── firmware/        # micro-ROS firmware for the RP2040/Pico on the XRP
├── ros2_ws/src/     # ROS 2 perception, stereo, and control packages
├── calibration/     # Camera intrinsics and stereo extrinsics
├── scripts/         # Standalone bring-up and test scripts (pre-ROS)
└── docs/            # Wiring diagrams, baseline measurements, notes
```

---

## 🎯 Project objectives

### Objective 1 — Gather & identify components

- Raspberry Pi 5
- 2× night-vision Raspberry Pi cameras
- 2× Grove AI V2 modules
- Pico/RP2040 on XRP
- Wiring, power, mounting hardware

### Objective 2 — Test cameras + Grove AI

- Test both cameras in **day and night/IR**
- Test both Grove AI modules
- Run object detection
- Get bounding boxes + confidence
- Confirm AI works independently before ROS 2

### Objective 3 — Build stereo vision system

- Rigidly mount the two cameras
- Measure camera baseline
- Get simultaneous left/right images
- Later perform proper stereo calibration
- Generate disparity/depth

### Objective 4 — Integrate everything into ROS 2

- Pi 5 receives both camera streams
- Grove AI outputs become ROS 2 data
- Create perception topics
- Combine AI detection + stereo distance

### Objective 5 — Integrate XRP with micro-ROS

- ROS 2 → micro-ROS → Pico → XRP
- Use `/cmd_vel` for movement
- Keep low-level motor control on the Pico
- Keep AI/stereo/decision-making on Pi 5

### Objective 6 — Test robot behaviors

Start simple:

- Color following
- Object following
- Person following
- Object distance measurement
- Stereo obstacle detection/avoidance
- Moving-object tracking
- Day vs night detection

### Objective 7 — Prepare for SLAM

- Keep ROS 2 architecture modular
- Establish camera calibration + TF properly
- Add stereo depth
- Later integrate SLAM
- Eventually add autonomous navigation

---

## ⏱️ Immediate 2h30 goal

> **Two cameras + AI → stereo/perception → ROS 2 → micro-ROS → XRP → follow a colored object.**

Everything beyond that is the next phase.

---

## ✅ Progress

| Objective | Status |
|-----------|--------|
| 1 — Components gathered | ⬜ |
| 2 — Cameras + Grove AI tested | ⬜ |
| 3 — Stereo rig built & calibrated | ⬜ |
| 4 — ROS 2 perception topics | ⬜ |
| 5 — micro-ROS → XRP `/cmd_vel` | ⬜ |
| 6 — Follow behaviors | ⬜ |
| 7 — SLAM-ready architecture | ⬜ |

---

## 📜 Prerequisites

- Raspberry Pi 5 with ROS 2 Jazzy
- `micro_ros_agent` (see the [micro-ROS guide](https://micro.ros.org/docs/tutorials/core/first_application_linux/))
- `libcamera` / `rpicam-apps` for the Pi cameras
- OpenCV (stereo calibration and disparity)
- PlatformIO or the Pico SDK for the RP2040 firmware

---

## 📖 Related

- [Project 01 — Teleoperation with micro-ROS](../01_teleop_microros/README.md) — the micro-ROS transport groundwork this project builds on
- [ICC kinematics](../../docs/resources/icckinematics.pdf)
