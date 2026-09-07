# 📷 test_camera — Grove Vision AI V2 feed on Windows

[← Back to repository index](../README.md)

A small Windows tool that pulls JPEG frames off a **Grove Vision AI V2** over
USB serial and displays them with OpenCV — the "Stage 2" fallback for when you
want a normal webcam-style feed instead of only detection results.

This is the bench tooling for
[Project 02, Objective 2](../projects/02_stereo_ai_perception_xrp/README.md)
(*test both cameras and both Grove AI modules, confirm AI works independently
before ROS 2*). No ROS 2 involved — it runs on your laptop.

---

## Why serial rather than a webcam driver

The Vision AI V2 is **not a UVC webcam**. It does not enumerate as a video
device, so OpenCV's `cv2.VideoCapture(0)` will never see it. What it does
expose is sscma-micro's **SSCMA AT protocol** over a USB CDC serial port, which
can hand you the last captured frame as a **base64-encoded JPEG**. This tool
requests those frames, decodes them, and paints them into a window — which is
functionally the webcam feed you wanted.

---

## Install

```powershell
cd test_camera
py -m venv .venv
.venv\Scripts\Activate.ps1
pip install -r requirements.txt
```

If PowerShell blocks the activate script:

```powershell
Set-ExecutionPolicy -Scope Process -ExecutionPolicy Bypass
```

---

## Use

```powershell
python stream_camera.py --list-ports     # see what is plugged in
python stream_camera.py                  # plain camera feed
python stream_camera.py --detect         # feed + the flashed model's boxes
python stream_camera.py --port COM5 --scale 3
```

The port is auto-detected when a single Seeed device is present; pass `--port`
when several are (which is exactly what happens once **both** modules are
connected for the stereo rig).

| Key | Action |
|-----|--------|
| `q` / `Esc` | quit |
| `s` | save a PNG into `snapshots/` |
| `d` | toggle the box + HUD overlay |

Naming the classes makes `--detect` readable — one label per line:

```powershell
python stream_camera.py --detect --labels labels.txt
```

---

## How it works

| Command | Purpose |
|---------|---------|
| `AT+SAMPLE=-1` | stream frames continuously, **no model** — the plain camera feed |
| `AT+INVOKE=-1,0,0` | stream frames **with** inference; `0,0` = include the JPEG, report every frame |
| `AT+BREAK` | stop streaming (sent on startup and on exit) |
| `AT+ID?` / `AT+NAME?` / `AT+VER?` | identity, printed once so you can confirm the right board |

Replies are one JSON object per line, wrapped in `\r` … `\n`:

```json
{"type": 1, "name": "SAMPLE", "code": 0,
 "data": {"count": 12, "image": "<base64 jpeg>"}}
```

`type` is `0` for a direct response to a command, `1` for a streamed event,
`2` for a log line. Base64 never contains a newline, so splitting the byte
stream on `\n` is enough framing — no brace-matching across a 60 kB payload.

```
test_camera/
├── stream_camera.py     # CLI entry point
└── sscma/
    ├── protocol.py      # framing + JSON reply parsing
    ├── client.py        # serial transport, AT commands, frame decoding
    └── viewer.py        # OpenCV window, overlays, snapshots
```

`sscma/` has no OpenCV dependency at import time (it is imported lazily inside
`decode_jpeg`), so the protocol layer can be reused later from a ROS 2 node
that publishes `sensor_msgs/Image` instead of calling `cv2.imshow`.

---

## Troubleshooting

| Symptom | Cause / fix |
|---------|-------------|
| "No serial ports found" | Charge-only USB-C cable — very common. Swap for a data cable. |
| "Could not open COMx: Access is denied" | Another program holds the port. Close the SenseCraft web tool, Arduino Serial Monitor, or a previous run. |
| Port opens but no frames | Wrong baud rate. Default is **921600**; try `--baud 115200`. |
| `AT+INVOKE` rejected | No model flashed. Use `--detect` only after loading one, or drop it for a plain feed. |
| Boxes sit down-and-right of the object | Firmware reports corner-origin boxes: add `--box-format corner`. |
| Window opens then freezes | Board reset mid-stream. Unplug, replug, rerun. |

---

## Verification status

The protocol and decode path are covered by a fake-device harness — reply
framing (including byte-at-a-time arrival), junk tolerance, JPEG decoding,
centre→corner box conversion, out-of-bounds clamping, malformed boxes, command
rejection and read timeouts.

**Not yet run against real hardware.** The AT command set and reply shape
follow sscma-micro's documented protocol, but the exact field names your
firmware build emits may differ. If frames do not arrive, run with a serial
monitor open first and check what the board actually replies to `AT+SAMPLE=1`.
