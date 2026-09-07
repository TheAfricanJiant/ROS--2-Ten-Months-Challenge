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
python diagnose.py                       # what firmware is each board running?
python stream_camera.py --info           # identity, cameras, model slots
python stream_camera.py                  # plain camera feed
python stream_camera.py --detect         # feed + the flashed model's boxes
python stream_camera.py --port COM4 --resolution 2 --scale 2
```

Auto-detection **asks each port who it is** and picks the board that answers,
so a second board running unrelated firmware cannot be selected by accident.
With two healthy modules connected it will refuse to guess and list both with
their `SER=` values, which are stable per board — use those to pin left and
right for the stereo rig.

### Resolution

`--info` lists what the sensor advertises. Measured over serial on a real
board:

| `--resolution` | Frame size | Throughput |
|----------------|------------|------------|
| `0` (default)  | 240×240    | ~12 fps |
| `1`            | 480×480    | — |
| `2`            | 640×480    | ~7 fps |

The serial link is the bottleneck, not the sensor: every frame crosses as
base64 JPEG. Worth knowing before you plan the stereo pipeline around it.

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
├── diagnose.py          # which firmware is a board actually running?
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
| Port opens but no frames | Run `python diagnose.py COMx` — usually the wrong firmware, not the camera. |
| "did not answer AT+NAME?" | The board is not running sscma-micro. See **Recovering a board** below. |
| "No model is flashed on this board" | `--detect` needs a model. Load one with SenseCraft AI, or drop `--detect`. |
| Boxes sit down-and-right of the object | Firmware reports corner-origin boxes: add `--box-format corner`. |
| Window opens then freezes | Board reset mid-stream. Unplug, replug, rerun. |

---

## Recovering a board that never sees a camera

If a board produces no frames **no matter which camera or cable you try**, stop
swapping hardware — the camera is almost certainly fine. Run:

```powershell
python diagnose.py
```

This captures the bootloader banner and scans baud rates. The line that matters
is `slot flash_offset`:

| | Healthy board | Board running the wrong image |
|---|---|---|
| `slot flash_offset` | `0x00100000` | `0x00000000` |
| After `jump_addr` | `Build date: …` → `sensor_type: 15` → `{"name": "INIT@STAT?", … "is_ready": 1}` | unreadable at 921600; a console at 115200 answering `Command not found!` |
| `AT+NAME?` @ 921600 | `{"type": 0, "name": "NAME?", "code": 0, "data": "Grove Vision AI V2"}` | nothing |

A board booting slot `0x00000000` is running **Himax factory test firmware**,
which contains no camera application at all — hence no frames, ever, with any
camera. The bootloader itself is healthy (it verifies and jumps), so the board
is fully recoverable by reflashing.

**To reflash:** connect the board alone, open
[SenseCraft AI](https://sensecraft.seeed.cc/ai/#/model) in Chrome or Edge,
choose *Grove Vision AI V2*, and flash a model — this rewrites the application
slot and restores sscma-micro. Then confirm:

```powershell
python diagnose.py COM3      # expect: HEALTHY
python stream_camera.py --port COM3 --info
```

Web Serial needs Chrome or Edge, and nothing else may be holding the port —
close this tool first.

---

## Verification status

**Verified against real hardware** — a Grove Vision AI V2 (`id=a4ea3fe8`,
firmware `2025.01.02`) on Windows: identity, sensor enumeration, model-slot
reporting, resolution switching (240×240 and 640×480), and continuous frame
decoding.

Also covered by a fake-device harness: reply framing including byte-at-a-time
arrival, junk tolerance, JPEG decoding, centre→corner box conversion,
out-of-bounds clamping, malformed boxes, command rejection and read timeouts.

**Not yet exercised:** `--detect`, because no model is currently flashed on the
test board. The box-decoding path is covered by the harness but has not seen a
real model's output, so `--box-format` may need flipping the first time you
try it.
