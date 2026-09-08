# 📷 test_camera — cameras, night vision and stereo depth

[← Back to repository index](../README.md)

Bench tooling for [Project 02](../projects/02_stereo_ai_perception_xrp/README.md),
Objectives 2 and 3: prove the cameras and the Grove Vision AI V2 modules work,
in daylight **and** in the dark, then measure the stereo rig and get a depth
reading out of it — all before any of it is wired into ROS 2.

The tests build on each other, so work through them in order:

| # | Test | What it proves |
|---|------|----------------|
| 1 | [SenseCraft AI](#test-1--sensecraft-ai) | The board is alive and a detection model is loaded |
| 2 | [Live camera feed](#test-2--live-camera-feed) | Frames actually reach your computer |
| 3 | [Night mode](#test-3--night-mode) | The IR illumination works in darkness |
| 4 | [Camera properties](#test-4--camera-properties) | You know your lens's real focal length |
| 5 | [Stereo calibration](#test-5--stereo-calibration) | The rig is measured, and ROS 2 can load it |
| 6 | [Stereo vision](#test-6--stereo-vision) | Two eyes agree on how far away something is |

---

## Platform support

| OS | Status |
|----|--------|
| **Windows 11** | **Tested.** Everything here was developed and run on Windows. |
| **Linux** | **Not tested.** Should work — nothing is Windows-specific — but no one has run it. |
| **macOS** | **Not tested.** Same. |

The code itself is platform-neutral: ports are discovered through `pyserial`,
paths through `pathlib`, and error messages adapt to the OS you are on. The
Linux and macOS instructions below are written from the documented behaviour of
those tools, not from a machine anyone has run them on. If you try them,
corrections are welcome.

### Install

<table>
<tr><th>Windows</th><th>Linux</th><th>macOS</th></tr>
<tr valign="top"><td>

```powershell
cd test_camera
py -m venv .venv
.venv\Scripts\Activate.ps1
pip install -r requirements.txt
```

If PowerShell blocks activation:

```powershell
Set-ExecutionPolicy -Scope Process -ExecutionPolicy Bypass
```

</td><td>

```bash
cd test_camera
python3 -m venv .venv
source .venv/bin/activate
pip install -r requirements.txt
```

Serial access needs group membership:

```bash
sudo usermod -a -G dialout $USER
# log out and back in
```

</td><td>

```bash
cd test_camera
python3 -m venv .venv
source .venv/bin/activate
pip install -r requirements.txt
```

The boards use a WCH CH343 bridge. Recent macOS has a driver built in; older
versions need WCH's.

</td></tr>
</table>

### Finding your serial port

Every command below takes `--port`. Leave it out and the tool asks each port
who it is and picks the board that answers — which also means it will not grab
the wrong board when two are plugged in.

| OS | Ports look like | List them |
|----|-----------------|-----------|
| Windows | `COM3`, `COM4` | `python stream_camera.py --list-ports` |
| Linux | `/dev/ttyACM0`, `/dev/ttyUSB0` | same, or `ls /dev/ttyACM* /dev/ttyUSB*` |
| macOS | `/dev/cu.wchusbserial*` | same, or `ls /dev/cu.*` |

Examples in this README use `PORT` as a placeholder. Substitute whatever your
OS calls it.

---

## Test 1 — SenseCraft AI

**Start here.** This is Seeed's web tool: it talks to the board over Web
Serial, confirms it is alive, and — the part everything later depends on —
**loads a detection model onto it**. Without a model the board returns pictures
but no detections, and stereo depth has nothing to triangulate.

![SenseCraft AI model library](../assets/images/senseCraft.jpg)

1. Open [SenseCraft AI](https://sensecraft.seeed.cc/ai/model) in **Chrome or
   Edge** (Web Serial does not exist in Firefox or Safari).
2. Connect the board by USB-C, choose **Grove Vision AI V2**, and connect.
3. Pick a **Detection** model — filter by task *Detection* and device
   *Grove - Vision AI V2*. **Face Detection** or **Gesture Detection** are good
   first choices; both are Swift-YOLO models that run on this hardware.
4. Flash it, and watch the preview. You should see boxes drawn on the live
   image.

> **For stereo, load the _same_ model on _both_ boards.** The two eyes have to
> agree on what they are looking at before anything can be matched between
> them.

**Close the SenseCraft tab before running anything else.** It holds the serial
port open even after you navigate away, and every other tool will then fail
with *access denied*. This catches people out constantly.

<details>
<summary>If SenseCraft refuses to connect</summary>

Its connect flow waits for the board's SSCMA handshake, so a board running the
wrong firmware can never get past this screen — and SenseCraft cannot fix that,
because the fix needs the firmware that is missing. Diagnose it instead:

```
python diagnose.py
```

See [Recovering a board](#recovering-a-board-that-never-sees-a-camera).
</details>

---

## Test 2 — Live camera feed

Now get frames onto your own machine, outside the browser.

```
python stream_camera.py --list-ports     # what is plugged in
python stream_camera.py --info           # identity, cameras, model slots
python stream_camera.py                  # plain camera feed
python stream_camera.py --detect         # feed + the model's boxes
python stream_camera.py --port PORT --resolution 2 --scale 2
```

| Key | Action |
|-----|--------|
| `q` / `Esc` | quit |
| `s` | save a PNG into `snapshots/` |
| `d` | toggle the box + HUD overlay |

`--detect` needs the model from Test 1. `--info` reads the model's own
metadata, so it can tell you exactly what is loaded:

```
Model: Face Detection  (id 60094, task detect)
  classes: face
  confidence threshold 60%, IoU 45%
  checksum: 377aee70190387cc4cf2435f13aab3af
```

Those class names are picked up automatically, so **you do not need a labels
file** — `--labels` is only for overriding them.

> **Do not trust `AT+MODELS?` for this.** It reports `size: 0` whether or not a
> model is flashed, on every board tested. The only reliable source is the
> base64 metadata blob in `AT+INFO?`, which is what these tools read.

### Resolution and throughput

`--info` lists what the sensor offers. Measured on a real board:

| `--resolution` | Frame size | Throughput |
|----------------|------------|------------|
| `0` (default)  | 240×240    | ~12 fps |
| `1`            | 480×480    | — |
| `2`            | 640×480    | ~7 fps |

The serial link is the bottleneck, not the sensor — every frame crosses as
base64-encoded JPEG. This matters for stereo: two cameras share your USB bus,
and higher resolution means slower frames and worse synchronisation.

---

## Test 3 — Night mode

The whole point of these IR modules is seeing in the dark. This checks that
they do.

```
python camera_info.py --night --port PORT
```

Turn the lights off when prompted. The tool grabs a dozen frames and reports
mean brightness and contrast:

| Result | Meaning |
|--------|---------|
| **usable image in darkness** | Night mode works. |
| **black** | IR LEDs are not powered, **or** this camera has an IR-cut filter and physically cannot see IR — see the table below. |
| **lit but flat** | LEDs work but nothing is in range. Put an object 0.3–1 m away and retry. |

The IR LEDs are invisible to you but a phone camera will show them glowing
faintly purple — a quick way to tell "not powered" from "not detected".

> A **plain Raspberry Pi camera will fail this test by design.** Its IR-cut
> filter blocks exactly the wavelength the LEDs emit. You need a **NoIR** or a
> dedicated IR module. Run `python camera_info.py --list` and check the `IR`
> column.

---

## Test 4 — Camera properties

Stereo depth needs a focal length **in pixels**. That comes from the lens and
the sensor:

```
f_px = focal_length_mm × image_width_px ÷ sensor_width_mm
```

```
python camera_info.py --list              # every known camera
python camera_info.py --show ir-3.6mm     # one in detail
python camera_info.py --add               # describe a camera you own
python camera_info.py --measure           # measure it for real
```

### The cameras

| | Camera | Key | Sensor | Focal | HFOV (computed) | Vendor claim | Sees IR | IR LEDs |
|---|---|---|---|---|---|---|---|---|
| <img src="../assets/images/IR3_6mm.jpg" width="190"> | **IR 3.6 mm 1080P** | `ir-3.6mm` | OV5647, 1/4″ (3.67×2.74 mm) | 3.6 mm | **54.0°** | 72° | ✅ | ✅ |
| <img src="../assets/images/IR1_7mm.jpg" width="190"> | **IR 1.7 mm 5MP 1/2.5″ fisheye** | `ir-1.7mm` | OV5647, 1/4″ (3.67×2.74 mm) | 1.7 mm | **94.4°** | 150° | ✅ | ✅ |
| <img src="../assets/images/rpicam.jpg" width="190"> | **Raspberry Pi Camera v1.3** | `rpi-v1` | OV5647, 1/4″ (3.67×2.74 mm) | 3.6 mm | **54.0°** | 53.5° | ❌ | ❌ |

Also in the database, for people who own them: `rpi-v1-noir`, `rpi-v2`,
`rpi-v2-noir`, `rpi-v3`.

**Which to use for stereo:** the **3.6 mm**. It is much closer to a pinhole
camera, which is the model every depth formula here assumes. The 1.7 mm sees a
far wider scene but bows straight lines badly, and that distortion turns
directly into depth error away from the image centre.

**On the vendor numbers.** The gap in the fisheye row is real, not a typo. The
`1/2.5″` on that lens is the *image circle it can cover*, not the sensor fitted
behind it. On a 1/4″ OV5647 only the middle of that circle is used, so you get
about 94°, not 150°. Computed values come from sensor size and focal length;
vendor figures are usually the lens's diagonal coverage on a larger sensor.

### Describing a camera we do not know

```
python camera_info.py --add
```

It asks for sensor format (with the real millimetre sizes behind the
nicknames), native resolution, focal length and whether the module sees IR,
then writes `custom_cameras.yaml`. Every other tool picks it up automatically,
so `--camera my-6mm` works everywhere afterwards.

### Measuring focal length — do this before you trust any distance

Every number in that table is **nominal**. It assumes the board maps the full
sensor width onto the output image, and the Grove Vision AI V2 crops and scales
to 240×240 before you ever see a frame. If it crops rather than scales, the
true focal length is larger — and every distance you measure is wrong by the
same ratio.

```
python camera_info.py --measure
```

Show the camera something of known width (a sheet of A4 is 0.297 m) at a
measured distance, note how many pixels wide it lands, and the tool solves
`f_px = pixels × distance ÷ width`. No datasheet involved, and it absorbs
whatever the board does internally. Feed the result to `calibrate_stereo.py`
when it asks.

---

## Test 5 — Stereo calibration

```
python calibrate_stereo.py
```

Interactive: it finds both boards, asks which is the left eye, which camera is
on each side, and takes two measurements from you.

![Measuring the stereo rig](../assets/images/stero_vision_setup.jpg)

**Baseline** — the distance between the two **lens centres**, as in the photo
above. Measure the lenses, not the boards. A caliper is ideal, a ruler is fine.
This single number sets the entire depth scale: get it 10% wrong and every
distance you ever measure is 10% wrong.

**Convergence** — the total angle between the two optical axes. If both cameras
point straight ahead, this is **0** and you can just press Enter. Parallel is
easier to get right and is what most rigs use; a guessed angle is worse than an
honest zero. If you have toed them in, measure it with a protractor.

It writes three files:

| File | Purpose |
|------|---------|
| `stereo_config.yaml` | The rig, in ROS 2 parameter layout. `stereo_vision.py` reads this. |
| `left.yaml` | `sensor_msgs/CameraInfo` for the left camera. |
| `right.yaml` | Ditto for the right, with the baseline encoded as `Tx = -fx × B`. |

The last two are the layout `camera_info_manager` expects, so the stock ROS 2
image pipeline (`stereo_image_proc` and friends) can consume the same rig:

```bash
ros2 run <your_pkg> <your_node> --ros-args --params-file stereo_config.yaml
```

Non-interactive, if you already know the numbers:

```
python calibrate_stereo.py --left PORT --right PORT --baseline 0.079
```

### What the geometry buys you

The tool prints the usable range for your rig. For a 79 mm baseline at 240×240
with a 3.6 mm lens:

- **usable range** ≈ 0.08 m to 18 m
- **at 1 m**, one pixel of disparity is worth **51 mm** of depth
- **at 1.5 m**, one pixel is worth **112 mm**

Depth error grows with the **square** of distance — double the range, quadruple
the error. And because the model reports bounding boxes as whole pixels,
disparity is quantised: those per-pixel figures are your real precision floor,
not a rounding detail. A wider baseline or a higher resolution pushes it out.

---

## Test 6 — Stereo vision

```
python stereo_vision.py --config stereo_config.yaml
```

Three panels in one window — left eye and right eye on top, and the combined
view below, stretched wide because it carries the number that matters.

![The three-panel stereo layout](../assets/images/stereo_layout_example.png)

*(Rendered from synthetic frames constructed for a known 1.5 m distance, to
show the layout and verify the maths — not a photograph of a real scene.)*

- **LEFT / RIGHT** — each camera with its own detections. The distance shown is
  a *monocular guess*: one camera cannot measure depth, so it works backwards
  from an assumed real-world object size. It is a sanity check, not a
  measurement.
- **Combined** — the two frames overlaid as a red/cyan anaglyph, so you can
  *see* the disparity as colour fringing: wide fringes mean close, no fringe
  means far away. Matched objects get the **triangulated stereo distance**,
  which is the real one.

| Key | Action |
|-----|--------|
| `q` / `Esc` | quit |
| `s` | save the whole composed view |
| `d` | toggle overlays |
| `a` | anaglyph ⇄ plain blend |

Before streaming, both boards are asked what model they are running and the
metadata **checksums are compared**. Different models on the two eyes is a hard
error, because detections cannot be matched between eyes that disagree about
what they are looking at.

`--no-detect` streams both feeds with no model and no depth, which is a useful
way to check the two cameras are both alive and roughly aligned before you
worry about detections.

### Synchronisation — read this before trusting a number

These are two independent boards on two USB serial links. **There is no
hardware trigger.** Frames cannot be captured simultaneously, only *paired
after the fact* by arrival time. Every frame is timestamped as it arrives, and
each left frame is matched to the nearest right frame; pairs further apart than
`max_sync_skew_ms` (default 60 ms) are thrown away.

**Measured on two real boards** at 240×240 (~14.7 pairs/s): skew sat at
**46–48 ms with almost no spread** — median and maximum within 1 ms of each
other. That flatness is the important part. It is not jitter that smarter
pairing could average away; it is a near-constant phase offset between the two
capture loops, about half a frame period. Searching more frames back for a
tighter match bought only ~1.6 ms, and searching too far made it worse.

So treat **half a frame period as the floor**. The HUD shows live skew and
warns as it approaches the limit. This is fine for a slow-moving robot; it is
not fine for anything fast, because the object genuinely did move during those
48 ms, so the disparity — and the distance — comes out wrong.

Faster frames (lower resolution) shrink the offset. Only a hardware trigger
removes it.

### Getting a believable distance

1. Both boards need the **same model** (Test 1).
2. The object must be visible to **both** cameras — outside the overlap there
   is nothing to triangulate, and the HUD will say so.
3. Keep it near the **centre** of the image, especially with the fisheye, where
   the pinhole model holds worst at the edges.
4. Use a **measured** focal length (Test 4), not the nominal one.
5. Sanity-check against a tape measure at a known distance. If everything is
   off by a constant ratio, your baseline or focal length is wrong; that is the
   first thing to re-measure.

---

## How it works

| Command | Purpose |
|---------|---------|
| `AT+SAMPLE=-1` | stream frames continuously, **no model** — the plain camera feed |
| `AT+INVOKE=-1,0,0` | stream frames **with** inference; `0,0` = include the JPEG, report every frame |
| `AT+BREAK` | stop streaming (sent on startup and on exit) |
| `AT+SENSORS?` | cameras the firmware can see, and their resolutions |
| `AT+MODELS?` | model slots; `size: 0` means nothing is loaded |
| `AT+ID?` / `AT+NAME?` / `AT+VER?` | identity |

Replies are one JSON object per line, wrapped in `\r` … `\n`:

```json
{"type": 1, "name": "SAMPLE", "code": 0,
 "data": {"count": 12, "image": "<base64 jpeg>"}}
```

`type` is `0` for a response to a command, `1` for a streamed event, `2` for a
log line (including `Unknown command`). Base64 never contains a newline, so
splitting the byte stream on `\n` is enough framing — no brace-matching across
a 60 kB payload.

Depth uses **ray intersection**, not `Z = f·B/d`. The textbook formula assumes
perfectly parallel cameras; intersecting the two viewing rays in 3D handles a
toed-in rig as well, and gives the identical answer when the rig is parallel.

```
test_camera/
├── stream_camera.py     # single camera feed
├── camera_info.py       # camera properties, custom cameras, focal measurement, night test
├── calibrate_stereo.py  # measure the rig -> ROS 2 config
├── stereo_vision.py     # three-panel live stereo
├── diagnose.py          # which firmware is a board actually running?
├── flash_firmware.py    # reflash over the bootloader's X-Modem receiver
└── sscma/
    ├── protocol.py      # framing + JSON reply parsing, no I/O
    ├── client.py        # serial transport, AT commands, frame decoding
    ├── cameras.py       # lens/sensor properties and the optics that follow
    ├── config.py        # rig config, ROS 2 readable
    ├── stereo.py        # synchronised capture + triangulation
    └── viewer.py        # OpenCV presentation
```

`sscma/` imports OpenCV lazily, so the protocol, optics and stereo maths carry
no GUI dependency. A ROS 2 node can reuse `client.py`, `cameras.py`,
`config.py` and `stereo.py` directly and publish `sensor_msgs/Image` instead of
calling `imshow` — which is the whole point of building it this way.

---

## Troubleshooting

| Symptom | Cause / fix |
|---------|-------------|
| "No serial ports found" | Charge-only USB-C cable — very common. Swap for a data cable. |
| Access denied / permission error | **Windows/macOS:** another program holds the port — a SenseCraft tab in Chrome keeps it open even when it fails to connect. **Linux:** add yourself to `dialout` and log back in. |
| Port opens but no frames | Run `python diagnose.py PORT` — usually the wrong firmware, not the camera. |
| "did not answer AT+NAME?" | Not running sscma-micro. See [Recovering a board](#recovering-a-board-that-never-sees-a-camera). |
| "could not read model metadata" | The board did not answer `AT+INFO?`. The tools warn and try anyway — `AT+INVOKE` is the real authority. |
| "The two boards are running different models" | Flash the same model on both from SenseCraft; the eyes cannot be matched otherwise. |
| Night test says "black" | IR LEDs unpowered, or the camera has an IR-cut filter (check `camera_info.py --list`). |
| Boxes offset down-and-right | Firmware reports corner-origin boxes: add `--box-format corner`. The default (`center`) is confirmed correct on firmware `2025.01.02`. |
| "no object matched in both eyes" | The object is outside the overlap, or only one board has the model. |
| Stereo distances all wrong by the same ratio | Baseline or focal length is off. Re-measure the baseline; run `camera_info.py --measure`. |
| Stereo distances jump around | Poor sync (check the HUD skew), or the object is at the range limit where one pixel is worth a lot of depth. |

---

## Recovering a board that never sees a camera

If a board produces no frames **no matter which camera or cable you try**, stop
swapping hardware — the camera is almost certainly fine. Run:

```
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
camera. The bootloader itself is healthy (it verifies images and jumps), so the
board is fully recoverable.

**SenseCraft cannot fix this.** Its connect flow first waits for an SSCMA
handshake, which is exactly the firmware that is missing, so it just times out
and reports that it refuses to connect. Reflash over the bootloader instead.

### Reflashing

1. **Free the port.** Close the SenseCraft tab.
2. **Download the firmware** matching your working board's version from the
   [official releases](https://github.com/Seeed-Studio/sscma-example-we2/releases)
   — e.g. `grove_vision_ai_v2_20250102.img` for firmware `2025.01.02`. Check
   which version you want with `python stream_camera.py --info` on a healthy
   board, so both modules end up identical.
3. **Dry run first** — enters the bootloader and confirms the X-Modem receiver
   answers, without writing anything:

   ```
   python flash_firmware.py --image grove_vision_ai_v2_20250102.img --port PORT --dry-run
   ```

   Expect `Set X-modem flag = Yes` and `receiver is ready ('C' handshake seen)`.
   If it cannot get there, hold the **BOOT** button while plugging the USB-C
   cable in, release it, and try again.

4. **Flash** (same command without `--dry-run`). Roughly 18 s for a 630 kB image
   at 921600 baud. Do not unplug the board.

5. **Verify:** `python diagnose.py PORT` → expect `HEALTHY`.

`flash_firmware.py` only ever writes the application slot — the bootloader is
untouched, so an interrupted transfer leaves the board exactly as recoverable
as it was. Retry it.

---

## Verification status

**Verified on real hardware** — two Grove Vision AI V2 modules on Windows 11
(`id=a4ea3fe8` and `id=ae83564f`, both firmware `2025.01.02`): identity, sensor
enumeration, model-slot reporting, resolution switching (240×240 and 640×480),
and continuous frame decoding. `flash_firmware.py` recovered a board stuck on
Himax factory firmware end to end.

**Verified by test harness** — protocol framing including byte-at-a-time
arrival, junk tolerance, JPEG decode, box conversion and clamping, command
rejection, read timeouts; and for the stereo maths, recovery of known 3D points
to within 1e-6 m for both parallel and toed-in rigs, disparity agreeing with
`f·B/Z`, detection matching, config round-trips, and `Tx = -fx·B` in the
generated `CameraInfo`.

**Verified with a model flashed** — with SenseCraft's *Face Detection* on both
boards: model metadata read and identified, class names adopted automatically,
`--detect` streaming detections at ~21 ms inference, and the centre-origin box
format confirmed by rendering both interpretations against a real detection.
Two-board capture ran at 14.7 synchronised pairs/s with matching model
checksums.

**Not yet exercised on hardware:**

- **A real stereo distance.** The capture, pairing, matching and checksum
  checks all run against the two boards, but no object has yet been placed in
  front of both cameras to produce a triangulated reading. The maths is
  verified against synthetic frames to 1e-6 m; what remains unproven is the
  accuracy of a real measurement against a tape measure.
- **Test 3 (night mode).** The code path runs, but no one has yet done the
  lights-off run with an IR module attached.
- **Linux and macOS.** Windows only, so far.
- **Fisheye undistortion.** Not implemented. `distortion_coefficients` in the
  generated `CameraInfo` are all zero. For the 1.7 mm lens that is a real
  limitation, not a formality — run a checkerboard calibration
  (`ros2 run camera_calibration cameracalibrator`) and paste the coefficients in
  if you need accuracy off-centre.
