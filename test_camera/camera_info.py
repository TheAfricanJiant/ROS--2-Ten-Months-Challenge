#!/usr/bin/env python3
"""Inspect camera properties, describe your own, and measure focal length.

    python camera_info.py --list                # every known camera
    python camera_info.py --show ir-3.6mm       # one camera in detail
    python camera_info.py --add                 # describe a camera you own
    python camera_info.py --measure             # measure focal length for real
    python camera_info.py --night --port PORT   # is the IR illumination working?

Datasheet numbers are nominal. The board crops and scales the sensor before it
ever reaches you, so `--measure` is the only way to get a focal length you can
trust for depth.
"""

from __future__ import annotations

import argparse
import sys
from pathlib import Path

from sscma import SSCMAClient, SSCMAError, find_port, port_hint
from sscma.cameras import (
    BUILTIN_KEYS,
    CAMERAS,
    CUSTOM_CAMERAS_FILE,
    CameraSpec,
    focal_px_from_measurement,
    get_camera,
    load_custom_cameras,
    save_custom_camera,
)


# --- prompting helpers ----------------------------------------------------

def ask(prompt: str, default: str | None = None) -> str:
    suffix = f" [{default}]" if default is not None else ""
    while True:
        answer = input(f"{prompt}{suffix}: ").strip()
        if answer:
            return answer
        if default is not None:
            return default
        print("  (a value is required)")


def ask_float(prompt: str, default: float | None = None,
              minimum: float = 0.0) -> float:
    while True:
        raw = ask(prompt, None if default is None else str(default))
        try:
            value = float(raw)
        except ValueError:
            print(f"  '{raw}' is not a number")
            continue
        if value <= minimum:
            print(f"  must be greater than {minimum}")
            continue
        return value


def ask_int(prompt: str, default: int | None = None) -> int:
    while True:
        raw = ask(prompt, None if default is None else str(default))
        try:
            return int(raw)
        except ValueError:
            print(f"  '{raw}' is not a whole number")


def ask_yes_no(prompt: str, default: bool) -> bool:
    suffix = "Y/n" if default else "y/N"
    while True:
        raw = input(f"{prompt} [{suffix}]: ").strip().lower()
        if not raw:
            return default
        if raw in ("y", "yes"):
            return True
        if raw in ("n", "no"):
            return False
        print("  answer y or n")


def ask_choice(prompt: str, choices: list[str], default: str) -> str:
    while True:
        raw = ask(f"{prompt} ({'/'.join(choices)})", default)
        if raw in choices:
            return raw
        print(f"  pick one of: {', '.join(choices)}")


# --- listing --------------------------------------------------------------

def list_cameras() -> int:
    print(f"{'key':<14} {'name':<44} {'focal':>7} {'HFOV':>7} {'IR':>4} {'LEDs':>5}")
    print("-" * 88)
    for key, cam in CAMERAS.items():
        tag = "" if key in BUILTIN_KEYS else "  (custom)"
        print(f"{key:<14} {cam.name[:44]:<44} {cam.focal_length_mm:>6.2f}mm "
              f"{cam.hfov_deg:>6.1f}° {'yes' if cam.ir_capable else 'no':>4} "
              f"{'yes' if cam.ir_leds else 'no':>5}{tag}")
    print("\nHFOV is computed from sensor size and focal length. Vendor figures "
          "often differ;\nrun --measure for the number that actually matters.")
    return 0


def show_camera(key: str, widths: list[int]) -> int:
    cam = get_camera(key)
    print(f"\n{cam.name}\n{'=' * len(cam.name)}")
    for field, value in cam.summary().items():
        print(f"  {field:<18} {value}")

    print("\n  Nominal focal length in pixels, by output resolution:")
    for width in widths:
        print(f"    {width:>4}px wide -> f = {cam.focal_px(width):7.1f} px")

    if cam.is_fisheye:
        print("\n  This is a fisheye lens. Straight lines bow outwards, and the\n"
              "  pinhole model that stereo depth relies on only holds near the\n"
              "  centre of the image. Expect depth error to grow towards the\n"
              "  edges until you run a proper checkerboard calibration.")
    if cam.notes:
        print(f"\n  {cam.notes}")
    if cam.image:
        print(f"\n  Photo: {cam.image}")
    return 0


# --- describing a new camera ---------------------------------------------

# Optical formats are nicknames, not sizes. These are the real active areas.
SENSOR_PRESETS = {
    '1/4"': (3.67, 2.74),
    '1/3.2"': (4.54, 3.42),
    '1/3"': (4.80, 3.60),
    '1/2.9"': (5.12, 2.88),
    '1/2.5"': (5.70, 4.28),
    '1/2.3"': (6.17, 4.55),
    '1/2"': (6.40, 4.80),
}


def add_camera(path: Path) -> int:
    print("\nDescribe your camera. Anything you are unsure of can stay at the\n"
          "default - only the sensor size and focal length affect the maths.\n")

    key = ask("Short key (no spaces), e.g. my-ir-6mm")
    if key in BUILTIN_KEYS:
        print(f"error: {key!r} is a built-in camera; choose another key", file=sys.stderr)
        return 1

    name = ask("Full name", key)
    sensor = ask("Sensor chip (e.g. OV5647, IMX219)", "unknown")

    print("\n  Common optical formats:")
    for fmt, (w, h) in SENSOR_PRESETS.items():
        print(f"    {fmt:<8} active area {w} x {h} mm")
    print("    custom   enter the millimetres yourself")

    fmt = ask_choice("Optical format", [*SENSOR_PRESETS, "custom"], '1/4"')
    if fmt == "custom":
        width_mm = ask_float("Sensor active width (mm)")
        height_mm = ask_float("Sensor active height (mm)")
        fmt = "custom"
    else:
        width_mm, height_mm = SENSOR_PRESETS[fmt]
        print(f"  -> {width_mm} x {height_mm} mm")

    native_w = ask_int("Native pixel width", 2592)
    native_h = ask_int("Native pixel height", 1944)
    focal_mm = ask_float("Lens focal length in mm (engraved on the lens)")
    lens_type = ask_choice("Lens type", ["standard", "wide", "fisheye"], "standard")
    ir_capable = ask_yes_no("Does it see infrared (no IR-cut filter / 'NoIR')?", False)
    ir_leds = ask_yes_no("Does the board carry its own IR LEDs?", False)
    vendor_fov = input("Vendor-advertised FOV in degrees (blank to skip): ").strip()
    notes = input("Notes (blank to skip): ").strip()

    spec = CameraSpec(
        key=key, name=name, sensor=sensor, sensor_format=fmt,
        sensor_width_mm=width_mm, sensor_height_mm=height_mm,
        native_width=native_w, native_height=native_h,
        focal_length_mm=focal_mm, lens_type=lens_type,
        ir_capable=ir_capable, ir_leds=ir_leds, notes=notes,
        vendor_fov_deg=float(vendor_fov) if vendor_fov else None,
    )

    print(f"\n  Computed HFOV: {spec.hfov_deg:.1f}°   "
          f"VFOV: {spec.vfov_deg:.1f}°   DFOV: {spec.dfov_deg:.1f}°")
    if spec.vendor_fov_deg and abs(spec.vendor_fov_deg - spec.dfov_deg) > 20:
        print("  Note: that is a long way from the vendor figure. Vendors often\n"
              "  quote the lens's diagonal coverage on a larger sensor than the\n"
              "  one actually fitted.")

    if not ask_yes_no("\nSave this camera?", True):
        print("Not saved.")
        return 1

    save_custom_camera(spec, path)
    print(f"Saved to {path}. Use it anywhere with --camera {key}")
    return 0


# --- measuring ------------------------------------------------------------

def measure(widths: list[int]) -> int:
    print("""
Measuring focal length
======================
This needs no datasheet, and it absorbs whatever cropping the board does.

  1. Put a flat object of known width squarely in front of the camera,
     face-on, roughly centred. A sheet of A4 (0.297 m wide, landscape) or a
     ruler works well.
  2. Measure the distance from the lens to the object with a tape measure.
  3. Stream the camera and note how many pixels wide the object appears:
        python stream_camera.py --port PORT
     Press 's' to save a snapshot, then measure the object's pixel width in
     any image editor.
""")
    object_width = ask_float("Object real width (m), e.g. 0.297")
    distance = ask_float("Distance from lens to object (m)")
    pixels = ask_float("Object width in the image (pixels)")
    image_width = ask_int("Image width in pixels", 240)

    focal = focal_px_from_measurement(object_width, distance, pixels)
    print(f"\n  Measured focal length: {focal:.1f} px at {image_width}px wide")
    print(f"  Equivalent HFOV:       "
          f"{2 * __import__('math').degrees(__import__('math').atan(image_width / (2 * focal))):.1f}°")

    for other in widths:
        if other != image_width:
            print(f"  Scaled to {other}px wide:  {focal * other / image_width:.1f} px")

    print("\n  Put this in your rig config as focal_px, or pass it to\n"
          "  calibrate_stereo.py when it asks.")
    return 0


# --- night-mode check -----------------------------------------------------

def night_check(port: str, baud: int, samples: int = 12) -> int:
    """Confirm the sensor still yields a usable image in the dark."""
    print(f"\nNight-mode check on {port}")
    print("Turn the room lights off. The IR LEDs are invisible to you but the\n"
          "sensor should still see. A phone camera will show them glowing.\n")
    input("Press Enter when the lights are off... ")

    try:
        with SSCMAClient(port=port, baudrate=baud) as client:
            spec_note = ""
            frames = []
            for frame in client.stream(detect=False):
                frames.append(frame)
                if len(frames) >= samples:
                    break
    except SSCMAError as exc:
        print(f"error: {exc}", file=sys.stderr)
        return 1

    means = [float(f.image.mean()) for f in frames]
    stds = [float(f.image.std()) for f in frames]
    mean, contrast = sum(means) / len(means), sum(stds) / len(stds)

    print(f"\n  {len(frames)} frames: mean brightness {mean:.1f}/255, "
          f"contrast (std dev) {contrast:.1f}")

    if mean < 3:
        print("  RESULT: black. Either the IR LEDs are not powered, or this\n"
              "          camera has an IR-cut filter and cannot see IR at all.")
        verdict = 1
    elif contrast < 6:
        print("  RESULT: lit but flat - almost no detail. The LEDs are working\n"
              "          but nothing is in range, or the scene is washed out.\n"
              "          Put an object 0.3-1 m away and try again.")
        verdict = 1
    else:
        print("  RESULT: usable image in darkness. Night mode works.")
        verdict = 0

    print(f"\n  Compare with the lights on: mean should rise noticeably.{spec_note}")
    return verdict


# --- day / night mode -----------------------------------------------------

#: Below this mean saturation the image is effectively monochrome, which means
#: infrared is reaching the sensor unfiltered.
MONOCHROME_SATURATION_PCT = 8.0


def mode_check(ports: list[str], baud: int, samples: int = 8) -> int:
    """Report whether each camera's IR-cut filter is in or out.

    There is no AT command for this - the filter and the illuminators are
    driven by a light sensor on the camera module itself, which the Vision AI
    V2 can neither read nor control. But the effect is measurable: with the
    filter removed, infrared floods the red, green and blue channels about
    equally, so a lit scene comes back almost colourless.
    """
    import numpy as np

    print("\nDay / night mode")
    print("An IR-cut filter blocks infrared. With it removed (night mode) IR")
    print("reaches all three colour channels equally, so even a lit room looks")
    print("nearly monochrome. That is what this measures.\n")

    states = {}
    for port in ports:
        try:
            with SSCMAClient(port=port, baudrate=baud) as client:
                frames = []
                for frame in client.stream(detect=False):
                    frames.append(frame)
                    if len(frames) >= samples:
                        break
        except SSCMAError as exc:
            print(f"  {port}: {exc}")
            continue

        saturations, spreads = [], []
        for frame in frames:
            image = frame.image.astype("float32")
            high = image.max(axis=2)
            low = image.min(axis=2)
            lit = high > 25
            if lit.any():
                saturations.append(float((((high - low) / np.maximum(high, 1))[lit]).mean() * 100))
            means = [float(image[:, :, c].mean()) for c in range(3)]
            spreads.append(max(means) - min(means))

        if not saturations:
            print(f"  {port}: too dark to judge - try again with the lights on")
            continue

        saturation = sum(saturations) / len(saturations)
        spread = sum(spreads) / len(spreads)
        night = saturation < MONOCHROME_SATURATION_PCT
        states[port] = night

        print(f"  {port}: saturation {saturation:5.1f}%  channel spread {spread:4.1f}  "
              f"-> {'NIGHT (IR-cut removed)' if night else 'DAY (IR-cut in place)'}")

    if len(states) > 1 and len(set(states.values())) > 1:
        print("\n  MISMATCH: these cameras are in different modes.")
        print("  For stereo that matters - one is seeing infrared and the other")
        print("  is not, so the same scene looks different to each eye and")
        print("  matching between them gets harder.")
        return 1

    if states:
        print("\n  Both cameras agree." if len(states) > 1 else "")
        print("  This switches automatically from the light sensor on the camera")
        print("  module. To force it: light that sensor for day mode, shade it")
        print("  for night. No AT command can do it.")
    return 0


# --- entry point ----------------------------------------------------------

def main(argv: list[str] | None = None) -> int:
    parser = argparse.ArgumentParser(
        description="Camera properties, custom cameras, and focal-length measurement.",
        formatter_class=argparse.RawDescriptionHelpFormatter,
    )
    action = parser.add_mutually_exclusive_group(required=True)
    action.add_argument("--list", action="store_true", help="List every known camera.")
    action.add_argument("--show", metavar="KEY", help="Show one camera in detail.")
    action.add_argument("--add", action="store_true", help="Describe a camera you own.")
    action.add_argument("--measure", action="store_true",
                        help="Work out focal length in pixels by measurement.")
    action.add_argument("--night", action="store_true",
                        help="Check the camera still sees in darkness.")
    action.add_argument("--mode", action="store_true",
                        help="Report day/night (IR-cut filter) state per camera.")

    parser.add_argument("--port", action="append", default=None,
                        help=f"Serial port, e.g. {port_hint()}. Repeat for --mode.")
    parser.add_argument("--baud", type=int, default=921600)
    parser.add_argument("--widths", type=int, nargs="+", default=[240, 480, 640],
                        help="Output widths to report focal length for.")
    parser.add_argument("--custom-file", type=Path, default=Path(CUSTOM_CAMERAS_FILE),
                        help=f"Where custom cameras live (default: {CUSTOM_CAMERAS_FILE}).")
    args = parser.parse_args(argv)

    try:
        load_custom_cameras(args.custom_file)
    except ValueError as exc:
        print(f"error: {exc}", file=sys.stderr)
        return 1

    try:
        if args.list:
            return list_cameras()
        if args.show:
            return show_camera(args.show, args.widths)
        if args.add:
            return add_camera(args.custom_file)
        if args.measure:
            return measure(args.widths)
        if args.night:
            return night_check((args.port or [None])[0] or find_port(), args.baud)
        if args.mode:
            return mode_check(args.port or [find_port()], args.baud)
    except KeyError as exc:
        print(f"error: {exc}", file=sys.stderr)
        return 1
    except SSCMAError as exc:
        print(f"error: {exc}", file=sys.stderr)
        return 1
    except (KeyboardInterrupt, EOFError):
        print("\nCancelled.")
        return 130
    return 0


if __name__ == "__main__":
    sys.exit(main())
