#!/usr/bin/env python3
"""Measure your stereo rig once and write a config ROS 2 can load.

    python calibrate_stereo.py                    # interactive
    python calibrate_stereo.py --baseline 0.079 --left PORT --right PORT

Produces:

  stereo_config.yaml   the rig, in ROS 2 parameter layout; read by stereo_vision.py
  left.yaml            sensor_msgs/CameraInfo for the left camera
  right.yaml           ditto for the right, carrying the baseline in Tx

See assets/images/stero_vision_setup.jpg for how to take the measurements.
"""

from __future__ import annotations

import argparse
import sys
from pathlib import Path

from sscma import (
    SSCMAClient,
    SSCMAError,
    available_ports,
    identify,
    port_hint,
)
from sscma.cameras import (
    CAMERAS,
    CUSTOM_CAMERAS_FILE,
    get_camera,
    load_custom_cameras,
)
from sscma.config import CameraConfig, StereoConfig

from camera_info import ask, ask_choice, ask_float, ask_yes_no


#: Measured frame periods, so the sync tolerance matches reality. Bigger
#: frames take longer over serial, so the two boards drift further apart.
#: 240x240 runs ~13.7 fps (73 ms), 640x480 ~6.1 fps (164 ms).
SKEW_BY_WIDTH = {240: 80.0, 480: 120.0, 640: 170.0}


def default_skew(width: int) -> float:
    """Tolerance a bit above one frame period for this resolution."""
    return SKEW_BY_WIDTH.get(width, 80.0)


def probe_ports() -> list[tuple[str, str | None, str | None]]:
    """Every candidate port, with the board name if it answers."""
    found = []
    for info in available_ports():
        if not info.is_candidate:
            continue
        found.append((info.device, identify(info.device), info.serial_number))
    return found


def choose_ports(args) -> tuple[tuple[str, str | None], tuple[str, str | None]]:
    """Work out which port is the left eye and which is the right."""
    if args.left and args.right:
        return (args.left, None), (args.right, None)

    print("\nLooking for boards...")
    found = probe_ports()
    alive = [(port, name, ser) for port, name, ser in found if name]

    for port, name, ser in found:
        state = name or "did not answer (wrong firmware? see diagnose.py)"
        print(f"  {port:<22} {state}" + (f"   [SER={ser}]" if ser else ""))

    if len(alive) < 2:
        print(f"\nNeed two working boards, found {len(alive)}.", file=sys.stderr)
        print("Plug both in, and check each one with:  python diagnose.py",
              file=sys.stderr)
        raise SSCMAError("not enough boards")

    print("\nWhich board is on the LEFT as the robot looks forward?")
    print("(Cover one lens with a finger and stream it if you are unsure:")
    print(f" python stream_camera.py --port {alive[0][0]})")

    ports = [port for port, _, _ in alive]
    left = ask_choice("Left port", ports, ports[0])
    remaining = [p for p in ports if p != left]
    right = ask_choice("Right port", remaining, remaining[0])

    lookup = {port: (name, ser) for port, name, ser in alive}
    return (left, lookup[left][1]), (right, lookup[right][1])


def choose_camera(side: str, default: str) -> str:
    print(f"\nWhich camera is on the {side}?")
    for key, cam in CAMERAS.items():
        marker = "*" if key == default else " "
        print(f"  {marker} {key:<14} {cam.name}")
    print("  (add your own first with: python camera_info.py --add)")
    return ask_choice(f"{side.capitalize()} camera", list(CAMERAS), default)


def measure_baseline() -> float:
    print("""
Baseline - the distance between the two lens centres
====================================================
Lay the rig flat and measure centre-of-lens to centre-of-lens, as in
assets/images/stero_vision_setup.jpg. A caliper is ideal; a ruler is fine.

This single number sets the whole depth scale: get it 10% wrong and every
distance you measure is 10% wrong. Measure the lenses, not the boards.
""")
    unit = ask_choice("Units", ["mm", "cm", "m"], "mm")
    value = ask_float(f"Distance between lens centres ({unit})")
    return value * {"mm": 0.001, "cm": 0.01, "m": 1.0}[unit]


def measure_convergence() -> float:
    print("""
Convergence - how far the cameras are toed in
=============================================
If both cameras point straight ahead, parallel, this is 0 and you can just
press Enter. If you have angled them inwards to overlap their views sooner,
measure the total angle between the two optical axes with a protractor.

Parallel is easier to get right and is what most rigs use. An angle you
guessed at is worse than an honest zero.
""")
    if not ask_yes_no("Are the cameras toed in (not parallel)?", False):
        return 0.0
    return ask_float("Total angle between the optical axes (degrees)", minimum=-90.0)


def measure_focal(side: str, camera_key: str, image_width: int) -> float | None:
    cam = get_camera(camera_key)
    nominal = cam.focal_px(image_width)
    print(f"\n{side.capitalize()} focal length")
    print(f"  Nominal from the datasheet: {nominal:.1f} px at {image_width}px wide")
    print("  This assumes the board maps the full sensor width onto the image.")
    print("  If it crops instead, the true value is larger and every distance")
    print("  you measure will be wrong by the same ratio.")

    if not ask_yes_no("  Do you have a measured value (camera_info.py --measure)?", False):
        return None
    return ask_float(f"  Measured focal length for the {side} camera (px)")


def build_interactive(args) -> StereoConfig:
    (left_port, left_ser), (right_port, right_ser) = choose_ports(args)

    default = args.camera if args.camera in CAMERAS else "ir-3.6mm"
    left_key = choose_camera("left", default)
    right_key = choose_camera("right", left_key)

    if get_camera(left_key).is_fisheye or get_camera(right_key).is_fisheye:
        print("\nNote: a fisheye lens is in the rig. Depth near the centre of the\n"
              "image will be reasonable; towards the edges it will drift, because\n"
              "the pinhole model does not describe a fisheye. Keep the target\n"
              "central until you have run a checkerboard calibration.")

    width = args.width
    baseline = args.baseline if args.baseline else measure_baseline()
    convergence = args.convergence if args.convergence is not None else measure_convergence()

    left_focal = measure_focal("left", left_key, width)
    right_focal = measure_focal("right", right_key, width)

    print("\nThe per-camera distance readout assumes it knows how big the target")
    print("really is (one camera alone cannot measure depth). The stereo figure")
    print("does not use this.")
    object_height = ask_float("Real height of the object you will track (m)", 0.20)

    notes = input("\nNotes for this rig (blank to skip): ").strip()

    return StereoConfig(
        left=CameraConfig(port=left_port, camera_key=left_key,
                          serial_number=left_ser, focal_px=left_focal,
                          image_width=width, image_height=args.height),
        right=CameraConfig(port=right_port, camera_key=right_key,
                           serial_number=right_ser, focal_px=right_focal,
                           image_width=width, image_height=args.height),
        baseline_m=baseline,
        convergence_deg=convergence,
        max_sync_skew_ms=args.max_skew or default_skew(width),
        object_height_m=object_height,
        notes=notes,
    )


def report(config: StereoConfig) -> None:
    focal = config.left.resolved_focal_px()
    print("\n" + "=" * 62)
    print("Rig summary")
    print("=" * 62)
    print(f"  baseline            {config.baseline_m * 1000:.1f} mm")
    print(f"  convergence         {config.convergence_deg:.1f}° "
          f"({'parallel' if config.is_parallel else 'toed in'})")
    print(f"  left                {config.left.port}  {config.left.camera_key}")
    print(f"  right               {config.right.port}  {config.right.camera_key}")
    print(f"  focal length        {focal:.1f} px "
          f"({'measured' if config.left.focal_px else 'nominal'})")
    print(f"  image               {config.left.image_width}x{config.left.image_height}")
    print(f"  sync tolerance      {config.max_sync_skew_ms:.0f} ms")

    near = config.min_measurable_distance_m()
    far = config.max_measurable_distance_m()
    print(f"\n  Usable range        {near:.2f} m to about {far:.1f} m")
    print(f"  At 1 m, one pixel of disparity is worth "
          f"{1.0 - (focal * config.baseline_m) / ((focal * config.baseline_m / 1.0) + 1):.3f} m "
          f"of depth error.")
    print("  Depth error grows with the square of distance: doubling the range")
    print("  quadruples the error. A wider baseline pushes that out.")

    if config.baseline_m < 0.04:
        print("\n  Warning: that is a very short baseline. Depth will be noisy\n"
              "  beyond a metre or so. Move the cameras further apart if you can.")


def main(argv: list[str] | None = None) -> int:
    parser = argparse.ArgumentParser(
        description="Measure a stereo rig and write a ROS 2 config.",
        formatter_class=argparse.RawDescriptionHelpFormatter,
    )
    parser.add_argument("--left", help=f"Left camera port, e.g. {port_hint()}.")
    parser.add_argument("--right", help="Right camera port.")
    parser.add_argument("--camera", default="ir-3.6mm",
                        help="Default camera key to offer (default: ir-3.6mm).")
    parser.add_argument("--baseline", type=float,
                        help="Lens-centre spacing in metres; skips the prompt.")
    parser.add_argument("--convergence", type=float,
                        help="Total toe-in angle in degrees (0 = parallel).")
    parser.add_argument("--width", type=int, default=240, help="Frame width.")
    parser.add_argument("--height", type=int, default=240, help="Frame height.")
    parser.add_argument("--max-skew", type=float, default=None,
                        help="Largest tolerable gap between paired frames (ms). "
                             "Defaults to about one frame period for the chosen "
                             "resolution, since bigger frames arrive slower.")
    parser.add_argument("--output", type=Path, default=Path("stereo_config.yaml"))
    parser.add_argument("--camera-info-dir", type=Path, default=Path("."),
                        help="Where to write left.yaml / right.yaml.")
    parser.add_argument("--custom-file", type=Path, default=Path(CUSTOM_CAMERAS_FILE))
    args = parser.parse_args(argv)

    try:
        load_custom_cameras(args.custom_file)

        if args.left and args.right and args.baseline:
            config = StereoConfig(
                left=CameraConfig(port=args.left, camera_key=args.camera,
                                  image_width=args.width, image_height=args.height),
                right=CameraConfig(port=args.right, camera_key=args.camera,
                                   image_width=args.width, image_height=args.height),
                baseline_m=args.baseline,
                convergence_deg=args.convergence or 0.0,
                max_sync_skew_ms=args.max_skew or default_skew(args.width),
            )
        else:
            config = build_interactive(args)
    except (SSCMAError, ValueError, KeyError) as exc:
        print(f"error: {exc}", file=sys.stderr)
        return 1
    except (KeyboardInterrupt, EOFError):
        print("\nCancelled.")
        return 130

    report(config)

    config.save(args.output)
    written = config.save_camera_info(args.camera_info_dir)

    print("\nWrote:")
    print(f"  {args.output}          rig config (stereo_vision.py reads this)")
    for path in written:
        print(f"  {path}          ROS 2 CameraInfo")
    print("\nNext:")
    print("  python stereo_vision.py --config " + str(args.output))
    print("\nIn ROS 2:")
    print(f"  ros2 run <your_pkg> <your_node> --ros-args --params-file {args.output}")
    return 0


if __name__ == "__main__":
    sys.exit(main())
