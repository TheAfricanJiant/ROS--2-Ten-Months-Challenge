#!/usr/bin/env python3
"""Show a Grove Vision AI V2 camera feed on Windows using OpenCV.

    python stream_camera.py --list-ports        # what is plugged in
    python stream_camera.py                     # plain camera feed
    python stream_camera.py --detect            # feed + model detections
    python stream_camera.py --port COM5 --scale 3

Press q or Esc to quit, s to save a snapshot, d to toggle overlays.
"""

from __future__ import annotations

import argparse
import sys
from pathlib import Path

from sscma import (
    DEFAULT_BAUDRATE,
    SSCMAClient,
    SSCMAError,
    Viewer,
    available_ports,
    find_port,
)


def build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(
        description="Stream JPEG frames from a Grove Vision AI V2 over serial.",
        formatter_class=argparse.RawDescriptionHelpFormatter,
        epilog="Keys:  q/Esc quit   s snapshot   d toggle overlays",
    )
    parser.add_argument(
        "--port",
        help="Serial port, e.g. COM5. Auto-detected when omitted.",
    )
    parser.add_argument(
        "--baud",
        type=int,
        default=DEFAULT_BAUDRATE,
        help=f"Baud rate (default: {DEFAULT_BAUDRATE}).",
    )
    parser.add_argument(
        "--detect",
        action="store_true",
        help="Run the flashed model and draw its boxes (AT+INVOKE) instead of "
             "a plain camera feed (AT+SAMPLE).",
    )
    parser.add_argument(
        "--labels",
        type=Path,
        help="Text file with one class name per line, used to name detections.",
    )
    parser.add_argument(
        "--box-format",
        choices=("center", "corner"),
        default="center",
        help="Origin of the x/y in each box. SSCMA uses centre coordinates; "
             "switch to corner if overlays sit down-and-right of the object.",
    )
    parser.add_argument(
        "--resolution",
        type=int,
        metavar="OPT_ID",
        help="Sensor option id to select before streaming, as listed by "
             "--info (0 = 240x240, 1 = 480x480, 2 = 640x480 on stock firmware).",
    )
    parser.add_argument(
        "--info",
        action="store_true",
        help="Print device identity, cameras and model slots, then exit.",
    )
    parser.add_argument(
        "--scale",
        type=int,
        default=2,
        help="Integer upscale for the preview window (default: 2).",
    )
    parser.add_argument(
        "--snapshot-dir",
        type=Path,
        default=Path("snapshots"),
        help="Where the s key writes PNGs (default: ./snapshots).",
    )
    parser.add_argument(
        "--list-ports",
        action="store_true",
        help="List serial ports and exit.",
    )
    return parser


def print_ports() -> int:
    ports = available_ports()
    if not ports:
        print("No serial ports found.")
        print("A charge-only USB-C cable powers the board but carries no data.")
        return 1
    print("Serial ports:")
    for port in ports:
        print(f"  {port}")
    return 0


def load_labels(path: Path | None) -> list[str]:
    if path is None:
        return []
    try:
        lines = path.read_text(encoding="utf-8").splitlines()
    except OSError as exc:
        raise SSCMAError(f"Could not read labels from {path}: {exc}") from exc
    return [line.strip() for line in lines if line.strip()]


def print_info(client: SSCMAClient) -> int:
    """Dump what the board reports about itself."""
    info = client.device_info()
    if not info:
        print("The board did not answer AT+NAME?. It is probably not running "
              "sscma-micro firmware.")
        return 1
    print("Device: " + ", ".join(f"{k}={v}" for k, v in info.items()))

    sensors = client.sensors()
    if not sensors:
        print("Cameras: none reported")
    for sensor in sensors:
        state = "ready" if sensor.get("state") == 1 else f"state={sensor.get('state')}"
        print(f"Camera id={sensor.get('id')} {state}, "
              f"current={sensor.get('opt_detail')}")
        for opt_id, detail in sorted((sensor.get("opts") or {}).items()):
            mark = "*" if str(sensor.get("opt_id")) == str(opt_id) else " "
            print(f"  {mark} --resolution {opt_id}   {detail}")

    models = client.models()
    if not models:
        print("Models: none reported")
    for model in models:
        size = int(model.get("size") or 0)
        print(f"Model slot {model.get('id')}: "
              + (f"{size} bytes" if size else "empty - --detect will not work"))
    return 0


def run(args: argparse.Namespace) -> int:
    port = args.port or find_port()
    labels = load_labels(args.labels)

    print(f"Opening {port} at {args.baud} baud...")

    with SSCMAClient(
        port=port,
        baudrate=args.baud,
        box_format=args.box_format,
    ) as client:
        if labels:
            client.set_labels(labels)

        if args.info:
            return print_info(client)

        info = client.device_info()
        if info:
            print("Device: " + ", ".join(f"{k}={v}" for k, v in info.items()))
        else:
            print("Warning: the board did not answer AT+NAME?. If no frames "
                  "arrive, check it is running sscma-micro firmware.")

        if args.resolution is not None:
            client.set_resolution(args.resolution)
            print(f"Sensor resolution option set to {args.resolution}.")

        if args.detect and not client.has_model():
            raise SSCMAError(
                "No model is flashed on this board, so AT+INVOKE has nothing "
                "to run. Load one with SenseCraft AI, or drop --detect for a "
                "plain camera feed."
            )

        mode = "detection (AT+INVOKE)" if args.detect else "camera feed (AT+SAMPLE)"
        print(f"Streaming {mode}. Press q or Esc in the window to stop.")

        frames = 0
        with Viewer(scale=args.scale, snapshot_dir=args.snapshot_dir) as viewer:
            for frame in client.stream(detect=args.detect):
                frames += 1
                if not viewer.show(frame):
                    break

        if frames == 0:
            print("No frames arrived. See the troubleshooting table in README.md.")
            return 1

        print(f"Stopped after {frames} frames.")
    return 0


def main(argv: list[str] | None = None) -> int:
    args = build_parser().parse_args(argv)

    if args.list_ports:
        return print_ports()

    try:
        return run(args)
    except SSCMAError as exc:
        print(f"error: {exc}", file=sys.stderr)
        return 1
    except KeyboardInterrupt:
        print("\nInterrupted.")
        return 130


if __name__ == "__main__":
    sys.exit(main())
