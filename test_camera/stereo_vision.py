#!/usr/bin/env python3
"""Live stereo view: left eye, right eye, and the combined depth estimate.

    python stereo_vision.py --config stereo_config.yaml

Three panels in one window - left camera top-left, right camera top-right,
and a larger combined view below showing the red/cyan overlay of the two.
Each camera panel shows its own rough distance to whatever it detects; the
combined panel shows the real stereo distance, from triangulation.

Both boards must be running the **same detection model**, loaded from
SenseCraft AI - the two eyes have to agree on what they are looking at before
anything can be matched between them.

Keys:  q/Esc quit   s save a snapshot   d toggle overlays   a toggle anaglyph
"""

from __future__ import annotations

import argparse
import sys
import time
from collections import deque
from pathlib import Path

import cv2
import numpy as np

from sscma import SSCMAClient, SSCMAError, Viewer
from sscma.cameras import distance_from_height, load_custom_cameras, CUSTOM_CAMERAS_FILE
from sscma.config import StereoConfig, load_config
from sscma.stereo import StereoCapture, match_detections, triangulate

FONT = cv2.FONT_HERSHEY_SIMPLEX
COLOR_LEFT = (255, 200, 0)      # BGR cyan-ish
COLOR_RIGHT = (0, 160, 255)     # orange
COLOR_DEPTH = (80, 255, 120)    # green
COLOR_HUD = (240, 240, 240)
COLOR_WARN = (60, 60, 255)


def label(canvas, text, org, color=COLOR_HUD, scale=0.45, thickness=1):
    """Text with a dark outline, so it stays readable over any image."""
    cv2.putText(canvas, text, org, FONT, scale, (0, 0, 0), thickness + 2, cv2.LINE_AA)
    cv2.putText(canvas, text, org, FONT, scale, color, thickness, cv2.LINE_AA)


def draw_eye(frame, config_side, object_height_m, scale, color, title):
    """One camera panel, with a monocular distance guess per detection."""
    canvas = cv2.resize(frame.image, None, fx=scale, fy=scale,
                        interpolation=cv2.INTER_NEAREST)
    focal = config_side.resolved_focal_px()

    for det in frame.detections:
        x, y = int(det.x * scale), int(det.y * scale)
        w, h = int(det.width * scale), int(det.height * scale)
        cv2.rectangle(canvas, (x, y), (x + w, y + h), color, 2)

        distance = distance_from_height(focal, object_height_m, det.height)
        caption = f"{det.label} {det.score}%"
        if distance:
            caption += f"  ~{distance:.2f}m"
        label(canvas, caption, (x + 3, max(12, y - 5)), color)

    label(canvas, title, (8, 18), color, scale=0.5, thickness=1)
    label(canvas, f"{len(frame.detections)} det", (8, canvas.shape[0] - 8), color)
    return canvas


def draw_combined(pair, config, scale, anaglyph_on):
    """The wide panel: both eyes overlaid, with triangulated distances."""
    left_img, right_img = pair.left.image, pair.right.image
    if right_img.shape[:2] != left_img.shape[:2]:
        right_img = cv2.resize(right_img, (left_img.shape[1], left_img.shape[0]))

    if anaglyph_on:
        left_gray = cv2.cvtColor(left_img, cv2.COLOR_BGR2GRAY)
        right_gray = cv2.cvtColor(right_img, cv2.COLOR_BGR2GRAY)
        base = np.zeros_like(left_img)
        base[:, :, 2] = left_gray       # red   <- left eye
        base[:, :, 1] = right_gray      # green <- right eye
        base[:, :, 0] = right_gray      # blue  <- right eye
    else:
        base = cv2.addWeighted(left_img, 0.5, right_img, 0.5, 0)

    canvas = cv2.resize(base, None, fx=scale, fy=scale, interpolation=cv2.INTER_NEAREST)

    # Pass the config so mixed lenses match in angular units, not pixels.
    matches = match_detections(pair.left.detections, pair.right.detections,
                               config=config)
    results = []

    for left_det, right_det in matches:
        left_c = (left_det.x + left_det.width / 2.0, left_det.y + left_det.height / 2.0)
        right_c = (right_det.x + right_det.width / 2.0, right_det.y + right_det.height / 2.0)

        depth = triangulate(left_c, right_c, config)
        if depth is None:
            continue
        results.append((left_det, depth))

        lx, ly = int(left_c[0] * scale), int(left_c[1] * scale)
        rx, ry = int(right_c[0] * scale), int(right_c[1] * scale)

        # A line between where each eye sees it: the length is the disparity.
        cv2.line(canvas, (lx, ly), (rx, ry), COLOR_DEPTH, 1, cv2.LINE_AA)
        cv2.circle(canvas, (lx, ly), 4, COLOR_LEFT, -1, cv2.LINE_AA)
        cv2.circle(canvas, (rx, ry), 4, COLOR_RIGHT, -1, cv2.LINE_AA)

        x = int(left_det.x * scale)
        y = int(left_det.y * scale)
        w = int(left_det.width * scale)
        h = int(left_det.height * scale)
        cv2.rectangle(canvas, (x, y), (x + w, y + h), COLOR_DEPTH, 2)
        label(canvas,
              f"{left_det.label}  {depth.distance_m:.2f} m  (d={depth.disparity_px:.1f}px)",
              (x + 3, max(14, y - 6)), COLOR_DEPTH, scale=0.5)

    return canvas, results


def compose(left_panel, right_panel, combined_panel, hud_lines, gap=8, margin=8):
    """Lay the three panels out: two eyes on top, combined larger below.

    The combined panel is stretched to the full width of the two eyes above
    it, so it is always the biggest thing on screen - it carries the number
    that actually matters.
    """
    top_h = max(left_panel.shape[0], right_panel.shape[0])
    top_w = left_panel.shape[1] + gap + right_panel.shape[1]

    if combined_panel.shape[1] != top_w:
        aspect = combined_panel.shape[0] / combined_panel.shape[1]
        combined_panel = cv2.resize(
            combined_panel, (top_w, max(1, int(round(top_w * aspect)))),
            interpolation=cv2.INTER_NEAREST,
        )

    hud_h = 22 * len(hud_lines) + 16
    height = margin + top_h + gap + combined_panel.shape[0] + hud_h + margin

    canvas = np.full((height, top_w + 2 * margin, 3), 24, np.uint8)

    canvas[margin:margin + left_panel.shape[0],
           margin:margin + left_panel.shape[1]] = left_panel
    x_right = margin + left_panel.shape[1] + gap
    canvas[margin:margin + right_panel.shape[0],
           x_right:x_right + right_panel.shape[1]] = right_panel

    y = margin + top_h + gap
    canvas[y:y + combined_panel.shape[0],
           margin:margin + combined_panel.shape[1]] = combined_panel

    y += combined_panel.shape[0] + 24
    for line in hud_lines:
        label(canvas, line, (margin + 2, y), COLOR_HUD, scale=0.45)
        y += 22

    return canvas


def run(args) -> int:
    config = load_config(args.config)
    print(f"Rig: baseline {config.baseline_m * 1000:.1f} mm, "
          f"convergence {config.convergence_deg:.1f}°, "
          f"focal {config.left.resolved_focal_px():.1f} px")
    print(f"Left  {config.left.port}  ({config.left.camera_key})")
    print(f"Right {config.right.port}  ({config.right.camera_key})")
    print(f"Usable range about {config.min_measurable_distance_m():.2f} m "
          f"to {config.max_measurable_distance_m():.1f} m")

    left_client = SSCMAClient(config.left.port, baudrate=args.baud,
                              box_format=args.box_format)
    right_client = SSCMAClient(config.right.port, baudrate=args.baud,
                               box_format=args.box_format)

    with left_client, right_client:
        models = {}
        for side, client in (("left", left_client), ("right", right_client)):
            info = client.device_info()
            print(f"  {side}: {info.get('name', 'unknown')} "
                  f"{info.get('firmware', '')}")

            if not args.detect:
                continue

            model = client.model_info()
            models[side] = model
            if model:
                classes = client.adopt_model_labels()
                print(f"    model: {model.get('model_name', 'unknown')}"
                      + (f"  classes: {', '.join(classes)}" if classes else ""))
            else:
                # AT+INVOKE is the authority on whether inference can run, so
                # warn rather than refuse - the metadata is only a hint.
                print(f"    warning: no model metadata from the {side} board; "
                      "continuing anyway")

        # Both eyes must run the same model, or detections cannot be matched
        # between them. The metadata checksum makes that checkable.
        if args.detect and models.get("left") and models.get("right"):
            left_sum = models["left"].get("checksum")
            right_sum = models["right"].get("checksum")
            if left_sum and right_sum and left_sum != right_sum:
                raise SSCMAError(
                    "The two boards are running different models:\n"
                    f"  left  {models['left'].get('model_name')} ({left_sum})\n"
                    f"  right {models['right'].get('model_name')} ({right_sum})\n"
                    "Detections cannot be matched between eyes unless both run "
                    "the same model. Flash the same one from SenseCraft AI."
                )

        capture = StereoCapture(
            left_client, right_client,
            max_skew_ms=config.max_sync_skew_ms,
            detect=args.detect,
        )

        title = "Stereo vision - left / right / combined"
        cv2.namedWindow(title, cv2.WINDOW_NORMAL)

        fps_times: deque[float] = deque(maxlen=30)
        skews: deque[float] = deque(maxlen=30)
        anaglyph_on = True
        overlays_on = True
        snapshots = 0
        snapshot_dir = Path(args.snapshot_dir)

        print("\nStreaming. q or Esc to quit, s to save, a to toggle the anaglyph.")

        with capture:
            for pair in capture.pairs(timeout=args.timeout):
                fps_times.append(time.monotonic())
                skews.append(pair.skew_ms)

                left_panel = draw_eye(pair.left, config.left, config.object_height_m,
                                      args.scale, COLOR_LEFT, "LEFT")
                right_panel = draw_eye(pair.right, config.right, config.object_height_m,
                                       args.scale, COLOR_RIGHT, "RIGHT")
                combined, results = draw_combined(
                    pair, config, args.scale * args.combined_scale,
                    anaglyph_on and overlays_on)

                fps = 0.0
                if len(fps_times) > 1 and fps_times[-1] > fps_times[0]:
                    fps = (len(fps_times) - 1) / (fps_times[-1] - fps_times[0])
                mean_skew = sum(skews) / len(skews)

                hud = [
                    f"{fps:4.1f} pairs/s    sync skew {pair.skew_ms:5.1f} ms "
                    f"(avg {mean_skew:5.1f})    baseline {config.baseline_m*1000:.0f} mm",
                ]
                if results:
                    nearest = min(results, key=lambda r: r[1].distance_m)
                    hud.append(f"nearest: {nearest[0].label} at "
                               f"{nearest[1].distance_m:.2f} m")
                elif args.detect:
                    hud.append("no object matched in both eyes - "
                               "it must be visible to BOTH cameras")
                else:
                    hud.append("--no-detect: feeds only, no depth")

                if mean_skew > config.max_sync_skew_ms * 0.8:
                    hud.append("WARNING: frames poorly synchronised; "
                               "depth on moving objects will be wrong")

                frame = compose(left_panel, right_panel, combined, hud)
                cv2.imshow(title, frame)

                if cv2.getWindowProperty(title, cv2.WND_PROP_VISIBLE) < 1:
                    break
                key = cv2.waitKey(1) & 0xFF
                if key in (ord("q"), 27):
                    break
                if key == ord("a"):
                    anaglyph_on = not anaglyph_on
                elif key == ord("d"):
                    overlays_on = not overlays_on
                elif key == ord("s"):
                    snapshot_dir.mkdir(parents=True, exist_ok=True)
                    snapshots += 1
                    path = snapshot_dir / (
                        f"stereo-{time.strftime('%Y%m%d-%H%M%S')}-{snapshots:03d}.png")
                    cv2.imwrite(str(path), frame)
                    print(f"  saved {path}")

        cv2.destroyWindow(title)
        counts = capture.frame_counts
        print(f"\nStopped. Frames received: left={counts['left']} right={counts['right']}")
    return 0


def main(argv: list[str] | None = None) -> int:
    parser = argparse.ArgumentParser(
        description="Live stereo vision from two Grove Vision AI V2 boards.",
        formatter_class=argparse.RawDescriptionHelpFormatter,
        epilog="Keys:  q/Esc quit   s snapshot   d overlays   a anaglyph",
    )
    parser.add_argument("--config", type=Path, default=Path("stereo_config.yaml"),
                        help="Rig config from calibrate_stereo.py.")
    parser.add_argument("--baud", type=int, default=921600)
    parser.add_argument("--scale", type=float, default=1.6,
                        help="Upscale for the two camera panels.")
    parser.add_argument("--combined-scale", type=float, default=1.6,
                        help="Extra upscale for the combined panel.")
    parser.add_argument("--no-detect", dest="detect", action="store_false",
                        help="Stream both feeds without a model; no depth.")
    parser.add_argument("--box-format", choices=("center", "corner"), default="center")
    parser.add_argument("--timeout", type=float, default=15.0,
                        help="Give up if no synchronised pair arrives in this long.")
    parser.add_argument("--snapshot-dir", type=Path, default=Path("snapshots"))
    parser.add_argument("--custom-file", type=Path, default=Path(CUSTOM_CAMERAS_FILE))
    args = parser.parse_args(argv)

    if not args.config.is_file():
        print(f"error: no config at {args.config}", file=sys.stderr)
        print("Run this first:  python calibrate_stereo.py", file=sys.stderr)
        return 1

    try:
        load_custom_cameras(args.custom_file)
        return run(args)
    except (SSCMAError, ValueError, KeyError) as exc:
        print(f"error: {exc}", file=sys.stderr)
        return 1
    except KeyboardInterrupt:
        print("\nInterrupted.")
        return 130


if __name__ == "__main__":
    sys.exit(main())
