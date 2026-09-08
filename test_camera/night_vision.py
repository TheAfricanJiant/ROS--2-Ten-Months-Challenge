#!/usr/bin/env python3
"""Live IR night-vision viewer: six ways of looking at the same dark frame.

    python night_vision.py --port PORT
    python night_vision.py --port PORT --detect --record night.avi

One window, six panels, all fed from the same camera:

  RAW          what the sensor actually sends
  ENHANCED     CLAHE - pulls detail out of the shadows
  FALSE COLOUR intensity mapped to a heat-style palette
  NIGHT VISION the classic green-phosphor look
  IR SPREAD    where the illuminators are actually throwing light
  ANALYSIS     live histogram, exposure and a night-readiness verdict

**These are not thermal cameras.** A Grove Vision AI V2 with an IR module sees
near-infrared *light reflected off things*, exactly like a normal camera sees
visible light - the IR LEDs are just a torch you cannot see. It measures
brightness, never temperature. The heat-style palette is a way of reading
brightness, not a thermal image: a cold white wall under the LEDs looks "hot",
and a warm dark jumper looks "cold". For real temperature you need a thermal
sensor such as an MLX90640.

Keys:  q/Esc quit   s snapshot   r record   p palette   1-6 focus a panel   0 grid
"""

from __future__ import annotations

import argparse
import sys
import time
from collections import deque
from pathlib import Path

import cv2
import numpy as np

from sscma import SSCMAClient, SSCMAError, find_port, port_hint

FONT = cv2.FONT_HERSHEY_SIMPLEX
INK = (235, 235, 235)
DIM = (150, 150, 150)
ACCENT = (90, 220, 255)
GOOD = (110, 230, 130)
WARN = (70, 190, 255)
BAD = (80, 80, 255)

#: Heat-style palettes, in the order `p` cycles them.
PALETTES = [
    ("INFERNO", cv2.COLORMAP_INFERNO),
    ("JET", cv2.COLORMAP_JET),
    ("TURBO", getattr(cv2, "COLORMAP_TURBO", cv2.COLORMAP_JET)),
    ("HOT", cv2.COLORMAP_HOT),
    ("MAGMA", cv2.COLORMAP_MAGMA),
    ("OCEAN", cv2.COLORMAP_OCEAN),
]

#: Pixels at or above this are blown out; at or below, nothing was recorded.
SATURATED = 250
DEAD = 6


def text(canvas, s, org, colour=INK, scale=0.42, weight=1):
    cv2.putText(canvas, s, org, FONT, scale, (0, 0, 0), weight + 2, cv2.LINE_AA)
    cv2.putText(canvas, s, org, FONT, scale, colour, weight, cv2.LINE_AA)


class NightStats:
    """Exposure numbers for one frame, and what they mean."""

    def __init__(self, gray: np.ndarray) -> None:
        self.mean = float(gray.mean())
        self.contrast = float(gray.std())
        total = gray.size
        self.saturated_pct = 100.0 * int((gray >= SATURATED).sum()) / total
        self.dead_pct = 100.0 * int((gray <= DEAD).sum()) / total
        self.p_low, self.p_high = (float(v) for v in np.percentile(gray, (1, 99)))
        self.dynamic_range = self.p_high - self.p_low

    @property
    def verdict(self) -> tuple[str, tuple[int, int, int]]:
        if self.mean < 3:
            return "NO SIGNAL - IR light not reaching the sensor", BAD
        if self.saturated_pct > 25:
            return "OVEREXPOSED - IR washing out, back off or dim", BAD
        if self.contrast < 6:
            return "FLAT - lit but no detail; move the subject closer", WARN
        if self.dead_pct > 60:
            return "MOSTLY DARK - subject beyond the IR throw", WARN
        if self.contrast > 35:
            return "EXCELLENT - strong detail in darkness", GOOD
        return "USABLE - night mode working", GOOD


def titled(image, title, subtitle=None, bar=22):
    """Put a caption bar above a panel."""
    height, width = image.shape[:2]
    canvas = np.zeros((height + bar, width, 3), np.uint8)
    canvas[bar:] = image
    canvas[:bar] = (32, 32, 32)
    text(canvas, title, (6, 15), ACCENT, 0.42, 1)
    if subtitle:
        size = cv2.getTextSize(subtitle, FONT, 0.36, 1)[0]
        text(canvas, subtitle, (width - size[0] - 6, 15), DIM, 0.36, 1)
    cv2.line(canvas, (0, bar - 1), (width, bar - 1), (60, 60, 60), 1)
    return canvas


def draw_boxes(canvas, detections, scale, colour=GOOD):
    for det in detections:
        x, y = int(det.x * scale), int(det.y * scale)
        w, h = int(det.width * scale), int(det.height * scale)
        cv2.rectangle(canvas, (x, y), (x + w, y + h), colour, 2)
        text(canvas, f"{det.label} {det.score}%", (x + 3, max(11, y - 4)), colour, 0.4)
    return canvas


# --- the six views --------------------------------------------------------

def view_raw(gray, scale):
    return cv2.resize(cv2.cvtColor(gray, cv2.COLOR_GRAY2BGR), None,
                      fx=scale, fy=scale, interpolation=cv2.INTER_NEAREST)


def view_enhanced(gray, scale, clahe):
    """CLAHE equalises locally, so shadow detail survives a bright hotspot."""
    boosted = clahe.apply(gray)
    return cv2.resize(cv2.cvtColor(boosted, cv2.COLOR_GRAY2BGR), None,
                      fx=scale, fy=scale, interpolation=cv2.INTER_NEAREST)


def view_false_colour(gray, scale, colormap):
    """Intensity -> palette. Brightness, not temperature."""
    stretched = cv2.normalize(gray, None, 0, 255, cv2.NORM_MINMAX)
    coloured = cv2.applyColorMap(stretched, colormap)
    return cv2.resize(coloured, None, fx=scale, fy=scale,
                      interpolation=cv2.INTER_NEAREST)


def view_nightvision(gray, scale):
    """Green phosphor, with a gamma lift and a little sensor grain."""
    lifted = np.power(gray.astype(np.float32) / 255.0, 0.65)
    green = np.zeros((*gray.shape, 3), np.uint8)
    green[:, :, 1] = np.clip(lifted * 255.0, 0, 255).astype(np.uint8)
    green[:, :, 0] = np.clip(lifted * 40.0, 0, 255).astype(np.uint8)

    grain = np.random.default_rng().normal(0, 6, gray.shape).astype(np.int16)
    green[:, :, 1] = np.clip(green[:, :, 1].astype(np.int16) + grain, 0, 255).astype(np.uint8)

    scaled = cv2.resize(green, None, fx=scale, fy=scale, interpolation=cv2.INTER_NEAREST)
    # Vignette, so it reads like an image intensifier tube.
    h, w = scaled.shape[:2]
    yy, xx = np.ogrid[:h, :w]
    radius = np.sqrt(((xx - w / 2) / (w / 2)) ** 2 + ((yy - h / 2) / (h / 2)) ** 2)
    mask = np.clip(1.15 - 0.55 * radius, 0, 1).astype(np.float32)
    return (scaled * mask[:, :, None]).astype(np.uint8)


def view_ir_spread(gray, scale):
    """Where the illuminators actually put light, with the scene blurred away."""
    small = cv2.resize(gray, (48, 48), interpolation=cv2.INTER_AREA)
    smooth = cv2.GaussianBlur(small.astype(np.float32), (0, 0), 6)
    smooth = cv2.normalize(smooth, None, 0, 255, cv2.NORM_MINMAX).astype(np.uint8)

    height, width = gray.shape
    field = cv2.resize(smooth, (int(width * scale), int(height * scale)),
                       interpolation=cv2.INTER_CUBIC)
    canvas = cv2.applyColorMap(field, cv2.COLORMAP_PARULA
                               if hasattr(cv2, "COLORMAP_PARULA") else cv2.COLORMAP_JET)

    # Outline what is clipped or empty at full resolution.
    for mask, colour in ((gray >= SATURATED, (255, 255, 255)), (gray <= DEAD, (40, 40, 40))):
        if not mask.any():
            continue
        big = cv2.resize(mask.astype(np.uint8) * 255,
                         (canvas.shape[1], canvas.shape[0]),
                         interpolation=cv2.INTER_NEAREST)
        contours, _ = cv2.findContours(big, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        cv2.drawContours(canvas, contours, -1, colour, 1)

    brightest = np.unravel_index(int(np.argmax(cv2.GaussianBlur(gray, (9, 9), 0))), gray.shape)
    cv2.drawMarker(canvas, (int(brightest[1] * scale), int(brightest[0] * scale)),
                   (255, 255, 255), cv2.MARKER_CROSS, 14, 1)
    return canvas


def view_analysis(gray, stats, size, history):
    """Histogram, exposure numbers and the verdict.

    Laid out from both ends: the histogram and readings grow down from the
    top, the verdict is anchored to the bottom, and the trend line only gets
    drawn if there is room left between them.
    """
    canvas = np.full((size, size, 3), 22, np.uint8)

    hist = cv2.calcHist([gray], [0], None, [64], [0, 256]).ravel()
    hist = hist / max(hist.max(), 1.0)
    plot_h, plot_top, left, right = 64, 28, 10, size - 10
    bar_w = (right - left) / len(hist)

    for i, value in enumerate(hist):
        x0 = int(left + i * bar_w)
        x1 = int(left + (i + 1) * bar_w) - 1
        y = int(plot_top + plot_h - value * plot_h)
        shade = int(30 + 225 * (i / len(hist)))
        cv2.rectangle(canvas, (x0, y), (max(x1, x0), plot_top + plot_h),
                      (shade // 3, shade // 2, shade), -1)

    cv2.rectangle(canvas, (left, plot_top), (right, plot_top + plot_h), (70, 70, 70), 1)
    text(canvas, "histogram   dark", (left, plot_top - 6), DIM, 0.34)
    text(canvas, "bright", (right - 42, plot_top - 6), DIM, 0.34)

    y = plot_top + plot_h + 20
    rows = [
        ("mean brightness", f"{stats.mean:6.1f} / 255", GOOD if stats.mean > 20 else WARN),
        ("contrast (std)", f"{stats.contrast:6.1f}", GOOD if stats.contrast > 15 else WARN),
        ("dynamic range", f"{stats.dynamic_range:6.1f}", GOOD if stats.dynamic_range > 60 else WARN),
        ("blown out", f"{stats.saturated_pct:5.1f} %", BAD if stats.saturated_pct > 10 else INK),
        ("no signal", f"{stats.dead_pct:5.1f} %", BAD if stats.dead_pct > 50 else INK),
    ]
    for name, value, colour in rows:
        text(canvas, name, (left, y), DIM, 0.4)
        text(canvas, value, (right - 78, y), colour, 0.4)
        y += 17

    # Verdict first, anchored to the bottom, so it can never be clipped off.
    verdict, colour = stats.verdict
    lines = _wrap(verdict, 32)
    block_h = 15 * len(lines) + 8
    block_top = size - block_h - 6
    cv2.rectangle(canvas, (left - 2, block_top), (right + 2, size - 6),
                  (colour[0] // 6, colour[1] // 6, colour[2] // 6), -1)
    ty = block_top + 15
    for line in lines:
        text(canvas, line, (left + 2, ty), colour, 0.4)
        ty += 15

    # Brightness trend, only if it fits between the readings and the verdict.
    available = block_top - y - 14
    if len(history) > 2 and available >= 26:
        track_h = min(32, available)
        text(canvas, "brightness over time", (left, y + 8), DIM, 0.34)
        top = y + 14
        pts = list(history)
        step = (right - left) / max(len(pts) - 1, 1)
        for i in range(len(pts) - 1):
            p0 = (int(left + i * step), int(top + track_h - pts[i] / 255 * track_h))
            p1 = (int(left + (i + 1) * step), int(top + track_h - pts[i + 1] / 255 * track_h))
            cv2.line(canvas, p0, p1, ACCENT, 1, cv2.LINE_AA)
        cv2.rectangle(canvas, (left, top), (right, top + track_h), (60, 60, 60), 1)
    return canvas


def _wrap(s, width):
    words, lines, current = s.split(), [], ""
    for word in words:
        if len(current) + len(word) + 1 > width:
            lines.append(current)
            current = word
        else:
            current = f"{current} {word}".strip()
    if current:
        lines.append(current)
    return lines


def compose_grid(panels, cols=3, gap=6, margin=8, hud=()):
    rows = (len(panels) + cols - 1) // cols
    pw = max(p.shape[1] for p in panels)
    ph = max(p.shape[0] for p in panels)

    hud_h = 22 * len(hud) + (14 if hud else 0)
    width = margin * 2 + cols * pw + (cols - 1) * gap
    height = margin * 2 + rows * ph + (rows - 1) * gap + hud_h
    canvas = np.full((height, width, 3), 18, np.uint8)

    for index, panel in enumerate(panels):
        r, c = divmod(index, cols)
        x = margin + c * (pw + gap)
        y = margin + r * (ph + gap)
        canvas[y:y + panel.shape[0], x:x + panel.shape[1]] = panel

    y = margin + rows * ph + (rows - 1) * gap + 20
    for line, colour in hud:
        text(canvas, line, (margin + 2, y), colour, 0.44)
        y += 22
    return canvas


def run(args) -> int:
    port = args.port or find_port()
    print(f"Opening {port}...")

    with SSCMAClient(port, baudrate=args.baud, box_format=args.box_format) as client:
        info = client.device_info()
        print(f"Device: {info.get('name', 'unknown')} {info.get('firmware', '')}")

        if args.detect:
            model = client.model_info()
            if model:
                classes = client.adopt_model_labels()
                print(f"Model: {model.get('model_name')}"
                      + (f"  classes: {', '.join(classes)}" if classes else ""))
            else:
                print("Warning: no model metadata; trying --detect anyway.")

        if args.resolution is not None:
            client.set_resolution(args.resolution)

        clahe = cv2.createCLAHE(clipLimit=args.clip, tileGridSize=(8, 8))
        title = "IR night vision"
        cv2.namedWindow(title, cv2.WINDOW_NORMAL)

        palette_index = 0
        focus = 0                       # 0 = grid, 1..6 = single panel
        fps_times: deque[float] = deque(maxlen=30)
        brightness: deque[float] = deque(maxlen=90)
        writer = None
        snapshots = 0

        print("\nStreaming. q/Esc quit, s snapshot, r record, p palette, 1-6 focus, 0 grid.")

        for frame in client.stream(detect=args.detect):
            fps_times.append(time.monotonic())
            gray = cv2.cvtColor(frame.image, cv2.COLOR_BGR2GRAY)
            stats = NightStats(gray)
            brightness.append(stats.mean)

            palette_name, colormap = PALETTES[palette_index]
            scale = args.scale
            side = int(gray.shape[0] * scale)

            raw = draw_boxes(view_raw(gray, scale), frame.detections, scale)
            colour = draw_boxes(view_false_colour(gray, scale, colormap),
                                frame.detections, scale, (255, 255, 255))

            panels = [
                titled(raw, "1 RAW IR", f"{gray.shape[1]}x{gray.shape[0]}"),
                titled(view_enhanced(gray, scale, clahe), "2 ENHANCED", f"CLAHE {args.clip:g}"),
                titled(colour, "3 FALSE COLOUR", f"{palette_name} - not thermal"),
                titled(view_nightvision(gray, scale), "4 NIGHT VISION", "green phosphor"),
                titled(view_ir_spread(gray, scale), "5 IR SPREAD", "illuminator falloff"),
                titled(view_analysis(gray, stats, side, brightness), "6 ANALYSIS",
                       f"{stats.dynamic_range:.0f} DR"),
            ]

            fps = 0.0
            if len(fps_times) > 1 and fps_times[-1] > fps_times[0]:
                fps = (len(fps_times) - 1) / (fps_times[-1] - fps_times[0])

            verdict, verdict_colour = stats.verdict
            hud = [
                (f"{fps:4.1f} fps   mean {stats.mean:5.1f}   contrast {stats.contrast:5.1f}"
                 f"   blown {stats.saturated_pct:4.1f}%   dark {stats.dead_pct:4.1f}%"
                 + (f"   {len(frame.detections)} det" if args.detect else ""),
                 INK),
                (verdict, verdict_colour),
            ]
            if writer is not None:
                hud.append(("REC", BAD))

            canvas = (panels[focus - 1] if focus else compose_grid(panels, hud=hud))
            cv2.imshow(title, canvas)

            if writer is not None:
                if writer[1] != canvas.shape[:2]:
                    print("  frame size changed; stopping recording")
                    writer[0].release()
                    writer = None
                else:
                    writer[0].write(canvas)

            if cv2.getWindowProperty(title, cv2.WND_PROP_VISIBLE) < 1:
                break
            key = cv2.waitKey(1) & 0xFF
            if key in (ord("q"), 27):
                break
            if key == ord("p"):
                palette_index = (palette_index + 1) % len(PALETTES)
                print(f"  palette: {PALETTES[palette_index][0]}")
            elif key in [ord(str(d)) for d in range(7)]:
                focus = int(chr(key))
            elif key == ord("s"):
                Path(args.snapshot_dir).mkdir(parents=True, exist_ok=True)
                snapshots += 1
                path = Path(args.snapshot_dir) / (
                    f"night-{time.strftime('%Y%m%d-%H%M%S')}-{snapshots:03d}.png")
                cv2.imwrite(str(path), canvas)
                print(f"  saved {path}")
            elif key == ord("r"):
                if writer is None:
                    out = Path(args.record or "night_vision.avi")
                    fourcc = cv2.VideoWriter_fourcc(*"MJPG")
                    handle = cv2.VideoWriter(str(out), fourcc, max(fps, 8.0),
                                             (canvas.shape[1], canvas.shape[0]))
                    if handle.isOpened():
                        writer = (handle, canvas.shape[:2])
                        print(f"  recording to {out}")
                    else:
                        print("  could not open the video writer")
                else:
                    writer[0].release()
                    writer = None
                    print("  recording stopped")

        if writer is not None:
            writer[0].release()
        cv2.destroyWindow(title)
    return 0


def main(argv: list[str] | None = None) -> int:
    parser = argparse.ArgumentParser(
        description="Live IR night-vision viewer with six simultaneous views.",
        formatter_class=argparse.RawDescriptionHelpFormatter,
        epilog="Keys:  q/Esc quit   s snapshot   r record   p palette   1-6 focus   0 grid",
    )
    parser.add_argument("--port", help=f"Serial port, e.g. {port_hint()}.")
    parser.add_argument("--baud", type=int, default=921600)
    parser.add_argument("--detect", action="store_true",
                        help="Run the flashed model and draw its boxes.")
    parser.add_argument("--scale", type=float, default=1.1, help="Panel upscale.")
    parser.add_argument("--clip", type=float, default=2.5,
                        help="CLAHE clip limit for the ENHANCED panel.")
    parser.add_argument("--resolution", type=int, metavar="OPT_ID",
                        help="Sensor option id to select first (see --info).")
    parser.add_argument("--box-format", choices=("center", "corner"), default="center")
    parser.add_argument("--record", metavar="FILE",
                        help="File the r key records to (default night_vision.avi).")
    parser.add_argument("--snapshot-dir", default="snapshots")
    args = parser.parse_args(argv)

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
