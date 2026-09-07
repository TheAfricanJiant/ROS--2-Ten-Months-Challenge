"""OpenCV window: draws frames, detections and a small HUD."""

from __future__ import annotations

import time
from collections import deque
from pathlib import Path

import cv2

from .client import Frame

__all__ = ["Viewer"]

_FONT = cv2.FONT_HERSHEY_SIMPLEX
_BOX_COLOR = (0, 214, 255)      # BGR amber
_TEXT_COLOR = (16, 16, 16)
_HUD_COLOR = (240, 240, 240)


class Viewer:
    """A resizable preview window with FPS and snapshot support.

    Keys: ``q`` / ``Esc`` quit, ``s`` save a snapshot, ``d`` toggle overlays.
    """

    def __init__(
        self,
        title: str = "Grove Vision AI V2",
        scale: int = 2,
        snapshot_dir: Path | str = "snapshots",
    ) -> None:
        self.title = title
        self.scale = max(1, scale)
        self.snapshot_dir = Path(snapshot_dir)
        self.show_overlays = True

        self._fps_times: deque[float] = deque(maxlen=30)
        self._snapshots = 0
        self._opened = False

    def __enter__(self) -> "Viewer":
        cv2.namedWindow(self.title, cv2.WINDOW_NORMAL)
        self._opened = True
        return self

    def __exit__(self, *_exc_info) -> None:
        self.close()

    def close(self) -> None:
        if self._opened:
            cv2.destroyWindow(self.title)
            self._opened = False

    # -- drawing -----------------------------------------------------------

    @property
    def fps(self) -> float:
        if len(self._fps_times) < 2:
            return 0.0
        span = self._fps_times[-1] - self._fps_times[0]
        return (len(self._fps_times) - 1) / span if span > 0 else 0.0

    def _draw_detections(self, canvas, frame: Frame, scale: int) -> None:
        for det in frame.detections:
            x, y = det.x * scale, det.y * scale
            w, h = det.width * scale, det.height * scale

            cv2.rectangle(canvas, (x, y), (x + w, y + h), _BOX_COLOR, 2)

            caption = f"{det.label} {det.score}%"
            (text_w, text_h), baseline = cv2.getTextSize(caption, _FONT, 0.5, 1)
            label_top = max(0, y - text_h - baseline - 4)
            cv2.rectangle(
                canvas,
                (x, label_top),
                (x + text_w + 8, label_top + text_h + baseline + 4),
                _BOX_COLOR,
                cv2.FILLED,
            )
            cv2.putText(
                canvas,
                caption,
                (x + 4, label_top + text_h + 2),
                _FONT,
                0.5,
                _TEXT_COLOR,
                1,
                cv2.LINE_AA,
            )

    def _draw_hud(self, canvas, frame: Frame) -> None:
        width, height = frame.size
        parts = [f"{width}x{height}", f"{self.fps:4.1f} fps", f"#{frame.index}"]
        if frame.inference_ms is not None:
            parts.append(f"{frame.inference_ms} ms infer")
        if frame.detections:
            parts.append(f"{len(frame.detections)} det")
        text = "   ".join(parts)

        cv2.putText(canvas, text, (9, 23), _FONT, 0.5, (0, 0, 0), 3, cv2.LINE_AA)
        cv2.putText(canvas, text, (9, 23), _FONT, 0.5, _HUD_COLOR, 1, cv2.LINE_AA)

    def render(self, frame: Frame):
        """Scale the frame up and paint overlays onto the copy."""
        self._fps_times.append(time.monotonic())

        canvas = frame.image
        if self.scale > 1:
            canvas = cv2.resize(
                canvas, None, fx=self.scale, fy=self.scale,
                interpolation=cv2.INTER_NEAREST,
            )
        else:
            canvas = canvas.copy()

        if self.show_overlays:
            self._draw_detections(canvas, frame, self.scale)
            self._draw_hud(canvas, frame)
        return canvas

    # -- interaction -------------------------------------------------------

    def _save(self, frame: Frame) -> Path:
        self.snapshot_dir.mkdir(parents=True, exist_ok=True)
        self._snapshots += 1
        path = self.snapshot_dir / (
            f"{time.strftime('%Y%m%d-%H%M%S')}-{self._snapshots:03d}.png"
        )
        cv2.imwrite(str(path), frame.image)
        return path

    def show(self, frame: Frame) -> bool:
        """Display one frame. Returns False when the user wants to quit."""
        cv2.imshow(self.title, self.render(frame))

        # Treat closing the window with the X button as a quit.
        if cv2.getWindowProperty(self.title, cv2.WND_PROP_VISIBLE) < 1:
            return False

        key = cv2.waitKey(1) & 0xFF
        if key in (ord("q"), 27):
            return False
        if key == ord("s"):
            print(f"saved {self._save(frame)}")
        elif key == ord("d"):
            self.show_overlays = not self.show_overlays
        return True
