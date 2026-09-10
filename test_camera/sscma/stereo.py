"""Synchronised capture from two boards, and depth from the resulting pairs.

**On synchronisation.** These are two independent boards on two USB serial
links. There is no shared trigger and no hardware sync, so frames cannot be
captured at the same instant - only *paired* after the fact by arrival time.
:class:`StereoCapture` timestamps every frame as it arrives and pairs each
left frame with the nearest right frame, rejecting pairs further apart than
``max_sync_skew_ms``.

Measured on two Grove Vision AI V2 boards at 240x240 (~14.7 pairs/s), skew sat
at **46-48 ms with almost no spread** - median and maximum within 1 ms of each
other. That flatness matters: it is not jitter that better pairing could
average away, it is a near-constant phase offset between the two capture
loops, roughly half a frame period. Widening ``search_depth`` from 1 to 3
bought only ~1.6 ms, and 5 made it worse by dragging in staler frames.

So treat ~half a frame period as the floor. It is fine for a slow-moving
robot. It is not fine for anything fast: the object really did move during
those 48 ms, so it sits at genuinely different places in the two frames and
the disparity - and therefore the distance - comes out wrong. Faster frames
(lower resolution) shrink the offset; only a hardware trigger removes it.

**On depth.** :func:`triangulate` intersects the two viewing rays in 3D rather
than using ``Z = f*B/d``. The simple formula assumes perfectly parallel
cameras; ray intersection handles a toed-in rig too, and degrades to the same
answer when the rig is parallel.
"""

from __future__ import annotations

import math
import threading
import time
from collections import deque
from dataclasses import dataclass
from typing import Iterator, Sequence

from .client import Detection, Frame, SSCMAClient, SSCMAError
from .config import StereoConfig

__all__ = [
    "StereoPair",
    "StereoCapture",
    "triangulate",
    "match_detections",
    "DepthResult",
]


@dataclass
class StereoPair:
    """Two frames judged close enough in time to belong together."""

    left: Frame
    right: Frame
    left_time: float
    right_time: float

    @property
    def skew_ms(self) -> float:
        return abs(self.left_time - self.right_time) * 1000.0


@dataclass
class DepthResult:
    """Where a matched object sits, in metres, in the rig's frame."""

    x: float
    y: float
    z: float
    disparity_px: float

    @property
    def distance_m(self) -> float:
        """Straight-line range from the midpoint between the two cameras."""
        return math.sqrt(self.x * self.x + self.y * self.y + self.z * self.z)


def _ray(u: float, v: float, fx: float, fy: float,
         cx: float, cy: float, yaw_rad: float) -> tuple[float, float, float]:
    """Unit viewing ray for a pixel, rotated into the rig frame.

    Camera axes follow the ROS optical convention: X right, Y down, Z forward.
    ``yaw_rad`` rotates about Y, which is how a toed-in camera is aimed.
    """
    dx = (u - cx) / fx
    dy = (v - cy) / fy
    dz = 1.0

    norm = math.sqrt(dx * dx + dy * dy + dz * dz)
    dx, dy, dz = dx / norm, dy / norm, dz / norm

    cos_y, sin_y = math.cos(yaw_rad), math.sin(yaw_rad)
    return (cos_y * dx + sin_y * dz, dy, -sin_y * dx + cos_y * dz)


def triangulate(
    left_px: tuple[float, float],
    right_px: tuple[float, float],
    config: StereoConfig,
) -> DepthResult | None:
    """Intersect the two viewing rays. None when they do not meet in front.

    The rays almost never meet exactly, so this returns the midpoint of their
    closest approach - the standard least-squares answer for two rays.
    """
    half_base = config.baseline_m / 2.0
    half_conv = math.radians(config.convergence_deg) / 2.0

    fx_l = fy_l = config.left.resolved_focal_px()
    cx_l, cy_l = config.left.principal_point
    fx_r = fy_r = config.right.resolved_focal_px()
    cx_r, cy_r = config.right.principal_point

    # Cameras sit either side of the origin, toed in by half the convergence.
    origin_l = (-half_base, 0.0, 0.0)
    origin_r = (half_base, 0.0, 0.0)
    dir_l = _ray(left_px[0], left_px[1], fx_l, fy_l, cx_l, cy_l, half_conv)
    dir_r = _ray(right_px[0], right_px[1], fx_r, fy_r, cx_r, cy_r, -half_conv)

    # Closest approach of two rays: solve the 2x2 normal equations.
    w0 = tuple(origin_l[i] - origin_r[i] for i in range(3))
    b = sum(dir_l[i] * dir_r[i] for i in range(3))
    d = sum(dir_l[i] * w0[i] for i in range(3))
    e = sum(dir_r[i] * w0[i] for i in range(3))

    denom = 1.0 - b * b
    if abs(denom) < 1e-9:
        return None                      # parallel rays: object at infinity

    t_l = (b * e - d) / denom
    t_r = (e - b * d) / denom
    if t_l <= 0 or t_r <= 0:
        return None                      # behind one of the cameras

    point_l = tuple(origin_l[i] + t_l * dir_l[i] for i in range(3))
    point_r = tuple(origin_r[i] + t_r * dir_r[i] for i in range(3))
    x, y, z = ((point_l[i] + point_r[i]) / 2.0 for i in range(3))

    if z <= 0:
        return None

    disparity = (left_px[0] - cx_l) - (right_px[0] - cx_r)
    return DepthResult(x=x, y=y, z=z, disparity_px=disparity)


def _normalised(det: Detection, camera) -> tuple[float, float, float]:
    """Detection centre and height as angles, not pixels.

    Dividing by the camera's own focal length removes the lens from the
    numbers, so a 3.6 mm and a 1.7 mm camera looking at the same object
    produce comparable values. Without this, mixed lenses never match: the
    same object is roughly (3.6/1.7) = 2.1x wider in the narrower lens, which
    is a 4.5x area ratio - past any sane size test.
    """
    focal = camera.resolved_focal_px()
    cx, cy = camera.principal_point
    centre_y = (det.y + det.height / 2.0 - cy) / focal
    centre_x = (det.x + det.width / 2.0 - cx) / focal
    return centre_x, centre_y, det.height / focal


def match_detections(
    left: Sequence[Detection],
    right: Sequence[Detection],
    max_row_offset_px: float = 40.0,
    config: StereoConfig | None = None,
    max_row_offset_deg: float = 9.0,
    max_size_ratio: float = 3.0,
) -> list[tuple[Detection, Detection]]:
    """Pair up detections that plausibly show the same object.

    With the cameras side by side and level, the same object lands at roughly
    the same *height* in both images, so vertical offset is the cheapest
    discriminator, with size breaking ties. Each detection is used once.

    Pass ``config`` when the two cameras have **different lenses**. Matching
    then happens in angular units rather than pixels, which is the only way a
    mixed rig can work; ``max_row_offset_px`` is ignored in that mode in
    favour of ``max_row_offset_deg``.
    """
    pairs: list[tuple[Detection, Detection]] = []
    unused_right = list(right)

    angular = config is not None
    row_limit = math.tan(math.radians(max_row_offset_deg)) if angular else max_row_offset_px

    for left_det in left:
        if angular:
            _, left_row, left_size = _normalised(left_det, config.left)
        else:
            left_row = left_det.y + left_det.height / 2.0
            left_size = float(max(1, left_det.height))

        best: Detection | None = None
        best_cost = float("inf")

        for right_det in unused_right:
            if right_det.target != left_det.target:
                continue

            if angular:
                _, right_row, right_size = _normalised(right_det, config.right)
            else:
                right_row = right_det.y + right_det.height / 2.0
                right_size = float(max(1, right_det.height))

            row_offset = abs(right_row - left_row)
            if row_offset > row_limit:
                continue

            ratio = (max(left_size, right_size) / min(left_size, right_size)
                     if min(left_size, right_size) > 0 else float("inf"))
            if ratio > max_size_ratio:
                continue            # too different in angular size to be one object

            # Normalise the row term so the two modes weigh alike.
            cost = row_offset / max(row_limit, 1e-9) + 0.35 * (ratio - 1.0)
            if cost < best_cost:
                best, best_cost = right_det, cost

        if best is not None:
            pairs.append((left_det, best))
            unused_right.remove(best)

    return pairs


def refine_match(
    left_image,
    right_image,
    left_det: Detection,
    right_det: Detection,
    config: StereoConfig,
    search_px: float = 28.0,
    min_score: float = 0.35,
) -> tuple[float, float, float] | None:
    """Locate the object in the right image to sub-pixel accuracy.

    The model reports bounding boxes as whole pixels, and on a short baseline
    one pixel of disparity is worth a lot of depth - 308 mm at 1.5 m on a
    240x240 fisheye. Matching the actual image content instead of the box
    corners removes most of that quantisation.

    The left patch is rescaled by ``f_right / f_left`` first, so a rig with two
    different lenses can still be correlated: without that the two patches are
    at different angular scales and never line up. The correlation peak is then
    fitted with a parabola to get a fractional-pixel position.

    Returns ``(x, y, score)`` in right-image pixels, or None when the match is
    too weak to trust - in which case the caller should fall back to the box
    centre.
    """
    import cv2
    import numpy as np

    focal_left = config.left.resolved_focal_px()
    focal_right = config.right.resolved_focal_px()
    if focal_left <= 0 or focal_right <= 0:
        return None
    scale = focal_right / focal_left

    left_gray = cv2.cvtColor(left_image, cv2.COLOR_BGR2GRAY) if left_image.ndim == 3 else left_image
    right_gray = cv2.cvtColor(right_image, cv2.COLOR_BGR2GRAY) if right_image.ndim == 3 else right_image

    # Take a patch a little tighter than the box: box edges tend to sit on
    # background, which correlates poorly.
    cx = left_det.x + left_det.width / 2.0
    cy = left_det.y + left_det.height / 2.0
    half = max(6.0, 0.40 * min(left_det.width, left_det.height))

    x0, x1 = int(round(cx - half)), int(round(cx + half))
    y0, y1 = int(round(cy - half)), int(round(cy + half))
    x0, y0 = max(0, x0), max(0, y0)
    x1 = min(left_gray.shape[1], x1)
    y1 = min(left_gray.shape[0], y1)
    if x1 - x0 < 6 or y1 - y0 < 6:
        return None

    patch = left_gray[y0:y1, x0:x1]

    # A flat patch correlates perfectly with anything, so it would happily
    # "match" empty wall. Refuse to guess when there is nothing to lock onto.
    if float(patch.std()) < 6.0:
        return None

    # Bring the left patch to the right camera's angular scale, and upsample
    # both by the same factor. Correlating at higher resolution is what makes
    # the parabolic peak fit worth anything - at native scale the peak is only
    # a few pixels wide.
    upsample = 4
    new_w = max(8, int(round(patch.shape[1] * scale * upsample)))
    new_h = max(8, int(round(patch.shape[0] * scale * upsample)))
    patch = cv2.resize(patch, (new_w, new_h), interpolation=cv2.INTER_CUBIC)

    # Search a band around where the right camera already thinks it is. The
    # rig is level, so the vertical uncertainty is much smaller than the
    # horizontal one.
    pred_x = right_det.x + right_det.width / 2.0
    pred_y = right_det.y + right_det.height / 2.0
    pad_x = search_px + patch.shape[1] / (2.0 * upsample)
    pad_y = 10.0 + patch.shape[0] / (2.0 * upsample)

    sx0 = max(0, int(round(pred_x - pad_x)))
    sy0 = max(0, int(round(pred_y - pad_y)))
    sx1 = min(right_gray.shape[1], int(round(pred_x + pad_x)))
    sy1 = min(right_gray.shape[0], int(round(pred_y + pad_y)))
    window = right_gray[sy0:sy1, sx0:sx1]
    if window.size == 0 or float(window.std()) < 4.0:
        return None

    window = cv2.resize(window, (window.shape[1] * upsample, window.shape[0] * upsample),
                        interpolation=cv2.INTER_CUBIC)
    if window.shape[0] < patch.shape[0] or window.shape[1] < patch.shape[1]:
        return None

    surface = cv2.matchTemplate(window, patch, cv2.TM_CCOEFF_NORMED)
    _, best, _, best_loc = cv2.minMaxLoc(surface)
    if not np.isfinite(best) or best < min_score:
        return None

    px, py = float(best_loc[0]), float(best_loc[1])

    # Parabolic interpolation through the correlation peak, per axis. This is
    # where the sub-pixel accuracy actually comes from.
    for axis in (0, 1):
        index = int(best_loc[axis])
        limit = surface.shape[1 - axis] - 1
        if 0 < index < limit:
            if axis == 0:
                a, b, c = (float(surface[int(best_loc[1]), index + d]) for d in (-1, 0, 1))
            else:
                a, b, c = (float(surface[index + d, int(best_loc[0])]) for d in (-1, 0, 1))
            denom = a - 2.0 * b + c
            if abs(denom) > 1e-9:
                offset = 0.5 * (a - c) / denom
                if -1.0 < offset < 1.0:
                    if axis == 0:
                        px += offset
                    else:
                        py += offset

    # Back out of the upsampled window into right-image pixels.
    return (sx0 + (px + patch.shape[1] / 2.0) / upsample,
            sy0 + (py + patch.shape[0] / 2.0) / upsample,
            float(best))


class DepthSmoother:
    """Median of the last few readings, to damp per-frame jitter.

    Depth from short-baseline stereo is noisy frame to frame. A median is used
    rather than a mean because it ignores the occasional wild mismatch instead
    of being dragged by it.
    """

    def __init__(self, window: int = 5) -> None:
        self.window = max(1, window)
        self._by_key: dict[int, deque] = {}

    def update(self, key: int, value: float) -> float:
        history = self._by_key.setdefault(key, deque(maxlen=self.window))
        history.append(value)
        return sorted(history)[len(history) // 2]

    def reset(self) -> None:
        self._by_key.clear()


class StereoCapture:
    """Streams both boards concurrently and yields time-matched pairs."""

    def __init__(
        self,
        left_client: SSCMAClient,
        right_client: SSCMAClient,
        max_skew_ms: float = 60.0,
        detect: bool = True,
        buffer_size: int = 8,
        search_depth: int = 3,
    ) -> None:
        self.left_client = left_client
        self.right_client = right_client
        self.max_skew_s = max_skew_ms / 1000.0
        self.detect = detect
        #: How many recent left frames to consider when hunting for the
        #: tightest match. 1 reproduces "newest frame wins".
        self.search_depth = max(1, search_depth)

        self._buffers: dict[str, deque] = {
            "left": deque(maxlen=buffer_size),
            "right": deque(maxlen=buffer_size),
        }
        self._lock = threading.Lock()
        self._new_frame = threading.Condition(self._lock)
        self._stop = threading.Event()
        self._threads: list[threading.Thread] = []
        self._errors: dict[str, BaseException] = {}
        self._counts = {"left": 0, "right": 0}

    # -- lifecycle ---------------------------------------------------------

    def _pump(self, side: str, client: SSCMAClient) -> None:
        try:
            for frame in client.stream(detect=self.detect):
                if self._stop.is_set():
                    break
                with self._new_frame:
                    self._buffers[side].append((time.monotonic(), frame))
                    self._counts[side] += 1
                    self._new_frame.notify_all()
        except BaseException as exc:            # noqa: BLE001 - surfaced below
            self._errors[side] = exc
        finally:
            with self._new_frame:
                self._new_frame.notify_all()

    def start(self) -> "StereoCapture":
        for side, client in (("left", self.left_client), ("right", self.right_client)):
            thread = threading.Thread(
                target=self._pump, args=(side, client),
                name=f"sscma-{side}", daemon=True,
            )
            thread.start()
            self._threads.append(thread)
        return self

    def stop(self) -> None:
        self._stop.set()
        for thread in self._threads:
            thread.join(timeout=2.0)

    def __enter__(self) -> "StereoCapture":
        return self.start()

    def __exit__(self, *_exc_info) -> None:
        self.stop()

    @property
    def frame_counts(self) -> dict[str, int]:
        return dict(self._counts)

    def _raise_if_failed(self) -> None:
        for side, exc in self._errors.items():
            raise SSCMAError(f"{side} camera stopped: {exc}") from exc

    # -- pairing -----------------------------------------------------------

    def _best_pair(self, after: float = -1.0) -> StereoPair | None:
        """The tightest-matched pair available, not simply the newest.

        Always taking the newest left frame gives an average skew of about
        half a frame period, because the two streams are uncorrelated. Looking
        a few frames back instead lets us pick the left frame that happens to
        sit closest to a right one, which measurably tightens the pairing.
        ``search_depth`` bounds how far back we look, so this buys sync
        quality without unbounded latency.
        """
        rights = self._buffers["right"]
        if not self._buffers["left"] or not rights:
            return None

        best: tuple[float, float, "Frame", float, "Frame"] | None = None
        candidates = list(self._buffers["left"])[-self.search_depth:]

        for left_time, left_frame in candidates:
            if left_time <= after:
                continue
            right_time, right_frame = min(
                rights, key=lambda item: abs(item[0] - left_time)
            )
            skew = abs(left_time - right_time)
            if skew > self.max_skew_s:
                continue
            if best is None or skew < best[0]:
                best = (skew, left_time, left_frame, right_time, right_frame)

        if best is None:
            return None
        _, left_time, left_frame, right_time, right_frame = best
        return StereoPair(left_frame, right_frame, left_time, right_time)

    def pairs(self, timeout: float = 10.0) -> Iterator[StereoPair]:
        """Yield synchronised pairs until stopped."""
        last_left_time = -1.0
        deadline = time.monotonic() + timeout

        while not self._stop.is_set():
            self._raise_if_failed()

            with self._new_frame:
                self._new_frame.wait(timeout=0.2)
                pair = self._best_pair(after=last_left_time)
                is_new = pair is not None and pair.left_time > last_left_time

            if is_new:
                last_left_time = pair.left_time
                deadline = time.monotonic() + timeout
                yield pair
            elif time.monotonic() > deadline:
                counts = self.frame_counts
                raise SSCMAError(
                    "No synchronised pair in "
                    f"{timeout:.0f}s (left={counts['left']} frames, "
                    f"right={counts['right']}). If one side is 0 that board is "
                    "not streaming; if both are counting up, the frames are "
                    "arriving too far apart - raise max_sync_skew_ms."
                )
