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


def match_detections(
    left: Sequence[Detection],
    right: Sequence[Detection],
    max_row_offset_px: float = 40.0,
) -> list[tuple[Detection, Detection]]:
    """Pair up detections that plausibly show the same object.

    With the cameras side by side and level, the same object lands at roughly
    the same *height* in both images, so vertical offset is the cheapest
    discriminator. Size is used to break ties. Each detection is used once.
    """
    pairs: list[tuple[Detection, Detection]] = []
    unused_right = list(right)

    for left_det in left:
        left_cy = left_det.y + left_det.height / 2.0
        left_area = max(1, left_det.width * left_det.height)

        best: Detection | None = None
        best_cost = float("inf")

        for right_det in unused_right:
            if right_det.target != left_det.target:
                continue

            row_offset = abs((right_det.y + right_det.height / 2.0) - left_cy)
            if row_offset > max_row_offset_px:
                continue

            right_area = max(1, right_det.width * right_det.height)
            ratio = max(left_area, right_area) / min(left_area, right_area)
            if ratio > 4.0:
                continue                # wildly different sizes: not the same thing

            cost = row_offset + 8.0 * (ratio - 1.0)
            if cost < best_cost:
                best, best_cost = right_det, cost

        if best is not None:
            pairs.append((left_det, best))
            unused_right.remove(best)

    return pairs


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
