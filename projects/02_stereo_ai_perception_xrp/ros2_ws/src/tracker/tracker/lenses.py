"""Per-camera geometry: turning a pixel into an angle.

Steering is an angular problem. The tracker needs to know "how many degrees
off centre is the target", and the answer depends on the lens - the same pixel
offset means a very different angle through a 3.6 mm lens than through a
1.7 mm fisheye. Getting this wrong makes the robot oversteer with one camera
and understeer with the other.

Two projection models are implemented, because the two cameras genuinely obey
different ones:

**Rectilinear** (the 3.6 mm module, and any ordinary lens). Straight lines stay
straight, and the pinhole relation holds::

    tan(theta) = (u - cx) / f

**Equidistant** (the 1.7 mm fisheye). Fisheyes are built so that angle maps
*linearly* onto radius, which is what lets them fit 90-plus degrees onto a
sensor at all::

    theta = r / f

Using the rectilinear formula on a fisheye is fine near the middle - the two
agree to within a degree out to about 20 degrees off-axis - but it overstates
the angle badly at the edges, which is exactly where a target is when the
robot most needs to turn.
"""

from __future__ import annotations

import math
from dataclasses import dataclass

__all__ = [
    "LensModel",
    "RectilinearLens",
    "EquidistantFisheyeLens",
    "lens_for",
    "FISHEYE_KEYS",
]

#: Camera keys (from the sscma camera database) that need the fisheye model.
FISHEYE_KEYS = frozenset({"ir-1.7mm"})


@dataclass(frozen=True)
class LensModel:
    """Maps between pixels and angles for one camera.

    Angles are radians, measured from the optical axis: positive x is to the
    right of centre, positive y is below it (the ROS optical convention).
    """

    focal_px: float
    cx: float
    cy: float
    width: int
    height: int

    name: str = "lens"

    # -- to be provided by subclasses -----------------------------------

    def angles(self, u: float, v: float) -> tuple[float, float]:
        """Horizontal and vertical angle of the ray through pixel (u, v)."""
        raise NotImplementedError

    def horizontal_fov(self) -> float:
        """Total horizontal field of view, radians."""
        raise NotImplementedError

    # -- shared -----------------------------------------------------------

    def normalised_x(self, u: float) -> float:
        """Horizontal angle as a fraction of half the field of view.

        -1 is the left edge, 0 centre, +1 the right edge. This is the number
        a steering controller wants: it behaves the same on both cameras even
        though their fields of view differ by nearly a factor of two.
        """
        half_fov = self.horizontal_fov() / 2.0
        if half_fov <= 0:
            return 0.0
        angle, _ = self.angles(u, self.cy)
        return max(-1.5, min(1.5, angle / half_fov))

    def __str__(self) -> str:
        return (f"{self.name}: f={self.focal_px:.1f}px, "
                f"HFOV={math.degrees(self.horizontal_fov()):.1f}deg, "
                f"{self.width}x{self.height}")


class RectilinearLens(LensModel):
    """Ordinary lens. tan(theta) = offset / f."""

    def angles(self, u: float, v: float) -> tuple[float, float]:
        return (math.atan2(u - self.cx, self.focal_px),
                math.atan2(v - self.cy, self.focal_px))

    def horizontal_fov(self) -> float:
        return 2.0 * math.atan2(self.width / 2.0, self.focal_px)


class EquidistantFisheyeLens(LensModel):
    """Fisheye. theta = r / f, applied radially.

    The radial form matters: a fisheye compresses uniformly outward from the
    centre, so splitting the pixel offset into x and y *before* converting
    would overstate the angle for anything off the horizontal axis. Here the
    total off-axis angle is found first, then split back along the same
    direction the pixel lay in.
    """

    def angles(self, u: float, v: float) -> tuple[float, float]:
        dx = u - self.cx
        dy = v - self.cy
        radius = math.hypot(dx, dy)

        if radius < 1e-9 or self.focal_px <= 0:
            return 0.0, 0.0

        theta = radius / self.focal_px          # equidistant projection
        return theta * (dx / radius), theta * (dy / radius)

    def horizontal_fov(self) -> float:
        return 2.0 * (self.width / 2.0) / self.focal_px if self.focal_px > 0 else 0.0


def lens_for(
    camera_key: str,
    focal_px: float,
    width: int,
    height: int,
    cx: float | None = None,
    cy: float | None = None,
    force_model: str | None = None,
) -> LensModel:
    """Pick the right projection model for a camera.

    ``force_model`` accepts ``"rectilinear"`` or ``"fisheye"`` to override the
    choice, which is what you need for a camera the database has never heard
    of.
    """
    centre_x = width / 2.0 if cx is None else cx
    centre_y = height / 2.0 if cy is None else cy

    if force_model == "rectilinear":
        fisheye = False
    elif force_model == "fisheye":
        fisheye = True
    else:
        fisheye = camera_key in FISHEYE_KEYS

    model = EquidistantFisheyeLens if fisheye else RectilinearLens
    return model(
        focal_px=focal_px,
        cx=centre_x,
        cy=centre_y,
        width=width,
        height=height,
        name=f"{camera_key} ({'fisheye' if fisheye else 'rectilinear'})",
    )
