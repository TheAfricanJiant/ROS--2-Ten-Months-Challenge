"""Camera and lens properties, and the optics needed to turn them into pixels.

Stereo depth needs a focal length **in pixels**, not millimetres. For a pinhole
camera that is::

    f_px = f_mm * image_width_px / sensor_width_mm

The catch is that the Grove Vision AI V2 does not hand you the sensor's native
frame - it crops and scales to 240x240, 480x480 or 640x480. The formula above
assumes the full sensor width is mapped onto the image width, which is a
reasonable first guess but not a measurement.

So treat every number here as **nominal**. When accuracy matters, measure the
real focal length with :func:`focal_px_from_measurement` (or
``camera_info.py --measure``), which needs no datasheet at all: show the camera
an object of known width at a known distance and read off how many pixels wide
it lands.
"""

from __future__ import annotations

import math
from dataclasses import dataclass, field

__all__ = [
    "CameraSpec",
    "CAMERAS",
    "CUSTOM_CAMERAS_FILE",
    "get_camera",
    "camera_keys",
    "load_custom_cameras",
    "save_custom_camera",
    "focal_px_from_measurement",
    "distance_from_height",
]

#: Cameras the user described themselves are merged in from here, so the
#: built-in list never becomes a limit.
CUSTOM_CAMERAS_FILE = "custom_cameras.yaml"


@dataclass(frozen=True)
class CameraSpec:
    """Physical properties of a camera module.

    Dimensions are millimetres, angles degrees. ``sensor_width_mm`` is the
    *active* sensor width, which is what sets the field of view - not the
    optical format nickname (a '1/4"' sensor is not 6.35 mm wide).
    """

    key: str
    name: str
    sensor: str
    sensor_format: str
    sensor_width_mm: float
    sensor_height_mm: float
    native_width: int
    native_height: int
    focal_length_mm: float
    lens_type: str                  # standard | wide | fisheye
    ir_capable: bool                # no IR-cut filter, so it sees near-IR
    ir_leds: bool                   # module carries its own illuminators
    image: str | None = None        # path relative to the repository root
    notes: str = ""
    vendor_fov_deg: float | None = None   # as advertised, often optimistic

    # -- field of view ------------------------------------------------------

    @staticmethod
    def _fov(extent_mm: float, focal_mm: float) -> float:
        return math.degrees(2.0 * math.atan(extent_mm / (2.0 * focal_mm)))

    @property
    def hfov_deg(self) -> float:
        return self._fov(self.sensor_width_mm, self.focal_length_mm)

    @property
    def vfov_deg(self) -> float:
        return self._fov(self.sensor_height_mm, self.focal_length_mm)

    @property
    def dfov_deg(self) -> float:
        diagonal = math.hypot(self.sensor_width_mm, self.sensor_height_mm)
        return self._fov(diagonal, self.focal_length_mm)

    @property
    def is_fisheye(self) -> bool:
        return self.lens_type == "fisheye"

    # -- pixels -------------------------------------------------------------

    def focal_px(self, image_width: int) -> float:
        """Nominal focal length in pixels for a given output width.

        Assumes the full sensor width is mapped onto ``image_width``. If the
        board crops instead of scaling, the true value is larger - measure it.
        """
        return self.focal_length_mm * image_width / self.sensor_width_mm

    def summary(self) -> dict[str, str]:
        return {
            "key": self.key,
            "name": self.name,
            "sensor": f"{self.sensor} ({self.sensor_format})",
            "sensor size": f"{self.sensor_width_mm} x {self.sensor_height_mm} mm",
            "native": f"{self.native_width}x{self.native_height}",
            "focal length": f"{self.focal_length_mm} mm",
            "lens": self.lens_type,
            "HFOV (computed)": f"{self.hfov_deg:.1f} deg",
            "VFOV (computed)": f"{self.vfov_deg:.1f} deg",
            "DFOV (computed)": f"{self.dfov_deg:.1f} deg",
            "vendor FOV": f"{self.vendor_fov_deg:.0f} deg" if self.vendor_fov_deg else "-",
            "sees IR": "yes" if self.ir_capable else "no (IR-cut filter fitted)",
            "IR LEDs": "yes" if self.ir_leds else "no",
        }


# Sensor dimensions are the active areas from the sensor datasheets; the
# optical format ('1/4"') is only a naming convention and is not a size.
_OV5647 = dict(sensor="OV5647", sensor_format='1/4"',
               sensor_width_mm=3.67, sensor_height_mm=2.74,
               native_width=2592, native_height=1944)

_IMX219 = dict(sensor="IMX219", sensor_format='1/4"',
               sensor_width_mm=3.68, sensor_height_mm=2.76,
               native_width=3280, native_height=2464)

_IMX708 = dict(sensor="IMX708", sensor_format='1/2.43"',
               sensor_width_mm=6.45, sensor_height_mm=3.63,
               native_width=4608, native_height=2592)


CAMERAS: dict[str, CameraSpec] = {
    spec.key: spec
    for spec in [
        # --- the two used for this stereo rig ---------------------------
        CameraSpec(
            key="ir-3.6mm",
            name='IR 3.6mm 1080P night-vision module',
            focal_length_mm=3.6,
            lens_type="standard",
            ir_capable=True,
            ir_leds=True,
            image="assets/images/IR3_6mm.jpg",
            vendor_fov_deg=72.0,
            notes="Lens engraved '3.6mm IR 1080P'. Two IR LEDs on the board. "
                  "The better choice of the two for stereo: much less "
                  "distortion, so the pinhole model actually holds.",
            **_OV5647,
        ),
        CameraSpec(
            key="ir-1.7mm",
            name='IR 1.7mm 5MP 1/2.5" fisheye night-vision module',
            focal_length_mm=1.7,
            lens_type="fisheye",
            ir_capable=True,
            ir_leds=True,
            image="assets/images/IR1_7mm.jpg",
            vendor_fov_deg=150.0,
            notes="Lens engraved 'IR 1.7mm 5MP 1/2.5\"'. The 1/2.5\" rating is "
                  "the lens image circle, not the sensor - on a 1/4\" sensor "
                  "only the middle of that circle is used, so the real field "
                  "of view is well short of the advertised figure. Strong "
                  "barrel distortion: the pinhole model breaks down away from "
                  "the centre, so undistort before trusting stereo depth.",
            **_OV5647,
        ),
        # --- other modules people may already own -----------------------
        CameraSpec(
            key="rpi-v1",
            name="Raspberry Pi Camera v1.3",
            focal_length_mm=3.60,
            lens_type="standard",
            ir_capable=False,
            ir_leds=False,
            image="assets/images/rpicam.jpg",
            vendor_fov_deg=53.5,
            notes="The original OV5647 module. Has an IR-cut filter, so it is "
                  "blind in the dark - fine by day, useless for the night "
                  "tests.",
            **_OV5647,
        ),
        CameraSpec(
            key="rpi-v1-noir",
            name="Raspberry Pi Camera v1.3 NoIR",
            focal_length_mm=3.60,
            lens_type="standard",
            ir_capable=True,
            ir_leds=False,
            vendor_fov_deg=53.5,
            notes="Same optics as rpi-v1 with the IR-cut filter removed. Sees "
                  "IR, but carries no illuminators - pair it with a separate "
                  "IR light source.",
            **_OV5647,
        ),
        CameraSpec(
            key="rpi-v2",
            name="Raspberry Pi Camera v2",
            focal_length_mm=3.04,
            lens_type="standard",
            ir_capable=False,
            ir_leds=False,
            vendor_fov_deg=62.2,
            **_IMX219,
        ),
        CameraSpec(
            key="rpi-v2-noir",
            name="Raspberry Pi Camera v2 NoIR",
            focal_length_mm=3.04,
            lens_type="standard",
            ir_capable=True,
            ir_leds=False,
            vendor_fov_deg=62.2,
            notes="No IR-cut filter; no on-board illuminators.",
            **_IMX219,
        ),
        CameraSpec(
            key="rpi-v3",
            name="Raspberry Pi Camera v3",
            focal_length_mm=4.74,
            lens_type="standard",
            ir_capable=False,
            ir_leds=False,
            vendor_fov_deg=66.0,
            notes="Autofocus. Check your board actually drives this sensor "
                  "before planning around it.",
            **_IMX708,
        ),
    ]
}


#: Keys that shipped with the tool, so custom entries can be told apart.
BUILTIN_KEYS = frozenset(CAMERAS)


def camera_keys() -> list[str]:
    return list(CAMERAS)


def get_camera(key: str) -> CameraSpec:
    try:
        return CAMERAS[key]
    except KeyError:
        raise KeyError(
            f"unknown camera {key!r}. Known: {', '.join(CAMERAS)}"
        ) from None


def load_custom_cameras(path: "Path | str" = CUSTOM_CAMERAS_FILE) -> int:
    """Merge user-described cameras into the registry. Returns how many.

    Missing file is not an error - most people never need one.
    """
    from pathlib import Path as _Path

    import yaml

    file = _Path(path)
    if not file.is_file():
        return 0

    raw = yaml.safe_load(file.read_text(encoding="utf-8")) or {}
    entries = raw.get("cameras", raw) if isinstance(raw, dict) else {}
    if not isinstance(entries, dict):
        raise ValueError(f"{file}: expected a mapping of camera key -> properties")

    loaded = 0
    for key, values in entries.items():
        if key in BUILTIN_KEYS:
            raise ValueError(
                f"{file}: {key!r} would shadow a built-in camera; rename it"
            )
        fields = {**values, "key": key}
        try:
            CAMERAS[key] = CameraSpec(**fields)
        except TypeError as exc:
            raise ValueError(f"{file}: camera {key!r}: {exc}") from None
        loaded += 1
    return loaded


def save_custom_camera(spec: CameraSpec, path: "Path | str" = CUSTOM_CAMERAS_FILE) -> None:
    """Append (or replace) one camera in the custom file."""
    from dataclasses import asdict
    from pathlib import Path as _Path

    import yaml

    file = _Path(path)
    raw = {}
    if file.is_file():
        raw = yaml.safe_load(file.read_text(encoding="utf-8")) or {}
    entries = raw.get("cameras") if isinstance(raw, dict) else None
    if not isinstance(entries, dict):
        entries = {}

    values = asdict(spec)
    values.pop("key")
    entries[spec.key] = values

    file.parent.mkdir(parents=True, exist_ok=True)
    with file.open("w", encoding="utf-8") as handle:
        handle.write("# Cameras added with: python camera_info.py --add\n")
        yaml.safe_dump({"cameras": entries}, handle, sort_keys=False)


def focal_px_from_measurement(
    object_width_m: float,
    distance_m: float,
    observed_width_px: float,
) -> float:
    """Focal length in pixels, measured rather than assumed.

    Show the camera something of known width at a known distance, note how
    wide it appears in the image, and the pinhole relation gives::

        f_px = observed_px * distance / real_width

    This is the number to trust: it absorbs the board's cropping and scaling,
    which no datasheet can tell you.
    """
    if object_width_m <= 0 or distance_m <= 0 or observed_width_px <= 0:
        raise ValueError("width, distance and pixel width must all be positive")
    return observed_width_px * distance_m / object_width_m


def distance_from_height(
    focal_px: float,
    real_height_m: float,
    observed_height_px: float,
) -> float | None:
    """Rough distance to an object of known size, from one camera.

    A single camera cannot measure depth; this only works because the object's
    real size is assumed. Treat it as a sanity check next to the stereo
    figure, not as a measurement.
    """
    if observed_height_px <= 0 or focal_px <= 0 or real_height_m <= 0:
        return None
    return focal_px * real_height_m / observed_height_px
