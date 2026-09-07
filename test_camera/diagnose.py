#!/usr/bin/env python3
"""Work out what a Grove Vision AI V2 board is actually doing.

When a board never produces frames no matter which camera or cable you try,
the fault is usually not the camera - it is that the board is not running the
sscma-micro application at all. This prints the bootloader banner and scans
baud rates so you can see which firmware actually booted.

    python diagnose.py            # every port
    python diagnose.py COM3       # one port

Reading only: it opens ports and listens, and sends nothing but a harmless
identity query.
"""

from __future__ import annotations

import sys
import time

import serial

from sscma import DEFAULT_BAUDRATE, available_ports

#: Rates worth trying. sscma-micro uses 921600; Himax factory images and
#: bootloader consoles are commonly at 115200.
SCAN_BAUDS = (115200, 230400, 460800, 921600, 1000000)

BOOT_WINDOW = 5.0


def collect(ser: serial.Serial, seconds: float) -> bytes:
    buf = bytearray()
    end = time.monotonic() + seconds
    while time.monotonic() < end:
        waiting = ser.in_waiting
        if waiting:
            buf += ser.read(waiting)
        else:
            time.sleep(0.01)
    return bytes(buf)


def open_quiet(port: str, baud: int) -> serial.Serial:
    """Open without pulsing DTR/RTS, which on some boards drives reset."""
    ser = serial.Serial()
    ser.port = port
    ser.baudrate = baud
    ser.timeout = 0.2
    ser.dtr = False
    ser.rts = False
    ser.open()
    return ser


def readable_ratio(raw: bytes) -> float:
    if not raw:
        return 0.0
    printable = sum(1 for b in raw if 32 <= b < 127 or b in (9, 10, 13))
    return printable / len(raw)


def boot_banner(port: str) -> bytes:
    """Whatever the board says on its own at the sscma-micro baud rate."""
    try:
        with open_quiet(port, DEFAULT_BAUDRATE) as ser:
            return collect(ser, BOOT_WINDOW)
    except serial.SerialException as exc:
        print(f"  cannot open at {DEFAULT_BAUDRATE}: {exc}")
        return b""


def baud_scan(port: str) -> dict[int, bytes]:
    results: dict[int, bytes] = {}
    for baud in SCAN_BAUDS:
        try:
            with open_quiet(port, baud) as ser:
                # Let the board finish booting, or the query lands mid-banner
                # and comes back mangled.
                time.sleep(0.5)
                ser.reset_input_buffer()
                ser.write(b"AT+NAME?\r\n")
                ser.flush()
                results[baud] = collect(ser, 0.8)
        except serial.SerialException:
            results[baud] = b""
    return results


def verdict(banner: bytes, scan: dict[int, bytes]) -> list[str]:
    """Turn the raw evidence into something actionable."""
    text = banner.decode("ascii", "replace")
    notes: list[str] = []

    if b'"name"' in scan.get(DEFAULT_BAUDRATE, b"") or '"name"' in text:
        notes.append("HEALTHY: the board answers SSCMA JSON at 921600.")
        return notes

    if "1st BL" in text:
        notes.append("The Himax bootloader ran, so the board and USB link are fine.")

    offset = None
    for line in text.splitlines():
        if "slot flash_offset" in line:
            offset = line.split("slot flash_offset", 1)[1].strip()
    if offset:
        notes.append(f"Bootloader jumped to flash slot at offset {offset}.")
        if offset.startswith("0x00000000"):
            notes.append(
                "  A healthy sscma-micro board boots slot 0x00100000. Offset 0 "
                "is a different image - typically Himax factory test firmware, "
                "which has no camera application at all."
            )

    talky = [b for b, raw in scan.items() if readable_ratio(raw) > 0.8 and raw]
    if talky:
        notes.append(
            "A readable console answered at "
            + ", ".join(str(b) for b in talky)
            + " but did not reply to AT+NAME? with JSON, so it is not sscma-micro."
        )

    if not notes:
        notes.append("The board said nothing at any rate tried.")

    notes.append(
        "FIX: reflash sscma-micro. See the recovery section of README.md."
    )
    return notes


def diagnose(port: str) -> None:
    print(f"\n{'=' * 66}\n{port}\n{'=' * 66}")

    banner = boot_banner(port)
    print(f"\nBoot banner @ {DEFAULT_BAUDRATE} ({len(banner)} bytes):")
    print("  " + (banner.decode("ascii", "replace").replace("\ufffd", ".")
                  .strip().replace("\n", "\n  ") or "(silence)"))

    print("\nBaud scan (sent AT+NAME?):")
    scan = baud_scan(port)
    for baud, raw in scan.items():
        preview = raw.decode("ascii", "replace").replace("\ufffd", ".")
        preview = preview.replace("\r", "\\r").replace("\n", "\\n")[:80]
        print(f"  {baud:>8} : {len(raw):5}B  readable={readable_ratio(raw):4.0%}  {preview!r}")

    print("\nVerdict:")
    for note in verdict(banner, scan):
        print(f"  {note}")


def main(argv: list[str]) -> int:
    ports = argv[1:] or [p.device for p in available_ports()]
    if not ports:
        print("No serial ports found.")
        return 1
    for port in ports:
        diagnose(port)
    return 0


if __name__ == "__main__":
    sys.exit(main(sys.argv))
