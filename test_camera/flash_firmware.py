#!/usr/bin/env python3
"""Reflash a Grove Vision AI V2 over the bootloader's X-Modem receiver.

Use this when `diagnose.py` reports a board booting the wrong flash slot - it
runs some other image (often Himax factory test firmware) and will never
produce camera frames. SenseCraft cannot fix that itself, because its connect
flow needs the very firmware that is missing.

The bootloader is untouched by this, so a failed transfer leaves the board
exactly as recoverable as it was.

    python flash_firmware.py --image grove_vision_ai_v2_20250102.img --port COM3

Official images: https://github.com/Seeed-Studio/sscma-example-we2/releases

If the board will not drop into the bootloader on its own, hold the **BOOT**
button while plugging the USB-C cable in, then release it, and run this again.
"""

from __future__ import annotations

import argparse
import sys
import time
from pathlib import Path

import serial

from sscma import DEFAULT_BAUDRATE, SSCMAClient, SSCMAError

SOH = 0x01          # 128-byte packet
EOT = 0x04          # end of transmission
ACK = 0x06
NAK = 0x15
CAN = 0x18          # cancel
CRC_REQUEST = ord("C")

PACKET_SIZE = 128
MAX_RETRIES = 10
MAX_IMAGE_BYTES = 1024 * 1024   # bootloader accepts at most 1 MB


def crc16_xmodem(data: bytes) -> int:
    """CRC-16/XMODEM: polynomial 0x1021, initial value 0."""
    crc = 0
    for byte in data:
        crc ^= byte << 8
        for _ in range(8):
            crc = ((crc << 1) ^ 0x1021) & 0xFFFF if crc & 0x8000 else (crc << 1) & 0xFFFF
    return crc


class FlashError(RuntimeError):
    pass


def open_quiet(port: str, baud: int) -> serial.Serial:
    ser = serial.Serial()
    ser.port = port
    ser.baudrate = baud
    ser.timeout = 0.05
    ser.write_timeout = 5.0
    ser.dtr = False
    ser.rts = False
    ser.open()
    return ser


def read_for(ser: serial.Serial, seconds: float) -> bytes:
    buf = bytearray()
    end = time.monotonic() + seconds
    while time.monotonic() < end:
        waiting = ser.in_waiting
        if waiting:
            buf += ser.read(waiting)
        else:
            time.sleep(0.005)
    return bytes(buf)


def show(label: str, raw: bytes) -> None:
    text = raw.decode("ascii", "replace").replace("�", ".").strip()
    print(f"  [{label}] {text[:400] if text else '(silence)'}")


def enter_bootloader(ser: serial.Serial, verbose: bool = True) -> bytes:
    """Interrupt the boot window, then select the X-Modem receiver.

    The banner offers roughly 100 ms to press any key, so we hold a key down
    across the window rather than trying to hit it once. Menu option '1' is
    the receive mode.
    """
    banner = bytearray()
    end = time.monotonic() + 1.5
    while time.monotonic() < end:
        try:
            ser.write(b"a")
        except serial.SerialTimeoutException:
            pass
        waiting = ser.in_waiting
        if waiting:
            banner += ser.read(waiting)
        time.sleep(0.005)

    if verbose:
        show("boot window", bytes(banner))

    ser.reset_input_buffer()
    ser.write(b"1\r\n")
    ser.flush()
    menu = read_for(ser, 1.5)
    if verbose:
        show("after selecting 1", menu)
    return bytes(banner) + menu


def await_crc_request(ser: serial.Serial, timeout: float = 12.0) -> None:
    """Wait for the receiver's 'C', which means it is ready for CRC packets."""
    end = time.monotonic() + timeout
    while time.monotonic() < end:
        chunk = ser.read(ser.in_waiting or 1)
        if CRC_REQUEST in chunk:
            return
        if chunk and CAN in chunk:
            raise FlashError("The bootloader cancelled the transfer.")
    raise FlashError(
        "The bootloader never asked for data (no 'C' handshake). It is "
        "probably not in X-Modem receive mode. Hold the BOOT button while "
        "plugging the cable in, then run this again."
    )


def send_packet(ser: serial.Serial, block: int, chunk: bytes) -> None:
    payload = chunk.ljust(PACKET_SIZE, b"\x1a")   # X-Modem pads with SUB
    frame = bytes([SOH, block & 0xFF, 0xFF - (block & 0xFF)]) + payload
    crc = crc16_xmodem(payload)
    frame += bytes([(crc >> 8) & 0xFF, crc & 0xFF])

    for attempt in range(MAX_RETRIES):
        ser.write(frame)
        ser.flush()
        deadline = time.monotonic() + 2.0
        while time.monotonic() < deadline:
            reply = ser.read(1)
            if not reply:
                continue
            if reply[0] == ACK:
                return
            if reply[0] == NAK:
                break                      # resend
            if reply[0] == CAN:
                raise FlashError(f"Receiver cancelled at block {block}.")
        # fall through to retry
    raise FlashError(f"Block {block} was not acknowledged after {MAX_RETRIES} tries.")


def send_image(ser: serial.Serial, image: bytes) -> None:
    total = (len(image) + PACKET_SIZE - 1) // PACKET_SIZE
    print(f"  sending {len(image)} bytes as {total} packets...")

    started = time.monotonic()
    for index in range(total):
        chunk = image[index * PACKET_SIZE:(index + 1) * PACKET_SIZE]
        send_packet(ser, index + 1, chunk)
        if (index + 1) % 250 == 0 or index + 1 == total:
            pct = (index + 1) * 100 // total
            print(f"    {pct:3}%  ({index + 1}/{total})")

    # EOT, repeated once because some receivers NAK the first one.
    for _ in range(2):
        ser.write(bytes([EOT]))
        ser.flush()
        reply = b""
        deadline = time.monotonic() + 3.0
        while time.monotonic() < deadline and not reply:
            reply = ser.read(1)
        if reply and reply[0] == ACK:
            break
    print(f"  transfer finished in {time.monotonic() - started:.1f}s")


def verify(port: str, baud: int) -> bool:
    """Ask the freshly flashed board who it is."""
    time.sleep(2.0)
    try:
        with SSCMAClient(port=port, baudrate=baud, read_timeout=0.5) as client:
            info = client.device_info()
            if not info:
                return False
            print("  device: " + ", ".join(f"{k}={v}" for k, v in info.items()))
            for sensor in client.sensors():
                print(f"  camera id={sensor.get('id')} "
                      f"state={sensor.get('state')} {sensor.get('opt_detail')}")
            return True
    except (SSCMAError, serial.SerialException) as exc:
        print(f"  verification failed: {exc}")
        return False


def main(argv: list[str] | None = None) -> int:
    parser = argparse.ArgumentParser(
        description="Reflash a Grove Vision AI V2 via the bootloader's X-Modem receiver.",
    )
    parser.add_argument("--image", type=Path, required=True, help="Firmware .img file.")
    parser.add_argument("--port", required=True, help="Serial port, e.g. COM3.")
    parser.add_argument("--baud", type=int, default=DEFAULT_BAUDRATE)
    parser.add_argument(
        "--dry-run",
        action="store_true",
        help="Enter the bootloader and confirm it is ready, but write nothing.",
    )
    args = parser.parse_args(argv)

    if not args.image.is_file():
        print(f"error: no such image: {args.image}", file=sys.stderr)
        return 1

    image = args.image.read_bytes()
    if not image:
        print("error: image is empty", file=sys.stderr)
        return 1
    if len(image) > MAX_IMAGE_BYTES:
        print(f"error: image is {len(image)} bytes; the bootloader accepts at "
              f"most {MAX_IMAGE_BYTES}", file=sys.stderr)
        return 1
    if not image.startswith(b"ckBS"):
        print("warning: image does not start with the expected 'ckBS' Himax "
              "header - check you have the right file.", file=sys.stderr)

    print(f"Image : {args.image.name}  ({len(image)} bytes)")
    print(f"Port  : {args.port} @ {args.baud}")

    try:
        ser = open_quiet(args.port, args.baud)
    except serial.SerialException as exc:
        print(f"error: could not open {args.port}: {exc}", file=sys.stderr)
        print("Close anything else holding the port - a SenseCraft tab in "
              "Chrome keeps it open even when it fails to connect.",
              file=sys.stderr)
        return 1

    try:
        with ser:
            print("\nEntering bootloader...")
            enter_bootloader(ser)

            print("\nWaiting for the X-Modem receiver...")
            await_crc_request(ser)
            print("  receiver is ready ('C' handshake seen)")

            if args.dry_run:
                print("\n--dry-run: cancelling without writing anything.")
                ser.write(bytes([CAN, CAN, CAN]))
                ser.flush()
                return 0

            print("\nFlashing (do not unplug the board)...")
            send_image(ser, image)

            print("\nRebooting...")
            ser.write(b"y\r\n")
            ser.flush()
            show("reboot", read_for(ser, 4.0))
    except FlashError as exc:
        print(f"\nerror: {exc}", file=sys.stderr)
        print("The bootloader was not modified, so the board is no worse off.",
              file=sys.stderr)
        return 1
    except serial.SerialException as exc:
        print(f"\nerror: serial failure: {exc}", file=sys.stderr)
        return 1

    print("\nVerifying...")
    if verify(args.port, args.baud):
        print("\nSUCCESS - the board answers as an SSCMA device.")
        return 0
    print("\nThe board did not answer yet. Unplug it, plug it back in, then run:")
    print(f"  python diagnose.py {args.port}")
    return 1


if __name__ == "__main__":
    sys.exit(main())
