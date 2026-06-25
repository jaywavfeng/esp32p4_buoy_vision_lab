#!/usr/bin/env python3
"""Capture ESP32 serial output for a fixed number of seconds."""

from __future__ import annotations

import argparse
import sys
import time

import serial


def reset_board(ser: serial.Serial) -> None:
    """Pulse the common ESP auto-reset lines used by dev boards."""
    ser.dtr = False
    ser.rts = True
    time.sleep(0.12)
    ser.rts = False
    time.sleep(0.25)


def main() -> int:
    parser = argparse.ArgumentParser(description="Capture serial logs without idf_monitor.")
    parser.add_argument("--port", default="COM3")
    parser.add_argument("--baud", type=int, default=115200)
    parser.add_argument("--seconds", type=float, default=20.0)
    parser.add_argument("--reset", action="store_true")
    args = parser.parse_args()

    ser = serial.Serial()
    ser.port = args.port
    ser.baudrate = args.baud
    ser.timeout = 0.1
    # CH343 boards wire DTR/RTS to auto-reset. Set both states before opening
    # the port so a log-only capture cannot reboot a board under test.
    ser.dtr = False
    ser.rts = False
    ser.open()
    with ser:
        if args.reset:
            reset_board(ser)

        deadline = time.monotonic() + args.seconds
        while time.monotonic() < deadline:
            data = ser.read(4096)
            if data:
                sys.stdout.write(data.decode("utf-8", "replace"))
                sys.stdout.flush()
            else:
                time.sleep(0.03)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
