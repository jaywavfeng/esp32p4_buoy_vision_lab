#!/usr/bin/env python3
"""Capture ESP32-P4 camera frames into the board-sample dataset folder."""

from __future__ import annotations

import argparse
import time
import urllib.request
from pathlib import Path


def fetch(url: str, timeout: float = 5.0) -> bytes:
    with urllib.request.urlopen(url, timeout=timeout) as response:
        return response.read()


def main() -> None:
    parser = argparse.ArgumentParser()
    parser.add_argument("--ip", default="192.168.31.8")
    parser.add_argument("--label", required=True, choices=["coke", "sprite", "unknown"])
    parser.add_argument("--count", type=int, default=30)
    parser.add_argument("--interval", type=float, default=0.5)
    parser.add_argument("--out", default="data/board_samples")
    args = parser.parse_args()

    base_url = f"http://{args.ip}"
    try:
        fetch(f"{base_url}/api/power?cmd=wake", timeout=3.0)
    except Exception:
        pass

    out_dir = Path(args.out) / args.label
    out_dir.mkdir(parents=True, exist_ok=True)
    stamp = time.strftime("%Y%m%d_%H%M%S")

    for i in range(args.count):
        data = fetch(f"{base_url}/api/frame.jpg?ts={time.time()}", timeout=5.0)
        path = out_dir / f"{args.label}_{stamp}_{i:03d}.jpg"
        path.write_bytes(data)
        print(path)
        time.sleep(args.interval)


if __name__ == "__main__":
    main()
