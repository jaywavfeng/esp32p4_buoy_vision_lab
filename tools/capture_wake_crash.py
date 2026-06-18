#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""触发 Wake 并抓取固定时间窗口串口日志。

这是调试工具，不参与固件构建。它解决了 PowerShell 一行脚本难以写循环的问题：
先打开串口，再通过 HTTP 请求触发 `/api/power?cmd=wake`，随后按固定秒数抓取日志。
"""

from __future__ import annotations

import argparse
import socket
import time

import serial


def http_get(host: str, path: str, timeout: float = 5.0) -> bytes:
    request = f"GET {path} HTTP/1.0\r\nHost: {host}\r\n\r\n".encode()
    with socket.create_connection((host, 80), timeout=timeout) as sock:
        sock.settimeout(timeout)
        sock.sendall(request)
        try:
            return sock.recv(4096)
        except TimeoutError:
            return b""


def wait_http_ready(host: str, seconds: float = 20.0) -> None:
    """串口打开可能触发开发板复位；等待 Web 恢复后再发送 Wake。"""
    deadline = time.time() + seconds
    last_error: Exception | None = None
    while time.time() < deadline:
        try:
            http_get(host, "/api/status", timeout=2.0)
            return
        except Exception as exc:  # noqa: BLE001 - 调试脚本需要保留最后一次异常
            last_error = exc
            time.sleep(0.5)
    raise TimeoutError(f"等待 Web 服务恢复超时: {last_error!r}")


def main() -> None:
    parser = argparse.ArgumentParser(description="Capture serial log while triggering wake.")
    parser.add_argument("--port", default="COM3")
    parser.add_argument("--baud", type=int, default=115200)
    parser.add_argument("--host", default="192.168.31.7")
    parser.add_argument("--seconds", type=float, default=18.0)
    args = parser.parse_args()

    ser = serial.Serial(args.port, args.baud, timeout=0.1)
    try:
        time.sleep(0.2)
        wait_http_ready(args.host)
        response = http_get(args.host, "/api/power?cmd=wake")
        print(response.decode("utf-8", "replace"), flush=True)

        deadline = time.time() + args.seconds
        chunks: list[bytes] = []
        while time.time() < deadline:
            data = ser.read(4096)
            if data:
                chunks.append(data)
            else:
                time.sleep(0.05)
        print(b"".join(chunks).decode("utf-8", "replace"), flush=True)
    finally:
        ser.close()


if __name__ == "__main__":
    main()
