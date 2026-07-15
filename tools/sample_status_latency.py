#!/usr/bin/env python3
"""Sample ESP32-P4 /api/status latency metrics and report p95.

Example:
    python tools/sample_status_latency.py 192.168.4.1 --duration-min 1 --interval-ms 200 --method tinycls --wake --set-interval-zero
"""

from __future__ import annotations

import argparse
import csv
import http.client
import json
import math
import socket
import subprocess
import sys
import time
import urllib.error
import urllib.parse
import urllib.request
from dataclasses import dataclass
from pathlib import Path
from typing import Any


@dataclass
class Sample:
    index: int
    elapsed_ms: int
    request_ms: int
    ok: bool
    method: str = ""
    power_mode: str = ""
    network_mode: str = ""
    label: str = ""
    object_name: str = ""
    model: str = ""
    analysis_ms: int | None = None
    inference_ms: int | None = None
    inference_fps: float | None = None
    capture_fps: float | None = None
    stream_fps: float | None = None
    error: str = ""


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description="Sample board /api/status every N ms and compute analysis_ms p95."
    )
    parser.add_argument(
        "board",
        help="Board host or base URL, for example 192.168.4.1 or http://192.168.4.1",
    )
    parser.add_argument(
        "--duration-min",
        type=float,
        default=1.0,
        help="Sampling duration in minutes. Recommended range: 1 to 5. Default: 1",
    )
    parser.add_argument(
        "--interval-ms",
        type=int,
        default=200,
        help="Sampling interval in milliseconds. Default: 200",
    )
    parser.add_argument(
        "--timeout-s",
        type=float,
        default=2.0,
        help="HTTP timeout per request in seconds. Default: 2",
    )
    parser.add_argument(
        "--bind-ip",
        default="",
        help="Optional local source IPv4 address for link-local boards.",
    )
    parser.add_argument(
        "--method",
        default="",
        help="Optional recognition method to set before sampling, for example tinycls.",
    )
    parser.add_argument(
        "--wake",
        action="store_true",
        help="POST /api/power?cmd=wake before sampling.",
    )
    parser.add_argument(
        "--set-interval-zero",
        action="store_true",
        help="POST inference_interval_ms=0 to /api/config before sampling.",
    )
    parser.add_argument(
        "--output",
        default="reports/status_latency_samples.csv",
        help="CSV output path. Default: reports/status_latency_samples.csv",
    )
    return parser.parse_args()


def normalize_base_url(text: str) -> str:
    if not text.startswith(("http://", "https://")):
        text = "http://" + text
    parsed = urllib.parse.urlparse(text)
    if not parsed.netloc:
        raise ValueError(f"bad board URL: {text}")
    return f"{parsed.scheme}://{parsed.netloc}"


def make_bound_opener(bind_ip: str) -> urllib.request.OpenerDirector | None:
    if not bind_ip:
        return None

    class BoundHTTPConnection(http.client.HTTPConnection):
        def __init__(self, *args: Any, **kwargs: Any) -> None:
            kwargs["source_address"] = (bind_ip, 0)
            super().__init__(*args, **kwargs)

    class BoundHTTPHandler(urllib.request.HTTPHandler):
        def http_open(self, req: urllib.request.Request) -> Any:
            return self.do_open(BoundHTTPConnection, req)

    return urllib.request.build_opener(BoundHTTPHandler)


def detect_link_local_bind_ip(base_url: str) -> str:
    host = urllib.parse.urlparse(base_url).hostname or ""
    if not host.startswith("169.254."):
        return ""

    candidates: list[str] = []
    try:
        infos = socket.getaddrinfo(socket.gethostname(), None, socket.AF_INET)
    except OSError:
        return ""
    for info in infos:
        ip = info[4][0]
        if ip.startswith("169.254.") and ip not in candidates:
            candidates.append(ip)
    if len(candidates) == 1:
        return candidates[0]
    return ""


def open_request(
    req: urllib.request.Request,
    timeout_s: float,
    opener: urllib.request.OpenerDirector | None,
) -> Any:
    if opener is not None:
        return opener.open(req, timeout=timeout_s)
    return urllib.request.urlopen(req, timeout=timeout_s)


def read_url(
    url: str,
    timeout_s: float,
    opener: urllib.request.OpenerDirector | None,
    bind_ip: str,
    method: str = "GET",
    data: bytes | None = None,
) -> bytes:
    if bind_ip:
        curl_bin = "curl.exe" if sys.platform.startswith("win") else "curl"
        command = [
            curl_bin,
            "--interface",
            bind_ip,
            "--max-time",
            f"{timeout_s:g}",
            "--fail",
            "-sS",
            "-H",
            "Cache-Control: no-store",
        ]
        if method != "GET":
            command += ["--request", method]
        if data is not None:
            command += [
                "--header",
                "Content-Type: application/x-www-form-urlencoded",
                "--data-binary",
                data.decode("ascii"),
            ]
        command.append(url)
        result = subprocess.run(
            command,
            capture_output=True,
            timeout=max(timeout_s + 2.0, 3.0),
        )
        if result.returncode != 0:
            message = result.stderr.decode("utf-8", errors="replace").strip()
            raise RuntimeError(message or f"curl exited with {result.returncode}")
        return result.stdout

    headers = {"Cache-Control": "no-store"}
    if data is not None:
        headers["Content-Type"] = "application/x-www-form-urlencoded"
    req = urllib.request.Request(url, data=data, headers=headers, method=method)
    with open_request(req, timeout_s, opener) as response:
        return response.read()


def get_json(
    url: str,
    timeout_s: float,
    opener: urllib.request.OpenerDirector | None,
    bind_ip: str,
) -> dict[str, Any]:
    body = read_url(url, timeout_s, opener, bind_ip)
    return json.loads(body.decode("utf-8"))


def call_endpoint(
    base_url: str,
    path: str,
    timeout_s: float,
    opener: urllib.request.OpenerDirector | None,
    bind_ip: str,
    method: str = "POST",
    data: bytes | None = None,
) -> None:
    url = base_url + path
    read_url(url, timeout_s, opener, bind_ip, method=method, data=data)


def preflight_endpoint(
    base_url: str,
    path: str,
    timeout_s: float,
    label: str,
    opener: urllib.request.OpenerDirector | None,
    bind_ip: str,
    data: bytes | None = None,
) -> bool:
    last_error: BaseException | None = None
    for attempt in range(1, 4):
        try:
            call_endpoint(base_url, path, timeout_s, opener, bind_ip, data=data)
            return True
        except (RuntimeError, TimeoutError, OSError, urllib.error.URLError) as exc:
            last_error = exc
            if attempt < 3:
                time.sleep(0.5 * attempt)
    print(f"warning: {label} failed before sampling: {last_error}", file=sys.stderr)
    return False


def percentile(values: list[int], pct: float) -> int | None:
    if not values:
        return None
    ordered = sorted(values)
    if len(ordered) == 1:
        return ordered[0]
    rank = math.ceil((pct / 100.0) * len(ordered)) - 1
    rank = max(0, min(rank, len(ordered) - 1))
    return ordered[rank]


def mean(values: list[int]) -> float | None:
    if not values:
        return None
    return sum(values) / len(values)


def sample_status(
    base_url: str,
    args: argparse.Namespace,
    opener: urllib.request.OpenerDirector | None,
    bind_ip: str,
) -> list[Sample]:
    duration_s = args.duration_min * 60.0
    interval_s = args.interval_ms / 1000.0
    samples: list[Sample] = []
    start = time.monotonic()
    next_at = start
    index = 0

    while True:
        now = time.monotonic()
        if now - start >= duration_s:
            break
        if now < next_at:
            time.sleep(next_at - now)

        index += 1
        elapsed_ms = int((time.monotonic() - start) * 1000)
        request_start = time.monotonic()
        url = base_url + f"/api/status?ts={int(time.time() * 1000)}"
        try:
            data = get_json(url, args.timeout_s, opener, bind_ip)
            request_ms = int((time.monotonic() - request_start) * 1000)
            vision = data.get("vision") or {}
            samples.append(
                Sample(
                    index=index,
                    elapsed_ms=elapsed_ms,
                    request_ms=request_ms,
                    ok=True,
                    method=str(data.get("recognition_method") or ""),
                    power_mode=str(data.get("power_mode") or ""),
                    network_mode=str(data.get("network_mode") or ""),
                    label=str(vision.get("label") or ""),
                    object_name=str(vision.get("object") or ""),
                    model=str(vision.get("model") or ""),
                    analysis_ms=as_int(vision.get("analysis_ms")),
                    inference_ms=as_int(vision.get("inference_ms")),
                    inference_fps=as_x100(data.get("inference_fps_x100")),
                    capture_fps=as_x100(data.get("capture_fps_x100")),
                    stream_fps=as_x100(data.get("stream_fps_x100")),
                )
            )
        except (RuntimeError, urllib.error.URLError, TimeoutError, json.JSONDecodeError, OSError) as exc:
            request_ms = int((time.monotonic() - request_start) * 1000)
            samples.append(
                Sample(
                    index=index,
                    elapsed_ms=elapsed_ms,
                    request_ms=request_ms,
                    ok=False,
                    error=str(exc),
                )
            )

        next_at += interval_s

    return samples


def as_int(value: Any) -> int | None:
    try:
        if value is None:
            return None
        return int(value)
    except (TypeError, ValueError):
        return None


def as_x100(value: Any) -> float | None:
    raw = as_int(value)
    if raw is None:
        return None
    return raw / 100.0


def write_csv(path: Path, samples: list[Sample]) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    with path.open("w", newline="", encoding="utf-8") as f:
        writer = csv.DictWriter(
            f,
            fieldnames=[
                "index",
                "elapsed_ms",
                "request_ms",
                "ok",
                "method",
                "power_mode",
                "network_mode",
                "label",
                "object",
                "model",
                "analysis_ms",
                "inference_ms",
                "inference_fps",
                "capture_fps",
                "stream_fps",
                "error",
            ],
        )
        writer.writeheader()
        for s in samples:
            writer.writerow(
                {
                    "index": s.index,
                    "elapsed_ms": s.elapsed_ms,
                    "request_ms": s.request_ms,
                    "ok": int(s.ok),
                    "method": s.method,
                    "power_mode": s.power_mode,
                    "network_mode": s.network_mode,
                    "label": s.label,
                    "object": s.object_name,
                    "model": s.model,
                    "analysis_ms": "" if s.analysis_ms is None else s.analysis_ms,
                    "inference_ms": "" if s.inference_ms is None else s.inference_ms,
                    "inference_fps": "" if s.inference_fps is None else f"{s.inference_fps:.2f}",
                    "capture_fps": "" if s.capture_fps is None else f"{s.capture_fps:.2f}",
                    "stream_fps": "" if s.stream_fps is None else f"{s.stream_fps:.2f}",
                    "error": s.error,
                }
            )


def fmt_ms(value: int | float | None) -> str:
    if value is None:
        return "n/a"
    if isinstance(value, float):
        return f"{value:.1f} ms"
    return f"{value} ms"


def fmt_float(value: float | None) -> str:
    if value is None:
        return "n/a"
    return f"{value:.2f}"


def print_summary(samples: list[Sample], output: Path) -> None:
    ok_samples = [s for s in samples if s.ok]
    analysis = [s.analysis_ms for s in ok_samples if s.analysis_ms is not None and s.analysis_ms > 0]
    inference = [s.inference_ms for s in ok_samples if s.inference_ms is not None and s.inference_ms > 0]
    request = [s.request_ms for s in samples if s.request_ms >= 0]
    fps_values = [s.inference_fps for s in ok_samples if s.inference_fps is not None and s.inference_fps > 0]
    latest = ok_samples[-1] if ok_samples else None

    print("\n=== /api/status latency summary ===")
    print(f"samples: total={len(samples)} ok={len(ok_samples)} errors={len(samples) - len(ok_samples)}")
    print(f"latest: method={latest.method if latest else 'n/a'} power={latest.power_mode if latest else 'n/a'} "
          f"label={latest.label if latest else 'n/a'} model={latest.model if latest else 'n/a'}")
    print(f"analysis_ms: avg={fmt_ms(mean(analysis))} p50={fmt_ms(percentile(analysis, 50))} "
          f"p95={fmt_ms(percentile(analysis, 95))} p99={fmt_ms(percentile(analysis, 99))} "
          f"max={fmt_ms(max(analysis) if analysis else None)}")
    print(f"inference_ms: avg={fmt_ms(mean(inference))} p95={fmt_ms(percentile(inference, 95))} "
          f"max={fmt_ms(max(inference) if inference else None)}")
    print(f"http_request_ms: avg={fmt_ms(mean(request))} p95={fmt_ms(percentile(request, 95))} "
          f"max={fmt_ms(max(request) if request else None)}")
    print(f"inference_fps: avg={fmt_float(sum(fps_values) / len(fps_values) if fps_values else None)} "
          f"latest={fmt_float(latest.inference_fps if latest else None)}")
    print(f"target check: p95_analysis<200ms => {percentile(analysis, 95) is not None and percentile(analysis, 95) < 200}")
    print(f"target check: latest_inference_fps>=5 => {latest is not None and latest.inference_fps is not None and latest.inference_fps >= 5.0}")
    print(f"csv: {output}")


def main() -> int:
    args = parse_args()
    if args.duration_min <= 0 or args.duration_min > 5:
        print("--duration-min must be > 0 and <= 5", file=sys.stderr)
        return 2
    if args.interval_ms <= 0:
        print("--interval-ms must be > 0", file=sys.stderr)
        return 2

    base_url = normalize_base_url(args.board)
    bind_ip = args.bind_ip or detect_link_local_bind_ip(base_url)
    opener = None
    if bind_ip:
        print(f"Binding local source IP: {bind_ip}")

    if args.wake:
        preflight_endpoint(base_url, "/api/power?cmd=wake", args.timeout_s, "wake", opener, bind_ip)
        time.sleep(1.0)
    if args.method:
        method = urllib.parse.quote(args.method)
        preflight_endpoint(
            base_url,
            f"/api/recognition?method={method}",
            args.timeout_s,
            "set method",
            opener,
            bind_ip,
        )
        time.sleep(0.2)
    if args.set_interval_zero:
        preflight_endpoint(
            base_url,
            "/api/config",
            args.timeout_s,
            "set interval",
            opener,
            bind_ip,
            data=b"inference_interval_ms=0",
        )
        time.sleep(0.2)

    print(
        f"Sampling {base_url}/api/status for {args.duration_min:g} min "
        f"every {args.interval_ms} ms..."
    )
    samples = sample_status(base_url, args, opener, bind_ip)
    output = Path(args.output)
    write_csv(output, samples)
    print_summary(samples, output)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
