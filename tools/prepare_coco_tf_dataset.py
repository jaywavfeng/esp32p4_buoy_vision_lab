#!/usr/bin/env python3
"""Prepare a COCO-style video frame dataset for ESP32-P4 validation.

The board-side COCO model consumes JPEG images. This helper converts the
public sample video into an ordered frame folder that can be copied to:

    /sdcard/esp32p4/datasets/coco_video

It is also used to generate the compact firmware-embedded coco_video_demo
dataset. Use --tf-root when the TF card is mounted on the PC, or copy the
generated output directory manually before inserting the card into the board.
"""

from __future__ import annotations

import argparse
import json
import re
import shutil
from pathlib import Path


def portable_source_path(path: Path) -> str:
    try:
        return str(path.resolve().relative_to(Path.cwd().resolve()))
    except ValueError:
        return path.name


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument(
        "--input",
        default="data/coco_video/person-bicycle-car-detection.mp4",
        help="Input video path.",
    )
    parser.add_argument(
        "--output",
        default="data/tf_datasets/coco_video",
        help="Local output dataset directory.",
    )
    parser.add_argument("--dataset", default="coco_video", help="Board dataset name.")
    parser.add_argument("--source-url", default="", help="Public source URL recorded in summary.json.")
    parser.add_argument("--license-url", default="", help="Source license URL recorded in summary.json.")
    parser.add_argument("--start-frame", type=int, default=0, help="First source frame to consider.")
    parser.add_argument("--every", type=int, default=12, help="Export one frame every N input frames.")
    parser.add_argument("--max-frames", type=int, default=180, help="Maximum exported frames.")
    parser.add_argument("--width", type=int, default=512, help="Resize frame width, preserving aspect ratio.")
    parser.add_argument("--jpeg-quality", type=int, default=82, help="JPEG quality for TF dataset frames.")
    parser.add_argument(
        "--tf-root",
        default="",
        help="Optional mounted TF card root, for example E:\\\\. If set, copy dataset to esp32p4/datasets/coco_video.",
    )
    return parser.parse_args()


def export_frames(args: argparse.Namespace) -> dict:
    if not re.fullmatch(r"[A-Za-z0-9_-]{1,47}", args.dataset):
        raise SystemExit("--dataset must contain 1-47 letters, digits, underscores, or hyphens")
    if args.start_frame < 0:
        raise SystemExit("--start-frame must be non-negative")
    if args.every <= 0:
        raise SystemExit("--every must be positive")
    if args.max_frames <= 0:
        raise SystemExit("--max-frames must be positive")

    try:
        import cv2
    except ModuleNotFoundError as exc:
        raise SystemExit("opencv-python is required to export video frames") from exc

    input_path = Path(args.input)
    output_dir = Path(args.output)
    frames_dir = output_dir / "frames"
    if frames_dir.exists():
        shutil.rmtree(frames_dir)
    output_dir.mkdir(parents=True, exist_ok=True)
    frames_dir.mkdir(parents=True, exist_ok=True)

    cap = cv2.VideoCapture(str(input_path))
    if not cap.isOpened():
        raise SystemExit(f"cannot open input video: {input_path}")

    source_fps = cap.get(cv2.CAP_PROP_FPS) or 0.0
    source_frames = int(cap.get(cv2.CAP_PROP_FRAME_COUNT) or 0)
    source_w = int(cap.get(cv2.CAP_PROP_FRAME_WIDTH) or 0)
    source_h = int(cap.get(cv2.CAP_PROP_FRAME_HEIGHT) or 0)

    manifest_path = output_dir / "manifest.jsonl"
    exported = 0
    frame_idx = args.start_frame
    if frame_idx:
        cap.set(cv2.CAP_PROP_POS_FRAMES, frame_idx)
    with manifest_path.open("w", encoding="utf-8") as manifest:
        while exported < args.max_frames:
            ok, frame = cap.read()
            if not ok:
                break
            if (frame_idx - args.start_frame) % args.every != 0:
                frame_idx += 1
                continue

            if args.width > 0 and frame.shape[1] != args.width:
                scale = args.width / frame.shape[1]
                height = max(1, round(frame.shape[0] * scale))
                frame = cv2.resize(frame, (args.width, height), interpolation=cv2.INTER_AREA)

            exported += 1
            name = f"frame_{exported:05d}.jpg"
            out_path = frames_dir / name
            cv2.imwrite(str(out_path), frame, [int(cv2.IMWRITE_JPEG_QUALITY), args.jpeg_quality])
            manifest.write(
                json.dumps(
                    {
                        "index": exported,
                        "source_frame": frame_idx,
                        "source_time_s": frame_idx / source_fps if source_fps else None,
                        "file": f"frames/{name}",
                        "width": int(frame.shape[1]),
                        "height": int(frame.shape[0]),
                        "bytes": out_path.stat().st_size,
                    },
                    ensure_ascii=False,
                )
                + "\n"
            )
            frame_idx += 1

    cap.release()

    summary = {
        "dataset": args.dataset,
        "input": portable_source_path(input_path),
        "source_url": args.source_url,
        "license_url": args.license_url,
        "output": str(output_dir),
        "board_path": f"/sdcard/esp32p4/datasets/{args.dataset}",
        "source_fps": source_fps,
        "source_frames": source_frames,
        "source_size": [source_w, source_h],
        "start_frame": args.start_frame,
        "every": args.every,
        "exported_frames": exported,
        "frame_width": args.width,
        "jpeg_quality": args.jpeg_quality,
    }
    (output_dir / "summary.json").write_text(json.dumps(summary, ensure_ascii=False, indent=2), encoding="utf-8")
    (output_dir / "README_IMPORT.txt").write_text(
        f"Copy this directory to TF card path: esp32p4/datasets/{args.dataset}\r\n"
        f"Expected board absolute path: /sdcard/esp32p4/datasets/{args.dataset}\r\n"
        "The board COCO model consumes the JPEG files under frames/.\r\n",
        encoding="utf-8",
    )
    return summary


def copy_to_tf(output_dir: Path, tf_root: str, dataset: str) -> Path:
    tf_root_path = Path(tf_root)
    target = tf_root_path / "esp32p4" / "datasets" / dataset
    if target.exists():
        shutil.rmtree(target)
    target.parent.mkdir(parents=True, exist_ok=True)
    shutil.copytree(output_dir, target)
    return target


def main() -> None:
    args = parse_args()
    summary = export_frames(args)
    print(json.dumps(summary, ensure_ascii=False, indent=2))
    if args.tf_root:
        target = copy_to_tf(Path(args.output), args.tf_root, args.dataset)
        print(f"copied to TF dataset path: {target}")


if __name__ == "__main__":
    main()
