"""Run a PC-side COCO video check and prepare board validation frames.

The board runs Espressif's quantized COCO YOLO11n 320 model. This script uses
the local Ultralytics YOLO11n weights at the same input size to preview the
video-level effect on the PC, then selects dense frames for board-side JPEG
validation.
"""

from __future__ import annotations

import argparse
import json
import shutil
import time
from dataclasses import dataclass
from pathlib import Path

import cv2
from ultralytics import YOLO


ROOT = Path(__file__).resolve().parents[1]


def project_path(path: Path) -> str:
    resolved = path.resolve()
    try:
        return str(resolved.relative_to(ROOT))
    except ValueError:
        return str(resolved)


@dataclass
class FrameCandidate:
    frame_index: int
    timestamp_s: float
    score: int
    object_count: int
    class_count: int
    frame: object
    detections: list[dict]


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument(
        "--video",
        type=Path,
        default=ROOT / "data" / "coco_video" / "person-bicycle-car-detection.mp4",
        help="Input public sample video.",
    )
    parser.add_argument(
        "--weights",
        type=Path,
        default=ROOT / "yolo11n.pt",
        help="Ultralytics COCO YOLO11n weights used for PC preview.",
    )
    parser.add_argument("--imgsz", type=int, default=320)
    parser.add_argument("--conf", type=float, default=0.25)
    parser.add_argument("--iou", type=float, default=0.7)
    parser.add_argument("--device", default="cpu")
    parser.add_argument("--max-frames", type=int, default=0, help="0 means all frames.")
    parser.add_argument("--frame-stride", type=int, default=1)
    parser.add_argument("--top-k", type=int, default=4)
    parser.add_argument("--min-frame-gap", type=int, default=36)
    parser.add_argument("--board-width", type=int, default=512)
    parser.add_argument(
        "--output-video",
        type=Path,
        default=ROOT / "reports" / "coco_video" / "person_bicycle_car_yolo11n_320_annotated.mp4",
    )
    parser.add_argument(
        "--report",
        type=Path,
        default=ROOT / "reports" / "coco_video" / "prediction_summary.json",
    )
    parser.add_argument(
        "--classic-dir",
        type=Path,
        default=ROOT / "reports" / "coco_video" / "selected_frames",
    )
    parser.add_argument(
        "--firmware-dir",
        type=Path,
        default=None,
        help="Optional firmware embed path for demo_01..demo_04.jpg. Leave unset to avoid overwriting board samples.",
    )
    return parser.parse_args()


def resize_to_width(frame, width: int):
    if width <= 0 or frame.shape[1] <= width:
        return frame
    height = max(1, round(frame.shape[0] * width / frame.shape[1]))
    return cv2.resize(frame, (width, height), interpolation=cv2.INTER_AREA)


def draw_detections(frame, detections: list[dict], names: dict[int, str]):
    palette = [
        (56, 189, 248),
        (52, 211, 153),
        (251, 191, 36),
        (248, 113, 113),
        (196, 181, 253),
        (244, 114, 182),
    ]
    for i, det in enumerate(detections):
        x1, y1, x2, y2 = [int(v) for v in det["xyxy"]]
        color = palette[i % len(palette)]
        label = f"{names.get(det['class_id'], str(det['class_id']))} {det['score']:.2f}"
        cv2.rectangle(frame, (x1, y1), (x2, y2), color, 2)
        (tw, th), _ = cv2.getTextSize(label, cv2.FONT_HERSHEY_SIMPLEX, 0.52, 1)
        y0 = max(0, y1 - th - 8)
        cv2.rectangle(frame, (x1, y0), (x1 + tw + 8, y0 + th + 8), color, -1)
        cv2.putText(
            frame,
            label,
            (x1 + 4, y0 + th + 3),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.52,
            (0, 0, 0),
            1,
            cv2.LINE_AA,
        )
    return frame


def result_to_detections(result) -> list[dict]:
    detections: list[dict] = []
    if result.boxes is None:
        return detections
    for box in result.boxes:
        xyxy = box.xyxy[0].detach().cpu().numpy().tolist()
        class_id = int(box.cls[0].item())
        score = float(box.conf[0].item())
        detections.append({"xyxy": xyxy, "class_id": class_id, "score": score})
    detections.sort(key=lambda d: d["score"], reverse=True)
    return detections


def candidate_score(detections: list[dict]) -> tuple[int, int, int]:
    classes = {d["class_id"] for d in detections}
    object_count = len(detections)
    class_count = len(classes)
    # Prefer frames that are dense, diverse, and include common COCO traffic classes.
    useful = {0, 1, 2, 3, 5, 7}  # person, bicycle, car, motorcycle, bus, truck
    useful_hits = sum(1 for d in detections if d["class_id"] in useful)
    return object_count * 10 + class_count * 25 + useful_hits * 5, object_count, class_count


def select_candidates(candidates: list[FrameCandidate], top_k: int, min_gap: int) -> list[FrameCandidate]:
    selected: list[FrameCandidate] = []
    for candidate in sorted(candidates, key=lambda c: (c.score, c.object_count, c.class_count), reverse=True):
        if all(abs(candidate.frame_index - picked.frame_index) >= min_gap for picked in selected):
            selected.append(candidate)
            if len(selected) >= top_k:
                break
    return sorted(selected, key=lambda c: c.frame_index)


def save_board_images(selected: list[FrameCandidate], args: argparse.Namespace, names: dict[int, str]) -> list[dict]:
    args.classic_dir.mkdir(parents=True, exist_ok=True)
    if args.firmware_dir:
        args.firmware_dir.mkdir(parents=True, exist_ok=True)
    saved: list[dict] = []
    for i, item in enumerate(selected, start=1):
        raw = resize_to_width(item.frame, args.board_width)
        classic_path = args.classic_dir / f"video_{i:02d}.jpg"
        cv2.imwrite(str(classic_path), raw, [int(cv2.IMWRITE_JPEG_QUALITY), 86])
        firmware_path = None
        if args.firmware_dir:
            firmware_path = args.firmware_dir / f"demo_{i:02d}.jpg"
            shutil.copyfile(classic_path, firmware_path)

        annotated = draw_detections(raw.copy(), item.detections, names)
        annotated_path = args.classic_dir / f"video_{i:02d}_pc_annotated.jpg"
        cv2.imwrite(str(annotated_path), annotated, [int(cv2.IMWRITE_JPEG_QUALITY), 88])

        saved.append(
            {
                "sample": f"demo_{i:02d}",
                "classic_image": project_path(classic_path),
                "firmware_image": project_path(firmware_path) if firmware_path else None,
                "pc_annotated_image": project_path(annotated_path),
                "frame_index": item.frame_index,
                "timestamp_s": round(item.timestamp_s, 3),
                "object_count": item.object_count,
                "class_count": item.class_count,
                "labels": [names.get(d["class_id"], str(d["class_id"])) for d in item.detections],
            }
        )
    return saved


def main() -> int:
    args = parse_args()
    args.output_video.parent.mkdir(parents=True, exist_ok=True)
    args.report.parent.mkdir(parents=True, exist_ok=True)

    if not args.video.exists():
        raise SystemExit(f"input video not found: {args.video}")
    if not args.weights.exists():
        raise SystemExit(f"weights not found: {args.weights}")

    model = YOLO(str(args.weights))
    names = model.names
    cap = cv2.VideoCapture(str(args.video))
    if not cap.isOpened():
        raise SystemExit(f"failed to open video: {args.video}")

    src_fps = float(cap.get(cv2.CAP_PROP_FPS) or 12.0)
    src_w = int(cap.get(cv2.CAP_PROP_FRAME_WIDTH))
    src_h = int(cap.get(cv2.CAP_PROP_FRAME_HEIGHT))
    frame_count = int(cap.get(cv2.CAP_PROP_FRAME_COUNT))
    out_fps = max(1.0, src_fps / max(1, args.frame_stride))
    writer = cv2.VideoWriter(
        str(args.output_video),
        cv2.VideoWriter_fourcc(*"mp4v"),
        out_fps,
        (src_w, src_h),
    )
    if not writer.isOpened():
        raise SystemExit(f"failed to create output video: {args.output_video}")

    candidates: list[FrameCandidate] = []
    processed = 0
    written = 0
    inference_ms: list[float] = []
    per_class_counts: dict[str, int] = {}
    started = time.perf_counter()

    while True:
        ok, frame = cap.read()
        if not ok:
            break
        frame_index = processed
        processed += 1
        if frame_index % max(1, args.frame_stride) != 0:
            continue
        if args.max_frames and written >= args.max_frames:
            break

        t0 = time.perf_counter()
        result = model.predict(
            frame,
            imgsz=args.imgsz,
            conf=args.conf,
            iou=args.iou,
            device=args.device,
            verbose=False,
        )[0]
        dt_ms = (time.perf_counter() - t0) * 1000.0
        inference_ms.append(dt_ms)

        detections = result_to_detections(result)
        for det in detections:
            label = names.get(det["class_id"], str(det["class_id"]))
            per_class_counts[label] = per_class_counts.get(label, 0) + 1

        score, object_count, class_count = candidate_score(detections)
        if object_count > 0:
            candidates.append(
                FrameCandidate(
                    frame_index=frame_index,
                    timestamp_s=frame_index / src_fps,
                    score=score,
                    object_count=object_count,
                    class_count=class_count,
                    frame=frame.copy(),
                    detections=detections,
                )
            )

        annotated = draw_detections(frame.copy(), detections, names)
        header = f"YOLO11n COCO 320 | frame {frame_index} | objects {object_count} | {dt_ms:.1f} ms"
        cv2.rectangle(annotated, (0, 0), (src_w, 34), (0, 0, 0), -1)
        cv2.putText(annotated, header, (10, 23), cv2.FONT_HERSHEY_SIMPLEX, 0.62, (255, 255, 255), 2, cv2.LINE_AA)
        writer.write(annotated)
        written += 1

    cap.release()
    writer.release()

    if not inference_ms:
        raise SystemExit("no frames were processed")

    selected = select_candidates(candidates, args.top_k, args.min_frame_gap)
    saved_samples = save_board_images(selected, args, names)

    elapsed_s = time.perf_counter() - started
    report = {
        "source_video": project_path(args.video),
        "source_url": "https://github.com/intel-iot-devkit/sample-videos/raw/master/person-bicycle-car-detection.mp4",
        "weights": project_path(args.weights),
        "pc_model_note": "Ultralytics YOLO11n COCO at imgsz=320; board uses Espressif quantized COCO YOLO11n 320.",
        "imgsz": args.imgsz,
        "conf": args.conf,
        "iou": args.iou,
        "device": args.device,
        "input_frames": frame_count,
        "source_fps": src_fps,
        "source_size": [src_w, src_h],
        "processed_frames": written,
        "wall_time_s": round(elapsed_s, 3),
        "pc_inference_ms": {
            "avg": round(sum(inference_ms) / len(inference_ms), 3),
            "min": round(min(inference_ms), 3),
            "max": round(max(inference_ms), 3),
        },
        "output_video": project_path(args.output_video),
        "per_class_counts": dict(sorted(per_class_counts.items(), key=lambda kv: kv[1], reverse=True)),
        "selected_samples": saved_samples,
    }
    args.report.write_text(json.dumps(report, indent=2, ensure_ascii=False), encoding="utf-8")

    print(json.dumps(report, indent=2, ensure_ascii=False))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
