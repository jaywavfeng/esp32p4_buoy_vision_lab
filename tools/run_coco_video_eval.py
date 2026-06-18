"""Run PC-side COCO YOLO prediction on a video and export board samples.

The board model is ESP-DL COCO YOLO11n 320, so this helper uses the local
Ultralytics YOLO11n weights with imgsz=320 and prepares 320x320 JPEG samples
for the firmware validation page.
"""

from __future__ import annotations

import argparse
import csv
import json
import time
from pathlib import Path

import cv2
from ultralytics import YOLO


def letterbox_square(frame, size: int = 320):
    h, w = frame.shape[:2]
    scale = min(size / w, size / h)
    nw, nh = max(1, int(round(w * scale))), max(1, int(round(h * scale)))
    resized = cv2.resize(frame, (nw, nh), interpolation=cv2.INTER_AREA)
    canvas = cv2.copyMakeBorder(
        resized,
        (size - nh) // 2,
        size - nh - (size - nh) // 2,
        (size - nw) // 2,
        size - nw - (size - nw) // 2,
        cv2.BORDER_CONSTANT,
        value=(114, 114, 114),
    )
    return canvas


def detections_from_result(result):
    names = result.names
    detections = []
    if result.boxes is None:
        return detections
    for box in result.boxes:
        cls_id = int(box.cls[0].item())
        conf = float(box.conf[0].item())
        xyxy = [float(v) for v in box.xyxy[0].tolist()]
        detections.append(
            {
                "label": str(names.get(cls_id, cls_id)),
                "class_id": cls_id,
                "score": round(conf, 4),
                "xyxy": [round(v, 1) for v in xyxy],
            }
        )
    detections.sort(key=lambda item: item["score"], reverse=True)
    return detections


def select_diverse(candidates, count: int, min_gap: int):
    selected = []
    for cand in sorted(candidates, key=lambda item: item["rank_score"], reverse=True):
        if all(abs(cand["frame_index"] - old["frame_index"]) >= min_gap for old in selected):
            selected.append(cand)
        if len(selected) >= count:
            break
    if len(selected) < count:
        for cand in sorted(candidates, key=lambda item: item["rank_score"], reverse=True):
            if cand not in selected:
                selected.append(cand)
            if len(selected) >= count:
                break
    selected.sort(key=lambda item: item["frame_index"])
    return selected


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--video", default="data/coco_video/person-bicycle-car-detection.mp4")
    parser.add_argument("--weights", default="yolo11n.pt")
    parser.add_argument("--imgsz", type=int, default=320)
    parser.add_argument("--conf", type=float, default=0.25)
    parser.add_argument("--iou", type=float, default=0.7)
    parser.add_argument("--device", default="cpu")
    parser.add_argument("--frame-step", type=int, default=2)
    parser.add_argument("--max-frames", type=int, default=0)
    parser.add_argument("--board-samples", type=int, default=4)
    parser.add_argument("--min-gap", type=int, default=30)
    parser.add_argument("--out-dir", default="reports/coco_video")
    parser.add_argument("--sample-dir", default="test_assets/coco_classic/images")
    args = parser.parse_args()

    video_path = Path(args.video)
    out_dir = Path(args.out_dir)
    sample_dir = Path(args.sample_dir)
    frame_dir = out_dir / "selected_frames"
    out_dir.mkdir(parents=True, exist_ok=True)
    sample_dir.mkdir(parents=True, exist_ok=True)
    frame_dir.mkdir(parents=True, exist_ok=True)

    cap = cv2.VideoCapture(str(video_path))
    if not cap.isOpened():
        raise SystemExit(f"cannot open video: {video_path}")

    src_fps = cap.get(cv2.CAP_PROP_FPS) or 30.0
    width = int(cap.get(cv2.CAP_PROP_FRAME_WIDTH))
    height = int(cap.get(cv2.CAP_PROP_FRAME_HEIGHT))
    total = int(cap.get(cv2.CAP_PROP_FRAME_COUNT))
    out_fps = max(1.0, src_fps / max(1, args.frame_step))
    out_video = out_dir / f"{video_path.stem}_yolo11n320.mp4"
    writer = cv2.VideoWriter(
        str(out_video),
        cv2.VideoWriter_fourcc(*"mp4v"),
        out_fps,
        (width, height),
    )
    if not writer.isOpened():
        raise SystemExit(f"cannot create annotated video: {out_video}")

    model = YOLO(args.weights)
    rows = []
    candidates = []
    processed = 0
    start = time.perf_counter()

    frame_index = 0
    while True:
        ok, frame = cap.read()
        if not ok:
            break
        if frame_index % max(1, args.frame_step) != 0:
            frame_index += 1
            continue
        if args.max_frames and processed >= args.max_frames:
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
        infer_ms = round((time.perf_counter() - t0) * 1000.0, 2)
        detections = detections_from_result(result)
        labels = sorted({det["label"] for det in detections})
        annotated = result.plot()
        writer.write(annotated)

        row = {
            "frame_index": frame_index,
            "time_s": round(frame_index / src_fps, 3) if src_fps else 0,
            "inference_ms_pc": infer_ms,
            "detection_count": len(detections),
            "labels": ",".join(labels),
            "top_label": detections[0]["label"] if detections else "",
            "top_score": detections[0]["score"] if detections else 0,
        }
        rows.append(row)
        candidates.append(
            {
                **row,
                "rank_score": len(detections) * 10 + len(labels) * 2 + float(row["top_score"]),
                "frame_bgr": frame.copy(),
                "annotated_bgr": annotated.copy(),
                "detections": detections[:12],
            }
        )
        processed += 1
        frame_index += 1

    cap.release()
    writer.release()
    elapsed = time.perf_counter() - start

    selected = select_diverse(candidates, args.board_samples, args.min_gap)
    selected_meta = []
    for i, item in enumerate(selected, start=1):
        name = f"classic_{i:02d}.jpg"
        sample_path = sample_dir / name
        frame_path = frame_dir / f"classic_{i:02d}_frame_{item['frame_index']:06d}.jpg"
        anno_path = frame_dir / f"classic_{i:02d}_annotated_{item['frame_index']:06d}.jpg"
        cv2.imwrite(str(sample_path), letterbox_square(item["frame_bgr"], args.imgsz), [int(cv2.IMWRITE_JPEG_QUALITY), 88])
        cv2.imwrite(str(frame_path), item["frame_bgr"], [int(cv2.IMWRITE_JPEG_QUALITY), 90])
        cv2.imwrite(str(anno_path), item["annotated_bgr"], [int(cv2.IMWRITE_JPEG_QUALITY), 90])
        selected_meta.append(
            {
                "sample": f"classic_{i:02d}",
                "sample_path": str(sample_path),
                "frame_path": str(frame_path),
                "annotated_path": str(anno_path),
                "frame_index": item["frame_index"],
                "time_s": item["time_s"],
                "detection_count_pc": item["detection_count"],
                "labels_pc": item["labels"],
                "top_label_pc": item["top_label"],
                "top_score_pc": item["top_score"],
                "detections_pc": item["detections"],
            }
        )

    csv_path = out_dir / "pc_video_detections.csv"
    with csv_path.open("w", newline="", encoding="utf-8") as f:
        writer_csv = csv.DictWriter(
            f,
            fieldnames=[
                "frame_index",
                "time_s",
                "inference_ms_pc",
                "detection_count",
                "labels",
                "top_label",
                "top_score",
            ],
        )
        writer_csv.writeheader()
        writer_csv.writerows(rows)

    summary = {
        "video": str(video_path),
        "weights": args.weights,
        "imgsz": args.imgsz,
        "conf": args.conf,
        "iou": args.iou,
        "device": args.device,
        "source_width": width,
        "source_height": height,
        "source_fps": src_fps,
        "source_frames": total,
        "frame_step": args.frame_step,
        "processed_frames": processed,
        "elapsed_s": round(elapsed, 2),
        "avg_processed_fps": round(processed / elapsed, 2) if elapsed else 0,
        "avg_pc_inference_ms": round(sum(r["inference_ms_pc"] for r in rows) / len(rows), 2) if rows else 0,
        "annotated_video": str(out_video),
        "detections_csv": str(csv_path),
        "selected_samples": selected_meta,
    }
    summary_path = out_dir / "pc_video_summary.json"
    summary_path.write_text(json.dumps(summary, ensure_ascii=False, indent=2), encoding="utf-8")
    print(json.dumps(summary, ensure_ascii=False, indent=2))


if __name__ == "__main__":
    main()
