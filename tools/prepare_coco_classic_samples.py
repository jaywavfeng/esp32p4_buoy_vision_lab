"""Prepare dense COCO val2017 images for ESP32-P4 board validation.

The script downloads only the COCO annotation ZIP plus a limited set of image
files, ranks them with local YOLO11n COCO inference at 320px, and copies the
best samples to the firmware's embedded demo image paths.
"""

from __future__ import annotations

import argparse
import json
import shutil
import urllib.request
import zipfile
from collections import defaultdict
from dataclasses import dataclass
from pathlib import Path

import cv2
from ultralytics import YOLO


ROOT = Path(__file__).resolve().parents[1]
ANNOTATION_URL = "http://images.cocodataset.org/annotations/annotations_trainval2017.zip"
VAL_IMAGE_URL = "http://images.cocodataset.org/val2017/{file_name}"
USEFUL_CATEGORY_NAMES = {
    "person",
    "bicycle",
    "car",
    "motorcycle",
    "bus",
    "truck",
    "traffic light",
    "bench",
    "chair",
    "dining table",
    "bottle",
    "cup",
    "bowl",
    "dog",
    "cat",
    "backpack",
    "handbag",
    "umbrella",
}


@dataclass
class ImageInfo:
    id: int
    file_name: str
    width: int
    height: int
    annotation_count: int
    annotation_class_count: int
    useful_count: int
    useful_class_count: int
    annotation_score: int


def project_path(path: Path) -> str:
    resolved = path.resolve()
    try:
        return str(resolved.relative_to(ROOT))
    except ValueError:
        return str(resolved)


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--weights", type=Path, default=ROOT / "yolo11n.pt")
    parser.add_argument("--imgsz", type=int, default=320)
    parser.add_argument("--conf", type=float, default=0.25)
    parser.add_argument("--iou", type=float, default=0.7)
    parser.add_argument("--device", default="cpu")
    parser.add_argument("--candidate-count", type=int, default=48)
    parser.add_argument("--top-k", type=int, default=4)
    parser.add_argument("--board-width", type=int, default=512)
    parser.add_argument(
        "--data-dir",
        type=Path,
        default=ROOT / "data" / "coco_val2017_subset",
    )
    parser.add_argument(
        "--classic-dir",
        type=Path,
        default=ROOT / "test_assets" / "coco_classic" / "images",
    )
    parser.add_argument(
        "--firmware-dir",
        type=Path,
        default=ROOT / "test_assets" / "video_frames_320" / "images",
    )
    parser.add_argument(
        "--report",
        type=Path,
        default=ROOT / "reports" / "coco_video" / "coco_classic_samples.json",
    )
    return parser.parse_args()


def download(url: str, output: Path) -> None:
    if output.exists() and output.stat().st_size > 0:
        return
    output.parent.mkdir(parents=True, exist_ok=True)
    print(f"Downloading {url} -> {output}")
    urllib.request.urlretrieve(url, output)


def load_annotations(data_dir: Path) -> tuple[list[dict], list[dict], dict[int, str]]:
    zip_path = data_dir / "annotations_trainval2017.zip"
    json_path = data_dir / "annotations" / "instances_val2017.json"
    download(ANNOTATION_URL, zip_path)
    if not json_path.exists():
        with zipfile.ZipFile(zip_path) as zf:
            zf.extract("annotations/instances_val2017.json", data_dir)
    data = json.loads(json_path.read_text(encoding="utf-8"))
    categories = {int(c["id"]): c["name"] for c in data["categories"]}
    return data["images"], data["annotations"], categories


def rank_by_annotations(images: list[dict], annotations: list[dict], categories: dict[int, str]) -> list[ImageInfo]:
    anns_by_image: dict[int, list[dict]] = defaultdict(list)
    for ann in annotations:
        if ann.get("iscrowd", 0):
            continue
        area = float(ann.get("area") or 0)
        if area < 300:
            continue
        anns_by_image[int(ann["image_id"])].append(ann)

    ranked: list[ImageInfo] = []
    for image in images:
        image_id = int(image["id"])
        anns = anns_by_image.get(image_id, [])
        if len(anns) < 5:
            continue
        classes = {int(a["category_id"]) for a in anns}
        useful = [a for a in anns if categories.get(int(a["category_id"])) in USEFUL_CATEGORY_NAMES]
        useful_classes = {int(a["category_id"]) for a in useful}
        if len(useful) < 3 or len(useful_classes) < 2:
            continue
        score = len(anns) * 12 + len(classes) * 38 + len(useful) * 8 + len(useful_classes) * 28
        ranked.append(
            ImageInfo(
                id=image_id,
                file_name=str(image["file_name"]),
                width=int(image["width"]),
                height=int(image["height"]),
                annotation_count=len(anns),
                annotation_class_count=len(classes),
                useful_count=len(useful),
                useful_class_count=len(useful_classes),
                annotation_score=score,
            )
        )
    return sorted(ranked, key=lambda x: x.annotation_score, reverse=True)


def detections_from_result(result) -> list[dict]:
    detections: list[dict] = []
    if result.boxes is None:
        return detections
    for box in result.boxes:
        detections.append(
            {
                "xyxy": box.xyxy[0].detach().cpu().numpy().tolist(),
                "class_id": int(box.cls[0].item()),
                "score": float(box.conf[0].item()),
            }
        )
    detections.sort(key=lambda d: d["score"], reverse=True)
    return detections


def draw_detections(frame, detections: list[dict], names: dict[int, str]):
    palette = [
        (56, 189, 248),
        (52, 211, 153),
        (251, 191, 36),
        (248, 113, 113),
        (196, 181, 253),
        (244, 114, 182),
        (147, 197, 253),
        (163, 230, 53),
    ]
    for i, det in enumerate(detections):
        x1, y1, x2, y2 = [int(v) for v in det["xyxy"]]
        color = palette[i % len(palette)]
        label = f"{names.get(det['class_id'], str(det['class_id']))} {det['score']:.2f}"
        cv2.rectangle(frame, (x1, y1), (x2, y2), color, 2)
        (tw, th), _ = cv2.getTextSize(label, cv2.FONT_HERSHEY_SIMPLEX, 0.5, 1)
        y0 = max(0, y1 - th - 8)
        cv2.rectangle(frame, (x1, y0), (x1 + tw + 8, y0 + th + 8), color, -1)
        cv2.putText(
            frame,
            label,
            (x1 + 4, y0 + th + 3),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.5,
            (0, 0, 0),
            1,
            cv2.LINE_AA,
        )
    return frame


def resize_to_width(frame, width: int):
    if width <= 0 or frame.shape[1] <= width:
        return frame
    height = max(1, round(frame.shape[0] * width / frame.shape[1]))
    return cv2.resize(frame, (width, height), interpolation=cv2.INTER_AREA)


def model_score(detections: list[dict], names: dict[int, str]) -> tuple[int, int, int, int]:
    classes = {d["class_id"] for d in detections}
    useful = [d for d in detections if names.get(d["class_id"]) in USEFUL_CATEGORY_NAMES]
    useful_classes = {d["class_id"] for d in useful}
    score = len(detections) * 20 + len(classes) * 50 + len(useful) * 10 + len(useful_classes) * 35
    return score, len(detections), len(classes), len(useful)


def main() -> int:
    args = parse_args()
    args.data_dir.mkdir(parents=True, exist_ok=True)
    args.classic_dir.mkdir(parents=True, exist_ok=True)
    args.firmware_dir.mkdir(parents=True, exist_ok=True)
    args.report.parent.mkdir(parents=True, exist_ok=True)
    images_dir = args.data_dir / "val2017"
    images_dir.mkdir(parents=True, exist_ok=True)

    images, annotations, categories = load_annotations(args.data_dir)
    ranked = rank_by_annotations(images, annotations, categories)[: args.candidate_count]
    if not ranked:
        raise SystemExit("no COCO candidates matched the selection criteria")

    model = YOLO(str(args.weights))
    names = model.names
    evaluated: list[dict] = []
    for item in ranked:
        image_path = images_dir / item.file_name
        download(VAL_IMAGE_URL.format(file_name=item.file_name), image_path)
        frame = cv2.imread(str(image_path), cv2.IMREAD_COLOR)
        if frame is None:
            continue
        result = model.predict(
            frame,
            imgsz=args.imgsz,
            conf=args.conf,
            iou=args.iou,
            device=args.device,
            verbose=False,
        )[0]
        detections = detections_from_result(result)
        score, object_count, class_count, useful_count = model_score(detections, names)
        evaluated.append(
            {
                "image": item,
                "image_path": image_path,
                "frame": frame,
                "detections": detections,
                "score": score,
                "object_count": object_count,
                "class_count": class_count,
                "useful_count": useful_count,
                "labels": [names.get(d["class_id"], str(d["class_id"])) for d in detections],
            }
        )

    evaluated.sort(
        key=lambda e: (e["score"], e["object_count"], e["class_count"], e["image"].annotation_score),
        reverse=True,
    )
    selected = evaluated[: args.top_k]
    if len(selected) < args.top_k:
        raise SystemExit(f"only selected {len(selected)} images")

    samples: list[dict] = []
    for index, entry in enumerate(selected, start=1):
        raw = resize_to_width(entry["frame"], args.board_width)
        classic_path = args.classic_dir / f"coco_{index:02d}.jpg"
        firmware_path = args.firmware_dir / f"demo_{index:02d}.jpg"
        annotated_path = args.classic_dir / f"coco_{index:02d}_pc_annotated.jpg"
        cv2.imwrite(str(classic_path), raw, [int(cv2.IMWRITE_JPEG_QUALITY), 86])
        shutil.copyfile(classic_path, firmware_path)
        annotated = draw_detections(raw.copy(), entry["detections"], names)
        cv2.imwrite(str(annotated_path), annotated, [int(cv2.IMWRITE_JPEG_QUALITY), 88])
        image_info: ImageInfo = entry["image"]
        samples.append(
            {
                "sample": f"demo_{index:02d}",
                "source_file": image_info.file_name,
                "source_image": project_path(entry["image_path"]),
                "classic_image": project_path(classic_path),
                "firmware_image": project_path(firmware_path),
                "pc_annotated_image": project_path(annotated_path),
                "coco_image_url": VAL_IMAGE_URL.format(file_name=image_info.file_name),
                "source_size": [image_info.width, image_info.height],
                "board_size": [raw.shape[1], raw.shape[0]],
                "annotation_count": image_info.annotation_count,
                "annotation_class_count": image_info.annotation_class_count,
                "pc_object_count": entry["object_count"],
                "pc_class_count": entry["class_count"],
                "pc_labels": entry["labels"],
            }
        )

    report = {
        "dataset": "COCO val2017",
        "annotation_url": ANNOTATION_URL,
        "image_url_pattern": VAL_IMAGE_URL,
        "weights": project_path(args.weights),
        "imgsz": args.imgsz,
        "conf": args.conf,
        "iou": args.iou,
        "candidate_count": len(ranked),
        "evaluated_count": len(evaluated),
        "selected_samples": samples,
    }
    args.report.write_text(json.dumps(report, indent=2, ensure_ascii=False), encoding="utf-8")
    print(json.dumps(report, indent=2, ensure_ascii=False))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
