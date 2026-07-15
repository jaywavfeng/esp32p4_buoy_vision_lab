#!/usr/bin/env python3
"""
Build board-verified TinyCNN validation assets from local LaRS/PoTATO-like
sources.

The script intentionally does not fall back to random camera frames. Candidate
images are first generated from known source files, then optionally sent through
the ESP32-P4 board dataset API. Only samples whose board Top-1 matches the
intended TinyCNN class and meets the score threshold are copied into the
firmware validation asset slots.
"""

from __future__ import annotations

import argparse
import hashlib
import json
import os
import re
import shutil
import subprocess
import sys
import time
import zipfile
from dataclasses import dataclass, field
from datetime import datetime, timezone
from pathlib import Path
from typing import Any
from urllib.parse import quote

from PIL import Image, ImageOps


LABEL_ORDER = ["unknown", "plastic_bottle", "foam", "buoy", "net", "ship_part"]
LARS_THING_CATEGORIES = {11, 12, 13, 14, 15, 16, 17, 19}
LARS_TARGET_CATEGORIES = {
    14: "buoy",
    11: "ship_part",
    12: "ship_part",
    17: "foam",
    19: "net",
}
DEFAULT_DOWNLOADS = Path("D:/Jaywav/Downloads")
DEFAULT_IMAGES_ZIP = DEFAULT_DOWNLOADS / "lars_v1.0.0_images.zip"
DEFAULT_ANNOTATIONS_ZIP = DEFAULT_DOWNLOADS / "lars_v1.0.0_annotations.zip"
DEFAULT_WORK_DIR = Path("artifacts/tinycls_lars_candidates")
DEFAULT_ASSET_DIR = Path("test_assets/tinycls_marine_demo")
DEFAULT_CONFIG_HEADER = Path("main/validation_assets_config.h")
BOARD_DATASET = "tinycls_lars_candidates"


@dataclass
class Candidate:
    label: str
    source_dataset: str
    split: str
    source_file: str
    source_detail: str
    path: Path
    sha256: str
    candidate_id: str
    result: dict[str, Any] = field(default_factory=dict)


def now_iso() -> str:
    return datetime.now(timezone.utc).isoformat(timespec="seconds")


def sha256_file(path: Path) -> str:
    h = hashlib.sha256()
    with path.open("rb") as f:
        for chunk in iter(lambda: f.read(65536), b""):
            h.update(chunk)
    return h.hexdigest()


def reset_dir(path: Path) -> None:
    if path.exists():
        shutil.rmtree(path)
    path.mkdir(parents=True, exist_ok=True)


def save_square_jpeg(img: Image.Image, out: Path, size: int = 320, quality: int = 88) -> None:
    img = ImageOps.exif_transpose(img).convert("RGB")
    img = ImageOps.fit(img, (size, size), method=Image.Resampling.LANCZOS)
    out.parent.mkdir(parents=True, exist_ok=True)
    # ESP-IDF's validation JPEG decode path expects ordinary baseline JPEGs.
    img.save(out, "JPEG", quality=quality, optimize=True, progressive=False)


def crop_bbox_square(img: Image.Image, bbox: list[int] | tuple[int, int, int, int], scale: float) -> Image.Image:
    x, y, w, h = [float(v) for v in bbox]
    iw, ih = img.size
    cx = x + w / 2.0
    cy = y + h / 2.0
    side = max(w, h) * scale
    side = max(side, 80.0)
    side = min(side, float(max(iw, ih)))
    left = max(0.0, min(float(iw) - 1.0, cx - side / 2.0))
    top = max(0.0, min(float(ih) - 1.0, cy - side / 2.0))
    right = min(float(iw), left + side)
    bottom = min(float(ih), top + side)
    if right - left < side:
        left = max(0.0, right - side)
    if bottom - top < side:
        top = max(0.0, bottom - side)
    return img.crop((int(left), int(top), int(right), int(bottom)))


def lars_image_name(split: str, annotation_file: str) -> str:
    stem = Path(annotation_file).stem
    return f"{split}/images/{stem}.jpg"


def open_zip_image(zip_file: zipfile.ZipFile, name: str) -> Image.Image:
    with zip_file.open(name) as f:
        return Image.open(f).copy()


def add_candidate(
    candidates: list[Candidate],
    label: str,
    source_dataset: str,
    split: str,
    source_file: str,
    source_detail: str,
    img: Image.Image,
    out_dir: Path,
    size: int,
) -> None:
    serial = len(candidates) + 1
    safe_label = label.replace("_", "-")
    out = out_dir / f"{serial:04d}_{safe_label}.jpg"
    save_square_jpeg(img, out, size=size)
    candidates.append(
        Candidate(
            label=label,
            source_dataset=source_dataset,
            split=split,
            source_file=source_file,
            source_detail=source_detail,
            path=out,
            sha256=sha256_file(out),
            candidate_id=out.stem,
        )
    )


def generate_lars_candidates(
    images_zip_path: Path,
    annotations_zip_path: Path,
    out_dir: Path,
    max_per_label: int,
    image_size: int,
) -> list[Candidate]:
    candidates: list[Candidate] = []
    per_label_counts = {label: 0 for label in LABEL_ORDER}
    with zipfile.ZipFile(images_zip_path) as images_zip, zipfile.ZipFile(annotations_zip_path) as ann_zip:
        image_names = set(images_zip.namelist())
        for split in ["val", "train"]:
            panoptic = json.load(ann_zip.open(f"{split}/panoptic_annotations.json"))
            annotations = panoptic["annotations"]

            unknown_rows = []
            object_rows: dict[str, list[tuple[int, dict[str, Any], dict[str, Any]]]] = {
                label: [] for label in LABEL_ORDER
            }
            for ann in annotations:
                segments = ann.get("segments_info", [])
                if not any(seg.get("category_id") in LARS_THING_CATEGORIES for seg in segments):
                    unknown_rows.append(ann)
                for seg in segments:
                    label = LARS_TARGET_CATEGORIES.get(seg.get("category_id"))
                    if not label:
                        continue
                    bbox = seg.get("bbox") or [0, 0, 0, 0]
                    bbox_area = int(bbox[2]) * int(bbox[3])
                    if bbox_area < 350:
                        continue
                    object_rows[label].append((bbox_area, ann, seg))

            for ann in unknown_rows:
                if per_label_counts["unknown"] >= max_per_label:
                    break
                image_name = lars_image_name(split, ann["file_name"])
                if image_name not in image_names:
                    continue
                img = open_zip_image(images_zip, image_name)
                add_candidate(
                    candidates,
                    "unknown",
                    "LaRS v1.0.0",
                    split,
                    image_name,
                    "full scene with no panoptic dynamic obstacle",
                    img,
                    out_dir,
                    image_size,
                )
                per_label_counts["unknown"] += 1

            for label in ["buoy", "ship_part", "foam", "net"]:
                rows = sorted(object_rows[label], key=lambda x: x[0], reverse=True)
                for bbox_area, ann, seg in rows:
                    if per_label_counts[label] >= max_per_label:
                        break
                    image_name = lars_image_name(split, ann["file_name"])
                    if image_name not in image_names:
                        continue
                    img = open_zip_image(images_zip, image_name)
                    scales = (1.20, 1.75) if label != "ship_part" else (1.00, 1.35)
                    for scale in scales:
                        if per_label_counts[label] >= max_per_label:
                            break
                        crop = crop_bbox_square(img, seg["bbox"], scale)
                        add_candidate(
                            candidates,
                            label,
                            "LaRS v1.0.0",
                            split,
                            image_name,
                            f"panoptic category {seg.get('category_id')} bbox={seg.get('bbox')} scale={scale}",
                            crop,
                            out_dir,
                            image_size,
                        )
                        per_label_counts[label] += 1
    return candidates


def discover_local_plastic_zips(downloads_dir: Path) -> list[Path]:
    if not downloads_dir.exists():
        return []
    zips: list[Path] = []
    for path in downloads_dir.glob("*.zip"):
        name = path.name.lower()
        if "lars" in name or "tinycls_marine_project" in name:
            continue
        if not any(token in name for token in ("plastic", "bottle", "potato")):
            continue
        if path.stat().st_size > 300 * 1024 * 1024:
            continue
        zips.append(path)
    return zips


def generate_plastic_candidates(
    plastic_zips: list[Path],
    out_dir: Path,
    max_per_label: int,
    image_size: int,
) -> list[Candidate]:
    candidates: list[Candidate] = []
    for zip_path in plastic_zips:
        with zipfile.ZipFile(zip_path) as z:
            image_names = [
                n for n in z.namelist()
                if n.lower().endswith((".jpg", ".jpeg", ".png", ".webp"))
                and not n.endswith("/")
            ]
            for name in image_names:
                if len(candidates) >= max_per_label:
                    return candidates
                try:
                    img = open_zip_image(z, name)
                except Exception as exc:
                    print(f"skip unreadable plastic candidate {zip_path.name}:{name}: {exc}", file=sys.stderr)
                    continue
                add_candidate(
                    candidates,
                    "plastic_bottle",
                    zip_path.name,
                    "local",
                    name,
                    "local plastic bottle candidate zip",
                    img,
                    out_dir,
                    image_size,
                )
    return candidates


def curl_request(
    method: str,
    url: str,
    interface: str | None = None,
    data_file: Path | None = None,
    form_data: str | None = None,
    timeout: int = 30,
) -> str:
    cmd = [
        "curl.exe",
        "--silent",
        "--show-error",
        "--fail-with-body",
        "--max-time",
        str(timeout),
    ]
    if interface:
        cmd += ["--interface", interface]
    if method != "GET":
        cmd += ["-X", method]
    if data_file:
        cmd += ["--data-binary", f"@{data_file}"]
    if form_data is not None:
        cmd += [
            "--header",
            "Content-Type: application/x-www-form-urlencoded",
            "--data",
            form_data,
        ]
    cmd.append(url)
    proc = subprocess.run(cmd, capture_output=True, text=True)
    if proc.returncode != 0:
        raise RuntimeError(f"curl failed {proc.returncode}: {proc.stderr.strip()} {proc.stdout.strip()}")
    return proc.stdout


def board_get_json(
    base_url: str,
    path: str,
    interface: str | None,
    timeout: int = 30,
    method: str = "GET",
) -> dict[str, Any]:
    text = curl_request(method, base_url.rstrip("/") + path, interface=interface, timeout=timeout)
    return json.loads(text)


def parse_overlay_text(svg: str) -> tuple[str, int, list[dict[str, Any]]]:
    # TinyCNN overlay text: classification label NN% / threshold NN%
    m = re.search(r"classification\s+([A-Za-z0-9_/-]+)\s+(\d+)%", svg)
    if not m:
        m = re.search(r">([A-Za-z0-9_/-]+)\s+(\d+)%\s*/\s*threshold", svg)
    if not m:
        return "unknown", 0, []
    label = m.group(1)
    score = int(m.group(2))
    return label, score, [{"label": label, "score": score}]


def parse_jsonl_result(text: str) -> dict[str, Any] | None:
    for line in text.splitlines():
        line = line.strip()
        if not line:
            continue
        try:
            row = json.loads(line)
        except json.JSONDecodeError:
            continue
        if row.get("ok") is True:
            return row
    return None


def run_candidate_on_board(
    candidate: Candidate,
    base_url: str,
    interface: str | None,
    dataset: str,
    wake: bool,
) -> dict[str, Any]:
    if wake:
        curl_request("POST", base_url.rstrip("/") + "/api/power?cmd=wake", interface=interface, timeout=10)
    curl_request("POST", base_url.rstrip("/") + "/api/recognition?method=tinycls", interface=interface, timeout=10)
    curl_request(
        "POST",
        base_url.rstrip("/") + "/api/config",
        interface=interface,
        form_data="inference_interval_ms=0",
        timeout=10,
    )

    path = "frames/frame_00001.jpg"
    put_url = (
        base_url.rstrip()
        + f"/api/dataset/file?dataset={quote(dataset)}&path={quote(path)}"
    )
    curl_request("PUT", put_url, interface=interface, data_file=candidate.path, timeout=45)
    start = board_get_json(
        base_url,
        f"/api/dataset/run/start?dataset={quote(dataset)}&method=tinycls&limit=1&stride=1",
        interface,
        timeout=20,
        method="POST",
    )
    run_id = start.get("run_id", "")
    deadline = time.time() + 60
    status: dict[str, Any] = {}
    while time.time() < deadline:
        status = board_get_json(base_url, "/api/dataset/run/status", interface, timeout=10)
        if status.get("run_id") == run_id and status.get("done"):
            break
        time.sleep(0.25)
    if not status.get("done"):
        raise RuntimeError(f"dataset run timeout for {candidate.candidate_id}")

    result: dict[str, Any] = {
        "ok": bool(status.get("ok_frames", 0)),
        "run_id": run_id,
        "processed": status.get("processed", 0),
        "analysis_ms": status.get("avg_analysis_ms", 0),
        "p95_analysis_ms": status.get("p95_analysis_ms", 0),
        "labels": status.get("labels", []),
        "top1": "",
        "score": 0,
        "top_k": [],
        "raw_status": status,
    }

    result_uri = status.get("result_uri") or ""
    if result_uri:
        try:
            row = parse_jsonl_result(curl_request("GET", base_url.rstrip("/") + result_uri, interface=interface))
            if row:
                top_k = row.get("top_k") or []
                if top_k:
                    result["top1"] = top_k[0].get("label", "")
                    result["score"] = int(top_k[0].get("score", 0))
                    result["top_k"] = top_k
                elif row.get("label"):
                    result["top1"] = row.get("label", "")
                    result["score"] = int(row.get("best_score", 0))
                    result["top_k"] = [{"label": result["top1"], "score": result["score"]}]
                result["inference_ms"] = row.get("inference_ms", 0)
                result["analysis_ms"] = row.get("analysis_ms", result["analysis_ms"])
        except Exception as exc:
            result["result_fetch_error"] = str(exc)

    if not result["top1"]:
        overlay_uri = status.get("last_overlay_uri") or (
            f"/api/dataset/frame.svg?run_id={quote(run_id)}&dataset={quote(dataset)}&index=1"
        )
        try:
            svg = curl_request("GET", base_url.rstrip("/") + overlay_uri, interface=interface, timeout=20)
            label, score, top_k = parse_overlay_text(svg)
            result["top1"] = label
            result["score"] = score
            result["top_k"] = top_k
        except Exception as exc:
            result["overlay_fetch_error"] = str(exc)

    if not result["top1"] and result["labels"]:
        first = result["labels"][0]
        result["top1"] = first.get("label", "")
        result["score"] = 0
        result["top_k"] = [{"label": result["top1"], "score": 0}]

    result["matched"] = result.get("top1") == candidate.label
    return result


def select_verified(candidates: list[Candidate], score_min: int, sample_slots: int) -> tuple[list[Candidate], dict[str, str]]:
    by_label: dict[str, list[Candidate]] = {label: [] for label in LABEL_ORDER}
    for c in candidates:
        r = c.result
        if (
            c.label != "unknown"
            and r.get("ok")
            and r.get("top1") == c.label
            and int(r.get("score", 0)) >= score_min
        ):
            by_label[c.label].append(c)
    for rows in by_label.values():
        rows.sort(key=lambda c: (-int(c.result.get("score", 0)), int(c.result.get("analysis_ms", 999999))))

    selected_by_label: list[Candidate] = []
    skipped: dict[str, str] = {}
    for label in LABEL_ORDER:
        if label == "unknown":
            skipped[label] = "unknown is intentionally excluded from TinyCNN phone validation"
            continue
        if by_label[label]:
            selected_by_label.append(by_label[label][0])
        else:
            skipped[label] = f"no board-verified Top-1 match >= {score_min}%"

    all_verified = [
        c for c in candidates
        if c.label != "unknown"
        and c.result.get("ok")
        and c.result.get("top1") == c.label
        and int(c.result.get("score", 0)) >= score_min
    ]
    all_verified.sort(
        key=lambda c: (
            c not in selected_by_label,
            -int(c.result.get("score", 0)),
            int(c.result.get("analysis_ms", 999999)),
        )
    )

    selected: list[Candidate] = []
    seen_sha: set[str] = set()
    seen_source: set[str] = set()
    for c in selected_by_label + all_verified:
        if c.sha256 in seen_sha or c.source_file in seen_source:
            continue
        selected.append(c)
        seen_sha.add(c.sha256)
        seen_source.add(c.source_file)
        if len(selected) >= sample_slots:
            break
    if len(selected) < sample_slots:
        for c in selected_by_label + all_verified:
            if c.sha256 in seen_sha:
                continue
            selected.append(c)
            seen_sha.add(c.sha256)
            seen_source.add(c.source_file)
            if len(selected) >= sample_slots:
                break
    return selected, skipped


def load_reuse_manifest(path: Path, candidates_dir: Path) -> tuple[list[Candidate], list[Path], dict[str, Any]]:
    manifest = json.loads(path.read_text(encoding="utf-8"))
    candidates: list[Candidate] = []
    for item in manifest.get("candidates", []):
        candidate_id = item.get("candidate_id", "")
        if not candidate_id:
            continue
        image_path = candidates_dir / f"{candidate_id}.jpg"
        if not image_path.exists():
            raise FileNotFoundError(f"candidate image missing for reuse: {image_path}")
        candidates.append(
            Candidate(
                label=item.get("label", ""),
                source_dataset=item.get("source_dataset", ""),
                split=item.get("split", ""),
                source_file=item.get("source_file", ""),
                source_detail=item.get("source_detail", ""),
                path=image_path,
                sha256=item.get("candidate_sha256") or sha256_file(image_path),
                candidate_id=candidate_id,
                result=item.get("result") or {},
            )
        )
    board_status = {
        "model_info": (manifest.get("board") or {}).get("status_model", {}),
        "recognition_method": (manifest.get("board") or {}).get("recognition_method", ""),
    }
    plastic_zips = [Path(name) for name in manifest.get("plastic_source_zips", [])]
    return candidates, plastic_zips, board_status


def copy_final_assets(selected: list[Candidate], asset_dir: Path, sample_slots: int) -> tuple[list[dict[str, Any]], list[dict[str, Any]]]:
    images_dir = asset_dir / "images"
    frames_dir = asset_dir / "frames"
    reset_dir(images_dir)
    reset_dir(frames_dir)

    if len(selected) < sample_slots:
        raise RuntimeError(f"need {sample_slots} board-verified non-unknown TinyCNN samples, got {len(selected)}")

    ranked = list(selected)

    sample_manifest: list[dict[str, Any]] = []
    for i, c in enumerate(ranked[:sample_slots], start=1):
        out = images_dir / f"tiny_{i:02d}.jpg"
        shutil.copyfile(c.path, out)
        sample_manifest.append(candidate_manifest(c, extra={"slot": f"tiny_{i:02d}", "sha256": sha256_file(out)}))

    video_source = list(selected)
    frame_manifest: list[dict[str, Any]] = []
    for i in range(1, 17):
        c = video_source[(i - 1) % len(video_source)]
        out = frames_dir / f"tiny_frame_{i:05d}.jpg"
        shutil.copyfile(c.path, out)
        frame_manifest.append(candidate_manifest(c, extra={"frame": i, "sha256": sha256_file(out)}))
    return sample_manifest, frame_manifest


def candidate_manifest(c: Candidate, extra: dict[str, Any] | None = None) -> dict[str, Any]:
    item = {
        "candidate_id": c.candidate_id,
        "label": c.label,
        "source_dataset": c.source_dataset,
        "split": c.split,
        "source_file": c.source_file,
        "source_detail": c.source_detail,
        "candidate_sha256": c.sha256,
        "board_top1": c.result.get("top1", ""),
        "board_score": c.result.get("score", 0),
        "board_top_k": c.result.get("top_k", []),
        "inference_ms": c.result.get("inference_ms", 0),
        "analysis_ms": c.result.get("analysis_ms", 0),
        "run_id": c.result.get("run_id", ""),
    }
    if extra:
        item.update(extra)
    return item


def update_validation_header(header: Path, selected: list[Candidate]) -> None:
    if not header.exists():
        return
    text = header.read_text(encoding="utf-8")
    for i, c in enumerate(selected[:4], start=1):
        text = re.sub(
            rf'#define VALIDATION_TINYCLS_{i:02d}_LABEL ".*?"',
            f'#define VALIDATION_TINYCLS_{i:02d}_LABEL "{c.label}"',
            text,
        )
    header.write_text(text, encoding="utf-8")


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--lars-images-zip", type=Path, default=DEFAULT_IMAGES_ZIP)
    parser.add_argument("--lars-annotations-zip", type=Path, default=DEFAULT_ANNOTATIONS_ZIP)
    parser.add_argument("--downloads-dir", type=Path, default=DEFAULT_DOWNLOADS)
    parser.add_argument("--plastic-source-zip", type=Path, action="append", default=[])
    parser.add_argument("--work-dir", type=Path, default=DEFAULT_WORK_DIR)
    parser.add_argument("--asset-dir", type=Path, default=DEFAULT_ASSET_DIR)
    parser.add_argument("--config-header", type=Path, default=DEFAULT_CONFIG_HEADER)
    parser.add_argument("--board", default="http://169.254.100.2")
    parser.add_argument("--interface", default="")
    parser.add_argument("--score-min", type=int, default=75)
    parser.add_argument("--max-candidates-per-label", type=int, default=16)
    parser.add_argument("--sample-slots", type=int, default=4)
    parser.add_argument("--image-size", type=int, default=320)
    parser.add_argument("--run-board", action="store_true")
    parser.add_argument("--reuse-manifest", type=Path)
    parser.add_argument("--wake", action="store_true")
    parser.add_argument("--dataset", default=BOARD_DATASET)
    args = parser.parse_args()

    candidates_dir = args.work_dir / "candidates"
    board_status: dict[str, Any] = {}

    if args.reuse_manifest:
        candidates, plastic_zips, board_status = load_reuse_manifest(args.reuse_manifest, candidates_dir)
    else:
        if not args.lars_images_zip.exists():
            raise FileNotFoundError(args.lars_images_zip)
        if not args.lars_annotations_zip.exists():
            raise FileNotFoundError(args.lars_annotations_zip)

        reset_dir(candidates_dir)

        candidates = generate_lars_candidates(
            args.lars_images_zip,
            args.lars_annotations_zip,
            candidates_dir,
            args.max_candidates_per_label,
            args.image_size,
        )

        plastic_zips = args.plastic_source_zip or discover_local_plastic_zips(args.downloads_dir)
        candidates.extend(
            generate_plastic_candidates(
                plastic_zips,
                candidates_dir,
                args.max_candidates_per_label,
                args.image_size,
            )
        )

    print(f"generated {len(candidates)} candidates")
    print("candidate counts:", json.dumps({label: sum(c.label == label for c in candidates) for label in LABEL_ORDER}))
    print("plastic candidate zips:", ", ".join(str(p) for p in plastic_zips) or "none")

    if args.run_board and not args.reuse_manifest:
        interface = args.interface or None
        try:
            curl_request("POST", args.board.rstrip("/") + "/api/recognition?method=tinycls", interface=interface, timeout=10)
            board_status = board_get_json(args.board, "/api/status", interface, timeout=10)
            print(
                "board:",
                board_status.get("recognition_method"),
                board_status.get("model_info", {}).get("name"),
                board_status.get("model_info", {}).get("input_size"),
            )
        except Exception as exc:
            raise RuntimeError(f"board status failed: {exc}") from exc
        for i, c in enumerate(candidates, start=1):
            print(f"[{i:03d}/{len(candidates):03d}] {c.label} {c.source_file}", flush=True)
            try:
                c.result = run_candidate_on_board(c, args.board, interface, args.dataset, args.wake)
            except Exception as exc:
                c.result = {"ok": False, "error": str(exc)}
            print(
                f"    top1={c.result.get('top1', '')} score={c.result.get('score', 0)} "
                f"analysis={c.result.get('analysis_ms', 0)} matched={c.result.get('matched', False)}",
                flush=True,
            )
    elif not args.reuse_manifest:
        for c in candidates:
            c.result = {"ok": False, "error": "not run on board"}

    selected, skipped = select_verified(candidates, args.score_min, args.sample_slots)
    sample_manifest: list[dict[str, Any]] = []
    frame_manifest: list[dict[str, Any]] = []
    if (args.run_board or args.reuse_manifest) and len(selected) >= args.sample_slots:
        sample_manifest, frame_manifest = copy_final_assets(selected, args.asset_dir, args.sample_slots)
        update_validation_header(args.config_header, selected)

    manifest = {
        "generated_at": now_iso(),
        "source_policy": "LaRS/local candidates only; no random camera fallback; selected phone samples must be non-unknown board Top-1 matches",
        "lars_images_zip": args.lars_images_zip.name,
        "lars_annotations_zip": args.lars_annotations_zip.name,
        "plastic_source_zips": [p.name for p in plastic_zips],
        "score_min": args.score_min,
        "board": {
            "base_url": args.board,
            "interface": args.interface,
            "status_model": board_status.get("model_info", {}),
            "recognition_method": board_status.get("recognition_method", ""),
        },
        "selected_samples": sample_manifest,
        "selected_video_frames": frame_manifest,
        "skipped_labels": [{"label": label, "reason": reason} for label, reason in skipped.items()],
        "candidates": [candidate_manifest(c) | {"result": c.result} for c in candidates],
    }

    args.asset_dir.mkdir(parents=True, exist_ok=True)
    manifest_path = args.asset_dir / "manifest.json"
    manifest_path.write_text(json.dumps(manifest, indent=2, ensure_ascii=False) + "\n", encoding="utf-8")
    print(f"selected labels: {[c.label for c in selected]}")
    print(f"wrote {manifest_path}")
    return 0 if (not args.run_board and not args.reuse_manifest) or len(selected) >= args.sample_slots else 2


if __name__ == "__main__":
    raise SystemExit(main())
