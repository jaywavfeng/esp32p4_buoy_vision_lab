#!/usr/bin/env python3
"""Build board-verified Fish31 phone validation assets.

The script prefers an existing standard dataset layout:

  data/fish31_cls/{valid,test,train}/<class>/*.jpg

If that dataset is absent, it downloads a small set of Creative Commons
iNaturalist candidates for the Fish31 supplementary species used by the model
package. Final assets are copied only after ESP32-P4 board inference returns a
Top-1 label matching the expected class with score >= --score-min.
"""

from __future__ import annotations

import argparse
import hashlib
import json
import re
import shutil
import ssl
import subprocess
import time
import urllib.error
import urllib.parse
import urllib.request
from dataclasses import dataclass, field
from datetime import datetime, timezone
from pathlib import Path
from typing import Any

from PIL import Image, ImageOps


FISH31_CLASSES = [f"fish_{i:02d}" for i in range(1, 24)] + [
    "sand_seabed",
    "rubble_rock_seabed",
    "live_coral",
    "dead_bleached_coral",
    "benthic_invertebrate",
    "seagrass",
    "algae_substrate",
    "complex_underwater_bg",
]
INAT_SPECIES = {
    "fish_15": "Hemigymnus melapterus",
    "fish_19": "Pempheris vanicolensis",
    "fish_20": "Zanclus cornutus",
    "fish_21": "Neoglyphidodon nigroris",
    "fish_23": "Siganus fuscescens",
}
INAT_API = "https://api.inaturalist.org/v1/observations"
CC_LICENSES = {"cc0", "cc-by", "cc-by-nc", "cc-by-sa", "cc-by-nd", "cc-by-nc-sa", "cc-by-nc-nd"}
DEFAULT_DATASET = Path("data/fish31_cls")
DEFAULT_WORK_DIR = Path("artifacts/fish31_validation_candidates")
DEFAULT_ASSET_DIR = Path("test_assets/fish31_validation")
DEFAULT_VIDEO_DIR = Path("test_assets/fish31_video_demo")
DEFAULT_CONFIG_HEADER = Path("main/validation_assets_config.h")
BOARD_DATASET = "fish31_candidates"


@dataclass
class Candidate:
    label: str
    source_dataset: str
    split: str
    source_file: str
    source_url: str
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


def save_square_jpeg(src: Path, out: Path, size: int, quality: int) -> None:
    with Image.open(src) as img:
        img = ImageOps.exif_transpose(img).convert("RGB")
        img = ImageOps.fit(img, (size, size), method=Image.Resampling.LANCZOS)
        out.parent.mkdir(parents=True, exist_ok=True)
        img.save(out, "JPEG", quality=quality, optimize=True, progressive=False)


def add_candidate(
    candidates: list[Candidate],
    label: str,
    source_dataset: str,
    split: str,
    source_file: str,
    source_url: str,
    src: Path,
    out_dir: Path,
    image_size: int,
) -> None:
    serial = len(candidates) + 1
    out = out_dir / f"{serial:04d}_{label}.jpg"
    save_square_jpeg(src, out, image_size, 88)
    candidates.append(
        Candidate(
            label=label,
            source_dataset=source_dataset,
            split=split,
            source_file=source_file,
            source_url=source_url,
            path=out,
            sha256=sha256_file(out),
            candidate_id=out.stem,
        )
    )


def discover_dataset_candidates(dataset: Path, out_dir: Path, per_class: int, image_size: int) -> list[Candidate]:
    candidates: list[Candidate] = []
    exts = {".jpg", ".jpeg", ".png", ".webp", ".bmp"}
    for split in ("valid", "test", "train"):
        split_dir = dataset / split
        if not split_dir.exists():
            continue
        for label in FISH31_CLASSES:
            class_dir = split_dir / label
            if not class_dir.exists():
                continue
            count = 0
            for src in sorted(class_dir.rglob("*")):
                if count >= per_class:
                    break
                if src.suffix.lower() not in exts or not src.is_file():
                    continue
                try:
                    add_candidate(
                        candidates,
                        label,
                        "fish31_cls",
                        split,
                        str(src.relative_to(dataset)).replace("\\", "/"),
                        "",
                        src,
                        out_dir,
                        image_size,
                    )
                    count += 1
                except Exception as exc:
                    print(f"skip {src}: {exc}")
    return candidates


def fetch_json(url: str, retries: int, sleep_s: float) -> dict[str, Any] | None:
    req = urllib.request.Request(url, headers={"User-Agent": "fish31-validation-assets/1.0"})
    for attempt in range(1, retries + 1):
        try:
            with urllib.request.urlopen(req, timeout=45) as response:
                return json.loads(response.read().decode("utf-8"))
        except (urllib.error.URLError, TimeoutError, OSError, ssl.SSLError) as exc:
            wait = sleep_s * attempt * 3
            print(f"  iNat API retry {attempt}/{retries}: {exc}; wait {wait:.1f}s")
            time.sleep(wait)
    return None


def download_file(url: str, dst: Path, retries: int, sleep_s: float) -> bool:
    if dst.exists() and dst.stat().st_size > 1024:
        return True
    for attempt in range(1, retries + 1):
        try:
            req = urllib.request.Request(url, headers={"User-Agent": "fish31-validation-assets/1.0"})
            with urllib.request.urlopen(req, timeout=60) as response:
                data = response.read()
            if len(data) < 1024:
                return False
            dst.parent.mkdir(parents=True, exist_ok=True)
            dst.write_bytes(data)
            return True
        except (urllib.error.URLError, TimeoutError, OSError, ssl.SSLError) as exc:
            if attempt == retries:
                print(f"  skip image after {retries} retries: {exc}")
                return False
            time.sleep(sleep_s * attempt * 2)
    return False


def allowed_photo(photo: dict[str, Any]) -> bool:
    return str(photo.get("license_code") or "").lower() in CC_LICENSES


def inat_photo_url(photo: dict[str, Any], size: str) -> str | None:
    url = photo.get("url")
    if not url:
        return None
    return str(url).replace("square.", f"{size}.")


def download_inat_candidates(raw_dir: Path, per_class: int, photo_size: str, sleep_s: float) -> list[tuple[str, Path, str, str]]:
    downloaded: list[tuple[str, Path, str, str]] = []
    for label, species in INAT_SPECIES.items():
        class_dir = raw_dir / label
        class_dir.mkdir(parents=True, exist_ok=True)
        seen = {p.stem for p in class_dir.glob("*.jpg")}
        page = 1
        print(f"{label} / {species}: start")
        while len([x for x in downloaded if x[0] == label]) < per_class and page <= 20:
            params = {
                "taxon_name": species,
                "photos": "true",
                "quality_grade": "research",
                "page": str(page),
                "per_page": "80",
                "order": "desc",
                "order_by": "created_at",
            }
            payload = fetch_json(INAT_API + "?" + urllib.parse.urlencode(params), retries=5, sleep_s=sleep_s)
            if not payload or not payload.get("results"):
                break
            for obs in payload.get("results", []):
                for photo in obs.get("photos", []):
                    pid = str(photo.get("id") or "")
                    if not pid or pid in seen or not allowed_photo(photo):
                        continue
                    url = inat_photo_url(photo, photo_size)
                    if not url:
                        continue
                    dst = class_dir / f"{pid}.jpg"
                    if download_file(url, dst, retries=4, sleep_s=sleep_s):
                        seen.add(pid)
                        obs_url = f"https://www.inaturalist.org/observations/{obs.get('id')}"
                        downloaded.append((label, dst, obs_url, species))
                        print(f"  {label}: {len([x for x in downloaded if x[0] == label])}/{per_class} {dst.name}")
                    time.sleep(sleep_s)
                    if len([x for x in downloaded if x[0] == label]) >= per_class:
                        break
                if len([x for x in downloaded if x[0] == label]) >= per_class:
                    break
            page += 1
            time.sleep(sleep_s)
    return downloaded


def generate_inat_candidates(raw_dir: Path, out_dir: Path, per_class: int, image_size: int, photo_size: str) -> list[Candidate]:
    downloaded = download_inat_candidates(raw_dir, per_class, photo_size, sleep_s=0.2)
    candidates: list[Candidate] = []
    for label, src, obs_url, species in downloaded:
        try:
            add_candidate(candidates, label, "iNaturalist", "research-grade", src.name, obs_url, src, out_dir, image_size)
        except Exception as exc:
            print(f"skip {src}: {exc}")
    return candidates


def curl_request(method: str, url: str, interface: str | None = None, data_file: Path | None = None, timeout: int = 30) -> str:
    cmd = ["curl.exe", "--silent", "--show-error", "--fail-with-body", "--max-time", str(timeout)]
    if interface:
        cmd += ["--interface", interface]
    if method != "GET":
        cmd += ["-X", method]
    if data_file:
        cmd += ["--data-binary", f"@{data_file}"]
    cmd.append(url)
    proc = subprocess.run(cmd, capture_output=True, text=True)
    if proc.returncode != 0:
        raise RuntimeError(f"curl failed {proc.returncode}: {proc.stderr.strip()} {proc.stdout.strip()}")
    return proc.stdout


def board_json(base_url: str, path: str, interface: str | None, timeout: int = 30) -> dict[str, Any]:
    return json.loads(curl_request("GET", base_url.rstrip("/") + path, interface=interface, timeout=timeout))


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


def run_candidate_on_board(candidate: Candidate, base_url: str, interface: str | None, dataset: str, wake: bool) -> dict[str, Any]:
    base_url = base_url.rstrip("/")
    if wake:
        curl_request("GET", base_url + "/api/power?cmd=wake", interface=interface, timeout=10)
    curl_request("GET", base_url + "/api/recognition?method=fish31", interface=interface, timeout=10)
    curl_request("GET", base_url + "/api/config?inference_interval_ms=0", interface=interface, timeout=10)
    path = "frames/frame_00001.jpg"
    put_url = f"{base_url}/api/dataset/file?dataset={urllib.parse.quote(dataset)}&path={urllib.parse.quote(path)}"
    curl_request("PUT", put_url, interface=interface, data_file=candidate.path, timeout=45)
    start = board_json(
        base_url,
        f"/api/dataset/run/start?dataset={urllib.parse.quote(dataset)}&method=fish31&limit=1&stride=1",
        interface,
        timeout=20,
    )
    run_id = start.get("run_id", "")
    deadline = time.time() + 70
    status: dict[str, Any] = {}
    while time.time() < deadline:
        status = board_json(base_url, "/api/dataset/run/status", interface, timeout=10)
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
        "top1": "",
        "score": 0,
        "top_k": [],
        "raw_status": status,
    }
    result_uri = status.get("result_uri") or ""
    if result_uri:
        row = parse_jsonl_result(curl_request("GET", base_url + result_uri, interface=interface, timeout=20))
        if row:
            top_k = row.get("top_k") or []
            if top_k:
                result["top1"] = top_k[0].get("label", "")
                result["score"] = int(top_k[0].get("score", 0))
                result["top_k"] = top_k
            else:
                result["top1"] = row.get("label", "")
                result["score"] = int(row.get("best_score", 0))
                result["top_k"] = [{"label": result["top1"], "score": result["score"]}]
            result["inference_ms"] = row.get("inference_ms", 0)
            result["analysis_ms"] = row.get("analysis_ms", result["analysis_ms"])
    if not result["top1"] and status.get("labels"):
        first = status["labels"][0]
        result["top1"] = first.get("label", "")
        result["top_k"] = [{"label": result["top1"], "score": 0}]
    result["matched"] = result.get("top1") == candidate.label
    return result


def select_verified(candidates: list[Candidate], score_min: int, slots: int) -> list[Candidate]:
    verified = [
        c for c in candidates
        if c.result.get("ok")
        and c.result.get("top1") == c.label
        and int(c.result.get("score", 0)) >= score_min
    ]
    verified.sort(key=lambda c: (-int(c.result.get("score", 0)), int(c.result.get("analysis_ms", 999999))))
    selected: list[Candidate] = []
    seen_sha: set[str] = set()
    seen_label: set[str] = set()

    for c in verified:
        if c.sha256 in seen_sha or c.label in seen_label:
            continue
        selected.append(c)
        seen_sha.add(c.sha256)
        seen_label.add(c.label)
        if len(selected) >= slots:
            return selected

    for c in verified:
        if c.sha256 in seen_sha:
            continue
        selected.append(c)
        seen_sha.add(c.sha256)
        seen_label.add(c.label)
        if len(selected) >= slots:
            return selected
    return selected


def load_reuse_manifest(path: Path, candidates_dir: Path) -> tuple[list[Candidate], str, dict[str, Any]]:
    manifest = json.loads(path.read_text(encoding="utf-8"))
    candidates: list[Candidate] = []
    for item in manifest.get("candidates", []):
        candidate_id = item.get("candidate_id", "")
        if not candidate_id:
            continue
        image_path = candidates_dir / f"{candidate_id}.jpg"
        if not image_path.exists():
            raise FileNotFoundError(f"candidate image missing for reuse: {image_path}")
        result = item.get("result") or {}
        candidates.append(
            Candidate(
                label=item.get("label", ""),
                source_dataset=item.get("source_dataset", ""),
                split=item.get("split", ""),
                source_file=item.get("source_file", ""),
                source_url=item.get("source_url", ""),
                path=image_path,
                sha256=item.get("candidate_sha256") or sha256_file(image_path),
                candidate_id=candidate_id,
                result=result,
            )
        )
    source_policy = manifest.get("source_policy", "reused board manifest")
    board_status = {
        "model_info": (manifest.get("board") or {}).get("status_model", {}),
        "recognition_method": (manifest.get("board") or {}).get("recognition_method", ""),
    }
    return candidates, source_policy, board_status


def candidate_manifest(c: Candidate, extra: dict[str, Any] | None = None) -> dict[str, Any]:
    item = {
        "candidate_id": c.candidate_id,
        "label": c.label,
        "source_dataset": c.source_dataset,
        "split": c.split,
        "source_file": c.source_file,
        "source_url": c.source_url,
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


def copy_assets(selected: list[Candidate], asset_dir: Path, video_dir: Path, slots: int) -> tuple[list[dict[str, Any]], list[dict[str, Any]]]:
    if len(selected) < slots:
        raise RuntimeError(f"need {slots} board-verified Fish31 samples, got {len(selected)}")
    images_dir = asset_dir / "images"
    frames_dir = video_dir / "frames"
    reset_dir(images_dir)
    reset_dir(frames_dir)
    sample_manifest: list[dict[str, Any]] = []
    frame_manifest: list[dict[str, Any]] = []
    for i, c in enumerate(selected[:slots], start=1):
        out = images_dir / f"fish31_{i:02d}.jpg"
        shutil.copyfile(c.path, out)
        sample_manifest.append(candidate_manifest(c, {"slot": f"fish31_{i:02d}", "sha256": sha256_file(out)}))
    for i in range(1, 17):
        c = selected[(i - 1) % len(selected)]
        out = frames_dir / f"fish31_frame_{i:05d}.jpg"
        shutil.copyfile(c.path, out)
        frame_manifest.append(candidate_manifest(c, {"frame": i, "sha256": sha256_file(out)}))
    return sample_manifest, frame_manifest


def update_validation_header(header: Path, selected: list[Candidate]) -> None:
    if not header.exists():
        return
    text = header.read_text(encoding="utf-8")
    for i, c in enumerate(selected[:4], start=1):
        text = re.sub(
            rf'#define VALIDATION_FISH31_{i:02d}_LABEL ".*?"',
            f'#define VALIDATION_FISH31_{i:02d}_LABEL "{c.label}"',
            text,
        )
    header.write_text(text, encoding="utf-8")


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--dataset-root", type=Path, default=DEFAULT_DATASET)
    parser.add_argument("--work-dir", type=Path, default=DEFAULT_WORK_DIR)
    parser.add_argument("--asset-dir", type=Path, default=DEFAULT_ASSET_DIR)
    parser.add_argument("--video-dir", type=Path, default=DEFAULT_VIDEO_DIR)
    parser.add_argument("--config-header", type=Path, default=DEFAULT_CONFIG_HEADER)
    parser.add_argument("--board", default="http://169.254.100.2")
    parser.add_argument("--interface", default="")
    parser.add_argument("--score-min", type=int, default=70)
    parser.add_argument("--max-candidates-per-class", type=int, default=12)
    parser.add_argument("--sample-slots", type=int, default=4)
    parser.add_argument("--image-size", type=int, default=320)
    parser.add_argument("--photo-size", choices=["medium", "large", "original"], default="medium")
    parser.add_argument("--run-board", action="store_true")
    parser.add_argument("--reuse-manifest", type=Path)
    parser.add_argument("--wake", action="store_true")
    parser.add_argument("--dataset", default=BOARD_DATASET)
    return parser.parse_args()


def main() -> int:
    args = parse_args()
    candidates_dir = args.work_dir / "candidates"
    raw_dir = args.work_dir / "raw_inaturalist"

    board_status: dict[str, Any] = {}
    if args.reuse_manifest:
        candidates, source_policy, board_status = load_reuse_manifest(args.reuse_manifest, candidates_dir)
    elif args.dataset_root.exists():
        reset_dir(candidates_dir)
        candidates = discover_dataset_candidates(
            args.dataset_root, candidates_dir, args.max_candidates_per_class, args.image_size
        )
        source_policy = "data/fish31_cls standard dataset"
    else:
        reset_dir(candidates_dir)
        candidates = generate_inat_candidates(
            raw_dir, candidates_dir, args.max_candidates_per_class, args.image_size, args.photo_size
        )
        source_policy = "iNaturalist Creative Commons candidates because data/fish31_cls is absent"

    print(f"generated {len(candidates)} Fish31 candidates")
    print("candidate counts:", json.dumps({label: sum(c.label == label for c in candidates) for label in FISH31_CLASSES}))
    if not candidates:
        raise RuntimeError("no Fish31 candidates generated")

    if args.run_board and not args.reuse_manifest:
        interface = args.interface or None
        board_status = board_json(args.board, "/api/status", interface, timeout=10)
        print("board:", board_status.get("recognition_method"), board_status.get("model_info", {}))
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

    selected = select_verified(candidates, args.score_min, args.sample_slots)
    sample_manifest: list[dict[str, Any]] = []
    frame_manifest: list[dict[str, Any]] = []
    if (args.run_board or args.reuse_manifest) and len(selected) >= args.sample_slots:
        sample_manifest, frame_manifest = copy_assets(selected, args.asset_dir, args.video_dir, args.sample_slots)
        update_validation_header(args.config_header, selected)

    manifest = {
        "generated_at": now_iso(),
        "source_policy": source_policy,
        "score_min": args.score_min,
        "board": {
            "base_url": args.board,
            "interface": args.interface,
            "status_model": board_status.get("model_info", {}),
            "recognition_method": board_status.get("recognition_method", ""),
        },
        "selected_samples": sample_manifest,
        "selected_video_frames": frame_manifest,
        "candidates": [candidate_manifest(c) | {"result": c.result} for c in candidates],
    }
    args.asset_dir.mkdir(parents=True, exist_ok=True)
    args.video_dir.mkdir(parents=True, exist_ok=True)
    (args.asset_dir / "manifest.json").write_text(json.dumps(manifest, indent=2, ensure_ascii=False) + "\n", encoding="utf-8")
    (args.video_dir / "manifest.json").write_text(json.dumps(manifest, indent=2, ensure_ascii=False) + "\n", encoding="utf-8")
    print(f"selected labels: {[c.label for c in selected]}")
    print(f"wrote {args.asset_dir / 'manifest.json'}")
    return 0 if (not args.run_board and not args.reuse_manifest) or len(selected) >= args.sample_slots else 2


if __name__ == "__main__":
    raise SystemExit(main())
