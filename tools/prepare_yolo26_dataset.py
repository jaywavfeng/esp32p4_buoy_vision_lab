#!/usr/bin/env python3
"""Prepare a Coke/Sprite YOLO dataset for the buoy vision lab.

The source dataset uses the original soda-bottles class ids:
0 = Coca-Cola, 1 = Fanta/background-like class for this project, 2 = Sprite.
This script remaps only Coke and Sprite into a two-class detector dataset and
optionally copies board-captured unknown images as empty-label negative samples.
"""

from __future__ import annotations

import argparse
import shutil
from pathlib import Path


CLASS_MAP = {0: 0, 2: 1}
CLASS_NAMES = ["coke", "sprite"]
IMAGE_SUFFIXES = {".jpg", ".jpeg", ".png", ".bmp", ".webp"}


def convert_label(src: Path, dst: Path) -> int:
    kept = 0
    lines: list[str] = []
    if src.exists():
        for line in src.read_text(encoding="utf-8", errors="ignore").splitlines():
            parts = line.split()
            if len(parts) < 5:
                continue
            old_cls = int(float(parts[0]))
            if old_cls not in CLASS_MAP:
                continue
            parts[0] = str(CLASS_MAP[old_cls])
            lines.append(" ".join(parts[:5]))
            kept += 1
    dst.parent.mkdir(parents=True, exist_ok=True)
    dst.write_text("\n".join(lines) + ("\n" if lines else ""), encoding="utf-8")
    return kept


def copy_split(source_root: Path, output_root: Path, split: str) -> tuple[int, int]:
    image_dir = source_root / split / "images"
    label_dir = source_root / split / "labels"
    out_images = output_root / split / "images"
    out_labels = output_root / split / "labels"
    out_images.mkdir(parents=True, exist_ok=True)
    out_labels.mkdir(parents=True, exist_ok=True)

    images = 0
    boxes = 0
    for image in sorted(image_dir.iterdir()):
        if image.suffix.lower() not in IMAGE_SUFFIXES:
            continue
        label_src = label_dir / f"{image.stem}.txt"
        label_dst = out_labels / f"{image.stem}.txt"
        kept = convert_label(label_src, label_dst)
        if kept == 0:
            label_dst.unlink(missing_ok=True)
            continue
        shutil.copy2(image, out_images / image.name)
        images += 1
        boxes += kept
    return images, boxes


def copy_unknown_negatives(board_root: Path, output_root: Path, split: str, limit: int) -> int:
    unknown_dir = board_root / "unknown"
    if not unknown_dir.exists() or limit <= 0:
        return 0
    out_images = output_root / split / "images"
    out_labels = output_root / split / "labels"
    out_images.mkdir(parents=True, exist_ok=True)
    out_labels.mkdir(parents=True, exist_ok=True)

    copied = 0
    for image in sorted(unknown_dir.iterdir()):
        if image.suffix.lower() not in IMAGE_SUFFIXES:
            continue
        name = f"board_unknown_{copied:04d}{image.suffix.lower()}"
        shutil.copy2(image, out_images / name)
        (out_labels / f"board_unknown_{copied:04d}.txt").write_text("", encoding="utf-8")
        copied += 1
        if copied >= limit:
            break
    return copied


def write_yaml(output_root: Path) -> None:
    data_yaml = output_root / "data.yaml"
    data_yaml.write_text(
        "path: " + output_root.resolve().as_posix() + "\n"
        "train: train/images\n"
        "val: valid/images\n"
        "test: test/images\n"
        "names:\n"
        "  0: coke\n"
        "  1: sprite\n",
        encoding="utf-8",
    )


def main() -> None:
    parser = argparse.ArgumentParser()
    parser.add_argument("--source", default="data/public_soda_bottles")
    parser.add_argument("--board-samples", default="data/board_samples")
    parser.add_argument("--output", default="data/yolo26_coke_sprite")
    parser.add_argument("--unknown-limit", type=int, default=80)
    parser.add_argument("--clean", action="store_true")
    args = parser.parse_args()

    source = Path(args.source)
    output = Path(args.output)
    if args.clean and output.exists():
        shutil.rmtree(output)

    total_images = 0
    total_boxes = 0
    for split in ("train", "valid", "test"):
        images, boxes = copy_split(source, output, split)
        total_images += images
        total_boxes += boxes
        print(f"{split:5s}: images={images} boxes={boxes}")

    negatives = copy_unknown_negatives(Path(args.board_samples), output, "train", args.unknown_limit)
    write_yaml(output)
    print(f"unknown negatives copied to train: {negatives}")
    print(f"total images={total_images + negatives} boxes={total_boxes}")
    print(f"dataset yaml: {output / 'data.yaml'}")


if __name__ == "__main__":
    main()
