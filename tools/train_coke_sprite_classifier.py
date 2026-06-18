#!/usr/bin/env python3
"""Train a lightweight Coke/Sprite image classifier for ESP32-P4.

This version intentionally does not use hand-written color thresholds.  It
trains a tiny 16x16 RGB pixel MLP from a public YOLO dataset and emits a C
header that can be compiled into the ESP-IDF firmware.

Dataset source used by default:
    Hugging Face dataset LibreYOLO/soda-bottles

Expected dataset layout:
    data/public_soda_bottles/
      data.yaml
      train/images/*.jpg
      train/labels/*.txt
      valid/images/*.jpg
      valid/labels/*.txt
      test/images/*.jpg
      test/labels/*.txt
"""

from __future__ import annotations

import argparse
import json
import math
import random
import shutil
from dataclasses import dataclass
from pathlib import Path

import numpy as np
from PIL import Image, ImageEnhance


CLASSES = ["unknown", "coke", "sprite"]
YOLO_TO_CLASS = {0: 1, 1: 0, 2: 2}  # coca-cola, fanta, sprite
INPUT_W = 16
INPUT_H = 16
INPUT_C = 3
INPUT_N = INPUT_W * INPUT_H * INPUT_C


@dataclass(frozen=True)
class Box:
    cls: int
    cx: float
    cy: float
    w: float
    h: float


@dataclass(frozen=True)
class Example:
    image: Path
    label: int
    box: Box
    tag: str


def parse_yolo_label(path: Path) -> list[Box]:
    boxes: list[Box] = []
    if not path.exists():
        return boxes
    for line in path.read_text(errors="ignore").splitlines():
        parts = line.split()
        if len(parts) < 5:
            continue
        cls = int(float(parts[0]))
        boxes.append(Box(cls, *(float(v) for v in parts[1:5])))
    return boxes


def collect_examples(root: Path, split: str, max_per_class: int, seed: int) -> list[Example]:
    rng = random.Random(seed)
    image_dir = root / split / "images"
    label_dir = root / split / "labels"
    by_class: list[list[Example]] = [[] for _ in CLASSES]

    for image_path in sorted(image_dir.glob("*.jpg")):
        boxes = parse_yolo_label(label_dir / f"{image_path.stem}.txt")
        if not boxes:
            continue
        for box in boxes:
            if box.cls not in YOLO_TO_CLASS:
                continue
            label = YOLO_TO_CLASS[box.cls]
            by_class[label].append(Example(image_path, label, box, "object"))

        for bg_box in random_background_boxes(boxes, rng, count=2):
            by_class[0].append(Example(image_path, 0, bg_box, "background"))

    examples: list[Example] = []
    for label, items in enumerate(by_class):
        rng.shuffle(items)
        if max_per_class > 0:
            items = items[:max_per_class]
        examples.extend(items)
        print(f"{split:5s} {CLASSES[label]:7s}: {len(items)}")
    rng.shuffle(examples)
    return examples


def collect_extra_classification_examples(root: Path, max_per_class: int, seed: int) -> list[Example]:
    rng = random.Random(seed)
    by_class: list[list[Example]] = [[] for _ in CLASSES]
    if not root.exists():
        return []

    suffixes = {".jpg", ".jpeg", ".png", ".bmp", ".webp"}
    for label, name in enumerate(CLASSES):
        folder = root / name
        if not folder.exists():
            continue
        for image_path in sorted(folder.rglob("*")):
            if image_path.suffix.lower() not in suffixes:
                continue
            by_class[label].append(Example(image_path, label, Box(100, 0.5, 0.5, 1.0, 1.0), "extra-full"))
            if label == 0:
                for _ in range(5):
                    side = rng.uniform(0.25, 0.75)
                    cx = rng.uniform(side / 2.0, 1.0 - side / 2.0)
                    cy = rng.uniform(side / 2.0, 1.0 - side / 2.0)
                    by_class[label].append(Example(image_path, label, Box(99, cx, cy, side, side), "extra-crop"))

    examples: list[Example] = []
    for label, items in enumerate(by_class):
        rng.shuffle(items)
        if max_per_class > 0:
            items = items[:max_per_class]
        examples.extend(items)
        if items:
            print(f"extra {CLASSES[label]:7s}: {len(items)}")
    rng.shuffle(examples)
    return examples


def box_to_xyxy(box: Box, width: int, height: int) -> tuple[float, float, float, float]:
    x1 = (box.cx - box.w / 2.0) * width
    y1 = (box.cy - box.h / 2.0) * height
    x2 = (box.cx + box.w / 2.0) * width
    y2 = (box.cy + box.h / 2.0) * height
    return x1, y1, x2, y2


def iou(a: Box, b: Box) -> float:
    ax1, ay1, ax2, ay2 = a.cx - a.w / 2.0, a.cy - a.h / 2.0, a.cx + a.w / 2.0, a.cy + a.h / 2.0
    bx1, by1, bx2, by2 = b.cx - b.w / 2.0, b.cy - b.h / 2.0, b.cx + b.w / 2.0, b.cy + b.h / 2.0
    ix1, iy1 = max(ax1, bx1), max(ay1, by1)
    ix2, iy2 = min(ax2, bx2), min(ay2, by2)
    iw, ih = max(0.0, ix2 - ix1), max(0.0, iy2 - iy1)
    inter = iw * ih
    area_a = max(1e-6, a.w * a.h)
    area_b = max(1e-6, b.w * b.h)
    return inter / (area_a + area_b - inter + 1e-6)


def random_background_boxes(boxes: list[Box], rng: random.Random, count: int) -> list[Box]:
    out: list[Box] = []
    for _ in range(count * 30):
        if len(out) >= count:
            break
        side = rng.uniform(0.18, 0.42)
        cx = rng.uniform(side / 2.0, 1.0 - side / 2.0)
        cy = rng.uniform(side / 2.0, 1.0 - side / 2.0)
        candidate = Box(99, cx, cy, side, side)
        if all(iou(candidate, b) < 0.04 for b in boxes):
            out.append(candidate)
    return out


def crop_square(img: Image.Image, box: Box, rng: random.Random, train: bool) -> Image.Image:
    width, height = img.size
    x1, y1, x2, y2 = box_to_xyxy(box, width, height)
    bw = max(2.0, x2 - x1)
    bh = max(2.0, y2 - y1)
    cx = (x1 + x2) / 2.0
    cy = (y1 + y2) / 2.0

    pad = rng.uniform(1.10, 1.65) if train else 1.28
    side = max(bw, bh) * pad
    if train and box.cls != 99:
        cx += rng.uniform(-0.12, 0.12) * bw
        cy += rng.uniform(-0.12, 0.12) * bh

    left = int(round(cx - side / 2.0))
    top = int(round(cy - side / 2.0))
    right = int(round(cx + side / 2.0))
    bottom = int(round(cy + side / 2.0))
    canvas = Image.new("RGB", (max(1, right - left), max(1, bottom - top)), (118, 118, 118))
    paste_x = max(0, -left)
    paste_y = max(0, -top)
    source = img.crop((max(0, left), max(0, top), min(width, right), min(height, bottom)))
    canvas.paste(source, (paste_x, paste_y))
    return canvas


def augment(img: Image.Image, rng: random.Random) -> Image.Image:
    if rng.random() < 0.5:
        img = img.transpose(Image.Transpose.FLIP_LEFT_RIGHT)
    img = ImageEnhance.Brightness(img).enhance(rng.uniform(0.72, 1.25))
    img = ImageEnhance.Contrast(img).enhance(rng.uniform(0.80, 1.28))
    img = ImageEnhance.Color(img).enhance(rng.uniform(0.82, 1.22))
    return img


def image_to_vector(img: Image.Image) -> np.ndarray:
    img = img.resize((INPUT_W, INPUT_H), Image.Resampling.BILINEAR)
    arr = np.asarray(img, dtype=np.float32)
    return (arr.reshape(-1) / 127.5) - 1.0


def materialize(examples: list[Example], train: bool, seed: int) -> tuple[np.ndarray, np.ndarray]:
    rng = random.Random(seed)
    grouped: dict[Path, list[Example]] = {}
    for ex in examples:
        grouped.setdefault(ex.image, []).append(ex)

    xs: list[np.ndarray] = []
    ys: list[int] = []
    for image_path, items in grouped.items():
        try:
            img = Image.open(image_path).convert("RGB")
        except OSError:
            continue
        for ex in items:
            crop = crop_square(img, ex.box, rng, train=train)
            if train:
                crop = augment(crop, rng)
            xs.append(image_to_vector(crop))
            ys.append(ex.label)
    if not xs:
        raise SystemExit("No usable training images found.")
    x = np.stack(xs).astype(np.float32)
    y = np.asarray(ys, dtype=np.int64)
    return x, y


def softmax(logits: np.ndarray) -> np.ndarray:
    logits = logits - logits.max(axis=1, keepdims=True)
    exp = np.exp(logits)
    return exp / exp.sum(axis=1, keepdims=True)


def accuracy(logits: np.ndarray, y: np.ndarray) -> float:
    return float(np.mean(np.argmax(logits, axis=1) == y))


def confusion_matrix(logits: np.ndarray, y: np.ndarray) -> list[list[int]]:
    pred = np.argmax(logits, axis=1)
    m = np.zeros((len(CLASSES), len(CLASSES)), dtype=np.int64)
    for truth, guess in zip(y, pred):
        m[int(truth), int(guess)] += 1
    return m.tolist()


def init_model(hidden: int, seed: int) -> dict[str, np.ndarray]:
    rng = np.random.default_rng(seed)
    return {
        "w1": rng.normal(0.0, math.sqrt(2.0 / INPUT_N), size=(INPUT_N, hidden)).astype(np.float32),
        "b1": np.zeros((hidden,), dtype=np.float32),
        "w2": rng.normal(0.0, math.sqrt(2.0 / hidden), size=(hidden, len(CLASSES))).astype(np.float32),
        "b2": np.zeros((len(CLASSES),), dtype=np.float32),
    }


def forward(model: dict[str, np.ndarray], x: np.ndarray) -> tuple[np.ndarray, np.ndarray]:
    hidden = np.maximum(0.0, x @ model["w1"] + model["b1"])
    logits = hidden @ model["w2"] + model["b2"]
    return hidden, logits


def train_mlp(
    x_train: np.ndarray,
    y_train: np.ndarray,
    x_val: np.ndarray,
    y_val: np.ndarray,
    hidden: int,
    epochs: int,
    batch_size: int,
    lr: float,
    seed: int,
) -> tuple[dict[str, np.ndarray], dict[str, float]]:
    rng = np.random.default_rng(seed)
    model = init_model(hidden, seed)
    moments = {name: np.zeros_like(value) for name, value in model.items()}
    velocities = {name: np.zeros_like(value) for name, value in model.items()}
    best = {name: value.copy() for name, value in model.items()}
    best_val = -1.0
    step = 0

    for epoch in range(1, epochs + 1):
        order = rng.permutation(len(x_train))
        for start in range(0, len(order), batch_size):
            step += 1
            idx = order[start:start + batch_size]
            xb = x_train[idx]
            yb = y_train[idx]
            hidden_act, logits = forward(model, xb)
            probs = softmax(logits)
            probs[np.arange(len(yb)), yb] -= 1.0
            probs /= len(yb)

            grads: dict[str, np.ndarray] = {}
            grads["w2"] = hidden_act.T @ probs
            grads["b2"] = probs.sum(axis=0)
            dh = probs @ model["w2"].T
            dh[hidden_act <= 0.0] = 0.0
            grads["w1"] = xb.T @ dh
            grads["b1"] = dh.sum(axis=0)

            for name in model:
                moments[name] = 0.9 * moments[name] + 0.1 * grads[name]
                velocities[name] = 0.999 * velocities[name] + 0.001 * (grads[name] * grads[name])
                mhat = moments[name] / (1.0 - 0.9 ** step)
                vhat = velocities[name] / (1.0 - 0.999 ** step)
                model[name] -= lr * mhat / (np.sqrt(vhat) + 1e-8)

        _, train_logits = forward(model, x_train)
        _, val_logits = forward(model, x_val)
        train_acc = accuracy(train_logits, y_train)
        val_acc = accuracy(val_logits, y_val)
        if val_acc > best_val:
            best_val = val_acc
            best = {name: value.copy() for name, value in model.items()}
        print(f"epoch {epoch:02d}: train_acc={train_acc:.4f} val_acc={val_acc:.4f}")

    return best, {"best_val_acc": best_val}


def write_float_array(f, name: str, arr: np.ndarray, dims: str) -> None:
    arr = arr.astype(np.float32)
    f.write(f"static const float {name}{dims} = {{\n")
    if arr.ndim == 1:
        flat = arr.reshape(-1)
        for i in range(0, len(flat), 8):
            values = ", ".join(f"{v:.7g}f" for v in flat[i:i + 8])
            f.write(f"    {values},\n")
    elif arr.ndim == 2:
        for row in arr:
            values = ", ".join(f"{v:.7g}f" for v in row)
            f.write(f"    {{ {values} }},\n")
    else:
        raise ValueError(f"{name} must be 1D or 2D")
    f.write("};\n\n")


def write_header(path: Path, model: dict[str, np.ndarray], report: dict) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    hidden = int(model["b1"].shape[0])
    with path.open("w", encoding="utf-8", newline="\n") as f:
        f.write("/* Generated by tools/train_coke_sprite_classifier.py. */\n")
        f.write("#pragma once\n\n")
        f.write("#define CAN_CLASSIFIER_MODEL_NAME \"can-mlp16-public-soda-v1\"\n")
        f.write(f"#define CAN_CLASSIFIER_INPUT_W {INPUT_W}\n")
        f.write(f"#define CAN_CLASSIFIER_INPUT_H {INPUT_H}\n")
        f.write(f"#define CAN_CLASSIFIER_INPUT_C {INPUT_C}\n")
        f.write(f"#define CAN_CLASSIFIER_INPUTS {INPUT_N}\n")
        f.write(f"#define CAN_CLASSIFIER_HIDDEN {hidden}\n")
        f.write(f"#define CAN_CLASSIFIER_CLASSES {len(CLASSES)}\n")
        f.write(f"#define CAN_CLASSIFIER_TRAIN_SAMPLES {report['train_samples']}\n")
        f.write(f"#define CAN_CLASSIFIER_VALID_ACC_X100 {int(round(report['valid_acc'] * 10000))}\n")
        f.write(f"#define CAN_CLASSIFIER_TEST_ACC_X100 {int(round(report['test_acc'] * 10000))}\n")
        f.write("#define CAN_CLASSIFIER_INPUT_SCALE 0.0078431373f\n")
        f.write("#define CAN_CLASSIFIER_INPUT_OFFSET -1.0f\n\n")
        f.write("static const char *const CAN_CLASSIFIER_LABELS[CAN_CLASSIFIER_CLASSES] = {\n")
        for cls in CLASSES:
            f.write(f"    \"{cls}\",\n")
        f.write("};\n\n")
        write_float_array(f, "CAN_CLASSIFIER_W1", model["w1"].T, f"[CAN_CLASSIFIER_HIDDEN][CAN_CLASSIFIER_INPUTS]")
        write_float_array(f, "CAN_CLASSIFIER_B1", model["b1"], "[CAN_CLASSIFIER_HIDDEN]")
        write_float_array(f, "CAN_CLASSIFIER_W2", model["w2"].T, f"[CAN_CLASSIFIER_CLASSES][CAN_CLASSIFIER_HIDDEN]")
        write_float_array(f, "CAN_CLASSIFIER_B2", model["b2"], "[CAN_CLASSIFIER_CLASSES]")


def save_report(path: Path, report: dict) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_text(json.dumps(report, ensure_ascii=False, indent=2), encoding="utf-8")


def make_phone_gallery(root: Path, output: Path, count_per_class: int, seed: int) -> None:
    rng = random.Random(seed)
    if output.exists():
        shutil.rmtree(output)
    (output / "images").mkdir(parents=True, exist_ok=True)
    examples = collect_examples(root, "test", count_per_class * 4, seed)
    picked: dict[int, list[Example]] = {1: [], 2: []}
    for ex in examples:
        if ex.label in picked and len(picked[ex.label]) < count_per_class:
            picked[ex.label].append(ex)

    pages: list[tuple[str, str]] = []
    for label, items in picked.items():
        for index, ex in enumerate(items, 1):
            img = Image.open(ex.image).convert("RGB")
            crop = crop_square(img, ex.box, rng, train=False).resize((640, 640), Image.Resampling.BICUBIC)
            name = f"{CLASSES[label]}_{index:02d}.jpg"
            crop.save(output / "images" / name, quality=92)
            page = f"{CLASSES[label]}_{index:02d}.html"
            (output / page).write_text(
                "<!doctype html><html><head><meta charset='utf-8'>"
                "<meta name='viewport' content='width=device-width,initial-scale=1'>"
                "<style>html,body{margin:0;height:100%;background:#050505;display:grid;place-items:center}"
                "img{max-width:100vw;max-height:100vh;width:auto;height:auto}</style></head>"
                f"<body><img src='images/{name}'></body></html>",
                encoding="utf-8",
            )
            pages.append((page, name))

    links = "\n".join(
        f"<a href='{page}'><img src='images/{name}'><span>{page[:-5]}</span></a>"
        for page, name in pages
    )
    (output / "index.html").write_text(
        "<!doctype html><html><head><meta charset='utf-8'>"
        "<meta name='viewport' content='width=device-width,initial-scale=1'>"
        "<style>body{font-family:Arial,sans-serif;margin:16px;background:#111;color:#eee}"
        ".grid{display:grid;grid-template-columns:repeat(auto-fill,minmax(120px,1fr));gap:12px}"
        "a{color:#eee;text-decoration:none;border:1px solid #333;padding:8px;background:#1b1b1b}"
        "img{width:100%;display:block}span{display:block;margin-top:6px;font-size:13px}</style></head>"
        "<body><h2>可乐 / 雪碧手机测试图</h2><p>点击任意图片进入全屏，再用开发板摄像头对准手机屏幕测试。</p>"
        f"<div class='grid'>{links}</div></body></html>",
        encoding="utf-8",
    )
    print(f"phone gallery: {output / 'index.html'}")


def main() -> None:
    parser = argparse.ArgumentParser()
    parser.add_argument("--dataset", default="data/public_soda_bottles")
    parser.add_argument("--extra-classification-root", default="data/board_samples")
    parser.add_argument("--output", default="main/coke_sprite_mlp_model.h")
    parser.add_argument("--report", default="build/coke_sprite_classifier_report.json")
    parser.add_argument("--phone-gallery", default="test_assets/phone_gallery")
    parser.add_argument("--hidden", type=int, default=24)
    parser.add_argument("--epochs", type=int, default=18)
    parser.add_argument("--batch-size", type=int, default=192)
    parser.add_argument("--lr", type=float, default=0.0015)
    parser.add_argument("--max-train-per-class", type=int, default=6000)
    parser.add_argument("--max-eval-per-class", type=int, default=900)
    parser.add_argument("--max-extra-per-class", type=int, default=1500)
    parser.add_argument("--seed", type=int, default=20260529)
    args = parser.parse_args()

    root = Path(args.dataset)
    train_examples = collect_examples(root, "train", args.max_train_per_class, args.seed)
    extra_examples = collect_extra_classification_examples(
        Path(args.extra_classification_root), args.max_extra_per_class, args.seed + 3
    )
    train_examples.extend(extra_examples)
    valid_examples = collect_examples(root, "valid", args.max_eval_per_class, args.seed + 1)
    test_examples = collect_examples(root, "test", args.max_eval_per_class, args.seed + 2)

    print("materializing images...")
    x_train, y_train = materialize(train_examples, train=True, seed=args.seed + 10)
    x_val, y_val = materialize(valid_examples, train=False, seed=args.seed + 11)
    x_test, y_test = materialize(test_examples, train=False, seed=args.seed + 12)

    print(f"x_train={x_train.shape} x_val={x_val.shape} x_test={x_test.shape}")
    model, train_info = train_mlp(
        x_train, y_train, x_val, y_val,
        hidden=args.hidden,
        epochs=args.epochs,
        batch_size=args.batch_size,
        lr=args.lr,
        seed=args.seed + 20,
    )

    _, train_logits = forward(model, x_train)
    _, valid_logits = forward(model, x_val)
    _, test_logits = forward(model, x_test)
    report = {
        "model": "can-mlp16-public-soda-v1",
        "dataset": str(root),
        "classes": CLASSES,
        "input": [INPUT_W, INPUT_H, INPUT_C],
        "hidden": args.hidden,
        "train_samples": int(len(y_train)),
        "extra_samples": int(len(extra_examples)),
        "valid_samples": int(len(y_val)),
        "test_samples": int(len(y_test)),
        "train_acc": accuracy(train_logits, y_train),
        "valid_acc": accuracy(valid_logits, y_val),
        "test_acc": accuracy(test_logits, y_test),
        "best_val_acc": train_info["best_val_acc"],
        "test_confusion_matrix": confusion_matrix(test_logits, y_test),
        "note": "Trained from public YOLO bottle crops; Fanta/background are treated as unknown.",
    }
    print(json.dumps(report, ensure_ascii=False, indent=2))
    write_header(Path(args.output), model, report)
    save_report(Path(args.report), report)
    make_phone_gallery(root, Path(args.phone_gallery), count_per_class=12, seed=args.seed + 30)
    print(f"wrote {args.output}")
    print(f"wrote {args.report}")


if __name__ == "__main__":
    main()
