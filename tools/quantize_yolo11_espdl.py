#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""把 Coke/Sprite YOLO11n six-output ONNX 量化为 ESP32-P4 `.espdl`。

输入 ONNX 必须来自 `train_yolo11_coke_sprite.py`，输出名固定为：

    box0, score0, box1, score1, box2, score2

这与 ESP-DL 的 `dl::detect::yolo11PostProcessor` 完全一致。量化流程基于
乐鑫官方 `quantize_yolo11n/quantize_onnx_model.py`，但做了三处工程化调整：

1. 使用本项目的 Coke/Sprite 数据集做校准，不下载 COCO 校准包；
2. 校准预处理采用 114 灰边 letterbox，和板端 ImagePreprocessor 一致；
3. 在 ESP-DL exporter 写 FlatBuffer 前修复 0 维参数 shape，避免 P4 端
   `dl::Model` 加载时因空 shape 参数崩溃。
"""

from __future__ import annotations

import argparse
import random
from pathlib import Path
from typing import Iterable

import numpy as np
import torch
from PIL import Image
from torch.utils.data import DataLoader, Dataset


ROOT = Path(__file__).resolve().parents[1]


def iter_images(roots: Iterable[Path], limit: int) -> list[Path]:
    """收集 PTQ 校准图片，优先使用训练、验证和手机图库样例。"""
    exts = {".jpg", ".jpeg", ".png", ".bmp", ".webp"}
    files: list[Path] = []
    for root in roots:
        if root.exists():
            files.extend(p for p in root.rglob("*") if p.suffix.lower() in exts)
    files = sorted(dict.fromkeys(files))
    return files[:limit]


class LetterboxCalibrationDataset(Dataset):
    """YOLO11 PTQ 校准集。

    板端 `ImagePreprocessor.enable_letterbox({114,114,114})` 会按比例缩放并
    灰边填充，所以这里也用相同策略，避免校准分布和真实推理输入不一致。
    返回张量格式为 NCHW、范围 0..1。
    """

    def __init__(self, images: list[Path], size: int) -> None:
        self.images = images
        self.size = size

    def __len__(self) -> int:
        return len(self.images)

    def __getitem__(self, index: int) -> torch.Tensor:
        img = Image.open(self.images[index]).convert("RGB")
        w, h = img.size
        scale = min(self.size / w, self.size / h)
        nw, nh = int(round(w * scale)), int(round(h * scale))
        resized = img.resize((nw, nh), Image.BILINEAR)
        canvas = Image.new("RGB", (self.size, self.size), (114, 114, 114))
        canvas.paste(resized, ((self.size - nw) // 2, (self.size - nh) // 2))
        arr = np.asarray(canvas, dtype=np.float32).transpose(2, 0, 1) / 255.0
        return torch.from_numpy(arr)


def patch_espdl_scalar_exporter() -> None:
    """修复 ESP-DL exporter 写出的 0 维参数 shape。

    这和 YOLO26 脚本中的补丁相同：只把单值参数 reshape 为 `[1]`，不改变
    scale/zero-point 数值。它用于规避当前 ESP-PPQ + P4 fbs loader 对空 shape
    参数不够稳健的问题。
    """
    try:
        from esp_ppq.parser.espdl_exporter import EspdlExporter
    except Exception as exc:
        print(f"Skip ESP-DL scalar exporter patch: {exc}")
        return

    if getattr(EspdlExporter, "_buoy_scalar_shape_patch", False):
        return

    original = EspdlExporter.build_variable_proto

    def patched(self, variable, exponent, layout, perm=None):
        value = getattr(variable, "value", None)
        if getattr(variable, "is_parameter", False) and value is not None:
            try:
                value_shape = list(value.shape)
                value_size = int(value.numel()) if hasattr(value, "numel") else int(value.size)
            except AttributeError:
                scalar = np.asarray(value)
                value_shape = list(scalar.shape)
                value_size = int(scalar.size)
                value = scalar
                variable.value = scalar
            if value_size >= 1 and (value_shape == [] or getattr(variable, "shape", None) in (None, [])):
                fixed_shape = value_shape if value_shape else [1]
                try:
                    if hasattr(value, "reshape"):
                        variable.value = value.reshape(fixed_shape)
                except Exception:
                    pass
                variable.shape = fixed_shape
        return original(self, variable, exponent, layout, perm)

    EspdlExporter.build_variable_proto = patched
    EspdlExporter._buoy_scalar_shape_patch = True
    print("Patched ESP-DL exporter scalar parameter shape handling.")


def seed_everything(seed: int) -> None:
    random.seed(seed)
    np.random.seed(seed)
    torch.manual_seed(seed)
    if torch.cuda.is_available():
        torch.cuda.manual_seed_all(seed)


def main() -> None:
    parser = argparse.ArgumentParser(description="Quantize Coke/Sprite YOLO11n ONNX to ESP-DL")
    parser.add_argument("--onnx", default="models/yolo11_coke_sprite_416.onnx")
    parser.add_argument("--output", default="models/yolo11_coke_sprite_416_s8_p4.espdl")
    parser.add_argument("--input-size", type=int, default=416)
    parser.add_argument("--calib-limit", type=int, default=128)
    parser.add_argument("--calib-steps", type=int, default=32)
    parser.add_argument("--batch-size", type=int, default=8)
    parser.add_argument("--device", default="cpu")
    parser.add_argument("--target", default="esp32p4")
    parser.add_argument("--bits", type=int, default=8)
    parser.add_argument("--verbose", type=int, default=0)
    args = parser.parse_args()

    seed_everything(1234)
    patch_espdl_scalar_exporter()

    from esp_ppq import QuantizationSettingFactory
    from esp_ppq.api import espdl_quantize_onnx

    onnx_path = ROOT / args.onnx
    output_path = ROOT / args.output
    if not onnx_path.exists():
        raise SystemExit(f"找不到 YOLO11 ONNX: {onnx_path}")

    roots = [
        ROOT / "data" / "yolo26_coke_sprite" / "train" / "images",
        ROOT / "data" / "yolo26_coke_sprite" / "valid" / "images",
        ROOT / "test_assets" / "phone_gallery" / "images",
    ]
    images = iter_images(roots, args.calib_limit)
    if not images:
        raise SystemExit("找不到校准图片，请先准备 data/yolo26_coke_sprite 或 test_assets/phone_gallery。")

    dataset = LetterboxCalibrationDataset(images, args.input_size)
    dataloader = DataLoader(dataset=dataset, batch_size=args.batch_size, shuffle=False)
    calib_steps = min(args.calib_steps, len(dataloader))
    output_path.parent.mkdir(parents=True, exist_ok=True)

    print(
        f"Quantizing YOLO11: onnx={onnx_path}, output={output_path}, "
        f"images={len(dataset)}, steps={calib_steps}, device={args.device}"
    )
    setting = QuantizationSettingFactory.espdl_setting()

    def collate_fn(batch: torch.Tensor) -> torch.Tensor:
        return batch.to(args.device).type(torch.float32)

    espdl_quantize_onnx(
        onnx_import_file=str(onnx_path),
        espdl_export_file=str(output_path),
        calib_dataloader=dataloader,
        calib_steps=calib_steps,
        input_shape=[1, 3, args.input_size, args.input_size],
        target=args.target,
        num_of_bits=args.bits,
        collate_fn=collate_fn,
        setting=setting,
        device=args.device,
        error_report=True,
        skip_export=False,
        export_test_values=False,
        verbose=args.verbose,
        inputs=None,
    )
    print("Done.")


if __name__ == "__main__":
    main()
