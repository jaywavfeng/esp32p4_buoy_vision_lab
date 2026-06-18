#!/usr/bin/env python3
"""把 YOLO26 raw ONNX 量化转换为 ESP-DL `.espdl`。

本脚本固化本工程已经验证过的转换路线：

- 输入：raw ONNX，形状为 `[1, 3, 416, 416]`，输出为 YOLO26 检测头原始张量。
- 修复：部分导出图中 `Concat/Slice` 等节点的负 axis 会让 ESP-PPQ 解析不稳定，
  所以先统一转为正 axis。
- 校准：从训练集、验证集和手机测试图库抽样，做 416 letterbox + RGB + 0..1 浮点输入。
- 输出：`models/yolo26_coke_sprite.espdl`，同时保留 ESP-PPQ 生成的 JSON/INFO 文件。
"""

from __future__ import annotations

import argparse
from pathlib import Path
from typing import Iterable

import numpy as np


def iter_images(roots: Iterable[Path], limit: int) -> list[Path]:
    exts = {".jpg", ".jpeg", ".png", ".bmp"}
    files: list[Path] = []
    for root in roots:
        if root.exists():
            files.extend(p for p in root.rglob("*") if p.suffix.lower() in exts)
    files = sorted(dict.fromkeys(files))
    return files[:limit]


def fix_negative_axes(src: Path, dst: Path) -> None:
    """把 ONNX 节点属性中的负 axis 转为正数，减少转换器兼容性问题。"""
    import onnx

    model = onnx.load(src)
    for node in model.graph.node:
        for attr in node.attribute:
            if attr.name == "axis" and attr.i < 0:
                # YOLO26 raw 输出主要是 NCHW/NCx 张量；负 axis 在这里等价于最后几维。
                attr.i = 4 + attr.i
    dst.parent.mkdir(parents=True, exist_ok=True)
    onnx.save(model, dst)


class ImageCalibDataset:
    """ESP-PPQ DataLoader 使用的轻量校准数据集。"""

    def __init__(self, images: list[Path], size: int) -> None:
        self.images = images
        self.size = size

    def __len__(self) -> int:
        return len(self.images)

    def __getitem__(self, index: int):
        import cv2
        import torch

        path = str(self.images[index])
        bgr = cv2.imread(path, cv2.IMREAD_COLOR)
        if bgr is None:
            raise RuntimeError(f"cannot read calibration image: {path}")
        rgb = cv2.cvtColor(bgr, cv2.COLOR_BGR2RGB)
        h, w = rgb.shape[:2]
        scale = min(self.size / w, self.size / h)
        nw, nh = int(round(w * scale)), int(round(h * scale))
        resized = cv2.resize(rgb, (nw, nh), interpolation=cv2.INTER_LINEAR)
        canvas = np.full((self.size, self.size, 3), 114, dtype=np.uint8)
        ox, oy = (self.size - nw) // 2, (self.size - nh) // 2
        canvas[oy : oy + nh, ox : ox + nw] = resized
        chw = canvas.transpose(2, 0, 1).astype(np.float32) / 255.0
        # DataLoader 会自动补上 batch 维度，返回 [3, H, W] 可避免变成 [1, 1, 3, H, W]。
        return torch.from_numpy(chw)


def main() -> None:
    parser = argparse.ArgumentParser(description="Convert YOLO26 ONNX to ESP-DL model")
    parser.add_argument("--onnx", default="models/yolo26_coke_sprite_raw416_axisfixed.onnx")
    parser.add_argument("--output", default="models/yolo26_coke_sprite.espdl")
    parser.add_argument("--input-size", type=int, default=416)
    parser.add_argument("--calib-limit", type=int, default=96)
    parser.add_argument("--calib-steps", type=int, default=32)
    parser.add_argument("--device", default="cpu", help="cpu or cuda")
    parser.add_argument("--fix-axis-from", default="", help="optional raw ONNX path to fix before conversion")
    args = parser.parse_args()

    onnx_path = Path(args.onnx)
    if args.fix_axis_from:
        fix_negative_axes(Path(args.fix_axis_from), onnx_path)
    if not onnx_path.exists():
        raise SystemExit(f"找不到 ONNX：{onnx_path}")

    roots = [
        Path("data/yolo26_coke_sprite/train/images"),
        Path("data/yolo26_coke_sprite/valid/images"),
        Path("test_assets/phone_gallery/images"),
    ]
    images = iter_images(roots, args.calib_limit)
    if not images:
        raise SystemExit("找不到校准图片，请先准备 data/yolo26_coke_sprite 或 test_assets/phone_gallery")

    import torch
    from esp_ppq.api.espdl_interface import espdl_quantize_onnx

    dataset = ImageCalibDataset(images, args.input_size)
    dataloader = torch.utils.data.DataLoader(dataset, batch_size=1, shuffle=False)

    espdl_quantize_onnx(
        onnx_import_file=str(onnx_path),
        espdl_export_file=str(Path(args.output)),
        calib_dataloader=dataloader,
        calib_steps=min(args.calib_steps, len(dataset)),
        input_shape=[1, 3, args.input_size, args.input_size],
        target="esp32p4",
        num_of_bits=8,
        device=args.device,
        error_report=True,
        export_config=True,
        verbose=1,
    )
    print(f"ESP-DL model written to {args.output}")


if __name__ == "__main__":
    main()
