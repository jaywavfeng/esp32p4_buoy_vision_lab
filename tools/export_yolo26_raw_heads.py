#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""导出 ESP-DL YOLO26 量化所需的 raw-head ONNX。

Ultralytics 默认导出的 YOLO26 ONNX 通常已经包含了解码后的 `output0`，
这种结构更适合 PC 推理，但不适合 ESP-DL 的 `espressif/yolo26` 后处理组件。
ESP-DL 官方量化流程要求检测头直接输出 feature map：

- one2many_p3 / p4 / p5：训练辅助分支，量化后会被裁剪掉；
- one2one_p3 / p4 / p5：部署分支，每个输出是 box 与 class 的 concat。

后续 `quantize_yolo26_official_pipeline.py` 会把 one2one 的 concat 拆成
`one2one_p*_box` 和 `one2one_p*_cls` 六个张量，并导出 `.espdl`。
"""

from __future__ import annotations

import argparse
from pathlib import Path
from types import MethodType

import onnx
import torch


def patch_yolo26_detect_head(model) -> None:
    """把 YOLO26 Detect 头替换成 ESP-DL 官方 raw-head 输出形式。"""

    def esp_forward(self, x):
        one2many = []
        for i in range(self.nl):
            box = self.cv2[i](x[i])
            cls = self.cv3[i](x[i])
            one2many.append(torch.cat((box, cls), 1))

        one2one = []
        for i in range(self.nl):
            box = self.one2one_cv2[i](x[i])
            cls = self.one2one_cv3[i](x[i])
            one2one.append(torch.cat((box, cls), 1))

        return one2many + one2one

    detect = model.model.model[-1]
    if not hasattr(detect, "one2one_cv2") or not hasattr(detect, "one2one_cv3"):
        raise RuntimeError("This checkpoint is not a YOLO26/one2one detection head.")
    detect.forward = MethodType(esp_forward, detect)


def patch_attention_modules(model) -> int:
    """修正 Attention 的静态 shape 导出，保持与 ESP-DL 官方脚本一致。"""
    try:
        from ultralytics.nn.modules import Attention
    except Exception:
        return 0

    def attention_forward(self, x):
        b, c, h, w = x.shape
        n = h * w
        qkv = self.qkv(x)
        q, k, v = qkv.view(-1, self.num_heads, self.key_dim * 2 + self.head_dim, n).split(
            [self.key_dim, self.key_dim, self.head_dim], dim=2
        )
        attn = (q.transpose(-2, -1) @ k) * self.scale
        attn = attn.softmax(dim=-1)
        x = (v @ attn.transpose(-2, -1)).view(-1, c, h, w) + self.pe(v.reshape(-1, c, h, w))
        return self.proj(x)

    patched = 0
    for module in model.modules():
        if isinstance(module, Attention):
            module.forward = MethodType(attention_forward, module)
            patched += 1
    return patched


def main() -> None:
    parser = argparse.ArgumentParser(description="Export YOLO26 raw heads for ESP-DL quantization.")
    parser.add_argument(
        "--weights",
        default="runs/detect/runs/yolo26_coke_sprite/gpu_yolo26n_416/weights/best.pt",
        help="self-trained YOLO26 checkpoint",
    )
    parser.add_argument("--output", default="models/yolo26_coke_sprite_raw_heads_416.onnx")
    parser.add_argument("--imgsz", type=int, default=416)
    parser.add_argument("--opset", type=int, default=13)
    parser.add_argument("--simplify", action="store_true", default=True)
    parser.add_argument("--no-simplify", action="store_false", dest="simplify")
    args = parser.parse_args()

    from ultralytics import YOLO

    weights = Path(args.weights)
    output = Path(args.output)
    if not weights.exists():
        raise SystemExit(f"weights not found: {weights}")

    yolo = YOLO(str(weights))
    model = yolo.model.eval()
    patch_yolo26_detect_head(yolo)
    attn_count = patch_attention_modules(model)

    dummy = torch.zeros(1, 3, args.imgsz, args.imgsz, dtype=torch.float32)
    output_names = [
        "one2many_p3",
        "one2many_p4",
        "one2many_p5",
        "one2one_p3",
        "one2one_p4",
        "one2one_p5",
    ]
    output.parent.mkdir(parents=True, exist_ok=True)

    print(f"Exporting raw-head ONNX: {output}")
    print(f"Input size: {args.imgsz}, opset: {args.opset}, patched Attention modules: {attn_count}")
    torch.onnx.export(
        model,
        dummy,
        str(output),
        verbose=False,
        opset_version=args.opset,
        do_constant_folding=False,
        input_names=["images"],
        output_names=output_names,
        dynamic_axes=None,
        dynamo=False,
    )

    onnx_model = onnx.load(str(output))
    if args.simplify:
        import onnxsim

        print(f"Simplifying ONNX with onnxsim {onnxsim.__version__}...")
        onnx_model, ok = onnxsim.simplify(onnx_model)
        if not ok:
            raise RuntimeError("onnxsim validation failed")
        onnx.save(onnx_model, str(output))

    onnx.checker.check_model(onnx_model)
    for out in onnx_model.graph.output:
        dims = [d.dim_value or d.dim_param for d in out.type.tensor_type.shape.dim]
        print(f"{out.name}: {dims}")


if __name__ == "__main__":
    main()
