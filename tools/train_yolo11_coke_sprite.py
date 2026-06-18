#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""训练可乐/雪碧 YOLO11n，并导出 ESP-DL 友好的 six-output ONNX。

这个脚本和普通 Ultralytics 导出最大的区别在于：默认 YOLO11 ONNX 会把
box 解码、concat 等后处理一起放进图里，不适合 ESP32-P4 上的定点量化。
乐鑫 ESP-DL 官方 YOLO11 教程建议重载 Detect.forward，只导出三层检测头的
原始 box/score：

    box0, score0, box1, score1, box2, score2

板端再用 `dl::detect::yolo11PostProcessor` 做 DFL 解码、sigmoid、NMS 和
letterbox 坐标映射。这样模型更小、量化误差更容易控制，也和官方
`coco_detect` 组件的运行方式一致。
"""

from __future__ import annotations

import argparse
import json
import shutil
from datetime import datetime
from pathlib import Path
from typing import Any

import torch


ROOT = Path(__file__).resolve().parents[1]


def load_ultralytics():
    try:
        from ultralytics import YOLO  # type: ignore
        from ultralytics.engine.exporter import Exporter, arange_patch, try_export
        from ultralytics.nn.modules import Attention, Detect
        from ultralytics.utils import LOGGER, colorstr
        from ultralytics.utils.checks import check_requirements
    except ImportError as exc:
        raise SystemExit(
            "缺少 ultralytics/torch 训练依赖。请先运行：\n"
            "python -m pip install ultralytics torch torchvision onnx onnxsim onnxruntime opencv-python pillow\n"
            "国内源可用：-i https://pypi.tuna.tsinghua.edu.cn/simple"
        ) from exc
    def get_latest_opset() -> int:
        return 13

    return YOLO, Exporter, arange_patch, try_export, Attention, Detect, LOGGER, colorstr, check_requirements, get_latest_opset


def pick_device(requested: str) -> tuple[str, dict[str, Any]]:
    """选择训练设备，并把信息写入报告，方便后续复现实验。"""
    info: dict[str, Any] = {"requested": requested, "torch": torch.__version__}
    if requested != "auto":
        info["selected"] = requested
        return requested, info
    cuda = bool(torch.cuda.is_available())
    info["cuda_available"] = cuda
    if cuda:
        info["cuda_name"] = torch.cuda.get_device_name(0)
        info["selected"] = "0"
        return "0", info
    info["selected"] = "cpu"
    return "cpu", info


def metric_value(metrics: Any, *names: str) -> float | None:
    """兼容不同 Ultralytics 版本的指标字段名。"""
    for name in names:
        value = getattr(metrics.box, name, None)
        if value is not None:
            try:
                return float(value)
            except TypeError:
                pass
    return None


def patch_yolo11_for_esp_export(model: Any, modules: tuple[Any, Any]) -> None:
    """按 ESP-DL 官方 export_onnx.py 的方式重载 YOLO11 检测头。

    `Detect.forward()` 输出三层检测头的原始 bbox 分布和类别分数；
    `Attention.forward()` 保持官方脚本中的写法，避免部分 Ultralytics 版本
    导出注意力模块时产生 ESP-PPQ 不易处理的图结构。
    """
    Attention, Detect = modules

    def esp_detect_forward(self, x):
        box0 = self.cv2[0](x[0])
        score0 = self.cv3[0](x[0])
        box1 = self.cv2[1](x[1])
        score1 = self.cv3[1](x[1])
        box2 = self.cv2[2](x[2])
        score2 = self.cv3[2](x[2])
        return box0, score0, box1, score1, box2, score2

    def esp_attention_forward(self, x):
        batch, channels, height, width = x.shape
        tokens = height * width
        qkv = self.qkv(x)
        q, k, v = qkv.view(-1, self.num_heads, self.key_dim * 2 + self.head_dim, tokens).split(
            [self.key_dim, self.key_dim, self.head_dim], dim=2
        )
        attn = (q.transpose(-2, -1) @ k) * self.scale
        attn = attn.softmax(dim=-1)
        x = (v @ attn.transpose(-2, -1)).view(-1, channels, height, width) + self.pe(
            v.reshape(-1, channels, height, width)
        )
        return self.proj(x)

    for module in model.modules():
        if isinstance(module, Attention):
            module.forward = esp_attention_forward.__get__(module)
        if isinstance(module, Detect):
            module.forward = esp_detect_forward.__get__(module)


def export_esp_onnx(trained_model: Any, output: Path, imgsz: int, opset: int, simplify: bool) -> Path:
    """导出 six-output ONNX，并复制到 models/ 下固定文件名。"""
    (
        _YOLO,
        _Exporter,
        _arange_patch,
        _try_export,
        Attention,
        Detect,
        _LOGGER,
        _colorstr,
        _check_requirements,
        _get_latest_opset,
    ) = load_ultralytics()

    patch_yolo11_for_esp_export(trained_model.model, (Attention, Detect))

    import onnx

    output.parent.mkdir(parents=True, exist_ok=True)
    output_names = ["box0", "score0", "box1", "score1", "box2", "score2"]
    model = trained_model.model.cpu().eval()
    dummy = torch.zeros(1, 3, imgsz, imgsz, dtype=torch.float32)
    print(f"Exporting ESP-DL YOLO11 ONNX: {output}, opset={opset}, imgsz={imgsz}")
    torch.onnx.export(
        model,
        dummy,
        str(output),
        verbose=False,
        opset_version=opset,
        do_constant_folding=False,
        input_names=["images"],
        output_names=output_names,
        dynamic_axes=None,
        dynamo=False,
    )
    model_onnx = onnx.load(str(output))
    if simplify:
        import onnxsim

        model_onnx, ok = onnxsim.simplify(model_onnx)
        if not ok:
            raise RuntimeError("ONNX simplify check failed")
    for index, expected in enumerate(output_names):
        actual = model_onnx.graph.output[index].name
        if actual != expected:
            raise RuntimeError(f"ONNX output[{index}]={actual}, expected {expected}")
    onnx.save(model_onnx, str(output))
    return output


def main() -> None:
    parser = argparse.ArgumentParser(description="Train YOLO11n Coke/Sprite detector for ESP32-P4")
    parser.add_argument("--dataset", default="data/yolo26_coke_sprite")
    parser.add_argument("--model", default="yolo11n.pt")
    parser.add_argument("--weights", default="", help="已有训练权重；配合 --export-only 可跳过训练")
    parser.add_argument("--epochs", type=int, default=40)
    parser.add_argument("--imgsz", type=int, default=416)
    parser.add_argument("--batch", type=int, default=8)
    parser.add_argument("--device", default="auto")
    parser.add_argument("--project", default="runs/yolo11_coke_sprite")
    parser.add_argument("--name", default="gpu_yolo11n_416")
    parser.add_argument("--report", default="reports/yolo11_coke_sprite_pc_report.json")
    parser.add_argument("--output-onnx", default="models/yolo11_coke_sprite_416.onnx")
    parser.add_argument("--opset", type=int, default=13)
    parser.add_argument("--simplify", action="store_true", default=True)
    parser.add_argument("--no-simplify", action="store_false", dest="simplify")
    parser.add_argument("--dry-run", action="store_true")
    parser.add_argument("--export-only", action="store_true", help="只从 --weights 导出 ESP-DL six-output ONNX")
    args = parser.parse_args()

    data_yaml = ROOT / args.dataset / "data.yaml"
    if not data_yaml.exists():
        raise SystemExit(f"找不到数据集配置: {data_yaml}")

    device, device_info = pick_device(args.device)
    report_path = ROOT / args.report
    report_path.parent.mkdir(parents=True, exist_ok=True)
    report: dict[str, Any] = {
        "time": datetime.now().isoformat(timespec="seconds"),
        "dataset": str(data_yaml),
        "model": args.model,
        "imgsz": args.imgsz,
        "epochs": args.epochs,
        "batch": args.batch,
        "device": device_info,
        "status": "dry-run" if args.dry_run else "training",
    }
    if args.dry_run:
        report_path.write_text(json.dumps(report, ensure_ascii=False, indent=2), encoding="utf-8")
        print(json.dumps(report, ensure_ascii=False, indent=2))
        return

    YOLO, *_ = load_ultralytics()
    if args.export_only:
        if not args.weights:
            raise SystemExit("--export-only 需要同时指定 --weights")
        weights = Path(args.weights)
        if not weights.exists():
            raise SystemExit(f"找不到权重: {weights}")
        save_dir = weights.parents[1]
        trained_model = YOLO(str(weights))
        metrics = trained_model.val(data=str(data_yaml), imgsz=args.imgsz, device=device)
    else:
        model = YOLO(args.model)
        train_result = model.train(
            data=str(data_yaml),
            imgsz=args.imgsz,
            epochs=args.epochs,
            batch=args.batch,
            device=device,
            project=args.project,
            name=args.name,
            exist_ok=True,
        )
        save_dir = Path(getattr(train_result, "save_dir", Path(args.project) / args.name))
        weights = save_dir / "weights" / "best.pt"
        trained_model = YOLO(str(weights)) if weights.exists() else model
        metrics = trained_model.val(data=str(data_yaml), imgsz=args.imgsz, device=device)
    onnx_path = export_esp_onnx(trained_model, ROOT / args.output_onnx, args.imgsz, args.opset, args.simplify)

    report.update(
        {
            "status": "ok",
            "save_dir": str(save_dir),
            "best_pt": str(weights),
            "onnx": str(onnx_path),
            "metrics": {
                "map50": metric_value(metrics, "map50"),
                "map50_95": metric_value(metrics, "map"),
                "precision": metric_value(metrics, "mp", "p"),
                "recall": metric_value(metrics, "mr", "r"),
            },
        }
    )
    report_path.write_text(json.dumps(report, ensure_ascii=False, indent=2), encoding="utf-8")
    print(json.dumps(report, ensure_ascii=False, indent=2))


if __name__ == "__main__":
    main()
