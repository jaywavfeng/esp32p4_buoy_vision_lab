#!/usr/bin/env python3
"""训练可乐/雪碧 YOLO26n 检测模型，并导出 PC 端验证报告。

这个脚本只负责电脑端训练和 ONNX 导出，不直接生成 ESP-DL 模型。完整路线是：

1. `prepare_yolo26_dataset.py` 把公开数据和板端补充样本整理为 YOLO 数据集。
2. 本脚本用 Ultralytics 训练 `yolo26n.pt`，输入固定 416x416。
3. 导出 raw ONNX，后续由 `convert_yolo26_to_espdl.py` 量化成
   `models/yolo26_coke_sprite.espdl`。

默认 `--device auto` 会优先使用 CUDA；如果本机没有可用 GPU，会自动退回 CPU。
"""

from __future__ import annotations

import argparse
import json
import subprocess
import sys
from datetime import datetime
from pathlib import Path
from typing import Any


def run_prepare_dataset(args: argparse.Namespace) -> None:
    """可选地先生成 YOLO 数据目录，方便从零复现实验。"""
    script = Path(__file__).with_name("prepare_yolo26_dataset.py")
    cmd = [
        sys.executable,
        str(script),
        "--source",
        args.source,
        "--board-samples",
        args.board_samples,
        "--output",
        args.dataset,
        "--unknown-limit",
        str(args.unknown_limit),
    ]
    if args.clean:
        cmd.append("--clean")
    subprocess.run(cmd, check=True)


def load_ultralytics():
    try:
        from ultralytics import YOLO  # type: ignore
    except ImportError as exc:
        raise SystemExit(
            "缺少 ultralytics。请先安装依赖，例如：\n"
            "python -m pip install ultralytics onnx onnxruntime opencv-python pillow\n"
            "如果官方源慢，可以加：-i https://pypi.tuna.tsinghua.edu.cn/simple"
        ) from exc
    return YOLO


def pick_device(requested: str) -> tuple[str, dict[str, Any]]:
    """根据本机环境选择训练设备，并把结果写入报告。"""
    info: dict[str, Any] = {"requested": requested}
    if requested != "auto":
        info["selected"] = requested
        return requested, info

    try:
        import torch

        cuda = bool(torch.cuda.is_available())
        info["cuda_available"] = cuda
        if cuda:
            info["cuda_name"] = torch.cuda.get_device_name(0)
            info["selected"] = "0"
            return "0", info
    except Exception as exc:  # pragma: no cover - 仅用于环境报告
        info["torch_error"] = repr(exc)

    info["selected"] = "cpu"
    return "cpu", info


def metric_value(metrics: Any, *names: str) -> float | None:
    for name in names:
        value = getattr(metrics.box, name, None)
        if value is not None:
            try:
                return float(value)
            except TypeError:
                pass
    return None


def export_raw_onnx(model: Any, args: argparse.Namespace) -> str | None:
    """导出 ESP-DL 转换更容易处理的 raw ONNX。

    注意：YOLO26 板端后处理由 `espressif/yolo26` 组件完成，因此导出时需要保留
    原始检测头输出，避免把 NMS/后处理节点塞进 ONNX。
    """
    if not args.export_onnx:
        return None

    try:
        exported = model.export(
            format="onnx",
            imgsz=args.imgsz,
            opset=args.opset,
            simplify=args.simplify,
            dynamic=False,
            end2end=False,
        )
        return str(exported)
    except TypeError:
        exported = model.export(
            format="onnx",
            imgsz=args.imgsz,
            opset=args.opset,
            simplify=args.simplify,
            dynamic=False,
        )
        return str(exported)


def main() -> None:
    parser = argparse.ArgumentParser(description="Train YOLO26n Coke/Sprite detector for ESP32-P4")
    parser.add_argument("--dataset", default="data/yolo26_coke_sprite", help="prepared YOLO dataset root")
    parser.add_argument("--source", default="data/public_soda_bottles", help="source soda-bottles dataset")
    parser.add_argument("--board-samples", default="data/board_samples", help="board captured samples")
    parser.add_argument("--unknown-limit", type=int, default=80, help="empty-label hard negatives to copy")
    parser.add_argument("--prepare", action="store_true", help="prepare dataset before training")
    parser.add_argument("--clean", action="store_true", help="clean prepared dataset when --prepare is used")
    parser.add_argument("--model", default="yolo26n.pt", help="pretrained checkpoint")
    parser.add_argument("--epochs", type=int, default=40)
    parser.add_argument("--imgsz", type=int, default=416)
    parser.add_argument("--batch", type=int, default=8)
    parser.add_argument("--device", default="auto", help="auto, cpu, or GPU id such as 0")
    parser.add_argument("--project", default="runs/yolo26_coke_sprite")
    parser.add_argument("--name", default="gpu_yolo26n_416")
    parser.add_argument("--report", default="reports/yolo26_coke_sprite_pc_report.json")
    parser.add_argument("--export-onnx", action="store_true", default=True)
    parser.add_argument("--no-export-onnx", action="store_false", dest="export_onnx")
    parser.add_argument("--opset", type=int, default=17)
    parser.add_argument("--simplify", action="store_true", default=True)
    parser.add_argument("--no-simplify", action="store_false", dest="simplify")
    parser.add_argument("--dry-run", action="store_true", help="check paths and write a plan without training")
    args = parser.parse_args()

    if args.prepare:
        run_prepare_dataset(args)

    dataset_root = Path(args.dataset)
    data_yaml = dataset_root / "data.yaml"
    if not data_yaml.exists():
        raise SystemExit(
            f"找不到 {data_yaml}。请先运行：\n"
            f"python tools\\prepare_yolo26_dataset.py --output {args.dataset} --clean"
        )

    report_path = Path(args.report)
    report_path.parent.mkdir(parents=True, exist_ok=True)
    device, device_info = pick_device(args.device)

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

    YOLO = load_ultralytics()
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
    exported_onnx = export_raw_onnx(trained_model, args)

    report.update(
        {
            "status": "ok",
            "save_dir": str(save_dir),
            "best_pt": str(weights),
            "onnx": exported_onnx,
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
