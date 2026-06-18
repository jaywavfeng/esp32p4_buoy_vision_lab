#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""按 ESP-DL 官方 YOLO26 教程量化并导出板端 `.espdl` 模型。

这个脚本用于替代早期的 `convert_yolo26_to_espdl.py` 简化流程。简化流程只调用
`espdl_quantize_onnx()`，容易在 YOLO26 上漏掉 LUT、量化对齐和图输出整理步骤，导致
板端 `dl::Model` 加载时崩溃。

本脚本显式执行官方 notebook 的关键步骤：
1. 加载已经导出的 YOLO26 raw-head ONNX；
2. 给 ESP-PPQ 注册 ESP-DL runtime patch 和 LUT exporter；
3. 对图中算子分派 INT8/INT16/FP32 平台；
4. 运行 PTQ、可选 TQT、Passive 参数量化、量化对齐和 LUT 融合；
5. 确保输出是 YOLO26 组件期望的六个 one2one 张量；
6. 使用 ESP-PPQ 的 ESP-DL exporter 写出 `.espdl`。

默认输入 ONNX 是本工程已验证的六输出 raw-head:
`models/yolo26_coke_sprite_o2o_416.onnx`。
"""

from __future__ import annotations

import argparse
import random
import sys
from pathlib import Path
from typing import Iterable

import numpy as np
import torch


ROOT = Path(__file__).resolve().parents[1]
YOLO26_REF = ROOT / "reference_models" / "quantize_yolo26"
YOLO26_SCRIPTS = YOLO26_REF / "scripts"


def add_reference_paths() -> None:
    """把官方 quantize_yolo26 辅助模块加入 Python 搜索路径。"""
    for path in (YOLO26_REF, YOLO26_SCRIPTS):
        text = str(path)
        if text not in sys.path:
            sys.path.insert(0, text)


def iter_images(roots: Iterable[Path], limit: int) -> list[Path]:
    """收集 PTQ/TQT 校准图片，优先使用训练集、验证集和手机图库。"""
    exts = {".jpg", ".jpeg", ".png", ".bmp", ".webp"}
    files: list[Path] = []
    for root in roots:
        if root.exists():
            files.extend(p for p in root.rglob("*") if p.suffix.lower() in exts)
    files = sorted(dict.fromkeys(files))
    return files[:limit]


class LetterboxCalibrationDataset(torch.utils.data.Dataset):
    """ESP-PPQ 校准集，和板端 YOLO26 预处理保持一致。

    板端 `YOLO26::preprocess()` 会把 RGB 图按比例缩放到正方形输入，并用 114 灰边
    letterbox 填充，再通过 LUT 量化写入模型输入。这里返回 0..1 的 NCHW 浮点张量，
    量化器会据此统计激活范围。
    """

    def __init__(self, images: list[Path], size: int) -> None:
        self.images = images
        self.size = size

    def __len__(self) -> int:
        return len(self.images)

    def __getitem__(self, index: int) -> torch.Tensor:
        import cv2
        import numpy as np

        bgr = cv2.imread(str(self.images[index]), cv2.IMREAD_COLOR)
        if bgr is None:
            raise RuntimeError(f"无法读取校准图片: {self.images[index]}")
        rgb = cv2.cvtColor(bgr, cv2.COLOR_BGR2RGB)
        h, w = rgb.shape[:2]
        scale = min(self.size / w, self.size / h)
        nw, nh = int(round(w * scale)), int(round(h * scale))
        resized = cv2.resize(rgb, (nw, nh), interpolation=cv2.INTER_LINEAR)
        canvas = np.full((self.size, self.size, 3), 114, dtype=np.uint8)
        ox, oy = (self.size - nw) // 2, (self.size - nh) // 2
        canvas[oy : oy + nh, ox : ox + nw] = resized
        chw = canvas.transpose(2, 0, 1).astype("float32") / 255.0
        return torch.from_numpy(chw)


def ensure_outputs(graph) -> None:
    """确认导出的图输出满足 YOLO26 组件的六张量约定。"""
    expected = [
        "one2one_p3_box",
        "one2one_p3_cls",
        "one2one_p4_box",
        "one2one_p4_cls",
        "one2one_p5_box",
        "one2one_p5_cls",
    ]
    missing = [name for name in expected if name not in graph.outputs]
    if missing:
        raise RuntimeError(f"ONNX/PPQ 图缺少 YOLO26 输出: {missing}")
    ordered = {name: graph.outputs[name] for name in expected}
    graph.outputs.clear()
    graph.outputs.update(ordered)


def prune_graph_safely(graph):
    """删除已经与最终输出断开的算子和变量。"""
    while True:
        ops_removed = 0
        vars_removed = 0

        dead_ops = []
        for op in list(graph.operations.values()):
            is_output = any(var.name in graph.outputs for var in op.outputs)
            has_consumers = any(len(var.dest_ops) > 0 for var in op.outputs)
            if not is_output and not has_consumers:
                dead_ops.append(op)

        for op in dead_ops:
            for var in list(op.inputs):
                if op in var.dest_ops:
                    var.dest_ops.remove(op)
                if var in op.inputs:
                    op.inputs.remove(var)
            graph.remove_operation(op, keep_coherence=False)
            ops_removed += 1

        dead_vars = []
        for var in list(graph.variables.values()):
            if var.name in graph.inputs or var.name in graph.outputs:
                continue
            if len(var.dest_ops) == 0:
                dead_vars.append(var)

        for var in dead_vars:
            if var.name in graph.variables:
                graph.variables.pop(var.name)
                vars_removed += 1

        if ops_removed == 0 and vars_removed == 0:
            return graph


def get_exclusive_ancestors(graph, target_outputs, excluded_outputs):
    """找出只属于某一组输出的祖先算子，用来区分 one2many 与 one2one 分支。"""

    def collect(output_vars):
        ancestors = set()
        stack = [var.source_op for var in output_vars if var.source_op is not None]
        while stack:
            op = stack.pop()
            if op in ancestors:
                continue
            ancestors.add(op)
            for inp in op.inputs:
                if inp.source_op is not None:
                    stack.append(inp.source_op)
        return ancestors

    target_vars = [graph.outputs[name] for name in target_outputs if name in graph.outputs]
    excluded_vars = [graph.outputs[name] for name in excluded_outputs if name in graph.outputs]
    return collect(target_vars) - collect(excluded_vars)


def split_yolo26_concat_outputs(graph, class_count: int) -> bool:
    """把官方 raw-head concat 输出拆为 ESP-DL C++ 期望的 box/cls 六输出。"""
    output_names = list(graph.outputs.keys())
    targets = ["one2one_p3", "one2one_p4", "one2one_p5"]
    if not all(name in graph.outputs for name in targets):
        return False

    if len(output_names) >= 6:
        for name in output_names[0:3]:
            if name in graph.outputs and name.startswith("one2many"):
                graph.outputs.pop(name)
        prune_graph_safely(graph)

    collected_outputs = {}
    for target_name in targets:
        original_output_var = graph.variables[target_name]
        producer = original_output_var.source_op
        if producer is None or producer.type != "Concat":
            continue

        box_var = None
        cls_var = None
        for input_var in producer.inputs:
            dims = input_var.shape
            if dims is None or len(dims) < 2:
                continue
            if dims[1] == 4:
                box_var = input_var
            elif dims[1] == class_count:
                cls_var = input_var

        if box_var is None or cls_var is None:
            raise RuntimeError(f"cannot split {target_name}: box={box_var}, cls={cls_var}")

        for var, new_name in ((box_var, f"{target_name}_box"), (cls_var, f"{target_name}_cls")):
            old_name = var.name
            if old_name in graph.variables:
                graph.variables.pop(old_name)
            var._name = new_name
            graph.variables[new_name] = var
            collected_outputs[new_name] = var

        graph.outputs.pop(target_name)
        graph.remove_operation(producer, keep_coherence=False)
        for var in producer.inputs:
            if producer in var.dest_ops:
                var.dest_ops.remove(producer)

    final_output_list = [
        "one2one_p3_box",
        "one2one_p3_cls",
        "one2one_p4_box",
        "one2one_p4_cls",
        "one2one_p5_box",
        "one2one_p5_cls",
    ]
    graph.outputs.clear()
    for name in final_output_list:
        if name in collected_outputs:
            graph.outputs[name] = collected_outputs[name]

    prune_graph_safely(graph)
    ensure_outputs(graph)
    return True


def repair_vector_parameter_shapes(graph) -> int:
    """修复 ESP-PPQ 导出的标量/伪标量参数形状。

    在 YOLO26 图中，注意力缩放常量和 `RequantizeLinear` scale/zero-point 都可能以
    参数输入存在。当前 ESP-PPQ 版本偶尔把这些参数的 `shape` 留成 `[]`：有时 value
    是真正的单值标量，有时实际是长度为 4、16 等的一维数组。ESP-DL runtime 在加载
    operation parameter 时会按 shape 创建张量，空 shape 容易让
    `FbsModel::get_operation_parameter()` 访问空指针并崩溃。

    因此导出前统一检查所有参数变量：只要 `shape` 为空且 value 存在，就把 shape 改成
    `[1]` 或真实一维长度。这不改变数值，只补齐 FlatBuffer 需要的维度元数据。
    """
    repaired = 0
    for var in graph.variables.values():
        value = getattr(var, "value", None)
        shape = getattr(var, "shape", None)
        if value is None:
            continue
        try:
            value_shape = list(value.shape)
            if hasattr(value, "numel"):
                value_size = int(value.numel())
            else:
                value_size = int(value.size)
        except AttributeError:
            scalar = np.asarray(value)
            value_shape = list(scalar.shape)
            value_size = int(scalar.size)
            value = scalar
            var.value = scalar
        if (shape == [] or shape is None) and value_size >= 1:
            fixed_shape = value_shape if value_shape else [1]
            try:
                if hasattr(value, "reshape"):
                    var.value = value.reshape(fixed_shape)
            except Exception:
                pass
            var.shape = fixed_shape
            repaired += 1
    if repaired:
        print(f"Repaired vector parameter shapes: {repaired}")
    return repaired


def patch_espdl_scalar_exporter() -> None:
    """在 ESP-DL 写 FlatBuffer 前修复 0 维参数。

    ESP-PPQ 的 ESP-DL exporter 会在导出阶段额外插入 QuantizeLinear /
    DequantizeLinear / RequantizeLinear 节点。某些 scale 或 zero-point 是
    PyTorch 0 维张量，原始导出结果会被写成 shape=[]。ESP32-P4 端的
    libfbs_model 在读取 operation parameter 时对这种标量形状不够健壮，
    会在 dl::Model 加载阶段触发 Load access fault。

    这里不改变量化数值，只把单值参数在序列化前 reshape 为 [1]，让板端
    loader 能按普通一维参数读取。补丁放在项目脚本中，避免手工修改
    site-packages 后难以复现。
    """
    try:
        from esp_ppq.parser.espdl_exporter import EspdlExporter
    except Exception as exc:
        print(f"Skip ESP-DL scalar exporter patch: {exc}")
        return

    if getattr(EspdlExporter, "_buoy_scalar_shape_patch", False):
        return

    original_build_variable_proto = EspdlExporter.build_variable_proto

    def patched_build_variable_proto(self, variable, exponent, layout, perm=None):
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

        return original_build_variable_proto(self, variable, exponent, layout, perm)

    EspdlExporter.build_variable_proto = patched_build_variable_proto
    EspdlExporter._buoy_scalar_shape_patch = True
    print("Patched ESP-DL exporter scalar parameter shape handling.")


def seed_everything(seed: int) -> None:
    """固定随机种子，保证校准/TQT 过程尽量可复现。"""
    random.seed(seed)
    np.random.seed(seed)
    torch.manual_seed(seed)
    if torch.cuda.is_available():
        torch.cuda.manual_seed_all(seed)


def register_mod_op() -> None:
    """给 ESP-PPQ 注册 ONNX `Mod` 的简单实现。

    某些 Ultralytics 导出图会出现 `Mod`，基础 ESP-PPQ 版本不一定默认实现。
    官方 notebook 也有同名 patch，这里内联一份，避免依赖 notebook 的 `config` 模块。
    """
    try:
        from esp_ppq.core import TargetPlatform
        from esp_ppq.executor import register_operation_handler

        def mod_impl(op, values, ctx):
            return values[0] % values[1]

        for platform in (TargetPlatform.SOI, TargetPlatform.FP32):
            try:
                register_operation_handler(mod_impl, "Mod", platform)
            except Exception:
                pass
        print("Registered PPQ Mod handler.")
    except Exception as exc:
        print(f"Skip Mod handler patch: {exc}")


def build_dispatch_table(
    graph,
    quantizer,
    target_platform,
    int16_platform,
    aux_ops=None,
    main_ops=None,
    enable_int16: bool = True,
):
    """按官方教程给关键 YOLO26 出口层提高精度。

    名称集合来自 ESP-DL 官方 quantize_yolo26 notebook。若自训练模型的算子名和官方
    名称一致，就把这些层放到 INT16；不存在的名称会自然跳过。
    """
    import esp_ppq.lib as PFL
    from esp_ppq.core import TargetPlatform

    dispatching_table = PFL.Dispatcher(graph=graph, method="conservative").dispatch(
        quantizer.quant_operation_types
    )
    for opname, platform in list(dispatching_table.items()):
        if platform == TargetPlatform.UNSPECIFIED:
            dispatching_table[opname] = TargetPlatform(target_platform)

    int16_layers = {
        "/model.16/cv2/conv/Conv",
        "/model.16/cv2/conv/Conv/Swish",
        "/model.19/cv2/conv/Conv",
        "/model.19/cv2/conv/Conv/Swish",
        "/model.22/cv2/conv/Conv",
        "/model.22/cv2/conv/Conv/Swish",
        "/model.23/one2one_cv2.0/one2one_cv2.0.0/conv/Conv",
        "/model.23/one2one_cv2.0/one2one_cv2.0.0/conv/Conv/Swish",
        "/model.23/one2one_cv2.0/one2one_cv2.0.1/conv/Conv",
        "/model.23/one2one_cv2.0/one2one_cv2.0.1/conv/Conv/Swish",
        "/model.23/one2one_cv2.0/one2one_cv2.0.2/Conv",
        "/model.23/one2one_cv2.1/one2one_cv2.1.0/conv/Conv",
        "/model.23/one2one_cv2.1/one2one_cv2.1.0/conv/Conv/Swish",
        "/model.23/one2one_cv2.1/one2one_cv2.1.1/conv/Conv",
        "/model.23/one2one_cv2.1/one2one_cv2.1.1/conv/Conv/Swish",
        "/model.23/one2one_cv2.1/one2one_cv2.1.2/Conv",
        "/model.23/one2one_cv2.2/one2one_cv2.2.0/conv/Conv",
        "/model.23/one2one_cv2.2/one2one_cv2.2.0/conv/Conv/Swish",
        "/model.23/one2one_cv2.2/one2one_cv2.2.1/conv/Conv",
        "/model.23/one2one_cv2.2/one2one_cv2.2.1/conv/Conv/Swish",
        "/model.23/one2one_cv2.2/one2one_cv2.2.2/Conv",
        "/model.23/one2one_cv3.0/one2one_cv3.0.0/one2one_cv3.0.0.0/conv/Conv",
        "/model.23/one2one_cv3.0/one2one_cv3.0.0/one2one_cv3.0.0.0/conv/Conv/Swish",
        "/model.23/one2one_cv3.0/one2one_cv3.0.0/one2one_cv3.0.0.1/conv/Conv",
        "/model.23/one2one_cv3.0/one2one_cv3.0.0/one2one_cv3.0.0.1/conv/Conv/Swish",
        "/model.23/one2one_cv3.0/one2one_cv3.0.1/one2one_cv3.0.1.0/conv/Conv",
        "/model.23/one2one_cv3.0/one2one_cv3.0.1/one2one_cv3.0.1.0/conv/Conv/Swish",
        "/model.23/one2one_cv3.0/one2one_cv3.0.1/one2one_cv3.0.1.1/conv/Conv",
        "/model.23/one2one_cv3.0/one2one_cv3.0.1/one2one_cv3.0.1.1/conv/Conv/Swish",
        "/model.23/one2one_cv3.0/one2one_cv3.0.2/Conv",
        "/model.23/one2one_cv3.1/one2one_cv3.1.0/one2one_cv3.1.0.0/conv/Conv",
        "/model.23/one2one_cv3.1/one2one_cv3.1.0/one2one_cv3.1.0.0/conv/Conv/Swish",
        "/model.23/one2one_cv3.1/one2one_cv3.1.0/one2one_cv3.1.0.1/conv/Conv",
        "/model.23/one2one_cv3.1/one2one_cv3.1.0/one2one_cv3.1.0.1/conv/Conv/Swish",
        "/model.23/one2one_cv3.1/one2one_cv3.1.1/one2one_cv3.1.1.0/conv/Conv",
        "/model.23/one2one_cv3.1/one2one_cv3.1.1/one2one_cv3.1.1.0/conv/Conv/Swish",
        "/model.23/one2one_cv3.1/one2one_cv3.1.1/one2one_cv3.1.1.1/conv/Conv",
        "/model.23/one2one_cv3.1/one2one_cv3.1.1/one2one_cv3.1.1.1/conv/Conv/Swish",
        "/model.23/one2one_cv3.1/one2one_cv3.1.2/Conv",
        "/model.23/one2one_cv3.2/one2one_cv3.2.0/one2one_cv3.2.0.0/conv/Conv",
        "/model.23/one2one_cv3.2/one2one_cv3.2.0/one2one_cv3.2.0.0/conv/Conv/Swish",
        "/model.23/one2one_cv3.2/one2one_cv3.2.0/one2one_cv3.2.0.1/conv/Conv",
        "/model.23/one2one_cv3.2/one2one_cv3.2.0/one2one_cv3.2.0.1/conv/Conv/Swish",
        "/model.23/one2one_cv3.2/one2one_cv3.2.1/one2one_cv3.2.1.0/conv/Conv",
        "/model.23/one2one_cv3.2/one2one_cv3.2.1/one2one_cv3.2.1.0/conv/Conv/Swish",
        "/model.23/one2one_cv3.2/one2one_cv3.2.1/one2one_cv3.2.1.1/conv/Conv",
        "/model.23/one2one_cv3.2/one2one_cv3.2.1/one2one_cv3.2.1.1/conv/Conv/Swish",
        "/model.23/one2one_cv3.2/one2one_cv3.2.2/Conv",
    }
    matched = 0
    if enable_int16:
        for op in graph.operations.values():
            if op.name in dispatching_table and op.name in int16_layers:
                dispatching_table[op.name] = int16_platform
                matched += 1

    for op in aux_ops or []:
        if op.name in dispatching_table:
            dispatching_table[op.name] = TargetPlatform.FP32

    # 官方 notebook 建议把部署分支末端 Concat 保持 FP32，规避部分量化对齐限制。
    fp32_layers = {"/model.23/Concat_3", "/model.23/Concat_4", "/model.23/Concat_5"}
    for op in main_ops or []:
        if op.name in fp32_layers and op.name in dispatching_table:
            dispatching_table[op.name] = TargetPlatform.FP32

    print(f"INT16 precision layers matched: {matched} (enabled={enable_int16})")
    return dispatching_table


def main() -> None:
    parser = argparse.ArgumentParser(description="Official ESP-DL YOLO26 quantization pipeline")
    parser.add_argument("--onnx", default="models/yolo26_coke_sprite_raw_heads_416.onnx")
    parser.add_argument("--output", default="models/yolo26_coke_sprite_o2o_416_official_s8_p4.espdl")
    parser.add_argument("--input-size", type=int, default=416)
    parser.add_argument("--classes", type=int, default=2)
    parser.add_argument("--calib-limit", type=int, default=256)
    parser.add_argument("--calib-steps", type=int, default=32)
    parser.add_argument("--tqt-steps", type=int, default=40)
    parser.add_argument("--batch-size", type=int, default=1)
    parser.add_argument("--device", default="cuda" if torch.cuda.is_available() else "cpu")
    parser.add_argument("--lut-step", type=int, default=32)
    parser.add_argument("--calib-method", choices=["percentile", "kl", "minmax"], default="percentile")
    parser.add_argument(
        "--no-int16-head",
        action="store_true",
        help="不把 YOLO26 检测头强制分派到 INT16，用于生成最保守的全 INT8 兼容模型。",
    )
    parser.add_argument(
        "--no-alignment",
        action="store_true",
        help="跳过 QuantAlignmentPass。用于排查 RequantizeLinear 加载崩溃问题。",
    )
    args = parser.parse_args()

    add_reference_paths()

    from esp_ppq.api.interface import load_onnx_graph
    from esp_ppq.api.espdl_interface import get_target_platform
    from esp_ppq.core import QuantizationVisibility
    from esp_ppq.executor import TorchExecutor
    from esp_ppq.quantization.optim import (
        ParameterQuantizePass,
        PassiveParameterQuantizePass,
        QuantAlignmentPass,
        QuantizeFusionPass,
        QuantizeSimplifyPass,
        RuntimeCalibrationPass,
        TrainedQuantizationThresholdPass,
    )
    import esp_ppq.lib as PFL

    from esp_ppq_patch import apply_esp_ppq_patches
    from esp_ppq_patch_2 import apply_addlut_patch
    from esp_ppq_lut.passes import EspdlLUTFusionPass

    import esp_ppq_lut as esp_lut

    seed_everything(1234)
    register_mod_op()
    apply_esp_ppq_patches()
    apply_addlut_patch()
    patch_espdl_scalar_exporter()
    esp_lut.initialize(step=args.lut_step, verbose=True)

    onnx_path = ROOT / args.onnx
    output_path = ROOT / args.output
    if not onnx_path.exists():
        raise SystemExit(f"找不到 ONNX: {onnx_path}")

    roots = [
        ROOT / "data" / "yolo26_coke_sprite" / "train" / "images",
        ROOT / "data" / "yolo26_coke_sprite" / "valid" / "images",
        ROOT / "test_assets" / "phone_gallery" / "images",
    ]
    images = iter_images(roots, args.calib_limit)
    if not images:
        raise SystemExit("找不到校准图片，请先准备 data/yolo26_coke_sprite 或 test_assets/phone_gallery。")
    dataset = LetterboxCalibrationDataset(images, args.input_size)
    dataloader = torch.utils.data.DataLoader(dataset, batch_size=args.batch_size, shuffle=False)
    calib_steps = min(args.calib_steps, len(dataloader))
    print(f"Calibration images={len(dataset)}, steps={calib_steps}, device={args.device}")

    target_platform = get_target_platform("esp32p4", 8)
    int16_platform = get_target_platform("esp32p4", 16)

    print("Loading ONNX graph...")
    graph = load_onnx_graph(onnx_import_file=str(onnx_path))
    output_names = list(graph.outputs.keys())
    aux_ops = set()
    main_ops = set()
    if len(output_names) >= 6 and all(name.startswith("one2many") for name in output_names[0:3]):
        aux_outputs = output_names[0:3]
        main_outputs = output_names[3:6]
        aux_ops = get_exclusive_ancestors(graph, aux_outputs, main_outputs)
        main_ops = get_exclusive_ancestors(graph, main_outputs, aux_outputs)
        print(f"Detected official raw-head graph: aux_ops={len(aux_ops)}, main_ops={len(main_ops)}")
    else:
        ensure_outputs(graph)

    quantizer = PFL.Quantizer(platform=target_platform, graph=graph)
    dispatching_table = build_dispatch_table(
        graph,
        quantizer,
        target_platform,
        int16_platform,
        aux_ops,
        main_ops,
        enable_int16=not args.no_int16_head,
    )
    print("Applying quantizer dispatch...")
    for op in graph.operations.values():
        quantizer.quantize_operation(op_name=op.name, platform=dispatching_table[op.name])

    executor = TorchExecutor(graph=graph, device=args.device)
    dummy_input = torch.zeros([1, 3, args.input_size, args.input_size], device=args.device)
    executor.tracing_operation_meta(inputs=dummy_input)

    passes = [
        QuantizeSimplifyPass(),
        QuantizeFusionPass(activation_type=quantizer.activation_fusion_types),
        ParameterQuantizePass(),
        RuntimeCalibrationPass(method=args.calib_method),
    ]
    if args.tqt_steps > 0:
        passes.append(
            TrainedQuantizationThresholdPass(
                steps=args.tqt_steps,
                lr=1e-5,
                int_lambda=0.25,
                block_size=4,
                collecting_device="cpu",
            )
        )
    passes.extend(
        [
            PassiveParameterQuantizePass(clip_visiblity=QuantizationVisibility.EXPORT_WHEN_ACTIVE),
        ]
    )
    if not args.no_alignment:
        passes.append(QuantAlignmentPass(elementwise_alignment="Align to Output"))
    passes.append(EspdlLUTFusionPass(target_ops=["Swish"], lut_step=args.lut_step))

    print("Running PTQ/TQT/LUT pipeline...")
    pipeline = PFL.Pipeline(passes)
    pipeline.optimize(
        calib_steps=calib_steps,
        collate_fn=lambda x: x.type(torch.float32).to(args.device),
        graph=graph,
        dataloader=dataloader,
        executor=executor,
    )

    if not split_yolo26_concat_outputs(graph, args.classes):
        ensure_outputs(graph)
    repair_vector_parameter_shapes(graph)
    output_path.parent.mkdir(parents=True, exist_ok=True)
    print(f"Exporting ESP-DL model: {output_path}")
    exporter = PFL.Exporter(platform=target_platform)
    exporter.export(str(output_path), graph=graph, int16_lut_step=args.lut_step, export_config=True)
    print("Done.")


if __name__ == "__main__":
    main()
