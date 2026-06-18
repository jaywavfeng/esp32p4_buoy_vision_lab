# 板端 AI 模型参考说明

本工程当前保留四种识别状态，用于学习、调试和对比：

```text
off
mlp
yolo11
yolo26
```

`off` 用于测纯图传和无线延时；`mlp` 是低延时 baseline；`yolo11` 和 `yolo26` 是当前已经完成训练、量化、嵌入和板端验证的 Coke/Sprite YOLO 模型。

## 1. MLP baseline

入口：

```text
main/coke_sprite_mlp_model.h
main/camera_web_main.c
```

特点：

- 输入为 `16x16 RGB` 网格特征。
- 单隐藏层 MLP，类别为 `unknown/coke/sprite`。
- 推理开销小，适合默认低功耗和无线链路稳定性验证。
- 误检控制主要依赖 `box_min_score` 和训练样本覆盖度。

## 2. YOLO11n Coke/Sprite

入口：

```text
models/yolo11_coke_sprite_416_s8_p4.espdl
main/yolo11_espdl_bridge.cpp
tools/train_yolo11_coke_sprite.py
tools/quantize_yolo11_espdl.py
```

路线：

1. 使用 Ultralytics `yolo11n.pt` 训练 `coke/sprite` 两类。
2. 按 ESP-DL YOLO11 教程导出六个 raw head：`box0/score0/box1/score1/box2/score2`。
3. 使用 `espdl_quantize_onnx` 量化为 ESP32-P4 INT8 `.espdl`。
4. 板端用 `dl::detect::yolo11PostProcessor` 完成 DFL 解码、NMS 和坐标映射。

PC 指标：

```text
mAP50=0.9547  mAP50-95=0.5468  Precision=0.9435  Recall=0.9312
```

板端验证：

```text
model=coke-sprite-yolo11n-416-p4
model_bytes=2977040
inference_ms≈15479
```

## 3. YOLO26n Coke/Sprite

入口：

```text
models/yolo26_coke_sprite_raw_heads_416_allint8_p4.espdl
main/yolo26_espdl_bridge.cpp
tools/export_yolo26_raw_heads.py
tools/quantize_yolo26_official_pipeline.py
```

路线：

1. 使用 Ultralytics `yolo26n.pt` 训练 `coke/sprite` 两类。
2. 导出 `espressif/yolo26` 后处理器需要的 raw one2one heads。
3. 使用 ESP-PPQ/ESP-DL 量化为 ESP32-P4 INT8 `.espdl`。
4. 板端用 `YOLO26` 组件完成后处理，类别表为 `coke/sprite`。

PC 指标：

```text
mAP50=0.9419  mAP50-95=0.5375  Precision=0.9044  Recall=0.9093
```

板端验证：

```text
model=coke-sprite-yolo26n-416-p4
model_bytes=2815168
inference_ms≈14302
```

## 4. 技术取舍

- 当前轻量化方式是 `nano` 网络 + 固定 `416x416` 输入 + INT8 量化。
- 没有额外做会改变图结构的剪枝，因为 ESP-DL loader 和后处理器对输出 head 形状比较敏感。
- 如果后续要继续剪枝，建议先在 PC 端保证导出的 ONNX raw head 不变，再重新跑 ESP-DL 量化和板端加载验证。
- 当前模型在暗场景下不会误画框；真实准确率需要用 `/validate` 手机图和真实罐体继续测。
- `/validate` 已接入板端验证可视化：页面按钮会把固件内嵌 Coke/Sprite JPEG 直接送入 `inference_task` 队列，返回 JSON 结果和 `/api/validate/overlay.svg?id=<result_id>` 后处理框图。这个入口可在 `Standby` 下运行，方便只测模型推理链路，不依赖摄像头实时采集。
- 当前手机验证推荐选择 `YOLO11`：最新板端实测 Coke 命中 65%，Sprite 命中 80%，框图为内嵌原图加绿色检测框。`YOLO26` 已部署但 Sprite 样例会误判为 Coke，保留用于观察量化误差和后续校准，不作为主演示路线。
- 验证页已经做了连续点击保护：推理过程中按钮会禁用，前端会丢弃过期响应，后端 overlay 也校验结果 id，避免“上一张图的框叠到下一张图”。
- 当前多框版本最多保留 8 个正式检测框。YOLO bridge 会收集 ESP-DL 后处理后的多个候选，按分数排序后做一次 IoU=0.70 的兜底 NMS，再由主程序按 `box_min_score` 过滤出 `detections`。
- 旧的单框字段仍保留：`object/object_score/object_x/y/w/h` 表示最高分正式命中框；新字段 `detections/detection_count/raw_candidate_count` 用于多目标显示和历史记录。
- `/api/status` 和 `/api/config` 的 `model_info` 会返回模型名、模型字节数、输入尺寸、类别数、最大框数和 NMS 阈值。当前 YOLO11 约 2.98 MB，YOLO26 约 2.82 MB，二者输入尺寸都是 416。
- 历史记录只写超过阈值的正式命中，`source=camera` 表示实时摄像头，`source=validation` 表示手机验证页直接送入推理队列的内嵌图片。

## 5. 迁移到海洋漂流检测

建议路线：

1. 保留 `off/mlp/yolo11/yolo26` 四档，便于做功耗、延时和误检率对比。
2. 建立海洋场景数据集：漂浮塑料瓶、泡沫、渔网、浮标、船只局部、海面 hard negative。
3. 优先用 YOLO11n 路线迁移，因为 ESP-DL 官方 YOLO11 后处理接口更清晰。
4. 板端替换模型后只需要更新类别表和阈值，Web/API 字段可以沿用。
