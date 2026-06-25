# 给禹杰：模型选型、训练和轻量化交接

## 背景

当前 ESP32-P4 板端已经能跑模型，但速度不够：

| 当前模型 | 类型 | 板端格式 | 大小 | 输入 | 板端速度 |
|---|---|---|---:|---:|---:|
| COCO YOLO11n 320 | 检测 | ESP-DL INT8 `.espdl` | 2,860,704 bytes | 320x320 | `analysis_ms=651-702`，平均约 666 ms |
| Coke/Sprite YOLO11n | 检测 | ESP-DL INT8 `.espdl` | 2,977,040 bytes | 416x416 | 约 15.5 s |
| Coke/Sprite YOLO26n | 检测 | ESP-DL INT8 `.espdl` | 2,815,168 bytes | 416x416 | 约 14.3 s |

最终目标是板端推理达到 5 FPS 以上，也就是完整 `analysis_ms <= 200 ms`。这里的 `analysis_ms` 包含预处理、模型推理、后处理，不只是模型 forward 时间。

当前 YOLO 检测框不是必须的。为了速度，建议改成分类模型，只输出当前画面属于哪一类以及置信度。

## 希望你负责的任务

1. 重新选型轻量分类网络，目标是 ESP32-P4 上 5 FPS 以上。
2. 训练并导出可部署模型。
3. 给出 PC 端精度、混淆矩阵和模型大小。
4. 提供量化校准集和验证集，方便我们转成 ESP-DL INT8 后做板端验证。

## 推荐模型方向

优先考虑：

- MobileNetV3-Small
- MobileNetV2 小宽度版本，例如 width multiplier 0.35/0.5
- ShuffleNetV2 0.5x
- EfficientNet-Lite0 的缩小版本
- 自定义小 CNN

谨慎考虑：

- ResNet18：结构简单、好训，但原始模型可能偏大偏慢。可以作为 baseline，但不一定满足 200 ms。

暂时不建议：

- YOLO 系列检测模型；
- Transformer/Attention 类模型；
- 动态输入尺寸模型；
- 大量 MatMul、复杂 reshape、动态 shape 的模型。

原因是 ESP32-P4 上最终要走 ESP-DL/ESP-PPQ 量化，结构越规整越容易部署。

## 分类任务建议

如果任务只是判断“当前画面里是什么目标”，分类输出可以设计成：

```text
background / target_a / target_b / target_c ...
```

或者按项目最终需求改成：

```text
background / bottle / foam / buoy / ship / other
```

类别表必须固定，并随模型一起交付。

## 输入尺寸建议

从小到大试：

```text
96x96
128x128
160x160
224x224
```

建议先训 `128x128` 或 `160x160`。如果准确率明显不够，再升到 `224x224`。当前 DeepShip 附件模型是 `224x224` ResNet18，ONNX 约 11.3 MB，可能仍偏大。

## 交付物清单

请最终给我这些文件：

```text
model_best.pth 或 model_best.pt
model_best.onnx
classes.txt 或 classes.json
preprocess.json
calib_images/
val_images/
train_report.json 或 README.md
```

`preprocess.json` 至少写清：

```json
{
  "input_size": [1, 3, 128, 128],
  "color_order": "RGB",
  "resize": "resize_short_then_center_crop 或 direct_resize",
  "mean": [0.485, 0.456, 0.406],
  "std": [0.229, 0.224, 0.225],
  "input_range": "0..1",
  "output": "logits",
  "activation": "softmax",
  "classes": ["background", "target_a", "target_b"]
}
```

校准集要求：

- 每类尽量均衡；
- 至少 100 张，最好 300-500 张；
- 必须是真实部署场景或接近真实部署场景；
- 包含 hard negative，例如海面、反光、水花、天空、远处无关物体；
- 不要只给训练曲线截图，量化不能用曲线图做校准。

验证集要求：

- 不和训练集重复；
- 文件夹结构清楚；
- 能复现 PC 精度和板端 INT8 精度。

## ONNX 导出要求

ONNX 尽量满足：

- opset 12 或 13；
- 固定 batch，最好 `[1, 3, H, W]`；
- 输出一个 `logits`，形状 `[1, num_classes]`；
- BN 已经 fold；
- 不包含训练态节点；
- 不包含动态 shape；
- 能用 `onnxruntime` 在 PC 上直接跑通。

导出后请自己先跑：

```powershell
python -m pip install onnx onnxruntime
python - <<'PY'
import onnx
model = onnx.load("model_best.onnx")
onnx.checker.check_model(model)
print("onnx ok")
print("inputs:", [i.name for i in model.graph.input])
print("outputs:", [o.name for o in model.graph.output])
PY
```

## 板端部署环境

硬件：

```text
ESP32-P4
CPU 360 MHz
PSRAM 32 MB
Flash 16 MB
摄像头 OV5647 MIPI-CSI
```

软件：

```text
ESP-IDF v6.0.1
ESP-DL
ESP-PPQ / espdl_quantize_onnx
模型最终格式：ESP-DL INT8 .espdl
```

当前应用结构：

```text
main/camera_web_main.c          统一推理队列和 API
main/coco_espdl_bridge.cpp      当前 COCO 检测模型桥接
main/yolo11_espdl_bridge.cpp    自训练 YOLO11 桥接
main/yolo26_espdl_bridge.cpp    自训练 YOLO26 桥接
main/CMakeLists.txt             把 .espdl 嵌入固件
```

你训练好的 ONNX 不能直接放到板子运行。我们会先用校准集把 ONNX 量化成 `.espdl`，然后在固件里新增分类 bridge。

## 验收指标

第一阶段：

```text
ONNX 可以用 onnxruntime 复现
PC 验证集准确率和混淆矩阵清楚
ONNX 文件大小可接受
校准集/验证集齐全
```

第二阶段：

```text
ESP-DL INT8 量化成功
板端能加载 .espdl
/api/status 返回模型名、模型大小、label、score、inference_ms、analysis_ms
analysis_ms <= 200 ms
```

如果 `analysis_ms` 超过 200 ms，需要继续减小输入尺寸、减小通道数或换更轻网络。

## 我这边会做的板端工作

拿到你的模型后，我会：

1. 用校准集把 ONNX 量化为 `.espdl`。
2. 在 `main/CMakeLists.txt` 里嵌入 `.espdl`。
3. 新增类似 `main/classifier_espdl_bridge.cpp` 的桥接代码。
4. 在 `main/camera_web_main.c` 里新增一个识别方法，例如 `cls` 或 `deepship_cls`。
5. 在 `/api/status` 和 `/validate` 里返回分类结果。
6. 在板端测 `inference_ms` 和 `analysis_ms`。

## 一句话版需求

请提供一个适合 ESP32-P4 的轻量分类 ONNX 模型，固定输入、固定类别、输出 logits，并配套类别表、预处理参数、校准图片和验证集。目标是板端完整分析时间 200 ms 以内。
