# 模型目录说明

本目录保存可乐/雪碧识别模型、ONNX 导出文件、ESP-DL 量化产物和转换报告。当前固件实际嵌入并运行的模型是：

```text
yolo11_coke_sprite_416_s8_p4.espdl
yolo26_coke_sprite_raw_heads_416_allint8_p4.espdl
```

嵌入位置在 `main/CMakeLists.txt`，板端桥接代码在：

```text
main/yolo11_espdl_bridge.cpp
main/yolo26_espdl_bridge.cpp
```

## 已部署模型

```text
yolo11_coke_sprite_416.onnx
yolo11_coke_sprite_416_s8_p4.espdl
yolo11_coke_sprite_416_s8_p4.json
yolo11_coke_sprite_416_s8_p4.info
```

YOLO11n 可乐/雪碧模型。训练脚本会把 Ultralytics `Detect.forward` 改为导出 `box0/score0/box1/score1/box2/score2` 六个 raw head，量化脚本再使用 `espdl_quantize_onnx` 生成 P4 可加载的 INT8 `.espdl`。板端后处理复用 ESP-DL 的 `yolo11PostProcessor` 做 DFL 解码、sigmoid、NMS 和 letterbox 坐标映射。

```text
yolo26_coke_sprite_raw_heads_416.onnx
yolo26_coke_sprite_raw_heads_416_allint8_p4.espdl
yolo26_coke_sprite_raw_heads_416_allint8_p4.json
yolo26_coke_sprite_raw_heads_416_allint8_p4.info
```

YOLO26n 可乐/雪碧模型。导出时保留 YOLO26 组件需要的 six raw one2one heads，再按 ESP-DL/ESP-PPQ 流程量化。早期转换产物中部分标量参数被导出为 0 维 shape，会导致 P4 loader 崩溃；当前 `tools/quantize_yolo26_official_pipeline.py` 已加入导出修复，把这些参数改为长度为 1 的向量，板端已验证能加载和推理。

## 历史实验产物

目录里还保留了若干旧转换文件，例如：

```text
yolo26_coke_sprite.espdl
yolo26_coke_sprite_o2o_416_*.espdl
yolo26_coke_sprite_raw_heads_416_official_*.espdl
```

这些文件用于记录排查过程，不是当前固件默认嵌入对象。当前固件只使用 `main/CMakeLists.txt` 中指定的两个 `.espdl`。

## 训练报告

```text
reports/yolo11_coke_sprite_pc_report.json
reports/yolo26_coke_sprite_pc_report.json
```

当前 PC 端指标：

```text
YOLO11n  mAP50=0.9547  mAP50-95=0.5468  Precision=0.9435  Recall=0.9312
YOLO26n  mAP50=0.9419  mAP50-95=0.5375  Precision=0.9044  Recall=0.9093
```

## 重新生成

YOLO11：

```powershell
.\.venv_yolo\Scripts\python.exe -u tools\train_yolo11_coke_sprite.py --epochs 40 --imgsz 416 --batch 8 --device auto --name gpu_yolo11n_416
.\.venv_yolo\Scripts\python.exe -u tools\quantize_yolo11_espdl.py --onnx models\yolo11_coke_sprite_416.onnx --output models\yolo11_coke_sprite_416_s8_p4.espdl --input-size 416 --calib-limit 96
```

YOLO26：

```powershell
.\.venv_yolo\Scripts\python.exe -u tools\train_yolo26_coke_sprite.py --model yolo26n.pt --epochs 40 --imgsz 416 --batch 8 --device 0 --name gpu_yolo26n_416
.\.venv_yolo\Scripts\python.exe -u tools\export_yolo26_raw_heads.py --weights runs\detect\runs\yolo26_coke_sprite\gpu_yolo26n_416\weights\best.pt --output models\yolo26_coke_sprite_raw_heads_416.onnx --imgsz 416
.\.venv_yolo\Scripts\python.exe -u tools\quantize_yolo26_official_pipeline.py --onnx models\yolo26_coke_sprite_raw_heads_416.onnx --output models\yolo26_coke_sprite_raw_heads_416_allint8_p4.espdl --input-size 416 --calib-limit 96
```

重新生成后需要执行 `idf.py build flash`，并访问 `/api/status` 确认 `model_bytes`、`vision.model`、`inference_ms` 正常。

## MLP baseline

MLP baseline 不放在本目录，它以 C 头文件形式保存在：

```text
main/coke_sprite_mlp_model.h
```
