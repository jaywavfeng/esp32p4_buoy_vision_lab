# YOLO / ESP-DL 板端部署路线记录

本文记录本工程当前实际可运行的 YOLO 部署状态，以及后续迁移到海洋漂流检测目标时可以复用的训练、量化和验证路线。

## 1. 当前固件使用的模型

当前固件嵌入两套自训练 Coke/Sprite 模型：

```text
YOLO11  models/yolo11_coke_sprite_416_s8_p4.espdl
YOLO26  models/yolo26_coke_sprite_raw_heads_416_allint8_p4.espdl
```

运行入口：

```text
main/yolo11_espdl_bridge.cpp
main/yolo26_espdl_bridge.cpp
main/CMakeLists.txt
```

Web/API 选择：

```text
/api/recognition?method=yolo11
/api/recognition?method=yolo26
```

两个模型类别表都是：

```text
0 coke
1 sprite
```

## 2. 板端验证结果

最终固件已完成构建、烧录和 API 验证：

```text
network_mode=sta
sta_ip=192.168.31.7
power_mode=standby  // 验证结束后已回到待机
yolo_input_size=416
yolo26_available=true
yolo11_available=true
```

Wake 后暗场景推理结果：

```text
YOLO11  model=coke-sprite-yolo11n-416-p4  model_bytes=2977040  inference_ms≈15479
YOLO26  model=coke-sprite-yolo26n-416-p4  model_bytes=2815168  inference_ms≈14302
```

暗场景下 YOLO11 返回 `no-object`，YOLO26 返回 `candidate_score=50` 且低于 `box_min_score=96`，因此不画框。这个结果说明阈值过滤生效，并且不会把暗场景误识别成可乐/雪碧。

手机验证可视化也已接入板端 YOLO 队列：

```text
http://<board-ip>/validate
http://<board-ip>/api/validate/run?sample=coke&method=yolo11
http://<board-ip>/api/validate/run?sample=sprite&method=yolo26
http://<board-ip>/api/validate/overlay.svg
```

验证流程不是 PC 端推理，也不是简单显示图片：HTTP handler 会把固件内嵌 JPEG 拷贝到 PSRAM，按 `method=yolo11|yolo26` 构造一个验证任务，投递到 `inference_task` 的同一个 FreeRTOS 队列中。推理完成后页面拿到 JSON 结果，并加载 `overlay.svg` 查看后处理框图。该入口在 `Standby` 下也可运行，适合独立验证模型加载、JPEG 解码、ESP-DL 推理、NMS、坐标映射和阈值过滤。

框图规则：

```text
detections[]                     绿色实线框，可同时显示多个正式命中目标
candidate_score < box_min_score   橙色虚线最高候选框，label=coke-low 或 sprite-low，object=unknown
无有效候选                         不画框，label=no-object 或 unknown
```

最新实测固件已经重新替换 Sprite 验证图，并重新构建烧录。验证接口使用 `box_min_score=50`，结果如下：

```text
sample=coke   method=yolo11  matched=true   detection_count=1  coke   65%  box=[77,182,493,198]
sample=sprite method=yolo11  matched=true   detection_count=1  sprite 80%  box=[255,170,326,194]
sample=coke   method=yolo26  matched=true   detection_count=2  coke   62%  另有低分 sprite 干扰框
sample=sprite method=yolo26  matched=false  detection_count=1  coke   73%  误检局部区域
```

结论：

- `YOLO11` 是当前手机验证和现场演示的稳定路线。
- `YOLO26` 已经完成 raw-head 导出、INT8 量化、板端加载和推理，但 Sprite 类在板端量化模型上仍存在明显误判；PC 端 PyTorch 模型能正确识别同一张图，说明问题更可能来自 YOLO26 的导出、校准集或 INT8 量化误差，而不是样例图本身。
- `/api/history` 已确认写入 `source=validation` 记录，包含模型、类别、置信度、推理耗时、总分析耗时和框坐标。
- 后续如果继续优化 YOLO26，优先扩大校准集并检查 raw one2one head 的输出顺序、输入归一化和 ESP-DL 组件版本；在修复前，README 和网页都把 YOLO26 标注为“实验对比”。

## 3. 多框、NMS 与历史记录

早期固件只显示一个框，并不是 YOLO 只能输出一个目标，而是 `main/yolo11_espdl_bridge.cpp` 和 `main/yolo26_espdl_bridge.cpp` 在遍历 ESP-DL 后处理结果时只保留了最高分候选。当前多框版本的处理流程是：

```text
ESP-DL postprocessor -> 合法候选框 -> 按 score 降序排序 -> IoU=0.70 兜底 NMS -> Top-8 -> box_min_score 过滤正式 detections
```

说明：

- ESP-DL 的 YOLO11/YOLO26 后处理器本身已经完成解码和 NMS；固件里再做一次轻量 IoU NMS 是兜底保护，避免不同组件版本或 raw head 输出导致重复框。
- `candidate_score` 始终记录最高候选分，即使没有达到阈值也返回，方便调参。
- `detections` 只包含超过 `box_min_score` 的正式框，最多 8 个；旧字段 `object/object_score/object_x/y/w/h` 保留，表示最高分正式命中框。
- `/api/status`、`/api/validate/run` 和 `/api/history` 都返回 `detection_count`、`raw_candidate_count` 和 `detections`。
- 历史记录只保存正式命中目标，来源字段 `source` 为 `camera` 或 `validation`；低置信候选不写历史，避免污染实验数据。
- `/api/validate/overlay.svg` 现在把 JPEG 原图以 base64 data URI 嵌入 SVG，再叠加多个框，手机端不会再因为 SVG 外链图片加载失败而只看到黑底框。
- 验证页前端使用 `validationBusy + validationSeq` 锁定按钮并丢弃过期响应；后端要求 `/api/validate/overlay.svg?id=<result_id>` 与最近一次完成的验证 id 一致。这样即使用户在一次推理未完成时快速切换样例，也不会把旧框图套到新图片上。

## 4. YOLO11n 路线

YOLO11n 采用乐鑫 ESP-DL YOLO11 教程推荐的 raw head 导出方式。训练脚本 `tools/train_yolo11_coke_sprite.py` 会重载 Ultralytics `Detect.forward`，让 ONNX 输出六个张量：

```text
box0   [1,64,52,52]
score0 [1,2,52,52]
box1   [1,64,26,26]
score1 [1,2,26,26]
box2   [1,64,13,13]
score2 [1,2,13,13]
```

训练命令：

```powershell
.\.venv_yolo\Scripts\python.exe -u tools\train_yolo11_coke_sprite.py --epochs 40 --imgsz 416 --batch 8 --device auto --name gpu_yolo11n_416
```

PC 端结果：

```text
mAP50      0.9547
mAP50-95   0.5468
Precision  0.9435
Recall     0.9312
```

量化命令：

```powershell
.\.venv_yolo\Scripts\python.exe -u tools\quantize_yolo11_espdl.py --onnx models\yolo11_coke_sprite_416.onnx --output models\yolo11_coke_sprite_416_s8_p4.espdl --input-size 416 --calib-limit 96
```

板端关键点：

- `main/yolo11_espdl_bridge.cpp` 使用 `dl::detect::yolo11PostProcessor` 进行 DFL 解码和 NMS。
- 预处理使用 RGB888、`0..1` 归一化和 `114` 灰边 letterbox，与训练/校准保持一致。
- 模型从 flash rodata 加载，`param_copy=false`，减少 PSRAM 占用。

## 5. YOLO26n 路线

YOLO26n 的关键是导出 `espressif/yolo26` 组件需要的 raw one2one heads，而不是导出 Ultralytics 默认的最终拼接输出。

训练命令：

```powershell
.\.venv_yolo\Scripts\python.exe -u tools\train_yolo26_coke_sprite.py --model yolo26n.pt --epochs 40 --imgsz 416 --batch 8 --device 0 --name gpu_yolo26n_416
```

PC 端结果：

```text
mAP50      0.9419
mAP50-95   0.5375
Precision  0.9044
Recall     0.9093
```

raw head 导出：

```powershell
.\.venv_yolo\Scripts\python.exe -u tools\export_yolo26_raw_heads.py --weights runs\detect\runs\yolo26_coke_sprite\gpu_yolo26n_416\weights\best.pt --output models\yolo26_coke_sprite_raw_heads_416.onnx --imgsz 416
```

量化命令：

```powershell
.\.venv_yolo\Scripts\python.exe -u tools\quantize_yolo26_official_pipeline.py --onnx models\yolo26_coke_sprite_raw_heads_416.onnx --output models\yolo26_coke_sprite_raw_heads_416_allint8_p4.espdl --input-size 416 --calib-limit 96
```

已经修复的坑点：

- 早期 ONNX 只有一个 `output0`，不适合 ESP-DL 的 YOLO26 后处理器。
- 部分 ESP-PPQ 导出的标量参数是 0 维 shape，P4 loader 会在读取参数时崩溃。
- 当前量化脚本在导出前修补这些标量参数，把它们改为长度为 1 的向量；板端已验证不再崩溃。

## 6. 工具链和依赖

当前使用：

```text
torch 2.11.0+cu128
CUDA available: true
GPU: NVIDIA GeForce RTX 3060 Laptop GPU
ultralytics
onnx==1.17.0
onnxruntime
onnxscript
esp-ppq
opencv-python
pillow
```

如果官方源慢，使用清华源或阿里源安装 Python 包。PyTorch CUDA 包仍建议优先使用 PyTorch 官方 index。

## 7. 后续迁移到海洋目标

迁移时建议保持同样的接口：

```text
method=yolo11     主路线
method=yolo26     对照路线
method=mlp        轻量 baseline 或前置过滤
```

需要替换的部分：

1. 数据集从 Coke/Sprite 改为海面漂浮物、塑料瓶、泡沫、渔网、浮标、船只局部和海面 hard negative。
2. 训练脚本中的类别名改为海洋目标类别。
3. `main/yolo11_espdl_bridge.cpp` 和 `main/yolo26_espdl_bridge.cpp` 的类别表同步更新。
4. 网页阈值、推理间隔和历史保存策略按海上误检/漏检情况重新标定。

不需要改的部分：

- 摄像头采集。
- HTTP/MJPEG 服务。
- `/api/status`、`/api/config`、`/api/history` 的结构。
- STA/SoftAP/AP+STA 无线模式对比逻辑。
