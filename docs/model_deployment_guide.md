# 模型部署教程：从别人训练好的模型到 ESP32-P4 板端运行

这份文档给自己复现用。目标是看完之后能回答四个问题：

1. 别人训练好模型后应该给我什么？
2. `.pth/.pt`、`.onnx`、`.espdl` 分别是什么？
3. 当前工程里的模型是怎么被部署到板子上的？
4. 如果我要换成新的分类模型，应该改哪里、怎么验证？

## 先理解三种模型文件

### `.pth` / `.pt`

这是训练框架里的权重文件，常见于 PyTorch 或 Ultralytics。

用途：

- 继续训练；
- 在 PC 上评估；
- 导出 ONNX。

板端不能直接运行 `.pth/.pt`。

### `.onnx`

ONNX 是跨框架的中间格式。

用途：

- 让训练同学和部署同学交接；
- 在 PC 上用 `onnxruntime` 验证；
- 送进 ESP-DL/ESP-PPQ 量化工具。

ESP32-P4 当前也不是直接运行 ONNX。ONNX 还需要继续量化和转换。

### `.espdl`

这是 ESP-DL 给 ESP32-P4 使用的模型格式。

用途：

- 嵌入固件；
- 板端 `dl::Model` 加载；
- 板端实际推理。

本工程最终部署到板子上的模型就是 `.espdl`。

## 当前工程已有模型

| 方法 | 来源 | 当前部署方式 |
|---|---|---|
| `fish31` | Fish31 MobileNetV3-Small 0.75x 224x224 31-class | `models/fish31_mbv3s_075_224_s8_p4.espdl` 嵌入固件，当前默认 |
| `tinycls` | TinyCNN-XL-Deep 192x192 6-class | `models/tiny_cls_xl_deep_192_6cls_s8_p4.espdl` 嵌入固件 |
| `coco` | `espressif/coco_detect` 组件 | 通过 Kconfig 选择官方 COCO YOLO11n 320 INT8 模型 |
| `yolo11` / `yolo26` / `mlp` | 历史 Coke/Sprite 实验路线 | 源码和复现资料保留；当前首页、手机验证和 `/api/recognition` 主路径不再启用 |

当前默认配置：

```text
CONFIG_APP_DEFAULT_RECOGNITION_METHOD=6
CONFIG_APP_FISH31_INPUT_SIZE=224
CONFIG_APP_FISH31_MODEL_FILE="../models/fish31_mbv3s_075_224_s8_p4.espdl"
CONFIG_APP_TINY_CLS_INPUT_SIZE=192
CONFIG_APP_TINY_CLS_MODEL_FILE="../models/tiny_cls_xl_deep_192_6cls_s8_p4.espdl"
```

也就是默认使用 `fish31` / `fish31-mbv3s075-224-p4`。当前主路径运行时可通过 API 改：

```text
http://<board-ip>/api/recognition?method=fish31
http://<board-ip>/api/recognition?method=tinycls
http://<board-ip>/api/recognition?method=coco
http://<board-ip>/api/recognition?method=off
```

注意：API 会写 NVS，重启后可能保留上一次选择。如果怀疑状态不对，直接调用上面的 API 切回 `fish31`。

## 当前 COCO 模型是怎么部署的

COCO 模型不是放在 `models/` 目录里的本地文件，而是来自组件：

```yaml
# main/idf_component.yml
espressif/coco_detect:
  version: "0.3.2"
```

模型选择在 `sdkconfig.defaults`：

```text
CONFIG_FLASH_COCO_DETECT_YOLO11N_320_S8_V3=y
CONFIG_COCO_DETECT_YOLO11N_320_S8_V3=y
CONFIG_DEFAULT_COCO_DETECT_MODEL=3
CONFIG_COCO_DETECT_MODEL_IN_FLASH_RODATA=y
CONFIG_COCO_DETECT_MODEL_LOCATION=0
```

代码入口：

```text
main/coco_espdl_bridge.cpp
main/coco_espdl_bridge.h
```

关键代码逻辑：

```cpp
extern const uint8_t coco_detect_model_start[] asm("_binary_coco_detect_espdl_start");
extern const uint8_t coco_detect_model_end[] asm("_binary_coco_detect_espdl_end");
```

这两个符号指向固件里的 COCO `.espdl` 模型二进制。桥接代码用它们计算模型大小，并创建 ESP-DL 模型对象。

运行路径大致是：

```text
JPEG 输入
  -> ESP-DL JPEG decode
  -> resize / preprocess 到 320x320
  -> coco_detect / ESP-DL 推理
  -> YOLO 后处理、NMS、坐标映射
  -> vision_result_t
  -> /api/status、/validate、历史记录、录像 sidecar
```

验证结果在：

```text
reports/coco_video/latest_board_coco_verify_summary.json
```

当前实测：

```text
model=coco-yolo11n-320-s8-v3-p4
model_bytes=2860704
inference_ms=596-649
analysis_ms=651-702
avg_analysis_ms=666
```

## 本地 YOLO 模型是怎么部署的

本地 `.espdl` 文件在 `models/`：

```text
models/yolo11_coke_sprite_416_s8_p4.espdl
models/yolo26_coke_sprite_raw_heads_416_allint8_p4.espdl
```

它们在 `main/CMakeLists.txt` 中被嵌入固件：

```cmake
set(YOLO26_MODEL_FILE "${CMAKE_CURRENT_LIST_DIR}/../models/yolo26_coke_sprite_raw_heads_416_allint8_p4.espdl")
target_add_aligned_binary_data(${COMPONENT_LIB} "${YOLO26_MODEL_FILE}" BINARY)

set(YOLO11_MODEL_FILE "${CMAKE_CURRENT_LIST_DIR}/../models/yolo11_coke_sprite_416_s8_p4.espdl")
target_add_aligned_binary_data(${COMPONENT_LIB} "${YOLO11_MODEL_FILE}" BINARY)
```

桥接代码再用链接符号拿到模型地址：

```cpp
// main/yolo11_espdl_bridge.cpp
extern const uint8_t yolo11_model_start[] asm("_binary_yolo11_coke_sprite_416_s8_p4_espdl_start");
extern const uint8_t yolo11_model_end[] asm("_binary_yolo11_coke_sprite_416_s8_p4_espdl_end");

// main/yolo26_espdl_bridge.cpp
extern const uint8_t yolo26_model_start[] asm("_binary_yolo26_coke_sprite_raw_heads_416_allint8_p4_espdl_start");
extern const uint8_t yolo26_model_end[] asm("_binary_yolo26_coke_sprite_raw_heads_416_allint8_p4_espdl_end");
```

如果你改了 `.espdl` 文件名，符号名也会变，bridge 里的 `asm("_binary_...")` 必须一起改。为了少踩坑，可以先保持文件名不变，直接替换文件内容。

## 当前 YOLO 训练和量化复现

创建 Python 环境：

```powershell
cd <project-directory>
python -m venv .venv_yolo
.\.venv_yolo\Scripts\python.exe -m pip install --upgrade pip
.\.venv_yolo\Scripts\python.exe -m pip install torch torchvision --index-url https://download.pytorch.org/whl/cu128
.\.venv_yolo\Scripts\python.exe -m pip install ultralytics onnx==1.17.0 onnxruntime onnxscript opencv-python pillow esp-ppq
```

YOLO11n：

```powershell
.\.venv_yolo\Scripts\python.exe -u tools\train_yolo11_coke_sprite.py --epochs 40 --imgsz 416 --batch 8 --device auto --name gpu_yolo11n_416
.\.venv_yolo\Scripts\python.exe -u tools\quantize_yolo11_espdl.py --onnx models\yolo11_coke_sprite_416.onnx --output models\yolo11_coke_sprite_416_s8_p4.espdl --input-size 416 --calib-limit 96
```

YOLO26n：

```powershell
.\.venv_yolo\Scripts\python.exe -u tools\train_yolo26_coke_sprite.py --model yolo26n.pt --epochs 40 --imgsz 416 --batch 8 --device 0 --name gpu_yolo26n_416
.\.venv_yolo\Scripts\python.exe -u tools\export_yolo26_raw_heads.py --weights runs\detect\runs\yolo26_coke_sprite\gpu_yolo26n_416\weights\best.pt --output models\yolo26_coke_sprite_raw_heads_416.onnx --imgsz 416
.\.venv_yolo\Scripts\python.exe -u tools\quantize_yolo26_official_pipeline.py --onnx models\yolo26_coke_sprite_raw_heads_416.onnx --output models\yolo26_coke_sprite_raw_heads_416_allint8_p4.espdl --input-size 416 --calib-limit 96
```

重新构建和烧录：

```powershell
.\build_tmp.ps1
.\flash_p4.ps1 -Port COM3
```

验证：

```text
http://<board-ip>/api/recognition?method=yolo11
http://<board-ip>/api/status
http://<board-ip>/validate
```

## 如果禹杰给了分类模型，我该怎么部署

假设他给你：

```text
ship_cls.onnx
classes.json
preprocess.json
calib_images/
val_images/
```

第一步：先 PC 验证 ONNX。

```powershell
.\.venv_yolo\Scripts\python.exe - <<'PY'
import onnx
m = onnx.load("models/ship_cls.onnx")
onnx.checker.check_model(m)
print("onnx ok")
for i in m.graph.input:
    print("input", i.name)
for o in m.graph.output:
    print("output", o.name)
PY
```

第二步：用 ESP-DL/ESP-PPQ 量化成 `.espdl`。

当前仓库还没有通用分类量化脚本，建议新增：

```text
tools/quantize_classifier_espdl.py
```

脚本要做这些事：

1. 读取 `ship_cls.onnx`；
2. 读取 `calib_images/`；
3. 按 `preprocess.json` 做 resize、RGB/BGR、mean/std；
4. 调用 ESP-PPQ/ESP-DL 量化；
5. 输出 `models/ship_cls_s8_p4.espdl`；
6. 同时输出 `.json/.info` 报告，方便排查量化结果。

第三步：在 `main/CMakeLists.txt` 嵌入模型。

示例：

```cmake
set(SHIP_CLS_MODEL_FILE "${CMAKE_CURRENT_LIST_DIR}/../models/ship_cls_s8_p4.espdl")
target_add_aligned_binary_data(${COMPONENT_LIB} "${SHIP_CLS_MODEL_FILE}" BINARY)
```

第四步：新增 bridge 文件。

建议新增：

```text
main/classifier_espdl_bridge.h
main/classifier_espdl_bridge.cpp
```

bridge 要提供类似这些函数：

```cpp
bool classifier_espdl_available(void);
uint32_t classifier_espdl_model_bytes(void);
esp_err_t classifier_espdl_classify_jpeg(const uint8_t *jpg_data,
                                         uint32_t jpg_len,
                                         classifier_result_t *out);
```

内部流程：

```text
JPEG decode
  -> resize/crop 到模型输入尺寸
  -> RGB/BGR 转换
  -> mean/std 归一化或 INT8 输入处理
  -> dl::Model forward
  -> 读取 logits
  -> softmax
  -> top1/topk label + score
```

第五步：把 bridge 加进构建。

在 `main/CMakeLists.txt` 的 `SRCS` 增加：

```cmake
"classifier_espdl_bridge.cpp"
```

第六步：在 `main/camera_web_main.c` 接入新方法。

需要改的典型位置：

```text
recognition_method_t enum                  增加 RECOGNITION_METHOD_CLS
recognition_method_name()                  返回 "cls"
parse_recognition_method()                 支持 method=cls
active_model_bytes()/model_bytes_for_method()
model_name_for_method()
model_input_size_for_method()
recognition_method_available()
inference_task 或 classify_* 分发逻辑
/api/status 的 model_info 和 vision 字段
/api/recognition?method=cls
```

分类模型不需要检测框，所以 `vision_result_t` 可以这样填：

```text
label=分类结果
object=分类结果
object_score=top1 分数
candidate_score=top1 分数
detection_count=0
object_x/y/w/h=0
model=模型名
model_bytes=.espdl 字节数
inference_ms=模型推理时间
analysis_ms=完整分析时间
```

第七步：构建、烧录、验证。

```powershell
.\build_tmp.ps1
.\flash_p4.ps1 -Port COM3
```

板端验证：

```text
http://<board-ip>/api/recognition?method=cls
http://<board-ip>/api/status
http://<board-ip>/validate
```

重点看：

```text
model_info.name
model_info.bytes
vision.label
vision.object_score
vision.inference_ms
vision.analysis_ms
```

目标是：

```text
vision.analysis_ms <= 200
```

## 从代码角度理解当前推理链路

主程序在 `main/camera_web_main.c`。

### 摄像头任务

`camera_task` 做：

```text
打开 /dev/video*
采集 V4L2 frame
编码 JPEG
发布最新 JPEG 到 PSRAM 缓存
按 inference_interval_ms 抽帧送入 inference_task
按 recording_max_fps 抽帧送入 recording_task
```

### 推理任务

`inference_task` 做：

```text
从长度为 1 的队列取 inference_job_t
按 job.method 选择模型
调用 classify_coco_jpeg / classify_yolo11_jpeg / classify_yolo26_jpeg / MLP
把结果写回 latest vision
必要时写历史记录
```

队列长度是 1。这样模型慢的时候不会排队堆几十秒，新的帧会被丢弃，图传仍然继续。

### HTTP 输出

HTTP server 输出：

```text
/stream       只发送最新 JPEG，不等待推理
/api/status   返回当前模型、最新识别、存储、网络、FPS
/validate     把内置图片送入同一个推理队列做验证
```

所以换模型时，核心不是改网页，而是把新的分类结果正确填入统一的 `vision_result_t`。

## 常见坑

1. 直接把 `.pth` 放进 `models/` 没用，板子不能跑。
2. 直接把 `.onnx` 放进 `models/` 也没用，必须先转 `.espdl`。
3. 改 `.espdl` 文件名后，bridge 里的 `_binary_..._start/end` 符号也要改。
4. 没有校准集就做 INT8，板端精度不可相信。
5. PC 上快不代表板端快，最终只认 `/api/status` 的 `analysis_ms`。
6. 分类模型没有框，网页 overlay 不能继续按检测框逻辑理解结果。
7. 输入预处理必须和训练一致，RGB/BGR、mean/std、resize/crop 错一个都会导致结果乱。

## 最小复现检查表

拿到新模型后按这个顺序做：

```text
1. 确认有 ONNX、classes、preprocess、calib_images、val_images
2. onnx.checker 检查 ONNX
3. onnxruntime 跑一张图，确认输出 logits 正常
4. 量化生成 .espdl
5. CMake 嵌入 .espdl
6. bridge 用 _binary_xxx_start/end 加载模型
7. camera_web_main.c 增加 method
8. build/flash
9. /api/recognition?method=cls
10. /api/status 看 label/score/inference_ms/analysis_ms
11. 用真实图片重复验证精度
```
