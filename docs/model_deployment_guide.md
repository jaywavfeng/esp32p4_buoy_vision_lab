# 模型部署说明（v3.1.0）

本文说明当前固件实际开放和验收的模型链路。客户首页、FIELD 野外录像、手动补帧和 `/validate` 只开放三种模型：Fish31、TinyCNN、COCO。

## 1. 当前模型

| 方法 | 类型 | 板端模型 | 当前用途 |
|---|---|---|---|
| `fish31` | 分类 | `models/fish31_mbv3s_075_224_s8_p4.espdl` | 默认模型，FIELD 和手动补帧主链路 |
| `tinycls` | 分类 | `models/tiny_cls_xl_deep_192_6cls_s8_p4.espdl` | 轻量 Marine 分类对照 |
| `coco` | 检测 | `espressif/coco_detect` 组件内置 COCO YOLO11n 320 | 通用检测演示和对照 |

板端最终运行 `.espdl` 或 ESP-DL 组件内置模型，不直接运行 `.pth`、`.pt` 或 `.onnx`。

## 2. 文件交付要求

新增模型交接时，至少需要保留：

```text
model.espdl 或 model.onnx
classes.json
preprocess.json
calib_images/
validation_images/
```

即使对方已经交付 `.espdl`，也应保留 `classes.json` 和 `preprocess.json`。label 顺序、RGB/BGR、resize/crop、mean/std 任一项不一致，都会导致板端结果异常。

## 3. 构建嵌入入口

本地分类模型在 `main/CMakeLists.txt` 中嵌入：

```cmake
set(TINY_CLS_MODEL_FILE "${CONFIG_APP_TINY_CLS_MODEL_FILE}")
target_add_aligned_binary_data(${COMPONENT_LIB} "${TINY_CLS_MODEL_EMBED_FILE}" BINARY)

set(FISH31_MODEL_FILE "${CONFIG_APP_FISH31_MODEL_FILE}")
target_add_aligned_binary_data(${COMPONENT_LIB} "${FISH31_MODEL_EMBED_FILE}" BINARY)
```

默认路径来自 `main/Kconfig.projbuild` 和 `sdkconfig.defaults`：

```text
CONFIG_APP_DEFAULT_RECOGNITION_METHOD=6
CONFIG_APP_FISH31_MODEL_FILE="../models/fish31_mbv3s_075_224_s8_p4.espdl"
CONFIG_APP_TINY_CLS_MODEL_FILE="../models/tiny_cls_xl_deep_192_6cls_s8_p4.espdl"
CONFIG_FLASH_COCO_DETECT_YOLO11N_320_S8_V3=y
```

COCO 模型由 `espressif/coco_detect` 组件提供，不在 `models/` 目录保留本地副本。

## 4. 运行时选择

客户页面「模型切换」会保存到 NVS。也可以通过 API 验证：

```powershell
curl.exe --noproxy "*" -X POST -H "Content-Type: application/x-www-form-urlencoded" --data "method=fish31" "http://169.254.100.2/api/config"
curl.exe --noproxy "*" -X POST -H "Content-Type: application/x-www-form-urlencoded" --data "method=tinycls" "http://169.254.100.2/api/config"
curl.exe --noproxy "*" -X POST -H "Content-Type: application/x-www-form-urlencoded" --data "method=coco" "http://169.254.100.2/api/config"
curl.exe --noproxy "*" "http://169.254.100.2/api/status"
```

状态里重点检查：

```text
recognition_method
model_info.name
model_info.bytes
vision.label
vision.top_k
vision.inference_ms
vision.analysis_ms
```

FIELD 野外录像和手动补帧都使用当前保存模型。分类模型在推理视频左上角写 Top-1/Top-K，COCO 绘制检测框。

## 5. 新增分类模型的最小接入方式

建议从 `fish31_espdl_bridge.cpp` 或 `tiny_cls_espdl_bridge.cpp` 复制新 bridge，不要把模型逻辑直接堆在 `camera_web_main.c`。

典型文件：

```text
main/new_cls_espdl_bridge.h
main/new_cls_espdl_bridge.cpp
models/new_cls_s8_p4.espdl
```

bridge 对外至少提供：

```cpp
bool new_cls_espdl_available(void);
uint32_t new_cls_espdl_model_bytes(void);
esp_err_t new_cls_espdl_classify_frame(const uint8_t *data,
                                       uint32_t data_size,
                                       uint32_t width,
                                       uint32_t height,
                                       uint32_t pixel_format,
                                       new_cls_espdl_result_t *out);
esp_err_t new_cls_espdl_classify_validation_jpeg(const uint8_t *jpg_data,
                                                 uint32_t jpg_len,
                                                 new_cls_espdl_result_t *out);
```

主流程需要同步更新：

```text
recognition_method_t enum
recognition_method_name()
configured_default_recognition_method()
active_model_bytes()
model_input_size_for_method()
model_class_count_for_method()
model_name_for_method()
recognition_method_available()
analyze_frame()
classify_validation_*()
/api/config method parsing
Web model selector
/validate METHOD_DEMOS
```

分类结果写入 `vision_result_t` 时遵循当前约定：

```text
label/object = Top-1 label
object_score/candidate_score = Top-1 score
top_k = Top-K array
detection_count = 0
object_x/y/w/h = 0
model = model name
inference_ms = model inference time
analysis_ms = full analysis time
```

## 6. 板端实测摘要

| 方法 | p95 analysis | latest FPS | 说明 |
|---|---:|---:|---|
| `fish31` | 176 ms | 4.91 | 默认分类链路 |
| `tinycls` | 102 ms | 8.61 | 轻量分类对照 |
| `coco` | 1424 ms | 0.69 | 检测对照，速度明显更低 |

完整记录见 [model_benchmark_results_cn.md](model_benchmark_results_cn.md)。

## 7. 验收清单

1. `idf.py build` 或 `.\build_tmp.ps1` 通过。
2. 烧录后 `/api/status.model_info.name` 为预期模型。
3. Web 保存 Fish31/TinyCNN/COCO 后，重启仍保持。
4. `/validate` 图片和短视频可运行，无乱码、无卡死。
5. FIELD 采集后 raw/annotated 成对出现。
6. 推理视频标注模型正确，分类标签在左上角更新。
7. 点击「补帧」后该条记录按当前模型重建 annotated，进度到满帧。

## 8. 常见问题

- 直接把 `.pth` 或 `.onnx` 放进 `models/` 不会让固件自动加载；当前固件只嵌入配置好的 `.espdl` 或组件模型。
- 修改 `.espdl` 文件名后，CMake 嵌入路径和 bridge 中 `_binary_..._start/end` 符号要同步。
- PC 速度不能替代板端速度，最终只认可 `/api/status` 的 `analysis_ms` 和 `inference_fps_x100`。
- 分类模型没有检测框，Web overlay 和推理视频必须按 Top-1/Top-K 展示。
