# COCO 模型验证说明

本文记录当前 `v3.0.1` 固件中 COCO 对照模型的验证路径。COCO 不是默认模型，客户默认使用 Fish31；COCO 通过 Web「模型切换」或 `POST /api/config`（表单请求体 `method=coco`）选择后用于演示、实时预览、FIELD 录像和手动补帧。

## 模型信息

| 项目 | 内容 |
|---|---|
| Backend | Espressif `espressif/coco_detect` |
| Model | `coco_detect_yolo11n_320_s8_v3.espdl` |
| Target | ESP32-P4 |
| Input | `320 x 320 x 3` |
| Classes | COCO 80 classes |
| Board model bytes | `2,860,704` |

该模型作为通用检测对照保留。当前客户首页只开放 `fish31`、`tinycls`、`coco` 三种模型。

## 板端单图验证

固件嵌入 4 张 COCO val2017 验证图，路径为：

```text
test_assets/video_frames_320/images/demo_01.jpg
test_assets/video_frames_320/images/demo_02.jpg
test_assets/video_frames_320/images/demo_03.jpg
test_assets/video_frames_320/images/demo_04.jpg
```

历史板端测量摘要：

```text
CPU:                360 MHz
L2 cache:           256 KB, 128-byte line
Model:              coco-yolo11n-320-s8-v3-p4
Input:              320 x 320
demo_01 measure:   inference 645 ms, analysis 697 ms, detections 8
demo_02 measure:   inference 597 ms, analysis 650 ms, detections 8
demo_03 measure:   inference 612 ms, analysis 666 ms, detections 8
demo_04 measure:   inference 605 ms, analysis 660 ms, detections 7
Measure average:   inference 614.75 ms, analysis 668.25 ms
```

`analysis_ms` 包含 JPEG decode、ESP-DL preprocess、model inference 和 postprocess，是板端验收的主要指标。

## `/validate` 演示

验证页入口：

```text
http://169.254.100.2/validate
```

相关接口：

```text
POST /api/validate/run  (表单请求体：method=coco&sample=demo_01&box_min_score=50)
GET  /api/validate/overlay.svg?id=<id>
POST /api/dataset/run/start?dataset=coco_video_demo&method=coco&limit=16&stride=1
GET  /api/dataset/frame.svg?run_id=<run>&dataset=coco_video_demo&index=<n>
```

COCO 演示短视频使用 `test_assets/coco_video_demo/frames/` 中 16 帧 JPEG。浏览器会逐帧请求板端推理 overlay，用于确认检测框绘制和视频演示链路正常。

## FIELD 和补帧行为

当当前模型保存为 `coco` 时：

- FIELD 中 raw/annotated 仍保持相同帧数和时长。
- annotated 每帧使用当前 raw 图像叠加最近一次 COCO 检测结果。
- 手动补帧会 stride=1 重建 annotated AVI，COCO 继续绘制检测框。
- Web 录像记录会标注该条推理视频使用的模型。

COCO 推理速度明显低于 Fish31/TinyCNN，因此标签或检测框的更新频率会低于 raw 采集帧率，这是预期行为；视频帧数不会因此减少。

## 验收点

1. Web 保存 `method=coco` 后 `/api/status.recognition_method` 为 `coco`。
2. `/validate` 四张 COCO 图片可生成 overlay。
3. 16 帧 `coco_video_demo` 能完成运行，状态不报错。
4. FIELD 采集后 raw/annotated 成对出现，annotated 有检测框。
5. 手动补帧完成后进度到满帧，annotated 可下载播放。
