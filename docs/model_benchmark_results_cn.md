# Fish31 / TinyCNN / COCO 三模型板端基准

本页记录当前分支的三条主演示模型路径。所有板端指标均来自 2026-07-02 在 ESP32-P4 实板上通过 `/api/status` 连续 5 分钟采样，不使用 PC-only 结果代替。

## 模型概览

| 方法 | 模型 | 输入 | 类别数 | 输出类型 | 板端资产 |
|---|---|---:|---:|---|---|
| `fish31` | Fish31 MobileNetV3-Small 0.75x | 224x224 | 31 | 分类 Top-K | `models/fish31_mbv3s_075_224_s8_p4.espdl` |
| `tinycls` | TinyCNN-XL-Deep Marine | 192x192 | 6 | 分类 Top-K | `models/tiny_cls_xl_deep_192_6cls_s8_p4.espdl` |
| `coco` | Espressif COCO YOLO11n INT8 | 320x320 | 80 | 检测框 | `espressif/coco_detect` 组件内置 |

Fish31 是当前默认方法，`/api/status.model_info` 应返回 `name=fish31-mbv3s075-224-p4`、`input_size=224`、`class_count=31`、`bytes=1184144`。TinyCNN 的 `model_info.name` 为 `tiny-cnn-cls-192-6cls-p4`，COCO 的 `model_info.name` 为 `coco-yolo11n-320-s8-v3-p4`。

## PC / 训练侧指标

| 模型 | PC / 训练侧指标 |
|---|---|
| Fish31 | 附件训练报告：MobileNetV3-Small-0.75x，参数量 `908759`，valid accuracy `91.18%`，test accuracy `90.16%`。 |
| TinyCNN | 本分支只接收 ESP-DL 部署资产和数据集说明，未收到可复算的完整 PC accuracy 报告；当前验收以板端样例 Top-1 和 5 分钟 `/api/status` 采样为准。 |
| COCO | 官方 COCO YOLO11n INT8 检测模型；组件参考指标见 `docs/coco_image_validation.md`，本页只记录当前板端速度。 |

## 板端 5 分钟实测

命令格式：

```powershell
python tools\sample_status_latency.py 169.254.100.2 --duration-min 5 --interval-ms 200 --method <method> --wake --set-interval-zero --bind-ip <host-apipa-ip> --output reports\<method>_status_latency_5min.csv
```

| 方法 | samples | p95 analysis_ms | p95 inference_ms | avg FPS | latest FPS | 结果说明 |
|---|---:|---:|---:|---:|---:|---|
| `fish31` | 1501 | 176 ms | 172 ms | 4.96 | 4.91 | p95 < 200 ms；5 FPS 目标非常接近但本次 latest 未达到 5。 |
| `tinycls` | 1501 | 102 ms | 98 ms | 8.54 | 8.61 | p95 < 200 ms 且 latest FPS >= 5。 |
| `coco` | 1501 | 1424 ms | 1296 ms | 0.73 | 0.69 | 检测参考模型，明显慢于分类模型。 |

## 手机验证资产

Fish31 样例来自 iNaturalist/Fish31 标准候选，经过 ESP32-P4 板端 `method=fish31` 筛选，最终内置 4 张不同鱼类类别：`fish_23`、`fish_15`、`fish_19`、`fish_21`。筛选记录在 `test_assets/fish31_validation/manifest.json`，16 帧视频 demo 为 `fish31_video_demo`。

TinyCNN 样例来自 LaRS 候选，经过 ESP32-P4 板端 `method=tinycls` 筛选，最终内置 4 张非 `unknown` 图片：`buoy`、`ship_part`、`buoy`、`buoy`，且源图互不相同。`foam`、`net`、`plastic_bottle` 没有高置信正确候选，因此不硬塞进演示。筛选记录在 `test_assets/tinycls_marine_demo/manifest.json`，16 帧视频 demo 为 `tinycls_marine_demo`。

COCO 继续使用 COCO classic 图片和 `coco_video_demo`。分类模型的 `/validate` 可视化显示 Top-1 横幅和 Top-K 条形信息，不画检测框；COCO 保持检测框可视化。

## 本次板端验证

- `/api/status`：默认 `recognition_method=fish31`，`model_info.name=fish31-mbv3s075-224-p4`，`input_size=224`，`class_count=31`，`bytes=1184144`。
- Fish31 4 张内置样例：`fish31_01..04` Top-1 全部正确。
- TinyCNN 4 张内置样例：`tiny_01..04` Top-1 全部正确且均非 `unknown`。
- `fish31_video_demo`：16/16 成功，分类 labels 覆盖 `fish_23`、`fish_15`、`fish_19`、`fish_21`。
- `tinycls_marine_demo`：16/16 成功，分类 labels 覆盖 `buoy` 和 `ship_part`。
- `coco_video_demo`：16/16 成功，检测 labels 包含 `person`，overlay 为检测框。
