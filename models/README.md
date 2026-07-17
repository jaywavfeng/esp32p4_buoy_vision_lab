# 模型目录说明

当前 `v3.1.0` 客户交付固件只保留并默认使用以下模型资产：

| 文件 | 用途 |
|---|---|
| `fish31_mbv3s_075_224_s8_p4.espdl` | 默认 Fish31 分类模型，固件嵌入 |
| `fish31_mbv3s_075_224_s8_p4.json` / `.info` | Fish31 量化和结构记录，用于排查 |
| `tiny_cls_xl_deep_192_6cls_s8_p4.espdl` | TinyCNN Marine 分类模型，固件嵌入 |
| `tiny_cls_xl_deep_192_6cls_s8_p4.json` / `.info` | TinyCNN 量化和结构记录，用于排查 |

COCO YOLO11n 320 INT8 来自 `espressif/coco_detect` 组件，不在 `models/` 目录放置本地副本。

旧实验 ONNX、量化中间件和 legacy 检测资产已从仓库删除，以免客户交接时误认为仍是当前功能。当前默认部署不受这些删除影响。

## 嵌入位置

```text
main/CMakeLists.txt
main/fish31_espdl_bridge.cpp
main/tiny_cls_espdl_bridge.cpp
main/coco_espdl_bridge.cpp
```

## 板端实测摘要

| 方法 | 类型 | 实测判断 |
|---|---|---|
| `fish31` | MobileNetV3-Small 0.75x 224x224 31 类 | 5 分钟板端采样 p95 analysis `176 ms`，latest `4.91 FPS` |
| `tinycls` | TinyCNN-XL-Deep 192x192 6 类 | 5 分钟板端采样 p95 analysis `102 ms`，latest `8.61 FPS` |
| `coco` | 官方 YOLO11n COCO INT8 320x320 80 类 | 5 分钟板端采样 p95 analysis `1424 ms`，latest `0.69 FPS` |

完整记录见 `docs/model_benchmark_results_cn.md`。
