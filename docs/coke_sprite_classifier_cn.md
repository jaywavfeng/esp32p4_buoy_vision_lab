# 可乐/雪碧轻量分类器说明

## 当前方案

固件使用 `can-mlp16-public-soda-v1` 轻量分类器，识别类别为：

```text
unknown
coke
sprite
```

它不是颜色阈值规则，而是一个小型神经网络分类器。板端每次从摄像头原始帧中按多尺度滑窗取候选窗口，把窗口缩放为 `16x16 RGB`，像素归一化到 `[-1, 1]`，再运行单隐藏层 MLP。隐藏层为 24 个 ReLU 节点，输出 3 类 softmax 分数。

## 数据来源

- 公开数据集：`LibreYOLO/soda-bottles`
- 工程目录：`data/public_soda_bottles/`
- 开发板补充样本：`data/board_samples/`
- 手机屏幕测试图：`test_assets/phone_gallery/`

公开数据集主要是瓶装饮料，不是专门的易拉罐数据集。为了压低空场景误报，训练时加入了开发板实拍的 `unknown` hard negative。如果要提高真实易拉罐效果，建议继续采集你手上的可乐、雪碧和空背景样本再重新训练。

## 训练命令

```powershell
python tools\train_coke_sprite_classifier.py --dataset data\public_soda_bottles --extra-classification-root data\board_samples --output main\coke_sprite_mlp_model.h --report build\coke_sprite_classifier_report.json --phone-gallery test_assets\phone_gallery --epochs 18 --hidden 24 --max-train-per-class 6000 --max-eval-per-class 900
```

输出文件：

```text
main/coke_sprite_mlp_model.h
build/coke_sprite_classifier_report.json
test_assets/phone_gallery/
```

## 当前训练结果

```text
train_samples = 18480
extra_samples = 480
valid_samples = 2700
test_samples  = 2589
valid_acc     = 98.63%
test_acc      = 97.30%
```

测试集混淆矩阵，行是真实类别，列是预测类别：

```text
           unknown  coke  sprite
unknown       851    38      11
coke            5   808       1
sprite         14     1     860
```

## 板端推理流程

代码位置：[main/camera_web_main.c](../main/camera_web_main.c) 的 `classify_can_candidate()`。

流程：

1. 先对整帧运行一次 MLP，得到一个全局候选。
2. 再用 35%、50%、68%、88% 多个尺度滑窗扫描画面。
3. 每个窗口采样为 `16x16 RGB`，调用 `run_can_mlp_window()`。
4. `update_best_can_window()` 只保留可乐/雪碧分数最高的候选窗口。
5. `candidate_score` 始终记录最高候选分。
6. 只有候选分达到 `box_min_score`，并且高于 `unknown_score` 时，才设置 `object_count=1` 并显示目标框。

## 阈值调节

当前默认阈值：

```text
CONFIG_APP_CAN_BOX_MIN_SCORE=96
```

调节位置：

- 快速改：`sdkconfig.defaults` 中设置 `CONFIG_APP_CAN_BOX_MIN_SCORE=96`
- 菜单改：`idf.py menuconfig` -> `Wi-Fi Camera Web Control` -> `Recognition minimum box score`
- 代码查看：`main/camera_web_main.c` 中 `classify_can_candidate()`

调节建议：

- 空场景、桌面、手指等被框出来：提高到 `97` 或 `98`
- 手机测试图或真实罐体经常不出框：降低到 `94`、`92` 或 `90`
- 改完阈值后需要重新 `idf.py build flash`

## 网页和 API 字段

网页会显示：

```text
推理延时 Inference
候选分 candidate_score
画框阈值 box_min_score
目标分 object_score
可乐分 coke_score
雪碧分 sprite_score
未知分 unknown_score
总分析耗时 analysis_ms
```

`/api/status` 和 `/api/history` 都会返回这些字段，便于手机、电脑或后续程序直接读取。

## 与 YOLO 路线的关系

`mlp` 是本工程的可乐/雪碧轻量 baseline，用来验证摄像头、Wi-Fi、Web、历史记录和资源监控链路。当前固件已经接入 `yolo11` 与 `yolo26` 两个自训练 Coke/Sprite ESP-DL 后端，二者都已经在 ESP32-P4 上完成加载和推理验证。

建议对比方式：

```text
mlp       低延时 baseline，适合持续运行和粗筛
yolo11    主 YOLO 路线，416 输入，类别 coke/sprite
yolo26    YOLO26 对照路线，416 输入，类别 coke/sprite
```

YOLO 数据准备、电脑端训练、量化结果和板端验证记录见 [yolo26_espdl_route_cn.md](yolo26_espdl_route_cn.md)。

## 手机测试图

当前固件已经把两张手机验证图嵌入 flash，并提供板端验证可视化页面：

```text
http://<board-ip>/validate
```

页面上有两种测试方式：

1. 点击“可乐图板端推理”或“雪碧图板端推理”，样例 JPEG 会直接进入板端推理队列，网页返回 JSON 结果和后处理框图。
2. 点击“全屏原图”，把手机屏幕对准 ESP32-P4 摄像头，再在首页观察实时视频流、目标框、分数和推理延时。

如果需要脱离开发板单独查看图片，也可以直接打开：

```text
http://<board-ip>/validate/coke.jpg
http://<board-ip>/validate/sprite.jpg
```

工程仍保留原始文件：

```text
test_assets/phone_gallery/images/coke_01.jpg
test_assets/phone_gallery/images/sprite_01.jpg
```

## 采集开发板样本

```powershell
python tools\capture_board_samples.py --ip 192.168.31.8 --label coke --count 80
python tools\capture_board_samples.py --ip 192.168.31.8 --label sprite --count 80
python tools\capture_board_samples.py --ip 192.168.31.8 --label unknown --count 120
```

采集结果会写入：

```text
data/board_samples/coke/
data/board_samples/sprite/
data/board_samples/unknown/
```

目前补充样本按整图分类方式使用；如果后续想进一步提升目标框位置准确度，可以对真实图片做简单框标注，或者拍摄时把目标罐体放在画面中央并生成裁剪样本。
