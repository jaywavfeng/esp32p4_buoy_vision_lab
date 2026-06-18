# ESP32-P4 智能无线视觉浮标实验工程

本工程是 `<original-project-root>` 的实验副本，目录为 `<repo-root>`。原工程保持只读，本工程用于继续验证 ESP32-P4-WIFI6-DEV-KIT-A 的无线图传、板端识别、低功耗待机、网页调参和后续海洋无线漂流检测球原型。

当前固件已经升级到产品化记录模式。默认识别方法为 `coco`，上电直接启动摄像头、异步推理、历史记录和分段 MJPEG 录像；存储优先使用 TF 卡，若 TF 底层不可用则自动挂载内部 flash FAT fallback 到同一个 `/sdcard` 路径，继续保存最近监控片段、识别事件、快照、周期摘要和手机视频验证数据。默认网络模式为 AP+STA：板子热点 `YOUR_AP_SSID / YOUR_WIFI_OR_AP_PASSWORD` 和路由器 STA 同时启动，手机优先访问固定地址 `http://YOUR_AP_IP/`；连接同一路由器 `YOUR_WIFI_SSID` 的设备也可访问串口打印的 `STA URL`。

## 硬件

- 开发板：Waveshare ESP32-P4-WIFI6-DEV-KIT-A。
- 摄像头：官方 A 套件 OV5647 / Raspberry Pi Camera (B) Rev2.0，接 MIPI-CSI。
- 存储：板上 TF 卡接口已接入软件逻辑；同时提供约 1.9 MB 内部 flash FAT fallback。当前默认开启识别历史、JPEG 快照、分段 MJPEG 录像和周期识别摘要。
- Wi-Fi：ESP32-P4 + ESP32-C6 Wi-Fi 协处理器，默认 AP+STA，可切换 STA、SoftAP 和 AP+STA。

## 本次已验证

- `cmake --build build --target app` 通过，当前产品版本应用镜像 `0xc41790` 字节；14 MB app 分区剩余 `0x1be870` 字节，约 12%。
- `COM3` 分区表 + app 烧录通过，写入后 hash 校验通过。
- 上电摄像头运行：`800x640`、`RGBP`、传感器配置 `50 FPS`，图像采集和模型推理通过长度为 1 的异步队列解耦。
- COCO YOLO11n 320 官方 ESP-DL 模型已在板端加载并推理：`coco-yolo11n-320-s8-v3-p4`，模型 `2,860,704` bytes，COCO 80 类。
- PC 端已用 `yolo11n.pt`、`imgsz=320` 跑完公开样例视频 `data/coco_video/person-bicycle-car-detection.mp4`：源视频 `647` 帧、`12 FPS`、`768x432`，CPU 平均推理 `62.969 ms`，输出带框视频 `reports/coco_video/person_bicycle_car_yolo11n_320_annotated.mp4`。
- 固件样例已替换为 COCO val2017 多目标图 `demo_01~demo_04`，不再用 Coke/Sprite 作为主验证入口；清晰原图和 PC 标注图保存在 `test_assets/coco_classic/images/coco_01*.jpg` 到 `coco_04*.jpg`。
- 板端串口实测 COCO classic 四图：`analysis_ms=651~702`，平均 `666.00 ms`，四图均返回 `8` 个正式框，全部满足 `analysis_ms < 1000` 的单帧预测延迟要求。
- 使用 360MHz CPU、256KB L2 cache、128B cache line；ESP-DL P4 INT8 ISA 内核和检测多核 runtime 已启用，摄像头 JPEG 编码使用硬件编码器。COCO 验证路径当前仍使用 ESP-DL `sw_decode_jpeg`，SOC JPEG 硬件解码可用但未在本次保留为默认路径，因为需要板端对比确认不会让 `analysis_ms` 回退到 1000ms 以上。
- 自训练 YOLO11n 可乐/雪碧模型已在板端加载并推理：`models/yolo11_coke_sprite_416_s8_p4.espdl`，模型约 `2.98 MB`，暗场景实测单帧推理约 `15.5 s`。
- 自训练 YOLO26n 可乐/雪碧模型已在板端加载并推理：`models/yolo26_coke_sprite_raw_heads_416_allint8_p4.espdl`，模型约 `2.82 MB`，暗场景实测单帧推理约 `14.3 s`。
- 因此当前满足 1 秒需求的主路线为 `COCO YOLO11n 320`；自训练 YOLO11/YOLO26 保留为对比和后续重训路线。
- 阈值 `box_min_score=50` 时，四张 COCO classic 图板端分别返回 `8/8/8/8` 个正式框，目标数量明显多于原 Coke/Sprite 单物体验。
- 手机验证页已改为 COCO classic 多目标可视化：点击 `/validate` 中的“板端推理”按钮后，固件会把内嵌 JPEG 拷贝到 PSRAM，送入与摄像头共用的推理队列，并返回 JSON 结果和 `/api/validate/overlay.svg` 后处理框图。
- 自训练 Coke/Sprite 验证图仍保留为历史对比 API，不再放在 `/validate` 主卡片中。此前 `box_min_score=50` 实测：
  - `sample=coke&method=yolo11`：命中 Coke，`score=65%`，框 `x=77 y=182 w=493 h=198`。
  - `sample=sprite&method=yolo11`：命中 Sprite，`score=80%`，框 `x=255 y=170 w=326 h=194`。
  - `sample=coke&method=yolo26`：命中 Coke，但会留下 1 个低分 Sprite 干扰框；建议提高阈值或只作对比观察。
  - `sample=sprite&method=yolo26`：当前 INT8 板端模型仍把局部区域误判为 Coke，说明 YOLO26 量化/校准质量还需继续优化，不建议作为演示主路线。
- 当前可靠演示路线为 `COCO + /validate`；YOLO11/YOLO26 已完成部署和接口接入，但只作为实验对比项。
- 验证页增加了请求锁：一次推理未完成时按钮会禁用，避免连续点击导致上一张 overlay 套到下一张结果上；`/api/validate/overlay.svg?id=<result_id>` 也会校验结果 id，旧 id 会返回 `400`。
- 多框版本已实现 Top-8 检测框输出：ESP-DL 后处理结果会保留 NMS 后多个候选，主程序按 `box_min_score` 过滤正式 `detections`；旧的 `object_*` 字段继续表示最高分正式命中框。
- `/api/validate/overlay.svg` 已改为在 SVG 中内嵌 JPEG 原图 data URI，不再依赖 SVG 外链图片，避免手机端出现黑底只有框。
- 历史记录只写入正式命中目标：实时摄像头和 `/validate` 手机验证只要 `detection_count > 0` 就立即写入 RAM/TF 历史，记录来源 `camera` 或 `validation`、类别、置信度、模型、推理耗时和框坐标。
- 新增监控式分段录像：摄像头开机后按 `CONFIG_APP_RECORDING_MAX_FPS=4` 抽样保存 `/sdcard/esp32p4/recordings/*.avi`，默认每 `60s` 生成一个片段，同时写入 `recordings.jsonl` 和 `summaries.jsonl`。首页按 boot ID 和实际时间重叠将同一时段的原始录像、标注录像合并成一个“监控录像”词条，分别提供两个下载入口；摘要索引继续保留给 API 和搜索，内部 flash fallback 下自动降到 1 FPS。
- 录像时间优先通过 SNTP 同步，无法联网校时时由网页提交手机时间。新录像索引同时保存 UTC epoch 和启动相对时间，手机按本地时区显示日期。
- 删除合并后的监控录像会在同一个存储锁内同步清理原始/标注媒体、sidecar、当前/旧录像索引、摘要索引和事件索引；未配对录像仍可单独删除。录像下载改由异步 HTTP worker 发送，下载期间状态和 wake 请求仍可响应。
- Standby 只关闭 V4L2 流和设备缓冲，不再反复销毁传感器/XCLK/I2C/esp-video；重复 wake/standby 会合并，摄像头打开失败会限速重试。
- 手机视频验证已改为真实连续视频演示：固件内嵌 Intel `store-aisle-detection.mp4` 第 `2400~3300` 帧中每隔 60 帧抽取的一帧，共 16 张连续 JPEG（448 宽、质量 78、总计约 475 KB）。板端逐帧运行 COCO 推理，`/validate` 在同一个结果框中实时显示最新标注帧，完成后以 1 FPS 循环播放；现有 `demo_01~04` 图片验证保持不变。
- 当前刷写固件默认为 `running + coco + apsta`，摄像头上电后启动；网络/HTTP 上电至少开放 5 分钟，若有 AP 客户端、HTTP/API 请求或 `/stream` 客户端活跃则继续保活，全部空闲超过 5 分钟后关闭 HTTP 和 Wi-Fi，下次 reboot 再重新开放。
- ESP-Hosted 已切到板载 ESP32-C6 + SPI full duplex，避免继续占用 TF 需要的 SDMMC host：P4 host 引脚为 `MOSI=14`、`MISO=15`、`CLK=18`、`CS=19`、`HANDSHAKE=16`、`DATA_READY=17`、`RESET=54`。C6 侧必须同步刷写匹配的 SPI slave 固件；若仍是出厂 SDIO slave 固件，AP/STA/HTTP 都不会启动。
- C6 SPI slave 固件已在 C: 临时副本中构建通过，并把可刷写产物复制到 `artifacts/c6_spi_slave/`：`network_adapter.bin=0x1334a0`，C6 OTA app 分区剩余 36%。刷写命令见 `artifacts/c6_spi_slave/README.md`。
- AP+STA 目标模式保持为板子热点 `YOUR_AP_SSID / YOUR_WIFI_OR_AP_PASSWORD` + 路由器 `YOUR_WIFI_SSID / YOUR_WIFI_OR_AP_PASSWORD`。旧 SDIO 固件曾实测 AP/STA 可用；当前 SPI 版本的无线验收以“P4 app + C6 SPI slave 同步刷写后 `/api/status` 可访问”为准。
- HTTP 板端图片验证入口为 `http://<board-ip>/api/validate/run?sample=demo_01..demo_04&method=coco&box_min_score=50`，用于复测四张 COCO classic 图。

## 常用页面和 API

```text
/                                  控制台首页：视频流、识别框、状态、实验日志和调参
/validate                          板端验证可视化页面，可直接把内嵌样例图送入推理队列并显示框图
/stream                            MJPEG 视频流
/api/status                        实时状态 JSON
/api/config                        读取或设置调试参数，写入 NVS
/api/frame.jpg                     当前单帧 JPEG
/api/power?cmd=wake|standby        唤醒/待机摄像头
/api/time/sync?epoch_ms=<unix-ms>  POST，NTP 不可用时使用手机时间校时
/api/vision?enabled=1|0            打开/关闭识别
/api/recognition?method=off|mlp|coco|yolo26|yolo11
/api/netmode?mode=sta|softap|apsta
/api/history?limit=20              历史识别记录
/api/history.jsonl                 当前存储后端历史 JSONL
/api/recordings?limit=20           录像片段索引和周期识别摘要
/api/search?label=&from_ms=&to_ms=&min_score=&type=&limit=&cursor=
                                   服务端 JSONL 搜索，返回事件、片段、摘要和可定位帧
/recording/<name>.avi              播放或下载某个 MJPEG-in-AVI 录像片段
/api/recording/frame.svg?name=<segment>.avi&frame=<n>
                                   从原 MJPEG 抽帧并叠加 sidecar 检测框
/snapshot/<name>.jpg               查看某个历史快照
/api/validate/run?sample=demo_01&method=coco&box_min_score=50
                                   把固件内嵌样例图送入板端推理队列，返回 JSON 结果
/api/validate/overlay.svg?id=<id>  返回指定验证结果的后处理 SVG 框图，id 不匹配会拒绝旧结果
/api/dataset/frame.svg?run_id=<run>&dataset=<dataset>&index=<n>
                                   返回视频验证逐帧 overlay SVG
/validate/coke.jpg                 历史对比用可乐验证图
/validate/sprite.jpg               历史对比用雪碧验证图
```

## 固定地址访问和网络生命周期

1. 重启板子后，固件默认启动 `AP+STA`。手机连接热点 `YOUR_AP_SSID / YOUR_WIFI_OR_AP_PASSWORD`，打开 `http://YOUR_AP_IP/validate` 做板端图片推理验证。
2. 如果手机或电脑已经连接路由器 `YOUR_WIFI_SSID / YOUR_WIFI_OR_AP_PASSWORD`，也可以打开串口打印的 `STA URL`。DHCP 地址可能变化，以串口日志中的 `STA URL` 和 `/api/status` 里的 `sta_url` 为准。
3. 网络和 HTTP 服务上电后至少开放 `300000 ms`。有 AP 客户端、网页/API 请求或 `/stream` 客户端时会刷新保活时间；全部空闲超过 `300000 ms` 后串口打印 `network idle shutdown`，随后热点消失。
4. 空闲关闭后不会自动重开，按当前需求需要 reboot 才会再次开放 5 分钟窗口。
5. 如果串口出现 `Wi-Fi runtime unavailable: ESP_FAIL` 或 `Failed to get ESP_Hosted slave transport up`，说明 ESP32-C6 Wi-Fi 协处理器链路没有起来，固件无法提供 AP/STA/HTTP 页面。优先确认 P4 host 和 C6 slave 是否都已刷为 SPI transport；当前 P4 已是 SPI 配置，若 C6 仍是 SDIO 旧固件会导致 AP/STA 全不可用。此时仍可通过串口 `BOARD_IMAGE_VALIDATION` 验证板端模型延迟。

`/api/status` 现在包含 `network_active`、`ap_url`、`sta_url`、`network_idle_ms`、`network_boot_window_remaining_ms`、`ap_clients` 和 `stream_clients`，用于网页或脚本判断网络是否会被 watchdog 关闭。

网页控制台可以直接调整识别方法、无线模式、画框阈值、推理间隔、推流 FPS、JPEG 质量、历史记录开关、录像开关和历史保存间隔。状态区会显示采集 FPS、推流 FPS、网页 RTT、RSSI、AP 客户端数、重连次数、当前/最低 heap、当前/最低 PSRAM、模型大小、推理 FPS、丢弃推理帧、推理延时、总分析耗时、TF 剩余空间、录像段数量和当前录像片段。

## 识别方法

```text
off       关闭识别，只测试摄像头和无线图传
mlp       16x16 RGB MLP baseline，类别为 unknown/coke/sprite，速度快
coco      官方 COCO YOLO11n 320，ESP-DL INT8，80 类，当前满足 <1000ms 的主路线
yolo11    自训练 Coke/Sprite YOLO11n，416 输入，ESP-DL INT8，保留作对比路线
yolo26    自训练 Coke/Sprite YOLO26n，416 输入，ESP-DL INT8，用于对比 YOLO26 路线
```

这些识别路线都不是颜色阈值。MLP 用于低延时 baseline，`coco` 使用乐鑫官方 COCO YOLO11n 320 INT8 模型；YOLO11/YOLO26 使用本机训练后的 Ultralytics `n` 级模型，再按 ESP-DL 工具链量化成 ESP32-P4 可加载的 `.espdl`。为了保持板端兼容性，自训练路线采用 `nano` 网络、固定 `416x416` 输入和 INT8 量化；没有额外做破坏性结构剪枝，后续如果要继续剪枝，建议先保证导出的 ONNX raw head 与 ESP-DL 后处理器兼容。

PC 端训练指标：

```text
YOLO11n  mAP50=0.9547  mAP50-95=0.5468  Precision=0.9435  Recall=0.9312
YOLO26n  mAP50=0.9419  mAP50-95=0.5375  Precision=0.9044  Recall=0.9093
```

板端验证指标：

```text
COCO YOLO11n 320  model_bytes=2860704  COCO classic serial inference_ms=596~649  analysis_ms=651~702  avg_analysis_ms=666.00  detections=8  COCO mAP50-95=0.275
YOLO11n  model_bytes=2977040  inference_ms≈15479  analysis_ms≈15600
YOLO26n  model_bytes=2815168  inference_ms≈14302  analysis_ms≈15070
```

实际速度会随画面亮度、JPEG 大小、PSRAM 状态和同时访问的客户端变化。推荐调参：

```text
mlp       inference_interval_ms=500~1000
coco      inference_interval_ms=0~1000，可先用 0 做最高吞吐测试，队列忙时会自动丢帧
yolo11    inference_interval_ms=3000~5000
yolo26    inference_interval_ms=3000~10000，用于低频对比时可继续加大
```

## 附件 DeepShip 模型部署尝试

附件目录 `<local-model-reference-dir>` 已做部署检查，优先检查模型为：

```text
04_实验结果_精选/deepship_latest/exports/deepship_resnet18_half_dbb_se_folded.onnx
ONNX bytes: 11303918
opset: 12
input:  input  [batch, 3, 224, 224]
output: logits [batch, 4]
classes: 客船 / 拖船 / 油轮 / 货船
ops: Add, Concat, Constant, Conv, Expand, Gather, Gemm, GlobalAveragePool, MatMul, Mul, Relu, Reshape, Shape, Sigmoid, Unsqueeze
```

结论：该 ONNX 是 224 输入的四分类模型，只输出 `logits[4]`，不输出检测框、类别框坐标或 NMS 结果，因此不能替代当前“输入图像/后续视频，输出带框目标检测结果”的 COCO 主路线。它最多适合作为可选 `deepship_cls` 单图分类验证入口，overlay 不画框。

本次没有把它嵌入固件，原因如下：

- 附件包没有 DeepShip 原始图片或校准图片；递归检查到的图片只有 `cifar_curves/test_accuracy_curve.png` 和 `cifar_curves/train_loss_curve.png` 两张曲线图。ESP-DL/ESP-PPQ 的 ONNX 量化需要 `calib_dataloader` 和 `input_shape`，用曲线图或随机数据校准会让板端精度不可验证。
- 该模型 ONNX 约 `11.3 MB`，当前 app 分区虽然还有约 20% 空间，但嵌入一个未校准、不可画框的分类模型会挤占主检测固件空间。
- 训练摘要里的 DeepShip 数据路径指向外部本机目录，不随附件提供；因此无法复现实测准确率、无法计算板端 INT8 准确率，也无法证明它满足 `analysis_ms < 1000`。

后续如果要接入这条支线，需要提供 DeepShip 代表性校准图片和分类验证集。可按“ONNX 分类模型 -> ESP-DL INT8 -> `deepship_cls` API”的方式增加入口，返回 `label/score/model_bytes/inference_ms/analysis_ms`，但它仍不是视频检测框方案。

## 并发结构

为了避免 YOLO 推理阻塞图传，固件把实时链路拆成多个 FreeRTOS 任务：

```text
camera_task       高优先级，负责 Wake/Standby、V4L2 采集、JPEG 编码、发布最新帧
inference_task    较低优先级，负责 COCO/YOLO11/YOLO26 推理，输入来自长度为 1 的 JPEG 抽帧队列
HTTP workers      多个并发 worker，负责 /stream、/api/status、/api/config 等请求
history_task      负责 RAM 历史队列和文件写入，优先 TF，失败时使用内部 flash fallback
network_task      负责 STA/SoftAP/AP+STA 模式切换
```

关键策略：

- `camera_task` 不等待 YOLO。它只把 JPEG 副本投递给 `inference_task`，然后继续采集下一帧。
- 推理队列长度固定为 1。YOLO 忙时新抽帧会被丢弃并计入 `dropped_inference_frames / inference_queue_drops`，避免旧帧排队造成几十秒延迟。
- YOLO 完成后异步写回最新状态和历史记录；视频流持续刷新，识别框可能稍晚更新。
- 固件缓存最近一次完成的 YOLO 结果，下一轮慢推理进行中网页仍显示上一轮真实结果。
- `Standby` 会关闭摄像头和推理，Web 服务与网络仍保持在线，适合低功耗远程唤醒测试。

## Web 调参和 API 示例

读取配置：

```text
http://<board-ip>/api/config
```

调整阈值、推理间隔、推流帧率和 JPEG 质量：

```text
http://<board-ip>/api/config?box_min_score=96&inference_interval_ms=3000&stream_max_fps=8&jpeg_quality=70
```

切换识别方法：

```text
http://<board-ip>/api/recognition?method=mlp
http://<board-ip>/api/recognition?method=yolo11
http://<board-ip>/api/recognition?method=yolo26
```

切换无线模式：

```text
http://<board-ip>/api/netmode?mode=sta
http://<board-ip>/api/netmode?mode=softap
http://<board-ip>/api/netmode?mode=apsta
```

关键字段：

```text
box_min_score              画框置信度阈值，50-100；误框多就提高，漏检多就降低
candidate_score            最高候选框分数，即使未达到画框阈值也返回
detection_count            超过阈值的正式检测框数量，最多 8 个
detections                 正式检测框数组，包含 label/class_id/score/x/y/w/h
raw_candidate_count        ESP-DL 后处理和兜底 NMS 后的候选框数量
model_info                 当前模型名、大小、输入尺寸、类别数、最大框数和 NMS 阈值
inference_interval_ms      抽帧推理间隔，0 表示尽量每帧推理，不建议给 YOLO 长期开 0
stream_max_fps             单客户端 MJPEG 推流上限
jpeg_quality               JPEG 质量，数值越高图像越好但带宽越大
history                    1/0，打开或关闭历史记录
history_sample_interval_ms 历史抽帧保存间隔
method                     off / mlp / coco / yolo26 / yolo11
netmode                    sta / softap / apsta
rescue_ap                  STA 失败时是否额外开启救援热点
network_active             HTTP/Wi-Fi 当前是否处于开放状态
network_idle_ms            距离最近一次 AP/HTTP/stream 活动的毫秒数
network_boot_window_remaining_ms 上电保底开放窗口剩余毫秒数
ap_url/sta_url             固定 AP 入口和路由器 STA 补充入口
free_heap/min_free_heap    当前/最低内部堆内存余量
free_psram/min_free_psram  当前/最低 PSRAM 余量
```

默认值主要在 `sdkconfig.defaults` 和 `main/Kconfig.projbuild` 中：

```text
CONFIG_APP_WIFI_SSID="YOUR_WIFI_SSID"
CONFIG_APP_WIFI_PASSWORD="YOUR_WIFI_OR_AP_PASSWORD"
CONFIG_APP_AP_SSID="YOUR_AP_SSID"
CONFIG_APP_AP_PASSWORD="YOUR_WIFI_OR_AP_PASSWORD"
CONFIG_APP_AP_STATIC_IP="YOUR_AP_IP"
CONFIG_APP_DEFAULT_NETWORK_MODE=2
CONFIG_APP_NETWORK_BOOT_WINDOW_MS=300000
CONFIG_APP_NETWORK_IDLE_TIMEOUT_MS=300000
CONFIG_APP_DEFAULT_RECOGNITION_METHOD=4
CONFIG_APP_BOOT_STANDBY=n
CONFIG_APP_CAN_BOX_MIN_SCORE=96
CONFIG_APP_INFERENCE_INTERVAL_MS=0
CONFIG_ESP_HOSTED_CP_TARGET_ESP32C6=y
CONFIG_ESP_HOSTED_SPI_HOST_INTERFACE=y
CONFIG_ESP_HOSTED_SPI_HSPI_GPIO_MOSI=14
CONFIG_ESP_HOSTED_SPI_HSPI_GPIO_MISO=15
CONFIG_ESP_HOSTED_SPI_HSPI_GPIO_CLK=18
CONFIG_ESP_HOSTED_SPI_HSPI_GPIO_CS=19
CONFIG_ESP_HOSTED_SPI_GPIO_HANDSHAKE=16
CONFIG_ESP_HOSTED_SPI_GPIO_DATA_READY=17
CONFIG_ESP_HOSTED_SPI_GPIO_RESET_SLAVE=54
```

注意：`/api/config` 和网页设置会写入 NVS，重启后继续生效。若旧 NVS 保存了 `recognition_method=yolo11`，可调用 `/api/recognition?method=coco` 切回当前主路线；擦除 NVS 或全量 erase flash 后会回到 `sdkconfig.defaults` 的默认 COCO。

## YOLO 训练、量化和部署

本工程遵循乐鑫 ESP-DL 的模型量化和 YOLO11 部署思路：PC 端训练 Ultralytics 模型，导出 ESP-DL 后处理器需要的 raw head ONNX，再用 `esp-ppq`/ESP-DL 量化成 `.espdl`。参考：

- ESP-DL 量化文档：`https://docs.espressif.com/projects/esp-dl/zh_CN/latest/tutorials/how_to_quantize_model.html`
- ESP-DL YOLO11n 部署文档：`https://docs.espressif.com/projects/esp-dl/zh_CN/latest/tutorials/how_to_deploy_yolo11n.html`
- YOLO26 组件说明：`https://components.espressif.com/components/espressif/yolo26/versions/0.1.0/readme`

依赖环境：

```powershell
python -m venv .venv_yolo
.\.venv_yolo\Scripts\python.exe -m pip install --upgrade pip
.\.venv_yolo\Scripts\python.exe -m pip install torch torchvision --index-url https://download.pytorch.org/whl/cu128
.\.venv_yolo\Scripts\python.exe -m pip install ultralytics onnx==1.17.0 onnxruntime onnxscript opencv-python pillow esp-ppq
```

如果官方源慢或缺包，可使用清华源或阿里源安装非 PyTorch 包：

```powershell
.\.venv_yolo\Scripts\python.exe -m pip install -i https://pypi.tuna.tsinghua.edu.cn/simple ultralytics onnx==1.17.0 onnxruntime onnxscript opencv-python pillow esp-ppq
.\.venv_yolo\Scripts\python.exe -m pip install -i https://mirrors.aliyun.com/pypi/simple ultralytics onnx==1.17.0 onnxruntime onnxscript opencv-python pillow esp-ppq
```

YOLO11n 训练、导出和量化：

```powershell
.\.venv_yolo\Scripts\python.exe -u tools\train_yolo11_coke_sprite.py --epochs 40 --imgsz 416 --batch 8 --device auto --name gpu_yolo11n_416
.\.venv_yolo\Scripts\python.exe -u tools\quantize_yolo11_espdl.py --onnx models\yolo11_coke_sprite_416.onnx --output models\yolo11_coke_sprite_416_s8_p4.espdl --input-size 416 --calib-limit 96
```

YOLO26n 训练、raw head 导出和量化：

```powershell
.\.venv_yolo\Scripts\python.exe -u tools\train_yolo26_coke_sprite.py --model yolo26n.pt --epochs 40 --imgsz 416 --batch 8 --device 0 --name gpu_yolo26n_416
.\.venv_yolo\Scripts\python.exe -u tools\export_yolo26_raw_heads.py --weights runs\detect\runs\yolo26_coke_sprite\gpu_yolo26n_416\weights\best.pt --output models\yolo26_coke_sprite_raw_heads_416.onnx --imgsz 416
.\.venv_yolo\Scripts\python.exe -u tools\quantize_yolo26_official_pipeline.py --onnx models\yolo26_coke_sprite_raw_heads_416.onnx --output models\yolo26_coke_sprite_raw_heads_416_allint8_p4.espdl --input-size 416 --calib-limit 96
```

板端嵌入位置：

```text
main/CMakeLists.txt
models/yolo11_coke_sprite_416_s8_p4.espdl
models/yolo26_coke_sprite_raw_heads_416_allint8_p4.espdl
main/yolo11_espdl_bridge.cpp
main/yolo26_espdl_bridge.cpp
```

## 构建和烧录

推荐在 ESP-IDF PowerShell 中执行。Windows 路径较长时，组件缓存建议放到短目录：

```powershell
cd <repo-root>
$env:IDF_COMPONENT_CACHE_PATH="<idf-component-cache>"
$env:ESP_IDF_VERSION="6.0"
idf.py build
idf.py -p COM3 flash
```

本机 ESP-IDF 目录是 `<ESP_IDF_PATH>`，但当前 Wi-Fi Remote/ESP-Hosted 组件需要 `ESP_IDF_VERSION=6.0` 才会选择正确配置。

## COCO classic 图片与板端框图

手机访问：

```text
http://<board-ip>/validate
```

这个页面现在默认展示四张 COCO val2017 多目标样例，用于替代原来的 Coke/Sprite 单物体验：

1. 直接点击任意 COCO classic 卡片的“板端推理”。浏览器会请求 `/api/validate/run`，固件把内嵌 JPEG 拷贝到 PSRAM，投递到 `inference_task` 的推理队列，等待 COCO YOLO11n 320 完成后返回 JSON。
2. 页面随后加载 `/api/validate/overlay.svg?id=<id>`，显示后处理框图、类别、候选分、阈值、推理延时和总分析耗时。按钮在推理完成前会锁定，避免快速连续点击造成结果串图。
3. 点击“打开原图”可以全屏查看固件内嵌图；如果要验证真实摄像头链路，可把屏幕对准摄像头，再在首页观察实时视频流上的目标框和状态字段。

固件内嵌的主验证图：

```text
/validate/demo_01.jpg
/validate/demo_02.jpg
/validate/demo_03.jpg
/validate/demo_04.jpg
test_assets/video_frames_320/images/demo_01.jpg
test_assets/video_frames_320/images/demo_02.jpg
test_assets/video_frames_320/images/demo_03.jpg
test_assets/video_frames_320/images/demo_04.jpg
test_assets/coco_classic/images/coco_01.jpg
test_assets/coco_classic/images/coco_01_pc_annotated.jpg
test_assets/coco_classic/images/coco_02.jpg
test_assets/coco_classic/images/coco_02_pc_annotated.jpg
test_assets/coco_classic/images/coco_03.jpg
test_assets/coco_classic/images/coco_03_pc_annotated.jpg
test_assets/coco_classic/images/coco_04.jpg
test_assets/coco_classic/images/coco_04_pc_annotated.jpg
```

`/api/validate/run` 示例：

```text
http://<board-ip>/api/validate/run?sample=demo_01&method=coco&box_min_score=50
http://<board-ip>/api/validate/run?sample=demo_02&method=coco&box_min_score=50
http://<board-ip>/api/validate/run?sample=demo_03&method=coco&box_min_score=50
http://<board-ip>/api/validate/run?sample=demo_04&method=coco&box_min_score=50
```

当前建议使用 `method=coco` 做快速板端演示。`/validate/coke.jpg`、`/validate/sprite.jpg` 以及 `sample=coke|sprite` API 仍保留，用于对比自训练量化链路，但不再作为主页面默认卡片。

COCO classic 板端串口实测：

```text
demo_01  source=512x341  jpeg=65647  detections=8  top=person  inference_ms=649  analysis_ms=702
demo_02  source=500x375  jpeg=54506  detections=8  top=person  inference_ms=602  analysis_ms=654
demo_03  source=512x384  jpeg=59180  detections=8  top=chair   inference_ms=603  analysis_ms=657
demo_04  source=512x384  jpeg=62893  detections=8  top=person  inference_ms=596  analysis_ms=651
```

COCO classic HTTP 复测：

```text
demo_01  ok=true  detections=8  top=person  inference_ms=597  analysis_ms=648
demo_02  ok=true  detections=8  top=person  inference_ms=599  analysis_ms=652
demo_03  ok=true  detections=8  top=chair   inference_ms=602  analysis_ms=655
demo_04  ok=true  detections=8  top=person  inference_ms=598  analysis_ms=653
```

PC 端视频预测输出：

```text
input_video       data/coco_video/person-bicycle-car-detection.mp4
output_video      reports/coco_video/person_bicycle_car_yolo11n_320_annotated.mp4
summary_json      reports/coco_video/prediction_summary.json
source_frames     647
source_fps        12
pc_avg_infer_ms   62.969
detected_classes  person=161, airplane=116, car=90, kite=19, cell phone=4, bus=1
```

返回字段中重点看：

```text
matched           是否命中期望类别
vision.label      当前标签，低于阈值时可能显示 <label>-low
vision.object     达到阈值后的目标类别，低于阈值时为 unknown
candidate_score   最高候选框分数，即使低于阈值也返回
detection_count   正式检测框数量
detections         正式检测框数组，最多 8 个
raw_candidate_count 原始候选数量，便于观察 NMS 和阈值过滤前后的变化
model_bytes/model_input_size 当前模型大小和输入尺寸
box_min_score     当前画框阈值
object_x/y/w/h    后处理后的框坐标
inference_ms      ESP-DL 模型推理耗时
analysis_ms       JPEG 解码、预处理、推理、后处理总耗时
```

框图颜色含义：

```text
绿色实线框   detections 中的正式命中框，可同时显示多个
橙色虚线框   没有正式命中但有最高候选框，页面显示为候选低分标签，不计入命中
无框         没有有效候选，或模型判断为 no-object
```

如果空场景误框多，提高 `box_min_score`；如果真实罐体或手机图漏检，降低阈值或继续采集你手上的罐体照片补充训练集。注意：网页和 `/api/config` 写入的是 NVS，可能覆盖 `sdkconfig.defaults` 的默认 `CONFIG_APP_CAN_BOX_MIN_SCORE=96`；调参时请以 `/api/status` 或验证 JSON 返回的 `box_min_score` 为准。

## 存储、TF 卡和历史记录

客户使用手册和开发技术文档：

```text
docs/customer_manual.md
docs/developer_guide.md
```

历史和录像接口：

```text
http://<board-ip>/api/history?limit=20
http://<board-ip>/api/recordings?limit=20
http://<board-ip>/api/timeline?limit=50
```

统一存储目录结构。路径固定是 `/sdcard/esp32p4`，实际后端由 `/api/status.storage_backend` 表示：`tf_sdmmc` / `tf_sdspi` 为真 TF，`flash_fat` 为内部 flash fallback。

```text
/sdcard/esp32p4/history.jsonl
/sdcard/esp32p4/snapshots/*.jpg
/sdcard/esp32p4/recordings/*.avi
/sdcard/esp32p4/recordings/*.jsonl
/sdcard/esp32p4/recordings.jsonl
/sdcard/esp32p4/summaries.jsonl
/sdcard/esp32p4/datasets/coco_video
/sdcard/esp32p4/dataset_runs
```

历史记录现在只保存超过 `box_min_score` 的正式命中目标，低置信候选不会写入历史。每条记录包含：

```text
source                 camera 或 validation
detection_count        正式框数量
detections             每个框的类别、置信度和坐标
best_score             最高正式命中分数
model/model_bytes      模型名和固件内嵌模型大小
model_input_size       模型输入尺寸，COCO 当前为 320，自训练 YOLO 当前为 416
inference_ms/analysis_ms 推理耗时和总分析耗时
```

录像记录按 `CONFIG_APP_RECORDING_MAX_FPS=4` 抽样保存，默认 `CONFIG_APP_RECORDING_SEGMENT_MS=60000` 每 60 秒一个 `.avi` 片段。每段同时写一个同名 `.jsonl` sidecar，记录每帧检测框、类别、置信度、UTC epoch 和推理耗时。每段结束后写入：

```text
recordings.jsonl   片段 URI、UTC/启动相对时间、帧数、字节数、命中帧和目标统计
summaries.jsonl    周期识别摘要，方便从长视频中定位有目标的片段
```

首页时间线不会重复显示 `summaries.jsonl`，但 `/api/timeline` 和 `/api/recordings` 仍保留摘要数据以兼容现有调用方。页面请求最近 100 条录像索引，将同一 boot ID 且时间重叠达到较短录像 50% 的原始/标注文件合并为一个“监控录像 · 本地日期时间”词条；无法可靠配对时保留单独词条并标明缺失版本。旧录像没有 epoch 时显示“本次启动 + HH:MM:SS”。

统一监控时间线接口：

```text
GET    /api/timeline?limit=50
DELETE /api/recording?name=<file>.avi&confirm=DELETE
DELETE /api/recording?name=<primary>.avi&paired_name=<secondary>.avi&confirm=DELETE
DELETE /api/timeline?from_ms=<start>&to_ms=<end>&confirm=DELETE
DELETE /api/storage/records?scope=all&confirm=DELETE
```

单条删除会同步删除媒体、sidecar 和所有相关索引；提供 `paired_name` 时，两份录像在同一个存储锁内一起删除。成功响应包含 `deleted_files`、`removed_index_rows` 和 `freed_bytes`；目标不存在返回 `404`，文件或索引只完成一部分时返回 `500`。

TF 卡格式化维护接口：

```text
POST /api/storage/remount?confirm=REMOUNT
POST /api/storage/format?confirm=FORMAT
```

注意：格式化只能解决“卡已经被底层识别、但 FAT 文件系统挂载失败”的问题。当前固件已经把 ESP-Hosted 改为 SPI transport，不再用 Wi-Fi SDIO 去占用 TF 需要的 SDMMC host；TF 验收仍以 `/api/status` 中 `tf_ready=true`、`storage_backend=tf_sdmmc` 或 `tf_sdspi`、`sd_total_bytes > 100GiB`、`storage_acceptance_ok=true` 为准。

如果串口或 `/api/status` 显示 `ESP_ERR_TIMEOUT`，说明板端底层没有识别到卡，格式化命令无法到达 TF 卡；应优先检查卡是否插紧、卡槽触点、SDMMC 引脚/供电。`ESP_ERR_NOT_FOUND / no available sd host controller` 是旧 SDIO-host 冲突固件的典型现象；在当前 SPI transport 固件中不应再作为正常状态出现。电脑端建议格式化为 FAT32；大容量卡如果 Windows 只提供 exFAT，建议用 SD Card Formatter 或 DiskGenius 格式化成 FAT32。

2026-06-10 的旧 SDIO 固件实测曾出现 `ESP_ERR_NOT_FOUND / no available sd host controller`，因此本版已切到 C6 SPI transport。后续产品验收必须在同步刷写 P4 app 与 C6 SPI slave 后复测真 TF：`flash_fat` fallback 只能用于页面和小样例应急演示，不算野外长期存储通过。

COCO 视频数据集准备：

```powershell
.\.venv_yolo\Scripts\python.exe tools\prepare_coco_tf_dataset.py --output data\tf_datasets\coco_video
```

本次已从 `data/coco_video/person-bicycle-car-detection.mp4` 导出 `162` 张 512 宽 JPEG，总量约 `1.7 MB`。复制到 TF 卡后路径应为：

```text
TF卡根目录/esp32p4/datasets/coco_video
```

板端视频验证接口：

```text
GET  /api/datasets
PUT  /api/dataset/file?dataset=coco_video&path=frames/frame_00001.jpg
POST /api/dataset/run/start?dataset=coco_video_demo&limit=16&stride=1
GET  /api/dataset/run/status
GET  /api/dataset/run/results?run_id=<run_id>
GET  /api/dataset/frame.svg?run_id=<run_id>&dataset=coco_video_demo&index=<n>
```

`/validate` 页面现在同时提供互不混用的两条验证路径：四张 COCO classic 图片继续调用 `/api/validate/run`；视频按钮调用固件内置的 `coco_video_demo`，逐帧输出板端 overlay 并循环播放。`coco_video` 保留为 TF 卡可选长数据集，不再自动写入四张图片，也不再作为无卡 fallback。

## 面向海洋漂流检测球的下一步

- 把可乐/雪碧数据集替换为海面漂浮物、塑料瓶、泡沫、渔网、浮标、船只局部和海面 hard negative。
- 继续保留 `off/mlp/coco/yolo11/yolo26` 五档，便于做功耗、延时和误检率对比。
- 增加电池电压、太阳能输入、IMU 姿态、GPS、舱内进水检测等状态字段。
- 增加低功耗策略：定时 Wake、低频巡检、检测到目标后提高帧率、关键帧存 TF 卡。
- 用 STA、SoftAP、AP+STA 三种模式记录 RTT、RSSI、断连次数、推流错误数和多客户端稳定性。
