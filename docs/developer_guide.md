# ESP32-P4 板端视觉工程开发技术文档

本文面向开发人员，说明工程结构、核心任务、数据流、文件作用、构建验证流程，以及 TF 卡记录和 COCO 数据集工作流。

## 1. 工程入口

```text
main/camera_web_main.c          主程序：摄像头、HTTP、Wi-Fi、推理队列、存储历史、录像、验证页面
main/coco_espdl_bridge.cpp      COCO YOLO11n 320 ESP-DL 推理桥接层
main/yolo11_espdl_bridge.cpp    自训练 YOLO11 Coke/Sprite 实验模型桥接层
main/yolo26_espdl_bridge.cpp    自训练 YOLO26 Coke/Sprite 实验模型桥接层
main/CMakeLists.txt             组件依赖、模型和验证图片嵌入
main/Kconfig.projbuild          产品参数：网络、TF、历史、录像、推理间隔等
sdkconfig.defaults              当前产品默认配置
partitions.csv                  16 MB flash 分区表：14 MB app + 约 1.9 MB 内部 FAT fallback
tools/prepare_coco_tf_dataset.py
                                将 COCO 视频转换为 TF 卡 JPEG 帧数据集
docs/customer_manual.md         客户使用手册
docs/developer_guide.md         本文档
```

## 2. 运行任务

```text
camera_task       采集 V4L2 帧，编码 JPEG，发布最新帧，按策略投递推理和录像队列
inference_task    串行执行 COCO/YOLO 推理，避免多个任务同时进入 ESP-DL 模型
history_task      挂载文件存储，保存命中事件 JSONL 和快照；优先 TF，失败时内部 flash fallback
recording_task    保存分段 MJPEG 录像，写 recordings.jsonl 和 summaries.jsonl
dataset_task      读取当前存储后端 COCO JPEG 帧，复用 inference_task 做视频数据集批量验证
network_task      AP/STA/APSTA 切换，网络空闲 watchdog
stream_worker     HTTP async worker，给多个 /stream 客户端发送同一份最新 JPEG
image_selftest    启动后自动跑 demo_01~demo_04 COCO classic 串口验证
```

设计原则：

- 摄像头任务不等待模型推理，也不直接写 TF 卡。
- 推理队列长度为 1，忙时丢弃旧推理请求，保证结果尽量接近当前画面。
- 图片验证和视频验证都复用 `inference_task`，避免多个任务同时进入 ESP-DL。
- 录像队列也独立，存储写入慢时只丢录像帧，不影响实时画面。内部 flash fallback 下录像自动降到 1 FPS。
- COCO 是当前主演示路线；Coke/Sprite YOLO11/YOLO26 只保留实验对比。

## 3. 实时数据流

```text
Camera V4L2 frame
  -> JPEG encoder
  -> publish_frame() 更新最新帧缓存
  -> recording_maybe_queue() 抽样写入 TF 录像队列
  -> queue_yolo_inference() 按 inference_interval_ms 投递 COCO 推理
  -> /stream 多客户端读取最新 JPEG
  -> inference_task 完成后更新 latest vision
  -> history_maybe_queue() 命中时写历史记录
```

`vision_result_t` 是统一识别结果结构，网页、历史、录像摘要和验证 API 都基于它，不直接依赖某个模型的私有输出。

## 4. 存储数据格式

根目录固定为 `CONFIG_APP_SD_MOUNT_POINT/esp32p4`，默认即 `/sdcard/esp32p4`。这个路径可能是真 TF，也可能是内部 flash fallback；以 `/api/status.storage_backend` 为准。

```text
history.jsonl
  每行一个识别事件，包含 seq/time/source/model/detections/snapshot/inference_ms/analysis_ms。

snapshots/*.jpg
  命中事件保存的 JPEG 快照，文件名包含 boot_id 和帧号。

recordings/*.avi
  MJPEG-in-AVI 片段，默认每段 60 秒，默认记录 FPS 为 4。

recordings/*.jsonl
  与同名 MJPEG 片段对应的 sidecar，每行一帧识别结果，用于网页回放叠框和检索。

recordings.jsonl
  每行一个录像片段索引：uri/meta_uri、启动相对时间、UTC epoch、时间来源、帧数、大小和检测统计。

summaries.jsonl
  每段一个识别摘要，用于 API 搜索；首页过滤 summary，避免与录像词条重复。

datasets/coco_video
  由 tools/prepare_coco_tf_dataset.py 生成的 COCO 视频帧数据集。

dataset_runs
  板端视频验证结果：每帧 JSONL 和 summary JSON。
```

## 5. 关键配置

```text
CONFIG_APP_DEFAULT_RECOGNITION_METHOD=4       默认 COCO
CONFIG_APP_DEFAULT_NETWORK_MODE=2             默认 AP+STA
CONFIG_APP_STORAGE_TIMESHARE_BOOT_PROBE_MS=0  TF 维护窗口默认手动触发
CONFIG_APP_SD_ENABLE=y                        开启 TF 卡
CONFIG_APP_FLASH_STORAGE_LABEL="storage"      内部 FAT fallback 分区标签
CONFIG_APP_SD_USE_SDMMC=y                     TF 卡优先走 SDMMC Slot 0
CONFIG_APP_HISTORY_ENABLE=y                   开启识别历史
CONFIG_APP_RECORDING_ENABLE=y                 开启分段录像
CONFIG_APP_RECORDING_SEGMENT_MS=60000         每段 60 秒
CONFIG_APP_RECORDING_MAX_FPS=4                TF 录像 4 FPS
CONFIG_APP_SUMMARY_INTERVAL_MS=60000          每段输出一次摘要
CONFIG_APP_HISTORY_SAMPLE_INTERVAL_MS=2000    命中事件最小保存间隔
```

`SETTINGS_VERSION` 当前为 12。升级后会重置一次旧 NVS，避免坏卡时期保存的 `history=0` 或旧网络模式继续生效。

TF 卡挂载顺序：

```text
SDMMC Slot 0 4-bit -> SDMMC Slot 0 1-bit -> SDSPI2 -> SDSPI3 -> flash_fat fallback
```

内部 fallback 使用 `partitions.csv` 的 `storage` FAT 分区，挂到同一个 `/sdcard` VFS。`storage_prepare_dirs_after_mount()` 只负责创建 `history/snapshots/recordings/datasets/dataset_runs`；手机默认视频验证读取固件 rodata 中独立的 `coco_video_demo` 16 帧序列，不再把 `demo_01~04` 写进 `coco_video`。

正常 P4 Function EV Board / P4-Eye 的 TF 卡座使用 `CMD=44, CLK=43, D0-D3=39-42, LDO=4`。如果状态页看到 `ESP_ERR_TIMEOUT`，表示底层没有卡响应，格式化不会生效；如果能识别卡但 FAT 挂载失败，可通过 `POST /api/storage/format?confirm=FORMAT` 格式化。

`/api/storage/remount?confirm=REMOUNT` 和 `/api/storage/format?confirm=FORMAT` 会排队进入 `storage_service_task`。该任务先停止 HTTP/Wi-Fi，调用 `esp_hosted_deinit()` 暂停 C6 transport，再执行 TF 挂载/必要时格式化。维护完成后默认 `esp_restart()` 恢复 AP+STA；不要在当前构建中热重启 ESP-Hosted，因为 C6 transport 热恢复在本板上仍不如整机重启可靠。`CONFIG_ESP_HOSTED_MEMPOOL_PREFER_SPIRAM=y` 已启用，用于降低 Wi-Fi/视频/AI 同时运行时的内部 DMA RAM 压力。

2026-06-10 旧 SDIO 产品固件实测：ESP-Hosted 以 SDIO Slot 1 连接 C6 Wi-Fi 协处理器后，`sdmmc_host_init()` 对 TF 卡 Slot 0 返回 `ESP_ERR_NOT_FOUND`，底层日志为 `no available sd host controller`。因此 2026-06-11 起 P4 host 已改为 C6 SPI full duplex，以便 TF 保留 SDMMC Slot 0。现场若仍看到 `ESP_ERR_NOT_FOUND / no available sd host controller`，优先确认实际刷入的 P4 app 与 C6 slave 是否都已切到 SPI transport。

## 6. 构建与刷写

```powershell
$env:IDF_PATH='<ESP_IDF_PATH>'
$env:ESP_IDF_VERSION='6.0'
$env:ESP_ROM_ELF_DIR='<IDF_TOOLS_PATH>\esp-rom-elfs\20241011'
$env:PATH='<IDF_TOOLS_PATH>\ninja\1.12.1;<IDF_TOOLS_PATH>\cmake\4.0.3\bin;<IDF_TOOLS_PATH>\ccache\4.12.1\ccache-4.12.1-windows-x86_64;<IDF_TOOLS_PATH>\riscv32-esp-elf\esp-15.2.0_20251204\riscv32-esp-elf\bin;'+$env:PATH
& '<IDF_TOOLS_PATH>\cmake\4.0.3\bin\cmake.exe' --build build --target app
```

刷写 app-only：

```powershell
<IDF_PYTHON> -m esptool --chip esp32p4 -p COM3 -b 921600 --before default-reset --after hard-reset write-flash --flash-mode dio --flash-freq 80m --flash-size 16MB 0x8000 build\partition_table\partition-table.bin 0x10000 build\esp32p4_buoy_vision_lab.bin
```

## 7. 验证清单

1. 构建通过，`check_sizes.py` 显示 app 分区剩余为正。
2. 串口启动出现：
   - `TF card mounted at /sdcard ...`，或 `flash fallback storage mounted at /sdcard`
   - `AP URL: http://YOUR_AP_IP/`
   - `BOARD_IMAGE_VALIDATION ... analysis_ms < 1000`
3. 访问 `/api/status`，确认：
   - `file_storage_mounted=true`
   - `storage_backend=tf_sdmmc/tf_sdspi/flash_fat`
   - `sd_mount_mode=sdmmc_4bit` 或 `sdmmc_1bit`
   - `recording_enabled=true`
   - `recording_frames` 持续增长
   - `history_enabled=true`
4. 等待超过 60 秒后访问 `/api/timeline?limit=20`，确认 API 保留 `recording/history/summary`，首页只显示录像和识别事件。
5. 打开 `/recording/<name>.avi`，确认可以播放/下载；打开 `/recordingmeta/<name>.jsonl`，确认有每帧识别 sidecar 和 `epoch_ms`。
6. 打开 `/validate`，分别运行图片验证和视频验证。
7. 下载录像期间轮询 `/api/status` 并发送 wake，确认控制接口持续响应。
8. 连续执行 10 次 Standby/Wake，确认无 `video init failed`，每次可重新取得 JPEG。

## 8. COCO 视频数据集工作流

生成 TF 卡数据集：

```powershell
.\.venv_yolo\Scripts\python.exe tools\prepare_coco_tf_dataset.py --output data\tf_datasets\coco_video
```

如果 TF 卡在 PC 上挂载为 `E:\`：

```powershell
.\.venv_yolo\Scripts\python.exe tools\prepare_coco_tf_dataset.py --output data\tf_datasets\coco_video --tf-root E:\
```

生成帧默认来自 `data/coco_video/person-bicycle-car-detection.mp4`，会保存为 512 宽 JPEG 序列和 `manifest.jsonl`。板端路径固定为 `/sdcard/esp32p4/datasets/coco_video`。

板端视频验证 API：

```text
GET  /api/datasets
PUT  /api/dataset/file?dataset=coco_video&path=frames/frame_00001.jpg
POST /api/dataset/run/start?dataset=coco_video_demo&limit=16&stride=1
GET  /api/dataset/run/status
GET  /api/dataset/run/results?run_id=<run_id>
GET  /api/dataset/frame.svg?run_id=<run_id>&dataset=coco_video_demo&index=<n>
```

`/api/dataset/run/start` 会同步返回 `run_id` 并先发布 `queued` 状态。状态接口随后返回 `queued/running/done`、`last_frame_index` 和 `last_overlay_uri`，网页据此只展示当前运行的结果。缓存按 `run_id + dataset + frame_index` 匹配，开始新运行后旧 `run_id` 的内置帧 overlay 会返回 404。

Server 模式把当前运行结果保存在 PSRAM 中供网页播放；Field 模式另外写入 `/sdcard/esp32p4/dataset_runs/<run_id>.jsonl` 和 `<run_id>_summary.json`。验收时关注 `analysis_ms < 1000`、`ok_frames`、`failed_frames` 和类别统计。

删除和维护 API：

```text
DELETE /api/recording?name=<file>.avi&confirm=DELETE
DELETE /api/recording?name=<primary>.avi&paired_name=<secondary>.avi&confirm=DELETE
DELETE /api/timeline?from_ms=<start>&to_ms=<end>&confirm=DELETE
DELETE /api/storage/records?scope=all&confirm=DELETE
POST   /api/storage/format?confirm=FORMAT
POST   /api/storage/remount?confirm=REMOUNT
POST   /api/time/sync?epoch_ms=<unix-ms>

```

单条删除在 `s_storage_lock` 下删除媒体、`.part`/`.corrupt` 残留、同名 sidecar，并通过临时文件、`fsync`、原子替换重写 `recordings*.jsonl`、`summaries*.jsonl` 和 `events.jsonl`。提供 `paired_name` 时，两组文件和索引在同一次加锁期间处理；省略时保持原有单文件行为。目标不存在返回 `404`，任一文件或索引失败返回 `500`。

首页另外请求 `/api/recordings?limit=100`，按同一 boot ID 配对 raw/annotated：实际时间重叠必须达到较短录像时长的 50%，多个候选时优先重叠比例最高、开始时间最接近的记录。配对仅发生在浏览器展示层，不迁移 TF 文件或改变 `/api/timeline`、`/api/recordings` 的原始响应。

## 2026-06-11 offline monitor acceptance update
- True TF/SD is now the product acceptance target. `flash_fat` is a rescue/demo backend only.
- `/api/status` must report `tf_required=true`, `tf_ready=true`, `storage_backend=tf_sdmmc` or `tf_sdspi`, `sd_total_bytes > 100 GiB`, and `storage_acceptance_ok=true`.
- ESP-Hosted is configured for SPI full duplex instead of SDIO so TF can keep SDMMC Slot 0.
- P4 host SPI pins: `MOSI=14`, `MISO=15`, `CLK=18`, `CS=19`, `HANDSHAKE=16`, `DATA_READY=17`, `RESET=54`, `SPI clock=40 MHz`.
- The ESP32-C6 coprocessor firmware must be built/flashed with matching SPI slave transport; an SDIO C6 firmware will not work with this P4 app.
- Matching C6 SPI slave artifacts are available in `artifacts/c6_spi_slave/`; `network_adapter.bin` size is `0x1334a0`, C6 OTA app partition free is 36%.
- New APIs: `/api/search`, `/api/recording/frame.svg`, `/api/dataset/frame.svg`.
- JSONL index version is `4`; recording indexes add `start_epoch_ms`, `end_epoch_ms` and `clock_source`, while sidecar/event lines add `epoch_ms`.
- Recording retention keeps at least the larger of 4 GiB or 5% of TF capacity free by deleting oldest segments and sidecars first.
- AP+STA remains default: STA `YOUR_WIFI_SSID` plus AP `YOUR_AP_SSID`. No AP/HTTP/stream activity within 5 minutes after boot stops HTTP/Wi-Fi until reboot.
- Build verified on 2026-06-11: app bin `0xc41790`, smallest app partition `0xe00000`, `0x1be870` free (12%).

删除实现只允许白名单目录和白名单扩展名，避免通过 URL 删除工程外文件。

单条录像删除在 `s_storage_lock` 下处理媒体、sidecar、`recordings*.jsonl`、`summaries*.jsonl` 和 `events.jsonl`。索引先写临时文件并 `fsync`，再通过备份重命名替换；响应返回删除文件数、移除索引行数和释放字节数。

`/recording/*` 使用现有 async HTTP worker，Range 逻辑保持不变，避免大文件传输占住 httpd 主任务。摄像头待机只执行 `VIDIOC_STREAMOFF`、`munmap`、`VIDIOC_REQBUFS(count=0)` 和 `close`，保留 `example_video_init()` 建立的传感器/XCLK/I2C/esp-video 状态供下一次 wake 复用。

Server→Field 切换时，若真 TF 已在独立 SDSPI2 上健康挂载，则复用现有挂载并执行 Field 写入准备，不再立即卸载重挂，以避免 FatFS drive slot 暂未释放导致的 `ESP_ERR_NO_MEM`。

## 9. 后续扩展点

- 给 MJPEG 片段增加轻量索引跳转，让摘要可直接定位到片段内秒级位置。
- 在 `summaries.jsonl` 中加入每类目标持续时间估计和高置信度关键帧 URI。
- 若后续需要 MP4/H.264，应优先复用 `esp_h264` 组件，但要重新评估 TF 写入带宽和浏览器兼容性。
