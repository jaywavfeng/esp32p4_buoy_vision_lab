# ESP32-P4 板端视觉工程技术说明

本文记录当前工程的实际实现，优先服务开发和交接。入口文件是 `main/camera_web_main.c`。

## 1. 当前架构

```text
OV5647 MIPI-CSI camera
  -> camera_task
  -> latest JPEG buffer for debug HTTP
  -> recording_queue -> recording_task -> TF raw MJPEG AVI + JSONL sidecar
  -> inference_queue -> inference_task -> ESP-DL result metadata
  -> idle enrichment_task -> progressive annotated AVI + source-aware JSONL
  -> HTTP API / Web UI / Ethernet export
  -> USB HS MSC -> whole writable TF card, exclusive host ownership
```

网络链路：

```text
Wi-Fi: ESP32-C6 + ESP-Hosted SDIO
Ethernet: ESP32-P4 internal EMAC + IP101GRI PHY
HTTP: one server, listens on all active netif
mDNS: p4-buoy.local, HTTP service _http._tcp:80
```

存储链路：

```text
Primary: TF card
Current accepted backend: check /api/status.storage_backend
Fallback: internal flash FAT, only for emergency demo
```

## 2. 关键文件

| 文件 | 作用 |
|---|---|
| `main/camera_web_main.c` | 摄像头、HTTP、Wi-Fi/Ethernet、mDNS、录像、推理队列、模式切换 |
| `main/coco_espdl_bridge.cpp` | 官方 COCO YOLO11n ESP-DL bridge |
| `main/avi_mjpeg_writer.c` | MJPEG AVI 顺序读写、探测、恢复和 retime |
| `main/recording_enrichment.c` | 空闲补帧、渐进 stride、metadata 和两阶段替换 |
| `main/usb_msc_export.c` | TinyUSB MSC 初始化和 SDMMC TF 介质注册 |
| `main/yolo11_espdl_bridge.cpp` | 自训练 YOLO11 实验模型 bridge |
| `main/yolo26_espdl_bridge.cpp` | 自训练 YOLO26 实验模型 bridge |
| `main/CMakeLists.txt` | 组件依赖、模型和验证图片嵌入 |
| `main/idf_component.yml` | 外部组件依赖，包含 `espressif/mdns` 和 `espressif/esp_tinyusb` |
| `main/Kconfig.projbuild` | 项目配置项 |
| `sdkconfig.defaults` | 默认构建配置 |
| `tools/download_recordings_eth.ps1` | Ethernet/mDNS 下载录像脚本 |
| `tools/benchmark_usb_msc.ps1` | USB 可读写、速度和 SHA256 验收脚本 |
| `tools/apply_component_patches.py` | 构建期修补 managed component，目前用于 TinyUSB MSC 同步写入优化 |

## 3. 关键配置

```text
CONFIG_APP_MDNS_ENABLE=y
CONFIG_APP_HOSTNAME="p4-buoy"
CONFIG_APP_ETH_ENABLE=y
CONFIG_APP_ETH_DHCP_TIMEOUT_MS=8000
CONFIG_APP_ETH_STATIC_FALLBACK_IP="169.254.100.2"
CONFIG_APP_ETH_STATIC_FALLBACK_NETMASK="255.255.0.0"
CONFIG_APP_FILE_DOWNLOAD_CHUNK_BYTES=65536
CONFIG_APP_FIELD_RECORDING_MAX_FPS=12
CONFIG_APP_SD_MAX_FREQ_KHZ=20000
CONFIG_APP_RECORDING_ENABLE=y
CONFIG_APP_RECORDING_MAX_FPS=4
CONFIG_APP_USB_MSC_ENABLE=y
CONFIG_APP_USB_MSC_AUTO_EXPORT=y
CONFIG_APP_USB_MSC_SD_FREQ_KHZ=40000
CONFIG_TINYUSB_MSC_BUFSIZE=32768
CONFIG_FATFS_USE_LABEL=y
CONFIG_APP_ENRICHMENT_ENABLE=y
CONFIG_APP_ENRICHMENT_IDLE_MS=15000
CONFIG_APP_ENRICHMENT_INITIAL_STRIDE=8
```

说明：

- 普通 SERVER 模式保留网页调试能力。
- FIELD 模式用 `CONFIG_APP_FIELD_RECORDING_MAX_FPS` 控制 raw AVI 目标帧率，不再受旧的 `4 FPS` 默认录像限制。
- 文件下载 chunk 默认 64 KiB，用于减少 HTTP 大文件下载时的循环次数。
- TF 默认优先走 SDMMC 4-bit，`CONFIG_APP_SD_MAX_FREQ_KHZ=20000`。当前板端实测 `tf_sdmmc/sdmmc_4bit` 是 FIELD 帧率和 Ethernet 下载提速的关键：raw AVI 约 `11.45 FPS`，EXPORT 平均下载约 `3.08 MiB/s`。

## 4. 运行模式

代码中 `app_mode_t` 当前包含：

```text
APP_MODE_SERVER
APP_MODE_FIELD
APP_MODE_EXPORT
APP_MODE_USB_EXPORT
```

### SERVER

默认模式。HTTP、Wi-Fi、Ethernet、mDNS 可同时存在，用于调试、看状态、少量下载和网页预览。

### FIELD

入口：

```text
POST /api/mode/field?confirm=FIELD
```

代码路径：

```text
field_mode_start_handler()
  -> s_field_mode_requested = true
  -> network_task()
  -> network_watchdog_tick()
  -> enter_offline_tf_capture_mode()
```

行为：

- `stop_webserver()`，同时关闭 mDNS。
- `wifi_shutdown_for_storage_window()`，停止 Wi-Fi/ESP-Hosted 网络链路。
- 如果 TF 已通过 SDMMC 挂载，FIELD 会保留 ESP-Hosted 的底层 SDMMC 初始化状态，避免破坏共享 SDMMC host；网络服务仍会关闭。
- `eth_stop_runtime("field capture")`，停止 Ethernet。
- 重新确认 TF 挂载。
- 设置 `s_app_mode=APP_MODE_FIELD`。
- 开启 raw AVI 录像和 COCO 推理 metadata。
- `s_history_enabled=false`。
- `s_inference_interval_ms=0`。
- `s_jpeg_quality=70`。
- 不调用 `classify_coco_annotated_jpeg()`，不实时生成 annotated AVI。

回 SERVER 目前默认靠复位。这样做是为了让野外采集模式尽可能简单，把资源留给采集、TF 写入和推理。

### EXPORT

入口：

```text
POST /api/mode/export?confirm=EXPORT
```

代码路径：

```text
export_mode_start_handler()
  -> s_export_mode_requested = true
  -> network_task()
  -> network_watchdog_tick()
  -> enter_export_mode()
```

行为：

- `s_app_mode=APP_MODE_EXPORT`。
- 停止视觉、录像、history 和高频推理。
- 发送 `CAMERA_CMD_STANDBY`。
- 等待 `recording_task` 关闭当前 `.part` 分段。
- `wifi_stop_for_export_mode()` 停止 Wi-Fi/AP。
- 保留或启动 Ethernet。
- 保留 HTTP server 和 mDNS。
- `/stream`、`/validate`、dataset run、`/api/frame.jpg` 等高负载入口返回 `409 Conflict`。

EXPORT 模式下建议只访问：

```text
/api/status
/api/recordings
/recording/<name>.avi
/recordingmeta/<name>.jsonl
```

### USB_EXPORT

入口：USB HS 主机枚举自动触发，或手动调用：

```text
GET /api/mode/usb?confirm=USB
POST /api/mode/usb?confirm=USB
```

所有权切换路径：

```text
usb_host_event_callback()
  -> s_usb_export_requested=true
  -> network_watchdog_tick()
  -> enter_usb_export_mode()
  -> reject new storage/stream/dataset work
  -> camera standby + drain queues + finalize AVI
  -> stop HTTP/mDNS/Wi-Fi/Ethernet
  -> storage_unmount()
  -> esp_hosted_deinit()
  -> raw SDMMC 4-bit card init
  -> tinyusb_msc_new_storage_sdmmc(MOUNT_USB)
```

关键约束：

- MSC 注册整张 TF，默认可读写，不暴露内部 flash，也不限制到单一目录。
- `auto_mount_off=1`，拔线或安全弹出后不会自动把 TF 重新交给板端。
- USB 模式中板端不得访问 `/sdcard`；恢复只能安全弹出、等待 2 秒并重启。
- TinyUSB 使用 ESP32-P4 HS OTG、32 KiB MSC buffer；TF 使用 SDMMC 4-bit、默认 40 MHz。
- 构建时根 `CMakeLists.txt` 会调用 `tools/apply_component_patches.py` 修补 `managed_components/espressif__esp_tinyusb/tinyusb_msc.c`，把 MSC 写请求从 deferred 单缓冲路径切到同步 SDMMC 写入。独立 smoke app 实测 deferred 写约 `1.89 MiB/s`，同步写约 `5.35 MiB/s`；主固件最终 USB 验收读约 `7.25 MiB/s`、写约 `5.35 MiB/s`。
- FAT 卷标在正常板端挂载时设为 `P4_BUOY`，不会自动格式化。
- Waveshare 板必须断电后把黄色跳帽从 `HOST` 切到 `DEVICE`，并连接 DEVICE 对应 USB-A 数据口。

`/api/status` 的 USB 字段包括：

```text
usb_msc_enabled
usb_initialized
usb_host_connected
usb_export_requested
usb_storage_owner
usb_writable
storage_quiescing
file_download_clients
usb_last_error
```

### 空闲补帧

`enrichment_task` 仅在以下条件全部成立时运行：

```text
app_mode == SERVER
camera == standby
TF mounted by app
network active and idle >= 15 s
no HTTP download / stream / dataset / live inference
recording, history and inference queues empty
no USB or mode-switch request
```

每个 raw AVI 按 `8 -> 4 -> 2 -> 1` 逐级重建 annotated AVI。每个 pass 的输出帧数和时长都与 raw 相同；关键帧真实推理，其他帧传播最近结果，无目标帧复用 raw JPEG。`coco_espdl_annotate_jpeg()` 只画已有结果，不重复推理。

后台补帧使用独立的 COCO detector，FIELD 实时推理使用前台 detector。进入 FIELD、EXPORT 或 USB_EXPORT 前会等待补帧停止并释放后台实例，避免 ESP-DL 的模型执行上下文跨任务复用。

替换协议：

```text
annotated.avi.part -> annotated.avi.new
old annotated.avi -> annotated.avi.prev
new video + new metadata -> final names
validate frame count and duration
delete .prev
```

取消或失败会删除临时文件并恢复 `.prev`。`/api/status.enrichment` 上报 segment、stride、frame progress、真实推理覆盖率和错误。逐帧 metadata 增加：

```text
result_source
source_frame_index
inference_age_frames
pass_stride
```

## 5. mDNS 固定网址

依赖：

```text
main/idf_component.yml: espressif/mdns
main/CMakeLists.txt: mdns
```

启动逻辑：

```text
start_webserver()
  -> httpd_start()
  -> mdns_start_runtime()
```

注册内容：

```text
hostname: p4-buoy
service: _http._tcp:80
URL: http://p4-buoy.local/
```

`/api/status` 新增字段：

```json
{
  "hostname": "p4-buoy",
  "mdns_url": "http://p4-buoy.local/",
  "access_urls": {
    "mdns": "http://p4-buoy.local/",
    "ap": "http://192.168.4.1/",
    "sta": "http://...",
    "eth": "http://169.254.100.2/"
  }
}
```

Windows 上 `.local` 解析可能受系统服务、防火墙或网卡优先级影响，所以 README 保留 IP 兜底。

## 6. Ethernet

硬件参数：

```text
PHY: IP101GRI
PHY_ADDR=1
MDC=31
MDIO=52
RST=51
REF_CLK=50
TX_EN=49
TXD0=34
TXD1=35
CRS_DV=28
RXD0=29
RXD1=30
```

启动逻辑：

```text
app_main()
  -> esp_netif_init()
  -> esp_event_loop_create_default()
  -> storage_mount()
  -> eth_init_runtime()
  -> start_webserver()
  -> wifi_init_runtime()
```

DHCP fallback：

```text
Ethernet Link Up
  -> start DHCP
  -> wait CONFIG_APP_ETH_DHCP_TIMEOUT_MS
  -> if no DHCP IP:
       stop DHCP client
       set static 169.254.100.2/16
       log ETH static fallback URL
```

`network_watchdog_tick()` 会先处理 FIELD/EXPORT 请求，再判断 `s_eth_started`。这样 Ethernet 已启动时也不会挡住手动进入 FIELD。

## 7. 文件下载

索引：

```text
GET /api/recordings?limit=20
```

下载：

```text
GET /recording/<name>.avi
GET /recordingmeta/<name>.jsonl
```

Range：

```text
Range: bytes=0-1023
HTTP/1.1 206 Partial Content
Accept-Ranges: bytes
```

下载脚本：

```powershell
.\tools\download_recordings_eth.ps1 -Limit 5
```

脚本流程：

```text
try http://p4-buoy.local
  -> POST /api/mode/export?confirm=EXPORT
  -> GET /api/status
fallback http://169.254.100.2
  -> auto-detect local 169.254.* interface
  -> curl --interface <local-ip>
GET /api/recordings
download .avi files
compare downloaded size with metadata bytes
print MiB/s
```

## 8. 推理和模型

板端最终运行格式是 `.espdl`。

```text
.pth/.pt  训练权重
.onnx     中间交换格式
.espdl    ESP32-P4 固件调用格式
```

当前 COCO 模型是官方组件内置的 INT8 ESP-DL 模型，不是本工程本地量化生成。

当前速度结论：

```text
COCO YOLO11n 320 INT8: analysis_ms 约 650-700 ms，只能约 1.5 FPS
自训练 YOLO11/YOLO26 检测实验: 约 14-16 s/帧，不适合最终部署
```

达到 `>=5 FPS` 的下一步应切向轻量分类模型，例如 ResNet/MobileNet/EfficientNet-lite 一类的小输入分类网络，并由训练侧交付可转 `.espdl` 的资产。

完整部署说明见 [model_deployment_guide.md](model_deployment_guide.md)。

## 9. 构建、烧录和验证

构建：

```powershell
.\build_tmp.ps1
```

烧录：

```powershell
.\flash_p4.ps1 -Port COM3
```

只烧录不进 monitor：

```powershell
cd <project-directory>\build
idf.py -p COM3 flash
```

验证重点：

```text
build passes
serial log shows mDNS URL and Ethernet Link Up
/api/status has hostname/mdns_url/access_urls
download script gets at least one .avi
download size equals metadata bytes
Range returns 206
FIELD disables HTTP/Wi-Fi/Ethernet within 10 seconds
FIELD raw AVI target >=8 FPS in acceptance test
recording_sd_errors=0
```
