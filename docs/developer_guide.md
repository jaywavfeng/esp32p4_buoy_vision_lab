# ESP32-P4 板端视觉工程技术说明

本文记录 `v3.0.1` 固件的实现现状，面向开发维护和交接验收。客户操作说明见 [customer_manual.md](customer_manual.md) 和 Word 用户手册。

## 1. 总体链路

```text
OV5647 MIPI-CSI
  -> camera_task
  -> latest JPEG buffer for Web preview
  -> recording_queue -> recording_task -> raw AVI + annotated AVI + JSONL sidecar
  -> inference_queue -> inference_task -> selected ESP-DL model result cache
  -> manual enrichment_task -> stride=1 annotated rebuild
  -> HTTP API / Web UI / Ethernet download
  -> USB HS MSC -> whole TF card handoff
```

网络链路：

```text
Wi-Fi: ESP32-C6 + ESP-Hosted SDIO
Ethernet: ESP32-P4 EMAC + IP101GRI PHY
HTTP: one server, listens on active netifs
mDNS: p4-buoy.local, _http._tcp:80
```

存储链路：

```text
Primary: TF card, normally mounted as /sdcard
Recording dir: /sdcard/esp32p4/recordings
USB export: app unmounts FatFS, USB owns the whole card
```

## 2. 关键文件

| 文件 | 作用 |
|---|---|
| `main/camera_web_main.c` | Web/API、相机、电源状态、网络、录像、FIELD/USB 模式主流程 |
| `main/avi_mjpeg_writer.c` | MJPEG AVI 顺序读写、探测、恢复和 retime |
| `main/recording_enrichment.c` | 手动补帧、annotated 重建、metadata 和两阶段替换 |
| `main/fish31_espdl_bridge.cpp` | Fish31 分类模型 bridge |
| `main/tiny_cls_espdl_bridge.cpp` | TinyCNN 分类模型 bridge |
| `main/coco_espdl_bridge.cpp` | COCO YOLO11n 检测 bridge 和标注 fallback |
| `main/usb_msc_export.c` | TinyUSB MSC 初始化和 SDMMC 介质注册 |
| `main/CMakeLists.txt` | 组件依赖、模型和验证资产嵌入 |
| `main/Kconfig.projbuild` | 项目配置项 |
| `tools/download_recordings_eth.ps1` | 网线下载录像 |
| `tools/watch_usb_msc.ps1` | USB 状态观察 |
| `tools/benchmark_usb_msc.ps1` | USB 读写和 SHA256 验收 |
| `tools/apply_component_patches.py` | 构建期幂等修补 managed component |

历史实验 bridge 仍在源码中作为默认关闭的兼容路径，但不进入客户首页和默认构建资产。

## 3. 关键配置

```text
CONFIG_APP_MDNS_ENABLE=y
CONFIG_APP_HOSTNAME="p4-buoy"
CONFIG_APP_ETH_ENABLE=y
CONFIG_APP_ETH_STATIC_FALLBACK_IP="169.254.100.2"
CONFIG_APP_RECORDING_ENABLE=y
CONFIG_APP_RECORDING_SEGMENT_MS=60000
CONFIG_APP_RECORDING_SEGMENT_MAX_MS=14400000
CONFIG_APP_FIELD_RECORDING_MAX_FPS=12
CONFIG_APP_USB_MSC_ENABLE=y
CONFIG_APP_USB_MSC_AUTO_EXPORT=y
CONFIG_APP_USB_MSC_SD_FREQ_KHZ=40000
CONFIG_TINYUSB_MSC_BUFSIZE=32768
CONFIG_FATFS_USE_LABEL=y
CONFIG_APP_ENRICHMENT_ENABLE=y
```

录像片段时长运行时由 Web/NVS 控制，合法范围为 `5000-14400000 ms`。`/api/status.config.recording_segment_max_ms` 会返回当前上限。

## 4. 运行模式

### SERVER

默认模式。Web、API、AP/STA、Ethernet 和 mDNS 可用，用于配置、预览、下载和维护。实时图传通过 `POST /api/power?cmd=wake` 唤醒相机，再从 `/stream` 输出 MJPEG。

### FIELD

入口：

```text
POST /api/mode/field?confirm=FIELD
```

核心行为：

- 停止 Web 高负载活动，把资源优先留给摄像头、TF 写入和推理。
- 使用 Web/NVS 保存的模型，默认 `fish31`。
- 采集与推理并发运行。
- recording task 对每个 raw 采集帧写 1 帧 raw 和 1 帧 annotated。
- annotated 使用当前 raw 图像叠加最近一次真实推理结果，保证 raw/annotated 帧数一致。
- 每段闭合后保留 raw AVI、annotated AVI 和必要 sidecar，清理 `.part/.new/.prev` 等临时文件。
- raw/annotated 采用成对轮转和成对闭合。若 reset 或 USB 切换打断片段，启动恢复会删除 `.part` 文件和未配对的 orphan raw/annotated，客户 `recording_groups` 不展示半截记录。

无 Web 客户端、无推流、无下载、无补帧/整理任务并超过配置空闲时间后，自动 FIELD 才会触发。Web 客户端按远端 IP 活动表统计，网线、AP 和 STA 访问都计入。

### USB_EXPORT

入口：USB HS 主机枚举自动触发，或维护 API：

```text
POST /api/mode/usb?confirm=USB
```

切换流程：

```text
detect USB host
  -> reject new storage/stream/dataset work
  -> camera standby
  -> stop recording/inference/enrichment
  -> finalize AVI and cleanup temp files
  -> keep Web/AP/STA/Ethernet online
  -> storage_unmount()
  -> expose whole TF as writable TinyUSB MSC
```

约束：

- USB 模式下板端不得访问 `/sdcard`。
- `/api/status` 继续上报 `usb_host_connected`、`usb_storage_owner`、`storage_quiescing`、`usb_last_error`。
- TinyUSB `DETACHED` 或 USB 配置失活不代表数据线已物理拔出，不会触发 TF 自动重新挂载。
- 安全弹出并物理拔线后，TF 继续保持隔离；用户须从 Web 点击「USB 恢复存储」，或调用 `POST /api/mode/usb/restore?confirm=RESTORE`，成功完成挂载和写读验证后才交还应用。
- 不再设置“抑制下次自动导出”；下一次物理插入仍会自动导出。

## 5. API 摘要

```text
GET  /api/status
GET  /api/config
POST /api/config  (application/x-www-form-urlencoded 请求体)
GET  /api/recordings?limit=20
DELETE /api/recordings?confirm=DELETE
POST /api/recording/enrich?name=<raw-name>
POST /api/mode/field?confirm=FIELD
POST /api/mode/usb?confirm=USB
POST /api/mode/usb/restore?confirm=RESTORE
POST /api/power?cmd=wake|standby
GET  /stream
GET  /validate
```

`GET /api/recordings` 兼容旧 `recordings` 字段，同时提供客户页面使用的段级 `recording_groups`。每组包含时间、模型、raw 信息、annotated 信息、补帧状态和下载链接。

## 6. 手动补帧

补帧任务只处理用户显式点击排队的 raw AVI，不做闲时自动扫描。运行条件：

```text
app_mode == SERVER
camera == standby
TF mounted by app
no stream/download/dataset/mode switch
manual enrichment request queued
```

补帧按 stride=1 逐帧推理并覆盖生成 annotated AVI。分类模型写左上角 Top-1/Top-K，COCO 写检测框。失败时保留旧 annotated，临时文件清理后在 `/api/status.enrichment.last_error` 暴露错误。

## 7. 模型选择

当前客户入口只开放三种模型：

```text
fish31  -> Fish31 MobileNetV3-Small 224, default
tinycls -> TinyCNN Marine 192
coco    -> COCO YOLO11n 320 INT8
```

模型选择通过 Web 保存到 NVS。FIELD、实时预览推理、验证页和手动补帧都读取当前保存模型。

## 8. 构建和烧录

```powershell
.\build_tmp.ps1 -Profile rev1 -Clean
.\build_rev31.ps1 -Clean
.\flash_p4.ps1 -Profile rev31 -Port COM6 -SkipBuild
```

`flash_p4.ps1` 会使用所选 profile 的构建目录，只写 bootloader、partition table 和应用镜像，不执行 `erase-flash`，不会主动清空 NVS 或 TF。大镜像写入较慢；直接 esptool 写入时建议给命令 3-5 分钟超时。

## 9. 验收清单

1. 构建通过，应用分区剩余空间充足。
2. 烧录后 `/api/status`、`/api/config` 可访问。
3. 录像片段时长可设置到 `14400000 ms`，重启后保留。
4. `POST /api/power?cmd=wake` 后 `power_mode=running`，`/stream` 有 MJPEG 输出。
5. `DELETE /api/recordings?confirm=DELETE` 清空录像目录和索引。
6. FIELD 生成 raw/annotated 成对片段，帧数和时长一致。
7. 手动补帧进度到满帧，输出 annotated 可播放。
8. USB 插入后 Windows 识别 `P4_BUOY`，Web 保持在线，`usb_storage_owner=usb`。
9. 安全弹出并拔线后 TF 保持隔离；Web 显式恢复成功后应用重新持有 TF，下一次插入仍自动导出。
10. 文档和 Word 手册不包含面向客户的烧录要求。

## 10. 2026-07-08 板端实测摘要

- `.\build_tmp.ps1` 通过，应用镜像大小 `0x96f210`，最小 app 分区剩余约 33%。
- COM3 直接 esptool 写入 `bootloader.bin`、`partition-table.bin`、`esp32p4_buoy_vision_lab.bin` 并 hash verified。
- `/api/status` 返回 `recording_segment_max_ms=14400000`，首页 UTF-8 中文、模型切换、客户端计数和 14400 秒上限显示正常；有线 Web 访问时 `client_count=1` 且自动采集倒计时暂停。
- `/stream` 可从 standby 自动唤醒摄像头，12 秒抓取约 2.58 MB MJPEG 数据，随后 `POST /api/power?cmd=standby` 可回待机。
- `DELETE /api/recordings?confirm=DELETE` 删除 7 个录像相关文件，释放约 9.6 MB，`recording_groups` 清空。
- 模型保存 API 已验证 `tinycls -> fish31` 可切换，最终交付配置恢复为 Fish31 和 60 秒片段。
- 5 秒片段 FIELD 生成 2 条客户记录，raw/annotated 帧数分别为 24/24、23/23，`recording_groups` 全部 `ready`，raw 与 annotated 均可通过 Web 下载。
- 手动补帧 `POST /api/recording/enrich?name=raw_001...avi` 完成 24/24 帧，`inference_coverage_x1000=1000`，annotated 仍在同一条记录内下载。
- 片段时长从 5 秒恢复到 60 秒后，历史整理完成 `input=2, output=1`，合并后记录 raw/annotated 为 47/47 帧。
- USB 自动导出已实测：Windows 枚举 `USB\VID_303A&PID_4002\P4E8F60AE0A565`，出现 `E: P4_BUOY` FAT32 卷；Web 保持在线，`app_mode=usb_export`、`usb_storage_owner=usb`；已从 `P4_BUOY:\esp32p4\recordings\` 成功读取和复制 AVI。该记录只证明当时的导出和读取链路；当前“安全弹出、物理拔线、保持隔离、Web 显式恢复”流程仍须随修复固件重新验收。
