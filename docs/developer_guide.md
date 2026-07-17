# ESP32-P4 板端视觉工程技术说明

本文记录 `v3.1.0` 固件的实现现状，面向开发维护和交接验收。客户操作说明见 [customer_manual.md](customer_manual.md) 和 Word 用户手册。

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
CONFIG_APP_USB_MSC_SD_FREQ_KHZ=20000
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
- `CONFIG_APP_USB_MSC_SD_FREQ_KHZ` 只是 USB 上限。USB 必须沿用板端刚通过真实写入、`fsync`、重开和读回验证的总线宽度与频率；不能把已回退到 `1-bit/10 MHz` 的卡重新强制到 `4-bit/40 MHz`。
- ESP-Hosted 使用 SDMMC Slot 1，TF 使用 Slot 0。释放 Slot 0 时，IDF 6.0.1 补丁会把当前槽归还仍注册的 Slot 1，并保留空槽 ISR 防护，避免 C6 SDIO 中断丢失后重启 P4。
- `/api/status` 继续上报 `usb_host_connected`、`usb_storage_owner`、`storage_quiescing`、`usb_last_error`。
- Windows 安全弹出、TinyUSB `DETACHED`、USB host inactive 或物理拔线会触发 `USB_RESTORE`，自动把 TF 从 MSC 侧释放并恢复给应用。
- 一直插着 USB 时，用户也可以从 Web 点击「USB 恢复存储」，或调用 `POST /api/mode/usb/restore?confirm=RESTORE`，成功完成挂载和写读验证后交还应用；该手动恢复会抑制本次仍插着的线立即重新自动导出，下一次 USB 边沿仍会自动导出。
- 不再设置“抑制下次自动导出”；下一次物理插入仍会自动导出。

硬件连接：电脑应通过支持数据的 A-to-C 或 C-to-C 线连接 J15 USB 2.0 Type-C `DEVICE` 口。J18 叠层 Type-A 是 P4 作为 USB 主机时使用的 `HOST` 口；不要用 A-to-A 线连接电脑，也不要带电改变 HOST 供电跳接。官方端口定义见 [ESP32-P4-Function-EV-Board v1.5.2 User Guide](https://documentation.espressif.com/esp-dev-kits/en/latest/esp32p4/esp32-p4-function-ev-board/index.html)。

历史结论：USB MSC 在 `bca4815`（v2.0.0）首次加入；v2.0.0、v3.0.0 与修复前 v3.0.1 都采用“TinyUSB 无介质枚举 -> 卸载 FatFS/Slot 0 -> 重建 SDMMC 卡 -> `tinyusb_msc_new_storage_sdmmc()` -> 重新枚举”的同一主流程，也都把 USB TF 固定成 `4-bit/40 MHz`。因此本次故障不是近期重构改变了核心交接方式，而是新 TF 在该档位可枚举、可读但真实写入超时，加上 IDF 6.0.1 共享 Slot 0/1 的当前槽清理缺陷。v3.1.0 修复保留原交接方式，增加验证档位继承、共享控制器修复和 Web retry；安全弹出、host detach 或拔线仍会自动排队恢复 TF。

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

构建脚本会把 build 输出和 managed components 放到 `C:\Espressif\idf_build_outputs\...`、`C:\Espressif\idf_project_managed_components\...`，并在 ESP-IDF component manager 中启用 `IDF_PROJECT_MANAGED_COMPONENTS_PATH` 覆盖，避免异地 clone 路径过长或包含中文导致构建失败。`flash_p4.ps1` 会使用所选 profile 的短路径构建目录，只写 bootloader、partition table 和应用镜像，不执行 `erase-flash`，不会主动清空 NVS 或 TF。大镜像写入较慢；直接 esptool 写入时建议给命令 3-5 分钟超时。

## 9. 验收清单

1. 构建通过，应用分区剩余空间充足。
2. 烧录后 `/api/status`、`/api/config` 可访问。
3. 录像片段时长可设置到 `14400000 ms`，重启后保留。
4. `POST /api/power?cmd=wake` 后 `power_mode=running`，`/stream` 有 MJPEG 输出。
5. `DELETE /api/recordings?confirm=DELETE` 清空录像目录和索引。
6. FIELD 生成 raw/annotated 成对片段，帧数和时长一致。
7. 手动补帧进度到满帧，输出 annotated 可播放。
8. USB 插入后 Windows 识别 `P4_BUOY`，Web 保持在线，`usb_storage_owner=usb`。
9. 安全弹出、host detach 或拔线后 TF 自动恢复给应用；Web 显式恢复也可在 USB 仍插着时手动切回，下一次插入仍自动导出。
10. 文档和 Word 手册不包含面向客户的烧录要求。

## 10. 2026-07-17 v3.1.0 交付验证摘要

- 已从 GitHub `v3.1.0` tag 干净 clone，运行 `.\build_rev31.ps1 -Clean` 构建通过，证明远端源码、依赖锁定和短路径 managed component 方案可在异地 checkout 复现构建。
- 已用 `.\flash_p4.ps1 -Profile rev31 -Port COM6 -SkipBuild` 烧录真实板端，esptool 完成 bootloader、partition table 和 app 写入校验；脚本未执行 `erase-flash`，不清空 NVS 或 TF。
- 烧录后通过 `http://192.168.1.80/api/status` 验证 Web/API 在线，状态包含 `app_mode=server`、`recognition_method=fish31`、`eth_ip=169.254.100.2`、`usb_storage_owner=app`、`sd_mounted=true`、`tf_ready=true`、`storage_acceptance_ok=true`、`storage_write_verified=true`、`sd_mount_mode=sdmmc_1bit`、`recording_segment_ms=1800000`、`recording_segment_max_ms=14400000`。
- 历史功能复测记录保留在仓库根目录 [../TEST_ISSUES.md](../TEST_ISSUES.md)。该报告记录 2026-07-17 早期 `3.0.1` 板端功能复测，覆盖 USB、FIELD、reset 恢复、录像下载、实时图传和验证任务；当前 `v3.1.0` 沿用其已修复的主流程，并额外完成远端 clone 构建、烧录和文档一致性收尾。
