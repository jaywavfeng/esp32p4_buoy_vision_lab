# ESP32-P4 Buoy Vision Lab

这是 ESP32-P4 浮标视觉原型工程。当前主目标是：高帧率采集并存储视频，板端写入推理 metadata，离线或有线高效导出文件；网页图传只作为调试入口，不作为野外采集主链路。

## 连接方式速查

| 场景 | 地址 | 说明 |
|---|---|---|
| 固定网址 | `http://p4-buoy.local/` | 首选访问方式，依赖 mDNS；Windows 上不稳定时用下面的 IP 兜底 |
| Wi-Fi AP | `http://192.168.4.1/` | 开发板热点入口 |
| Ethernet 直连 fallback | `http://169.254.100.2/` | 网线直连电脑、没有 DHCP 时使用 |
| STA | 看 `/api/status.sta_url` 或串口日志 | 接入路由器后的地址不固定 |

常用确认命令：

```powershell
curl.exe http://p4-buoy.local/api/status
curl.exe http://169.254.100.2/api/status
curl.exe http://169.254.100.2/api/recordings?limit=20
```

当前实测：板端会注册 `p4-buoy.local`，但这台 Windows 电脑上 mDNS 解析会超时；直连 IP `169.254.100.2` 可稳定作为兜底。

如果电脑有多块网卡，直连 `169.254.100.2` 可能走错网卡。先查看本机 APIPA 地址：

```powershell
Get-NetIPAddress -AddressFamily IPv4 | Where-Object { $_.IPAddress -like "169.254.*" }
```

必要时绑定本机以太网地址：

```powershell
curl.exe --interface 169.254.12.109 http://169.254.100.2/api/status
```

## 当前硬件

| 模块 | 当前使用情况 |
|---|---|
| 主控 | ESP32-P4 |
| 开发板 | Waveshare ESP32-P4-WIFI6-DEV-KIT-A |
| 摄像头 | OV5647 / Raspberry Pi Camera(B) Rev2.0，MIPI-CSI |
| Wi-Fi | 板载 ESP32-C6，ESP-Hosted SDIO |
| TF 卡 | 已用于视频和 metadata 存储，当前实测后端为 `tf_sdmmc` / `sdmmc_4bit`；运行时仍以 `/api/status.storage_backend` 和 `sd_mount_mode` 为准 |
| Ethernet | RJ45，ESP32-P4 EMAC + IP101GRI PHY |
| USB | UART Type-C 用于烧录/日志；USB HS OTG 的 DEVICE 口可把整张 TF 作为可读写 U 盘导出 |
| LCD/触摸/DSI/音频等 | 当前核心流程未使用，定制板可按需求裁剪 |

给硬件设计的完整交接见 [docs/zeqi_hardware_handoff.md](docs/zeqi_hardware_handoff.md)。

## 文档入口

- 技术实现说明：[docs/developer_guide.md](docs/developer_guide.md)
- 有线/离线传输策略：[docs/wired_transfer_plan.md](docs/wired_transfer_plan.md)
- 模型部署教程：[docs/model_deployment_guide.md](docs/model_deployment_guide.md)
- 给禹杰的模型交接：[docs/yujie_model_handoff.md](docs/yujie_model_handoff.md)
- 给泽奇的硬件交接：[docs/zeqi_hardware_handoff.md](docs/zeqi_hardware_handoff.md)

历史参考文档：

- [docs/customer_manual.md](docs/customer_manual.md)
- [docs/ai_model_reference_cn.md](docs/ai_model_reference_cn.md)
- [docs/coke_sprite_classifier_cn.md](docs/coke_sprite_classifier_cn.md)
- [docs/coco_image_validation.md](docs/coco_image_validation.md)
- [docs/yolo26_espdl_route_cn.md](docs/yolo26_espdl_route_cn.md)

## 运行模式

### SERVER 调试模式

默认启动模式。Wi-Fi、Ethernet、HTTP、mDNS 都可用，适合网页调参、查看 `/stream`、下载少量文件。

常用接口：

```text
/                       控制台
/stream                 MJPEG 调试预览
/api/status             当前状态 JSON
/api/frame.jpg          当前单帧 JPEG
/api/config             参数读取/修改
/api/recordings?limit=20
/recording/<name>.avi
/recordingmeta/<name>.jsonl
```

### FIELD 野外采集模式

目标是把算力和 I/O 尽量留给采集、raw AVI 写入和推理 metadata。进入后会关闭 HTTP、Wi-Fi、Ethernet 和 mDNS；默认需要手动复位回 SERVER。

进入 FIELD：

```powershell
curl.exe -X POST "http://p4-buoy.local/api/mode/field?confirm=FIELD"
```

FIELD 默认策略：

```text
raw AVI 目标 FPS: CONFIG_APP_FIELD_RECORDING_MAX_FPS，默认 12
推理间隔: 0 ms，尽可能连续投递最新帧
history: 关闭
JPEG quality: 70
实时 annotated AVI: 不生成
输出: raw AVI + 每帧 sidecar metadata
```

当前板端实测：TF 走 `sdmmc_4bit` 后，FIELD raw AVI 达到 `687 frames / 59.994 s = 11.45 FPS`；sidecar 中包含 `model`、`inference_ms`、`analysis_ms`、`detection_count` 等推理 metadata。

如果 mDNS 不通，用 `http://169.254.100.2/` 或 `http://192.168.4.1/` 替代。

### EXPORT 导出模式

目标是稳定、尽量快地通过有线下载 TF 卡里的文件。进入后停止采集、推理、录像和 Wi-Fi/AP，保留 Ethernet + HTTP + mDNS。

进入 EXPORT：

```powershell
curl.exe -X POST "http://p4-buoy.local/api/mode/export?confirm=EXPORT"
```

EXPORT 下建议只访问：

```text
/api/status
/api/recordings
/recording/<name>.avi
/recordingmeta/<name>.jsonl
```

`/stream`、`/validate`、dataset run 等高负载接口会被拒绝。

## 通过网线下载录像

推荐直接使用脚本。它会优先访问 `http://p4-buoy.local`，失败后自动 fallback 到 `http://169.254.100.2`，并默认先切入 EXPORT 模式：

```powershell
.\tools\download_recordings_eth.ps1 -Limit 5
```

指定直连地址：

```powershell
.\tools\download_recordings_eth.ps1 -BaseUrl http://169.254.100.2 -Limit 5
```

指定本机以太网地址：

```powershell
.\tools\download_recordings_eth.ps1 -BaseUrl http://169.254.100.2 -InterfaceAddress 169.254.12.109 -Limit 5
```

下载文件保存到：

```text
artifacts/ethernet_downloads/
```

脚本会打印每个文件的 bytes、耗时、MiB/s，并校验下载大小是否等于 `/api/recordings` 里的 `bytes`。

Range 下载验证：

```powershell
curl.exe --interface 169.254.12.109 --fail -D - -H "Range: bytes=0-1023" -o NUL http://169.254.100.2/recording/<name>.avi
```

预期返回 `206 Partial Content`。

当前板端实测：EXPORT 稳定后单文件下载约 `3.38 MiB/s`，下载大小与 `/api/recordings` metadata 一致。下载脚本会等模式切换和后台补帧完全停止后再开始传输。

## 通过 USB 直接读写 TF

USB Mass Storage 适合离线批量导出。Windows 会把整张 TF 识别为卷标 `P4_BUOY` 的可读写磁盘，可以直接打开 AVI，也可以复制、新建、重命名和删除文件。

开发板需要两根 USB 线：

```text
UART Type-C -> 电脑：供电、烧录和串口日志
USB HS OTG DEVICE -> 电脑：TF U 盘数据链路
```

Waveshare 板上两只 USB-A 下方有黄色 `HOST / DEVICE` 跳帽。断电后把跳帽移到右侧 `DEVICE` 两针，并把 USB-A 对 USB-A 数据线接到右边、靠 `SPK/MIC` 的 USB-A 口。`HOST` 位置不会在电脑上枚举为 U 盘。

USB 主机枚举后固件会自动进入 `USB_EXPORT`。也可在接线前手动触发：

```powershell
curl.exe "http://169.254.100.2/api/mode/usb?confirm=USB"
```

切换过程会依次停止新请求、相机、推理、录像、补帧和网络，闭合当前 AVI，卸载 FatFS，再把 TF 独占交给 USB。进入后板端不会再访问 `/sdcard`，恢复采集必须重启。

重要操作顺序：

1. 等待 Windows 出现 `P4_BUOY` 后再读写文件。
2. 不要在复制过程中拔线或复位。
3. 完成后在 Windows 中“安全弹出”该磁盘。
4. 等待至少 2 秒，再复位或重新上电开发板。

速度和读写完整性验收：

```powershell
.\tools\watch_usb_msc.ps1
.\tools\benchmark_usb_msc.ps1
.\tools\eject_usb_msc.ps1
```

`watch_usb_msc.ps1` 会轮询板端 USB 状态、Windows PnP 设备和卷标；`benchmark_usb_msc.ps1` 使用至少 50 MiB 文件和 `robocopy /J` 无缓冲复制，校验双向 SHA256，并验证新建、重命名和删除。合入门槛为读取 `>=6 MiB/s`、写入 `>=4 MiB/s`。

当前 v2.0.0 USB 版本使用 40 MHz SDMMC 导出，并在构建时对 `esp_tinyusb` MSC 写路径应用一个幂等补丁，把单缓冲 deferred 写入改为同步 SDMMC 写入；主固件板端验收为读取约 `7.25 MiB/s`、写入约 `5.35 MiB/s`、SHA256 一致。

常见文件目录：

```text
P4_BUOY:\esp32p4\recordings\raw_*.avi
P4_BUOY:\esp32p4\recordings\annotated_*.avi
P4_BUOY:\esp32p4\recordings\*.jsonl
```

如果 Windows 没有出现磁盘，依次检查 `DEVICE` 跳帽、右侧 USB-A 数据口、线材是否支持数据，以及 `/api/status.usb_initialized`。USB 模式不会自动格式化 TF；文件系统异常时应安全弹出后用电脑检查。

## 空闲补帧

SERVER 模式下相机处于 standby、没有 HTTP/下载/stream/dataset/实时推理且网络空闲 15 秒后，低优先级任务会把 FIELD 的 raw AVI 渐进生成同帧数、同时长的 annotated AVI：

```text
推理 stride: 8 -> 4 -> 2 -> 1
关键帧: 真实 COCO 推理
中间帧: 沿用最近结果并重新画框
无目标: 直接复用 raw JPEG
```

每次更新先写 `.part`，再通过 `.prev` 替换完整文件。任何前台请求、相机唤醒或 USB 插入都会取消当前 pass，上一份完整 annotated AVI 保持不变。进度见 `/api/status.enrichment`；sidecar 包含 `result_source`、`source_frame_index`、`inference_age_frames` 和 `pass_stride`。

## 模型现状

板端最终运行的是 `.espdl`，不是 `.pth` 或 `.onnx`。

| 方法 | 类型 | 当前用途 | 实测速度判断 |
|---|---|---|---|
| `off` | 不识别 | 只测采集、存储和传输 | 不占推理 |
| `mlp` | 轻量分类 baseline | 链路测试 | 很快，非最终模型 |
| `coco` | 官方 YOLO11n COCO INT8 | 当前演示主链路 | 约 1.5 FPS |
| `yolo11` / `yolo26` | 自训练检测实验 | 对照实验 | 约 14-16 s/帧 |

要达到 `>=5 FPS`，下一步应让禹杰优先做轻量分类模型，并交付可转 ESP-DL 的模型资产。部署细节见 [docs/model_deployment_guide.md](docs/model_deployment_guide.md)。

## 构建和烧录

推荐脚本：

```powershell
cd <project-directory>
.\build_tmp.ps1
.\flash_p4.ps1 -Port COM3
```

只烧录、不进入 monitor 时，可用构建输出里的 `flash_args`：

```powershell
idf.py -B build -p COM3 flash
```

大 app 完整写入需要几分钟，命令超时建议放到 10 分钟左右。

## 验证清单

1. `.\build_tmp.ps1` 通过。
2. 串口看到 `mDNS URL: http://p4-buoy.local/`、`Ethernet Started`、插网线后 `Ethernet Link Up`。
3. `/api/status` 包含 `hostname`、`mdns_url`、`access_urls`、`eth_url`。
4. `.\tools\download_recordings_eth.ps1 -Limit 5` 至少下载 1 个完整 `.avi`，大小与 metadata 一致。
5. Range 请求返回 `206 Partial Content`。
6. FIELD 模式下 HTTP/Wi-Fi/Ethernet 在 10 秒内停止响应。
7. FIELD 采集 2-3 分钟后复位，下载最新 raw AVI 和 sidecar，检查 raw AVI FPS、`recording_sd_errors` 和 metadata。
8. `P4_BUOY` 可读写，`benchmark_usb_msc.ps1` 的速度和 SHA256 验收通过。
9. 安全弹出并重启后 TF 正常挂载，Windows 写入文件保留，删除的视频不再出现在索引中。
10. annotated 与 raw 帧数相同、时长误差不超过一帧，metadata 的来源字段正确。
