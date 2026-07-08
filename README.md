# ESP32-P4 Buoy Vision Lab

ESP32-P4 浮标视觉原型工程，当前交付版本为 `v3.0.0`。固件面向客户使用的主流程是：Web 配置参数，野外模式采集 raw/annotated 成对录像，录像完成后通过 Web、网线或 USB U 盘导出文件。

## v3.0.0 客户交付重点

- Web 首页显示客户主流程：连接地址、客户端计数、录像片段、模型切换、实时图传、USB 状态、录像记录。
- 录像记录按时间倒序展示，一段录像一条记录，同一行提供原视频、推理视频和手动补帧。
- 录像片段时长可在 Web 设置为 `5-14400` 秒（最高 4 小时），默认 60 秒，保存到 NVS。
- FIELD 野外录像中采集和推理并发运行；raw 与 annotated 视频帧数和时长一致，标签/检测框按真实推理频率更新。
- reset、USB 切换或异常恢复后会清理未配对的半截录像，客户列表只展示 raw/annotated 成对记录。
- 默认模型为 Fish31；Web「模型切换」可保存 `fish31`、`tinycls`、`coco`，后续 FIELD 和手动补帧使用保存后的模型。
- 手动补帧只在用户点击某条录像记录后执行，后台不再自动扫描补帧。
- 「清空录像记录」会清理 `recordings` 目录下的录像、sidecar、索引和临时残留。
- USB 插入后自动导出整张 TF 为卷标 `P4_BUOY` 的 U 盘；Web 服务不关闭，安全弹出并拔线后自动恢复 TF 给板端。
- 「客户端」按最近访问 Web 的远端 IP 计数，网线、热点和路由器访问都会阻止自动进入野外录像。

## 当前默认模型

| 方法 | 类型 | 当前用途 |
|---|---|---|
| `fish31` | 31 类鱼类/水下背景分类 | 默认模型，FIELD 和补帧主链路 |
| `tinycls` | Marine 6 类轻量分类 | 对照验证 |
| `coco` | COCO YOLO11n 通用检测 | 检测演示和对照 |

板端最终运行 `.espdl` 或组件内置 ESP-DL 模型，不直接运行 `.pth`、`.pt` 或 `.onnx`。模型部署细节见 [docs/model_deployment_guide.md](docs/model_deployment_guide.md)，板端基准记录见 [docs/model_benchmark_results_cn.md](docs/model_benchmark_results_cn.md)。

## 连接方式

| 场景 | 地址 | 说明 |
|---|---|---|
| 网线直连 | `http://169.254.100.2/` | 最稳定的维护入口。电脑与板端 RJ45 直连，浏览器打开该地址。 |
| Wi-Fi 热点 | `http://192.168.4.1/` | 连接热点 `P4_Buoy_Lab`，默认密码 `change-me-please`。 |
| 路由器 STA | 页面显示的 STA 地址 | 先用网线或热点进入 Web，在「用户设置」填写路由器 SSID/密码，保存后从「连接地址」读取路由器分配的 IP。 |
| mDNS | `http://p4-buoy.local/` | 作为便捷别名提供；Windows 上不稳定时以页面显示的 IP 为准。 |
| USB U 盘 | `P4_BUOY` | 插入 USB HS OTG DEVICE 数据线后，Windows 会出现卷标 `P4_BUOY` 的可读写磁盘。 |

路由器连接不需要重新烧录。密码只用于板端连接路由器，不会在 Web 状态或 `/api/config` 明文回显。

常用 API 检查：

```powershell
curl.exe --noproxy "*" http://169.254.100.2/api/status
curl.exe --noproxy "*" http://169.254.100.2/api/config
curl.exe --noproxy "*" http://169.254.100.2/api/recordings?limit=20
```

如果电脑有多块网卡，可先查看本机 APIPA 地址，再指定接口：

```powershell
Get-NetIPAddress -AddressFamily IPv4 | Where-Object { $_.IPAddress -like "169.254.*" }
curl.exe --interface 169.254.x.x --noproxy "*" http://169.254.100.2/api/status
```

## 当前硬件

| 模块 | 当前使用情况 |
|---|---|
| 主控 | ESP32-P4 |
| 开发板 | Waveshare ESP32-P4-WIFI6-DEV-KIT-A |
| 摄像头 | OV5647 / Raspberry Pi Camera(B) Rev2.0，MIPI-CSI |
| Wi-Fi | 板载 ESP32-C6，ESP-Hosted SDIO |
| Ethernet | RJ45，ESP32-P4 EMAC + IP101GRI PHY |
| TF 卡 | raw/annotated AVI、metadata 和索引存储 |
| USB | UART Type-C 用于供电/烧录/日志；USB HS OTG DEVICE 用于整张 TF U 盘导出 |

硬件交接见 [docs/zeqi_hardware_handoff.md](docs/zeqi_hardware_handoff.md)。

## Web 客户流程

1. 打开 Web 首页，确认「连接地址」里 AP、STA、ETH 三类地址和「客户端」计数。
2. 在「用户设置」里调整录像片段时长、自动进入野外录像等待时间、网络模式和路由器信息。
3. 点击「模型切换」，保存 Fish31/TinyCNN/COCO；需要演示时从同一面板进入验证页。
4. 需要临时观察画面时点击「打开实时图传」，使用结束后关闭图传。
5. 点击「立即进入野外录像」或等待无客户端倒计时结束后自动进入 FIELD。
6. 采集后回到 Web，录像记录按一段一条显示，可下载原视频、推理视频，或对某条记录执行手动补帧。
7. 确认已导出需要的视频后，可点击「清空录像记录」清理 `recordings` 目录。

## 运行模式

### SERVER

默认模式。Web、API、AP/STA、Ethernet 和 mDNS 可用，用于设置、预览、下载和维护。

常用接口：

```text
/                       Web 控制台
/stream                 MJPEG 实时图传
/validate               模型演示验证页
/api/status             当前状态
/api/config             客户配置读取/保存
/api/recordings         录像记录
/recording/<name>.avi   raw/annotated 视频下载
```

### FIELD

野外录像模式。进入后固件把资源优先留给摄像头、TF 写入和模型推理。每个片段闭合后生成：

```text
raw_*.avi
annotated_*.avi
raw_*.jsonl / annotated_*.jsonl
```

FIELD 中 raw 与 annotated 每个采集帧一一对应。推理任务尽可能处理最新帧，但不会阻塞 raw 采集；没有新推理结果的帧沿用最近一次标签或检测框。
如果 reset 或 USB 切换发生在片段未闭合时，启动恢复会丢弃 `.part` 和未配对的半截记录，避免客户页面出现只有原视频、没有推理视频的灰色记录。

### USB_EXPORT

插入 USB HS OTG DEVICE 数据线后自动进入。固件会停止相机/录像/补帧，闭合 AVI，卸载板端 FatFS，然后把整张 TF 交给电脑作为 `P4_BUOY` U 盘。Web 服务保持在线，但存储列表和下载接口会提示 TF 被 USB 占用。安全弹出并拔线后，板端自动重新挂载 TF。

## 有线与 USB 导出

网线下载适合维护和少量文件：

```powershell
.\tools\download_recordings_eth.ps1 -BaseUrl http://169.254.100.2 -Limit 5
```

USB 导出适合客户离线批量拷贝：

```text
1. 插入 USB HS OTG DEVICE 数据线。
2. 等待 Windows 出现 P4_BUOY。
3. 打开 P4_BUOY:\esp32p4\recordings\ 复制 raw_*.avi 和 annotated_*.avi。
4. Windows 安全弹出。
5. 拔掉 USB 数据线，等待板端自动恢复 TF。
```

USB 验收辅助脚本：

```powershell
.\tools\watch_usb_msc.ps1
.\tools\benchmark_usb_msc.ps1
.\tools\eject_usb_msc.ps1
```

## 构建和烧录

客户手册不包含烧录步骤；以下仅供开发维护使用。

```powershell
.\build_tmp.ps1
.\flash_p4.ps1 -Port COM3
```

也可以使用构建输出中的 esptool 命令直接写入 `bootloader.bin`、`partition-table.bin` 和应用镜像。

## 文档入口

- 客户 Markdown 手册：[docs/customer_manual.md](docs/customer_manual.md)
- 客户 Word 操作指南：[docs/p4_buoy_user_operation_guide.docx](docs/p4_buoy_user_operation_guide.docx)
- 技术实现说明：[docs/developer_guide.md](docs/developer_guide.md)
- 有线/离线传输策略：[docs/wired_transfer_plan.md](docs/wired_transfer_plan.md)
- 模型部署说明：[docs/model_deployment_guide.md](docs/model_deployment_guide.md)
- 模型板端基准：[docs/model_benchmark_results_cn.md](docs/model_benchmark_results_cn.md)
- COCO 验证说明：[docs/coco_image_validation.md](docs/coco_image_validation.md)

## v3.0.0 验收清单

1. `.\build_tmp.ps1` 构建通过。
2. 烧录后 `/api/status` 和 `/api/config` 可访问。
3. 录像片段时长支持 `5-14400` 秒（最高 4 小时），保存后重启仍保留。
4. 实时图传可以唤醒摄像头并从 `/stream` 输出 MJPEG。
5. 模型切换 Fish31/TinyCNN/COCO 后重启仍保留，FIELD 使用保存后的模型。
6. FIELD 采集至少 1-2 段后，Web 记录中每条都有 raw 与 annotated 下载。
7. raw 与 annotated 帧数一致，推理视频左上角标签或检测框正常。
8. 手动补帧进度能到满帧，完成后推理视频可下载。
9. 清空录像记录后 `recordings` 目录录像和索引被清理，Web 列表为空。
10. 插入 USB 后 Windows 出现 `P4_BUOY`，Web 保持在线；安全弹出并拔线后 TF 自动恢复。

本轮板端实测记录（2026-07-08）：`.\build_tmp.ps1` 通过；COM3 直接 esptool 写入并校验通过；首页中文、客户端计数、模型切换、`5-14400` 秒片段上限显示正常；实时图传从待机唤醒后 `/stream` 连续输出约 2.58 MB MJPEG；清空录像记录删除 7 个录像相关文件；模型 API 已验证 `tinycls -> fish31` 切换；5 秒片段 FIELD 生成 2 条 raw/annotated 成对记录，帧数为 24/24、23/23；手动补帧完成 24/24 帧，覆盖率 100%；改回 60 秒后历史整理将 2 段合并为 1 条 47/47 帧成对记录；USB 插入后 Windows 识别 `E: P4_BUOY`，Web 保持在线，`usb_storage_owner=usb`，并已从 `P4_BUOY:\esp32p4\recordings\` 成功读取和复制 AVI；安全弹出/拔线恢复流程已按客户随插随拔口径整理，Web 侧保留 TF 恢复入口作为兜底。
