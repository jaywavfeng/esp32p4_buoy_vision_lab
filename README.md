# ESP32-P4 Buoy Vision Lab

ESP32-P4 浮标视觉原型工程，当前交付版本为 `v3.1.0`。固件面向客户使用的主流程是：Web 配置参数，野外模式采集 raw/annotated 成对录像，录像完成后通过 Web、网线或 USB U 盘导出文件。

## v3.1.0 客户交付重点

- Web 首页显示客户主流程：连接地址、客户端计数、录像片段、模型切换、实时图传、USB 状态、录像记录。
- 录像记录按时间倒序展示，一段录像一条记录，同一行提供原视频、推理视频和手动补帧。
- Web「用户设置」可修改自动进入野外采集开关、无连接倒计时 `10-86400` 秒和录像片段时长 `5-14400` 秒（最高 4 小时）；三项一起保存到 NVS，软件重启后继续生效。片段时长只影响当前及后续采集，历史录像保持原样。
- 保存设置后，页面会回显设备实际采用的自动采集、倒计时和片段时长；输入不合法或持久化失败时保留原配置，不显示虚假的“已生效”。
- 自动采集倒计时会在 Web 客户端、图传、下载、模型验证、数据集任务、推理或存储维护期间暂停，并在页面说明暂停原因；忙碌状态结束后从完整倒计时重新计时，避免用户操作中途被切换到 FIELD。
- FIELD 野外录像中采集和推理并发运行；raw 与 annotated 视频帧数和时长一致，标签/检测框按真实推理频率更新。
- reset、USB 切换或异常恢复后会清理未配对的半截录像，客户列表只展示 raw/annotated 成对记录。
- 默认模型为 Fish31；Web「模型切换」可保存 `fish31`、`tinycls`、`coco`，后续 FIELD 和手动补帧使用保存后的模型。
- 手动补帧只在用户点击某条录像记录后执行，后台不再自动扫描补帧。
- 「清空录像记录」会清理 `recordings` 目录下的录像、sidecar、索引和临时残留。
- USB 插入后自动导出整张 TF 为卷标 `P4_BUOY` 的 U 盘，Web 服务不关闭。Windows 安全弹出、USB host detach 或物理拔线会触发自动恢复 TF；若一直插着 USB 想手动切回 Web 存储，或自动恢复卡住，可点击「USB 恢复存储」兜底。
- 「客户端」按最近访问 Web 的远端 IP 计数，网线、热点和路由器访问都会阻止自动进入野外录像。
- TF 写入异常会停止后续录像写入并给出可操作提示；用户可从 Web 执行真实写入/同步/读回重试，普通恢复失败不要求重启设备。
- 单图模型验证使用异步任务，提交后页面持续显示排队、运行或失败状态，推理期间其它 Web 功能仍可响应。

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
| USB U 盘 | `P4_BUOY` | 用支持数据的 A-to-C 或 C-to-C 线连接 J15 Type-C `DEVICE` 口后，Windows 会出现卷标 `P4_BUOY` 的可读写磁盘；不要使用叠层 Type-A `HOST` 口或 A-to-A 线连接电脑。 |

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

## 管理接口安全边界

- `GET /api/config` 只读取配置；任何带修改参数的 GET 请求返回 `405 Method Not Allowed`。状态变更必须使用 Web 页面或相应的 `POST`/`DELETE` 接口，避免浏览器预取、缓存或链接点击意外修改设备。
- 路由器密码由 Web 以 URL-encoded POST body 提交。服务端拒绝 URL 中的密码参数，且 `/api/status`、`/api/config` 只返回“是否已设置”，不返回明文密码。
- 管理员认证明确延期到后续版本，当前版本不能视为已实现登录或权限隔离。认证完成前，请仅在受信任的局域网或设备专网使用，不要把管理端口直接映射到互联网。

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
2. 在「用户设置」里调整自动采集开关、无连接进入采集倒计时、录像片段时长、网络模式和路由器信息，点击「保存设置」。以页面返回的“已生效”数值为准。
3. 点击「模型切换」，保存 Fish31/TinyCNN/COCO；需要演示时从同一面板进入验证页。
4. 需要临时观察画面时点击「打开实时图传」，使用结束后关闭图传。
5. 点击「立即进入野外录像」或等待无客户端倒计时结束后自动进入 FIELD。
6. 采集后回到 Web，录像记录按一段一条显示，可下载原视频、推理视频，或对某条记录执行手动补帧。
7. 确认已导出需要的视频后，可点击「清空录像记录」清理 `recordings` 目录。

## TF 故障恢复

首页只有在 TF 完成真实写入、`fsync` 和重开读回验证后，才显示“写入验证通过”。若显示“检测到写入故障”或“尚未通过写入验证”，设备会停止新增录像，避免继续生成损坏文件。

普通用户按以下顺序处理，无需先重启：

1. 确认 TF 已插稳；若错误反复出现，先更换一张已知良好的 TF 卡。
2. 在 Web「用户设置」点击「检查并重试 TF」。设备会短暂停止 HTTP 来安全重新挂载，Wi-Fi 保持连接，页面会自动继续查询状态。
3. 等待页面显示“TF 写入验证已通过，可以稳定录像”。维护过程中不要重复点击，也不要立即断电。
4. 若仍失败，保留页面状态与串口日志，检查卡片、供电和 SDMMC 信号；Web 仍可用于诊断，普通故障不需要通过重启掩盖问题。

开发维护接口还提供重新挂载与格式化。它们只在 SERVER 模式受理，会先暂停采集、录像、下载和后台存储任务，操作结束后自动恢复相机、网络及 Web；页面可能短暂重连，但不要求重启。

```text
POST /api/storage/retry?confirm=RETRY
POST /api/storage/remount?confirm=REMOUNT
POST /api/storage/format?confirm=FORMAT
```

`format` 会删除 TF 上的全部数据，必须先完成备份，不能作为日常重试手段。并发维护、FIELD、EXPORT 或 USB 占用时接口会返回 409/503 和下一步操作提示，等待当前流程结束后再重试。

## 模型验证任务

验证页不会让一次长推理独占 HTTP 请求。`POST /api/validate/run` 返回 `202 Accepted` 和任务 ID，页面再轮询 `/api/validate/status?id=<任务ID>`，状态为 `queued`、`running`、`done` 或 `failed`。活动任务会暂停自动 FIELD 倒计时；网络短暂不稳定时继续查询即可，不需要重复提交或重启设备。

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
/api/validate/status    异步模型验证任务状态
/api/storage/retry      TF 写入验证与恢复
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

插入 USB HS OTG DEVICE 数据线后自动进入。固件会停止相机/录像/补帧，闭合 AVI，卸载板端 FatFS，然后把整张 TF 交给电脑作为 `P4_BUOY` U 盘。Web 服务保持在线，但存储列表和下载接口会提示 TF 被 USB 占用。

Windows 安全弹出、TinyUSB `DETACHED` 或 USB host inactive 后，固件会排队自动恢复 TF：先软断开 MSC、删除 TinyUSB storage、释放 USB SDMMC，再把 TF 重新挂载给应用并执行写入、同步和读回验证。若 USB 线仍插着，页面也允许点击「USB 恢复存储」手动切回；恢复成功后会抑制本次仍插着的线立刻重新自动导出，下一次重新插入 USB 仍会自动出现 U 盘。若恢复失败，Web 保持在线并显示错误，用户可拔线后再次点击恢复或重试 TF。

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
5. 设备通常会在安全弹出后 1-5 秒自动恢复 TF；若仍显示 USB 占用，拔掉 USB 数据线或在 Web 点击「USB 恢复存储」。
6. 等待页面显示 TF 已恢复并通过写入验证，再继续录像或下载。
```

这里的 `DEVICE` 指开发板 J15 USB 2.0 Type-C 设备口。J18 叠层 Type-A `HOST` 口用于开发板连接 USB 外设，不是给电脑枚举 U 盘的端口；HOST/DEVICE 也不是需要应用设置电位的 GPIO。

USB 验收辅助脚本：

```powershell
.\tools\watch_usb_msc.ps1
.\tools\benchmark_usb_msc.ps1
.\tools\eject_usb_msc.ps1
```

## 构建和烧录

客户手册不包含烧录步骤；以下仅供开发维护使用。

```powershell
.\build_tmp.ps1 -Profile rev1 -Clean
.\build_rev31.ps1 -Clean
.\flash_p4.ps1 -Profile rev31 -Port COM6 -SkipBuild
```

构建脚本会把 ESP-IDF build 输出和 managed components 重定向到 `C:\Espressif\idf_build_outputs\...` 与 `C:\Espressif\idf_project_managed_components\...`，避免中文路径、深层 GitHub clone 路径和 Windows 路径长度导致异地构建失败。`flash_p4.ps1` 按所选 profile 使用对应的短路径构建目录，避免误烧默认构建目录；脚本只写 bootloader、partition table 和应用镜像，不执行 `erase-flash`，不会主动清空 NVS 或 TF。端口号按实际设备修改。也可以使用构建输出中的 esptool 命令直接写入相同镜像。

CMake 配置阶段会运行 `tools/apply_component_patches.py`。针对 ESP-IDF 6.0.1，项目补丁修复 FAT 格式化后重新挂载失败却返回成功的问题，并补齐 Hosted SDMMC slot 清理、移除槽后的当前槽归还与空槽 ISR 防护。USB TF 不再固定使用 `4-bit/40 MHz`，而是沿用板端刚通过真实写入、同步、重开和读回验证的稳定档位。补丁按版本和源码片段严格匹配；环境不符合预期时构建会停止，禁止静默使用未知 IDF 行为。

## 文档入口

- 客户 Markdown 手册：[docs/customer_manual.md](docs/customer_manual.md)
- 客户 Word 操作指南：[docs/p4_buoy_user_operation_guide.docx](docs/p4_buoy_user_operation_guide.docx)
- 技术实现说明：[docs/developer_guide.md](docs/developer_guide.md)
- 有线/离线传输策略：[docs/wired_transfer_plan.md](docs/wired_transfer_plan.md)
- 模型部署说明：[docs/model_deployment_guide.md](docs/model_deployment_guide.md)
- 模型板端基准：[docs/model_benchmark_results_cn.md](docs/model_benchmark_results_cn.md)
- COCO 验证说明：[docs/coco_image_validation.md](docs/coco_image_validation.md)

## v3.1.0 验收清单

1. `.\build_tmp.ps1` 构建通过。
2. 烧录后 `/api/status` 和 `/api/config` 可访问。
3. Web 可分别开启/关闭自动采集，设置 `10-86400` 秒无连接倒计时和 `5-14400` 秒录像片段时长；保存响应与状态页显示设备实际值，软件重启后仍保留，修改片段时长不重写历史录像。
4. 实时图传可以唤醒摄像头并从 `/stream` 输出 MJPEG。
5. 模型切换 Fish31/TinyCNN/COCO 后重启仍保留，FIELD 使用保存后的模型。
6. FIELD 采集至少 1-2 段后，Web 记录中每条都有 raw 与 annotated 下载。
7. raw 与 annotated 帧数一致，推理视频左上角标签或检测框正常。
8. 手动补帧进度能到满帧，完成后推理视频可下载。
9. 清空录像记录后 `recordings` 目录录像和索引被清理，Web 列表为空。
10. 插入 USB 后 Windows 出现 `P4_BUOY`，Web 保持在线；Windows 安全弹出、USB host detach 或拔线后 TF 自动恢复给应用并通过写读验证。若 USB 线一直插着，也可点击 Web「USB 恢复存储」手动切回，下一次重新插入仍会自动导出。
11. 单图验证 POST 返回任务 ID，状态可从 queued/running 进入 done 或 failed；任务执行时 Web 可继续读取状态，自动采集倒计时保持暂停，任务结束后从完整时长重新计时。
12. TF 重试、重新挂载和格式化过程中不会并发写卡；成功后 Web、相机和原运行设置恢复，失败时给出可重试状态且不主动重启。
13. 非法配置值不修改运行值或 NVS；修改型 GET 返回 405，Wi-Fi 密码出现在 URL 时被拒绝。

2026-07-17 v3.1.0 交付验证记录：从 GitHub `v3.1.0` tag 干净 clone 后运行 `.\build_rev31.ps1 -Clean` 构建通过；随后用 `.\flash_p4.ps1 -Profile rev31 -Port COM6 -SkipBuild` 烧录到真实板端并完成 esptool 校验。烧录后通过 `http://192.168.1.80/api/status` 验证 Web/API 在线，状态包含 `app_mode=server`、`recognition_method=fish31`、`eth_ip=169.254.100.2`、`usb_storage_owner=app`、`sd_mounted=true`、`tf_ready=true`、`storage_acceptance_ok=true`、`storage_write_verified=true`、`sd_mount_mode=sdmmc_1bit`、`recording_segment_ms=1800000` 和 `recording_segment_max_ms=14400000`。

历史功能复测记录见 [TEST_ISSUES.md](TEST_ISSUES.md)。该文件保留 2026-07-17 早期 `3.0.1` 板端功能复测事实，用于追溯 USB/录像/图传问题修复过程；当前交付口径以本 README、[docs/developer_guide.md](docs/developer_guide.md) 和 `v3.1.0` tag 为准。
