# 有线与离线传输策略

## 结论

在“视频和推理结果帧率尽可能高，其他功能从简，离线传输效率高”的需求下，实时图传不应该是主目标。推荐把流程拆成两个阶段：

1. **FIELD 野外采集**：关闭 HTTP/Wi-Fi/Ethernet/mDNS，把资源留给采集、TF 写入和推理 metadata。
2. **EXPORT 导出**：停止采集和推理，只保留 Ethernet + HTTP + mDNS 下载文件。
3. **USB_EXPORT 离线拷贝**：电脑独占整张 TF，以可读写 U 盘方式直接打开和复制文件。

RJ45 Ethernet 适合密封、长线、PoE、维护和浏览器下载；USB MSC 适合设备回收后近距离高速批量导出；若 TF 卡能取出，高速读卡器仍是最直接的方案。

## 访问方式

首选：

```text
http://p4-buoy.local/
```

这是 mDNS 固定网址，固件注册：

```text
hostname: p4-buoy
service: _http._tcp:80
```

兜底 IP：

```text
Wi-Fi AP:              http://192.168.4.1/
Ethernet direct link:  http://169.254.100.2/
STA/router:            read from /api/status.sta_url or serial log
```

`/api/status.access_urls` 会集中返回当前可用入口。

## 当前 RJ45 实现

当前分支实现：

```text
Ethernet + HTTP file download
DHCP first
no DHCP after timeout -> static fallback 169.254.100.2/16
Range download supported
download chunk default 64 KiB
script prefers p4-buoy.local, falls back to 169.254.100.2
current accepted storage path: tf_sdmmc / sdmmc_4bit
```

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

## 下载录像

推荐：

```powershell
.\tools\download_recordings_eth.ps1 -Limit 5
```

脚本默认流程：

```text
1. try http://p4-buoy.local
2. POST /api/mode/export?confirm=EXPORT
3. GET /api/status
4. GET /api/recordings?limit=<Limit>
5. download .avi files
6. compare bytes with metadata
7. print throughput MiB/s
8. if mDNS fails, retry http://169.254.100.2
```

手动指定直连 IP：

```powershell
.\tools\download_recordings_eth.ps1 -BaseUrl http://169.254.100.2 -Limit 5
```

手动绑定本机以太网 APIPA 地址：

```powershell
.\tools\download_recordings_eth.ps1 -BaseUrl http://169.254.100.2 -InterfaceAddress 169.254.12.109 -Limit 5
```

## FIELD 模式

进入：

```powershell
curl.exe -X POST "http://p4-buoy.local/api/mode/field?confirm=FIELD"
```

行为：

```text
close HTTP
close mDNS
stop Wi-Fi/AP
stop Ethernet
record raw AVI
write sidecar metadata
run COCO inference metadata path
do not generate realtime annotated AVI
return to server mode by reset
```

性能目标：

```text
raw AVI target: CONFIG_APP_FIELD_RECORDING_MAX_FPS, default 12
acceptance target: >=8 FPS
recording_sd_errors=0
recording_dropped should not keep increasing
board validation: 687 frames / 59.994 s = 11.45 FPS, with sidecar inference metadata
```

为什么关闭网络：HTTP、MJPEG、网页客户端、mDNS 和 Wi-Fi/Ethernet 都会消耗 CPU、PSRAM、JPEG buffer、任务调度和 I/O 资源。野外采集时最重要的是视频写入和推理 metadata。

## EXPORT 模式

进入：

```powershell
curl.exe -X POST "http://p4-buoy.local/api/mode/export?confirm=EXPORT"
```

行为：

```text
stop camera capture
stop vision/inference/recording
finalize current AVI segment
stop Wi-Fi/AP
keep Ethernet + HTTP + mDNS
reject high-load endpoints such as /stream, /validate, dataset run
```

EXPORT 模式下只建议访问：

```text
/api/status
/api/recordings
/recording/<name>.avi
/recordingmeta/<name>.jsonl
```

## 三种导出方案对比

| 方案 | 适用场景 | 需要新增 |
|---|---|---|
| 取 TF 卡 + 读卡器 | 当前开发最快验证、外壳允许取卡 | 高速 USB 3.0 TF 读卡器；工业级/高耐久 TF 卡 |
| USB MSC | 最终封装后不能取 TF，设备回收后像 U 盘一样复制 | USB HS DEVICE 数据线；当前固件已实现整卡可读写和文件系统互斥 |
| RJ45 Ethernet | 长线、PoE、密封接口、维护和浏览器/API 下载 | Cat5e/Cat6 网线；电脑无网口时 USB-RJ45 网卡；封装时防水 RJ45/尾线 |

## 是否还需要买线

当前你已经插上网线并完成直连测试，继续 RJ45 方案时建议准备：

```text
Cat5e 或 Cat6 普通网线
电脑没有 RJ45 口时：USB 3.0 转千兆 RJ45 网卡
需要封装时：防水 RJ45 面板口或防水网线尾线
若考虑一根线供电通信：PoE injector/PoE switch + 板端 PoE PD 方案
```

当前 Waveshare 开发板的 HS OTG DEVICE 实际位于 USB-A 口，因此实验阶段使用 USB-A 对 USB-A **数据线**，并必须断电把黄色跳帽从 `HOST` 移到 `DEVICE`。这是开发板验证接法；定制板不建议做 USB-A device，应改成符合规范的 USB-C device 口并正确处理 CC、VBUS 检测、ESD 和阻抗。

## USB_EXPORT 模式

自动入口：电脑通过 HS OTG DEVICE 完成枚举。手动入口：

```powershell
curl.exe "http://169.254.100.2/api/mode/usb?confirm=USB"
```

切换后：

```text
reject new HTTP storage work
camera standby
drain inference/history/recording queues
finalize AVI
stop HTTP/mDNS/Wi-Fi/Ethernet
unmount FatFS and deinit ESP-Hosted
reinitialize TF as SDMMC 4-bit
expose the whole card as writable USB MSC
never remount to the app before reboot
```

Windows 卷标为 `P4_BUOY`。完成操作后必须安全弹出、等待至少 2 秒，再重启开发板。速度与完整性命令：

```powershell
.\tools\watch_usb_msc.ps1
.\tools\benchmark_usb_msc.ps1
```

验收门槛：读取 `>=6 MiB/s`、写入 `>=4 MiB/s`、双向 SHA256 一致，新建/重命名/删除成功。

v2.0.0 USB 导出默认使用 SDMMC 4-bit 40 MHz，并通过构建期补丁把 `esp_tinyusb` MSC 写路径改为同步 SDMMC 写入。该补丁不提交 `managed_components` 生成物，只在本地构建时幂等应用；主固件板端验收读取约 `7.25 MiB/s`、写入约 `5.35 MiB/s`。

## 验收命令

固定网址：

```powershell
curl.exe http://p4-buoy.local/api/status
```

直连 IP：

```powershell
ping 169.254.100.2
curl.exe http://169.254.100.2/api/status
curl.exe http://169.254.100.2/api/recordings?limit=20
```

Range：

```powershell
curl.exe --interface 169.254.12.109 --fail -D - -H "Range: bytes=0-1023" -o NUL http://169.254.100.2/recording/<name>.avi
```

下载：

```powershell
.\tools\download_recordings_eth.ps1 -Limit 5
```

USB：

```powershell
.\tools\benchmark_usb_msc.ps1
```

验收标准：

```text
at least one full .avi downloaded
downloaded bytes == metadata bytes
Range returns 206 Partial Content
status remains reachable during download
throughput target > previous 0.78 MiB/s baseline, accept >=1.0 MiB/s before merging
board validation: 5 AVI downloads averaged about 3.08 MiB/s; file sizes matched metadata
```
