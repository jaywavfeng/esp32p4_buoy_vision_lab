# 有线与离线传输策略

## 结论

当前 `v3.0.0` 把采集和导出分成三条清晰流程：

1. FIELD 野外采集：优先保证摄像头、TF 写入和推理，生成 raw/annotated 成对录像。
2. Ethernet/Web 下载：适合维护、少量文件下载和浏览器/API 操作。
3. USB_EXPORT 离线拷贝：适合设备回收后批量导出，电脑把整张 TF 作为 `P4_BUOY` U 盘读写。

实时图传只作为调试观察入口，不作为野外长期采集主链路。

## 访问方式

```text
Ethernet direct link:  http://169.254.100.2/
Wi-Fi AP:              http://192.168.4.1/
mDNS:                  http://p4-buoy.local/
STA/router:            read from Web or /api/status.sta_url
```

`/api/status.access_urls` 会集中返回当前可用入口。

## Ethernet 实现

当前实现：

```text
DHCP first
no DHCP after timeout -> static fallback 169.254.100.2/16
HTTP Range download supported
download chunk default 64 KiB
script prefers p4-buoy.local, then falls back to 169.254.100.2
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

推荐脚本：

```powershell
.\tools\download_recordings_eth.ps1 -BaseUrl http://169.254.100.2 -Limit 5
```

脚本流程：

```text
GET /api/status
GET /api/recordings?limit=<Limit>
download raw/annotated .avi files
compare downloaded bytes with metadata
print throughput MiB/s
```

Range 验证：

```powershell
curl.exe --noproxy "*" --fail -D - -H "Range: bytes=0-1023" -o NUL http://169.254.100.2/recording/<name>.avi
```

预期返回 `206 Partial Content`。

## FIELD 模式

进入：

```powershell
curl.exe --noproxy "*" -X POST "http://169.254.100.2/api/mode/field?confirm=FIELD"
```

行为：

```text
record raw AVI
record annotated AVI with same frame count and duration
write sidecar metadata
run selected model inference
cleanup temporary files on segment close
return to server mode by reset or maintenance recovery flow
```

性能目标：

```text
raw AVI target: CONFIG_APP_FIELD_RECORDING_MAX_FPS, default 12
acceptance target: >=8 FPS for closed segments
recording_sd_errors=0
raw/annotated frame counts match
```

FIELD 中采集和推理并发运行。推理结果缓存按实际推理速度更新，annotated 每帧使用当前 raw 图像叠加最新结果，因此不会出现 annotated 帧数少于 raw 的客户体验问题。

## USB_EXPORT 模式

自动入口：电脑通过 USB HS OTG DEVICE 枚举。维护入口：

```powershell
curl.exe --noproxy "*" "http://169.254.100.2/api/mode/usb?confirm=USB"
```

切换后：

```text
reject new storage work
camera standby
stop recording/inference/enrichment
finalize AVI
keep Web/AP/STA/Ethernet online
unmount FatFS from app
expose whole TF as writable USB MSC
remount to app after safe eject + physical unplug
```

Windows 卷标为 `P4_BUOY`。客户主流程不需要打开 Web：插入 USB 自动导出，完成后先在 Windows 安全弹出，再拔掉 USB 数据线，板端自动恢复 TF 到应用。USB 导出期间 `/api/status` 会显示 `usb_storage_owner:"usb"`。

USB 工具：

```powershell
.\tools\watch_usb_msc.ps1
.\tools\benchmark_usb_msc.ps1
.\tools\eject_usb_msc.ps1
```

验收门槛：

```text
Windows shows P4_BUOY
Web remains reachable
read >= 6 MiB/s
write >= 4 MiB/s
SHA256 matches both directions
create / rename / delete pass
safe eject + unplug remounts TF to app
next physical insert auto-exports again
```

## 三种导出方案对比

| 方案 | 适用场景 | 注意事项 |
|---|---|---|
| Web/Ethernet | 维护、少量文件下载 | 依赖 Web 在线，速度低于直接 U 盘拷贝。 |
| USB MSC | 客户离线批量导出 | 必须安全弹出后再拔线，避免文件系统损坏。 |
| 取 TF 卡 | 实验室最快拷贝 | 需要外壳允许取卡，客户现场通常不推荐。 |

## 验收命令

```powershell
curl.exe --noproxy "*" http://169.254.100.2/api/status
curl.exe --noproxy "*" http://169.254.100.2/api/recordings?limit=20
.\tools\download_recordings_eth.ps1 -BaseUrl http://169.254.100.2 -Limit 5
.\tools\benchmark_usb_msc.ps1
```
