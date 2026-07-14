# ESP32-P4 新版板测试问题记录

## 测试基线

- 测试日期：2026-07-14（Asia/Shanghai）
- Git 提交：`2c725fdeab6052c9ea46ced538c33296723a5df0`
- 目标芯片：ESP32-P4 rev v3.1，16 MB Flash，32 MB PSRAM
- ESP-IDF：`v6.0.1-dirty`
- 固件版本：`3.0.0`
- 应用镜像大小：9,979,008 字节
- 应用镜像 SHA-256：`FB76F2ECFA0402E0D51C3A8658545B37871C8A539C7CDA82A99ACBB56FA41874`
- Flash 回读校验：从 `0x10000` 回读 9,979,008 字节，SHA-256 与本地镜像完全一致
- 测试网络：电脑保持连接 2.4 GHz 路由器；板端 STA 为 `192.168.1.80`，Ethernet 静态回退为 `169.254.100.2`
- TF 卡：`TW000`，SDHC，约 119,276 MB，启动时识别为 SDMMC 4-bit/20 MHz

板端热点按用户要求不连接、不验收，不属于缺陷。Wi-Fi 密码未写入本文档。

## ISSUE-001：TF 写入持续失败，FIELD 录像和后续数据链路整体不可用

- 严重度：Critical / P0
- 固件 SHA-256：`FB76F2ECFA0402E0D51C3A8658545B37871C8A539C7CDA82A99ACBB56FA41874`
- 前置条件：TF 卡已插入；SERVER 状态 API 显示 `sd_mounted=true`、`tf_ready=true`、`storage_acceptance_ok=true`
- 复现率：FIELD 写自检 3/3（Fish31、TinyCNN、COCO 各一次）；SERVER/数据集写入也多次出现

### 复现步骤

1. 重启进入 SERVER，确认 TF 显示已挂载。
2. 设置片段时长为 5 秒并关闭自动采集。
3. 分别选择 Fish31、TinyCNN、COCO。
4. 调用 `POST /api/mode/field?confirm=FIELD`。
5. 通过串口观察网络关闭后的 TF 写自检和录像初始化。

### 预期

板端关闭网络、进入 FIELD，持续生成成对的 raw/annotated AVI 和 sidecar；TF 写错误为 0。

### 实际

三个模型均在写入第一字节前失败，无法开始正常录像：

```text
TF post-mount: directories ready
sdmmc_write_sectors_dma: sdmmc_send_cmd returned 0x109, status 0xe00
diskio_sdmmc: sdmmc_write_blocks failed (0x109)
TF write self-test short write: offset=0 result=-1 errno=5
storage service: error - offline TF capture TF mount failed: ESP_FAIL
```

此外还观察到：

- 启动阶段设置 TF 卷标时出现同一 `0x109` 写失败。
- 一次较早的 FIELD 运行中，AVI 写入固定停在 382,976 字节，之后持续 `short write ... errno=5`，`errors` 至少累计到 256。
- Fish31 内置 16 帧数据集 API 报告 `processed=16`、`ok_frames=16`、`failed_frames=0`，但下载到的结果 JSONL 为 0 字节；汇总 JSON 和 SVG 偶尔仍可写入。

### 影响

核心 FIELD 录像不可用，并直接阻断以下验收：AVI 帧率/可播放性、raw/annotated 配对、完整下载与 Range、三模型补帧、短片段合并、录像中复位/USB 接管恢复，以及不少于 10 分钟的 Fish31 FIELD 稳定性测试。API 的 `tf_ready`/`storage_acceptance_ok` 还会在实际不可写时给出误导性的健康状态。

### 临时规避与建议

没有可靠的软件规避。重启只能恢复 SERVER 访问，不能保证写入。建议先用另一张已知良好的 TF 卡复测，以区分卡片/信号完整性与驱动资源问题；随后检查 SDMMC 供电、走线、总线宽度/频率和写后自检，并让状态 API 以实际写自检结果决定 `tf_ready`。

## ISSUE-002：相机唤醒可能因 JPEG 引擎创建失败，失败后资源未释放且无法重试恢复

- 严重度：High / P1
- 固件 SHA-256：`FB76F2ECFA0402E0D51C3A8658545B37871C8A539C7CDA82A99ACBB56FA41874`
- 前置条件：SERVER、Fish31、相机处于 standby
- 复现率：多个重启轮次中出现 2 次；进入该故障后再次唤醒 2/2 失败。另有一次重启后唤醒成功，说明具有间歇性

### 复现步骤

1. 重启并等待 SERVER/STA 就绪。
2. 调用 `GET /api/power?cmd=wake`。
3. 等待 5 至 10 秒，读取 `/api/status`。
4. 若首次失败，再次调用 wake。

### 预期

OV5647 与硬件 JPEG 初始化成功，`power_mode=running`、`camera_ready=true`，快照和 MJPEG 有帧；失败后重试应能完整清理并重新初始化。

### 实际

受控复现中 OV5647 已检测到，但 JPEG 编码引擎返回内存不足；随后的清理未注销 `video20`，所有重试均失败：

```text
ov5647: Detected Camera sensor PID=0x5647
failed to create jpeg encoder engine
failed to init JPEG encoder
resetting esp-video after camera open failure ret=ESP_ERR_NO_MEM
camera start failed: ESP_ERR_NO_MEM
esp_video: Failed to register video VFS dev name=video20
Failed to create hardware ISP video device
camera start failed: ESP_FAIL
```

API 最终为 `power_mode=error`、`camera_ready=false`、`camera_error="video init failed: ESP_FAIL"`；快照返回 `no frame available`，MJPEG 为 0 帧。

### 影响

图传、实时推理、历史记录和 FIELD 采集均被阻断。仅查看总 heap/PSRAM 仍有约 33 MB，无法提前判断该硬件 JPEG/内部内存资源失败。

### 临时规避与建议

重启后重试有时可恢复，但不可靠。建议核对硬件 JPEG 引擎和内部 DMA 内存的分配顺序；任何 `camera_open` 失败路径都应对称销毁 JPEG、ISP、video VFS 和设备句柄，然后加入“失败后再次 wake”回归测试。

## ISSUE-003：从 USB_EXPORT 手动恢复 TF 失败，进入挂载重试风暴并最终重启

- 严重度：High / P1
- 固件 SHA-256：`FB76F2ECFA0402E0D51C3A8658545B37871C8A539C7CDA82A99ACBB56FA41874`
- 前置条件：SERVER、TF 已挂载；本轮未连接 USB1 主机，使用受支持的手动 USB API 测试所有权切换
- 复现率：1/1

### 复现步骤

1. 调用 `POST /api/mode/usb?confirm=USB`。
2. 确认 API 显示 `app_mode=usb_export`、`usb_storage_owner=usb`、`sd_mounted=false`，Web 仍在线。
3. 调用 `POST /api/mode/usb/restore?confirm=RESTORE`。
4. 等待并观察串口及 `/api/status`。

### 预期

USB MSC 介质被安全撤下，TF 重新由应用挂载；状态回到 SERVER、`usb_storage_owner=app`、`sd_mounted=true`，Web 始终在线。

### 实际

USB 交接成功，但恢复阶段失败：

```text
USB MSC DMA reserve allocation failed (25600 bytes)
Writable TF card detached from USB MSC storage
TF mount failed on sdmmc_4bit: ESP_ERR_NO_MEM
slot is not available
TF remount after USB detach attempt ... failed: ESP_ERR_NO_MEM
USB detached; TF remount failed after retries: ESP_ERR_NO_MEM; rebooting
```

恢复 API 返回已受理，但随后状态为 `app_mode=server`、`usb_storage_owner=none`、`sd_mounted=false`，存储接口返回 409；固件高频尝试 SDMMC 4-bit、1-bit 和 SDSPI，最终主动重启。

### 影响

正常的安全弹出/拔线恢复路径可能使 TF 在本次运行中永久不可用，并产生大量错误日志和额外复位；USB 导出不能作为可靠的现场工作流。

### 临时规避与建议

当前只能重启恢复。建议检查 USB MSC DMA 预留缓冲的生命周期、SDMMC controller/slot 的对称释放，并在恢复失败时停止高频交叉回退，避免资源状态进一步恶化。

## ISSUE-004：Ethernet 与 STA 两个真实来源不能同时计入 Web 客户端数

- 严重度：Medium / P2
- 固件 SHA-256：`FB76F2ECFA0402E0D51C3A8658545B37871C8A539C7CDA82A99ACBB56FA41874`
- 前置条件：电脑同时有 Ethernet `169.254.12.109` 和 WLAN `192.168.1.21`；板端两个地址均可 HTTP 访问
- 复现率：1/1；代码只读检查与现象一致

### 复现步骤

1. 强制从 Ethernet 源地址请求 `http://169.254.100.2/healthz`。
2. 在 30 秒内从 WLAN 请求 `http://192.168.1.80/api/status`。
3. 读取 `client_count` 和 `web_clients`。

### 预期

两个不同源 IP 同时处于有效期内，客户端数为 2。

### 实际

客户端数始终为 1。客户端槽位查找在遇到第一个非匹配活动槽时提前保留了“最老槽”，后续即使存在空槽也不会改用空槽，因此新地址覆盖旧地址。

### 影响

多客户端显示不准确；自动采集的“任一 Web 客户在线即暂停”判断可能遗忘仍在使用的旧客户端，增加误关网风险。

### 临时规避与建议

测试时保持至少一个客户端持续轮询。修复时应先查找同地址，再优先选择空/过期槽，只有槽位全满时才替换最老槽。

## ISSUE-005：活动中的长耗时验证请求不会持续阻止自动 FIELD

- 严重度：Medium / P2
- 固件 SHA-256：`FB76F2ECFA0402E0D51C3A8658545B37871C8A539C7CDA82A99ACBB56FA41874`
- 前置条件：自动采集开启、空闲时间 300 秒；启动一次长时间未返回的 `/api/validate/run`
- 复现率：1/1

### 复现步骤

1. 开启自动采集，设置 300 秒空闲。
2. 发起 `/api/validate/run` 并保持连接等待，不再发送额外轮询。
3. 通过串口观察约 300 秒后的模式变化。

### 预期

HTTP 请求仍在执行时应视为客户端活跃，不能关闭网络。

### 实际

请求只在进入 handler 时登记一次，客户端槽位 30 秒后过期；请求仍未返回时板端自动关闭网络并进入 FIELD，客户端连接被中断。

### 影响

长模型验证或其他未单独维护 active counter 的耗时 API 会被自动采集打断，并可能触发 ISSUE-001。

### 临时规避与建议

执行板端验证前关闭自动采集，或由前端并行轮询状态。建议对活动 HTTP handler 维护进入/退出计数，空闲切换同时检查该计数。

## ISSUE-006：`/api/config` 对非数字参数和非法网络模式静默接受

- 严重度：Low / P3
- 固件 SHA-256：`FB76F2ECFA0402E0D51C3A8658545B37871C8A539C7CDA82A99ACBB56FA41874`
- 前置条件：SERVER Web 可访问
- 复现率：3/3

### 复现步骤与实际

- `GET /api/config?recording_segment_ms=abc` 返回 200，并把片段时长写成最小值 5,000 ms。
- `GET /api/config?field_idle_timeout_ms=abc` 返回 200，并把空闲时间写成最小值 10,000 ms。
- `GET /api/config?network_mode=invalid` 返回 200，静默保持原模式。

同一 API 对未知模型会正确返回 400，专用 `/api/netmode?mode=bad` 也会返回 400，因此错误处理不一致。

### 预期

非数字值或未支持的枚举值返回 HTTP 400，且不修改 NVS。

### 影响

调用者输入错误时会在未察觉的情况下写入边界值，可能把 60 秒片段改为 5 秒，或误以为网络模式已生效。

### 临时规避与建议

客户端必须先校验并读取响应值复核。服务端应使用带完整尾字符检查的数值解析，并对非法网络模式显式返回 400。

## ISSUE-007：失败录像后的“清空录像”首次调用可能残留文件

- 严重度：Medium / P2
- 固件 SHA-256：`FB76F2ECFA0402E0D51C3A8658545B37871C8A539C7CDA82A99ACBB56FA41874`
- 前置条件：ISSUE-001 产生了 `.avi.part` 和 sidecar 残片，随后重启回 SERVER
- 复现率：1/1

### 复现步骤

1. 让 FIELD 录像因 TF 写错误中断并重启。
2. 确认目录中有 4 个残片。
3. 调用一次 `POST /api/recordings/cleanup`。
4. 再调用 `/api/storage/files`。

### 预期

一次调用删除全部录像和临时文件，索引同步为空。

### 实际

首次响应 `deleted_files=3`，但仍残留一个 0 字节 `raw_*.jsonl`；第二次调用才删除最后一个文件。

### 影响

清空操作的“完成”响应不可信，故障恢复后可能遗留孤立 sidecar 或污染后续索引。

### 临时规避与建议

清空后再次查询文件列表，必要时重复清空。建议删除遍历结束后重新扫描，只有目录中无目标文件且索引已重建时才返回成功。

## 已通过与被阻断项目摘要

已通过：

- 新版应用分区 Flash 回读 SHA-256 校验。
- ESP32-P4、32 MB PSRAM、ESP32-C6、Ethernet、STA、中文首页、健康/状态 API 初始化。
- Ethernet `169.254.100.2` 回退地址与 STA `192.168.1.80` 均可访问；电脑 WLAN 始终保持原路由器连接。
- Wi-Fi 密码不在状态/配置响应中回显；STA、模型、片段时长、空闲时间和自动开关可跨软件重启持久化。
- 片段时长 5 至 14,400 秒和空闲时间 10 至 86,400 秒边界值；错误确认字、过长 SSID、过短密码、未知模型、路径穿越和超大配置请求均被拒绝。
- Fish31、TinyCNN、COCO 各 4 张单图验证均匹配，SVG 可读取。
- 三模型 16 帧内置视频的推理计数均达到 16/16、失败帧为 0；但持久化结果受 ISSUE-001 影响。
- 自动采集关闭时不切换；单客户端持续请求时暂停；停止请求后可自动关闭板端网络。本机 WLAN/互联网未切换。
- 手动 USB_EXPORT 可完成应用到 USB 的 TF 所有权交接，且 Web 保持在线；恢复路径存在 ISSUE-003。

被阻断或环境未满足：

- 三模型 FIELD 录像、录像下载/Range/哈希、补帧、合并、录像中复位/USB 接管和 10 分钟 Fish31 FIELD 稳定性均被 ISSUE-001 在写入第一字节前阻断。
- USB1/OTG 数据线未被本机枚举，因而 `P4_BUOY` 盘符、读写吞吐、双向哈希、创建/重命名/删除、安全弹出、拔线自动恢复和再次自动导出未执行；这属于物理测试条件未满足，不记为产品缺陷。
- `p4-buoy.local` 在本机 Windows 解析超时；串口确认板端已启动 mDNS 广播，但本机缺少可用的 mDNS 解析客户端，因此不据此记缺陷。

