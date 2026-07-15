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

## 当前修复分支状态（2026-07-15）

本节描述 `fix/test-issues-hardening` 的 `v3.0.1` 修复状态。下文各 ISSUE 的“实际”和日志仍是基线固件 `2c725fd` 的原始测试证据；本节的构建与板端结果只记录本修复版本重新取得的证据。

| 构建配置 | 应用镜像 | SHA-256 | 14 MiB 应用分区余量 |
|---|---:|---|---:|
| rev1 | 9,984,512 字节 | `14D35147F4E68E85B3CCF56C7416A89FD571921BD30C2485F65551F3F4D1ED65` | 4,695,552 字节（31.99%） |
| rev3.1 | 10,062,272 字节 | `594F272A370E71D967F5DCF414DC3E2A94156D16F4027A6AE653ED2B6956F80E` | 4,617,792 字节（31.46%） |

| 问题 | 当前代码处理 | 本轮验证与剩余风险 |
|---|---|---|
| ISSUE-001 TF 写入失败 | 挂载后执行真实 4 KiB 写入、`fsync`、重开读回验证；I/O 故障会锁存并停止后续录像写入，状态页不再把“仅挂载成功”当成“可录像”。Web 提供“检查并重试 TF”。 | 实机在 4-bit 真失败后自动降级到 `sdmmc_1bit` 并通过写入、同步、重开读回；完成 603.6 秒 FIELD 和 79 对 raw/annotated。最终镜像再次完成短片段实录。卡片、供电或信号完整性物理故障仍不能由软件掩盖。 |
| ISSUE-002 相机失败后无法唤醒 | 相机/JPEG/ISP/video VFS 的失败路径改为对称、best-effort 清理，并提前申请关键 JPEG/DMA 资源，避免一次失败污染后续唤醒。 | 已完成三轮 wake/JPEG/standby；最终镜像从 standby 到首批 JPEG 约 1.17 秒，未需重启。极端内存故障注入仍属于持续回归项。 |
| ISSUE-003 USB 恢复风暴 | 移除无效 USB DMA reserve，补齐 Hosted SDMMC slot 清理；TinyUSB `DETACHED`/配置失活不再被当成物理拔线，TF 会保持隔离，只有用户在安全弹出并拔线后从 Web 明确恢复；失败时不无限交叉重试或主动重启。 | 使用真实 USB1 主机完成安全弹出、物理拔线、隔离状态确认、Web 显式恢复、失败注入，以及恢复后重新插线导出回归。 |
| ISSUE-004 客户端计数 | 客户端槽位按“同 IP、空/过期槽、最老槽”的顺序选择，避免新地址覆盖仍活跃的旧地址。 | STA 与 Ethernet 同时访问实测 `client_count=2`；AP 多来源可继续作为扩展回归。 |
| ISSUE-005 长验证被自动 FIELD 打断 | `/api/validate/run` 改为异步任务：`POST` 立即返回任务 ID，页面轮询 `/api/validate/status?id=...`；活动验证任务计入自动采集暂停条件，不占住单个 HTTP worker。 | Fish31、TinyCNN、COCO 异步任务与各 16 帧数据集均通过；Web 在任务期间保持响应。 |
| ISSUE-006 配置静默接受错误值 | 配置写入统一使用 `POST /api/config`，严格校验完整数字、布尔值、枚举、长度和表单编码；候选配置先原子写入 NVS，成功后才切换运行值。带修改参数的 `GET /api/config` 返回 405。 | 最终镜像实测修改型 GET 为 405、非法 POST 为 400；8 秒片段/15 秒倒计时/自动采集开启经软件重启仍保留并实际进入 FIELD。交付前恢复 60 秒/300 秒/开启，并再次重启确认。 |
| ISSUE-007 清空后残留 | 后台 cleanup job 关闭目录后分批删除、覆盖 FAT 长文件名和原子索引备份，完成后复扫目录与索引；下载/上传/清理使用统一 admission 与锁。 | 慢下载期间 79 ms 返回 409；取消后 reader 归零；两个请求同一 job ID、均为 202，第二个合并；清理期间列表为 423；最终 19/19 删除、remaining=0、errors=0，列表为空。 |

### v3.0.1 最终板端门禁（2026-07-15）

- rev1 与 rev3.1 均从空构建目录完成 ESP-IDF 6.0.1 clean build；嵌入式客户页和验证页 JavaScript 通过 `node --check`，`git diff --check` 通过。
- COM6 确认为 ESP32-P4 rev3.1；未擦除 Flash，仅写入 `0x2000` bootloader、`0x8000` partition table、`0x10000` app。三段随后独立 `verify-flash`，digest 全部匹配。
- STA `192.168.1.80` 与 Ethernet `169.254.100.2` 的 `/healthz` 均为 200；Ethernet 连续 10 次响应为 48-72 ms，清理与下载竞态期间 Web 仍可查询状态。
- 最终状态为 SERVER/standby，TF `storage_acceptance_ok=true`、`storage_write_verified=true`、reader=0；正式默认配置为 `field_auto_enable=true`、`field_idle_timeout_ms=300000`、`recording_segment_ms=60000`，软件重启后回读一致。
- 管理员认证仍不在本轮范围内，只允许部署在受信任局域网。真实 USB1 主机当前不可用，因此 ISSUE-003 的物理枚举、安全弹出、拔线和再次导出仍是唯一关键环境缺口。

### 用户可见配置与恢复约定

- Web「用户设置」可修改并持久化：自动进入野外采集开关、无连接进入采集倒计时 `10-86400` 秒、录像片段时长 `5-14400` 秒。保存成功文案和 `/api/status.config` 显示的是设备实际采用值，不以前端输入值冒充成功；片段时长只影响当前及后续采集，不在后台重切历史录像。
- 倒计时遇到 Web/图传/下载/验证/数据集/推理或存储维护任务会暂停，页面显示暂停原因；任务结束后重新计时，避免用户操作被突然断网。
- TF 重试、重新挂载和格式化均通过互斥的维护流程执行：先停止采集与存储使用者，闭合录像，再操作文件系统并执行写读验证，最后恢复原相机状态、网络和 Web。普通失败不要求用户重启；页面可能短暂重连，用户应等待状态更新，不要重复点击。
- `POST /api/storage/retry?confirm=RETRY` 是普通用户首选恢复入口。`POST /api/storage/remount?confirm=REMOUNT` 和 `POST /api/storage/format?confirm=FORMAT` 只允许在 SERVER 模式运行；格式化会删除整张 TF 的数据，只能在明确备份并确认后使用。
- TinyUSB `DETACHED` 或 USB 配置失活只表示协议配置不再活动，不等于数据线已物理拔出。固件不会因此自动回收 TF 或再次枚举；用户须先安全弹出并拔线，再在 Web 点击「USB 恢复存储」。显式恢复完成写入、同步和读回验证前，TF 始终保持隔离。
- ESP-IDF 6.0.1 的 FAT 格式化函数存在“格式化完成、重新挂载失败却返回成功”的问题。构建期补丁现在会传播重新挂载错误并保留资源给应用统一清理，防止页面误报成功；同时包含 Hosted SDMMC slot 清理补丁。补丁脚本只接受预期 IDF 版本，无法安全匹配时构建直接失败。
- 所有修改状态的 Web/API 操作使用 `POST` 或对应的非 GET 方法；误用 GET 返回 `405 Method Not Allowed`。Wi-Fi 密码只通过 POST body 提交，服务端拒绝把密码放在 URL 中，状态和配置响应也不回显明文。
- 管理员认证不在本轮修复范围内，当前固件不能宣称管理接口已受身份认证保护。完成认证前只应部署在受信任的局域网/设备专网，不能把管理 Web 直接暴露到互联网。

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

旧版在 USB 所有权交回应用时可能使 TF 在本次运行中永久不可用，并产生大量错误日志和额外复位；USB 导出不能作为可靠的现场工作流。

### 临时规避与建议

当前只能重启恢复。建议检查 USB MSC DMA 预留缓冲的生命周期、SDMMC controller/slot 的对称释放，并在恢复失败时停止高频交叉回退，避免资源状态进一步恶化。

### 当前修复验收步骤（显式 Web 恢复）

1. 在 SERVER 且 TF 写读验证通过时插入真实 USB1 主机，确认 Windows 出现 `P4_BUOY`、Web 在线，状态为 `app_mode=usb_export`、`usb_storage_owner=usb`、`sd_mounted=false`。
2. 完成文件复制并在 Windows 安全弹出，然后物理拔掉 USB 数据线。
3. 确认 TinyUSB `DETACHED`/配置失活没有触发 TF 回收或重新枚举；TF 仍保持隔离，Web 明确提示点击「USB 恢复存储」。
4. 用户点击 Web「USB 恢复存储」（等价于 `POST /api/mode/usb/restore?confirm=RESTORE`）。
5. 确认状态回到 SERVER、`usb_storage_owner=app`、`sd_mounted=true`、`storage_acceptance_ok=true`，Web 始终可恢复访问，且没有重启或挂载重试风暴。
6. 分别注入 USB reset、`SetConfiguration(0)` 和恢复失败；线缆或主机配置仍可能活动时必须继续隔离 TF，失败后保留可操作提示。完成显式恢复后重新插线，验证下一次导出正常。

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

## 当前验收结论

ISSUE-001、002、004、005、006、007 的代码修复与当前可用板端门禁均已通过，可作为 `v3.0.1` 交付候选。ISSUE-003 的防重试风暴、显式恢复和所有权隔离代码已完成，但缺少真实 USB1 主机，不能宣称物理 USB 工作流验收完成。`p4-buoy.local` 在本机 Windows 仍无可用解析客户端，不据此记产品缺陷。
