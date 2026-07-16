# P4 Buoy Vision 板端验收测试问题记录

日期：2026-07-16  
测试角色：客户验收人员视角  
测试目标：按客户需求实际操作板子，记录“期望行为、实际行为、用户影响、修改建议”。  
测试约束：未修改 `main/`、`components/`、`tools/` 等源码；未切换本机 Wi-Fi；USB 线全程物理连接；使用 Windows 安全弹出模拟拔出；使用 COM6 串口 reset。

## 1. 测试环境

| 项目 | 实测值 |
|---|---|
| 板端地址 | `http://169.254.100.2/` |
| 串口 | `COM6` |
| 固件版本 | `3.0.1` |
| ESP-IDF | `v6.0.1-dirty` |
| USB 设备 | `P4_BUOY`，VID/PID `303A:4002` |
| TF 容量 | 约 119 GB，FAT32 |
| 最终状态 | 已恢复到 `server`，TF `sdmmc_1bit` 挂载，写入验证通过 |
| 关键配置 | `recording_segment_ms=1800000`，`field_idle_timeout_ms=30000`，`field_auto_enable=true`，`method=fish31` |
| 测试结束设置 | 为避免无人访问后继续进入 FIELD，测试结束后临时把 `field_auto_enable=false`；上表关键配置为验收测试期间使用的客户配置 |

## 2. 总体验收结论

当前固件已经具备 Web 配置、模型切换、单图/视频验证、实时图传、FIELD 并发写 raw/annotated、正常片段闭合、录像列表下载、补帧、USB 导出/恢复等主要能力。

但按客户最核心要求判断，当前版本还不能作为完整可靠交付版本，主要阻塞是：

- P0：FIELD 录制未达到片段长度时被 reset 中断，已写入几十帧的视频没有恢复成成对 AVI，而是从客户可见数据中消失。
- P0：FIELD/自动 FIELD 后 reset 的重启过程中至少两次复现 `tlsf_free` double-free 断言崩溃，系统二次重启后才恢复。
- P0/P1：Windows 安全弹出 USB 后，板端 60 秒内没有自动恢复 SERVER；如果客户把“弹出”理解为“拔出”，体验不满足需求。
- P1：USB 导出后 Windows 经常识别到 `P4_BUOY` 但不给盘符，普通用户在资源管理器里可能找不到 U 盘。
- P1：每次启动/恢复都会先尝试 SDMMC 4-bit，写入自检失败后降级到 1-bit，功能可用但可靠性和性能风险明显。

## 3. 板端实测矩阵

| 编号 | 场景 | 期望行为 | 实际行为 | 结论 | 修改建议 |
|---|---|---|---|---|---|
| T-01 | `/api/status` 状态读取 | SERVER 下能看到模式、网络、TF、USB、模型、相机状态 | 返回 `app_mode=server`、TF ready、Web 地址、模型等完整字段 | 通过 | 保持字段稳定，避免 UI 文案与实际模式不一致 |
| T-02 | 保存配置 | 保存 1800 秒片段、30 秒自动采集、Fish31 后回显生效 | `/api/config` 返回成功，回显 `recording_segment_ms=1800000`、`field_idle_timeout_ms=30000` | 通过 | `recording_enabled=false` 容易让用户误会，可改成“SERVER 待机，FIELD 时启用录像” |
| T-03 | TF 重试 | TF 已就绪时应快速返回成功 | `POST /api/storage/retry` 返回 `already_ready=true` | 通过 | 页面可显示“已就绪，无需重试” |
| T-04 | 清空录像 | 用户确认后后台清理并显示进度 | 空列表状态下仍创建 job，最终 `succeeded`，`total=1 deleted=1` | 通过但需谨慎 | 清空是真删除，应强化二次确认和删除数量展示 |
| T-05 | 模型切换 | TinyCNN/COCO/Fish31 都能保存并回显 | 三个模型均切换成功，`model_info` 对应更新 | 通过 | 保持模型名、输入尺寸、class_count 在 UI 中清晰显示 |
| T-06 | 单图验证 | 提交后异步完成，Web 不阻塞 | Fish31 样例 `fish31_01` 完成，expected `fish_23`，Top-1 `fish_23`，`matched=true` | 通过 | 页面继续保留任务 ID 和失败重试提示 |
| T-07 | 16 帧视频验证 | 16 帧全部板端推理完成 | `fish31_video_demo` 16/16 成功，平均约 169 ms，P95 177 ms | 通过 | 验证结果文件会写入 TF，页面可提示占用空间 |
| T-08 | 实时图传 | 唤醒相机，`/stream` 输出 MJPEG，关闭可回待机 | 唤醒后 `camera_ready=true`，5 秒抓取 652613 字节 MJPEG，待机接口返回成功 | 通过 | 首帧等待期间 UI 应显示“正在唤醒” |
| T-09 | 手动进入 FIELD | 点击后真实进入 FIELD，Web 断开，开始写 TF | 返回 `field_pending`；串口显示 HTTP/Wi-Fi/Ethernet 停止，FIELD active，raw/annotated 开始写入 | 通过 | Web 文案应明确“退出 FIELD 需要 reset/重启” |
| T-10 | FIELD 并发写入 | raw 和 annotated 同时运行，帧数一致 | 10 秒测试片段中 raw/annotated 分别成对闭合，帧数一致 | 通过 | 保持 raw 队列和 annotated 写入进度日志 |
| T-11 | 正常片段闭合 | 达到片段长度后自动形成成对 AVI | 临时设置 10 秒片段，生成 5 对 AVI；帧数分别为 47/47、28/28、28/28、28/28、28/28 | 通过 | 1800 秒完整长跑仍建议做夜间稳定性测试 |
| T-12 | 1800 秒片段中 reset | 只录 10-30 秒也必须恢复成一对 AVI | 1800 秒配置下录到 raw/annotated 64 帧以上后 reset；导出 TF 后没有 `20260716_112602` 对应 AVI，也没有 `.part` | 不通过，P0 | 必须实现 `.avi.part` 恢复闭合，不能静默删除客户数据 |
| T-13 | 30 秒无连接自动 FIELD | 无 Web/stream/download/validate 后 30 秒真实进入 FIELD | 串口显示 `idle_ms=30000, web_clients=0`，随后关闭网络并开始写 raw/annotated | 通过 | 自动进入前如页面在线，应明确倒计时暂停原因 |
| T-14 | 自动 FIELD 后 reset | 自动进入采集后，中断也应保留短视频 | 自动 FIELD 写到 64 帧以上后 reset；导出 TF 后没有 `20260716_112927` 对应 AVI | 不通过，P0 | 与 T-12 同源，恢复逻辑必须覆盖自动 FIELD |
| T-15 | reset 后启动稳定性 | reset 后应一次干净启动 | FIELD/自动 FIELD reset 后至少两次出现 `assert failed: tlsf_free ... block already marked as free`，随后二次重启 | 不通过，P0 | 开发需定位内存重复释放，特别是 USB/Hosted/TF 切换路径 |
| T-16 | SERVER 下 USB 立即导出 | 点击后进入 USB_EXPORT，Windows 枚举 `P4_BUOY` | 2 次轮询后 `app_mode=usb_export`、`usb_storage_owner=usb`；Windows 看到 `P4_BUOY` | 通过但有 UX 问题 | 解决无盘符问题，见 BUG-004 |
| T-17 | USB 占用期间 Web 列表 | Web 应明确说明 TF 被 USB 占用 | `/api/recordings` 返回 `TF card is exported to the computer over USB` | 通过 | UI 应把该错误翻译成用户可理解提示 |
| T-18 | Windows 安全弹出后自动恢复 | 安全弹出/拔出后恢复 SERVER | Windows eject 后 60 秒内仍 `app_mode=usb_export`、`usb_storage_owner=usb`、`usb_host_connected=true` | 不通过/需澄清，P0/P1 | 自动恢复实现、客户操作流程和文档口径必须统一 |
| T-19 | USB 恢复存储按钮 | 用户手动恢复后 TF 交还应用 | `POST /api/mode/usb/restore` 后回到 `server`，TF ready | 通过 | 如果仍需手动恢复，页面必须明确写“弹出后点击恢复” |
| T-20 | 录像列表 | 成对展示 raw/annotated | `/api/recordings` 返回 5 个 `recording_groups`，每组 raw/annotated 齐全 | 通过 | 编号每次 FIELD 从 001 开始，建议改成全局单调或突出时间戳 |
| T-21 | Web 下载 | raw/annotated 链接可下载有效文件 | raw 下载 922088 字节，annotated 下载 1174746 字节，与 TF 文件一致 | 通过 | 下载时自动 FIELD 倒计时应暂停 |
| T-22 | manifest/首帧 | manifest 和 SVG 可访问 | `/api/recording/manifest` 返回 ok，首帧 SVG 可打开 | 通过 | SVG 内容较大，页面应避免卡顿 |
| T-23 | 补帧 | raw 补帧应生成完整 annotated | `raw_001...` 补帧 47/47，coverage 1000，最终 `last_error=ok` | 通过 | 补帧会改写 annotated，应显示“正在重建”状态 |
| T-24 | USB 盘符可见性 | 普通用户应能在资源管理器看到 U 盘 | 多次导出后 Windows 识别到 `P4_BUOY` 卷，但无 DriveLetter；需要用卷 GUID 或手动分配 `Y:` 才能操作 | 不通过，P1 | 调整 USB MSC/分区/卷策略，或在 Web/说明书给出明确盘符处理方法 |
| T-25 | TF 挂载性能和稳定性 | TF 应稳定以预期总线模式写入 | 每次启动先 4-bit/20MHz 写入自检失败，降级到 1-bit/10MHz 后通过 | 有风险，P1 | 若硬件不适合 4-bit，应默认 1-bit；否则修复 4-bit 信号/时序 |
| T-26 | FIELD 中 USB 热插 | FIELD 录制中插 USB 应先闭合视频再导出 | 本轮 USB 线全程物理连接，无法真实复现“从未插到热插”；Windows eject 不能触发板端物理 detached | 未完全覆盖 | 后续必须用真实物理拔插复测；当前 reset 中断已证明短片段恢复不满足 |

## 4. P0 问题

### BUG-001：FIELD reset 中断后短录像丢失，未形成客户要求的成对 AVI

优先级：P0  
影响需求：FIELD 短录像保留、reset 中断恢复、自动 FIELD 后录像可验收。

期望行为：

- 片段长度设置为 1800 秒时，即使只录 10-30 秒后 reset，也要在重启后整理出一对 `raw_*.avi` 和 `annotated_*.avi`。
- 如果 raw/annotated 已经各写入部分帧，恢复逻辑应闭合 AVI 头/索引并保留 sidecar。
- 恢复失败时应报告明确错误，不能静默删除客户数据。

实测行为：

- 手动 FIELD：1800 秒配置下，串口显示 `raw_001_20260716_112602_fish31.avi` 和 `annotated_001_20260716_112602_fish31.avi` 已打开并写到 64 帧以上。
- reset 后 USB 导出检查 `esp32p4/recordings`，没有任何 `112602` 对应 AVI/JSONL，也没有 `.part`、`.idx` 或 `.corrupt` 留存。
- 自动 FIELD：串口显示 `raw_001_20260716_112927_fish31.avi` 和 annotated 写到 64 帧以上。
- reset 后同样没有 `112927` 对应文件。

用户影响：

- 野外设备只要未达到 1800 秒就遇到 reset、掉电或类似中断，客户会拿不到任何已录视频。
- 这正好违反“哪怕只录 10 秒，也必须形成一对视频”的核心需求。

修改建议：

- 开发启动恢复流程，扫描 `*.avi.part`、sidecar 和临时索引。
- 对 raw 和 annotated 分别尝试补写 AVI header/index 并 rename 为正式 `.avi`。
- 如果只有 raw 可恢复，也应基于 raw 生成 annotated 或至少生成明确的待补帧记录。
- 禁止恢复前静默删除可恢复的 `.part`。
- 在 `/api/status` 或事件日志中记录恢复成功/失败文件名和原因。

### BUG-002：FIELD/自动 FIELD 后 reset 启动过程中出现 double-free 断言崩溃

优先级：P0  
影响需求：野外可靠性、reset 恢复、USB 导出恢复。

期望行为：

- reset 后设备一次启动成功。
- 即使前一次处于 FIELD 录像，也不应发生堆内存断言。

实测行为：

- FIELD/自动 FIELD 录像后串口 reset。
- 启动过程中出现：

```text
assert failed: tlsf_free tlsf.c:630 (!block_is_free(block) && "block already marked as free")
```

- 本轮至少两次复现，随后系统二次重启，最终才进入 USB_EXPORT。

用户影响：

- 客户现场会看到设备重启时间变长，且存在无法恢复或循环重启风险。
- 该问题和 USB/TF/Hosted/网络切换路径相关，发生在关键恢复流程中，风险高。

修改建议：

- 优先分析 reset 后启动期间 ESP-Hosted、TinyUSB、TF mount/unmount、网络任务释放顺序。
- 对所有 destroy/free/deinit 路径加幂等保护。
- 增加 FIELD->reset->USB_EXPORT 连续压力测试。

### BUG-003：Windows 安全弹出不能让板端自动恢复 SERVER

优先级：P0/P1，取决于客户最终口径。  
影响需求：USB 拔出后恢复 Web 服务模式。

期望行为：

- 客户在 Windows 安全弹出 U 盘后，设备应进入可恢复路径。
- 如果需求定义为“拔出后自动恢复”，则物理拔线后应自动回到 SERVER。
- 用户不应卡在不知道下一步做什么的状态。

实测行为：

- 当前 USB_EXPORT 下 Windows 安全弹出 `P4_BUOY`。
- 60 秒轮询内，板端仍显示：
  - `app_mode=usb_export`
  - `usb_storage_owner=usb`
  - `usb_host_connected=true`
  - `sd_mounted=false`
- 只有手动调用 `POST /api/mode/usb/restore?confirm=RESTORE` 后，设备才回到 SERVER。

用户影响：

- 客户会以为已经“拔出/弹出”，但设备仍不恢复录像能力。
- 如果此时离开现场，设备可能长期停在 USB_EXPORT，无法自动采集。

修改建议：

- 明确区分“Windows 安全弹出”和“物理拔线”。
- 如果要满足客户“弹出即恢复”，需在 MSC eject/stop/unit attention 等状态后自动交还 TF。
- 如果必须物理拔线或手动恢复，Web、README、客户手册必须统一写清楚。

## 5. P1 问题

### BUG-004：USB 导出后 Windows 识别到卷但没有盘符

优先级：P1  
影响需求：USB U 盘导出可用性。

期望行为：

- 普通 Windows 用户插上/导出后能在资源管理器看到 `P4_BUOY` 盘符。

实测行为：

- Windows `Get-Volume` 能看到 `P4_BUOY`，但 `DriveLetter` 为空。
- 需要通过卷 GUID 路径访问，或手动分配 `Y:` 后才能使用脚本弹出。
- `tools/eject_usb_msc.ps1` 默认只查找带盘符的卷，因此无盘符时直接报 `Volume 'P4_BUOY' was not found`。

用户影响：

- 客户会认为 U 盘没有弹出或设备不可用。
- 现场复制数据的门槛变高。

修改建议：

- 检查 FAT32 分区、MBR/分区属性、Removable Media 行为和 Windows automount 兼容性。
- 工具脚本支持无盘符卷，通过 Volume GUID 或 DiskNumber 操作。
- Web 页面提示“若无盘符，请在磁盘管理中分配盘符”只能作为临时方案，不应作为最终体验。

### BUG-005：TF 每次先以 4-bit/20MHz 写入自检失败，再降级到 1-bit/10MHz

优先级：P1  
影响需求：长期录像可靠性、USB 导出速度、客户信心。

期望行为：

- TF 挂载模式稳定，写入自检一次通过。

实测行为：

- 多次启动/恢复日志均出现：
  - `TF mount attempt #1: sdmmc_4bit ...`
  - `sdmmc_write_sectors_dma returned 0x109`
  - `TF runtime I/O failure latched`
  - 降级到 `sdmmc_1bit` 后写入自检通过。

用户影响：

- 虽然最终可用，但启动/恢复变慢。
- 1-bit/10MHz 对长时间高码率录像和 USB 拷贝速度有压力。
- 日志中的 I/O failure 会削弱客户对存储可靠性的信任。

修改建议：

- 如果硬件当前只能稳定 1-bit，应默认使用 1-bit，减少失败路径。
- 如果目标是 4-bit，需修复 TF 线序、上拉、时钟、DMA 或 LDO 时序。
- Web 状态应把“已降级但可用”和“不可用故障”区分开。

### BUG-006：USB 导出期间状态文案和客户操作流程仍需统一

优先级：P1  
影响需求：USB 任意模式导出、拔出后恢复 Web。

实测行为：

- `/api/status.storage_service.status` 显示 `safe eject and unplug to restore automatically`。
- 开发文档中又存在“安全弹出并物理拔线后仍需 Web 点击恢复”的口径。
- 本次 Windows 安全弹出后并未自动恢复，手动恢复按钮可以恢复。

用户影响：

- 客户不知道应该等待自动恢复、拔线，还是返回 Web 点击恢复。

修改建议：

- 以客户最终需求为准统一：自动恢复就实现自动恢复；手动恢复就把“必须手动点击”写进所有 UI 和文档。
- Web 页面在 USB_EXPORT 下显示明确步骤：复制完成、Windows 弹出、物理拔线/点击恢复、恢复完成。

### BUG-007：FIELD 录像编号每次从 001 重新开始，易造成用户误解

优先级：P1/P2  
影响需求：录像可追溯性。

实测行为：

- 10 秒片段测试生成 `raw_001_20260716_112233...` 到 `raw_005_...`。
- 后续 1800 秒短录测试又从 `raw_001_20260716_112602...` 开始。
- 文件名靠时间戳保持唯一，但用户看到多个 `001` 容易误会覆盖或排序错误。

修改建议：

- 使用全局递增编号、session_id，或在 UI 中把日期时间作为主标识。
- 保证文件列表按时间排序，不按编号误排序。

## 6. P2/体验改进

### UX-001：SERVER 下 `recording_enabled=false` 容易误导

实测行为：保存配置和 SERVER 状态中 `recording_enabled=false`，但进入 FIELD 后确实会录像。  
建议：改成更贴近用户的状态，例如“SERVER 待机，FIELD 将启用录像”。

### UX-002：清空录像按钮是真删除，需要更强保护

实测行为：即使列表为空，后台清理任务仍报告删除 1 个相关文件。  
建议：确认框中显示预计删除范围，清理后显示删除数量和释放空间。

### UX-003：USB 导出期间 Web 列表不可读是合理的，但页面提示要更像用户语言

实测行为：`/api/recordings` 返回英文错误，说明 TF 已导出给电脑。  
建议：页面显示“TF 卡正在作为 U 盘给电脑使用，录像没有丢失；复制完成后请弹出并恢复存储”。

## 7. 已通过能力清单

以下能力在本次板端实测中可用：

- Web 状态读取。
- 保存 1800 秒片段、30 秒自动 FIELD、Fish31 模型。
- TF 重试接口。
- 模型切换 TinyCNN/COCO/Fish31。
- Fish31 单图验证。
- Fish31 16 帧视频验证。
- 相机唤醒和 MJPEG 实时图传。
- 手动进入 FIELD。
- FIELD 采集、推理、raw/annotated 写入并发运行。
- 正常达到片段长度后闭合成对 AVI。
- 30 秒无 Web 连接自动进入 FIELD。
- Web 录像列表成对显示。
- raw/annotated Web 下载。
- manifest 和首帧 SVG 访问。
- raw 补帧重建 annotated。
- SERVER 下 USB 立即导出。
- 手动 USB 恢复存储。

## 8. 未完全覆盖或需要后续上板复测

- 真正的 FIELD 物理热插 USB：本轮 USB 线按用户要求全程物理连接，只能用 Windows eject 和 Web USB 导出模拟部分流程，不能完全替代“FIELD 录像时从未插线到插线”的电气事件。
- 1800 秒完整长片段：本轮用 10 秒临时片段验证正常闭合，用 1800 秒配置验证短录 reset 中断；1800 秒完整跑满建议做夜间/长期稳定性测试。
- 物理拔线后的自动恢复：本轮用 Windows 安全弹出模拟拔出，但板端仍认为 host connected；需要实际物理拔线复测 TinyUSB detach 路径。
