# P4 Buoy Vision 新版固件功能性板端复测记录

> 版本说明：本文件是 2026-07-17 早期 `3.0.1` / `59e4f39` 的历史功能复测记录，用于追溯 USB、FIELD、reset 恢复、图传和录像下载等问题的修复过程。当前交付 tag 为 `v3.1.0`，最新构建、烧录和文档口径以 `README.md`、`docs/developer_guide.md` 与 `CHANGELOG.md` 为准。不要把本文件中的固件版本号或测试结束配置当作当前发布版本号。

测试时段：2026-07-17 01:30 - 01:57  
测试角色：客户验收测试人员视角  
测试方式：真实板端操作，包含 HTTP API、Web 按钮对应接口、COM6 串口 reset、Windows USB 安全弹出、FIELD 手动/自动采集、录像下载验证。  
代码约束：未修改 `main/`、`components/` 等源码；本文件仅记录测试结论。

## 1. 测试基线

| 项目 | 实测值 |
|---|---|
| Git HEAD | `59e4f39 fix: address board retest recovery issues` |
| 板端地址 | `http://169.254.100.2/` |
| 串口 | `COM6` |
| 固件版本 | `3.0.1` |
| 固件编译时间 | `Jul 16 2026 20:24:54` |
| ELF SHA | `fa96e2a36...` |
| USB 卷标 | `P4_BUOY`，本轮枚举为 `E:` |
| TF 卡 | 约 119 GB，FAT32，`sdmmc_1bit`，写入自检通过 |
| 客户配置 | `recording_segment_ms=1800000`，`field_idle_timeout_ms=30000`，`method=fish31` |
| 测试结束状态 | `server`，`usb_storage_owner=app`，`tf_ready=true`，`field_auto_enable=false` |

测试证据目录：

- `artifacts/functional_retest_20260717_014003`
- `artifacts/functional_retest_field_clean_20260717_014526`
- `artifacts/functional_retest_records_20260717_014824`
- `artifacts/functional_retest_auto_field_wait_20260717_015143`

## 2. 总体验收结论

本轮未发现影响用户实际使用的阻塞问题。按“功能是否正常稳定”的客户验收口径，当前版本主要功能通过：

- SERVER/Web：配置保存、TF 状态、模型切换、单图验证、16 帧视频验证、实时图传、录像列表、manifest、首帧 SVG、下载均可用。
- FIELD：手动进入野外录像真实生效，采集、推理、raw/annotated 存储并发运行；10 秒片段自然闭合成对 AVI。
- FIELD 中断恢复：1800 秒片段未录满时 reset，启动后能恢复出 raw/annotated 成对 AVI。
- 自动 FIELD：无人 HTTP 访问后，串口确认 `idle_ms=30000` 自动进入野外录像；reset 后短片段恢复成对。
- USB_EXPORT：USB 一直插着时，Web 手动导出可用；Windows 安全弹出后 1-3 秒恢复 SERVER/TF ready。
- USB 恢复体验：Windows 仍挂载 `P4_BUOY` 时点击 Web 恢复，不再超时，返回 409 JSON 并给出明确操作提示。
- 上轮关注的弹出脚本误报已修复，本轮 `tools/eject_usb_msc.ps1` 多次返回成功。

## 3. 功能实测矩阵

| 编号 | 场景 | 期望行为 | 实际行为 | 结论 |
|---|---|---|---|---|
| T-01 | 启动和 reset 稳定性 | reset 后不崩溃、不循环重启，TF 自动挂载 | 多次 COM6 reset 均正常启动；未见 double-free；TF `sdmmc_1bit` 写入验证通过 | 通过 |
| T-02 | USB 线一直插着启动 | 能导出 U 盘，Web 仍可访问 | reset 后自动进入 `usb_export`，Windows 出现 `E: P4_BUOY`，`/healthz` 可访问 | 通过 |
| T-03 | Windows 挂载时 Web 恢复 | 不应超时，应明确提示用户先安全弹出 | `POST /api/mode/usb/restore` 返回 409，提示 Windows 正在使用 `P4_BUOY`，需安全弹出 | 通过 |
| T-04 | Windows 安全弹出恢复 | 弹出后自动回 SERVER/TF ready | 脚本成功弹出，板端 1-3 秒内恢复 `server`、`tf_ready=true` | 通过 |
| T-05 | 保存配置 | 1800 秒片段、30 秒自动 FIELD、Fish31 可保存并回显 | `/api/config` 回显正确；测试结束已关闭 `field_auto_enable` | 通过 |
| T-06 | 模型切换 | TinyCNN/COCO/Fish31 都能真实切换 | 三个模型均切换成功，状态页模型信息同步更新 | 通过 |
| T-07 | 单图验证 | Fish31 样例异步推理完成 | `fish31_01` 验证完成，Top-1 `fish_23`，`matched=true` | 通过 |
| T-08 | 16 帧视频验证 | 16 帧全部完成推理 | `fish31_video_demo` 16/16 成功，平均 170 ms，P95 174 ms | 通过 |
| T-09 | 实时图传 | wake 后 `/stream` 输出 MJPEG，standby 可关闭 | 6 秒收到 799263 bytes MJPEG，随后 standby 成功 | 通过 |
| T-10 | 手动 FIELD | 点击后真实进入野外录像，网络关闭，开始写 TF | 串口显示 `manual FIELD_MODE request accepted`，相机打开并写 raw/annotated | 通过 |
| T-11 | 10 秒分段闭合 | 达到片段长度后自动形成成对 AVI | 生成 3 组完整 AVI：46/46、27/27、28/28 帧 | 通过 |
| T-12 | reset 边界归档 | 0 帧临时文件不应以 `.corrupt` 暴露给普通用户 | 第 4 段 reset 时 0 帧 part 被归档到 `lost_found/*.zero_frame`，Web 列表不受影响 | 通过 |
| T-13 | Web 录像列表/下载 | Web 能看到成对录像并下载实际 AVI | 列表显示 3 组后续增至 4 组；`raw_003` 下载 579136 bytes，`annotated_003` 下载 736080 bytes，与列表一致 | 通过 |
| T-14 | Manifest/首帧 SVG | manifest 和首帧预览可访问 | `manifest ok=true`，首帧 SVG 29371 bytes | 通过 |
| T-15 | 自动 FIELD | 无 HTTP 访问 30 秒后自动进入 FIELD | 串口确认 `idle_ms=30000`，随后关闭网络并开始写 TF | 通过 |
| T-16 | 1800 秒短录 reset 恢复 | 未达到 1800 秒 reset，也要形成成对短视频 | 自动 FIELD 中 reset 后恢复 `raw_001_20260716_175224...` 与 `annotated_001...`，均 192 帧、54486 ms | 通过 |
| T-17 | USB 占用时 Web 行为 | TF 被 USB 占用时 Web 应明确提示 | `/api/recordings` 返回 409：TF 正在通过 USB 导出给电脑 | 通过 |
| T-18 | USB 目录可见性 | Windows 用户能看到录像文件 | `E:\esp32p4\recordings` 可见 8 个 AVI 和 8 个 JSONL | 通过 |
| T-19 | Web 手动 USB 导出 | SERVER 下点击/调用“USB 立即导出”应真实导出 | `POST /api/mode/usb?confirm=USB` 后约 3 秒进入 `usb_export`，Windows 出现 `E:` | 通过 |
| T-20 | 最终恢复 | 测试后设备应回到可继续使用状态 | 已恢复 `server`，TF ready，Fish31，1800 秒片段，自动 FIELD 关闭 | 通过 |

## 4. 当前问题清单

本轮没有发现需要作为当前 bug 交给开发修复的问题。

上一轮问题复测状态：

| 旧问题 | 本轮结论 |
|---|---|
| raw/annotated 少量帧差 | 按客户口径允许；本轮关键样本帧数一致 |
| 0 帧 `.avi.part.corrupt` 暴露 | 已改为 `lost_found/*.zero_frame` 归档，用户录像列表不受影响 |
| Windows 安全弹出脚本误报失败 | 已修复，脚本多次返回成功 |
| USB 仍挂载时 Web 恢复超时 | 已修复，快速返回 409 和明确操作提示 |
| reset 后短录像丢失 | 已修复，1800 秒片段短录 reset 后恢复 192/192 成对 AVI |

## 5. 不列为问题的观察

- 自动 FIELD 在配置变更或存储恢复后，会先打开网络访问窗口；忙状态清除后再重新按 30 秒倒计时。最终串口确认在 `idle_ms=30000` 进入 FIELD，用户实际需求满足。
- PowerShell `Invoke-RestMethod` 对板端地址曾报 502，但 `curl.exe`、浏览器同类请求正常；本轮不判定为板端问题。
- `/api/recording?name=...` 对 GET 返回 405 是接口设计，实际 Web 下载链接使用 `/recording/<name>.avi`，下载验证通过。

## 6. 未覆盖项

- 未做物理拔插 USB：本轮按用户要求 USB 线一直插着，仅使用 Windows 安全弹出模拟拔出。
- 未做 1800 秒完整自然闭合长跑：本轮验证了 10 秒自然闭合和 1800 秒短录 reset 恢复。
- 未做 overnight 压力测试：建议交付前继续做长时间 FIELD、reset、USB 导出循环压力验证。

## 7. 测试结束状态

- `app_mode=server`
- `usb_storage_owner=app`
- `tf_ready=true`
- `storage_acceptance_ok=true`
- `recording_segment_ms=1800000`
- `field_idle_timeout_ms=30000`
- `field_auto_enable=false`
- `method=fish31`
- Windows 上 `P4_BUOY` 未挂载
