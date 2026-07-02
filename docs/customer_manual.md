# ESP32-P4 板端目标识别产品使用手册

本文面向现场使用人员，说明如何开机、连接、查看实时画面、回放录像、定位识别事件，以及如何使用 COCO 视频验证。存储优先使用 TF 卡；如果 TF 卡暂时无法被板端识别，设备会自动使用内部 flash fallback 保存最近记录。

## 1. 开机与连接

1. 插入 TF 卡并给开发板上电。
2. 板子默认同时启动热点和路由器连接：
   - 热点名称：`P4_Buoy_Lab`
   - 热点密码：`change-me-please`
   - 固定访问地址：`http://192.168.4.1/`
3. 手机或电脑连接热点后，打开：
   - 实时监控首页：`http://192.168.4.1/`
   - 图片/视频验证页：`http://192.168.4.1/validate`
4. 如果设备和板子在同一路由器，也可以使用串口或 `/api/status` 中显示的 `sta_url`。

网络服务上电后至少开放 5 分钟。只要有热点客户端、网页/API 请求或视频流客户端，网络会继续保活；全部空闲超过 5 分钟后，板子会关闭 Wi-Fi 和 HTTP 服务，下次重启再次开放。

## 2. 实时监控

首页左侧显示摄像头实时画面，右侧显示状态卡片：

- `采集帧率`：摄像头采集速度。
- `传输帧率`：网页 MJPEG 推流速度。
- `推理延时`：最近一次 Fish31/TinyCNN/COCO 推理耗时。
- `板端识别`：当前目标类别、置信度、检测框数量和模型信息。
- `存储`：当前后端、TF 卡挂载状态、历史数量、录像段数量、当前录像文件和剩余空间。`storage_backend=flash_fat` 表示正在使用内部 fallback，`tf_sdmmc/tf_sdspi` 表示真 TF。

默认识别方法为 `Fish31 MobileNetV3 224`。Fish31/TinyCNN 是分类模型，主要查看 Top-1 和 Top-K 置信度；COCO 是检测模型，如果画面误框多，可提高“框阈值”，如果漏检，可降低阈值。

## 3. 监控记录、录像与删除

固件默认生成三类数据：

- 离散识别事件：检测到目标时保存一条 JSON 记录，可选保存 JPEG 快照。
- 分段录像：按固定 FPS 保存摄像头 MJPEG 片段，默认每段 60 秒。
- 周期摘要：每段录像生成目标类别统计，供 API 搜索使用，不再作为首页重复词条显示。

统一存储目录结构：

```text
/sdcard/esp32p4/history.jsonl              识别事件记录
/sdcard/esp32p4/snapshots/*.jpg            命中帧快照
/sdcard/esp32p4/recordings/*.avi           MJPEG-in-AVI 分段录像
/sdcard/esp32p4/recordings/*.jsonl         每帧识别 sidecar，用于回放叠框
/sdcard/esp32p4/recordings.jsonl           录像片段索引
/sdcard/esp32p4/summaries.jsonl            周期识别摘要
/sdcard/esp32p4/datasets/coco_video        COCO 视频帧数据集
/sdcard/esp32p4/dataset_runs               视频验证结果
```

首页“监控记录 Timeline”显示录像片段和离散识别事件，不再重复显示摘要。一条“监控录像 · 本地日期时间”对应一个实际录像时段，右侧分别提供“下载原始视频”“下载标注视频”和“删除此录像”；页面不会暴露内部文件名。详情包含时长、两种版本的大小与帧数，以及标注录像优先的检测统计。旧录像没有 UTC 时间时显示启动后的相对时间，无法可靠配对的历史录像会保留为单独词条并显示缺失的视频版本。

删除合并词条会真实删除 TF 卡上该时段的原始/标注媒体、同名 sidecar 和临时残留，并同步清理当前/旧录像索引、摘要索引及事件索引。页面只有收到成功响应后才刷新词条；目标不存在会提示 `404`，部分失败会保留错误提示。批量删除和格式化均需要二次确认。

设备优先通过 NTP 校时。无法访问 NTP 时，首页会调用 `/api/time/sync` 使用手机时间作为回退，板端保存 UTC epoch，页面按手机本地时区显示。

## 4. COCO 视频数据集导入

板端 COCO 模型接收 JPEG 图像。工程提供脚本把公开视频转换成 TF 卡帧数据集：

```powershell
.\.venv_yolo\Scripts\python.exe tools\prepare_coco_tf_dataset.py --output data\tf_datasets\coco_video
```

生成结果：

```text
data/tf_datasets/coco_video/frames/*.jpg
data/tf_datasets/coco_video/manifest.jsonl
data/tf_datasets/coco_video/summary.json
```

如果要使用完整 162 帧数据集，将整个 `data/tf_datasets/coco_video` 目录复制到 TF 卡：

```text
TF卡根目录/esp32p4/datasets/coco_video
```

打开 `/validate` 后，可在 `Fish31 MobileNetV3`、`TinyCNN Marine`、`COCO YOLO11n` 三套验证之间切换。Fish31/TinyCNN 使用板端筛选过的分类样例，页面显示 Top-1 横幅和 Top-K；COCO 使用商店过道公开视频的 16 张连续帧，页面显示检测框。三套内置视频验证都不依赖 TF 卡。

TF 卡上的 `coco_video` 是可选长数据集，只通过 API 或后续扩展运行，不再由四张图片自动填充。图片验证中的 `demo_01~04` 保持为独立的 COCO classic 单图入口。

也可以通过首页“监控记录 Timeline”的“视频验证”按钮启动同一流程。

## 5. TF 卡格式化

新卡未格式化时，优先使用电脑格式化为 FAT32。Windows 对大容量卡有时只显示 exFAT，建议使用 SD Card Formatter 或 DiskGenius 选择 FAT32。

板端也提供维护接口：

```text
POST http://192.168.4.1/api/storage/remount?confirm=REMOUNT
POST http://192.168.4.1/api/storage/format?confirm=FORMAT
```

板端格式化会短暂关闭网页和无线，暂停 C6 transport 后尝试识别/格式化 TF 卡，随后自动重启并恢复热点。重要限制：如果 `/api/status` 或串口显示 `ESP_ERR_TIMEOUT`，说明板子还没有在底层识别到 TF 卡，格式化命令无法执行。这时应检查 TF 卡是否插紧、卡槽触点、板卡引脚/供电，而不是继续格式化。如果显示 `ESP_ERR_NOT_FOUND` 或串口出现 `no available sd host controller`，通常表示现场刷入的仍是旧 SDIO-host 冲突固件；当前验收版本应使用 P4 SPI host + C6 SPI slave。

## 6. 常用 API

```text
/api/status                         当前实时状态
/api/timeline?limit=50              统一监控时间线
/api/history?limit=20               最近识别事件
/api/history.jsonl                  当前存储后端历史 JSONL
/api/recordings?limit=100           录像片段和周期摘要；首页据此合并原始/标注版本
/api/search?label=person&min_score=50 服务端搜索历史、录像、摘要和逐帧 sidecar
/recording/<name>.avi               播放或下载录像片段
/recordingmeta/<name>.jsonl         查看某段录像每帧识别 sidecar
/api/recording/frame.svg?name=<name>.avi&frame=<n>
                                    查看录像指定帧的标注结果
DELETE /api/recording?name=<primary>&paired_name=<secondary>&confirm=DELETE
                                    删除一个录像时段的原始和标注版本
/snapshot/<name>.jpg                查看某个历史快照
/api/datasets                       列出当前存储后端视频数据集
/api/dataset/run/start              启动视频数据集板端验证
/api/dataset/run/status             查看视频验证进度
/api/dataset/frame.svg?run_id=<run>&dataset=coco_video_demo&index=<n>
                                    查看视频验证指定帧的标注结果
/api/storage/remount?confirm=REMOUNT 进入 TF 维护窗口并重启恢复网络
/api/storage/format?confirm=FORMAT  板端格式化 TF 卡
/api/time/sync?epoch_ms=<unix-ms>   POST，使用手机时间回退校时
/api/config?...                     修改阈值、推理间隔、推流 FPS、录像/历史开关
/api/validate/run?sample=demo_01&method=coco&box_min_score=50
                                    COCO classic 板端图片推理验证
```

录像文件通过异步下载 worker 发送，支持 HTTP Range。下载期间仍可查询状态或发送 wake；Standby/Wake 不再重复初始化整个摄像头硬件栈。

## 现场验收口径

本版本以真 TF/SD 卡为长期离线监控验收标准。`flash_fat` 只表示应急页面和小样例验证可用，不代表野外采集存储通过。打开 `/api/status` 后确认：

- `tf_required=true`
- `tf_ready=true`
- `storage_backend=tf_sdmmc` 或 `tf_sdspi`
- `sd_total_bytes > 100 GiB`
- `storage_acceptance_ok=true`

128 GB FAT32 TF 卡应满足以上条件。若 `storage_acceptance_ok=false`，请先处理 TF 挂载、容量或 C6 SPI transport 固件匹配问题，再做野外部署。

## 7. 常见问题

- 页面打不开：先连接 `P4_Buoy_Lab` 热点，再访问 `http://192.168.4.1/`；如果热点已消失，重启板子。
- TF 显示不可用：先看 `/api/status` 的 `storage_backend`、`tf_card_mounted`、`sd_mount_mode`、`sd_last_error`。`flash_fat` 表示产品正在用内部 fallback，历史和视频验证仍可用；`ESP_ERR_TIMEOUT` 表示底层未识别卡；`ESP_ERR_NOT_FOUND/no available sd host controller` 多半是旧 SDIO-host 冲突固件或刷写不一致；`ESP_FAIL` 更可能是文件系统问题，可尝试格式化。
- 录像没有增长：确认首页“录像 Recording”开关已打开，且 `/api/status` 中 `file_storage_mounted=true`。内部 fallback 空间有限，录像会自动降到 1 FPS 并只保留最近片段。
- 识别慢：COCO 正常单帧约 650-700 ms；自训练 Coke/Sprite YOLO11/YOLO26 是实验对比模型，单帧十几秒，不适合作为主演示。
