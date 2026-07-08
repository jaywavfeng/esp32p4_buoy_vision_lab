# 给泽奇：定制板硬件设计交接

## 先说结论

当前开发板验证链路已经跑通了摄像头、无线 HTTP 图传、板端推理、TF 存储和分段录像。但定制板不要照抄开发板所有接口，建议按最终封装场景重新取舍。

最建议的产品方向：

1. 保留 ESP32-P4、PSRAM、flash、MIPI-CSI 摄像头接口。
2. 如果需要长线部署或一根线供电通信，主通信/导出接口优先考虑 RJ45 Ethernet + PoE。
3. 存储不要依赖不可维护的可插拔 TF 卡。如果封装后卡取不出来，优先考虑板载 eMMC 或工业级 microSD 焊接/内置固定方案，并预留 USB 或 Ethernet 离线导出。
4. Wi-Fi/C6 可以作为调试或备用，但如果最终确定全程有线，可以考虑删除 C6，降低复杂度和功耗。
5. 保留 USB/UART 调试口和量产烧录入口。

## 当前实际硬件使用情况

| 模块 | 当前是否使用 | 说明 |
|---|---|---|
| ESP32-P4 | 使用 | 主控，跑摄像头、HTTP、推理、存储 |
| 32 MB PSRAM | 使用 | 放 JPEG 缓存、网页 buffer、任务数据 |
| 16 MB flash | 使用 | 固件、模型、验证图片、内部 FAT fallback |
| OV5647 MIPI 摄像头 | 使用 | 当前主摄像头 |
| MIPI-CSI | 使用 | 摄像头数据接口 |
| SCCB/I2C | 使用 | 摄像头控制，`SCL=8`、`SDA=7` |
| ESP32-C6 Wi-Fi 协处理器 | 使用 | 当前通过 ESP-Hosted SDIO 提供 AP/STA |
| TF 卡 | 使用 | 当前验收记录显示 `tf_sdmmc` / `sdmmc_4bit` 成功，SDSPI 仅作为 fallback |
| USB | 使用 | UART Type-C 用于烧录/日志；ESP32-P4 USB HS OTG 已实现整张 TF 的可读写 MSC 离线导出 |
| Ethernet/RJ45 | 使用 | 已验证直连电脑文件导出，DHCP 优先，无 DHCP 时 fallback 到 `169.254.100.2`；适合长线、PoE、远程维护和文件下载 |
| LCD/DSI | 未使用 | 可删除 |
| 触摸 | 未使用 | 可删除 |
| 音频 | 未使用 | 可删除 |
| DVP 摄像头 | 未使用 | 可删除 |
| SPI 摄像头 | 未使用 | 可删除 |
| USB UVC 摄像头 | 未使用 | 可删除 |
| 按键/扩展排针 | 非核心 | 只保留调试和生产需要的测试点 |

## 当前实际引脚

### 摄像头

```text
接口：MIPI-CSI
传感器：OV5647 / Raspberry Pi Camera(B) Rev2.0
SCCB I2C SCL：GPIO8
SCCB I2C SDA：GPIO7
Reset/PWDN/XCLK：当前 board 配置为 -1，走模组/板级默认连接
```

定制板建议：

- 保留 MIPI-CSI FFC；
- 摄像头线尽量短；
- 注意阻抗、差分走线、ESD 和防水结构；
- 如果未来换摄像头，要先确认 ESP-DL/esp_video 支持和 ISP 配置。

### ESP32-C6 Wi-Fi 协处理器

当前实际构建是 ESP-Hosted SDIO：

```text
P4 <-> C6 SDIO Slot 1
CMD：GPIO19
CLK：GPIO18
D0 ：GPIO14
D1 ：GPIO15
D2 ：GPIO16
D3 ：GPIO17
RESET：GPIO54
SDIO clock：20 MHz
```

注意：仓库旧文档曾写过 C6 SPI transport，但当前 `sdkconfig` 和 `build/config/sdkconfig.h` 显示仍是 SDIO。请按当前实际 SDIO 设计，除非后续专门重新验证 SPI 方案。

### TF 卡

配置中保留 SDMMC 4-bit 引脚：

```text
CMD：GPIO44
CLK：GPIO43
D0 ：GPIO39
D1 ：GPIO40
D2 ：GPIO41
D3 ：GPIO42
LDO：4
```

当前代码优先按 SDMMC 4-bit 挂载 TF，失败时再 fallback 到 SDSPI：

```text
MOSI/CMD：GPIO44
MISO/D0 ：GPIO39
SCLK    ：GPIO43
CS/D3   ：GPIO42
LDO     ：4
```

当前板端验收状态：

```text
storage_backend=tf_sdmmc
sd_mount_mode=sdmmc_4bit
sd_total_bytes=125033775104
tf_ready=true
storage_acceptance_ok=true
```

定制板建议：

- 当前 C6 仍可保留为 Wi-Fi 调试入口，TF 侧按已验证的 SDMMC 4-bit 设计；同时保留 SDSPI fallback 的兼容性。
- 如果删除 C6 或后续改 C6 通信方案，TF/eMMC 仍建议优先走 SDMMC 4-bit。
- 如果封装后存储不可取出，优先考虑 eMMC 或工业级 microSD，且必须提供远程下载、格式化、剩余容量查询和必要的调试入口。

## 额外硬件和采购建议

从工程和验收记录能确认用到的额外硬件：

| 硬件 | 当前情况 | 建议 |
|---|---|---|
| OV5647 摄像头模组 | 已使用 | 定制板继续按 MIPI-CSI 摄像头设计 |
| TF/microSD 卡 | 已使用，验收容量约 128 GB | 采购高耐久或工业级，不建议普通消费卡长期录像 |
| USB 数据线 | UART 烧录线和 HS OTG DEVICE 数据线均已使用 | 开发板 MSC 测试使用 USB-A 对 USB-A 数据线；定制板应改用规范 USB-C device 线缆和接口 |
| 网线 | 有线验证需要 | 后续做 Ethernet 文件下载、维护或图传时准备 |

还需要你确认采购的：

```text
工业级 microSD 或 eMMC
防水 RJ45 或防水网线接口
PoE 模块或 PoE PD 方案
摄像头 FFC/固定结构件
调试用 USB-C/串口测试点
必要的电源保护、ESD、防反接、浪涌保护
```

## 定制板接口建议

### 必须保留

```text
ESP32-P4
PSRAM
16 MB 或更大 flash
MIPI-CSI 摄像头接口
摄像头 I2C
主存储：eMMC 或工业级 microSD/TF
有线通信/导出：按场景选择 USB 离线导出或 RJ45 Ethernet/PoE
调试/烧录：USB-C 或 UART/JTAG 测试点
复位/下载模式测试点
必要电源测点
```

### 可以删除

```text
LCD/DSI
触摸
音频
DVP 摄像头
SPI 摄像头
USB UVC 摄像头
开发板大排针
不需要的按键和扩展口
```

### 待决策

```text
ESP32-C6 Wi-Fi 协处理器
TF 卡是否可插拔
是否使用 PoE
是否需要外露 USB-C
是否需要外置天线
```

## 三种硬件路线

### 路线 A：最终产品有线为主，删除 C6

优点：

- 硬件简单；
- 功耗更低；
- 省掉 P4-C6 SDIO 复杂性；
- TF/eMMC 更容易走 SDMMC 4-bit；
- 有线下载和维护更稳定。

缺点：

- 没有无线调试入口；
- 外场调试必须接网线或 USB。

这是最终封装最推荐的路线。

### 路线 B：保留 C6 SDIO，Ethernet/USB 导出为主，Wi-Fi 为备用

优点：

- 保留手机热点调试；
- 当前代码已经验证过 Wi-Fi/HTTP；
- 风险较小。

缺点：

- C6 SDIO 占用 `GPIO14-19/54`；
- TF 当前已验证可走 SDMMC 4-bit；
- 功耗、BOM 和固件复杂度更高。

如果需要手机现场调试，可以选这条。

### 路线 C：保留 C6，但改 P4-C6 为 SPI

优点：

- 理论上可以让 TF 更容易走 SDMMC；
- 仍保留 Wi-Fi。

缺点：

- 当前仓库实际构建不是 SPI；
- 需要同步刷 P4 app 和 C6 slave；
- 需要重新做 Wi-Fi、TF、录像、下载全链路验收。

这条不能直接进入硬件定版，只能作为后续验证项。

## 有线和离线导出接口建议

如果需要长线、PoE、远程维护或浏览器下载，首选 RJ45 Ethernet：

```text
RJ45 + PHY/MAC 方案
DHCP 获取 IP
HTTP server 继续复用现有 /api/* 和 /recording/*
Wi-Fi 作为备用或删除
```

如果结构允许，强烈建议 PoE：

```text
一根网线同时供电和通信
减少密封壳上的接口数量
提高长期部署可靠性
```

如果只需要离线拷贝，且设备回收后接电脑导出，当前工程已实现的软件方向是 USB 2.0 High-Speed MSC：电脑独占整张 TF，可读写文件；客户主流程为插入 USB 自动导出，安全弹出后拔线自动恢复板端采集。定制板建议：

```text
USB-C device connector, not USB-A device
USB 2.0 HS differential routing and ESD protection
correct CC resistors and VBUS sensing/power isolation
retain UART/JTAG as a separate service interface or test pads
hardware must prevent host and device power paths from back-feeding
storage ownership indicator or explicit export button is recommended
```

USB MSC 和板端 FatFS 绝不能同时访问存储。固件采用“停止任务 -> finalize AVI -> 卸载 FatFS -> USB 独占 -> 安全弹出后拔线自动恢复”的策略，硬件上也不要设计会绕开此互斥的第二主机路径。

## 需要你和我一起确认的问题

1. 最终产品到底需不需要 Wi-Fi？如果不需要，可以删 C6。
2. 存储是 eMMC、焊接 microSD，还是可插拔 TF？
3. 封装后是否允许取出存储卡？
4. 是否使用 PoE？
5. 摄像头模组是否沿用 OV5647，还是换更适合防水结构的模组？
6. 外部接口数量是否限制为一根线？如果是，优先选 RJ45/PoE 还是 USB-C？
7. 是否需要现场手机调试？如果需要，C6 或另一个调试入口要保留。

## 给你的当前设计建议

如果现在要定一版硬件，我建议：

```text
ESP32-P4 + PSRAM + >=16 MB flash
MIPI-CSI 摄像头
RJ45 Ethernet + PoE 或 USB-C 离线导出
板载 eMMC 或工业级 microSD
保留 USB/UART/JTAG 调试测试点
先不放 LCD/触摸/音频/DVP/SPI camera
C6 Wi-Fi 作为可选 BOM：第一版可以保留焊盘或模块位，但最终量产视有线验证结果决定是否贴装
```

这样既能延续当前软件链路，又不会被开发板上的多余接口拖住。
