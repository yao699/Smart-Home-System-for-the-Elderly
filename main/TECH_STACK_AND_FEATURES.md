# ESP32-P4 健康监测与视觉分析项目技术栈及功能实现说明

## 1. 项目概述

本工程是一个基于 **ESP32-P4** 的多传感器融合健康监测原型系统。代码实际实现了以下核心能力：

- 通过 **OV5645 MIPI-CSI 摄像头**持续采集图像；
- 通过板载 **TF 卡 / SDMMC** 保存启动验证文件、摄像头快照和演示图片；
- 通过 **R60ABD1 雷达模块**采集人体存在、运动、距离、心率、呼吸等数据；
- 通过 **ESP8266 AT 模块**连接 WiFi，并使用 SSL HTTP POST 调用百度 AI Studio 大模型接口；
- 将摄像头图像与雷达数据融合后，生成老人姿态、跌倒风险、生命体征风险等中文分析结果；
- 通过 LED 闪烁作为主循环运行状态与异常等待状态提示。

工程入口位于 `main/main.c`，主要业务模块位于 `main/APP/`，底层板级支持代码位于 `components/BSP/`。

## 2. 实际使用到的技术栈

### 2.1 硬件平台

| 类型 | 实际使用对象 | 说明 |
| --- | --- | --- |
| 主控芯片 | ESP32-P4 | 工程目标平台为 `esp32p4`，用于 MIPI 摄像头、SDMMC、UART、I2C、JPEG 编码等任务 |
| 摄像头 | OV5645 MIPI 摄像头 | 通过 MIPI-CSI 和 V4L2 接口采集 YUV422 图像 |
| 存储 | 板载 TF 卡槽 | 使用 SDMMC 4-bit 模式挂载 FAT 文件系统 |
| 雷达 | R60ABD1 雷达模块 | UART2 通信，解析人体存在、运动、距离、心率、呼吸数据 |
| WiFi 模块 | ESP8266 / ATK-MW8266D | UART1 发送 AT 指令，实现 WiFi 入网、SSL 连接和 HTTP 请求 |
| 指示灯 | 板载 LED0 | 用于主循环心跳和故障等待提示 |
| 按键 | BOOT / GPIO35 | 用于切换普通实时分析模式和 TF 卡演示跌倒图片模式 |

### 2.2 软件框架与系统组件

| 技术/组件 | 实际用途 |
| --- | --- |
| ESP-IDF 5.4.0 | 项目基础 SDK，提供 FreeRTOS、驱动、VFS、日志、NVS、组件管理等能力 |
| FreeRTOS | 创建摄像头采集任务、雷达解析任务、WiFi/AI 分析任务、按键任务 |
| CMake / idf_component_register | 管理 `main` 和 `BSP` 组件编译依赖 |
| NVS Flash | 系统启动时初始化 NVS，处理分区满或版本变化情况 |
| ESP Log | 使用 `ESP_LOGI/W/E` 输出初始化、采集、通信和 AI 分析状态 |
| FatFS + VFS | 挂载 `/sdcard`，读写 BOOT、图片、CSV、AI 快照等文件 |
| SDMMC Driver | 使用 ESP32-P4 SDMMC 外设连接板载 TF 卡 |
| UART Driver | UART1 连接 ESP8266，UART2 连接 R60ABD1 雷达 |
| I2C Master Driver | 初始化 SCCB/I2C 总线，供摄像头传感器配置使用 |
| ESP Video | 初始化 MIPI-CSI 视频设备，提供 `/dev/video0` V4L2 接口 |
| ESP Cam Sensor | 摄像头传感器驱动支持，工程依赖版本为 `0.5.x` |
| ESP SCCB Interface | 摄像头 SCCB/I2C 控制接口支持 |
| JPEG Encoder Driver | 将实时 YUV422 亮度数据下采样后编码为灰度 JPG，供 AI 上传 |
| PSRAM Heap Caps | 摄像头 USERPTR 帧缓冲、AI 请求体、Base64 字符串等大内存分配 |
| Linux V4L2 API | 使用 `VIDIOC_QUERYCAP`、`VIDIOC_S_FMT`、`VIDIOC_REQBUFS`、`VIDIOC_DQBUF` 等接口采集摄像头帧 |

### 2.3 第三方/托管组件

根据 `main/idf_component.yml` 与 `dependencies.lock`，工程实际引入以下托管组件：

| 组件 | 版本/约束 | 用途 |
| --- | --- | --- |
| `espressif/esp_video` | `^0.5.1`，锁定为 `0.5.1` | MIPI-CSI 视频采集、V4L2 设备抽象 |
| `espressif/esp_cam_sensor` | `^0.5.2`，锁定为 `0.5.3` | OV 系列等摄像头传感器驱动 |
| `espressif/esp_sccb_intf` | `^0.0.3`，锁定为 `0.0.3` | 摄像头 SCCB/I2C 控制接口 |
| `espressif/esp_h264` | 间接依赖，锁定为 `1.0.4` | `esp_video` 依赖项，本业务代码未直接调用 H.264 编码 |
| `espressif/cmake_utilities` | 间接依赖，锁定为 `0.5.3` | ESP 组件构建辅助工具 |

## 3. 目录结构与代码职责

```text
33_final_fusion_demo/
├── CMakeLists.txt
├── sdkconfig
├── dependencies.lock
├── main/
│   ├── main.c                         # 应用入口、SD 卡挂载、摄像头/雷达/WiFi 启动
│   ├── CMakeLists.txt                 # main 组件源码与依赖声明
│   ├── idf_component.yml              # ESP-IDF 组件管理依赖
│   └── APP/
│       ├── MIPI_CAM/
│       │   ├── app_video.c/.h         # ESP Video / V4L2 初始化封装
│       │   └── mipi_cam.c/.h          # 摄像头采集、快照、YUV->JPG、TF 卡写入
│       ├── RADAR/
│       │   └── radar_uart.c/.h        # R60ABD1 雷达串口协议解析
│       └── ESP8266_AT/
│           └── esp8266_at.c/.h        # ESP8266 AT、WiFi、HTTPS、AI 请求
├── components/
│   └── BSP/
│       ├── LED/                       # LED0 GPIO 驱动
│       ├── MYIIC/                     # I2C 总线初始化
│       └── LCD/                       # LCD/MIPI 相关 BSP，当前 main 中跳过 LCD 初始化
└── managed_components/                # ESP-IDF 组件管理器拉取的依赖组件
```

## 4. 启动流程实现

系统入口为 `app_main()`，实际启动流程如下：

1. 初始化 NVS；
2. 初始化 LED；
3. 初始化 I2C 总线；
4. 跳过 MIPI LCD 初始化，避免未接屏幕时卡死；
5. 挂载板载 TF 卡到 `/sdcard`；
6. 在 TF 卡写入 `/sdcard/BOOT.TXT` 作为启动验证文件；
7. 初始化 OV5645 MIPI 摄像头；
8. 主循环中第一次进入时启动雷达 UART 解析任务；
9. 主循环中第一次进入时启动 ESP8266 AT WiFi 测试与 AI 周期分析任务；
10. 主循环每 5 秒翻转 LED 并打印系统存活日志。

如果 TF 卡挂载失败，程序不会启动摄像头，而是进入 LED 闪烁等待状态，并提示检查 TF 卡、FAT32 格式和上电前插卡状态。

如果摄像头初始化失败，程序进入 LED 闪烁等待状态，并提示检查摄像头 FPC 方向和连接。

## 5. TF 卡存储功能实现

### 5.1 SDMMC 引脚

`main/main.c` 中配置了正点原子 ESP32-P4 板载 TF 卡槽的 SDMMC 引脚：

| 信号 | GPIO |
| --- | --- |
| CLK | GPIO43 |
| CMD | GPIO44 |
| D0 | GPIO39 |
| D1 | GPIO40 |
| D2 | GPIO41 |
| D3 | GPIO42 |

### 5.2 挂载方式

代码使用：

- `SDMMC_HOST_DEFAULT()` 初始化 SDMMC Host；
- `SDMMC_SLOT_CONFIG_DEFAULT()` 初始化 Slot；
- `slot_config.width = 4` 使用 4-bit SDMMC 模式；
- `esp_vfs_fat_sdmmc_mount()` 将 FAT 文件系统挂载到 `/sdcard`；
- `sdmmc_card_print_info()` 打印卡信息。

### 5.3 文件用途

| 文件路径 | 生成/读取位置 | 用途 |
| --- | --- | --- |
| `/sdcard/BOOT.TXT` | `main.c` | SD 卡挂载成功验证文件 |
| `/sdcard/CAM.CSV` | `mipi_cam.c` | 摄像头原始帧元数据，包含帧号、宽高、字节数、checksum、亮度统计等 |
| `/sdcard/F0001.YUV` 等 | `mipi_cam.c` | 调试用原始 YUV 帧；当前交付配置 `SAVE_FRAME_MAX_COUNT` 为 0，默认不保存隐私原始画面 |
| `/sdcard/AI_1.JPG`、`AI_2.JPG`、`AI_3.JPG` | `esp8266_at.c` + `mipi_cam.c` | 普通模式下每轮 AI 分析采集的三张实时快照 |
| `/sdcard/DEMO_FALL.JPG` / `DEMO_F~1.JPG` / `FALL.JPG` | `esp8266_at.c` | 演示跌倒图片模式下读取的 TF 卡图片 |

## 6. 摄像头采集功能实现

### 6.1 视频初始化

摄像头初始化由 `mipi_cam_init()` 完成，关键步骤：

1. 调用 `mipi_dev_bsp_enable_dsi_phy_power()` 保留 MIPI 设备/PHY 电源初始化；
2. 调用 `app_video_main(bus_handle)` 初始化 ESP Video；
3. 调用 `app_video_open(0)` 打开 `/dev/video0`；
4. 调用 `app_video_init(fd, APP_VIDEO_FMT_YUV422)` 设置采集格式为 YUV422；
5. 创建 `mipi_cam_capture` 任务并固定运行在 Core 1。

### 6.2 V4L2 USERPTR 采集

摄像头采集任务 `mipi_cam_capture_task()` 使用 V4L2 USERPTR 方式采集图像：

- `VIDIOC_G_FMT` 获取当前格式；
- `VIDIOC_REQBUFS` 请求 2 个 USERPTR 缓冲；
- `VIDIOC_QUERYBUF` 查询缓冲区长度；
- `heap_caps_aligned_alloc()` 在 PSRAM 中分配 64 字节对齐帧缓冲；
- `VIDIOC_QBUF` 将缓冲提交给驱动；
- `camera_stream_start()` 调用 `VIDIOC_STREAMON` 开始采集；
- 循环 `VIDIOC_DQBUF` 取出帧，分析并处理后再 `VIDIOC_QBUF` 归还缓冲。

### 6.3 帧统计

每帧会通过 `frame_analyze_sampled()` 做抽样统计：

- FNV 风格 checksum；
- 平均亮度；
- 最小值；
- 最大值；
- 抽样数量。

前 10 帧以及之后每隔 30 帧会打印帧信息和 FPS。

### 6.4 AI 快照生成

普通 AI 实时分析并不会上传完整原始视频，而是按需抽帧生成灰度 JPG：

1. `esp8266_at.c` 调用 `mipi_cam_save_snapshot_jpg(path, timeout)` 请求快照；
2. 摄像头采集任务在下一帧到来时检测快照请求；
3. `downsample_yuv422_packed_to_gray()` 从 YUV422 packed 数据中提取偶数字节亮度 Y，并下采样为 320x240 灰度图；
4. `jpeg_encoder_process()` 使用硬件/驱动 JPEG 编码器压缩为 JPG；
5. 写入 `/sdcard/AI_1.JPG`、`/sdcard/AI_2.JPG`、`/sdcard/AI_3.JPG`。

该设计避免长时间保存原始视频，只保存用于 AI 分析的短时低分辨率快照，有利于降低 TF 卡写入压力和网络上传数据量。

## 7. 雷达数据采集功能实现

雷达模块代码位于 `main/APP/RADAR/radar_uart.c`。

### 7.1 UART 配置

| 项目 | 配置 |
| --- | --- |
| UART 端口 | UART2 |
| TX | GPIO11 |
| RX | GPIO12 |
| 波特率 | 115200 |
| 数据位 | 8 bit |
| 校验 | 无 |
| 停止位 | 1 bit |
| RX 缓冲区 | 2048 字节 |

### 7.2 协议解析

雷达帧解析使用状态机，按以下结构解析：

```text
0x53 0x59 CTRL CMD LEN_H LEN_L DATA... CHECKSUM 0x54 0x43
```

状态机依次等待帧头、控制字、命令字、长度、数据、校验和、帧尾。校验失败会增加 `checksum_error` 计数。

### 7.3 已解析的数据

`radar_state_t` 保存当前雷达状态，字段包括：

| 字段 | 含义 |
| --- | --- |
| `presence` | 是否有人 |
| `motion` | 运动状态，包含无人、静止、活跃等 |
| `body_motion` | 体动强度，0-100 |
| `distance_cm` | 目标距离，单位 cm |
| `heart_rate` | 心率，次/分 |
| `breath_rate` | 呼吸，次/分 |
| `frame_count` | 已解析帧数 |
| `checksum_error` | 校验错误次数 |

### 7.4 启用命令

启动后会发送以下功能使能命令：

- 人体存在检测；
- 呼吸检测；
- 心率检测；
- 睡眠相关帧。

雷达状态通过 `radar_get_state()` 暴露给 AI 分析模块使用。

## 8. ESP8266 WiFi 与 AI 分析功能实现

ESP8266 和 AI 分析逻辑位于 `main/APP/ESP8266_AT/esp8266_at.c`。

### 8.1 UART 与 WiFi 配置

| 项目 | 配置 |
| --- | --- |
| UART 端口 | UART1 |
| TX | GPIO26 |
| RX | GPIO27 |
| 波特率 | 115200 |
| WiFi 模式 | Station 模式 |
| 连接方式 | ESP8266 AT 指令 |

程序通过以下 AT 指令完成基础 WiFi 初始化：

- `AT` 检测模块响应；
- `ATE0` 关闭回显；
- `AT+CWMODE=1` 设置 STA 模式；
- `AT+CIPMUX=0` 设置单连接模式；
- `AT+CWJAP="ssid","password"` 连接 WiFi；
- `AT+CIFSR` 查询 IP。

当前 `main.c` 中调用的是：

```c
esp8266_at_wifi_test_async("testwifi", "12345678");
```

### 8.2 AI Studio 接口

代码中配置的 AI 服务参数：

| 项目 | 值 |
| --- | --- |
| Host | `aistudio.baidu.com` |
| Port | `443` |
| Path | `/llm/lmapi/v3/chat/completions` |
| 文本模型 | `ernie-4.0-turbo-8k-latest` |
| 视觉语言模型 | `ernie-4.5-turbo-vl` |
| 连接方式 | `AT+CIPSTART="SSL",...` |
| 请求方式 | HTTP/1.1 POST + JSON |

注意：源码中存在 AI Studio Access Token 宏定义。正式提交、展示或开源前，应将真实 Token 移出源码，改为 NVS、Kconfig、环境注入或独立配置文件，并避免截图泄露。

### 8.3 HTTP 请求发送

AI 请求由代码手动构造 HTTP 报文：

1. 构造 JSON body；
2. 构造 HTTP header，包含 `Authorization: Bearer <token>`、`Content-Type: application/json` 和 `Content-Length`；
3. 使用 `AT+CIPSTART="SSL"` 建立 SSL 连接；
4. 使用 `AT+CIPSEND=<chunk_size>` 分块发送请求，每块最大 1024 字节；
5. 读取 ESP8266 返回数据，判断 `HTTP/1.1 200`、`choices`、`content`；
6. 从响应中提取并打印 AI 分析结果。

## 9. AI 分析模式实现

系统实际实现了两种 AI 分析模式。

### 9.1 普通实时三快照分析模式

这是默认模式。流程如下：

1. 清理上一次的 `/sdcard/AI_1.JPG`、`AI_2.JPG`、`AI_3.JPG`；
2. 立即采集第一张实时摄像头灰度 JPG；
3. 等待 10 秒；
4. 采集第二张；
5. 再等待 10 秒；
6. 采集第三张；
7. 读取三张 JPG 到内存；
8. 分别 Base64 编码；
9. 读取当前雷达数据；
10. 构造多模态 JSON，请求视觉语言模型分析约 20 秒内的姿态变化；
11. AI 需要输出心率、呼吸、三帧画面判断、风险等级和建议。

该模式将视觉信息和雷达生命体征数据融合，用于老人跌倒、低位静止、无人场景等风险提示。

### 9.2 TF 卡演示跌倒图片分析模式

按下 BOOT 键后，系统会切换 `s_ai_demo_fall_mode`，立即触发下一轮 AI 分析。

演示模式流程如下：

1. 在 TF 卡中查找演示图片：
   - `/sdcard/DEMO_FALL.JPG`
   - `/sdcard/DEMO_F~1.JPG`
   - `/sdcard/FALL.JPG`
   - 或目录中包含 `DEMO_F` 和 `.JPG` 的文件；
2. 读取图片到内存；
3. Base64 编码；
4. 读取雷达状态；
5. 调用视觉语言模型分析是否存在老人跌倒、躺倒、异常姿态或需要人工查看的风险；
6. 输出中文风险提示。

该模式适合现场演示或没有真实跌倒场景时进行功能验证。

## 10. BSP 底层功能实现

### 10.1 LED 驱动

`components/BSP/LED/led.c` 完成 LED0 GPIO 初始化：

- 设置为输入输出模式；
- 禁用上下拉；
- 默认关闭 LED；
- `main.c` 中周期性调用 `LED0_TOGGLE()` 作为运行心跳。

### 10.2 I2C 驱动

`components/BSP/MYIIC/myiic.c` 初始化 I2C Master 总线，并导出全局 `bus_handle`。

摄像头初始化时，`app_video_main(bus_handle)` 会复用该 I2C/SCCB 总线句柄，避免重复初始化 SCCB 总线。

### 10.3 LCD 相关代码

`components/BSP/LCD/` 中包含 RGB/MIPI LCD 驱动代码，但当前 `main.c` 明确跳过 LCD 初始化。这样做是为了避免未接屏幕时程序卡死。本项目当前重点是摄像头采集、TF 卡存储、雷达和 AI 分析，而不是本地屏幕显示。

## 11. 任务与并发模型

| 任务/流程 | 创建位置 | 作用 |
| --- | --- | --- |
| `app_main` 主循环 | ESP-IDF 启动 | 初始化系统，并周期性 LED 心跳 |
| `mipi_cam_capture` | `mipi_cam_init()` | 持续采集摄像头帧、处理快照请求、统计帧信息 |
| `radar_parser` | `radar_uart_start()` | 持续读取 UART2 并解析雷达帧 |
| `esp8266_wifi` | `esp8266_at_wifi_test_async()` | 连接 WiFi，并周期性发起 AI 分析 |
| `ai_demo_key` | `ai_demo_key_start_once()` | 轮询 BOOT 按键，切换普通/演示 AI 模式 |

快照请求通过 `SemaphoreHandle_t` 互斥保护，ESP8266 AI 任务发起请求，摄像头采集任务在实时帧流中完成快照落盘。

## 12. 数据流说明

### 12.1 普通实时分析数据流

```text
OV5645 摄像头
  -> MIPI-CSI / ESP Video / V4L2
  -> PSRAM USERPTR 帧缓冲
  -> YUV422 提取亮度 + 下采样
  -> JPEG 编码
  -> /sdcard/AI_1.JPG, AI_2.JPG, AI_3.JPG
  -> 读取文件 + Base64 编码

R60ABD1 雷达
  -> UART2
  -> 帧协议解析
  -> radar_state_t

三张图片 Base64 + 雷达数据
  -> ESP8266 UART1 AT 指令
  -> SSL HTTP POST
  -> 百度 AI Studio 视觉语言模型
  -> 中文风险分析结果日志输出
```

### 12.2 演示图片分析数据流

```text
/sdcard/DEMO_FALL.JPG
  -> 读取文件
  -> Base64 编码

R60ABD1 雷达状态
  -> radar_get_state()

演示图片 + 雷达数据
  -> ESP8266 SSL HTTP POST
  -> 百度 AI Studio 视觉语言模型
  -> 跌倒/异常姿态风险提示
```

## 13. 关键引脚汇总

| 功能 | 引脚 |
| --- | --- |
| SDMMC CLK | GPIO43 |
| SDMMC CMD | GPIO44 |
| SDMMC D0 | GPIO39 |
| SDMMC D1 | GPIO40 |
| SDMMC D2 | GPIO41 |
| SDMMC D3 | GPIO42 |
| ESP8266 UART1 TX | GPIO26 |
| ESP8266 UART1 RX | GPIO27 |
| 雷达 UART2 TX | GPIO11 |
| 雷达 UART2 RX | GPIO12 |
| AI 模式切换 BOOT 键 | GPIO35 |
| 摄像头 SCCB/I2C | 由 Kconfig / sdkconfig 中的 `CONFIG_EXAMPLE_MIPI_CSI_SCCB_*` 配置决定 |
| 摄像头 RESET/PWDN | 由 Kconfig / sdkconfig 中的 `CONFIG_EXAMPLE_MIPI_CSI_CAM_SENSOR_*` 配置决定 |

## 14. 当前代码中的重要配置与约束

| 配置 | 当前值/行为 | 影响 |
| --- | --- | --- |
| `SAVE_FRAME_MAX_COUNT` | `0` | 默认不保存原始 YUV，保护隐私并减少 TF 卡写入 |
| AI 快照尺寸 | 320x240 | 降低 JPEG 文件大小和上传耗时 |
| AI JPEG 质量 | 70 | 在图像清晰度和网络传输之间折中 |
| AI 分析周期 | 10000 ms | 每轮分析后等待，按键可打断等待 |
| 实时快照间隔 | 10000 ms | 三张图约覆盖 20 秒状态变化 |
| AI 单次发送分块 | 1024 字节 | 适配 ESP8266 AT `CIPSEND` 分块发送 |
| 演示图片最大大小 | 220 KB | 避免 Base64 和 HTTP 请求体占用过大内存 |
| 摄像头缓冲数 | 2 | 使用双缓冲 USERPTR 采集 |
| 摄像头缓冲内存 | PSRAM，64 字节对齐 | 适配大帧缓存和缓存对齐要求 |

## 15. 功能总结

本项目实际实现的是一个“摄像头 + 雷达 + AI 大模型”的健康风险监测融合演示系统：

- **摄像头负责视觉证据**：通过 MIPI-CSI 采集图像，并按需生成低分辨率 JPG；
- **雷达负责生命体征和存在检测**：解析人体存在、体动、距离、心率、呼吸；
- **TF 卡负责中间存储**：保存启动文件、快照、演示图片和调试数据；
- **ESP8266 负责联网**：通过 AT 指令连接 WiFi 和 HTTPS 服务；
- **AI Studio 负责智能分析**：融合三帧图像和雷达数据，输出中文风险等级与建议；
- **LED/按键负责交互**：LED 显示系统存活或故障等待，BOOT 键切换演示模式。

整体架构适合用于老人看护、跌倒风险演示、生命体征融合分析等嵌入式 AIoT 场景。正式产品化时，建议进一步完善 Token 安全存储、网络错误重试策略、隐私合规提示、异常状态机、功耗控制和离线告警机制。
