# 智显灵境——多模态交互终端

基于两片国产安路 EG4S20 FPGA 的 HDMI 多媒体与交互工程。系统以 480×800 竖屏 LCD 为统一操作入口，集成本地与网络音视频播放、图像处理、数字音效、FM 广播分析与时移、频谱显示和环境感知。

## 完整工程下载

**[下载 HX_Final.zip](release/HX_Final.zip)**

压缩包为团队最终整理的完整交付包，约 62.82 MB，包含 A/B 板工程、配套 bit、PC 发送程序、TF 视频转换工具及 FFmpeg 运行依赖，不含 TF 卡素材和演示视频。请下载到本地后解压使用。

仓库中的源码便于在线浏览；超过 GitHub 单文件限制的 `ffmpeg.exe` 随完整 ZIP 提供。完整 ZIP 内的文件内容保持原样。

## 双板分工

| 单元 | 主要职责 |
| --- | --- |
| A 板 | TF 卡媒体读取、SDRAM 缓存仲裁、图片与视频调度、多图层合成、LCD 交互及 HDMI 音视频输出 |
| B 板 | 以太网接收、RDA5807M 控制与扫描、麦克风/FM/环境光 ADC 采集、共享音频 DSP 与 FFT、音频特征提取及 HUB75 显示 |
| 板间链路 | 26 根信号线，交换媒体数据、PCM、控制请求、执行反馈、频谱与传感器状态 |

## 主要功能

- 图片播放：BMP 读取、缩放、亮度与对比度调整、标题正文分步呈现、热点与局部放大。
- 图片转场：直接切换、淡入淡出、左右横向擦除、纵向擦除、中心展开和百叶窗。
- 滑杆对比：原图、灰度、亮度增强、对比度增强与轻量化边缘显示。
- 本地音视频：WAV PCM 播放及 FVP4/RLE 轻量化 TF 视频，采用帧缓存与音频主时基调度。
- 网络音视频：接收 RGB565 图像与 PCM，通过双板缓存、流控和显示通路播放。
- 数字音效：共享五段 EQ、AGC、动态压缩、峰值限幅与噪声门；支持原声旁路及音效预设。
- FM 广播：触控调谐、音量和静音控制、逐频点 RSSI 扫描、候选电台筛选与图谱显示。
- FM 时移：SDRAM 循环缓存、暂停续播、回退及恢复直播。
- 音画交互：麦克风实时频谱、拍手换图、分层演示推进，以及实际节拍和音频特征驱动的白框、星星效果。
- 环境感知：环境光分级、平滑 RGB 数字增益调节及人体状态参与展示控制。

## 默认配置

| 项目 | 配置或设计参数 |
| --- | --- |
| FPGA | 2 × EG4S20BG256 |
| HDMI 显示 | 640×480 |
| LCD | 480×800 竖屏 |
| TF 视频 | FVP4/RLE，320×240，20 帧/秒 |
| 网络视频 | 640×480，20 帧/秒，RGB565 |
| PCM | 48 kHz；TF 视频为16位单声道，网络音频为16位立体声 |
| EQ | 100、400、1500、5000、12000 Hz 五段 |
| FFT | 1024 点，64 频带 |
| FM 扫描 | 87.5～108.0 MHz，100 kHz 步进，共206点 |
| FM 历史区 | 1 MiB；按48 kHz、16位单声道计算约10.92秒 |

PC 工具负责 MP4 解码、素材转换和网络发送，FPGA 完成规定格式的数据解析、缓存、实时处理、交互与输出；TF 视频采用 FVP 文件。

## 目录与工程入口

```text
HX_Final/
├─ BOARD_A/                 A 板 RTL、IP、ROM、约束与配套 bit
├─ BOARD_B/                 B 板 RTL、IP、ROM、约束与配套 bit
├─ tools/
│  ├─ PC_SENDER/            网络发送上位机与 C++ 源码
│  └─ TF_VIDEO_CONVERTER/   TF 视频转换脚本
├─ release/HX_Final.zip     完整交付包，包含工具运行依赖
└─ README.md
```

TD 工程入口：

- A 板：[`BOARD_A/src/td_project/HX4S20_Panel_Player.al`](BOARD_A/src/td_project/HX4S20_Panel_Player.al)
- B 板：[`BOARD_B/src/td_project/HX4S20_Panel_Player.al`](BOARD_B/src/td_project/HX4S20_Panel_Player.al)

器件选择 `EG4S20BG256`，顶层为 `top`。工程记录的 TD 版本为 `6.2.168116`。

## 配套固件与运行

- A 板：[`BOARD_A/best_result/top_merged.bit`](BOARD_A/best_result/top_merged.bit)
- B 板：[`BOARD_B/best_result/top_merged.bit`](BOARD_B/best_result/top_merged.bit)

下载时每次只添加当前板对应的一个 bit；A 板下载后按白色 KEY1 复位。SRAM 下载用于本次通电运行，Flash 固化按开发板已验证流程操作。

两板沿用既有26线通信接法并共地；外设接线以各板 `constraints_source/pin.adc` 和实际 PCB 为准。FM 模拟音频通过调理电路接入 B 板 ADC_CH3，HUB75 屏使用独立5V电源并正确共地。

TF 卡使用已有 FAT32 媒体卡，视频文件放在根目录并命名为 `VIDEO.FVP`。更换素材可使用独立 `tools/TF_VIDEO_CONVERTER` 工具。视频、BMP 和 WAV 演示素材单独准备。

网络播放时，PC 有线网卡配置为 `192.168.240.2/24`，B 板为 `192.168.240.1`，使用千兆有线链路。运行 PC 发送程序，将素材放入程序提示的 `Media folder`，LCD 视频页选择 NET 后发送。

## 工程版本

本仓库基于 `HX_Final` 最终交付包整理，保留 A/B 板 RTL、约束、IP、初始化数据和配套 bit 的原始内容。仓库说明和 Git 元数据独立添加，原交付 ZIP 保持不变。
