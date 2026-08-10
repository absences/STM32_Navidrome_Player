# STM32H743 Navidrome Player

基于 STM32H743、ESP8266-01S 和 WM8978G 的独立网络音乐播放器。固件通过 ESP-AT 接入 Wi-Fi，从 Navidrome（Subsonic API）随机获取并流式播放 MP3；SH1106/SSD1306 OLED 显示中文曲名、下一曲、播放进度和音量。

## 当前功能

- ESP8266-01S：上电时以 `115200 8N1` 完成 AT 握手和联网，随后切换到 `921600` 传输音频。
- Navidrome：获取随机歌曲、预取下一曲，并通过 HTTP/1.0 流式接收 MP3。
- 音频：使用 minimp3 软件解码，经 SAI1 输出到 WM8978G；外部 32 MiB SDRAM 用作播放缓存。
- 显示：128 × 64 OLED，支持 UTF-8 中文曲名（字库位于 `music_controller/cjk16.bin`）。
- 控制：旋钮调节音量/按下暂停或继续，CONFIRM 下一曲，BACK 返回上一曲。

## 硬件接线

### OLED 前面板

排针按 PCB 丝印从左到右为：

```text
CON  SDA  SCL  PSH  TRA  TRB  BAK  GND  VCC
```

| OLED 前面板 | STM32H743 | 功能 |
| --- | --- | --- |
| CON | PC4 | 下一曲 |
| SDA | PB9 | OLED I²C 数据 |
| SCL | PB8 | OLED I²C 时钟 |
| PSH | PC5 | 旋钮按压、播放/暂停 |
| TRA | PA4 | 旋转编码器 A 相 |
| TRB | PA5 | 旋转编码器 B 相 |
| BAK | PA6 | 上一曲 |
| GND | GND | 公共地 |
| VCC | 3.3V | 模块电源，不可接 5V |

OLED 使用 7-bit I²C 地址 `0x3C`。驱动按 SH1106 的 132 列显存和 2 列偏移刷新，也兼容多数 SSD1306 模块。控制输入使用 STM32 内部上拉，低电平触发。

### ESP8266-01S

| ESP8266-01S | STM32H743 |
| --- | --- |
| 3V3 | 稳定的外部 3.3V 电源 |
| GND | GND（必须与 STM32 共地） |
| TX | PB11 / USART3_RX |
| RX | PB10 / USART3_TX |
| EN | 通过 10 kΩ 上拉到 3.3V |

正常启动时 IO0、IO2 应保持高电平。建议使用至少 500 mA 的 3.3V 电源，并在模块附近放置 470 µF、10 µF 和 0.1 µF 去耦电容。`esp8266_firmware/` 中提供当前使用的 1 MiB ESP-AT 固件及刷写前备份。

### WM8978G

| WM8978G | STM32H743 |
| --- | --- |
| SCL | PD12 / I2C4_SCL |
| SDA | PD13 / I2C4_SDA |
| MCK | PE2 / SAI1_MCLK_A |
| WS | PE4 / SAI1_FS_A |
| SCK | PE5 / SAI1_SCK_A |
| SD | PE6 / SAI1_SD_A |
| SD_EXT | PE3 / SAI1_SD_B（当前不使用） |

模块的 `+5V`、`3V3` 和 `GND` 必须分别正确供电并与 STM32 共地。WM8978 使用 7-bit 地址 `0x1A`。

## 网络配置

先复制示例文件：

```powershell
Copy-Item music_controller\Src\wifi_config.example.h music_controller\Src\wifi_config.h
Copy-Item music_controller\Src\navidrome_config.example.h music_controller\Src\navidrome_config.h
```

然后填写 Wi-Fi、Navidrome 局域网地址和 Subsonic 账户。两个实际配置文件已被 `.gitignore` 排除，不会提交到 Git。当前实现使用普通 TCP/HTTP，STM32 必须能直接访问配置的 Navidrome 地址；请勿把服务暴露到不可信网络。

## 编译固件

构建脚本默认使用 STM32CubeIDE 2.2.0 附带的 GNU Arm 14.3 工具链。在仓库根目录运行：

```powershell
.\build.cmd
```

产物位于 `music_controller/build/`：

- `music_controller.elf`：含调试符号的 ELF 文件
- `music_controller.hex`：供 STM32CubeProgrammer/ST-Link 烧录
- `music_controller.bin`：原始固件镜像

连接 ST-Link 后可构建、烧录、校验并复位：

```powershell
.\flash.cmd
```

只烧录已有固件：

```powershell
.\flash.cmd --no-build
```

若默认路径中没有工具链，可修改根目录 `build.cmd` 中的 `TOOLCHAIN`。烧录工具也可通过环境变量 `STM32_PROGRAMMER_CLI` 指定完整路径。

## 启动状态

OLED 会依次显示 SDRAM、ESP、网络和音频状态。常见提示：

| 提示 | 含义 |
| --- | --- |
| `SDRAM: NO` | FMC/SDRAM 初始化或自检失败 |
| `ESP: NO` | ESP-AT 握手失败 |
| `NAV: NO` | Wi-Fi 未连接，无法进入在线播放 |
| `AUDIO: NO` | WM8978 未在 `0x1A` 应答 |
| `NO SONG` | Navidrome 随机歌曲请求失败或没有可播放歌曲 |
