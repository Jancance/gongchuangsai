# 预览开关 + CH340 视觉链路（2026-11-07）

## 一、先定死一个事实：CSV 遥测只从 USART1 出来

代码核对（`tmp/integration12_stage2_vision_20261107/project`）：

| 发送点 | 走哪条路 | 内容 |
|---|---|---|
| `main.c:2616` OD1 / `main.c:2674` PM1 / `vehicle_mission.inc:570` LR1 | `App_DebugSendString` -> `USART1_SendString` -> `USART_ITConfig(USART1,...)` | ASCII CSV |
| `integration10_complete.inc:200` | `USART6_VisionSend(command,3U)` | 只有 3 字节 `AA xx BB` |

全工程 `USART6_VisionSend(` 只有一个调用点，且长度固定 3；`USART1_SendString` 内部写死 `USART1`。
所以电脑在 COM14 上收到 12292 字节 `TMQ,/PM1,/LR1,/OD1,`，只能是 **CH340 当时接在 PA9(USART1 TX)** 上，
不是 USART6。结论：之前的接线接到了调试口，PC7 悬空 -> `g_uart6_rx_errors` 累积、`g_vis_rx_bytes` 涨的是噪声。

判据很好用：**接对了就应该"安静"**（USART6 只在被请求时发 3 字节）；一直刷 CSV 就是接在调试口。

## 二、香橙派侧脚本

`scripts/pi_vision_link_check.sh`：查 USB 枚举 -> 补 ch341 驱动 -> 停 brltty -> 建 /dev/ttyVISION 固定名 -> 被动监听 5s 并自动判定
（安静 / CSV=接错口 / AA..BB=正确）。

视觉程序启动参数：`--serial-device /dev/ttyVISION --serial-baud 115200`
（`turntable_serial.py` 默认 `/dev/ttyS7` 是香橙派自己 40pin 的 UART，插 USB-CH340 后必须改）。

## 三、VNC 图传预览

原本就有：`main.py` 每帧 `cv2.imshow`，所以在香橙派桌面上跑程序、用 RealVNC 看，就能看到画面和叠加。
缺的是"比赛时关掉"。本次加了两个参数（`vision_app/code/main.py`，12 处改动）：

- `--display window|off`：off 不建窗口、不绘制状态字、不传图；检测/串行逻辑完全不变
- `--display-every N`：窗口每 N 帧刷新一次，降 VNC 带宽

改动清单：全局 `display_mode/display_every/display_counter`、`--display` / `--display-every`、
`show_image()` / `pump_ui()` 两个封装、`namedWindow` 块、任务码窗口、断线等待分支（off 时连黑图都不生成）、
主通路 `draw_status/draw_measurement/imshow`、两处 `waitKey`、`destroyAllWindows`。

验证（本机 Windows，cv2 5.0.0 / numpy 2.4.6）：
- `python -m py_compile main.py` 通过
- `python main.py --check --display off` -> `Preview: off (every 1 frame(s))`，功能清单正常
- `--display off --device /dev/video99` 空跑 7s：无崩溃、无窗口、状态按 1s 打印
- `--display window --display-every 2 --device /dev/video99` 空跑 6s：窗口模式无回归

## 四、还没做的

- 改后的 `main.py` 只在 Windows 副本上，需要同步到香橙派实际部署目录再跑
- 帧率 A/B 要在香橙派上实测：同一场景 `--display window` 与 `--display off` 各跑 30s，比末端的 `measured_fps`
