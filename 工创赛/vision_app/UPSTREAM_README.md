# 创源启明 工训视觉开源

面向工训场景的 Python / OpenCV 视觉项目，通过 USB 摄像头采集画面，与 F407 功能板进行串口通信。支持物料颜色与抓取位置识别、转盘外圆圆心定位、三圆环工位识别、叠放位中环与基准线识别，以及两点像素测量。

## 快速运行

当前程序使用 **Linux 图形桌面和 V4L2 摄像头接口**，需要能显示 OpenCV 窗口。依赖 Python 3、OpenCV、NumPy 和 pyserial；转盘参数工具的中文面板还需要 Pillow 和 Noto CJK 字体，缺少时使用英文面板。

Ubuntu / Debian 系统可安装以下依赖：

```bash
sudo apt install python3-opencv python3-numpy python3-serial python3-pil fonts-noto-cjk
```

在项目根目录打开终端，启动主程序：

```bash
python3 code/main.py
```

默认摄像头为 `/dev/video0`，图像大小为 `1280×720`，目标帧率为 `30 FPS`；串口为 `/dev/ttyS7`，波特率为 `115200`。需要更换设备时，通过参数指定，例如：

```bash
python3 code/main.py --device /dev/video2 --serial-device /dev/ttyUSB0 --serial-baud 115200
python3 code/main.py --help
```

摄像头节点需支持视频采集，运行用户需有摄像头和串口的访问权限。程序启动后处于 `IDLE` 待机状态，可通过窗口按键或串口命令切换模式。

也可以使用带重复启动检查的入口：

```bash
bash code/启动视觉.sh
```

该脚本固定使用 `/usr/bin/python3`，依赖需安装在该解释器环境中，也可在命令后追加主程序参数。桌面快捷方式和登录自启动需要在目标机器上另行配置，复制项目不会自动安装。

## 主程序按键

先点击摄像头预览窗口使其获得焦点，字母按键不区分大小写。

| 按键 | 功能 |
|---|---|
| `A` | 转盘识别：检测真实外圆并定位圆心 |
| `D` | 像素测量：鼠标左键选取两点，显示 X / Y 像素差和直线距离 |
| `R` | 三圆环工位识别：检测圆心、工位角度及偏差 |
| `S` | 叠放位识别：检测中环，结合上下基准线估计工位角度 |
| `M` | 物料抓取识别：识别左、右、下抓取区颜色，启动一次新的上报周期 |
| `I` | 返回待机，停止识别和结果上报 |
| `Q` / `Esc` | 退出程序 |

测量模式下，第一次点击冻结画面并选取起点，第二次点击选取终点；第三次点击清空结果并恢复实时画面。测量单位为像素。

按 `A`、`R`、`S` 进入识别模式后不会立即上报，需要串口发送 `AA 14 BB` 开启连续上报，或发送 `AA 15 BB` 查询一次。物料模式需先检测到转动，再在接近停稳且抓取位置有效时上报一次；下一轮需再次按 `M` 或发送 `AA 02 BB`。

## 标定与参数工具

先退出主程序释放摄像头，再单独运行需要的工具。以下命令均在项目根目录执行；修改后按 `S` 保存，按 `Q` / `Esc` 退出，随后重启主程序加载新配置。

| 工具与启动命令 | 主要操作 |
|---|---|
| 颜色标定：`python3 code/颜色标定.py` | `1～6` 选择红、黄、蓝、绿、黑、浅蓝；左键拖框采样，滑条微调 HSV；`U` 撤销采样，`C` 清空当前颜色采样，`R` 恢复当前颜色预设，`S` 保存 |
| 转盘参数设置：`python3 code/转盘参数设置.py` | 拖动滑条调整外圆检测；`A` 自动诊断并填入建议值，`Tab` 切换参数页，空格冻结画面，`P` 显示采样点，`M` 显示边缘掩膜，`R` 重载已保存参数，`D` 恢复默认值，`S` 保存 |
| 物料抓取区域标定：`python3 code/物料抓取区域标定.py` | 已有区域需重画时先按 `R`，再依次框选活动区、左抓取区、右抓取区、下抓取区；`U` 撤销，`S` 保存。三个抓取区需位于活动区内且互不重叠 |
| 叠放位直线标定：`python3 code/叠放位直线标定.py` | 自动寻找中环，也可左键点选；确认最外圈和上下两条基准线正确且达到 `STABLE` 后按 `S` 保存，`R` 重新寻找 |

叠放位工具默认外圈直径为 `95 mm`，上下基准线到中环圆心各 `75 mm`，可用 `--diameter-mm`、`--line-offset-mm` 调整。保存要求上下两条线同时稳定；该工具只标定搜索范围，不修改工位零点和纠偏矩阵。

## 配置文件怎么用

配置集中在 [code/config](code/config)，主程序在启动时读取。优先使用对应工具调整并保存；手动修改 JSON 后同样需要重启主程序。检测器模块由主程序调用，无需单独运行。

| 配置文件 | 用途 | 修改方式 |
|---|---|---|
| [camera_processing.json](code/config/camera_processing.json) | 控制主程序是否进行畸变矫正 | 手动修改 `undistort_enabled`：`true` 开启，`false` 使用原始画面 |
| [realtek_rgb_camera.yaml](code/config/realtek_rgb_camera.yaml) | 相机内参、畸变系数、矫正矩阵和标定分辨率 | 更换为实际相机的标定结果；开启矫正时，运行分辨率须与标定一致 |
| [material_color_thresholds.json](code/config/material_color_thresholds.json) | 六种物料的 HSV 阈值，供物料抓取和三物料辅助定位共用 | 使用颜色标定工具保存 |
| [material_pickup_detector.json](code/config/material_pickup_detector.json) | 活动区、三个抓取区、物料形态筛选和转动 / 接近停稳判定 | 区域通过抓取区域工具保存；形态与运动阈值手动调整 |
| [turntable_detector.json](code/config/turntable_detector.json) | 转盘外圆粗搜索、圆弧精修和稳定判定 | 使用转盘参数设置工具保存 |
| [turntable_material_seed.json](code/config/turntable_material_seed.json) | 通过三个物料端面提供外圆搜索初值，包含启用开关、参考位置和搜索阈值 | 手动调整 `enabled`、`reference`、`detection`；最终上报圆心仍来自真实外圆 |
| [ring_station_detector.json](code/config/ring_station_detector.json) | 三圆环几何尺寸、检测阈值、参考圆心、像素到毫米换算、纠偏矩阵和验收门槛 | 按实际工位测量与标定结果手动调整；叠放位模式也使用其中的工位基准 |
| [stacked_ring_detector.json](code/config/stacked_ring_detector.json) | 叠放位中环和基准线检测参数、搜索标定数据 | 直线标定工具保存 `search_calibration`；其余检测阈值可手动调整 |

例如，在 `camera_processing.json` 中关闭畸变矫正：

```json
{
  "undistort_enabled": false
}
```

主程序也可通过命令行指定配置文件，例如：

```bash
python3 code/main.py --camera-processing-config code/config/camera_processing.json --turntable-config code/config/turntable_detector.json
```

将上面的路径替换为自己的配置副本即可。其他配置参数可通过 `--help` 查看；标定工具使用 `--config` 指定保存文件，相机内参使用 `--calibration` 指定。

现有配置来自特定相机和安装姿态，更换设备、分辨率、高度或角度后需重新核对区域、参考圆心和换算参数。叠放位工具跟随畸变矫正开关，颜色、转盘参数和抓取区域工具仍使用矫正画面；若主程序关闭矫正，需要重新核对坐标配置，叠放位搜索标定也需在一致的图像设置下重新保存。

## 串口通信

模式切换、任务码同步、颜色编号和结果帧格式见 [视觉程序串口通信协议](code/串口通信协议.md)。
