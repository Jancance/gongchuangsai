# 本项目集成视觉工作台（DECXIN 1280×960）

基于用户提供的创源启明工训视觉项目本地适配，原功能与作者README均保留。2026-10-03版；当前仍是桌面视觉工作台，未完成装车、机械臂或真实场地验收。

## 快速使用

关闭此前相机程序，在香橙派图形桌面/RealVNC终端运行：

```bash
python3 vision_app/run.py --check
python3 vision_app/run.py
```

`--check`只加载全部配置/模块，不打开相机、串口或图形窗口。主程序默认F跟随预览；点击相机窗口再按键。

| 按键 | 保留/新增功能 | 当前使用边界 |
|---|---|---|
| F | 新增：持续颜色目标ID、像素速度与短时像素预测点 | 原始像素诊断，不判断物理运动速度，不发抓取命令；目标丢失、身份或时间不可靠时取消预测。 |
| A | 保留：转盘真实外圆粗定位、圆弧精修、稳定性、局部跟踪和失锁重找 | 像素功能；参数需按自己的镜头/视野调整。三物料辅助初值功能代码保留，默认禁用旧参考位置。 |
| D | 保留：鼠标两点测量X/Y差和直线像素距离；第三次点击恢复实时 | 仅像素，不等于毫米测量。 |
| M | 保留：物料颜色/左、右、下区域匹配，确认运动后等待低速的位置上报参考流程 | 原模式用于对照；本机四个区域未标定时明确UNCALIBRATED。实际跟随抓取目标采用F及后续P1接口，不把等停作为项目目标。 |
| R | 保留：三圆环中心、排列方向、局部ROI追踪、稳定检查 | 自己的几何/纠偏未验证时只显示像素中心，毫米结果不得当真实纠偏。 |
| S | 保留：叠放工位中环、上下基准线和搜索标定 | 是原有工位识别，不等于已完成物料层数/倒塌验证；本机搜索区域需重标。 |
| I | 待机，停止当前识别和请求 | 无执行器动作。 |
| Q/Esc | 退出，释放相机进程 | 相机断线则显示等待并清除旧检测历史。 |

## 四个原有调参工具

先退出主程序，逐项运行，避免抢占相机：

```bash
python3 vision_app/run.py --feature color
python3 vision_app/run.py --feature turntable-settings
python3 vision_app/run.py --feature pickup-regions
python3 vision_app/run.py --feature stack-lines --diameter-mm <实测外圈直径> --line-offset-mm <实测线偏置>
```

最后一行的中文尖括号必须换成自己的实测数值；不是可直接复制运行的默认尺寸。

| 工具 | 原有操作保留 |
|---|---|
| 颜色标定 | 1～6选择六色、鼠标框选、HSV滑条、U撤销、C清空、R预设、S保存。F模式与M模式共同读取该颜色配置；预设只作起点。 |
| 转盘参数 | 多参数页、滑条、A自动诊断、Tab翻页、空格冻结、P采样点、M边缘图、R重载、D默认、S保存。 |
| 抓取区域 | 依次框选活动区/左/右/下、U撤销、R清空、S保存，拒绝区域重叠或越界。只标图像ROI，不是机械臂坐标标定。 |
| 叠放线标定 | 中环自动/点选、上下线检查、STABLE后S保存搜索标定，R重找；必须输入自己工位物理尺寸。 |

前三个工具默认原图；若要开启去畸变，加`--undistort --calibration <本机有效标定文件>`。第四个及主程序由`code/config/camera_processing.json`控制去畸变，默认false。原Realtek文件会被相机验证拒绝，不能仅更改分辨率后冒充本机标定。

## 我们的相机与配置

- 默认DECXIN `/dev/v4l/by-id/usb-DECXIN_DECXIN_CAMERA_01.00.00-video-index0`，1280×960 MJPG30；实际曝光50、增益30、手动白平衡4600、缓冲请求4均为台架候选。
- 原作者1280×720的相机内参/ROI/搜索标定/毫米矩阵没有直接作为本机参数。原配置完整保存于`reference_config/`，当前配置在`code/config/`。
- 主程序持续采集、只取最新软件序号，并保留主机read完成时间；F模式150ms主机年龄拒绝，500ms无新读帧则重置/等待。这些不是传感器曝光时间或总延迟证明。
- CPU图像处理，避免台架上曾出现的OpenCL编译异常；无需重装相机驱动。
- `project_profile.json`中的几何/高度版本默认未验证；80～100mm为用户提出的原料区转盘高度范围，需实测，与车载270°盘和TCP高度分开。

## 通信与正式抓取

原`turntable_serial.py`的任务码解析、模式命令、帧编解码和重连实现保留，供兼容性测试。当前主程序使用DisabledSerialLink，实际不打开串口；`--legacy-serial`会明确拒绝尚未集成的旧协议。原协议未按本项目版本/序号/CRC/ACCEPTED/DONE闭环，不能直接接本车执行机构。QR-04业务解释仍待技术答疑，不解除执行锁。

F的预测点为颜色轮廓的像素外推，短失检的内部ID可能暂存，但重捕获连续性未证实时不产生预测。原料移动中抓取还需要本机毫米标定、相机自身运动补偿、采集时间与端到端误差、P1接触时间/工作空间/安全门；核心项目`smart_carrier.moving_target`保留这些前置拒绝。本工作台不发裸PWM、不发抓取动作、不把一次检测当抓取成功。

## 离线验证

```bash
python3 vision_app/run.py --image <1280x960原图> --mode follow --output-dir diagnostics/follow
python3 vision_app/run.py --image <1280x960原图> --mode turntable --output-dir diagnostics/turntable
python3 vision_app/run.py --image <1280x960原图> --mode ring --output-dir diagnostics/ring
python3 vision_app/run.py --image <1280x960原图> --mode stack --output-dir diagnostics/stack
python3 vision_app/run.py --image <1280x960原图> --mode material --output-dir diagnostics/material
python3 vision_app/run.py --image <1280x960原图> --mode measurement --measure-points 20 20 120 70 --output-dir diagnostics/measurement
```

这些模式不打开任何相机/串口/窗口，保存overlay.png和result.json；未标定圆环结果明确metric_valid=false。软件验证不能替代硬件验收。

原说明见`UPSTREAM_README.md`；导入来源与初始文件哈希见`upstream_manifest.json`；引用说明见`UPSTREAM_NOTICE.md`。后续需装机、机械臂或场地时按主项目P2_INSTALLATION_AND_FIELD_GATES.md提前提醒。
