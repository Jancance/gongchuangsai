# 智能搬运机器人控制平台

这是面向 Orange Pi 5 Pro + 双 USB 摄像头 + STM32F407 的可部署工程骨架。当前版本优先完成板端基础设施、可靠串口、经典视觉避障、安全状态机、健康检查和验收工具；机械尺寸、电机参数、最终镜头与标定数据必须在实物确定后填入配置。

## 规则基线

所有后续方案、代码、采购、测试、排期和赛前验收必须先检查 [2027智能搬运竞赛规则与设计约束基线](docs/competition_requirements_2027.md)。该文档分别标注规则事实（R）、工程建议（E）和待技术答疑（Q）；不得把建议或未确认内容表述为正式规则。

## 快速开始

P1电控转交他人及其Codex：先读[2026-10-15独立交接文档](P1电控组交接_2026-10-15.md)。真实F407源码冻结快照已随项目放在`handoff/p1_2026-10-15/firmware/`，包含Keil入口；当前进度与唯一下一项、原路径/接手方工作副本、证据及未闭环项均在交接文档中说明。

2026-10-03集成视觉工作台已保留第三方转盘、物料、圆环、叠放工位、像素测量和四个调参工具，并新增本项目像素跟随入口。见 [vision_app操作说明](vision_app/README.md)。在项目根目录运行 `python vision_app/run.py --check` 可无硬件检查；香橙派运行 `python3 vision_app/run.py`。桌面默认串口关闭、毫米几何未验收。

P2视觉换模型接续：[当前进度与八周计划](docs/P2_MODEL_HANDOFF_2026-09-13.md)，[原视觉对话存档](docs/P2_CONVERSATION_ARCHIVE_2026-09-13.md)。

在 PC 上运行模拟与测试：

```powershell
$env:PYTHONPATH="src"
python -m unittest discover -s tests -v
robot-main --config config --simulate --cycles 20
```

在 Orange Pi 上按 [Orange Pi 配置指南](docs/ORANGE_PI_SETUP.md)操作。不要直接运行未知来源的一键脚本，也不要在没有断开电机使能的情况下做首次联调。

## 安全不变量

- 启动、相机失效、标定失效、STM32 心跳超时或路径不可行时输出 `SAFE_STOP`。
- 像素坐标不能直接发送给 STM32；必须经带版本的标定转换为毫米坐标。
- `ACCEPTED` 与 `DONE` 分离；相同序号重发不能重复物理动作。
- STM32 是最终运动安全责任方，必须独立实现 200ms 减速、500ms 停车和硬件急停。
- 局部车道净宽不足时请求全局换路，绝不强行侧绕。

## 目录

- `src/smart_carrier/`：运行代码
- `config/`：硬件、相机和视觉参数
- `systemd/`：板端服务文件
- `scripts/orangepi/`：安装、基线采集和验收脚本
- `docs/`：上板步骤、接口、周计划与验收说明
- 导航专项：[灰色车道、路口、随机避障与回启停区逻辑](docs/NAVIGATION_AND_RETURN_LOGIC.md)
- 定位专项：[PAW3395二维光学里程计＋HWT101航向方案与替代门禁](docs/LOCALIZATION_PAW3395_HWT101_PLAN.md)
- 电控与PCB专项：[STM32F407VET6 P1引脚资源、限位与扩展规划](docs/P1_STM32F407VET6_PIN_PLAN.md)
- 电源与安全专项：[单电池电源板、分轨供电、PE2/PE6与急停设计](docs/P1_POWER_BOARD_DESIGN.md)
- 视觉专项：[数字、码、物料、圆环、码垛、跟踪、光照与精度基线](docs/VISION_SYSTEM_REQUIREMENTS.md)
- P2实施与交接：[2027视觉专项实施基线](docs/P2_VISION_IMPLEMENTATION.md)
- P1当前执行入口：[P1执行记录与P1-0门禁](docs/P1_EXECUTION_LOG.md)
- P1更换模型接续：[2026-09-12电控交接、证据与未来计划](docs/P1_MODEL_HANDOFF_2026-09-12.md)
- `tests/`：协议、状态机、视觉和规划回归测试
