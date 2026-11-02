# P1电脑端分析与统一软件验收（2026-10-27）

同日后续增补：`--simulate`现会导出20cm/s模拟的10ms过程CSV/JSON与交互图，见[P1_SPEED20_SIMULATION_TRACES_2026-10-27.md](P1_SPEED20_SIMULATION_TRACES_2026-10-27.md)。模拟图与原低速实测图分别归档，不混合来源；最新验收为39项工具测试、105项主机测试、1221项参数/包检查和10场景50897次C断言通过，固件参数及冻结包未改。

本轮按用户同意的六项范围完成工具，继续遵守“当前不能实车测试、先别烧录”。只新增电脑脚本、测试适配器、示例和文档；未改运动固件、主机发车脚本、遥测格式、驱动设置或保护阈值。不安装新依赖、不打开串口、不调用Keil下载/复位、不发运动命令、不自动调参。

## 六项交付

| 项目 | 入口/结果 | 用途与边界 |
|---|---|---|
| 1. 日志分析与画图 | `scripts/p1_motion_offline.py analyze` | 读取已有JSONL/同名summary，导出HTML交互图、时间轴CSV、批次CSV、分析JSON；共用设备时间轴。 |
| 2. 版本/尺量对照 | 同脚本`compare`，示例`P1_OFFLINE_COMPARISON_EXAMPLE_2026-10-27.json` | 明确分开原主机状态、编码进度、地面尺量、横偏和主观平顺观察，不自动签优劣。 |
| 3. 参数一致性 | 同脚本`check` | 核对源码宏、主机预设/接收门限字面量、schema、冻结包文件哈希及当前USER源与候选差异。候选只允许两个启用开关与源默认不同。不是完整C预处理器。 |
| 4. 四轮独立响应模拟 | `scripts/p1_four_wheel_sim.py`、`scripts/p1_offline_tests/` | 复制当前真实后端/运动学/规划器/预设源码到输出目录，ARMCC编译、Unicorn运行；CAN/RTOS/VM用桩替换。 |
| 5. 统一软件验收 | `scripts/p1_validate_offline.py` | 固定允许的测试及分析步骤，汇总报告；模拟显式选择，不扫描运行任意脚本、不执行七配置全量构建。 |
| 6. 使用与交接 | 本说明、根交接、当前计划 | 记录当前软件证据、运行方法、结果解释和待实测边界。 |

## 直接使用

从仓库根目录运行。`--out`必须是**尚不存在的新目录**；工具拒绝往实际固件工程和`releases`内输出，避免修改冻结输入。已有输出不覆盖，重新运行请换目录名。

日常快速检查（Python标准库即可，不运行ARM模拟）：

```powershell
python -X utf8 -B 工创赛/scripts/p1_validate_offline.py --out 工创赛/tmp/offline_quick_01 --package 工创赛/releases/p1_speed20_candidate_2026-10-27
```

没有提供历史清单或`--simulate`时，对应步骤明确记为`skipped`，总状态为`passed_with_skips`，不冒充全部通过。参数不一致、文件改动、依赖缺失或测试失败，写入报告并返回非0；不自动安装、修复参数或重试车辆动作。

本机完整验收命令（已存在ARMCC5及Unicorn；依赖路径为当前电脑已有安装，不是新增下载）：

```powershell
python -X utf8 -B 工创赛/scripts/p1_validate_offline.py --out 工创赛/tmp/offline_full_01 --package 工创赛/releases/p1_speed20_candidate_2026-10-27 --manifest 工创赛/docs/P1_OFFLINE_COMPARISON_EXAMPLE_2026-10-27.json --simulate --unicorn-path C:/Users/lenovo/.codex/visualizations/2026/10/24/01a16e59-67fe-7b80-a24e-fb6eefeae807/arm_test_deps
```

ARMCC默认目录`D:/Keil_v5/ARM/arm5_compiler/bin`，其他电脑用`--arm-bin`指定；若Unicorn已在当前Python中安装，无须`--unicorn-path`。只生成测试AXF并在模拟器执行，未启动Keil GUI、下载器或调试会话。本次不改固件，因此不重新编译七个工程配置；此前三个完整Rebuild的证据仍单独保留在speed20记录。

单独分析已有记录：

```powershell
python -X utf8 -B 工创赛/scripts/p1_motion_offline.py analyze <已有记录.jsonl> --out 工创赛/tmp/log_analysis_01
python -X utf8 -B 工创赛/scripts/p1_motion_offline.py compare 工创赛/docs/P1_OFFLINE_COMPARISON_EXAMPLE_2026-10-27.json --out 工创赛/tmp/comparison_01
python -X utf8 -B 工创赛/scripts/p1_motion_offline.py check --package 工创赛/releases/p1_speed20_candidate_2026-10-27 --out 工创赛/tmp/config_check_01
```

第一条的占位路径需替换为实际文件。分析输入是现有采集脚本生成的已解码JSONL；原始CRC校验发生在采集阶段，本工具不重新认证原始串口字节。缺少summary时主机状态保留为unknown。

## 看哪些结果

统一输出`validation.json`给出各步骤状态，`README.md`用于快速查看，`config.json`给出逐项匹配/差异；`offline_tests.log`与`host_tests.log`为真实测试输出。

`run_01`～`run_05/report.html`为五次历史测试的交互图，可以直接用浏览器打开，无CDN或网络依赖。可选择共同时间范围、开关曲线、鼠标读取邻近样本。`timeline.csv`保留共同时间轴和原始上报基速，`batches.csv`列出最近保留批次的发送间隔、排队和单次接口耗时。`analysis.json`包含分块、缺口、丢弃记录及解释边界。

`comparison/comparison.html`、CSV/JSON列出人工尺量。`ground.lateral_cm_right_positive`使用右正左负；未量写null，不能填0。`ground.stopped`与`vm_lamp_off`同样允许null；遥测四轮0/VM0不自动变成现场确认。示例版本标签为人工归档，不根据串口日志猜测AXF版本，实际版本需对应既有下载与冻结记录。

图中`base_rpm`只画后端活动且START/RUN的上报值；待机配置值仍在CSV的`reported_base_rpm`中，避免误画为起步跳速。四轮反馈正负号包含安装方向，缓存采样不同步；基速或前端纠偏不代表每轮实际发送值。参考位置未上报，不凭空画成“实测参考轨迹”。

时钟回绕用uint32差值处理。设备时钟倒退、设备/电脑时间显著不连续及新的START会分块；默认选择最后含活动的块，可用`--block-index`显式选0起始块。大于300ms的TM缺口断线显示，不插值为正常响应。分块原因全部保留，不能仅凭这个启发式判断硬件重启或实际丢包原因。多次START需分别选块；同名summary的`capture_status`属于整个采集文件，不重命名为所选块的成功状态。

BT1按会话/批号/发送时刻去重，用原始发送时刻映射到同一设备时间轴。只接受选中块时间范围内的历史；无关联的旧批次丢弃。保留的16条不能还原整场全部指令，`cumulative_`字段与当前保留样本统计分开解释。接口应答耗时、整批时间均为应用毫秒时间，不是机械到速时间，也不证明CAN硬同步。

## 模拟覆盖与不能证明的内容

本次10个场景：相同响应、四轮分别0/20/40/60ms延迟、第三轮速度增益0.8、第三轮速度应答缺失、第一轮位置应答缺失、第二轮停止应答缺失、第二轮超速、第四轮反向、航向命令过期、停车期间CAN任务停顿。

四轮分别保存目标历史、速度和编码位置；模拟轮速为假定的一阶响应，四轮按实际后端串行发送推进。调度步长10ms，桩交易瞬时返回，不拟合真实电机响应、CAN排队或RTOS抢占。RUN前置状态由适配器设置，未模拟VM上电准备与驱动使能确认。主循环HWT控制器不在本适配器内，航向邮箱正常时刷新，指定场景停止刷新以检查原100ms门禁。

正常两场景必须到距、四轮停止应答齐全且后端DONE；异常场景必须按原保护报错并提出撤VM请求，不能报成功。`cut_vm_request`只表示真实C后端的请求值：桩不会操纵物理电源，不等于实物VM已掉电。模拟没有地面X/Y、侧滑或真实航向模型，四轮编码差不能当作真实横偏预测。

新增脚本不导入硬件发车客户端。参数检查通过AST读取主机常量/比较值，不执行其模块。统一入口执行原主机单元测试时由其已有测试桩覆盖命令生成与门禁，不打开WindowsSerial。运行前后对实际工程、主机脚本/schema及指定候选包文件做SHA256核对，任何变化都在汇总中失败；并发人工修改也会触发该检查。

## 本轮验证与后续

最终证据目录见`evidence/p1-foundation/2026-10-27/offline_tools_validation_final/`，其中有完整报告、5套图表和四轮模拟编译记录。工具测试还覆盖时钟回绕/重启/旧缓冲、重复BT、单位与缺值、失败状态保留、人工测量分离、参数/包篡改检测、输出保护、脚本不执行主机模块，以及汇总失败/跳过语义；图表JavaScript语法经Node检查，浏览器本地截图检查正常显示。

历史A2继续保留telemetry_timeout，B2继续保留motion_fault_or_cancelled、distance reason9；不能因为工具生成图表成功就改写成车辆通过。所有历史图都是低速结果，**不存在本轮20cm/s地面实测**。

当前源两个曲线/提速开关仍默认0，20cm/s专用冻结候选两个开关1；候选AXF SHA256仍为`7f2228dee9e38f9b943e4d4704e6cee44bf74e2f9d80abf3046c138032e63543`。下一步待用户恢复实测条件，再单独确认现场并验证20cm/s实际速度、到距、平顺性、停车距离及运行负载；不沿用旧就位答复、不自动发车。

核对R：RUN-01/PWR-01；E：LOC-03/05、MOT-02/04、RUN-06；本轮仅离线工程工具，不改正式比赛规则或Q待答疑项。全部仅本地完成，未提交或推送GitHub。
