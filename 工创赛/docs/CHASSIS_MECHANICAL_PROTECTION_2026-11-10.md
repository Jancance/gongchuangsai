# 第一步：机械与视觉保护清单

日期：2026-11-10。第一步的交接归并已在c2ed80f完成，本文件补齐保护范围与review方法。当前比较起点为c71eb0241b2492ce3103f965647dea0fd1eb1778；本步仅文档，不修改固件、配置值、视觉代码或冻结包。

本清单用于开发时核对差异，不进入单片机程序，不增加比赛停车条件。每个后续小步先计划，按已明确范围实施，再验证、自查review、更新进度并本地commit；不自动推送。计划未讲清或存在影响方向的疑问时先问用户。

## 1. 三种来源不能混用

| 来源 | 当前用途 | 保护要求 |
|---|---|---|
| [活动底盘工程](../../3.stm32f407工程/gongxun-f407.uvprojx) | 日常底盘入口，目标Vehicle_Integrated/Vehicle_Offline；实际主程序在CHASSIS | 后续仅实施计划列明的底盘差异，不把现有机械改成另一版本 |
| [已验pzq基线](../handoff/pzq/README.md) | 固定下→左→右的一轮三件机械抓取装车已实车通过；含原始镜像和验证证据 | pzq整个目录只读保留，不能把其大main/CAN/参数整包覆盖活动底盘 |
| [原integration13冻结包](../handoff/integration13_final_20261109/README.md) | pzq来源与历史回滚对照 | 整包保留；不因重复或“整理”删除、转码或重建覆盖原镜像 |

根工程的vehicle_arm与参数并非pzq已验版本。根工程仍含旧固定动作和回零测试配置；保存pzq不等于其完整机械链已接入根工程。以后需要接续时只通过已有底盘/任务接口适配，迁入哪部分须另列计划，本步不迁移、不重写抓取。

已验范围与尚未联调范围分开：原料区机械抓取已完成；视觉毫米对准、按任务颜色自主调用和离区路线的端到端接续，仍按[机械与底盘交接](PZQ_CHASSIS_TEAM_HANDOFF_2026-11-09.md)的证据边界处理，不借此否定已完成机械动作。

## 2. 独立机械、视觉文件

下表活动路径以仓库根3.stm32f407工程为前缀；pzq路径以工创赛/handoff/pzq/project为前缀。同名旧USER/副本也不作为顺手清理对象。

| 类别 | 文件或目录 | 后续底盘工作不得改动的内容 |
|---|---|---|
| 活动机械主逻辑 | CHASSIS/USER/vehicle_arm.c、vehicle_arm.h | 状态机、指令单位、开始/取消/完成、停止及轴所有权 |
| 活动启动与参考 | CHASSIS/USER/startup_axes.c/.h、startup_pose.c/.h、startup_extension_home.inc | 回零、工作零点、有效参考、上电准备、正常停止保持 |
| 机构驱动与参数 | HARDWARE/DM_CAN2/、GRIPPER/、CARRIER/、EXTENSION/、LIFT/；CHASSIS中同类实际编译文件 | 4310、夹爪、载盘、伸缩、升降协议、单位、缩放、目标、速度与标定 |
| pzq机械包含链 | CHASSIS/USER/vehicle_arm.c包含source_prepare.inc、extension_calibration.inc、vehicle_source_parallel.inc、integration10_complete.inc、stage2_fast_cycle.inc、source_stage2_backend.inc；integration10_complete.inc再包含integration13_immediate.inc | 包含链及其执行顺序；不能因为实现放在inc里就遗漏 |
| pzq辅助实现 | source_parallel_flow.c/.h、source_prepare_math.h、source_pick_single_action.h、source_pick_teach_table.h、lift_ruler_adapter.c/.h、lift_ruler_calibration.c/.h，以及所用startup/参考实现 | 三轴批处理、反馈放行、位置缩放、示教参数、故障与STOP收尾 |
| pzq任务/视觉桥 | source_observer、source_stage2_*、stage2_site_*、source_vision_runtime等 | 原有请求、观测版本、状态和接口含义；本轮不改冻结接口实现 |
| [视觉应用](../vision_app/README.md) | 工创赛/vision_app/及其中code、reference_config、run.py、start.sh | 算法、阈值、标定、任务解析、协议与部署入口；不为适配底盘改视觉 |

此表列举入口，不替代真实include和Keil引用核查；不在表内不等于可改。后续只编辑该小步计划中明确列出的文件。

## 3. 共享文件的边界

这些文件不能整份冻结，也不能整份放开。后续main拆分只移动底盘职责，机械行为保持；共享事务和供电时序的功能修改须另列小步，不夹在整理中。

| 活动文件/位置 | 必须保持 | 可在后续计划中局部调整 |
|---|---|---|
| [CHASSIS/USER/main.c](../../3.stm32f407工程/CHASSIS/USER/main.c)，App_CanTask原3353–3357行 | StartupAxes_Service→DM服务→VehicleArm_Service→DM服务的顺序、条件和底盘准入；Key/Safety中的取消/安全调用；任务优先级、临界区和周期语义 | 航向快照、定位适配、底盘执行、调试遥测分别移入独立c/h；main保留初始化/创建任务/短调度 |
| CHASSIS/SYSTEM/CAN/can.c，StartupHome_Capture、VehicleArm_Exchange及机构事务 | 地址5/6、回零通知、协议/长度/单位、反馈与ACK含义、原CAN唯一owner和停止出口；CAN1/2共享资源依赖 | 四轮地址1～4的必要定位/位置事务由原owner接入；不另起读取FIFO的任务 |
| CHASSIS/SYSTEM/SAFETY/safety.c及CHASSIS/HARDWARE/POWER/power.c | StartupAxes_NormalStopHold与VM请求/保持的关系、故障停止；不以底盘正常结束切断机构保持 | 底盘新模块复用现有接口；本次机械保护清单和随后main-A不改变此处逻辑 |
| [USER/vehicle_parameters.h](../../3.stm32f407工程/USER/vehicle_parameters.h) | STARTUP_*机械准备/回零，VEHICLE_EXTENSION/LIFT/DM/GRIP/CARRIER/SOURCE等机械参数；VehicleArmRow、VA_*、vehicle_arm_enable/repeat/rows及相关完成/观察条件 | 底盘路线、速度、几何等仅在对应功能步骤修改；检查宏引用，禁止仅凭前缀判断归属 |
| CHASSIS/USER/vehicle_mission.inc、app_config.h、Keil工程文件 | 机械调用/等待、装载事实、配置分支、接口定义、任务时序及受保护源文件引用 | 仅接入计划明确的底盘入口/新编译单元；不改变机械任务或伪造已完成 |

pzq的Integration10_Service在原main约3218、3252行调用，与根main不同。根main整理不得机械套用pzq行号或分支。行号只用于定位，文件变动后按函数确认边界。

正常工位流程保持为：底盘按目标朝向接近→现有视觉给毫米偏差→底盘四轮位置模式纠偏并实时累计实际XY→对准后上电保持→现有机械抓取→按真实完成/装载依据接后续路线。位置纠偏尚待实现，本步不改接口或开启旧测试入口。

## 4. 对照依据与已知换行差异

复用[pzq source-manifest](../handoff/pzq/source-manifest.json)，不生成另一份机械工程或改写清单来掩盖差异。2026-11-10只读检查2053项：无缺失，1165项原始SHA256匹配；888项全部位于project/FreeRTOS，当前字节将CRLF统一为LF后均匹配原清单，无其他无法解释的差异。

这是历史清单的换行差异，不能报告“2053项原始字节全部匹配”，也不代表本步修改了FreeRTOS。本步不转码、不改清单。原始内容与换行差异在review中分别记录；后续每步仍比较操作前后实际字节，不能无条件忽略换行。

机械关键文件vehicle_arm.c、integration10_complete.inc、integration13_immediate.inc、source_parallel_flow.c，以及在线镜像均匹配原清单。在线AXF SHA256为8da8171cb852556eaa8669ee200c18979195d078d084a0a195c78655bf587808；该身份只证明保存的已验镜像，不代表当前板端。

当前根工程用c71eb02作为本步前Git差异依据。每个后续小步另记录其操作前HEAD及相关文件原始SHA256；不把同名pzq机械文件的SHA要求套给根工程旧实现，也不因Git对换行归一化就宣称原始字节未变。

## 5. 每一步的自查review与验收

1. 先计划：写明目标、将改的文件/模块、机械共享边界、验证和完成标准；疑问先查记录，再问用户。计划模式由会话实际模式决定，不能声称自动切换成功。
2. 实施：限定改动；独立机械/视觉文件不修改，共享文件只做计划中的底盘差异。保留既有GBK/UTF-8与CRLF，不批量格式化。
3. 验证：独立文件比较操作前后原始SHA；共享文件逐段review调用、宏、优先级、周期、供电与停止语义。哈希只能证明文件身份，不能代替调度行为检查。
4. 自查review：完整阅读自己的git diff，检查遗漏的inc、共享宏或被动受影响调用；发现问题先修正并重验。适用功能先MATLAB，重构做受影响构建/既有检查；仅文档本步无需构建或仿真，不新增与要求无关的测试。
5. 交接与回滚：更新[进度表](../../底盘定位与路径规划_2026-11-09.md)、[长期记忆](../../长期记忆.md)和短交接，分别记录已编码/离线验证/下载/实车状态；确认无误后单独本地commit，不推送。

后续可用`git diff --name-status <该步开始HEAD>`查看范围，用`git diff <该步开始HEAD> -- <共享文件>`逐段审阅；检查待提交内容时再读`git diff --cached`。不能仅以编译通过或工作区干净证明机械不受影响。

第一步完成标准：当前入口可追溯，保护范围覆盖机械包含链、驱动、参数与视觉，共享文件边界明确，历史清单差异得到解释，操作前后固件/冻结/视觉文件零变化；本地提交保存清单及进度。下一步只规划main-A航向快照拆分，不自动进入第二步实现。

本步验证/review记录：6325个已跟踪的活动固件、pzq/原冻结包和视觉文件操作前后原始SHA一致；36处受影响文档链接有效，无冲突标记，diff检查通过。人工review覆盖机械include链、共享参数/宏、CAN/VM/调度边界和源码零修改要求，未发现本步待解决问题。仅文档，不进行构建、MATLAB、下载或实车操作；提交号通过本文件的git log查询，避免为写入自身提交号再增加一轮提交。

相关规则事实R：RUN-01/02、FIELD-06、PWR-01、MECH-01/02；相关工程选择E：SRC-04、LOC-02/03/05、MOT-02、RUN-06。本步不改R，也不解决或改写现有Q。
