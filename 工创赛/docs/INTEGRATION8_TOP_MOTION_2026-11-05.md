# 整合8当前：50mm抓取候选、顶部轨迹与检查接口

> 最新221545去纸图已将三处盘面圈补齐，MATLAB8项计算检查通过；当前最外缘车盘净距46.5mm（55已改，旧70基准非最外缘，相关净距验证撤回），低于50mm工程要求。图像条件模型约31.7mm与实量差14.8mm，需当前机械两项复量，不放行旧/新诊断纠偏；见[当前221545接续](INTEGRATION8_UNCOVERED_221545_2026-11-05.md)。

> 最新220715原图已收到并归档，本机MATLAB9检查0失败、纸内M/E/F RMS0.719mm/最大0.929mm。右上盘面圈被方纸遮挡，仅130°观测分箱，不接收为完整当前目标；左/近侧可见弧足够。下一项保持所有机构/车体/盘不动，仅拿开方纸再拍原图，以补全右圈；旧纠偏量保持停用。当前新图profile/证据和未闭环见[220715接续](INTEGRATION8_CURRENT_IMAGE_220715_2026-11-05.md)。

> 最新现场状态：用户澄清此前主控未供电、现在才供电，并确认底盘可能有位移、当前顶部空爪。随后只读attach成功，Flash148728字节与旧AXF逐字节匹配，run0/stage1/state0、升降参考valid0，TIM2_CCR1=1574对应上电名义145°。旧轮次/持位/回零参考和原图姿态不得继续沿用；左13.4/后14.9mm及原目标角/伸缩表只作历史仿真，当前不执行。**现场第一项是新姿态1920×1080原图采集，重核车盘与TCP/相机关系，再继续独立升降标定。** 50mm选择及机械尺量保留。下文记录本轮已完成候选的适用边界。

当前仍为270×250mm车体、尺量70+100mm伸缩行程、三个目标共用一个停车点。左13.4/后14.9mm的停车候选、实测输入及误差条件沿用[单停车点验证](INTEGRATION8_BODY270_SINGLE_STOP_MATLAB_2026-11-05.md)。本轮继续做本机MATLAB前置验证，再写对应独立检查候选；没有加入实际Keil或修改其限位/速度/增益，没有烧录、复位、暂停或发出车辆/机构动作。

## 用户当前高度决定

用户明确119mm是**升降顶部空爪时，夹爪最下面底面到原料圆盘的距离**，不是指尖接触点、夹爪安装面或光心高度。料高60mm。用户提出在料块50mm高处抓取；对比后曾选择45mm，随后明确“还是按照50mm”，因此当前严格保持**50mm抓取底面候选**，45mm仅保留比较，不部署。

理想刚性夹持、无滑落条件下：夹爪最低底面从119降到50mm，下降69mm；物料抓起回顶后，料底距盘面119−50=69mm，较其他60mm高物料顶部高9mm。这个模型不能代替实际接触、夹牢、滑落/倾斜或其他凸起物检查。

| 抓取时夹爪最低底面距盘面/mm | 顶部下降/mm | 带料回顶后越过其他料的名义净空/mm | 各高度±1mm声明扰动最差净空/mm |
|---|---:|---:|---:|
| **50，用户当前选择** | **69** | **9** | **6** |
| 49，仅对比 | 70 | 10 | 7 |
| 48，仅对比 | 71 | 11 | 8 |
| 45，用户已撤回选择 | 74 | 14 | 11 |

本轮10mm竖向净空是工程候选要求E，不是官方R；50mm方案不满足它。没有把用户坚持50mm解释成允许把结果改标为通过或删除保护。后续可检查真实受控路径是否需要跨越未取物料、误差/滑落范围是否可靠、是否有其他避让办法；尚未验证前不放行带料扫掠。

[高度MATLAB模型](../scripts/validate_source_pick_clearance_matlab_20261105.m)在本机R2020a运行，7项几何检查0失败；正确接受/拒绝比较项，不意味着用户当前50mm高度的10mm净空通过。`user_selected_grip_height_mm=50`、`comparison_selected_by_user=false`、`user_selected_height_meets10mm_gap=false`。

## 原到位判据与新检查候选

只读核对实际工程：

- `HARDWARE/DM_CAN2/dm_can2_motion.c`当前允许目标误差±3°、低角速度连续15样本后宣布保持。`CHASSIS/USER/vehicle_arm.c`也用±3°作为上层到位条件，目标范围仍0～180°。
- 对本轮远侧半径约329.4mm，±3°可对应约17.25mm端点弦长差。它不能作为本轮2mm级几何条件的已验末端精度。
- 本轮MATLAB候选将抓取到位检查设为±0.2°、伸缩±0.1mm、角/径向速度各≤0.5°/s、0.5mm/s，参考曲线结束后再要求连续3个50ms间隔的新鲜样本。关节自身的几何误差上界为约1.15+0.1=1.25mm；这不包括视觉、停车、安装、回差和负载误差，也不是实车已达到该精度。
- 实际通用伸缩目标仍限制0～50mm，源准备分段仅到70mm；升降/载盘参考默认仍未标定有效。不能只写新目标数字就接入原动作组。

源文件SHA和核对范围保存在`top-motion-review/actual-source-audit.json`。这是实际源码只读结果，不是本轮板端镜像匹配/运行状态证明。

## 同轮MATLAB顶部轨迹与故障候选

[顶部轨迹模型](../scripts/validate_source_top_sweep_matlab_20261105.m)实际使用`D:/MATLAB1/bin/matlab.exe` R2020a运行，batch退出码0。范围是定高顶部的旋转/伸缩参考、采样/到位条件和STOP应答模型，不是完整抓放三轴动力学或真实RTOS仿真。

模拟六种任务顺序、每件去源标记再回车载参考、三个声明的一阶驱动时间常数0.04/0.10/0.20s，共108次正常顶部轨迹。DM参考使用当前源准备150°/s、150°/s²上限与五次曲线；伸缩用当前源准备30RPM对应粗标50mm/s、60RPM/s对应100mm/s²，35mm分段以保持单条电机角≤180°。这是候选参考限速，不修改普通动作106RPM等当前参数。车载参考35.1079407°/伸缩0只是已有固定循环的迁移候选，当前新视野/包络/车载槽几何没有因此通过。

11类故障注入：取消、DM过期、伸缩过期、DM失能、伸缩参考丢失、姿态版本变化、服务间隔超时、驱动目标读回不符、夹爪命令失败、STOP不应答、DM持续1.6°残差。候选不能把它们当正常完成；STOP未确认时保留pending，模型最终明确记录未确认。1.6°残差比较项在旧3°门限下可以到位，在新0.2°候选下超时拒绝。

最终469项检查0失败。正常模型最终关节自身误差最大约0.154mm只属于该声明的一阶模型，不是实物精度。三件顶部往返参考最慢约39.10s，未包括下降/闭合/载台实际放置/实际视觉/位置微调及故障；不作为完整周期或3分钟比赛验收证据。

模型没有包含实际电机辨识、夹爪/线缆水平扫掠、真实抓牢/载台提交、车道、三轴Z运动、共享CAN调度、并行供电或CPU/任务级停止路径。不能拼接旧机构33/整车94场景通过记录宣称本轮完整动作通过。

## MATLAB之后实现的独立候选

本轮469项前置检查及既有几何计算通过后，新增[几何/反馈候选接口](../software_interfaces/source_pick_geometry/README.md)，实现仅在工作区独立目录`software_interfaces/source_pick_geometry/`，没有导入实际Keil。

- 目标换算：车体右/前毫米目标、停车平移及航向→径向伸缩/逻辑角。
- 明确区分物理范围、调用方要求的端点余量与当前固件支持能力；原10mm余量不满足仍拒绝，当前固件0～180°/通用50mm仍不支持三个候选。
- 单反馈样本检查：参考未结束、过期/未来样本、角/径向误差和速度不满足均拒绝。没有动作、事务稳定样本累计、抓取/载台DONE或电机接口，`motion_allowed`始终0。

[跨语言检查脚本](../scripts/check_source_pick_geometry_20261105.py)使用本轮MATLAB保存的三处目标生成独立C向量，69项检查0失败；GCC警告视为错误通过，ARMCC Cortex-M4单元编译0诊断。不是完整固件构建、运行调度或实物验收。

## 当前唯一接续项与边界

50mm高度已定，不再反复询问。由于本轮后续发生重新供电和底盘可能移位，先采集当前新原图并建立有效姿态绑定。之后建立**升降毫米与反馈/虚拟脉冲的有版本对应关系**，再接受50mm候选的有界Z目标，并将几何、带料避让、当前底盘/相机姿态与完整失败出口在本机MATLAB同轮验证。119/50给出了69mm候选下降距离，尚不能给出已验脉冲数。

只读历史记录`P2_ARM_RESUME_2026-10-28.md`有用户5000脉冲约3mm的粗测，旧固定抓放105000下降/104000预升有其限定示教范围；都不是当前精确69mm标定。按5000≈3mm直接算69mm会得到约115000脉冲，甚至超过当前105000下降上限，因此不能照抄、削除上限或把旧105000强行当69mm。不能因用户要求推进而重新复制这一未经标定的换算。

### 本轮只读调试通道诊断

为准备有界标定，尝试沿用无复位/无暂停attach并核对当前Flash。v2接口在探针信息查询时超时，pyOCD临时打开探针失败后未完成关闭，后续第二次open触发`assert self.closed is True`；完整栈表明该空AssertionError发生在USB探针打开阶段，尚未进入主控内存读取，不能当作STM32故障或Flash不匹配。

依据本机pyOCD代码支持的`cmsis_dap.prefer_v1`选项，另在独立进程尝试v1 HID只读连接，同样先发生探针查询超时，后续报告already open。两种初次USB传输均未读到当前主控状态；该历史状态已由下方成功读回接续。没有复位/暂停/恢复CPU、清错、固件下载或执行器写入，也没有终止其他调试进程。

初次失败证据为`top-motion-review/readonly-connect-report.json`、`readonly-connect-debug.log`、`readonly-v1-report.json`和`readonly-v1-debug.log`。用户随后澄清主控先前未供电、现在供电，不沿用此前“已通电”的判断。

使用探针USB描述符及有界原始DAP_Info查询，确认Horco FAED:4873/6afaae24探针可回复packet_count=32、packet_size=64、capabilities=0x0113等查询；没有SWD目标命令或USB设备复位。随后用[实例内同步收发只读脚本](../scripts/source_stage2_readonly_sync_probe_20261105.py)限制单包顺序传输，无复位attach成功，Flash148728字节与旧Vehicle_Integrated.axf逐字节匹配（AXF SHA256=e35eada30047a5f8ef8baf38f5948d92d231ab6f44f5dd9be9eaff2d65087e24）。改变USB收发方式与用户给主控供电发生在同一排查过程中，**不能单独归因于适配器修复**。脚本只替换当前进程实例收发方法，没有修改安装包或实际固件。

成功快照：RUNNING，run0/stage1/state0/cause0、源准备state0/mask0、startup0、g_vm_test_request0、底盘auto0、rtos/safety fault0、safety_ready1；升降reference_valid0/feedback_valid0；夹爪命令145°且CCR1=1574us。PWM不是实际角度反馈。无执行器写入、无固件下载/复位/暂停。证据为`top-motion-review/readonly-sync-report.json`和debug日志，原始查询另有`usb-descriptor-report.json`、`raw-dap-info-report.json`与`raw-dap-extended-info-report.json`。

用户确认当前空爪、升降顶部，但底盘可能有位移。因此旧图和旧运行参考失效，不能把g_source_prepare状态或旧70mm回零原点重新写成“已完成”。当前参数增加`live_session`明确关闭旧图姿态连续性与旧纠偏执行权限；机械尺寸/尺量值仍保留。下一步需要当前1920×1080原图，而不是重新使用194157等旧图或只凭旧候选发动作。

应在当前新姿态、运行入口和驱动单位核对后，准备有界升降标定：新鲜读取同一参考的反馈，独立尺量小步前后高度，保存脉冲/原生位置/机械位移。该项尚未部署或执行；本轮已经确认板端镜像与启动待机状态，但没有建立升降有效坐标参考，不能直接指导写Watch触发旧固定循环。必要仿真与版本核对先完成，保留用户此前不烧录/复位的决定，不擅自重新跑1→2段。

仍未闭环：实际升降标定、当前50mm带料扫掠净空条件、实际夹牢、工具/载台/线缆几何、定位/停车/驱动误差、新鲜视觉/位姿变换、全三轴/共享CAN/并行供电/故障退出及自动抓放。`full_motion_validation_passed=false`、`motion_allowed=false`。

## 证据与R/E/Q

证据位于`evidence/integration/2026-11-05/source-markers/paper100-transfer/continuation/body270x250/`：

- [顶部模型结果](../evidence/integration/2026-11-05/source-markers/paper100-transfer/continuation/body270x250/matlab-top-sweep/result.json)、`simulation.mat`、`top-sweep.png`、`matlab-top-sweep-run.log`。
- [高度比较结果](../evidence/integration/2026-11-05/source-markers/paper100-transfer/continuation/body270x250/matlab-pick-clearance/result.json)、`simulation.mat`。
- [C检查与编译证据](../evidence/integration/2026-11-05/source-markers/paper100-transfer/continuation/body270x250/geometry-candidate/result.json)、`checks.log`、`host-compile.log`、`armcc-compile.log`。

R已核SRC-01/02、QR-01、DSP-01、RUN-01/02/04、FIELD-06、PWR-01、MECH-01/02；E已核SRC-03/04、CAL-02/03/05、VIS-12/15、RUN-06、MECH-06、LOC-05/07、MOT-02/04。50mm为用户当前工程选择，10mm净空与0.2°等门限是候选E，未当成规则或当前性能。R不变，FIELD-05、OBS-03、QR-04等官方Q继续**待技术答疑确认**。
