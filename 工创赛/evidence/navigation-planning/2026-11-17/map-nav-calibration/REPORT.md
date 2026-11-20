# 连续地图四向50%补偿与跨段航向继承：软件交付

日期：2026-11-17。实现基线555086e2；本阶段提交号见git log。仅本地软件，不fetch/push，不访问硬件。现场保持暂停，未下载、复位或发车。

## 最终行为

- 新APP_MAP_NAV_RESIDUAL_ENABLE默认0；integrated_on隔离候选显式1，依赖连续地图但不依赖BASELINE40。合法四向distance直线按实际raw净进度应用已有固定50%主距和旁向系数，按用户决定不以原2100mm/40cm/s门禁阻止地图route应用。新距离、速度、载荷属于未验工程外推。
- 地图保留独立raw轮/HWT积分，发布pose=raw+各段累计correction；原单段ReadNavigation40和ICM锚、原控制距离倍率、左右控制trim、原航向PD完全保留。当前地图来源仍是轮/HWT加经验校准，不把它称为已完成ICM融合或绝对定位。
- 成功Move及两个Turn提交点发布唯一epoch/run/begin/方向scope；失败提交不发新tag。首个共同source sample>=begin只建段锚，该跨边界区间raw-only；后续从段锚raw净投影正部一次重算修正。换段保已发布历史，不按getter次数或绝对里程反复增加。
- 首锚使用实际共同样本heading旋转，不把90/180指令当已转到该角。显式turn、定时动作不套四向系数；完成/取消/故障后本tag不再增加经验修正，停车尾raw继续记录。真实大角转动夹在四向标签内的校准有效性未实测，本版未新增3度门限。
- 同run GAP保持原sticky语义，raw与corrected回退到同一最后发布sample；新run才清累计，旧active tag不能跨run接着补。ResetRecords/local SetPose/PLAN切换不重置全场累计修正。
- 四向直线原已统一通过HoldStep，故不新增四套PID。中央CombinedStart从同run最后成功完成的计划绝对heading继承reference；覆盖VehicleMission_Load和RuntimeRouteInstall两入口，失败预写/仅安装未执行的目标不进入continuation。独立Move在接受前准备参考、成功后才锁存，并在下一提交前捕获已完成动作，避免欠转被重新取当前yaw吞掉。
- 独立TurnStart保留原相对角API语义，并在非list提交时清除旧list绝对转向flag。用户规划中的90/180绝对目标防累积由正式action list/后续规划器负责；不声称连续独立relative Turn(90)+Turn(90)已经成为绝对目标接口。
- 新AppOdometry_MapTrackingRun只原子读取本轮有效/鲜度/GAP身份，避免运动栈上复制128B地图快照；不增加CAN交易、VM操作或新任务。

## 覆盖元数据

AppMapPose增加raw_x/y、correction_x/y、action_epoch、calibration_flags、calibration_direction。pose heading/sample_ms/age/valid/reasons仍来自真实同源raw状态，只改变发布XY。

source=1表示本轮尚未应用经验修正，2表示已有修正历史；nav_applied是本轮曾应用的标记。calibration_scope累计bit1=参考条件、bit2=外推；REFERENCE要求原2100mm/debug-RPM/trim匹配且控制距离比例关闭，不把接口允许的300/1500mm当实测工况，参考条件本身也不是新候选实车验收。flags累计bit1=边界未补、bit2=不支持动作、bit4=结束尾流未补。转向期间已有corr仍保留，不撤回或冒称全程pure raw。

## 实际验证

1. 产品前本机MATLAB R2020a：分段overlay模型1303检查，heading生命周期模型924检查，均实际exit0并有脚本SHA/起止回执。模型不覆盖真实车辆动力学、RTOS或全任务状态机。
2. 实际ARM C编译真实app_odometry/chassis_odometry/kinematics/heading_controller及生产heading header并由Unicorn执行：map_cal/map_cal_reference/map_cal_legacy各627断言、map_raw349、map_off4，失败行均0。涵盖四向符号/87与180真实朝向、net反向回退、重复get/sample、旧源边界、tail、local reset、same-ms多epoch、活动旧tag新run、stale/getter抢占GAP、REFERENCE/EXTRAP分类、raw/旧nav开关共存。
3. RTOS/tick/critical、heading/cache/power及MotionGetState为夹具桩。完整app_motion_runtime实际接受/完成hook和Mission调用路径按源码审查+整机实际链接核对；fixture记录其SHA，但不把这写成RTOS/Mission动态通过。独立检查另起Unicorn装载最终AXF重放，详见INDEPENDENT_REVIEW.md最终版本。
4. 四个有效整机配置均ICM45686_SAMPLE_ONLY=0并0errors/0warnings；仅Keil-r构建，未用-f下载。原六dirty SHA、五源原非ASCII字节及CRLF均保持，git限定diff-check通过。

| 配置 | AXF SHA256 | 静态大小 |
|---|---|---|
| integrated_on | 82010a29789f5d3d0e7f484418b1b18c70fc9233f1e9030dc14de2e516cbb3cb | Code=144588 RO-data=6892 RW-data=3612 ZI-data=124004   |
| integrated_off | 85ad6c279f3fa9dfcdb605a3acabbe08dbcc4a1e766b5d8879596e37255147c1 | Code=143740 RO-data=6860 RW-data=3612 ZI-data=123948   |
| map_off | e3840bb578cb6de39418dd4338ce2fddc30d0e11f92e7e21c417cb8e9c6265f9 | Code=141332 RO-data=6860 RW-data=3592 ZI-data=122192   |
| baseline_on | 2ff770a79ee2b0cc7e4d21db72637c1065fd7f3992c105f9430d40c81275199e | Code=115828 RO-data=6688 RW-data=3088 ZI-data=191636   |

## 资源与首错保留

integrated_on静态RW3612/ZI124004；相对同map校准关候选增加56B静态状态，相对上阶段连续map-on增加128B。最终CAN链912B+Unknown，CAN任务栈384words/1536B；MapStep frame136/chain260、NavStep56/92、MapTrackingRun16/32。编译器Unknown存在，不当作真实余栈证明。

采集配置固定bank A34020@0x20017300、B33600@0x10007300保持；两内存区最终分别余924/960B（静态链接边界，不是可随意分配预算）。新增56B状态和128B发布快照分独立ZI节让链接器容纳碎片，没有移动采集绝对地址或缩小容量。详见resources.json。

保留build_attempt1缺include与attempt2/3 RAM布局失败日志；修正包含正确motion_control声明并拆分新增ZI节。actual_c_attempt1保留测试输入100mm/20ms与瞬时90度违反既有源限的首错；夹具改为合法连续帧，产品源完整性保护未放宽。

## 下一步与边界

访谈已确认完整，INTERVIEW_DECISIONS.md的策略生效。后续分阶段实现合法路径连续跟踪、实际heading下剩余路程/减速、有界横向纠偏、路口分速与角格斜行；每一项先本机MATLAB再C/检查。横向纠偏本身产生混合速度，不能直接给它重复套四向经验系数；该校准覆盖需另验。

本阶段没有实现地图坐标到点控制、自动横向路线纠偏、连续分速、斜行或整场转向安排。四向校准/heading继承软件通过不等于实车导航精度，跨距/速/载荷、源时延、停车尾、运输包络/停车点、物理余栈及双起点整场仍未闭环。硬件测试需新明确动作授权，运动后按已授权流程直接保存RAW。

R核FIELD-01/02/06、OBS-01/04、RUN-01/02/07、PWR-01、MECH-01/02；E LOC-02/03/05/06/07、FIELD-08、MOT-02/03、CAL-02和本候选全部系数/元数据；Q FIELD-05/OBS-03/QR-04继续待技术答疑确认。来源为当前规则基线、活动五源、MATLAB/C/Keil实际回执；没有新的实测。
