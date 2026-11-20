# 底盘运动与雷达八阶段软件整链：实施草案

日期：2026-11-18。只读规划基线：HEAD `5bbc3ee8`，分支 `codex/chassis-navigation-plan`。本草案由既定 6.1-sol max 规划角色编写；后续 medium 实现、high 独立审查。**采访尚未结束，本文不是产品编码放行，也不替用户冻结下列待定策略。**

本轮只新增本文；不改产品、旧 dirty、根交接或规则基线，不运行 MATLAB/构建，不作 Git 写入，不访问/枚举/轮询硬件，不下载、reset 或发车。后续分阶段本地 commit 是用户已授权的实施安排，不在本规划回合执行。

## 1. 已确定目标、权威与成功标准

实施决定以本目录 `INTERVIEW_DECISIONS.md` 为准；旧文档中与本轮明确确认冲突的“未知格边界/粗加工暂存点未定/仅reference”属于历史状态。官方规则仍以 `工创赛/docs/competition_requirements_2027.md` 为权威。

已定：

- 软件完整接通“运动能力 → 同owner雷达事务/路径 → 路径转换 → 八阶段导航”；先模拟雷达验收，硬件暂停。
- 路线：本轮仅启停区2 → 扫码点 → 原料 → 加工 → 暂存 → 原料 → 加工 → 暂存 → 启停区2。区1后续再做，不列本轮产品扩展或验收；start_id身份仍保留且本轮只接受2。不执行真实扫码、抓放、StartupAxes 或 VehicleArm，不伪造任务码/抓放计数。
- 雷达地图 2400×2400mm，5×5均分480mm，蛇形编号；G与F同方向、无旋转/镜像，0区2角、4区1角。用户确认黄色区已在雷达设为永久禁行，主控仍验精确黄区与主体包络。
- 导航测试点：加工格10中心(240,1200)、暂存格22中心(1200,2160)；不称抓放精停点。其余沿当前确认的启停区、扫码、原料目标；所有目标以车体几何中心为参考，不把雷达中心外参重复加给场地格中心。
- 行驶全外形283×254mm，包含收拢雷达、软线及全部突出物。
- 纯四向目标400mm/s；斜行车体前/后与左/右分量各400mm/s，理论合速约565.685mm/s。驱动应用上限保留300RPM，不以抬限兑现速度。
- 原地转向复用此前已确认的峰值90°/s目标与180°/s²、1440°/s³候选限值；旧普通5RPM、30°/s分支保持，不将90°/s误作用到旧profile。
- heading：启停/扫码/原料+Y，加工−Y，暂存−X。90°变化在起点；180°在起、终点各90°，CW优先、空间不允许才同向CCW，两次绝对目标不随第一次实际欠转重设。
- 运行中用连续地图位姿跟踪合法路径剩余弧长/横偏；四角固定车头、非零速度平滑斜向过渡。中央路口提前降速，不能用低速替代几何可行性。
- 四向50%主/旁残差候选保留；小幅混合纠偏不叠两套；真正斜行/转向不直接套纯四向实测系数。站点真实停稳后自动续行，无新增人工确认或无依据固定等待。
- 每趟开局只扫描一次，整趟沿本场图规划与复用重复有向路段；异常另行处理，不到每站重扫。
- 补齐区2四相五线机构约定：733µs、180°收拢/45°展开、下4200/上4200、每个步进动作前Enable、正常动作完成立即Release、闲置失能。4200是命令数，733µs/名义20RPM不冒充实转速、高度或机械回零。
- 启动选A：一键后自动执行180°→45°、下4200、一次扫描、上4200、收180°，正常每腿及时Release；收拢后自动开始区2八阶段。沿已保存校准，不每趟校准或另发保存校准Flash命令；GRID_SCAN协议自身保存本场方格图与“重做校准”分开。

最终软件验收不是“新增几个pure模块/工程引用/单元测试通过”，而是：区2机构命令/Release时序正确，模拟雷达协议帧进入实际客户端，真实跨Task入口与CAN owner被调度，区2完成八阶段并回区2，最终四轮命令可追踪到实际产品控制代码；名义正常四角保持固定heading且不停车；故障/坏路径不误推进。实际板端精度、机械状态与3分钟整场另行验收，区1不算本轮完成。

## 2. 尚未冻结的采访与工程边界

1. 仅最后一个角策略仍待root采访：真实几何/障碍使某角不宜非零平滑时，是该角停稳沿雷达原路继续，还是拒该路线等待连续路径。未得到回答前不冻结fallback，不能用“首版角全停”代替已要求的正常四角非零。
2. 已定最小工程失效出口：未来预取失败不打断当前合法运动；未取得下一段合法plan则不推进；不无限重扫、不自行新建全局planner，异常留有界错误出口并复用既有严重故障停止能力。不得追加永久锁或人工比赛确认。

启动A是本轮软件导航测试入口。仍须记录R MECH-02的正式规则边界：本方案扫描展开先于底盘离区，不据此签“正式比赛启动流程已合规”，不擅自添加离区动作修改用户已选顺序。机构累计等待/最坏扫描须对RUN-04模型计时，未经实测不声称能满足15秒停滞或正式整场限制。

工程建议E而非用户实测：平移先取现baseline的a=150mm/s²、j=300mm/s³作为MATLAB起始候选；角向用已确认90/180/1440目标候选重新覆盖本轮执行合同，源时延与轮端a/j余量从已存在配置/日志核取、缺失项做显式敏感性，不造实测值。驱动236加速度字节不是主控实际加速度证明。未知真实延迟/停车误差不靠放宽验收阈值掩盖。

## 3. 已核源码与必须补齐的断点

| 现有位置 | 已有能力 | 本轮必要增量/不能误用之处 |
|---|---|---|
| `CHASSIS/HARDWARE/RADAR_V6/radar_client.c/.h` | 单普通事务、GRID/RESULT关联、匹配PATH序号/start/goal/status0/末点后发布 | 继续复用，不重注册第二套回调；测试保留真实二进制协议收发，不以直接调用on_path替代 |
| `radar_observer.c/.h` | 区2单次展开/下降/扫描/回升/收拢/一次PATH；发布Path/Status | 固化4200/及时Release；原owner明确node1/zone2本轮会话、完整占用/路径bundle与有向缓存；不把两次getter当一次原子快照，不扩区1机构 |
| `CHASSIS/USER/field_map.c/.h` | 名义480蛇形中心、Park覆盖、轴向黄区检查 | 新增有界雷达格路径到F折线转换；原函数行为保留，不用Park代替中间格中心 |
| `fusion_reference.c/.h` | 合法折线跟踪、绝对heading/沿程剩余量/横偏、静态几何与平移轮速预算 | 去除错误合速400假限，支持真正平滑corner；现wz为0、无真实A/J执行，不能直接驱轮 |
| `fusion_adapter.c/.h` | copied-value地图/raw估速与stop输入语义、完整同sample缓存 | 保留纯值用途；运行中增加实际measured路径或由owner构造measured Observation，不能悄悄把ESTIMATED改名MEASURED |
| `app_odometry.c/.h` | CAN owner持有连续地图、原子ReadMap，raw/corrected分离 | 导航run与动作scope接入；不能每站重置位姿/补偿，也不能因混合纠偏叠两次系数 |
| `heading_source.h`、`heading_controller.c/.h` | 一致yaw/rate及各自时刻；现有HoldStep PD | 实际复用PD并转为wz/轮端差速参与最终预算；不改原PD参数 |
| `chassis_auto_test.c/.h` | 原唯一CAN事务、批发送/监督、STOP、真实轮速与完成链 | 增加导航向量模式、实际已兑现命令状态、连续measured/stop事实快照；不另开CAN任务 |
| `app_chassis_service.c`、`main.c` | CAN任务10ms步进、原Safety、任务A/HWT发布、radar owner | 新profile的真实caller、唯一分支及启动/键处理；排除旧mission/机构业务，显式sample-only=0 |
| `app_input_runtime.c` | 原KeyTask服务PA1、RadarLift_Service、安全idle及PC13路由 | 新导航键分支在旧mission前消费事件；仍服务lift时钟/安全检查，不由PC13误进旧抓放或舵机调试 |
| `chassis_load_diag.c/.h` | 固定3072×24B长时记录，约73744B BSS | 仅nav=1不保留本轮不适用的大采集数组；nav=0旧采集保持，导航第一故障/批发送/measured证据继续存在 |

现 `VehicleMission_Load` 仍读取固定 `vehicle_drive` 和8项分立动作，`runtime_route`是调试上传动作表；均不是雷达执行器。保留作旧链，不扩旧容量、不把新曲线离散塞成许多“停车直线”。

## 4. 最小产品组织与真实调用链

新增仅两组必要文件，避免再造无caller薄适配层：

1. `CHASSIS/USER/fusion_motion.c/.h`：有状态A/J与制动/最终航向-平移轮端预算数学，明确当前v/a、已兑现命令和时间；没有CAN/许可。其产品调用者是下面的runtime，不以纯模块交付结束任务。
2. `CHASSIS/USER/app_navigation.c/.h`：本轮仅导航profile的runtime和八阶段小状态机，拥有Fusion计划/状态、measured采样绑定、stop事实缓存、A/J状态及run/stage身份。阶段逻辑和caller集中此处，不另拆一个泛化任务框架。

优先修改上述已有文件实现雷达缓存、地图转换、reference曲线和backend向量模式；只有真实依赖无法合理落在其中时才由medium给出理由再新增文件。`gongxun-f407.uvprojx`只给唯一工程两目标精确新增C/H引用。

真实链路应可在调用图和执行trace中逐项找到：

```text
Key / TaskA: 一键与本轮start_id -> AppNavigation请求/状态快照
Radar Task: 原RadarObserver_Service -> 原RadarClient_Poll -> 同场Map/Path bundle
CAN Task: AppChassis_CanStep
          -> AppNavigation_CanStep
          -> AppOdometry_ReadMap + HeadingSource + backend测量/stop快照
          -> FusionAdapter/受锁存measured Observation
          -> FusionReference_Step (位置/heading/几何路径目标)
          -> 现HeadingController_HoldStep / 绝对转角控制
          -> FusionMotion_Step (A/J、制动、轮端约束)
          -> ChassisAuto导航向量提交 -> 原ChassisAuto_CanStep
          -> 原CAN1四轮交易/监督/STOP
          -> AppMotion_StepOdometry -> 下一真实样本
```

命名可按现代码微调，owner与顺序合同不变。任务A不直接发轮速，雷达任务不操作底盘，调试旧runtime_route/Combined/Heading入口在导航运行时不能并行获owner。临界区只复制值/锁存，曲线准备、几何和CAN等待都在临界区外；提交前复核run/cancel/source/owner未变化。

### 4.1 导航profile

- 单一编译期 `APP_CHASSIS_NAV_ENABLE`（建议名，默认0）；启用要求明确 `ICM45686_SAMPLE_ONLY=0`、连续地图开。候选隔离构建显式启用，不能把统一工程默认仅IcmRaw路径作为整链通过证据。
- nav=1保持`APP_BASELINE40_ENABLE=0`，不为复用90°/s打开整个baseline采集分支；转向数学按导航明确调用。沿用现ICM_RAW_ENABLE默认0，不增加本轮不用的ICM记录任务。只用这一个新profile开关派生必要排除，不新增一排诊断开关；nav=0默认行为保持。
- `main.c`任务A/HWT、键服务、CAN/Safety及RadarTask的分派明确进入导航分支；`SAOMA_Update`、scan/display业务、VehicleMission、StartupAxes/VehicleArm/DM服务不被调用。禁止仅把机械“完成位”设真来绕过旧门禁。
- 雷达扫描机构仍由原observer/lift/servo owner执行已确认流程；本轮模型/受控UART代替硬件端点，不在产品中添加假扫描成功或硬编码fake路径。
- 软件真实执行候选仍保留原供电、使能、停止、看门狗和严重故障能力。`model_only/deployment_allowed`不是运动许可，不把常量改1当作接入；运行入口以新profile和既有实际许可/owner状态接通，模拟fixture始终不能触达真实外设。

### 4.1a 首个直接受益阶段：区2机构固化

- 修改仅实际编译的`CHASSIS/HARDWARE/RADAR_V6/radar_observer.c/.h`及`CHASSIS/USER/main.c`的区2配置/原邮箱必要常量引用；`CHASSIS/HARDWARE/RADAR_SERVO/radar_lift.c/.h`只在已有Release/claim语义确需修复时动。根`HARDWARE`重复副本不动，733µs与实际45°常量已经正确的不重写。
- 将实际main cfg的8559与Observer独立机构邮箱的8559/耗时计算收为同一个区2已确认4200命令常量；保留通用FourTurnCandidate为估计、原64步普通调试上限及显式bounded长程权限，不全局放宽。
- 正常序列：收拢参考180° → 45°展开/已定等待 → Enable → 下4200 → 同sequence正常完成马上Release → 停稳/一次GRID_SCAN → Enable → 上4200 → 同sequence正常完成马上Release → 180°收拢/已定等待 → 允许后续路径/导航。A/B只改变何时发起此序列，不改变每腿Enable/Release。
- 上行Release前移到stage5正常完成分支、必须先于收拢舵机命令；stage6不能因为重复Release/失能导致错误。Release只去激励，不等于Unclaim；observer在等待/收拢期间继续保留独占，最终terminal才Unclaim。`RadarLift_SafeIdle`由KeyTask持续刷新，Release后enable0不应被误认成不安全。
- 取消/故障不是正常完成，继续沿原首错和故障保相合同，不无条件Release掩盖异常，不自动回升/重试。每个正常完成的Release必须绑定精确sequence/result/done，不能只看时间。
- MATLAB先验名义单腿ceil(4200×733/1000)=3079ms、双腿6158ms加既有等待/14圈上限；覆盖实际Key1ms、Radar2ms服务和TIM7完成先后，Release“立即”定义为owner首次观察到该腿正常完成即执行，不声称机械零延迟。再用真实Observer/Lift/Servo产品代码与定时器/GPIO边界替身验同序列、两腿phase0/enable0、180°/45°实际PWM、claim未提前丢失、取消/超时与旧mailbox。
- 4200不证明高度，步骤成功不证明实物回顶；不把`measured`/候选标志伪造为新高度传感器。软件交付保留明确本轮已授权命令参数及历史实测边界。

### 4.2 持续measured与自动stop事实

- 在原CAN owner已有采集中原子发布各轮真实驱动速度、逐轮sample_ms/结果、session/run、active/state；按`Chassis_DriverFeedbackToLogical`转换方向，再用`Chassis_Forward`得到车体速度。保留异步采样时差与sample年龄，不能把ACK目标当反馈。
- HWT yaw/rate分开判断时效；世界速度使用同绑定heading转换。轮速/HWT一组完整身份采样生成单调`observation_sequence/sample_ms`；实际fresh来源集合不变不生成“新物理样本”。若只复用map sample，同sample整个Observation必须完整锁存，迟到HWT/stop放到下一fresh采样，不能破坏reference缓存一致性。
- 明确测量与差分两条来源：raw地图差分仍ESTIMATED，仅软件模型/诊断用途；实际执行的制动/停稳主要采用真实轮速与HWT rate，缺失不能把估速0升级为实测停稳。
- 当前WheelLog在active=0后mask归零，需发布/保存完成时的自动证据，不是放宽原mask定义。stop事实绑定run/session/停止请求epoch、停止后四轮实际新速度/时刻、STOP结果和HWT rate；使用已有停车事实生成，不新增人工确认。
- 起步前静止、折角/站点停止、原地转向后停止都须可自动成立。新run/再次使能/接受非零命令/对应源过期立即使旧stop失效。STOP ACK、DONE、VM关闭、desired0或差分0均不能单独置stopped。
- 起步/站间不能有逻辑死锁：旧`AppOdometry_MapStep`要求map基线建立时backend尚未active，否则置NO_BASELINE/GAP；且基线要本run begin之后的四轮位置/HWT与有效供电。扫描期间底盘VM请求保持0以满足原Lift safeIdle，收拢后先走原供电/零命令准备并由同CAN owner采集静止基线，map TRACKING后才接受首个非零向量。原`App_ComboIdlePoll`仅在Combined的1/2状态轮询额外位置，nav不用Combined，所以必须在nav PREP/STOP/站间安排原owner位置与速度轮询，不能只接getter等永远不会来的数据。
- 同run整趟保持原允许的供电epoch/地图连续性；若每站沿旧MotionFinish切VM会令continuous map GAP，不能用重置到名义站点绕过。导航backend应在真实停稳后保留允许的带电空闲，下一stage沿原epoch继续；严重故障仍按原策略切VM，不为保图伪造供电健康。
- 正常短窗/源更新先后造成WAIT不能消费进度，也不能复用旧非零输出当fresh；必须考虑此前已兑现持令并按已冻结有界失效出口处理，不能简单返回0冒充完成一次真实制动。

### 4.3 速度、heading与A/J/轮端

- 把400定义成车体平移分量上限：四向400，斜向每分量400；沿某方向允许速度按分量预算求，最大合速sqrt(2)*400。移除reference对speed/actual总速度固定400的多处假限，并保留有限值/异常速度拒绝。
- 横向回线与actual heading误差也消耗分量预算；不能对vx/vy独立硬裁后改变方向。先形成可行平移参考，再与实际heading控制输出统一求轮端命令。最终四轮<=300RPM及原协议3000 raw，使用真实kinematics/EncodeEmm，不改方向映射/单位。
- 行进heading PD复用原HoldStep。PD输出是轮差RPM，必须按真实geometry换算wz或等价有符号轮向量，再与vx/vy合成；不能把参考层wz=0/heading_reserve当成实际纠偏已执行。原地90°使用绝对target和锁定转向方向，不调用会再次按实际yaw重定目标的相对入口。
- A/J状态以已被原backend接受/实际批次兑现的命令为基准，目标改变、急减速、横移符号变化、heading纠偏都受向量及逐轮约束；不能每次调用把a重置0。四轮一批命令在边界锁存，批内不拼两代向量；保留逐轮发送与回包/监督间隔的实际时间。
- 制动可达性覆盖当前v和非零a、jerk建立制动、源时延/运算/排队/四轮持令、轮端余量。`sqrt(2*a_hint*remaining)`只可当desired建议，不作真实停车证明。不可行要及早进入已冻结恢复/停止出口，不放宽包络制造通过。
- 不得对最终轮速硬截断破坏A/J；应在规划/执行共同时间缩放、降低转弯速度或提前制动后重新核算。驱动量化0.1RPM及批次零阶保持单列，不把数学连续性宣称成物理已验。

## 5. 雷达、转换及八阶段的最小合同

### 5.1 同owner bundle与有向复用

原observer内部保存并一次原子发布：本地run/start、node/zone、local scan epoch、scan_sequence、layout版本、完整25格占用位集/列表、路径request_sequence/start/goal/status/count/waypoints及发布epoch。PATH未回显的run/layout/scan关联明确是本地主控绑定，不能写成设备返回字段。

事务串行，优先当前所需路径；其它路径可在当前段可行后预取，不无条件等全6条才允许开始。整趟只使用已确认的一次开局扫描图，不新增每路径/每站重扫；一键A扫描收拢后仅在当前合法路径/静止基线已就绪时开始运动。未来预取失败只记录该条未就绪，不打断当前合法段。

| 阶段 | 有向请求 | 起终heading |
|---|---|---|
| 1 | S→2 | +Y→+Y |
| 2 | 2→14 | +Y→+Y |
| 3 | 14→10 | +Y→−Y |
| 4 | 10→22 | −Y→−X |
| 5 | 22→14 | −X→+Y |
| 6 | 14→10，复用同场P3 | +Y→−Y |
| 7 | 10→22，复用同场P4 | −Y→−X |
| 8 | 22→S | −X→+Y |

本轮S固定格0、start_id=2、node=1、zone=2；不按zone号猜node=2。PA1锁存不是协议选区完成，选到区1时本轮明确非可用profile，不自动镜像到区2或执行区1。相反方向不可倒序缓存。新扫描、重连/重启且图连续性不可证、node/zone/layout改变使旧bundle失效；正常车辆运动不自动清除同场静态地图。仅这6种有向请求纳入实现与本轮验收，区1后续另议。

### 5.2 格路径到实际F路径

- 用已确认480、蛇形映射和同向G→F求几何中心。压缩长段必须先展开空间中间格，逐一验在场界、无占用、连接轴向/拓扑合法；不能按格号差乘480。
- 黄色精确盒(两轴[550,1000]/[1400,1850]的四组合)、场界及实际283×254矩形/heading/误差预算始终二次检查。占用仅有格信息时按完整格保守约束，不虚构phi50障碍在格中心；可用障碍坐标若另有同场可靠来源再单独规划，不本轮推测扩展。
- 当前实际车体位姿与第一格、最后格与明确停车点需要独立入口/出口连接；格中心不强写成实际位姿。若当前位姿/不确定度不支持缓存start入口，按最终失效策略重新请求或拒绝，不用旧目标当实际起点。
- 容量由24返回节点展开后的最坏原始简单路径、入口出口和每角最多一个blend推导；先去重/去共线，再证明确切上限。禁止静默截断，不能仅增加旧MotionAction容量解决。

### 5.3 八阶段runtime

`IDLE → 一键SCAN_PREP/SCAN/STOW → MAP_BASELINE/PATH_READY → FOLLOW/TURN/STOPPING → STATION_STOPPED → 下一stage → RETURN_DONE`等有限状态集中`app_navigation`；真实SCAN/机件状态由Radar owner发布，app只请求/等待，不重复写舵机或升降。阶段绑定run/start/map/layout/path/方向与绝对heading，缓存命中仍重置本段执行状态、不清全场地图。

到站必须位置/heading、真实速度与同run自动stop同时满足，自动进入下一段；初次prime不伪停稳。第8段回start_id=2，检查283×254主体全入300×300的工程保守回区模型及停止，不将最终中心命中当规则验收。车辆估计位置与模型未实测，返回模型通过不称回区实车通过。

## 6. 四角固定heading非零过渡：必须改变几何和状态

当前reference把非共线节点cap设0且等待stopped；单把cap改非零会产生速度方向突跳，不能采用。也不能把直角改成硬折线“直→斜→直”，或用直线接圆弧而忽略曲率跳变造成的加速度突跳。

最小建议为**一种固定heading的C2五次Bezier角过渡**：从同条原路径角前后裁出经包络验证的有界长度，直线两端位置/切向连续、曲率从0进入再归0。全程车头保持原planned heading，中心平移方向经过斜向；不引入移动转头或全局优化器。Bezier控制点凸包加旋转后车体支持函数/误差预算可作保守几何覆盖，必要时区间细分；不是少量点截图即宣称无碰。

- `FusionPlan`保留原有向点/段身份，新增有界LINE/BLEND标记及必要预计算；不存无限密集点。先确定line endpoint/curve入口与出口的段进度定义，再改reference状态。
- normal LINE→BLEND→LINE按单调弧长顺序推进，不要求命中小球、不global-nearest跳分支；进入/退出blend只改变几何primitive，保持v/a和heading/PD状态，不请求stopped、不释放backend会话。
- 弧长表/求值误差、连续曲率与曲率导数形成有界误差；速度帽同时满足切向加速、法向a=v²*kappa与jerk(v³*kappa'及加减速耦合)、轮端速度/a/j。短角不能一边保持565.7一边宣称150/300成立，必须提前降速。
- 几何裁长从可用相邻段/原走廊决定，不写未经验证固定斜切长度。邻角过近则共同收缩/降低速度；不可行的角fallback等待最后访谈答案，不能切黄区。名义合法四角用例必须有非零解，否则本任务的“四角不停”尚未完成，不能以全停fallback冒充完成。
- 中央路口先按本轮已定提前降速与精确几何检查；不擅自把“四角不停”扩大为所有路口不停。障碍使非零过渡不合法时允许安全出口，正常与故障用例分别验。
- 90/180在停车点的旋转扫掠必须精确比较锁定CW候选与CCW候选，不能继续以整圆AABB无法区分方向为由宣称CW失败后CCW已实现；若空间确实两边等价，应如实验得该结果。两次90保留原绝对目标。

## 7. 分阶段实施、MATLAB前置与本地保存

每个对应产品改变前，先运行本机MATLAB并保存版本/开始结束/退出码/脚本与输入SHA、未改产品SHA；失败保留首错。Python/C测试不能代替此前置。脚本可为验证新增，不能先写产品再补跑模型。采访完成前只允许本规划文档工作。

| 阶段 | 修改范围/真实调用成果 | MATLAB先验与该步通过条件 | 计划本地commit边界 |
|---|---|---|---|
| P0 合同收口 | 更新本目录最终决定、本文待问项与基线清单；不改产品 | 最后角fallback收口，核R/E/Q与源版本；固定A启动/区2/一次扫描/90°s目标 | 规划/采访记录独立保存，不混旧dirty |
| M 区2机构先补齐 | 仅真实main cfg/Observer常量及上行完成Release时序；共享4200命令参数，不改根重复副本 | 先本机MATLAB机构时序矩阵，再实际Observer/Lift/Servo+边界替身、两目标构建与high检查；上下4200/733µs/两腿及时Release、故障语义不退化 | 机构固化+对应模型/C/报告单独保存，明确未下载/实测 |
| P1 动态前置 | 验证模型，不写产品C | 当前v/a非零、制动、源延迟/四轮批次、heading PD合成、body400/diag400+400、实际量化和<=300RPM；失败场景可正确拒绝 | MATLAB模型/实际回执及冻结向量 |
| P2 运动与原owner | 新fusion_motion/app_navigation；reference速度语义、feedback/stop发布、backend向量模式及main/app_chassis真实导航调用；无有效计划就idle，不在产品硬编码测试路径 | P1通过后才编码；ARM真实C贯穿runtime→最终四轮发帧，固定F计划仅由测试fixture喂入公开计划入口；四向/斜向/holding/旋转/停止/旧链隔离通过 | 产品运动执行+对应C证据；尚未称全场完成 |
| P3 PATH与八段整链 | 原observer区2同场bundle/有向缓存；field_map转换；app_navigation区2八阶段；真实key/TaskA/Radar/Can分派；更新唯一工程引用 | 先MATLAB验蛇形/长段展开/入口出口/缓存身份和阶段、定位漂移/不确定度；再实际radar_client帧到8段mock外设整链，区2/6请求与重复复用/旧图错图被拒绝 | 雷达/转换/八段产品与跨Task证据；硬折角仍有显式未完成标记 |
| P4 非零四角 | reference有界LINE/BLEND几何/状态，motion速度与曲率预算；同runtime执行，不另起第二条轨迹链 | 先MATLAB验C2几何、连续扫掠、弧长误差、向量/轮端a/j与正常角非零；再ARM和跨Task端到端验证四角不请求stop，轨迹不切黄/撞障 | blend产品/模型/端到端证据，完成本轮功能目标 |
| P5 独审与交付 | high独立审实际最终diff/输入/trace，修复必须回对应MATLAB/C；根交接/长期记忆按最终事实更新 | 完整nav Integrated/Offline与旧profile构建、资源/栈静态证据、最终SHA整链重复；无未解释首错 | 最终检查/交接及必要修复独立保存，始终不fetch/push |

P2/P3只是内部可复查阶段，不可停在中间宣布用户目标完成。若P1模型已能同时覆盖P3/P4，则可一次冻结相应前置向量，但后续新增算法/修复超出模型仍须先补对应MATLAB。只运行受影响回归，不在每步重复无关机械/视觉全套测试。

## 8. 真实C与跨Task软件验收矩阵

### 8.1 实际产品代码，而非独立仿写

沿用ARMCC真实C + Unicorn夹具方法，并补充一个确定性多Task调度fixture：实际Key/TaskA导航入口、RadarObserver/RadarClient/link_protocol、AppNavigation、FusionReference/Adapter/Motion、Odometry、HeadingController、ChassisAuto与原CAN事务产品代码都编译进去；只在RTOS时钟/临界区/调度和UART/CAN寄存器/设备应答边界提供受控替身。不将高层Service、路径转换、向量限幅或自动stop替换成永远成功的桩。

如果main任务入口是static，最小只做必要的可调用service包装或在fixture同编译单元引用，不把产品核心逻辑复制进测试。分别记录真实参与的源文件/函数/编译宏、边界替身与未覆盖的硬件行为。真实RTOS调度延迟不能由该fixture签署。

模拟雷达使用现协议封包/CRC/node/ACK/GRID/locked STATUS/PATH分块顺序，先通过真实UART port消费接口再到client，禁止直接写published_path。模拟CAN保存逐轮实际发送/ACK时刻、使用方向/0.1RPM/加速度字节并生成对应真实速度/位置应答；动力学模型与固件执行分别报告，不把设备理想跟随当实车证据。

### 8.2 必须有结果的并列场景

| 组 | 正常与边界覆盖 | 必验断言 |
|---|---|---|
| 区2整场 | start2、8阶段、一次开局图、6有向请求、P3/P4复用、最终回区2 | node1/zone2/run不串；stage只前进一次；无扫码/抓放/StartupAxes调用；总时间为模型值，区1明确不在本轮 |
| 区2机构 | 一键A的180→45/下4200/Release/一次扫描/上4200/Release/180；沿保存校准 | 定时器/sequence/result/claim与失能时序正确；未到位或故障不误行车；不上电自扫/不每趟校准/不自动重复扫描或升降 |
| 正常运动 | 四向、斜向±400/±400、heading小误差与横向回线、短/长段 | 实际最终body分量/轮端与A/J限制；主向补偿只一次；纯斜/旋转标未标定 |
| 四角blend | 0/4/20/24、相反行进方向、相邻短段、所有已定hold heading | LINE/BLEND非零连续，heading不随路径切向转；不发送STOP、不重置a/PD；连续主体包络合法 |
| 90/180 | CW优先、CW可行/仅CCW可行模型、角度跨±180、第一次欠转 | 两绝对目标不漂；方向锁；最终heading由真实HWT闭环达到，wz非零实参与轮命令 |
| 雷达身份 | 错node/zone、ACK丢失/乱序、旧PATH、错seq/start/goal、空路径、24节点/容量、占用起终点/不可达 | 不误成功、不直行fallback；有界错误出口；占用完整且bundle原子 |
| 来源/反馈 | 源延迟/回绕、旧/同timestamp冲突、单轮掉样、各轮异步、HWT角新率旧、measured与ACK不同 | 无伪fresh/伪零速/伪stopped；历史stop不跨run/新非零命令；首窗不推进 |
| owner/竞态 | cancel发生于prepare/commit/四轮批中、旧调试请求并发、雷达重连/更新图 | 只有原CAN owner发帧；每批身份一致；cancel优先；失败不消费下一stage |
| 动态不可行 | 当前v/a过大、下游短距离急停/障碍、轮端饱和、晚到观测 | 首错保留，提前有界响应；不改driver300/不减包络/不放宽源阈值凑通过 |
| 停车与返回 | 站点横偏未收敛、速度0但未STOP、STOP ACK但仍运动、回区中心对但角点越界 | 全部不能误DONE；真正位置/heading/自动stop齐全才续行/终结 |

计数按实际断言/向量/独立场景分别报告，禁止将重复循环包装成独立场景；主夹具与high独立夹具不相加造覆盖。模型首错、未采用候选和修复前失败都保留。

## 9. 构建、资源和旧链回归

- 唯一工程两目标各构建nav=1/sample0/map1的实际M4F代码，并另外验证nav=0旧Integrated/Offline；默认sample1构建可补兼容但不代替完整入口。检查编译日志、链接输入及map/callgraph中确有runtime caller和新模块存活，不能再以链接器移除未调用模块的结果称接通。
- 旧六dirty逐字节SHA前后保持；固件原GBK/CRLF、工程原LF、新源约定ASCII/CRLF保持；不批量转码。实际修改只对应本文范围，旧vehicle_mission/runtime_route/原PD/控制倍率/机械源码优先不动。
- 读最终.map按IRAM1 128KiB与CCM64KiB分别核RW/ZI/堆栈分布，不以Code+ZI总数猜剩余。当前冻结Integrated报告ZI=124004B，但具体map为IRAM1 size0x17940、IRAM2 size0x7940，不能把ZI总数说成单区余量。DMA/UART/CAN缓冲不得放CCM；新增CPU-only计划上下文若需要搬移，先给实际布局证据，不盲改scatter。
- 已核`app_fusion_capture.c`只有`APP_BASELINE40_ACTIVE && APP_FUSION_CAPTURE_ENABLE`时分配固定bankA@0x20017300、bankB@0x10007300（既有34020/33600B）；`APP_FUSION_CAPTURE_ENABLE`默认1，`app_icm_observer`128条历史同样只在baseline打开。因此nav必须baseline=0，不能为借90°/s把旧bank带进来；旧baseline profile构建检查原地址/容量/采集语义完全保持。
- 新nav=1还需仅排除`chassis_load_diag`本轮不适用的3072×24B采集数组（实际旧map .bss=0x12010/73744B）；已有记录接口在nav返回明确无此采集，不伪造空记录为通过。nav=0原实现不变。新的有限导航trace/第一故障、批发送统计、实时measured/stop快照及原Safety仍保留，不把删除诊断当降低保护。只复用单个nav开关，不新增一堆capture配置。
- FreeRTOS heap固定20KiB；实际main任务栈CAN384字、Radar768字、TaskA512/256字、Rx384/256字、Key/Safety各256字。新nav不新增任务；最坏局部栈/调用深度必须核含真实linked curve/Prepare/PD路径，不能把FusionPlan/FusionInput大对象当局部变量放进去，不能只看链接总RAM就声称栈足够。
- 计划/曲线/完整bundle/缓存均固定上限；大对象caller-owned或单owner静态放置，不塞进CAN/Radar任务栈，不为每站复制多套1408B以上计划。先给sizeof、链接分配与最坏局部调用栈；当前sizeof不等于实际余栈。
- 不新增RTOS任务；检查既有任务stack分配与总heap开销。保留 `g_can_task_stack_words` 等现场高水位接口，模型/ARM静态检查不能签真实余栈；未连硬件就明确真实高水位待验。
- 分析CAN每批发送/监督/停止最大交易数和模拟最坏耗时，HWT/雷达收包不被计算或临界区饿死。新量化/曲线最坏执行成本保存实际ARM指令/编译器栈或主机成本证据，区分与F407实时测量。
- high独审针对最终SHA的全部相关改变、真实caller/编码器/停止链与前置时序独立检查；如发现有效问题先修复再复验，不以旧pre-review替最终审查。

## 10. R/E/Q、未闭环及最终交付措辞

R逐项检查：FIELD-01/02/06、OBS-01/04、RUN-01/02/03/04/07、PWR-01、MECH-01/02。导航测试不执行物料业务，不宣称RUN-02/03已获实物验收；仍保留这些正式任务规则边界。

E：LOC-02/03/04/05/06/07、MOT-01/02/03/04、CAL-02、FIELD-03/04/08、RUN-05/06及本轮设备480/GF/测试停车点/速度与软件合同。Q：FIELD-05、OBS-03、QR-04继续待技术答疑，不由工程模拟作官方答复。

交付需包含：最终决定/计划、实际MATLAB与ARM/跨Task trace、最终独审、两目标完整profile与旧链构建回执、资源与旧dirty保留核查、根交接/长期记忆、本地分阶段提交号。未闭环明确列真实停车/回区误差、源时延/滑移、混合补偿、实物雷达设置/扫描升降、真实栈余量及区2三分钟实车整场；区1留后续，不列本轮缺功能或追加验收。

最终只可称“本地软件整链实现并通过所列模拟/真实C/构建验收”；没有操作硬件就不能称已安装、已下载、已完成实车或规则验收。不fetch/push；不把工作区旧dirty、所有磁盘文件或默认裁剪AXF说成此次已交付/已验证。
