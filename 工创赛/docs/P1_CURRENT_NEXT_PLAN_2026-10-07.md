# 当前后续计划（2026-10-07）

## 2026-11-03 最新接续入口

以根目录《队友接续_2026-11-03.md》及MAP_STAGE1_TUNING_2026-11-03.md为当前入口。下一步优先确认现场并下载第一阶段候选，由用户按PC13单次启动，USART1只读保存日志，验收停车/真实读距/任务码/串口屏。之后处理重复运行记录容量、整合机械臂和八阶段调度，再接雷达路径并最终三分钟演练。当前未下载本候选，八阶段/机械臂/雷达未全部整合；下方历史计划按日期理解。

## 2026-10-28 20cm/s实车验收方案准备完成，待现场执行

本地新增docs/P1_SPEED20_VEHICLE_ACCEPTANCE_2026-10-28.md：冻结AXF/PLAN7核对、起点与90/120/210cm标记、30cm中段视频计时、完整日志及地面尺量、停车/VM/故障记录和首轮工程筛查。模拟参考在约85.7～124.5cm达到19.8cm/s以上，因此90～120cm仅作为候选测量窗；不能预判实车地速。首轮18～22cm/s、纵向±4.2cm为工程筛查，不是官方验收线；横偏对照既有低速右偏3.25～4cm，并单独核机械平顺、通信与停止。恢复实测条件后按此单次受控验证，再决定是否重复及针对性调参。

本轮不改实时固件、冻结候选、主机或保护；未开COM/烧录/复位/发车。fetch/pull --ff-only确认main/origin/main均54bb617、0/0；保留既有未提交成果，仅本地更新交接，未提交推送。核R RUN-01/PWR-01；E LOC-03/05、MOT-02/04、RUN-06；Q不变。

## 2026-10-27 20cm/s模拟过程图完成，候选参数未改

按用户确认继续补齐模拟过程记录：测试适配器每10ms通过Unicorn钩子导出参考位置/速度/加速度/jerk、四轮桩发送目标、假定模型轮速、异步缓存反馈/龄、编码进度、调用计数及停止/故障状态。新增scripts/p1_simulation_report.py，修改电脑端four_wheel.c适配器、p1_four_wheel_sim.py和共享离线图表工具；记录仅进入测试AXF，真实固件/主机发车脚本/schema和候选包均未改。没有开COM/复位/烧录/发车。

证据evidence/p1-foundation/2026-10-27/speed20_sim_trace_validation/，其中four_wheel/index.html为10场景总览，case_00/01为正常过程，其他八项为预期保护场景，各有trace.csv/json与report.html；trace_review.json共6648个10ms样本。与原offline_tools_validation_final/run_05低速B2实测图分开，页面明确写纯软件模拟，不是20cm/s实车日志。说明docs/P1_SPEED20_SIMULATION_TRACES_2026-10-27.md。

两个正常模型参考峰值20cm/s，参考A约2.3521cm/s²/J约3.0564cm/s³，原计划19.7726s；模型后端DONE约20.11/20.03s（含停车确认，从已设置RUN起计，不含真实准备）。到距时最后整数基速1/2RPM、当时原参考速度约0.82/1.01cm/s，仍有离散停止，不能签真实jerk连续或无顿挫。停车后保留原计划参考用于对照，不表示继续下发。异常CAN停推进场景将停车请求、实际停止桩调用与500ms撤VM请求分别记录，未伪造停止指令已发。

统一验收passed：39项工具测试、原host105测试、参数/候选1221项通过；真实后端/规划器/运动学/预设C的10场景50897断言（含循环）通过，增加参考V/A/J限值、相位边界连续与端点检查；Python过程检查覆盖采样、RPM上限、末端接近、正常停止与故障状态。HTML语法与独立本地无头浏览器截图已验证。模型仍为10ms调度/瞬时桩交易/设定的一阶轮速响应，RUN前置状态由桩设置，不模拟真实CAN/RTOS、VM准备、HWT控制器、地面侧滑及故障断电后的滑行。

task_source_audit.json对本轮开始前/结束后的2334个实际工程、host/schema和候选文件核对无变化。根默认开关仍0、专用候选开关1，冻结AXF SHA仍7f2228dee9e38f9b943e4d4704e6cee44bf74e2f9d80abf3046c138032e63543；仅隔离测试编译/运行，未重做固件全Rebuild，保留全部既有本地成果。后续先保留候选，等待用户恢复实测条件后新确认现场，再验证20cm/s实际速度、距离、载物平顺、停车及负载；不凭假定模型调参或放宽保护。fetch/pull确认main/origin/main仍54bb617、0/0；仅本地未提交推送。核R RUN-01/PWR-01；E LOC-03/05、MOT-02/04、RUN-06；Q不变。


## 2026-10-27 六项电脑端工具完成，离线验收通过，未操作硬件

按用户授权完成日志分析/交互图、版本与地面尺量对照、参数一致性检查、四轮独立响应实际C模拟、统一软件验收入口、说明与交接。新增scripts/p1_motion_offline.py、p1_four_wheel_sim.py、p1_validate_offline.py、test_p1_motion_offline.py及p1_offline_tests/。使用说明docs/P1_OFFLINE_TOOLS_2026-10-27.md，五份历史清单docs/P1_OFFLINE_COMPARISON_EXAMPLE_2026-10-27.json。

最终统一验收evidence/p1-foundation/2026-10-27/offline_tools_validation_final/validation.json为passed：新工具29测试、原host105测试、参数/包1221项检查、实际C四轮10场景17327断言（含循环，不是17327独立场景）通过。两场景正常完成，其余八场景分别注入单轮偏慢、速度/位置应答缺失、停止应答缺失、超速、反向、航向邮箱过期、停车期间CAN停推进，要求原保护报错并请求撤VM，不能把保护停当运动成功。RUN前置状态由适配器设置，10ms调度/瞬时CAN桩、独立轮速延迟/一阶响应均为假设；不证明VM准备、HWT控制器、真实总线/RTOS时序、地面横偏或机械平顺性。首轮模拟适配器环形历史索引有误，修正测试适配器后通过，未改固件来迎合模型。

统一入口仅固定软件步骤，不含串口/复位/下载/发车；模拟显式选--simulate，缺项明确skipped、依赖缺失报错，不自动安装或跑七配置全构建。分析只读JSONL/summary；时钟回绕/旧缓冲/重启/START分块，BT历史去重；原主机状态、编码进度、地面尺量与未量分开。待机基速配置不画成运动参考，不构造未上报参考位置或真实轮指令。5套历史图/CSV及comparison已生成，A2仍telemetry_timeout，B2仍motion_fault_or_cancelled/distance9；页面明确提示图表生成不代表实车通过。Node脚本语法及本地无头浏览器显示已检查。

验收前后实际工程、主机脚本/schema和冻结候选包2334文件SHA一致；实时固件/主机发车脚本/schema未改，两个启用开关默认0不变。候选冻结包1167文件校验通过，AXF SHA仍7f2228dee9e38f9b943e4d4704e6cee44bf74e2f9d80abf3046c138032e63543。保留先前未提交成果与删除状态；本轮未新做固件全Rebuild（只有隔离测试AXF的编译/模拟），未开COM/烧录/复位/发车；最后实际下载仍按历史B2记录，未读取当前硬件状态，20cm/s未实测。

下一步等待用户恢复实测条件，再新确认现场验证20cm/s单条候选的实际速度、距离、平顺、停车及负载，随后用这些工具归档分析。fetch/pull --ff-only确认main与origin/main均54bb617、差异0/0；全部仅本地完成，未提交/推送。核R RUN-01/PWR-01；E LOC-03/05、MOT-02/04、RUN-06；官方Q不变。


## 2026-10-27 20cm/s单条候选软件完成，按用户要求不烧录/不上车

候选包已实际完成并校验1167个冻结文件，AXF SHA256=7f2228dee9e38f9b943e4d4704e6cee44bf74e2f9d80abf3046c138032e63543，见speed20_package_review.json；本轮未读取当前硬件供电/故障状态，“原B2”指最后下载与实测记录。

用户将提速安排改为“直接20”，随后明确“先别烧录现在没办法上车测试”。当前仅本地代码、仿真、完整构建和候选包，无COM/复位/下载/发车；板上仍原B2故障候选。后续再新确认现场，不沿用10月26日就位答复。

新增PLAN7/forward210-20，仅前进210cm，参考峰值200mm/s，根工程两个开关默认0，专用debug/mode3候选S曲线/提速开关1；普通/比赛拒绝该入口。30cm不能形成当前加速度下20cm/s匀速，未开放高速短段/横移/定时。52RPM为位置跟随余量后的基速上限、航向修正后单轮最大54、反馈阈值80；普通15/30限值不变。高档P1/s、位置修正±2RPM，预留4RPM/s/12RPM/s²后参考A约23.52/J30.56，计划约19.773秒、峰值200mm/s；低速仍旧Kp0.5/±1。高档25秒运行/45秒总兜底，原2mm/2秒/50mm、源100ms/位置400ms/停止500ms、VM/故障锁存/看门狗保留。高档编码跳变121counts/ms配套80RPM反馈；低速66不变。

前端仅选中该单条动作表内部可提交高档，普通MoveStart/距离/横移仍封闭；后端Prepare/Begin双核精确参数与预算。新host显式选择PLAN7、接受回显后再健康检查（base52/feedback80/count1）、ARM/START，默认/未指定不能启动高档；运行档位/距离形状变更STOP，0.6秒失联不放宽，历史帧不保活。TM1字段不变，旧冻结host不混用。

实际C默认288、低速211086、高档245927断言（含循环）通过，高档九组0/80/160ms延迟×±3%模拟速度偏差均到距；实际前端提取与后端拒绝/超速/源失效/停止/不动超时覆盖。初版高档1RPM余量-3%末端不足报9，最终专用增加2RPM余量并扣预算，未松保护；正速度增益模型用47RPM可达到200mm/s，验收据模型速度而非固定RPM下限。主机105测试通过。三个完整Rebuild0错0警告，Code62840/72364/54068。仿真不证明MCU时序、机械jerk或地面精度，20cm/s仍未实测。

说明docs/P1_SPEED20_TRIAL_2026-10-27.md；证据2026-10-27/speed20_review.json、build/checks/任务差异；冻结候选包releases/p1_speed20_candidate_2026-10-27，使用其工程和新host，不把旧低速包/旧AXF混进新记录。旧包保持不变，源GBK/CRLF、CAN/驱动及既有保护保留。实际速度、停止包络、运行开销/栈、距离比例和平顺均待后续现场。fetch/pull54bb617一致，全部仅本地未提交推送。核R RUN-01/PWR-01；E LOC-03/05、MOT-02/04、RUN-06；Q不变。

## 2026-10-26 S曲线位置跟随/有限末端接近候选完成，未烧录；后续转20cm/s分级配置

用户授权继续修复末端衔接，随后明确低速看不出效果，后续要达到首版20cm/s。当前只完成末端修复，不重复安排多轮约6cm/s比较，不把20cm/s写成已开放或实测。

仅改USER/app_config.h及chassis_auto_test.c：校准位置误差P目标0.5/s，限±1RPM，经omega1/s二阶临界阻尼平滑，至少10ms更新、dt最多100ms；初始化/撤销/新会话清零。计划从原V/A/J预算预留1RPM、2RPM/s、6RPM/s²，参考峰值14RPM、合成基速上限15RPM，准备/提交双核新预算。减速段未到距时连续保留1RPM末端接近，仍在原duration+2s内，原2mm到距、50mm跟随误差、实际总时限、源/编码新鲜度、500ms停止、VM/故障锁存保留；不动仍reason9，不自动清故障或补无限行程。距离系数和主机/遥测/CAN未改，源S曲线默认0。

连续参考＋滤波修正预留预算不等于完整轮速jerk已验收：整数命令、末端1RPM下界、原到距/紧急停止及航向修正仍需实物。滤波expf锁内最多100Hz，运行开销/栈余量待实测。新命令含有限末端接近，不能继续引用旧候选“参考结束只零速等待”作为当前行为。

默认实际C回归286断言、候选211084断言（含循环）通过；三方向300/1000/2100mm合法段、0/80/160ms延迟、±3%模拟速度偏差、B2残差与不动超时/停止/源失效等覆盖。主机96测试通过。夹具长距跳跃缺少历史原编码首轮误触发跳变，修夹具后通过，未松保护；扩充仿真指令预算与固件时限无关。默认/candidate/比赛完整Rebuild均0错误0警告，Code62408/71680/53716，默认/比赛大小不变。仅GBK/CRLF窄修改，main.c及其他后台/驱动SHA核对原review一致。

新工程tmp/scurve_follow_default_20261026、candidate_20261026、competition_20261026各有完整源码/AXF/log；证据2026-10-26/scurve_follow_review.json、三个build.log、default/candidate_checks.json、task_only.patch；说明docs/P1_SCURVE_POSITION_FOLLOW_2026-10-26.md。新代码未烧录/开串口/发车，板上仍原B2故障候选，旧冻结包不修改。

下一步准备20cm/s所需速度/反馈/加减速/超时协调档位，再按少量分级短程各一次验证；当前15RPM命令/30RPM反馈门限不变，不能直接输入20cm/s发车。低速B2地面209.25cm/右0.75cm与编码残差2.514mm保留，末端修复尚无实测结论。fetch/pull54bb617同步一致，既有成果保留，仅本地未提交推送。核R RUN-01/PWR-01；E LOC-03/05、MOT-02/04、RUN-06；Q不变。

## 2026-10-26 B2尺量结果：前移209.25cm、右偏0.75cm，末端未闭环

用户反馈“前移209.25cm  右偏0.75cm”。地面相对210cm少7.5mm；故障时校准编码估计2097.486mm、少2.514mm，两者必须分开记录，不能拿编码容差当作地面精度。现场停稳灯灭和起停顿挫本轮未单独明确回答；末态遥测四轮0/VM0，保留原故障，不再发车。

B1为30cm/右0.8cm，旧分档修复版210cm/右3.25cm，本次S曲线B2为209.25cm/右0.75cm。仅各一轮且初始HWT方向/尺量条件未完全同质，横偏数值较小不能证明曲线必然降低侧滑或总体精度已通过。B2按reason9失败保留，待先设计速度曲线结束与实际距离收敛的衔接；不能直接调整前进系数或以放宽2mm门限消除报错冒充修复。

已更新scurve_B2_forward210_physical_review.json及scurve_B2_forward210_20261026_214229/review.json、measurement.csv。未改源码/编译/烧录/清故障/新发车，当前板上候选故障状态保持。低速集中测试已取得A1/A2/B1/B2实际结果，但B2未通过、平顺性未判定、底盘阶段未签完成。下一步先向用户说明末端控制改动方案，再按授权实施与软件验证；提速目标20cm/s尚未开放。仅本地记录未提交推送；核R RUN-01/PWR-01，E LOC-03/05、MOT-02/04、RUN-06，Q不变。

## 2026-10-26 B2低速S曲线210cm末端保护停止，未通过，保留故障

用户确认B1停稳灯灭并回起点后，复位同一candidate已构建AXF（Verify OK），只读检查新会话健康，明确PLAN6/forward210单次执行；无重编/参数改动。当前板上仍启用S曲线且带遥测优先修复，源默认开关0不变。

B2在参考曲线结束后基速0，编码进度停在约2097.5mm，当前2mm到距容差要求至少2098mm；零速等待原2000ms后，61440ms触发distance_fault_reason9。末态2097.486mm、短约2.514mm是编码估计，不是地面尺量；不得因为只超出容差约0.514mm便改写为通过。主机motion_fault_or_cancelled，完成0/1，phase4/trip5，backend_result2/cause3，主机收到故障后按原策略发送STOP，不自动重试/清故障。

末态四轮0、VM请求/输出0、post_settle15；stop_mask0，不能声称四轮停止控制应答齐全。CAN/RTOS/safety标志0、HWT源有效/无串口溢出或校验错、最大采样间隔11ms，后台命令龄10ms/反馈130ms。HF2通用backend_fault_reason8与distance reason9为不同字段，不能误读成曲线参考异常reason8。末端参考/实际距离残差是直接触发条件；1RPM量化、四轮异步/离散发送及实际轮速跟随各贡献待分析，不强指某个电机或HWT。

记录有效433/无效0、seq gaps75、末态telemetry_dropped4；BT1 17帧/16不同记录，累计91成功/0中断、覆盖75，整批最大67ms/接口1ms/初始排队394ms。CAN栈171words、RX244words、最低堆9792B。本次无成功PE2端点，不凭编码反算签地面精度，也不称遥测收尾超时复发。

证据evidence/p1-foundation/2026-10-26/scurve_B2_forward210_20261026_214229/（download.log、idle/run原始记录、summary、review.json、measurement.csv）及scurve_B2_forward210_physical_review.json。现场停稳灯灭/实际距离/左右偏移/平顺观察待反馈；已提示保留原位和故障、本轮不再发车。B1现场停稳灯灭按本轮明确就位答复补记。下一步先收尺量，再解释末端控制方案；不直接放宽门限、延长等待、清故障重跑或提速。仅本地实测与记录未提交推送。核R RUN-01/PWR-01；E LOC-03/05、MOT-02/04、RUN-06；Q不变。

## 2026-10-26 B1尺量反馈收齐：前移30cm、右偏0.8cm；平顺性未判定

用户反馈实际前移30cm、右偏0.8cm；关于起步/减速和末段停走，原话“看不太出来现在速度太慢了”。按低速下无法判断平顺改善记录，不写成更丝滑、无顿挫或已满足jerk要求。软件完成/四轮0/VM0已见上一条实测证据，现场停稳灯灭本轮未单独明确回答。

已补齐scurve_B1_forward30_physical_review.json及scurve_B1_forward30_20261026_213526/review.json、measurement.csv。当前板上候选固件不变，无新编译/下载/开串口/发车，原保护/最高15RPM/1RPM输入保持。下一步可在新现场停稳灯灭/回起点及210cm路径确认后，用同一候选单次B2前进210cm，不因低速看不出差别直接提速。B2尚未运行，20cm/s仍为后续逐级目标。仅本地记录未提交推送；核R RUN-01/PWR-01，E LOC-03/05、MOT-02/04、RUN-06，Q不变。

## 2026-10-26 B1低速S曲线30cm已软件完成，现场距离与平顺反馈待收

用户确认停稳灯灭并回起点后，下载telemetry_priority_candidate_20261026，AXF SHA=eaac6041c8181c582485e14a3acd79f23a9d2886aadb384fe3115992903fc33e，Verify OK/Application running。首次仅下载/只读待机即被中断，未发车；用户“重新测试”后只读确认仍未运行，随后明确PLAN2/forward30单次执行，没有重复烧录/重编或清故障。当前板上已是启用S曲线且带遥测优先级修复的候选版；实际主工程默认开关仍0，最高15RPM、1RPM输入及原保护不变。

本次software_complete，1/1段、phase3、stop_mask/post_settle15、四轮0、VM请求/输出0，CAN/RTOS/safety/distance故障0，UART接收错误0、源最大采样间隔11ms；主机无STOP，无收尾超时。TM观测基速0→1→…→15→14→…→0；这是整数应用目标，不等于实车轮速/jerk连续。到距进度299.812mm使用候选原2mm到距容差，PE2名义投影293.355mm，两者均不能代替地面尺量；完整动作会话约9.479s含准备/停车等，不写作纯行驶时长。

收到BT1 17帧/16个不同批次，累计33成功/0中断、覆盖17，整批最大67ms、接口1ms、初始排队394ms。有效134/无效0、seq gaps124、末态telemetry_dropped4，保留原始日志，不声称无丢帧；CAN栈171words、RX244words、最低堆9792B为本次观察。

证据evidence/p1-foundation/2026-10-26/scurve_B1_forward30_20261026_213526/download.log、idle/idle_resume/run原始串口与summary、review.json/measurement.csv，以及scurve_B1_forward30_physical_review.json。当前现场停稳灯灭、地面前移/横偏、起步减速是否平顺/末段有无停走均待用户反馈，未判实车平顺通过，不自动进入B2。上一轮修复210cm已停稳灯灭由本轮就位答复补记。仅本地实测/记录，未提交推送。核R RUN-01/PWR-01；E LOC-03/05、MOT-02/04、RUN-06；Q不变。

## 2026-10-26 停车遥测修复版已烧录，210cm单次对照完成

用户新确认上一轮停稳灯灭/回起点后，下载tmp/telemetry_priority_default_20261026同一已完整构建AXF，SHA=ecca1d075c43e0d7bea23a17e349f347cef775f6f3d166224f35a2df7509e127；Keil Verify OK/Application running。只读待机核健康后明确PLAN6/forward210一次，未编译/改参数。当前板上已是遥测优先级修复版，S曲线仍关闭、最高15RPM、距离补偿与原保护不变，不能继续说板上是旧冻结01。

本次run-once=software_complete，完成1/1、phase3、stop_mask/post_settle15、四轮0、VM请求/输出0，所有CAN/RTOS/safety/distance故障0、uart_rx_errors0；主机未发送STOP，未出现前次收尾telemetry_timeout。收尾窗口26条TM，设备相邻最大110ms、电脑记录相邻最大约125.953ms，同时收到最近16个不同实机BT1批次。累计65成功/0中断、缓存覆盖49，整批最大67ms、单轮接口最大1ms、初始最大排队394ms；不能据此认定机械到速或四轮同步。运行末态CAN栈155words、RX244words、最低堆9792B，均为观察值未签所有负载边界。

用户实测前移210cm，横偏先答3.25cm、随后明确向右，故记录右偏3.25cm；不是横偏清零或高精度定位验收。现场本次停稳/灯灭未单独明确回答，遥测四轮0/VM0与现场观察分别记录。原A1约30.1cm、A2约210cm/右偏约4cm为前后对照；本次实际右偏略小不能凭两次估读签趋势。

编码到距触发2100.265mm，PE2名义投影2056.416mm，定义不同且不代替尺量。有效441/无效1、seq gaps103、末态telemetry_dropped4，含启动旧缓冲/尝试丢弃等因素尚未逐条定位，不能声称无丢帧。收尾单轮对照不再超时，不能扩为所有工况通信可靠性完成。

证据evidence/p1-foundation/2026-10-26/telemetry_fix_forward210_20261026_212828/（download.log、idle/run原始串口与summary、review.json、measurement.csv）及telemetry_priority_forward210_physical_review.json；软件审查附后续实测指针。旧冻结实测包源码/AXF不修改，本轮不再发车。下一步可在新现场准备确认后使用已构建candidate修复版做B1前进30cm，观察起停/低速停走、收BT1和量尺；B1/B2尚未运行，不自动进入，不提速或放宽保护。既有本地成果保留，未提交推送。核R RUN-01/PWR-01；E LOC-03/05、MOT-02/04、RUN-06；Q不变。

## 2026-10-26 固定低速实测包整理完成，当前收口等待集中实测

用户确认暂缓独立CAN回帧分类/通信重构，先固定已有版本、顺序和记录表，不继续增加功能或切0.1RPM。当前实测包位于工创赛/releases/p1_low_speed_validation_2026-10-26：01_distance_cal为前进补偿＋原分档运动、S曲线关闭；02_scurve为相同补偿＋低速S曲线候选。两者均保留最高15RPM/1RPM输入配置及原保护，20cm/s仍是后续分级验证目标。两份完整源码/Keil工程/既有AXF/原构建证据与同一主机脚本已冻结，本轮没有重新编译。

已逐字节核对AXF与原完整构建一致：01的SHA256=6cab5182f2f23316ca5b1f527cc10b094ae3c6d709571f07e0081ba2dc17bf9f；02=1ff88ed1adf554dc80c1b7b2f8508fc647c13848cd6baffd1ded37f4f7685cb7。冻结manifest覆盖2096个文件，verify_package.py实际运行全部通过，工程登记源文件均存在；该工具仅文件校验，不访问串口/Keil/硬件，校验成功不等于实车通过。原主工程固件源码SHA仍匹配scurve_exec_review，未改代码或驱动参数。证据2026-10-26/low_speed_package_review.json及包内manifest/software_review/build_evidence。

四项均待测试：A1校准30cm→A2校准210cm，收齐软件及尺量结果再决定进入B1候选30cm→B2候选210cm。每次明确forward30/PLAN2或forward210/PLAN6、单次运行和新现场就位确认；不调用旧默认组合，不复活已退役237cm/PLAN5，不自动连续发车、清故障或重试。measurement_template.csv需复制至measurements再填；软件完成、编码终点、实际尺量、现场停稳灯灭及起停顿挫分开记录，未量不填0。精度验收阈值尚未冻结，不能用software_complete签地面精度。之前214.75/214.75/214.25cm为无补偿历史，不填入新测试。

两版本均尚未实车验证；01是本轮对比版本，不冒充已经验证的硬件回退基线。原无补偿210cm构建仍保留tmp/forward210_debug_20261025。包内无自动烧录/发车脚本，现场条件就绪后复用同一AXF复位测试，无须每次改路线重编。当前尚未创建/接入zdt_rx_router，持续接收、同步发送、主动反馈待BT1实测证据后再决定。底盘阶段仍缺实际距离/平顺/动态故障/可靠XY闭环，不签阶段完成。

本轮fetch核54bb617无分叉、保留全部本地修改；未开COM、未烧录、未发车，板上最后下载记录仍旧无补偿210cm固件。仅本地实测包/交接完成，未提交推送。核R RUN-01/PWR-01；E LOC-03/05、MOT-02/04、RUN-06；Q不变。

## 2026-10-26 低速距离S曲线接入候选完成，默认关闭、未烧录

按用户授权接入单段距离执行，固定当前1RPM配置。新增APP_DISTANCE_SCURVE_ENABLE=0，仅调试mode3可选；原默认运行和比赛禁止入口保持。准备阶段在临界区外生成计划、原30/60秒期限内不可行则拒绝；提交重查会话/参数、初批强制零速，候选main准备对象改静态以减栈。运行按实际累计时间取参考、整批边界接受新基速，候选START半批中也检查距离/失效；新低速生成函数支持0～15RPM，修正限到±min(基速,2)，0时不原地纠偏。旧Hold仍3～15RPM，旧驱动发送/反馈单位及当前15RPM最高设定未提速，未实施0.1RPM/同步/主动上报。

候选编码到距才触发原停止，不凭曲线结束报成功；参考结束后编码距离达目标减2mm容差可请求停止，否则零速有限等2000ms，不能无限补走。参考跟随误差50mm、加速度40mm/s²、jerk80mm/s³均是待实测候选。新增distance reason7跟随误差、8参考异常、9结束未到超时；保留原位置400ms、源/提交100ms、停止500ms等保护及故障锁存。到距不等DONE，仍需四轮停止确认、新鲜零速、原稳定等待及端点采样；编码估计不代替地面XY。前端纠偏请求与低速有效修正可不同，BT1保留实际应用值，TM1布局不变并正确显示0～2基速。

fetch核54bb617无分叉，保留既有本地成果。默认调试/开启候选调试/比赛完整Rebuild均0错误0警告，Code62292/70144/53716；源开关仍0。默认实际C后台286断言、候选13783断言（含循环采样）、主机96项测试通过，覆盖短段三方向、整数命令模拟延迟、到距与未到超时、真实后台停止确认/零速条件、源/位置失效、取消/500ms停止优先、整批冻结、回绕及准备参数拒绝。仿真不证明实际时序/跟随，航向修正和整数发送下实际轮速加速度/jerk未验收，候选临界区开销/CAN栈及低速可能停走待实测。

证据2026-10-26/scurve_exec_review.json、三份build.log、default/candidate_arm_checks.json、host_tests.log、task_only.patch，说明docs/P1_DISTANCE_SCURVE_EXECUTION_2026-10-26.md。新旧文件GBK/CRLF保持，主机生产控制脚本未改，仅补两项兼容测试。本轮未开COM/烧录/发车，板上仍旧无补偿210cm固件；仅本地完成未提交推送。后续能实测后先前进比例30/210，再单次低速候选收BT1与量尺/停车观察，参数验证后分级到首版20cm/s。核R RUN-01/PWR-01；E LOC-03/05、MOT-02/04、RUN-06；Q不变。

## 2026-10-26 ZDT原手册优化复核：先厘清单位与通信，再接S曲线执行

按用户要求复核110页原手册V1.0.5，目视第49/50/53/66/81/89页表格，并对照当前CAN、后台、运动学及四台V2.0.2旧实物记录。确认优化候选：Emm可0.1RPM输入（35速度反馈仍整数RPM）；逐台缓存+FF66或0xAA多电机命令可改善四轮执行偏差（AA在CAN上分包，仅ID1控制确认，不能当四轮ack）；11定时反馈可减少查询等待，但当前清FIFO/单应答轮询不兼容，须先持续接收与分发；Emm acc为非线性档位不是RPM/s、更不是jerk受限S曲线，0.1缩放后内部步进时间待确认。驱动心跳和43综合反馈可另评估，不关闭原保护。

当前S_Vel_IS仍按四台Disable旧现场记录，今天未读取或修改。20cm/s目标不变，S曲线纯模块及所有现有固件源码保持。建议先准备单位/量化与持续接收/多帧候选，补异步帧混入、缺包、突发、旧session、STOP优先等软件检查，再选同步方式受控验证；不是今天已授权改驱动/发车，也不宣称新功能已实物通过。原手册第37介绍及新章节、回包/配置表存在版本细节，明确保留厂家待确认，不拿第三方库版本套本机。

详见docs/P1_ZDT_OPTIMIZATION_REVIEW_2026-10-26.md，证据2026-10-26/zdt_manual_optimization_review.json，原PDF SHA0afae3f30e1a679c0cd425c790b495dee504c352d2afe234cfa474868319ee92。全书提取/渲染在tmp/zdt_audit_20261026，未编译/烧录、未开COM/发CAN、未改菜单/电机固件。fetch核54bb617一致、原本地修改保留，仅本地记录未提交推送。核R RUN-01/PWR-01；E LOC-03/05、RUN-06、MOT-02/04；官方Q不变，器件事项另列厂家确认。

## 2026-10-26 S曲线纯计算模块完成；首版比赛巡航目标改为20cm/s

用户本轮确认第一版完成比赛巡航目标20cm/s，52.5cm/s保留为后续优化候选，不是当前必达或已开放配置。当前实车调试预设仍最高15RPM（77mm名义轮径约6.05cm/s），到距前8/3RPM约3.23/1.21cm/s；20cm/s理论约49.61RPM，不能直接套低速距离比例或把本轮数学计算当作实车能力。

新增USER/motion_profile.c/.h并登记uvprojx，只做无CAN/RTOS/VM副作用的标量静止到静止七段S曲线：限速度、对称加减速度与加加速度，短段自动去掉匀速/匀加速段，按累计实际时间采样位置/速度/加速度/jerk；零长度结束、非有限或越数值边界输入拒绝。规划结束只表示参考结束，不等于实车到位/停稳，不触发VM。以现有Chassis_Inverse对固定航向直线方向计算四轮约束，为纠偏预留RPM/RPM/s/RPM/s²，再统一时间缩放：v乘λ、a乘λ²、jerk乘λ³；不逐轮削顶或改轮径。

本轮只实现静止到静止单段、固定航向平移。尚无非零端点速度、运动中重规划、途经点融合、旋转/平移联动、整数RPM发送、反馈跟踪、通信延迟停车余量或驱动内插接管；这些不能宣称由单段S曲线自动解决。数学边界、用例加速度/jerk/轮限及纠偏余量均不是已标定实车参数。当前main、发送、距离、批次诊断等既有源码SHA与上一轮一致，未接入实际发车，因此低速与保护不变，新函数暂无运行调用。

fetch核main/origin均54bb617无分叉、保留全部本地成果。隔离调试/比赛入口完整Rebuild均0错误0警告且日志明确编译motion_profile.c；未引用模块由链接器移除，最终大小与上一轮一致Code62244/53672，不据此声称模块已在车上运行。ARMCC实际C模块＋原运动学仿真46177次断言（含循环采样，不是46177种独立场景）通过：长短/零段、加速与巡航退化边界、阶段位置/速度/加速度连续性、速度/加速度/jerk界限、八方向四轮约束、独立限速/限加速度/限jerk缩放、异常与超时拒绝。

证据2026-10-26/scurve_review.json、scurve_arm_checks.json、scurve_debug_build.log、scurve_competition_build.log、scurve_task_only.patch；说明docs/P1_SCURVE_PLANNER_2026-10-26.md，测试tmp/scurve_unit_20261026。新C/H为GBK/CRLF，工程XML保留UTF8/LF。本轮未开COM、未烧录、未发车，板仍此前无补偿210cm固件；仅本地完成未提交推送。后续先设计受控接入/量化与跟随异常处理，能实测时先校准30/210cm，再按低速逐级走向20cm/s，收BT1评估实际更新节奏；动作衔接另行做连续途经点，保护不为平顺延迟。核R RUN-01/PWR-01；E MOT-01～04、RUN-06、LOC-03；官方Q不变。

## 2026-10-26 控制更新批次诊断代码与软件验证完成，未上板

按用户确认先补诊断。调试入口在既有发送路径中记录目标生成边界、第一轮接口调用前、各轮发送/返回及整批结束毫秒；记录session、全局batch_id、冻结纠偏/基础轮速、pending变化数、发送/确认掩码、失败地址与返回码。1仅表示四轮通信确认；2故障中断；3停止/撤VM中断。dry_run=1为模拟成功，不能认作实机应答。时刻使用现有RTOS tick（本工程1ms），是应用调用边界，含接口等待及调度，不是CAN总线边沿或机械到速时刻；生成来源0初始选择、1最近匹配邮箱发布、2直接API、3距离减速，不能把初始化准备等待误认为总线拥塞。

固定保留最近16批（重启清空），覆盖数及累计最大排队/整批/单轮接口耗时、成功/中断数另存；运行中只写有界内存，不格式化/输出。VM请求0、实际VM输出0且后端inactive时，新增BT1帧每次至多一条、间隔至少100ms导出，空闲后循环重发便于稍后只读采集；组合段间保电期间不输出。主机解析带CRC和范围/时间关系检查，只存诊断，不喂保活/ARM/START；结束后无控制命令的收尾采集延至2.5秒。TM1、CT1/CT2及PE字段数量/节奏不改；原查询、发送顺序、控制目标和所有保护门限不改。上一轮前进比例仍保留待实测。

fetch成功，main与origin/main均54bb617无分叉，既有未提交修改保留。隔离调试/比赛入口完整Rebuild均0错0警告：Code62244/53672，调试RAM静态占用较距离系数版增加约2760B，比赛入口不分配诊断缓存/不导出BT1。主机94项测试通过；实际C后台286项断言通过，覆盖完整四轮发送、7次原有交易顺序数量不变、失败0返回、发送中故障后晚回复不归新会话、撤VM中断、时间回绕、历史覆盖、pending和旧session、原距离/会话/时限保护。桩和仿真不证明实时调度/电机响应，新增记录的实机开销、栈余量与串口输出仍待验证。

证据：2026-10-26/batch_diag_review.json、batch_diag_debug_build.log、batch_diag_competition_build.log、batch_diag_backend_arm_checks.json、batch_diag_host_tests.log、batch_diag_task_only.patch；实现说明docs/P1_CONTROL_BATCH_DIAGNOSTIC_2026-10-26.md。源码GBK/CRLF保留。本轮未开串口、未烧录、未发车，板上仍是先前无补偿210cm版本；仅本地完成未提交推送。下一步可继续离线开发；能实测后先验证距离校准30/210cm，再在同一次受控运动结束后收BT1看实际延迟，不据软件结果启用提速或航向候选。核R RUN-01/PWR-01；E LOC-03/05、RUN-06、MOT-02/04；Q不变。

## 2026-10-26 独立前进距离系数已接入并构建，尚未烧录

按用户“ok写代码吧”授权，在当前三次210cm实测214.75/214.75/214.25cm基础上增加独立CHASSIS_FORWARD_DISTANCE_SCALE=1.02182540（实测均值/210），距离任务校准进度=原始编码投影×系数；等效目标乘数0.97864078。用户输入及限值/预设仍为30cm或210cm，不改成2055mm去绕过精确长距配置门禁。该值是低速经验初值，尚未单独扣除编码终点约1.1mm停车余量，须短长实测确认，不保证提速后精度。

本轮仅修改USER/chassis_kinematics.c/.h及chassis_auto_test.c：新增纯计算函数并在后端距离检查中使用，前进的剩余距离、减速、到距和TM1进度统一采用校准mm；原始位置投影、PE端点编码、轮径、轮速、左右横移及spread50mm仍保留原定义。TM1目标仍用户目标，主机PE推算translation_travel_mm仍为名义编码投影，二者不再直接等同。非法模式/NaN/无穷/溢出拒绝；源时效、位置400ms、反向/回退、超程、停止、VM和看门狗原保护路径保留；比赛入口仍未开放自主运动。原GBK/CRLF保留，无批量转码。

fetch成功核HEAD与origin/main均54bb617、无分叉，保留既有未提交成果。隔离完整重建仅调试/比赛两个入口，各0错误0警告，Code60692/53632；软件运行实际C代码241项断言通过，含同一编码行程前进到距但横移未到距、三方向反向/超程/四轮差异故障、位置过期停车、非法输入和原后台会话/时限回归。ARM测试RTOS/CAN为桩，不能代替实车与并发时序验收。证据2026-10-26/forward_cal_review.json、两个build.log、forward_cal_backend_arm_checks.json、forward_cal_task_only.patch；隔离固件tmp/forward_cal_debug_20261026与forward_cal_competition_20261026。

本轮未连接串口、未烧录、未发车，板上仍是此前无补偿210cm版本。下一步先就位确认，再下载校准版，分别尺量30cm和210cm；不自动沿用昨日就位确认发车，未验证前不提速。核R RUN-01/PWR-01；E LOC-03/05/07、MOT-02/04、RUN-06，官方Q不变。全部仅本地完成，未提交推送。

## 2026-10-26 两次210cm复测收齐，前进距离比例待校准

用户回报第2次复测214.25cm。原测和两次追加实测依次214.75、214.75、214.25cm，平均214.5833cm，均值超目标4.5833cm（约2.183%），三次极差0.50cm。当前低速同条件数据支持先处理前进比例偏差；不据此证明更高速或其他地面同样准确。候选目标距离乘数210/均值=0.97864078，仅为经验初值，尚未写入固件；实现时须明确距离量单位和系数方向，区分编码终点/停车余量与真实地面距离，不改左右横移或公共轮径。

第2轮沿用既有forward210固件，于2026-10-25 21:20:55下载校验成功后单次运行，software_complete、completed1/count1、elapsed40993ms，四轮0/stopmask15、VM请求输出0、backend result1/cause0、系统及距离fault0、uart_rx_errors0。编码起终估计2101.06036mm，不代替尺量。本次用户仅回报距离，未另行确认第2轮现场灯灭、左右偏移或转头，不补写现场结论；第1轮停稳灯灭已在第2轮就位回复确认。

证据：同日forward210_cal_repeat2_run原始串口/summary、forward210_cal_repeat2_review.json及forward210_calibration_measurements_review.json。本次仅保存实测和汇总，未改固件、未重编译/再次烧录、未另发车；fetch成功且HEAD与origin/main无分叉，既有本地修改全部保留，未提交推送。下一步加入独立前进距离校准，再单独确认现场条件后验短距和210cm；提速及航向方案仍后续处理，保留全部停止/故障保护。

## 2026-10-25 第1次复测量得214.75cm，与原测一致，等待第2轮就位

用户仅回复第1轮实际前移214.75cm，与原210cm目标实测214.75cm一致，均+4.75cm（约2.26%）。本次未独立反馈左右偏移/明显转头/停车灯灭，不沿用原右偏4cm；保持遥测四轮0/VM0证据。forward210_cal_repeat1_review.json已补地面数值，暂不改校准系数。

已询问确认停车灯灭、回同起点朝向及相同载荷/地面与路径空间后，再执行计划第2/2轮；当前尚未发车。本轮仅记录实测，未改代码、构建、烧录或开串口；本地未提交推送。

## 2026-10-25 21:18 距离比例第1/2次210cm复测软件完成，等待量尺

用户确认第1轮同载荷/地面/起点朝向、标记、空间及VM允许后，21:16:55重新下载同一forward210_debug固件Verify OK，先只读健康，再单次--plan forward210/PLAN6，未改源/重编/加补偿。software_complete、completed1/count1，最终四轮0/stopmask15、VM请求输出0、backend result1/cause0、系统及距离fault0、remote_result2、rejected0、uart_rx_errors0、telemetry_dropped0。编码起终估算2101.21539mm，仅为编码估计，不代替地面尺量。

已询问第1轮现场停稳/灯灭、实际前移cm、左右偏移与明显转头，先不挪车，等待测量；尚未开始第2轮。证据同日forward210_cal_repeat1_download.log、prestart/run及review.json。当前不再发车，先收本轮地面数值，再回起点单独确认第2轮，不依据原一次214.75cm直接改系数。所有保护保持，仅本地记录未提交推送。

## 2026-10-25 当前优先前进距离比例复测，换速批次诊断暂缓

用户确定第一版目标52.5cm/s（当前名义77mm轮径理论约130.22RPM），但认同先校准前进距离比例，再分级提速。当前仍用15RPM/约6.05cm/s、无补偿的forward210调试版，绝不因速度讨论直接改上限。优先同载荷/地面/起点朝向再量两次210cm，与此前214.75cm（右偏4cm）对比稳定性，再判断独立前进校准系数；左右不重复量，视觉精调和快速到物料区策略后续再定，航向候选/批次诊断暂缓，不默认为当前必做前置。

本轮fetch核54bb617一致，当前固件及主机SHA与forward210_review相同，复用已验证AXF不改代码/重编。用户已确认第1/2轮回标记起点/朝向、210cm终点、同条件和空间/停车余量、主控下载器连接及VM允许/可随时关闭，可以复位并单次运行。复测期间保留原普通/长距截止、故障、停止、VM和看门狗保护。完成后量实际前移/左右偏移及停车灯灭，再回起点确认第2轮，不自动连续发车。

记录仅本地未提交推送，尚未校准系数或提速。下方“先补批次诊断”为上一个阶段安排，优先级已按用户本轮决定调整。

## 2026-10-25 航向纠偏方案与离线验证完成，生产控制器未改，先补批次诊断

用户同意先做代码方案与软件验证。本轮fetch核54bb617一致，重读交接/规则及实际驱动/运动学/控制器：当前raw1=1RPM、编码拒绝非整数。0.6度低角速度时PD期望0.48RPM，原0档须>0.65才进1档，存在小误差残留。不能只把相对量化阈值降到0.5以下，否则固定0.48会反复跳0/1。

隔离候选A使用零档独立0.35进入/0.15退出、负向对称，1/2档仍1.65/1.35；B再加200ms最短正常换档间隔。原PD、0.3死区、80ms滤波、10RPM/s斜率、±2及异常拒绝保持。实际C原函数＋候选通过3589次断言（含重复采样，不是3589种用例）；18组一阶yaw假设×3控制器=54模型，B尾段平均角误差低于原版，但高增益/300ms延迟一例换档12→221次/60秒。模型不含真实CAN/四轮执行/载荷/侧滑，不能据此说实车更准或可直接接管；200ms未定为实车参数。

方案见docs/P1_HEADING_QUANTIZATION_DESIGN_2026-10-25.md，证据同日heading_quantization_offline.json、heading_quantization_design_review.json，实验源码tmp/heading_quantization_offline_20261025。原生产heading_controller.c/.h与隔离副本字节一致，forward210_review内生产源SHA仍相同；未改实际固件或驱动、未构建/烧录生产版、未开COM或发车。板仍210cm最后完成态；未校准轮径/比例，不加入提前停车补偿。

下一步先补生成/pending、批次开始、四轮应答完成的只观察计时，不能以100ms TM1推定准确批次时长；再设计与后端ready/会话核验对齐的换档门禁，等待期间用旧值正常刷新100ms邮箱，停止/失效/到距必须绕过正常节奏。先模拟旧session/ready抖动/超时/停止优先，再考虑单次实测。不直接启用A/B，不因模型平均误差小就放宽保护。距离误差和横向X/Y仍独立处理。仅本地记录未提交推送。核R RUN-01/PWR-01，E LOC-03/05/07、RUN-06、MOT-02/04，Q不变。

## 2026-10-25 210cm地面实测214.75cm、右偏4cm；航向闭环不等于横向位置闭环

用户回报同一参考点实际前移214.75cm、小车整体向右偏4cm，并问有HWT闭环为何右偏。目标210cm误差+4.75cm（约2.26%）；编码估算210.10401cm，与地面实际差约4.646cm，不能把编码到距当真实X/Y闭环。此前30cm实际30.5cm（+0.5cm），长距离误差明显增大，不支持统一提前5mm停车；尚未据此改轮径或比例。

本轮HWT起点94.833度、终点94.185度，残差约-0.648度，采集峰偏0.73度；有轮速384条TM1中纠偏0RPM为353条、1RPM为31条。当前HoldCorrection为0.8*角误差-0.25*角速度，后经滤波/斜率及整数RPM0.65/0.35迟滞量化；例如0.6度低角速度期望仅0.48RPM，在零档时不足越过0.65门槛，可留下小角度残差。该机制与日志支持“纠偏较粗”，不将它证明为4cm右偏唯一原因。

HWT测航向，不测横向位置；当前只有航向及轮编码距离控制，麦轮侧滑/轮子接触差异、初始车头与标线/HWT安装轴未对齐等仍需区分，车头不转也可能横移。按记录峰偏0.73度估算，2.1475m若全程同偏角约2.7cm侧移，不能仅据此完整解释4cm，需进一步核对初始对线、横滑及测量。各轮估算2087.53/2114.92/2087.01/2114.70mm是带纠偏后的轮行程，不直接据左右差判某电机坏或单轮标定错。

证据更新forward210_test_review.json（实际尺寸、航向及纠偏统计）及本轮原始记录。用户量尺回复未独立明确本轮停车灯灭/实际转头，保持遥测停车VM0证据，不补写现场确认。下一步按交接把距离比例/停车余量、航向残差、横向位置分开核对，先查对线和航向量化，再接有效X/Y或视觉横向纠偏；本轮仅诊断记录，不改参数、不再发车，不放宽保护，不上传GitHub。比赛入口仍禁止运动。

## 2026-10-25 20:20 纠正后的连续210cm软件完成，等待量尺结果

用户确认上次已停稳灯灭、回标记起点/朝向、210cm终点与空间余量后，20:19:24下载forward210_debug Verify OK/Application running，先只读健康，再明确--plan forward210/PLAN6，ARM/START各1。连续单段software_complete、completed1/count1、id6/direction0/kind4，目标遥测2100000 milli_mm准确，elapsed40949ms。最终四轮0/stopmask15、VM请求输出0、backend result1/cause0、系统及距离fault0、remote_result2、rejected0、uart_rx_errors0。未加入提前停车或轮径/比例补偿。

434TM1/218PM1/1PE2/2CT2（1唯一）/0HF2/1无效，180命令为PLAN6/ARM/START各1及KEEP177；telemetry_dropped1保留，不声称全程零丢帧。422运行/完成TM1姿态源全有效，最大角/速源龄12/13ms。CAN/RX水位163/248 words=652/992字节；前端临界区最大33.09524us、提交10.52381us。编码估算起终投影2101.04006mm，末到距进度2100.147mm，不替代地面实际距离。

已询问本次现场完全停下/灯灭、同一参考点实际前移cm、左右偏移和转头，待用户量尺回复，当前不再发车。30cm→实测30.5cm为唯一已收到地面数值；237cm为用户口误目标，记录保留，不用于210cm精度验收；其停车灯灭已由210cm就位答复确认。后续只依据210cm实测再判断固定停车余量/比例，不默认统一提前5mm。

证据同日forward210_test_download.log、prestart/run及forward210_test_review.json，构建与模拟见forward210_review.json。板为PLAN6/210cm专用调试版完成态，历史PLAN5/237cm已禁用；比赛入口仍拒绝运动。仅本地代码/协议/交接，未提交推送。核R RUN-01/PWR-01，E RUN-06、LOC-03/05/07、MOT-02/04，Q不变。

## 2026-10-25 用户纠正长距目标为210cm，独立PLAN6准备完成，等待就位

237cm软件完成后用户明确说目标口误，要求前进210cm重新测试。237cm原始/软件记录完整保留但不用于210cm精度验收；本轮未获得237cm地面量尺值。新预设forward210/PLAN6，禁用历史PLAN5/forward237，旧编号不静默映射成新距离。仅调试mode3前进15RPM、2100mm精确组合享有长预设60秒运行/65秒阶段/75秒后台总截止；普通10..100cm/30秒、原120秒组、姿态邮箱/位置/停止/VM/看门狗均保持，不加入提前5mm补偿。

debug/competition构建均0错0警告，Code/RO/RW/ZI为60432/868/2028/34340及53372/652/1944/29376；主机89项、实际C模拟后端162/入口131/比赛44通过，覆盖210cm准入、237cm及旧PLAN5拒绝、原超时与预算，模拟桩不当MCU证据。源与隔离构建核同、GBK/CRLF保持。证据同日forward210_review.json、build.log及ARM_checks.json。此刻尚未烧录/发车，已询问确认上一轮停车灯灭、回起点/朝向、210cm标记和路径余量后单次测试，不自动沿用237cm的现场许可。仅本地未提交推送。

## 2026-10-25 20:12 连续237cm软件完成，量尺结果及现场停车确认待回复

用户明确指定237cm并确认回标记起点、朝向、终点/直线及停车余量，允许烧录单次。20:11:32下载forward237_debug Verify OK/Application running；只读健康后--plan forward237，PLAN5确认后ARM/START各1次。连续单段software_complete、completed1/count1、id5/direction0/kind4，elapsed45470ms。最终四轮0/stopmask15、VM请求输出0、backend result1/cause0、系统及距离fault0、remote_result2、rejected0、uart_rx_errors0、telemetry_dropped0。

483TM1/241PM1/1PE2/2CT2（1唯一）/0HF2/0无效，199命令为PLAN5/ARM/START各1及KEEP196。468运行/完成TM1源全有效，最大角/速源龄6/7ms；CAN/RX最低163/248 words=652/992字节，前端临界区最大32.74405us、后端提交10.54167us。编码起终投影2371.22791mm，末到距进度2370.230mm，均不是地面实际位移。软件完成不宣称定位/物理精度已验收。

用户此前30cm量尺前移30.5cm已记入forward30_measure_review，未独立给左右偏移/朝向和停车灯灭，随后237cm就位回复确认上次停车。当前237cm现场完全停下/VM灯灭、同一参考点实际前移cm及左右偏移/转头仍待量测回复；已询问先不挪车，当前不再发车。未做提前5mm停车或比例校准。证据同日forward237_test_download.log、prestart/run及forward237_test_review.json；代码构建/模拟证据见forward237_review.json。

板为237cm专用调试版正常软件完成态，串口关闭；比赛入口依旧拒绝运动。下一步收到地面237cm实测后，对比固定停车误差与比例误差，再决定是否需要校准；不把一次测量扩大为统一5mm补偿。代码及协议/交接仅本地未提交推送。核R RUN-01/PWR-01，E RUN-06、LOC-03/05/07、MOT-02/04，Q不变。

## 2026-10-25 237cm连续前进专用配置本地完成，现场已允许单次测试，尚待烧录

用户报告前进30cm实测30.5cm（多5mm），继而明确改做连续237cm。未以单次手量直接校准轮径/提前5mm停车；左右不重复量。新增PLAN5/--plan forward237，仅调试mode3允许前进0、15RPM、2370mm这一组合，普通10..100cm范围不变，侧移/低速或其他超100cm请求仍拒绝，比赛入口不放行。新motion_distance_limits.h统一前后端校验及运行期限；指定长预设前端运行60秒、阶段65秒、后端run+1秒/总75秒均有截止，原短预设30秒/总45秒及组120秒不变，姿态/邮箱100ms、位置400ms、停止、50mm越界、反馈限速及VM/看门狗保持。距离序列化独立支持2370mm，角度原范围不变。

debug/competition构建均0错0警告，Code/RO/RW/ZI为60428/868/2028/34340及53372/652/1944/29376；主机88项通过（握手覆盖新增PLAN5），实际C模拟后端161、入口128、比赛42通过，含长距准入/错误方向速度距离拒绝、45秒不误断、运行超时故障/总75秒截止、2370mm数值转换及120秒组预算仍拒绝双长段。模拟桩不作MCU/物理证据。源与两隔离构建相同，GBK/CRLF保持。证据同日forward237_review.json、两build.log及三ARM_checks.json。

用户已确认上次停车灯灭、回标记起点/朝向、237cm终点与足够空间、主控及下载器连接、VM允许并可随时关闭，允许烧录单次连续前进237cm。此记录时尚未烧录/发车；下一项下载forward237_debug隔离版，先只读健康，再明确--plan forward237一次，完成后量同一参考点实际行程/偏移，比较固定停车余量与比例误差，不默认误差必定5mm。比赛调度仍禁止，保护不因测试绕过。仅本地未提交推送。

## 2026-10-25 19:56 前进30cm量尺轮软件完成，等待现场距离与偏移

用户认为左右较准，明确只量前进；已确认标记同一参考点/朝向、尺子及现场就位。此前fetch核54bb617一致、源SHA未变，复用同一entry_split_debug AXF，19:55:53重新下载Verify OK并只读核新会话健康后，单次--plan forward30。completed1/count1、id2、direction0、kind4，最终四轮0/stopmask15、VM请求输出0、backend result1/cause0、系统及距离fault0、rejected0、uart_rx_errors0；CAN/RX水位163/248 words，telemetry_dropped1不隐藏。

已询问现场完全停车/灯灭、同一参考点实际前移cm、左右偏移及明显转头，待用户量尺回复；编码到距不能代替地面实际距离，暂不调整轮径/比例。左右本轮不重复测，不扩大此前主观“较准”为精度验收。证据同日forward30_measure_download.log、prestart/run、forward30_measure_review.json。当前不再发车，等量测结果后再判断校准或进入HWT辅助相对X/Y。未改代码或重编，交接仅本地未推送。

## 2026-10-25 19:40 三个单段预设软件与现场确认均完成

用户同意left30/forward30/right30依次验证，每轮停车并回起点后才运行下一项。初次下载因USB未识别失败，换线/直连后识别Horco CMSIS-DAP v2与COM17恢复，失败历史保留。复用同一entry_split_debug固件，19:35:20/19:37:36/19:39:25重新下载Verify OK以复位一次性会话，未改源码/路线、未重编。每轮只读健康后分别PLAN1/2/3确认空闲VM0，再ARM1/START1；软件均completed1/count1，方向1左/0前/2右、id1/2/3正确，无第二段。elapsed11129/11117/11151ms。

三轮最终四轮0、stopmask15、VM请求输出0、backend result1/cause0、remote_result2、rejected0、系统及距离fault0、uart_rx_errors0，HF2为0。CAN最低163 words=652字节、RX最低248 words=992字节；CT2每轮前端最大32.22619us、提交9.82143us。三个编码估算投影301.13992/301.09747/301.14730mm，实际地面位移未量，不当厘米精度硬验收。遥测丢弃0/2/0、捕获无效1/0/1保留，不写链路全程零丢帧。各预设仅一轮成功不扩展为全部故障/可靠性证明。

用户已确认左移仅一段、停车灯灭及回起点，前进同样确认；最后右移用户也确认“右移完成，停下且灯灭”，三个预设的现场方向/仅一段/最终停车灯灭均已确认，当前不再发车。证据同日entry_single_left_retry/forward/right_download.log、各prestart/run与review、汇总entry_single_presets_review.json。当前板为新调试入口right30软件完成态，串口已关闭；不恢复取消后两段，不放宽保护。比赛任务/场地自主调度仍未实现且禁止发车，未烧录比赛版。仅本地记录未提交推送。核R RUN-01/PWR-01，E RUN-06、LOC-03/05/07、MOT-02/04，Q不变。

## 2026-10-25 19:36 连接恢复后left30单段软件完成，待现场确认及前进就位

用户再次回复已换线/直连本机后，设备枚举恢复Horco CMSIS-DAP v2及USB串行COM17。复用同一entry_split_debug AXF，19:35:20下载Verify OK/Application running；新会话健康核验后明确--plan left30，PLAN1确认后原ARM/START。software_complete、completed1/count1、id1、action_kind5、distance_direction1，最终四轮0/stopmask15、VM请求输出0、backend result1/cause0、系统及距离fault0、rejected0、uart_rx_errors0。CAN/RX末态水位163/248 words。未改源码或重编，先前无下载器失败记录保留，不写成固件故障。

证据同日entry_single_left_retry_download.log、prestart/run及entry_single_left_review.json，已询问现场仅左移一段/停车灯灭并回起点后前进30cm，回复前不执行下一项。forward30/right30仍未完成本轮硬测；仅本地记录未提交推送。

## 2026-10-25 重连后复查仍无下载器/USB串口，left30继续未启动

用户回复已重新连接并允许检查重试。重新枚举当前设备未发现CMSIS-DAP或COM17；再检查USB/HID/Ports及SERIALCOMM，只有蓝牙COM3/6/12，没有USB串口映射，不能解释为USB串口换了编号。固件SHA仍核同。本轮未再次启动已知会失败的下载进程，未开COM或发车；请用户检查下载器是否直连运行Codex的本机、换可传数据的USB线/转接器及下载器指示灯后再查。证据补入entry_single_left_blocked_review.json。三个单段预设硬测继续待连接恢复，本地记录未提交推送。

## 2026-10-25 单段left30验证因下载器掉线未开始，等待连接恢复

用户同意三个单段预设依次验证，并确认第1项left30已就位。测试前fetch核54bb617一致、源码SHA与entry_split_debug隔离构建相同，未改代码/重编。尝试重新下载同一AXF以复位一次性会话，entry_single_left_download.log仅Load，未出现Erase/Programming/Verify。只读检查本次UV4进程弹窗明确“CMSIS-DAP - Cortex-M Error / No Debug Unit Device found”；系统当前未识别CMSIS-DAP和COM17，不能当成固件故障或烧录成功。本次未开串口、未发送PLAN/ARM/START及运动命令，左移测试尚未开始。

结束仅本次阻塞下载进程（核进程命令行后，不影响原有Keil进程），保存entry_single_left_blocked_review.json。已请用户恢复USB/下载器及主控连接供电，恢复后再检查重试，不自动发车。板上最后已知仍新调试入口route-a完成态；失联期间现场状态未独立核实。left30/forward30/right30三个新预设均未完成本轮硬测，取消后两段不恢复。仅本地交接记录，未提交推送。

## 2026-10-25 15:59 新调试入口PLAN4实测成功，运行时选表后完成原A组三段

用户确认回起点/朝向/空间、主控及下载器上电和VM允许。fetch核54bb617一致，复核entry_split_review源SHA及隔离debug构建相同，复用AXF未改源/重编；15:58:45下载Verify OK/Application running。4秒只读核新会话66字段、debug_plan_id0/默认4项、姿态有效、故障0及VM0，未执行默认表。主机明确--plan route-a，15:59:14.980发送PLAN4；15:59:15.087收到id4、accepted1、count3、remote_state0且VM请求输出0的确认。重新健康观察后15:59:15.420 ARM，15.485 START，各一次，无隐含发车或重新编译换路线。

左/前/右各300mmsoftware_complete、completed3/3、32842ms，两次段间各10条状态2均VM请求输出1。最终四轮0、stopmask15、VM请求输出0、backend result1/cause0、系统/距离fault0、remote_result2、rejected0、uart_rx_errors0。用户确认“已完成，停下且灯灭”。PE2三段估算投影301.02918/301.13253/301.08086mm，实际地面位移未量，不签定位精度。

CAN任务生命周期最低163 words=652字节；UART接收任务最低248 words=992字节（包含新增选表调用），两者均为本轮已走路径实测，不当作全部故障/未来任务最坏栈需求。CT2三段前端累计最大32.22619us、backend提交9.82143us、动作装载37.76786/40.22024/40.22024us。6条CT2去重3段、351TM1/179PM1/3PE2/0HF2/0无效；144命令为PLAN4/ARM/START各1、KEEP141，accepted144/rejected0。telemetry_dropped6，总110序号缺口含开口积压，不全算运行丢包；338运行/完成TM1源有效，最大角/速源龄14/15ms。未复现ORE或换段故障，不扩大成可靠性全闭环。

证据同日entry_split_test_download.log、prestart/run原始/JSONL/summary及entry_split_test_review.json。本轮仅烧录并单次PLAN4三段实测；其他单段预设未另行发车，比赛入口未烧录/硬测，其任务/场地自主调度仍未实现且保持禁止发车。板当前为新调试入口route-a完成态，串口已关闭、不再发车；取消后两段不恢复。已本地更新交接及说明，未提交推送。下一步可在同一固件、重新就位/复位后用--plan left30/forward30/right30核单段预设，无须再改路线编译；或继续接比赛任务方案，但不可称正式比赛已可运行。核R RUN-01/PWR-01/QR-01/DSP-01，E QR-03、RUN-06、LOC-03/05/07、MOT-02/04，Q不变。

## 2026-10-25 调试/比赛入口拆分与运行时调试预设完成，本地未烧录

用户明确同意修改代码。fetch核54bb617一致，保留既有修改。本轮新增USER/app_entry_config.h及app_entry.c/.h并注册实际uvprojx；默认调试，APP_ENTRY编译时选DEBUG/COMPETITION，APP_DEBUG_MODE3为日常共享运动引擎，原APP_TEST_MODE仅内部派生。调试模式3同一固件可经空闲PLAN0..4选择原默认表、左30/前30/右30、左前右30组合；fresh seq/tick、源/自检、无旧会话及VM0门禁通过才复制参数。主机--plan回显id+accepted确认后重新健康观察，再原ARM/START；PLAN本身不发车，不清故障，不允许执行中/末态切表，每次上电一组保持。主机88项通过，TM1组合尾添debug_plan_id共66字段，新主机兼容旧帧，旧主机不能解析新长度。原默认表含转向且未改，当前A测试必须明确--plan route-a；取消后两段不恢复。

比赛入口关闭USART1调试接收及RC1控制分发，PC13沿原扫描/显示/平台门禁和版本确认冻结任务码。任务/场地自主调度尚未实现，入口明确BLOCKED/PLAN_UNIMPLEMENTED，统一AppCompetitionEntry_MotionAllowed拒绝，公共运动接收及CAN入口均核许可，不借用调试表发车；共用运动和全部停止/VM/看门狗保护保留。不是自主比赛功能验收通过。任务码仅基本格式快照，不替代完整业务校验或场地输入。

只构建debug与competition两个入口，最终0错0警告。Code/RO/RW/ZI分别59900/848/2028/34340、53024/652/1944/29376；拆分中不可达/未使用告警已集中到比赛许可函数修正并增量复核，不跑全七配置。实际调试C选择器/命令处理ARM模拟114项，比赛模块38项；为Cortex-M3 SoftVFP/Unicorn及RTOS/后台/电源桩，无MCU并发证明。比赛map无串口发车解析器/调试选表符号。源与两隔离构建核同（比赛仅入口默认值差异）、GBK/CRLF及Keil原LF保持、局部diff检查通过。证据同日entry_split_review.json、debug/competition_build.log及ARM_checks.json，说明docs/P1_APP_ENTRY_SPLIT_2026-10-25.md。

本轮未烧录、开COM或发车，板仍上轮KEEP+CAN栈成功完成态。新版PLAN/66字段及新增Rx栈用量尚未上板，旧CAN652字节不当新版实测；旧0/1/2兼容配置本轮未构建。下一步就位确认后烧录entry_split_debug_20261025，明确--plan route-a单次复测；随后接比赛正式任务/场地状态机，不提速不放宽保护。仅本地完成未提交推送。核R RUN-01/PWR-01/QR-01/DSP-01，E QR-03、RUN-06、LOC-03/05/07、MOT-02/04，Q不变。

## 2026-10-25 15:36 新KEEP处理及CAN栈遥测实测完成，终态单条KEEP确认通过

用户确认就位并允许烧录测试。fetch核54bb617一致，复核completed_keep_review源码SHA、隔离三段表与已构建AXF，未改源/重编。15:34:57下载Verify OK/Application running，4秒只读确认新会话源有效、故障0、VM0且收到can_stack_words后，只一次START执行左/前/右各300mm。software_complete、completed3/3、32830ms，两次段间各10条状态2均VM请求及输出1；最终四轮0、stopmask15、VM请求及输出0、backend result1/cause0、系统及距离fault0。用户确认“已完成，停下且灯灭”。三段轮编码估算投影300.73389/301.00888/301.15099mm，地面实际位移未量。

本轮350TM1/174PM1/3PE2/5CT2（3唯一）/0HF2，144命令（ARM1/START1/KEEP142）全部接受、拒绝0，uart_rx_errors0、telemetry_dropped0、捕获1无效帧；不写链路绝无无效帧。341运行及完成TM1源全有效、最大角/速源龄9/10ms。CT2前端最大32.47619/32.47024/32.48810us、提交9.85119us，未复现ORE或换段故障，一轮不作可靠性全部闭环。

首次取得CAN任务实机生命周期最低剩余163 words=652字节，分配384 words=1536字节，约42.45%剩余；三个动作段记录最低均163。该水位来自CAN任务既有扫描而非UART字段，覆盖本次路径与自启动以来，不能推断所有故障/未来扩展的最坏栈需求，当前不依据本次结果扩大栈。

现场确认停止后，为直接覆盖旧晚到KEEP问题，使用受限终态探针先读取当前成功/停止/VM0/故障0及令牌，仅发送一条同会话KEEP，不发送ARM/START、不续发或重启。接受计数144→145，拒绝仍0、remote_state3/result2、completed3、四轮0/VM0，判定通过；该现场证据直接验证成功终态分支。失败/取消/错令牌/启动边界仍依据此前各57项C模拟，不伪称本轮故障硬测通过。

证据同日completed_keep_test_download.log、prestart/run原始/JSONL/summary、completed_keep_test_review.json，以及completed_keep_terminal_probe.json/.raw；探针tmp/completed_keep_terminal_probe_20261025.py。当前板为新KEEP+CAN栈固件成功完成态，串口已关闭、不再发车，取消后两段不恢复。交接/计划/协议说明本地更新，未提交推送。下一步可整理调试/比赛入口边界方案或逐项扩大测试覆盖，暂不因本次成功放宽保护/直接提速。核R RUN-01/PWR-01，E LOC-03/05/07、RUN-06、MOT-02/04，Q不变。

## 2026-10-25 成功末态KEEP局部修复与CAN栈遥测完成，仅本地验证未烧录

用户同意继续。fetch核54bb617一致，保留全部既有未提交成果。main.c新增成功停车判定，原CRC及令牌核验后，同会话晚到KEEP仅在remote_state3/result2、前端成功完成、后台inactive/result1/cause0/stopmask15、VM请求及输出0且CAN/RTOS/安全fault0时空操作确认（accepted加1），不刷新keepalive、不设置start_permit或动作request。失败/取消终态、错误令牌、ARM/START/STOP及运行中KEEP原行为保留，不允许终态重发车，也不通过清故障放行。

TM1尾部追加can_stack_words，单段模式2为49字段、组合模式3为65字段，旧字段位置和100ms周期保持。值直接复制CAN任务既有g_can_task_stack_words最低剩余words（F407每word4字节），没有新增栈扫描/CAN查询。主机兼容旧48/51/55/56/58/63/64及新49/65，旧主机不识别新长度，需配套使用新脚本。0不能解释为余量充足，UART的rx_stack_words仍单独保留；CAN实值尚未取得。

仅构建受影响route_a及single两个配置，均0错0警告；Code/RO/RW/ZI为59248/768/2020/34180及51268/708/1948/33932。终态检查补齐RTOS/安全fault后只增量重编main并链接，不重跑七配置。83项主机测试通过，含新旧帧及诊断字段不授权发车；从实际main.c提取CRC/整数解析/命令处理/成功判定函数，ARM行为模拟mode3及mode2各57断言通过，覆盖重复晚到KEEP、运行期续期、VM/后台/故障/未完成拒绝、错误令牌/CRC、终态START/ARM与STOP。该模拟使用Cortex-M3 soft-float及RTOS/电源/后台桩，不替代F407现场或并发时序验收。main与隔离构建逐字节相同、GBK/CRLF及局部diff检查通过。

证据同日completed_keep_route_a/single_build.log、completed_keep_mode3/mode2_arm_checks.json、completed_keep_review.json；模拟源tmp/completed_keep_unit_20261025。协议schema及说明已更新。本轮未烧录、开COM或发车，板仍原优化版4轮成功后的最后完成态。下一步就位后烧录completed_keep_route_a_20261025单次原三段，用新脚本检查末态结果保留与can_stack_words，需现场数据后才能签修复/栈余量；不恢复取消后两段。仅本地完成未提交推送。核R RUN-01/PWR-01，E LOC-03/05/07、RUN-06、MOT-02/04，Q不变。

## 2026-10-25 结束KEEP拒绝只读核对完成，CAN栈仅有静态参考

用户同意继续核对。本轮仅查源与既有四轮记录，未修改固件/主机脚本、编译、烧录、开COM或发车。fetch确认HEAD/origin main仍54bb617一致，现有修改全部保留。

结束边界原因：main.c的App_SerialMotionSafety在组合完成后把remote_state设3；KEEP分支只接受state2，否则App_SerialReject(6)增加拒绝计数并覆盖remote_result。主机在首次收到有效完成遥测前仍按200ms发送KEEP，收到software_complete后不再发送。追加2轮最后两条KEEP主机发送时刻15:20:15.544/15.769，首条完成且拒绝2遥测15:20:15.888；追加3轮对应15:22:14.259/14.495，首条完成且拒绝1遥测15:22:14.560，随后拒绝2。两轮完成消息后均无新增TX。证据支持主控先完成、通知抵达电脑前在途KEEP被终态拒绝；没有设备逐条接收时间，不能声称已精确匹配每条拒绝。动作成功后结果字段被拒绝码覆盖，是协议状态表达问题，不是本轮电机/ORE故障证据。

CAN任务栈分配384 words=1536字节，main.c:2987持续更新g_can_task_stack_words（任务生命周期最低剩余words）。当前TM1只输出g_rx_stack_words，属于UART任务，不能替代CAN值。已构建AXF的静态调用报告给App_CanTask最大链792 bytes + Unknown；未计入项及上下文开销存在，不能据1536-792宣布实机余量744字节。当前未取得CAN运行水位，不用零fault或四轮成功替代测量。

证据同日completion_keep_stack_audit.json及既有startup_prepare_repeat2/3_run JSONL、隔离构建Objects/gongxun-f407.htm。下一步可局部让成功终态对同令牌晚到KEEP不覆盖完成结果，仍禁止重新启动/恢复VM，非法或运行中命令保持原门禁；另用调试器读取现有CAN水位变量或增诊断输出，取得实测再决定栈大小。上述修复/新输出本轮尚未实施，不自动发车。仅本地记录未提交推送。核R RUN-01/PWR-01，E LOC-03/05/07、RUN-06、MOT-02/04，Q不变。

## 2026-10-25 15:22 优化版累计4轮12段均完成，追加三轮结束，当前不再发车

用户要求多测几次，追加3轮各自就位确认、同一AXF重新下载及只读健康检查后单次运行。下载分别15:17:22/15:19:26/15:21:24 Verify OK，无代码修改或重编。连同15:13首次优化版实测共4轮，elapsed32885/32808/32820/32852ms，每轮左/前/右各300mm完成3/3；用户逐轮确认现场停车灯灭，最后明确“已完成，停下且灯灭”。共12段、8次段间保电，每个段间各10条状态2均VM请求及输出1；每轮终态四轮0、stop_mask15、VM请求输出0、backend result1/cause0，系统/距离fault0，HF2为0，无ORE故障复现。

四轮CT2下标0/1/2与session1/2/3齐全，hold_critical累计最大均32.1369us、backend提交9.8929us、action_load最大40.1012us。计时是scope覆盖范围的累计最大值，不代表每次调用相同；scope嵌套不能相加，锁外数学耗时不在scope1。相比优化前前端105~107us实测，屏蔽区间缩短约70%，不能据4轮成功宣布ORE唯一根因或整车可靠性全部闭环。CAN任务栈余量仍未独立取得，当前rx_stack_words是UART任务。

四轮各ARM1/START1，无重启发车；命令拒绝分别0/0/2/2，后两轮首次拒绝均见于completed3/3、VM0末态，remote_result6。结合最后TX为KEEP与完成遥测延迟，结束边界在途KEEP被拒绝是待核解释，不隐去计数、不写通信全正常。遥测丢弃1/0/7/7、捕获无效1/0/1/1；不能把总缺口/开口旧积压全部当运动期丢包。四轮运行/完成源均有效，最大源龄5/13/17/9ms。12段轮编码估算投影范围300.93598~301.45551mm，实际地面位移未量，不签定位精度。

证据同日startup_prepare_test/repeat1/repeat2/repeat3的下载、prestart、run原始/JSONL/summary、各review及startup_prepare_four_runs_review.json。当前板为优化版最后第3追加轮正常完成态，VM0，串口已关闭，本轮不再发车。后续建议先只读核结束边界KEEP拒绝和CAN任务栈，再决定是否需要改动；不扩大到取消后两段或提速。根交接/计划已本地更新，未提交推送。核R RUN-01/PWR-01，E LOC-03/05/07、RUN-06、MOT-02/04，Q不变。

## 2026-10-25 15:20 追加第2/3轮完成，末态2次命令拒绝单列，待第3轮

用户已确认第1轮停车灯灭并回起点。15:19:26同一固件下载Verify OK，健康检查后单次三段完成3/3，CT2三段最大hold_critical均32.1369us，HF2为0，最终四轮0/VM请求输出0，backend result1/cause0、uart_rx_errors0。终态remote_rejected2、remote_result6，首次观察在completed3/3/state3/VM0时，不隐去，不写通信零异常；可能为结束边界在途KEEP被终态拒绝，仍需结合命令时间核对，不当作ORE/电机故障。证据同日startup_prepare_repeat2_download.log、prestart/run及review.json。现场停车灯灭待确认，已询问回起点后最后第3轮。无源修改/重编，仅本地记录未提交推送；保护及取消后两段安排不变。

## 2026-10-25 15:18 追加第1/3轮软件完成，待现场确认及第2轮回起点

用户要求多次测试并确认回起点，15:17:22复用同一优化版重新下载Verify OK，仅复位单次会话，不改源/重编。只读健康核验后单次三段software_complete、completed3/3，HF2为0，三个CT2唯一段hold_critical最大均32.1369us。最终四轮0、stopmask15、VM请求/输出0、backend result1/cause0、uart_rx_errors0。证据同日startup_prepare_repeat1_download.log、prestart/run及startup_prepare_repeat1_review.json。当前现场停车灯灭确认待回复；已询问确认并回起点后第2/3轮，不自动发车，不恢复后两段。仅本地记录未提交推送，所有保护保留，实测成功不扩大为ORE可靠性闭环。

## 2026-10-25 15:14 优化版三段实测完成，临界区最大32.14us，待追加三轮

用户确认就位并允许烧录测试。fetch核54bb617一致，复核startup_prepare_review源SHA及隔离三段表，复用已通过构建未重编。15:13:17下载Verify OK/Application running；4秒只读确认新会话源有效、故障0、四轮0/VM0，区分开口旧积压后仅一次START。左/前/右各300mm软件完成3/3、32885ms，用户确认“已完成，停下且灯灭”。PE2三段估算投影301.02549/301.37061/301.20636mm，实际地面位移未量。

收到4条CT2、3唯一段，三段累计hold_critical最大均32.1369us，backend_begin提交9.8929us，action_load累计37.6310/40.1012/40.1012us；相较优化前105~107us的前端最大屏蔽区间明显缩短。scope1现仅提交，锁外准备不计其中，不能把工作量变化当CPU总耗时减少；嵌套scope不可相加。本轮无HF2、无ORE故障/邮箱超时，尚不能证明ORE唯一根因及可靠性全部闭环。

两次段间各10条状态2均VM请求/输出1。最终四轮0、stop_mask15、VM请求/输出0、backend result1/cause0、系统及距离fault0。349TM1/175PM1/3PE2/4CT2，147命令接受、拒绝0，341运行/完成TM1源全有效、最大源龄4/5ms，telemetry_dropped1；总101缺口含开口积压，不全当运行丢帧，捕获有1无效帧不记零丢帧。rx_stack_words309是UART任务，不能用作CAN任务栈余量结论；CAN栈仍待独立核实。

证据同日startup_prepare_test_download.log、prestart/run原始及startup_prepare_test_review.json。本轮无源修改/重编，仅烧录与单次实测，本地记录未提交推送。用户随后要求多测几次，安排追加3轮同路线，每轮回起点确认后单次运行，不自动连续发车，取消后两段仍不恢复。核R RUN-01/PWR-01，E LOC-03/05/07、RUN-06、MOT-02/04，Q不变。

## 2026-10-25 平移启动改为锁外准备、锁内复核提交，本地验证完成，未烧录

根据正常CT2实测启动临界区105~107us，局部调整main.c、chassis_auto_test.c/.h：命令预计算及距离数学自检在外层临界区退出后执行，Prepare仅写调用者局部缓冲，不改CAN/VM/运动状态；重新加锁后刷新时间与姿态源，复核HR_WAIT、trip、后台session、动作参数、CAN/电源就绪，后台再核IDLE/active/cut_vm、VM请求、原健康门禁及端点容量后提交。准备期间取消或故障不得靠旧许可启动。既有阈值、看门狗、停止、VM及旧到距串段修复保留，不通过清故障放行。

仅构建受影响route_a三段、single模式2、manual模式0，均0错0警告。Code/RO/RW/ZI分别59064/768/2020/34172、51096/708/1948/33932、41848/652/1836/27892。初次构建发现const数组形参警告后改为平坦只读指针并重建；隔离路线头核对恢复左/前/右各300mm后仅重建A，主工程默认动作表不变。实际后台C+运动学140项ARM模拟断言通过（Cortex-M3 soft-float/Unicorn，RTOS/CAN桩与顺序交错，不是F407实车或并发时序证明），主机81项通过。三个生产源与隔离构建/模拟源逐字节核对，GBK/CRLF及局部diff检查通过。证据同日startup_prepare_*_build.log、startup_prepare_arm_checks.json、startup_prepare_review.json，说明docs/P1_STARTUP_PREPARE_COMMIT_2026-10-25.md。

本轮未烧录、开COM或发车，板仍最后成功的正常CT2旧版；没有新临界区耗时，不能宣布ORE已修。scope0改为各独立屏蔽区间最大值，scope1仅测预计算后的提交，不再包含锁外数学；scope2动作装载和旧转向路径未优化。新增局部缓冲的任务栈余量仍待MCU检查。下一步就位确认后只烧录startup_prepare_route_a_20261025并单次原三段对照CT2/ORE及栈余量，不恢复已取消后两段。本轮保留既有未提交修改，仅本地完成未提交推送。开发前fetch核54bb617一致。核R RUN-01/PWR-01，E LOC-03/05/07、RUN-06、MOT-02/04，Q不变。

## 2026-10-25 14:39正常CT2实测三段完成，启动临界区105~107us，ORE本次未复现

用户允许继续并确认上电就位/VM允许。fetch核54bb617一致，normal_timing_review中main SHA与当前源及normal_timing_route_a隔离版逐字节一致；复用已通过构建不重编译。14:38:55重新下载Verify OK/Application running，4秒只读核新三段会话健康后提醒，仅一次START。组software_complete/completed3/3，总32873ms，PE2方向1/0/2、距离1、起终mask15，终点投影301.64006/301.34293/301.44259mm。两段间分别9/10条状态2均VM请求/输出1，最终四轮0/stop_mask15、VM请求及输出0、backend result1/cause0、系统及距离fault0。用户明确“已完成，停下且灯灭”，接受实际顺序完成/最终停车灯灭确认；最终实际位置未量，不签定位精度。

成功实收4条CT2，去重后下标0/1/2及session1/2/3齐全，outcome1、timer_enabled1、cycle_hz168000000，首次取得正常状态三段DWT计时。平移前端临界测量最大105.1131/107.1726/107.2857us；其中后端Begin调用84.1429/84.0298/84.1429us，动作装载37.9345/40.3512/40.3512us。三个scope有嵌套不能相加；各段前端最大tick与后端启动tick一致，显示最重区间在启动附近。115200/8N1连续字节约86.8056us，实测区间大于一个字节间隔，支持中断服务延迟风险，不能把本次无ORE的数据当作此前ORE独立唯一因果证明。数值是微秒，约0.105~0.107ms，不是先前命令过期107ms；插桩尾/临界入口出口未计，插桩本身也有开销。

本轮351TM1/176PM1/3PE2/4CT2/0HF2/0无效，148命令发送/接受且拒绝0、telemetry_dropped1。START后341TM1源全有效、帧龄最大5/6ms、序号缺口1与丢弃计数对应；总81缺口含开口旧积压，不全当运行丢包。各段固件峰偏0.422/0.703/0.538度，捕获整体峰速20/21/26/19RPM。无ORE/邮箱超时复现，不扩大成ORE已修或可靠性全过。

证据同日normal_timing_test_download.log、prestart/run原始/JSONL/summary及normal_timing_test_review.json。本轮未再修改源/计时/保护，仅烧录及一次测试；已本地记录未提交推送。当前板正常CT2版A组三段成功完成态；B组后两段继续取消，不恢复。下一项依据实测缩短启动临界区，优先把重复数学自检/命令预计算移到中断允许区，短锁内仍复核源/会话/故障并提交；尚未实施优化，待局部构建与后续现场确认单次对照，不自动发车。核R RUN-01/PWR-01，E LOC-03/05/07、RUN-06、MOT-02/04，Q不变。

## 2026-10-25本地补正常完成CT2计时输出，保留各成功段快照，未烧录未量实值

用户要求继续，仅处理计时与ORE，后两段取消保持。新增最多8段成功平移计时历史：每段原停稳/会话核验通过后复制DWT区间，整组DONE/completed齐全且后端成功/stopmask15/VM请求及命令输出0后才发送CT2。31字段带outcome1、原段下标及backend_session，再接计时快照；原故障CT1/HF2不改。运行中无新增计时串口流量，发送函数快速返回；正常终态按约100ms逐条发送，完成后一秒以上间隔回放，忙时不丢游标。主机兼容CT1/CT2，summary timing_records保留全部唯一段、重复回放去重，不参与令牌/健康/KEEP或重启。

81项主机测试通过，含成功后3段+重复CT2的模拟收包计4帧/3唯一记录，ARM/START各一次、无重启；不是MCU验证。精简为route_a当前三段、single模式2及manual模式0三个受影响编译分支，完整构建0错0警告；A Code58572/RO768/RW2020/ZI34172，single50004/708/1948/33932，manual41632/652/1836/27892。源与隔离构建逐字节一致、GBK/CRLF/diff通过；backend/HWT/计时机制/config/默认表均未改。证据同日normal_timing_*_build.log及normal_timing_review.json，指南docs/P1_NORMAL_TIMING_OUTPUT_2026-10-25.md。

本轮未烧录/开COM/发车、未取得新耗时，正常留存/实收及新增RAM/复制开销仍待MCU验；板仍最后已知旧计时A组三段完成态，尚无正常CT2能力。下一项现场确认后重新烧录normal_timing_route_a_20261025，单次原左/前/右30cm采CT2，核下标0/1/2与timer_enabled再分析启动时长及ORE；不恢复取消B组，不自动重复发车。既有保护及旧到距修复保留，ORE根因未证实不冒充修复。fetch确认54bb617一致，成果仅本地未提交推送。核R RUN-01/PWR-01，E LOC-03/05/07、RUN-06、MOT-02/04，Q不变。

## 2026-10-25用户取消后两段测试，下一项仅补计时证据与ORE排查

用户明确“后两段不用测了”，取消原B组顺时针90度→按新朝向右移30cm的测试安排，不再自动准备、下载或执行B组。已保存的隔离构建和历史记录保留，不把取消写成已通过，也不再把五段回原位作为当前测试目标。此前A组左/前/右各30cm的一轮软件完成结果保留，实际位置误差未量、ORE未闭环。

当前下一项仅补正常结束也可读取CT1的诊断出口，取得真实启动区间耗时，再按证据排查ORE；本次仅更新计划，未修改固件、编译、烧录、开COM或发车。保留全部保护及失败历史，仅本地未提交推送。核R RUN-01、E LOC-03/05/07、RUN-06，Q不变。

## 2026-10-25 14:24计时版左/前/右三段完整完成，两换段VM保持；ORE未复现、无CT1实值

用户要求开始并确认主控/下载器上电、标记起点/朝向/空间及VM允许。fetch确认54bb617一致，源SHA核对计时版review及隔离critical_timing_route_a_20261025通过；复用既有完整构建不重复编译。14:23:34重新下载Verify OK/Application running，4秒只读监听区分旧积压HF2与新健康三段会话后提醒发车，仅一次START。

本轮software_complete、completed3/3，总流程32874ms；三条PE2方向依次1左/0前/2右、距离模式1、起终mask15，投影估算终点均值301.52010/301.08639/300.87600mm，三段均reached1。首/次段间分别9/10条TM1状态2均VM请求1/PE2输出1，证实两次保电换段；最终四轮0/stop_mask15、VM请求/输出0、backend result1/cause0、系统及距离fault0。旧到距串段修复本轮完成两个距离换段回归，但一轮成功不扩展为可靠性全部解决。

350TM1/176PM1/3PE2/0无效/0HF2/0CT1，147命令发送及接受/0拒绝，telemetry_dropped2。START后340TM1源全有效、帧龄最大11/12ms；运行期序号缺口2，与遥测丢弃计数对应，不写零丢帧，总报告81含开口旧积压不能全当运行丢包。各段固件峰偏0.527/0.675/0.549度；捕获整体峰速21/22/22/21RPM。最后yaw相对主机首段参考约-0.725度，末段目标另有更新，主机heading_observation是跨段汇总，不能当每段独立误差。

用户对现场/测量问题仅回“未量”，最终底盘中心前后/左右误差记未量；未独立明确本轮物理动作顺序、灯灭及停车，不能扩大成全部现场确认。仅签三段估算到距、两次保电换段及软件安全结束；不签地面实际定位精度或原五段回起点通过。

ORE及命令过期本次均未复现。当前CT1设计为故障后才输出，故本轮虽运行计时插桩但没有返回CT1，尚未取得任何实测区间耗时，也未外部验证DWT可用，不能据成功运行声称临界区无问题或ORE已修。若后续继续量测，应先补正常终态也可读取计时结果的诊断出口，避免只能等故障；本轮未再改码或重复发车。证据同日critical_timing_repro_download.log、prestart/run原始/JSONL/summary及critical_timing_repro_review.json。

当前板是计时版A组三段成功完成态，B组转向/右移未执行，五段回位未验。源/参数/保护本轮未变，仅本地保存证据与交接，未提交推送。下一项按用户需求选择继续计时证据或B组路线；任何新运动先现场确认，不自动重跑。核R RUN-01/PWR-01，E LOC-03/05/07、RUN-06、MOT-02/04，Q不变。

## 2026-10-25本地增加DWT启动区间计时与CT1，未烧录未量到实值

用户允许继续核对启动临界区与ORE。新增DWT CYCCNT区间计时：平移前端临界区、其内后端启动、组合动作装载；各存次数/最后/最大周期及最大区间起止cycle/tick/arm等，首次HWT错误另记处理cycle。HWT初始化只启用并检查CYCCNT推进，不清计数器、不改时钟/中断配置；布防只清诊断统计，计时不可用时主机不报0us。首trip冻结28字段CT1，故障后约1Hz发送、不参与令牌或KEEP；HF2及其它帧不变。旧到距串段修复、backend/config/默认动作表保持。

77项主机测试通过，含周期微秒换算/回绕、无计时不虚报、格式与CRC、故障收HF+CT不重发控制；7配置完整Rebuild0错0警告，默认Code58200/RO788/RW2012/ZI33172，源逐字节匹配/GBK/CRLF/diff通过。证据同日critical_timing_*_build.log、critical_timing_review.json，指南docs/P1_CRITICAL_TIMING_DIAGNOSTIC_2026-10-25.md。

本轮未烧录/开COM/发车，没有实测耗时或ORE因果结论。区间是插桩之间耗时（含嵌套，不含末端记录及临界入口出口），不能当完整最大中断延迟证明；ISR周期戳不是字节到达时刻，回绕/断点及插桩开销均需解释。下一项现场确认后烧录critical_timing_route_a_20261025、只读健康、单次复现取CT1+HF2，保留现有失效即停及所有保护，不自动重跑。板上仅沿最后已知修复A组失败态，本轮未实读；组合及五段回位仍未通过。仅本地未提交推送。用户询问多次构建，已说明是不同模式；后续小修改优先当前实测配置，涉及共享接口/模式开关再扩验证。核R RUN-01/PWR-01，E LOC-03/05/07、RUN-06、MOT-02/04，Q不变。

## 2026-10-25 HF2复现命令年龄107ms并修复旧到距串段；重烧录复测遇HWT ORE，组合未通过

用户明确要求HF2复现、针对修复并记得重新烧录。首次14:03:08下载Target DLL cancelled失败，DAP/COM17在线但4秒无遥测/0命令；用户恢复主控供电/连接、VM允许和就位后，14:04:39诊断A组下载Verify OK/Application running，新会话健康后一次START。第一左移估算301.06425mm终点成功，第二前进准备再次中止：HF2 backend_fault_reason1/state4、command_age107ms、状态/速度mask15/15、feedback_age106ms、source_valid1/帧龄9/10ms/HWT错误0，明确邮箱过期，不是本次CAN查询失败或HWT失效。

源定位：下一段MoveStart清前端，但后端保留上段distance_mode/reached；App_ForwardHoldSafety在HR_WAIT未经会话核对就把旧reached消费为新段stopping1，之后前端不提交纠偏命令，107ms后保护。仅main.c到距锁存增加HR_RUN及App_ForwardHoldOwns门禁，保留真到距后的停止、历史记录和全部保护。backend/HWT/config字节未改，未调整查询、100ms命令姿态/400ms位置/120秒整组等阈值。70项主机回归通过，七种完整隔离构建0错0警告，源匹配/GBK/CRLF/diff核验通过；默认Code57284/RO788/RW2004/ZI32460。

用户确认回起点后，14:09:15修复版distance_handoff_fix_route_a_20261025重新下载Verify OK/Application running，先只读健康再提醒一次START。第一左30cm到距成功，PE2均值301.53302mm；第二段准备时首HF2 trip2/source_valid0、backend原VERIFY_ON/cause0，event_tick37857/reason1/SR232(0xE8)、UART错误1/ORE1，其余FE/NE/PE/环形溢出/校验错0。捕获HWT USART2硬件溢出这一独立故障；后续包装trip5/backend9不覆盖首因。没有再次重跑，用户确认已停下、灯灭，最后四轮0/VM请求及输出0。原换段修复已实施上板，但回归被ORE阻断，不称三段通过或HWT根因已修。启动临界区屏蔽过久仍重点候选，未测耗时，不能唯一归因接线/噪声。

详见docs/P1_DISTANCE_HANDOFF_FIX_2026-10-25.md、同日backend_fault_repro_*、distance_handoff_fix_*及review.json。下一项先量测/定位启动临界区与ORE，完成针对修复后再确认现场做单次两换段回归，不自动发车。当前板是修复版A组失败态；B组/五段回位未验。所有先前失败保留。本轮源码/隔离构建/烧录与实测分别记录，仅本地未提交推送。核R RUN-01/PWR-01，E LOC-03/05/07、RUN-06、MOT-02/04，Q不变。

## 2026-10-25本地补充保电换段原因3分支与HF2诊断，未下载未发车

继续排查上次A组左移后进入前进准备时cause3；HF1只证明当时HWT健康，不能从同一码唯一确定查询失败或命令/反馈过期。保持正常段间VM常开及原保护。新增后端首故障结构g_chassis_auto_fault_diagnostic/只读API，在原切故障前锁存触发分支、原state、IO结果/轮位、命令龄期、状态/速度mask与最大缓存年龄及tick。原因3分支细分1邮箱过期/2状态速度缺失过期/3端点批次过期/4端点读失败/5运行位置读失败/6状态速度读失败/7缺起点/8距离保护，0其它；后续Abort/STOP不覆盖。只在新后端会话被接受或初始化时重准备诊断，不清故障求接受。

平移首trip帧升级HF2共38字段（原HF1的29+后端9），仍仅故障后约1Hz输出。主机兼容HF1/HF2，不当令牌或续期，失败后仅STOP并收诊断，不重发START。70项主机测试通过，含HF2模拟串口失败保持失败且ARM/START/STOP各一次；默认/A组三段/B组转向右移/串口单段/手动/无线/位置诊断关七种完整Rebuild0错0警告。默认Code57268/RO788/RW2004/ZI32460。源逐字节匹配、GBK/CRLF/diff通过；保护宏及默认四段表与此前一致。证据evidence/p1-foundation/2026-10-25/backend_fault_diag_*_build.log、backend_fault_diag_review.json，指南docs/P1_BACKEND_FAULT_DIAGNOSTIC_2026-10-25.md。

本轮未开COM/下载/复位/发车，MCU新锁存/栈与开销/HF2实收未验证，原故障根因仍未证实；板上状态仅沿最后已知2026-10-24 A组失败记录，不称本轮已实读确认。下一项现场确认后，用backend_fault_diag_route_a_20261025原左/前/右三段单次复现，取HF2分支再针对修复；不自动重跑，B组/整条回位仍未验。100ms命令姿态、400ms位置、120秒整组、速度方向、VM/停车/看门狗均未放宽。fetch确认54bb617一致，既有本地成果保留，仅本地未提交推送。核R RUN-01/PWR-01，E LOC-03/05/07、RUN-06、MOT-02/04，Q不变。

## 2026-10-24 23:09五段回位试验中止：首左移完成，第二段前进准备cause3，HF1证实当时HWT健康

用户指定左→前→右→顺时针90度→转向后右移，确认各平移30cm且已就位。理论底盘中心可回起点，最终车头顺时针90度，不等于角点参考点重合。五段最坏预算172500ms超过原120000整组门禁，已向用户说明分A左/前/右（105500ms）、B顺时针90/右（72000ms）两组，未扩大保护。隔离return_square30_a/b只改动作表，全部C/H/S与已验诊断源码除动作表逐字节一致，两组完整Rebuild0错0警告，A Code56772/RO768/RW2000/ZI32224，B56776/748/2000/32224；主工程默认表不改。

A于23:09:12下载Verify OK/Application running，新会话4秒监听健康（3段/selftest1/源有效/VM0/故障0），发车提醒后一次START。第一段左30cm完成估算触发300.396mm，PE2方向1/距离1/双mask15，四轮投影301.6954/301.5552/301.5995/301.7471mm、均值301.64929mm；段间状态2/completed1保持VM1，约1秒后进入第二段前进。43700ms后台VERIFY_ON，43760ms首trip5、backend session2/state10/cause3，第二段progress0/reached0、全组completed1/3。主机发送STOP并继续收HF1，未自动重试；B未下载/未启动，后续右移/转向/最终回位均未执行，不能声称路线或回位成功。

HF1真实捕获2帧，首故障tick43760/arm2/原phase2/source_valid1/角度及速度龄期17/17ms，HWT UART错误/溢出/校验错及ORE/FE/NE/PE累计均0、event_reason0、source_sample_age10ms。因此本次捕获不是先前HWT源失效指纹，HF1锁存已在MCU真实运行并保留Abort前后端cause3（不再只见取消9）；但原因3被命令时效、反馈时效、CAN查询失败和端点读取等共用，现有HF1不含具体IO结果/地址/触发分支，不能唯一断言通信或命令时效根因。下一项优先查第二段VERIFY_ON换段初始化及CAN查询/端点首因，不放宽阈值或清故障求通过。

整次149TM1/74PM1/1PE2/2HF1/0无效，共56命令；终态四轮0/VM请求及PE2输出0，backend result2/cause3/stopmask0（非正常完成第二段停止mask15），系统fault0、HWT源仍有效。用户明确“已完全停下，VM灯灭”，保留原地失败场景、不复位。第一段成功不证明第二段毫秒级从无输出，也不作为完整回位测量。证据同日return_square30_a/b_build.log、a_download.log、a_prestart/a_run原始/JSONL/summary及return_square30_failure_review.json。

本轮未修改主工程运动代码或保护，新增隔离动作表/构建证据/交接，仅本地未提交推送。当前板A左/前/右三段程序失败态，车约在首左移终点，用户未移动；回位未测、B未执行。fetch确认54bb617一致。核R RUN-01/PWR-01，E LOC-03/05/07、RUN-06、MOT-02/04，官方Q不变。

## 2026-10-24 23:04单右移30cm诊断测试完成，用户手量约30cm略多

用户要求开始并确认“已就位，可以开始右移30cm”。fetch核main/origin均54bb617，诊断源SHA未变；从已通过左移诊断副本生成hwt_fault_diag_right_20261024，仅动作表LEFT_DISTANCE改RIGHT_DISTANCE/15RPM/300mm，所有C/H/S及工程文件除该表逐字节一致，未改主工程默认四段。完整Rebuild Code56768/RO728/RW2000/ZI32224、0错0警告。23:03:38下载Verify OK/Application running，新会话4秒只读核源有效/帧龄11/12ms、VM0/四轮0/系统fault0后提醒发车，仅一次START。

本轮software_complete，流程11195ms、15→8→3RPM，估算触发300.362mm/reached1。PE2方向2/距离模式1、起终mask15，四轮向右投影终点300.9424/300.9720/302.2122/301.7988mm，均值301.48135mm。终态completed1/1、四轮0/stop_mask15、backend result1/cause0、VM请求及输出0、系统与距离fault0。用户回复“30cm多差不多其实”，只记实际右移约30cm、略多，不录成精确30cm、30.148cm或具体误差；未给前后偏移/摆动、测量不确定度，未独立明确本次灯灭及物理停稳，不能扩大确认。手量与目标大致相符，不做比例/轮径/打滑补偿。

134TM1/67PM1/1PE2/0无效/0HF1，52命令发送/52接受/0拒绝。START后125TM1源全有效、两帧龄最大10/11ms、序号无跳；总87缺口含旧串口积压，不当运行期丢包。捕获峰速18/19/23/20RPM；yaw参考-2.642度，活动观测峰偏0.440度、固件peak_error0.659度，VM0首次-2.993度、补收-3.015度，相对起始-0.373度/关VM后-0.022度。正常完成无HF1，不代表此前偶发HWT失效/减速cause8已修或诊断故障分支已实测。

证据同日hwt_fault_diag_right_build/download.log、right30_prestart/run原始/JSONL/summary及right30_run_review.json。当前板为新几何及HF1诊断的单右距30cm固件，已完成会话；主工程默认原四段未改。仅本地新增隔离测试、证据与记录，未提交推送。下一项可按用户指令规划前进/左移/右移的短距离组合测试，先明确路线与摆位空间并验证；不自行发下一次运动，不将左右单次约30cm扩展为整车定位/重复精度/可靠性通过。核R RUN-01/PWR-01，E LOC-03/05/07、RUN-06、MOT-02/04，Q不变。

## 2026-10-24 23:01同诊断版左移30cm复测完成，用户手量30cm，不加补偿

用户表示可以测量并要求再来一次，已确认回标记起点、同参考点/朝向及空间条件“已就位，可以开始”。保留上一轮结果，fetch确认54bb617一致，main/hwt源SHA与既有隔离构建匹配，复用同AXF SHA256 2f9b8e1e2d0624e90c270ca10d859fe6c65072fd4189ab1cec89252dd50219de；未改源/参数、未重复构建。23:00:52下载Verify OK/Application running，新会话4秒只读核源有效/四轮0/VM0/故障0后提醒发车，仅一次START。

本轮software_complete，流程11141ms、15→8→3RPM，到距触发估算300.406mm/reached1；PE2左向1/距离模式1、起终mask15，四轮投影301.2230/301.3116/301.2746/301.1787mm，均值301.24696mm。最终completed1/1、四轮0/stop_mask15、backend result1/cause0、VM请求及输出0、系统与距离fault0。用户测量回复“30cm”，记同参考点实际左移手量约30cm，与300mm目标相符；未报告测量分辨率/不确定度、前后偏移/摆动，也未独立回答本轮灯灭/实际停稳，不能由30cm推定这些均已确认。编码器比手量多约1.247mm，单点手量不据此校准轮径、横移比例或宣称毫米级物理精度，不加补偿。

132TM1/64PM1/1PE2/1无效/0HF1，命令51发送/51接受/0拒绝；START后124TM1源全有效、帧龄最大7/8ms、序号无跳，总报告74缺口含开口旧积压不当运行期丢包。捕获峰速19/19/25/17RPM；yaw参考-3.103度，活动采样峰偏0.401度、固件peak_error0.538度，VM0首次-2.702度、补收-2.642度，相对参考+0.461度/关VM后+0.060度。正常完成未触发HF1，原HWT偶发失效及减速cause8仍待定位，不因两次成功解除可靠性问题。

证据同日left30_measure_repeat_download.log、idle/prestart/run原始/JSONL/summary及left30_measure_repeat_review.json。本轮源码/保护未变、未提交推送，板上仍同诊断单左移30cm程序且已完成；主工程默认原四段表不改。下一项按用户需求测右移或补横移前后偏移/重复测量；不自动再发车，不把本次左30cm手量推广到右移、其它距离或正式定位精度。仅本地记录。核R RUN-01/PWR-01，E LOC-03/05/07、RUN-06、MOT-02/04，Q不变。

## 2026-10-24 22:58诊断版单左移30cm完成，用户确认左移/停下/灯灭，原故障未复现

用户明确要求开始，fetch后确认main/origin均54bb617，核诊断main/hwt.c/h与review SHA及隔离构建逐字节一致、动作表仅左距15RPM/300mm。复用通过的hwt_fault_diag_left_20261024 AXF，22:58:08下载Verify OK/Application running；4秒只读监听分辨旧积压与新会话，最后源有效/VM0/四轮0/系统fault0。告知单次左移30cm后仅一次START，未连续重试。

本轮software_complete，流程11162ms，基础RPM15→8→3，目标300mm/估算触发300.119mm、reached1。PE2方向1/距离模式1、双mask15，四轮投影终点301.1565/301.2008/300.9203/300.9794mm，均值301.06425mm。最终phase3/组合completed1/1、四轮0/stop_mask15、backend result1/cause0、VM请求及PE2输出0、系统与距离fault0。用户对实际方向/完全停车/VM灯灭问题回复“是”，接受这三项现场定性确认；未给实际厘米数、前后偏移或摆动量，不推定物理距离精度已过。

135TM1/68PM1/1PE2/0无效/0HF1；发送52命令、接受51/拒绝1、终态remote_result6为结束边界，保留此限制，不写全部命令接受。START后125条TM1源全有效、帧龄最大14/15ms、序号无跳；总报告82个序号缺口含开口积压，不能当作运行期丢包。峰值捕获轮速19/18/22/18RPM。yaw参考-4.526度，串口捕获相对参考峰值0.346度、固件peak_error0.571度（采样/计算时点不同）；最终-4.306度，相对起始+0.220度，结束补收未再变化。保留原失败，不能因本次成功声称HWT偶发失效/减速cause8已修；没有trip故无HF1，新增锁存的故障分支尚未硬件验证，也不证明毫秒级无已恢复接收错误。

证据同日hwt_fault_diag_left_download.log、left_prestart/left_run原始/JSONL/summary及hwt_fault_diag_left_run_review.json。本轮未改源码/参数/保护、不重复构建、未上传；板上现为新几何和HF1诊断的单左距30cm固件，已完成会话；主工程源码仍默认原四段表。下一步可在可靠测量条件下确认横移距离/前后偏移，或按用户指令做另一方向单次测试；HWT首因仍需下次自然复现取HF1，不自行连续发车。仅本地保存。核R RUN-01/PWR-01，E LOC-03/05/07、RUN-06、MOT-02/04，Q不变。

## 2026-10-24本地新增HWT首次故障锁存与HF1诊断，未下载未复现

用户同意继续补诊断。HWT模块新增平移布防基线、首个UART/溢出/校验事件锁存、USART2 SR及ORE/FE/NE/PE累计分项；仅清诊断数据，不清源/错误计数/运动故障。主前端在首次平移trip、Abort之前锁存29字段含后端原state/cause/session与源龄期，g_motion_fault_diag供Watch，后续STOP不覆盖。独立HF1仅故障后约1Hz发送，运行期无新增HF1流量、不当控制令牌，TM1/PM1/PE2及动作表不改。主机失败先STOP再额外收1.5秒供HF1，不重发ARM/START/KEEP。首次软件锁存不等于独立物理首因；reason0也不代表无错。

64项主机测试通过，含新增6项诊断解码/CRC/uint32/非法标志/隔离及模拟串口ARM→START→故障STOP→接收HF1仍失败且不重发。默认组合/左距300mm/串口单段/手动/无线/位置诊断关六种完整隔离Rebuild全部0错0警告，源逐字节匹配、GBK/CRLF/diff检查通过。默认Code56776/RO788/RW2000/ZI32224。证据同日hwt_fault_diag_*_build.log、hwt_fault_diag_review.json，指南docs/P1_HWT_FAULT_DIAGNOSTIC_2026-10-24.md。

本轮未下载/开COM/发车/上传，板上仍之前单左距30cm失败固件（没有HF1），新诊断MCU行为/栈与开销尚未实测。未修改或测量原启动临界区，未放宽100ms姿态/400ms位置/速度方向/VM/停稳/看门狗保护，不称根因已修。下一项如实测先下载单段诊断版、只读核健康及现场摆位后一次复现，保存HF1；有ORE再定位临界区耗时及接收相位，不连续重跑刷通过。原根因、物理横移距离/停稳/精度仍待闭环。fetch确认54bb617无远程新提交，原本地成果保留，仅本地未提交推送。核R RUN-01/PWR-01，E LOC-03/05/07、RUN-06、MOT-02/04，Q不变。

## 2026-10-24最新本地：前进/左移/右移通用距离接口与方向历史帧完成，未实测

用户要求写左右横移距离代码，本轮新增MoveDistanceStart(direction,max_rpm,cm)，0前/1左/2右，无后退，3..15基础RPM/10..100cm；旧MoveForwardDistanceStart保留前进包装。主前端、后端通用距离启动与状态新增distance_direction，继续同一CAN任务/HWT/停稳/Finish保电流程。纯轮子行程按前进权重(+,+,+,+)、左移(+,-,-,+)、右移(-,+,+,-)投影后取均值及轮间差，避免横移直接求和抵消。400ms时效/跳变/倒退/轮差/截止/停止/故障保护原样，未加打滑或距离比例补偿。

动作表增加kind5左移距离、kind6右移距离，五字段/范围/预算规则及默认原四段表保留。TM1仍64字段、action_kind4/5/6明确方向，PM1不变；终点历史PE2新增translation_mode及distance_mode共31字段，锁存本段方向，非平移为3。主机兼容旧PE1及新PE2，新增投影行程输出，旧PE1未知方向不猜前进。方向标记不参与控制命令/令牌。公开接口示例可编译，8段历史容量与关VM后保留成功历史规则不变。

58项主机测试通过，新增8项PE2投影/左右/标志/CRC/兼容等；C自检扩展左右符号/反向/非法mode，仅编译未MCU执行。9个完整隔离构建（默认/前距离/左距离/右距离/混合/单段/无线/手动/诊断关闭）0错0警告；源比对/编码/CRLF/差异检查通过。证据lateral_distance_*_build.log和lateral_distance_review.json，指南docs/P1_LATERAL_DISTANCE_2026-10-24.md。主机测试不代表MCU横移距离控制实测。

本轮未下载/开COM/发车/上传，新通用接口、方向PE2和212/206几何都只在本地；板上仍之前旧几何单87cm前进程序（PE1）已完成会话。横移距离精度、打滑及原HWT偶发失效/减速cause8仍待验证，不把历史定时横移观察写成当前距离版通过。下一步如实测，选单段短左/右移隔离表，先健康监听、现场摆位并发车提醒；本轮只写代码。核R RUN-01/PWR-01，E LOC-03/05/07、RUN-06、MOT-01/02/04，Q不变。

## 2026-10-24最新本地：距离公开接口与厘米状态查询整理完成，不发车不补偿

用户同意先整理接口。motion_control.h将距离Start与其它启动接口归组，明确MoveForwardDistanceStart(max_rpm,distance_cm)参数顺序、3..15RPM/10..100cm、返回1仅接受、拒绝不自动重发；公开厘米边界常量保持原10/100数值，算法/减速/时限/保护不变。MotionStatus新增distance_mode/target_cm/progress_cm/reached/base_rpm/fault_reason，MotionReadStatus同一临界区复制，排队时目标可读、进度等只有后端模式/会话匹配才复制，防串入前一段。非距离和不可用模式新增字段全0；完成/故障后进度是最后估计，不是PE1终点或实时定位。

新增docs/P1_DISTANCE_API_2026-10-24.md，说明单次请求→周期查询→DONE且到距/底层停稳/故障检查→继续下一段或Finish→等输出关闭，示例不注册任务、不自动启动。五配置完整构建0错0警告，公开头文件调用示例独立ARMCC编译通过，原50项主机测试通过；新增快照尚未MCU运行验证，编译不冒充实测，未被调用时可以链接裁剪。证据distance_api_*_build.log和distance_api_review.json。源比对、GBK/CRLF及差异检查通过。

本轮不启用任何前进补偿，用户已明确手量约89cm不准确，只保留近似观察，不当标定真值，也不称真实稳定偏长已确认。轮径77/左右212/前后206本地配置保留。未下载/开COM/发车，板上仍旧几何及旧公开状态的单87cm已完成程序，源码默认四段未变。HWT偶发失效/减速cause8与实际距离/右偏/正式自主入口仍未闭环；按用户要求先保留问题，不通过改保护签通过。fetch确认54bb617无远程新提交，既有成果保留，仅本地未提交推送。核R RUN-01，E LOC-03/RUN-06/MOT-02，官方Q不变。

## 2026-10-24几何参数更正：左右212mm、前后206mm，仅本地编译

用户明确纠正CAD尺寸方向：前后轮心距206mm、左右轮心距212mm。按ChassisGeometry字段顺序wheel_diameter_mm/track_width_mm/wheelbase_mm，g_chassis_geometry由77/210/210改为77/212/206，补字段中文方向注释，轮径77mm保持；不是将前后/左右写反为77/206/212。几何旋转因子(track+wheelbase)/2由210改209mm，纯前进行程换算仍按轮径及位置增量，本轮不改距离补偿或运动保护。

隔离geometry_212_206_20261024默认模式3/原四段表完整Rebuild55252/776/1968/31480，0错0警告，源与构建一致、GBK/CRLF与差异检查通过；证据同日geometry_212_206_build.log及review.json。本轮未下载/开COM/发车，板上仍之前单87cm程序（旧210/210几何），不能把新参数当已实测。尺寸由用户确认，不声称独立物理标定。既有本地代码/构建产物保留，仅本地完成未提交推送；fetch确认54bb617无远程新提交。前进约89cm/右偏、偶发HWT失效及减速方向保护仍沿既有记录待闭环，不因轮心距更新消除。核E LOC-03/MOT-02，R与官方Q不变。

## 2026-10-24最新重测：相同87cm固件再次估算到距成功，用户再次量约89cm，横偏未量

用户“再试试”后的retry3已在835.812mm、切到基础3RPM附近触发backend cause8速度/方向保护，reached0、无PE1；失败瞬间缓存0/+1/+5/-10RPM、源有效、峰值未超过30RPM，提示方向/减速瞬态问题，但原因8未分项锁存不能称唯因已证实。用户随后再次明确“再来一次”，确认停下灯灭并放回标记起点；保留retry3失败证据，未改代码或放宽保护。

核源与distance_sched_87cm隔离构建逐字节一致，同AXF18:41:09下载Verify OK/Application running，先4秒只读新会话健康/源有效/VM0，再提醒发车。retry4本轮software_complete，目标870mm、触发估算870.312mm，完整PE1终点四轮866.5065/876.2585/866.1964/876.1994mm、平均871.290206mm；总流程20589ms、基础15→8→3，最终四轮0/停止mask15/VM请求及PE2均0、系统及距离原因0。途中固件yaw误差峰值0.769度、串口观测0.747度；参考−178.154，VM0时−178.703，补收后−178.698，相对起始−0.544度，关VM后+0.005度。用户仅回复89，记本轮左后轮中心前向手量约89cm，未单独提供横偏厘米数及灯/摆动确认；不能把之前“左89右87”解为右偏2cm。

本轮与先前成功distance_sched_run的编码器均值均约87.1cm，用户两次报告约89cm；本轮实测比估算多18.710mm，单点前进比例候选1.021474只记证据、不应用到77mm轮径或控制。手量投影/参考点/轮胎滑移尚未区分，实际距离精度与横向直线性不签通过。此前retry2准备阶段HWT短暂无效、retry3减速cause8仍未闭环，不能因retry4成功写成可靠性已解决。

228TM1/114PM1/1PE1/0无效行，遥测丢弃计数1；命令93条、接受92、拒绝1，结束边界remote_result6保留，不写全程零拒绝，不影响本轮已确认运动终态。原始DAP积压序号跳变不直接当运行期丢包。证据retry3_run与retry3_failure_review、retry4_download/prestart/run及distance_87cm_retry4_review.json。当前板仍相同单87cm固件且本轮已完成，源码默认四段未变；本轮未改代码/参数、不重复构建，仅本地记录未上传。

下一项先分开处理前进比例候选与右偏：若能测量，补同一参考点相对原前进线的横偏；若不能则保留定性右偏。更优先记录/定位HWT偶发失效与减速方向保护的触发条件，不连续重跑刷通过、不扩大保护门槛。未新建自动任务。核R RUN-01/PWR-01、E LOC-03/05/07、RUN-06、MOT-02/04，Q不变。

## 2026-10-24最新重测：87cm准备阶段HWT源失效中止，未形成新距离，暂不自动再跑

用户先说“左89，右87”，两侧各自纵向行程还是其它测法未澄清，不能当向右偏2cm或直接校准。用户随后要求再跑一次并确认已放回标记起点、前方至少1.5m且可观察。核当前源与已通过distance_sched_87cm隔离构建一致、单目标870mm/最高15RPM，代码参数未改不重复构建；18:33:19同AXF下载Verify OK/Application running。先监听新会话健康/VM0，再明确提醒发车。

本次仅准备约1067ms后motion_fault_or_cancelled，completed0/reached0/progress0，无PE1，全部捕获轮速0。31700遥测source_valid0、angle_age及rate_age均-1、yaw保持-178.665，backend FAULT/result2/cause9为上层取消/中止，组合cause5；CAN/RTOS/Safety均0，VM请求/PE2均0、stopmask0（不是正常完成四轮停止确认），不能宣称87cm或本轮yaw全程通过。31600之前捕获backend仍idle/源健康，不能证明毫秒级未发生任何输出，但没有正常运行或行程记录。主机发STOP保留终态，未重复START。

4秒只读postfault监测看到HWT源恢复，年龄11/12ms、轮速0、VM0，失败会话仍保持phase4，未清故障/自动重试。源码HWT101_Update在接收错误epoch变化时会作废parser，未见角度/速度时年龄为UINT32_MAX（有符号显示-1），与该帧相容；当前遥测没有HWT接收错误计数或原始字节，不能判定接线/上电噪声/具体错误根因，不能将恢复解释为此前无失败。

证据distance_87cm_repeat2_download.log、repeat2_prestart/run/postfault原始/JSONL/summary、distance_87cm_repeat2_failure_review.json。本次无需用户量尺；上次成功到距及约89cm/轻微右偏记录保留但不扩大为本次成功。当前板仍相同单87cm修复版、失败终态，源码默认四段不变。本轮未改代码/运动参数/保护，未提交推送。下一项先核HWT连接/供电及接收错误诊断，保留原失效即停，不盲目扩大时效或连续重新上电发车；故障诊断完成后再单次重测。不把“左89右87”当横偏量。核R RUN-01/PWR-01，E LOC-03/05/07、RUN-06、MOT-02/04，Q不变。

## 2026-10-24最新：前进距离接口87cm估算到距完成，用户约89cm且右偏，物理精度未闭环

按用户要求“前进87cm”并记录yaw，新增MoveForwardDistanceStart(max_rpm,cm)仅模式3前进10..100cm，MotionAction新增第五distance_mm和kind4；默认四段表未改，仅隔离表单870mm。起点基准+四轮int64增量/名义77mm换算均值，剩120mm最高8RPM、40mm基础3RPM、平时最高15RPM，HWT原±2RPM保持方向。目标到达才正常停止，时间到是失败；仅新距离路径30秒运行，旧定时8秒/后端10秒上界不变。新增400ms位置过期/跳变/倒退/轮间差等保护，原状态/速度/命令/姿态/停止/看门狗不放宽。

首版估算361.180mm时cause3保护关闭VM，用户确认停下灯灭，未完成87cm。日志第3轮在42610后未更新、43040已关VM，结合源码指向纠偏下发START阶段未服务位置造成400ms过期；首版无专用原因码，不称唯一硬件根因已捕获。失败原始数据和构建保留distance_87cm_run及first_failure_review。修复为距离模式运动中调速START也利用原monitor槽、180ms以上最旧位置优先，仍400ms截止，起点缓存用原真实时间。新增distance_fault_reason，TM1由58扩64（兼容63及历史），主机无reached不报成功。用户放回起点并确认后才下载重测。

六配置修复版完整构建均0错0警告，50项主机测试通过，源比对/编码/CRLF/差异核验通过；MCU距离数学自检作为启动门禁通过。18:19:10烧录Verify OK/Application running，先监听健康再提醒发车。修复版software_complete、总流程20600ms，到距触发870.206mm，停稳PE1四轮平均871.1241mm，15→8→3档真实出现。最终四轮0/停止mask15/VM请求及PE2均0，系统与距离原因0，无失联/UART错。运动期串口捕获位置年龄最大178ms，不代表全时WCET证明。229TM1/116PM1/1PE1/1无效行，旧DAP积压不计作已证实运行期丢包，主机旧baseline字段不代表实际源54bb617加本地修改。

yaw参考−179.406°，执行观测最大偏差0.743°、固件峰值0.747°；首次VM0读179.884°，补收后179.851°，跨±180处理后终态相对起始−0.743°，关VM后再变−0.033°。终态控制error冻结0.703°与之后yaw不同时间。用户实测以左后轮中心为基准约89cm，感觉略向右走歪，未给横偏厘米数和测量不确定度；未单独确认本轮灯/摆动。故只签估算到距与软件安全结束，不签87cm物理精度或直线性。89cm比目标多约2cm，轮径/手量投影/滑移等未区分，未应用补偿系数或改77mm轮径。

说明docs/P1_FORWARD_DISTANCE_2026-10-24.md，证据distance_sched_*_build/download.log、distance_sched_run原始/JSONL/summary、distance_sched_pass_review、distance_forward_build_review。当前板为修复后的单87cm距离版且已完成；真实源码默认原四段表。当前所有成果仅本地未上传。下一项先固定参考点、分别测前向及向右偏移，结合yaw和纠偏阈值分析，再确定校准/控制改动；不自动再发车，不把yaw保持等同横向闭环。X/Y/横移距离/正式自主入口/全部故障验证仍待后续。核R RUN-01/PWR-01、E LOC-03/05/07、RUN-06、MOT-01/02/04，Q不变。

## 2026-10-24补充测量：第二次单段实测31.5cm，名义估算约30.63cm，暂不改参数

用户对position_endpoint_repeat_run本轮测量问题回复31.5，单位沿问题记为cm；只补这一次的测量，不填入首轮。四轮名义理论行程为306.4053/308.1918/304.2202/306.3315mm，本轮近直行用四轮前向平均作粗对比为306.287212mm，实际315mm比该值多8.712788mm（约0.87cm）；估算相对实测偏差-2.766%。单次比例候选1.028446仅写证据，不写入轮径、运动学或控制参数，不据此认定物理轮径或唯一定性打滑/测量误差。

保留前述软件完成/完整起止记录证据；用户此回复仅给距离，未单独回答本轮摆动及VM灯现场现象，不擅自补“都符合”。相同固定参考点是此前给定测量要求，本轮未获得测量照片/精度或横向偏移，四轮均值不能替代独立定位。软件最终四轮0/VM请求和PE2均0已有记录，区别于现场灯与轨迹观察。

更新position_endpoint_repeat_review.json中actual_displacement_cm与comparison。当前距离单位/整段起止采样有首个量尺对照，比例重复性、实际轮径/滑移/固定偏移分辨与指定距离/X/Y尚未闭环。后续若校准优先再取不同长度的同参考点前进样本（现有范围可用15RPM/8秒），检查比例是否稳定后再决定参数；本轮未增加运动、未改固件、未下载，记录仅本地未上传。核R RUN-01/PWR-01，E LOC-03/05/07、RUN-06、MOT-02/04，Q不变。

## 2026-10-24追加实测：用户要求重跑一次以便量尺，第二次正常完成，仍待实测距离

首轮软件成功后用户回复“再来一次”。已提醒先标固定车体起点、留前方约1米空地；核当前源与原单段15RPM/5000ms隔离构建一致，未改代码或参数。相同AXF重新下载自动复位，17:49:46 Verify OK/Application running，先只读新会话健康/VM0，再明确提醒发车，一次启动完成。未清故障绕过新会话，未重复构建相同源。

本轮software_complete，完整PE1一条，有符号位置增量为83011/-83495/82419/-82991；四轮理论306.4053/308.1918/304.2202/306.3315mm。起止各批跨度33ms，每轮坐标区间6170ms；总流程7631ms，最终四轮0/stopmask15/VM请求及PE2为0，CAN/RTOS/Safety/backend cause0、无失联/UART错误。航向遥测峰值0.725度、结束控制误差0.346度，仅软件观察。93个TM1、48个PM1、1个PE1、1无效行，DAP启动积压跳变不作运行期丢包统计。

已再次请用户量这次同一固定车体参考点实际位移，并确认摆动/VM灯现象，等待回复；不把软件停止等同现场验收，不先改轮径/距离比例。首轮未取得量尺值，本轮与首轮数据分别保存，不混用。证据position_endpoint_repeat_download.log、position_endpoint_repeat_prestart/position_endpoint_repeat_run原始/JSONL/summary、position_endpoint_repeat_review.json。当前板仍单段完整起止诊断版且已完成，源码默认四段表不变。仅本地未上传，下一步等本轮实际厘米数后再校准，不自动再发车。核R RUN-01/PWR-01，E LOC-03/05/07、RUN-06、MOT-02/04，Q不变。

## 2026-10-24最新实测：完整起止位置/PE1单段前进正常完成，等待量尺与现场确认

用户要求开始，隔离position_endpoint_single_run_20261024只设前进15RPM/5000ms一段，不改源码默认四段表。完整编译52376/680/1936/31408、0错0警告，17:46:34命令行烧录Verify OK/Application running，先只读核新会话/sequence_count1/健康闲置/VM0，再明确提醒发车，一次ARM/START完成。当前板已变为本单段起止位置诊断版，已完成不自动再发车。

software_complete，完成1段、总流程7643ms（含准备和停车），stopmask15，最终四轮0、VM请求/PE2均0、CAN/RTOS/Safety/backend cause均0，无失联和UART错误。收到1条完整PE1，起止mask均15/outcome1，起点raw均0，终点有符号82961/-83191/82373/-82613；按77mm名义轮径及已验轮向换算四轮理论行程306.2208/307.0697/304.0504/304.9363mm，不当作已量尺的车体位移。起止各批跨度33ms，每轮坐标区间6170ms，包含稳定等待不等于运行5秒。PE1在设备tick31270发出，主机正常完成后仍收至32000，末尾补收有效。

94个TM1、48个PM1、1无效行，遥测最大航向误差0.686度；RX余栈309字、最小堆9792字节仅本轮观测。保留启动旧DAP积压与无效行，summary序号跳变124不能直接当运动期丢帧。旧baseline5f132ad字段不代表实际源，review注明54bb617加本地修改。已请用户量同一个固定车体参考点的实际位移并确认无摆动/VM熄灯，尚待回复，不先写物理精度通过或改参数。

证据position_endpoint_single_run_build.log/download.log、position_endpoint_prestart/position_endpoint_run原始与JSONL/summary、position_endpoint_single_run_review.json。后续只需补本轮现场测量值/现象，先比较理论与实际，不自动再次发车；轮径/滑移校准、指定距离停车与X/Y仍未闭环。仅本地未提交推送。核R RUN-01/PWR-01，E LOC-03/05/07、RUN-06、MOT-02/04，官方Q不变。

## 2026-10-24最新本地：起步前/停稳后位置批次与PE1完整行程记录已编译，未上板

用户同意继续补起止基准。诊断开启时，目标航向后端VERIFY_ON原状态/新鲜全零速条件成立后、START前逐轮读0x36起点；SETTLE原零速/至少1秒稳定成立后、DONE前读终点。每轮仍原检查，完整批次时效<1000ms，读失败原cause3停车，取消记录失败，不延长任何准备/运动/反馈/2.5秒停止截止。新起止各4个有界事务可能增加耗时，实际时序待验。

新增历史ChassisPositionEndpoints：每复位前8个动作，关VM后正常完成记录保留；失败/不完整不发成功PE1，超8不保存完整记录不影响原运动。独立PE1含29个uint32和CRC，输出session/outcome/mask及四轮起止符号/raw/时间。主机宽整数相减，再按已验+/−/+/−和77mm轮径换算四轮理论行程，不冒充实际车体位移；完成后多收0.7秒最终记录、不继续发命令。现有PM1实时有效位规则不变。

46项主机测试通过，包含模拟COM末尾PE1补收与不发控制命令；5配置完整隔离构建均0错0警告，源比对/编码/CRLF/差异检查通过。证据position_endpoint_*_build.log和position_endpoint_review.json，说明docs/P1_POSITION_ENDPOINTS_2026-10-24.md。主机测试不算MCU时序/实机通过。本轮未下载/开COM/发车，板上仍上轮RUN-only两段短前进诊断版，源码默认四段表未改；仅本地未提交推送。

唯一下一项是新起止版单段短前进实测，建议15RPM/5秒隔离表，先健康监听再发车提醒，核完整PE1、正常停稳/关VM及现场；随后同一车体参考点量实际位移，校准名义行程。尚无量尺数据、跳变/驱动坐标恢复/实际位移与X/Y或指定距离停车闭环，不扩大之前通过结论。核R RUN-01/PWR-01，E LOC-03/05/07、RUN-06、MOT-02/04，Q不变。

## 2026-10-24最新实测：运行期四轮位置诊断通过本轮短前进观察

用户要求开始；隔离position_diag_smoke_20261024仅将表改为两段15RPM/2000ms前进，实际源码默认四段表保留。编译51112/696/1924/30004、0错0警告，17:32:49命令行下载Verify OK/Application running；先监听3秒，确认新PM1、58字段、两段/健康闲置/VM0，再明确提醒现场后一次发车。当前板已替换为位置诊断两段测试版，已完成不自动重复运行。

software_complete，2段总流程8993ms，段间VM保持1，最终四轮0、stopmask15、VM请求/PE2均0、CAN/RTOS/Safety/backend cause均0，无失联和UART错误。用户确认两次前进、无明显摆动、段间灯亮、最终停稳熄灯。遥测最大角度误差0.494度，末段固件峰值0.521度，采样范围不同；RX余栈311字、最小堆9784字节只是本轮观察。

收到51个PM1、102个TM1、1无效行；两会话分别看到61次诊断更新，四轮原始位置均变化，所捕获去重样本中左侧增量正、右侧负，对应地址1/2/3/4为+/−/+/−。第二段地址1到67165、地址2负67199，均观测到跨65536不单圈归零；地址3/4未跨界，不扩大为四轮完整多圈边界全部验证。各轮去重样本在PM1中的间隔132至264ms，是串口200ms抽样后的观察间隔，不等于MCU全部样本的实际最大间隔。RUN结束后停止更新，年龄增长到1000ms即撤有效位，最终VM0的valid_mask0且signed_counts均None，未用发送时间伪刷新。

位置仅RUN采样，未覆盖起步前/停稳后，不能把这些增量称作整段距离或精度验收；没有量尺测量。本轮低速插入诊断未触发保护，不代表所有负载/故障/实时性通过。保留DAP启动旧缓存/1无效行，summary累计sequence_gaps87不作运行期丢包结论；summary遗留baseline5f132ad，实际源54bb617加本地修改已在review注明。

证据position_diag_smoke_build.log/download.log、position_diag_prestart/position_diag_run原始/JSONL/summary、position_diag_smoke_review.json。代码保护/参数本轮未再修改，37项主机检查此前通过，本轮仅隔离表变化及实车记录，未上传GitHub。下一项补安全的起步前和停稳后位置基准，先形成完整单段轮子行程；届时再请用户量尺校准，之后才接指定距离/HWT辅助X/Y。不重复本次已过的原始采样，不测后退。核R RUN-01/PWR-01，E LOC-03/05/07、RUN-06、MOT-02/04，Q不变。

## 2026-10-24最新本地：运行期四轮位置诊断与PM1已编译，未下载未实测

用户同意补当前运动期间的位置缺口。app_config新增APP_POSITION_DIAGNOSTIC，默认模式2/3随串口控制开启，0可关。ChassisAuto唯一CAN执行器在正常RUN成功速度读取后穿插该轮0x36，保留符号/32位raw/实际时间/结果/会话；准备/改速/STOP/SETTLE不插入，停止和原反馈先检查，状态或速度任一达500ms时跳过诊断，原1000ms保护/姿态/提交/停止截止/看门狗不放宽。位置读取失败原cause3停车，不隐藏错误。新动作清基准、撤VM失效，过期/供电/会话/故障检查有效位，快照只读。

新增独立PM1：25个uint32字段+CRC，每轮result/sign/raw/sample_ms/age_ms，带session/更新序号/有效位/VM输出。每200ms在TM1发送尝试后至少40ms非阻塞尝试，忙则丢诊断帧，原TM1和白名单不改。主机保存PM1原始/JSONL和last_position，宽整数解码，失效None；PM1不参与控制门禁、令牌或保活。原8事务RUN监测全部插入时变12事务，沿用20/30ms等待，不能以构建证明实时性；PM1独立丢弃计数尚无。

37项主机测试通过，5个完整隔离构建（组合/单段/无线/手动/诊断关闭）均0错0警告，源比对/编码/CRLF/差异检查通过。证据position_diag_*_build.log、position_diag_review.json，说明docs/P1_POSITION_DIAGNOSTIC_2026-10-24.md。本轮未下载/开COM/发车；板上仍上轮两段短前进旧测试版，源码默认四段表未改。新CAN频率/时效/PM1/资源与动态符号均未实测，仅本地未上传。

唯一下一项是本版短行程位置诊断实测：另建两段短前进隔离表，下载后先只读健康，再提醒现场观察后一次发车，核原始增量/各轮时间/安全与资源，不重跑全部组合。当前只取RUN位置，缺起步前和停稳后基准，会漏首尾行程，不可直接据此实现指定距离停车；之后补起止基准/连续性/校准再做HWT辅助X/Y。核R RUN-01/PWR-01，E LOC-03/05/07、RUN-06、MOT-02/04，Q不变。

## 2026-10-24最新只读：位置单位核对完成，下一项补运动期间位置诊断

按用户同意继续阶段3底盘与定位，直接渲染查看厂家手册第72页5.5.13并核当前CAN/main/执行器。FW_Emm位置0x36为独立正负号和32位幅值，每圈65536；现有解析一致。77mm直连轴1:1每圈理论24.190263cm。地址1左后/2右后/3左前/4右前；0/1/0/1是已验前进命令方向，位置增量正负尚未动态确认。采用64位有符号差值后换算，不能先int32截断或float大数相减，不把65536当回绕上限。

发现关键缺口：旧App_CanFeedbackStep虽有位置采样且2026-10-14四轮静态读取已通过，当前目标航向/平移分支却由ChassisAuto执行器只读状态/转速，未读位置，TM1也没有位置字段。旧位置变量不代表运动时新鲜里程。不重复静态查询，不直接接X/Y或指定距离停车。

下一项先补可关闭的位置诊断采样和只读串口输出：唯一CAN任务排队，保留符号/raw/有效位/各轮真实时间，评估额外总线时延、不影响停止/状态/速度保护，不放宽阈值。断VM/复位/过期/跳变重建基准，不跨未知驱动坐标连续累计。诊断版准备好后再明确提醒发车，以短行程核动态符号/多圈连续性及量尺行程；然后接HWT辅助X/Y和距离停车。

本轮只有文档与源码审查，无固件修改、无构建/下载/开串口/上电/发车；板上仍两段短前进测试版。详见docs/P1_POSITION_UNITS_REVIEW_2026-10-24.md及同日position_units_readonly_audit.json。动态位置和X/Y未实测，不把理论分辨率当精度。fetch确认54bb617无远程新提交，既有成果保留，仅本地未推送。核R RUN-01/PWR-01、E LOC-03/05/07、RUN-06、MOT-02/04，官方Q不变。

## 2026-10-24最新本地：运动接口整理完成，增加上层只读快照

用户要求先整理现有运动接口供后续调用。本轮motion_control.h按方向、范围、状态、读取、完成、取消和显示组织，补充非阻塞语义/任务上下文/模式3开发限制；MoveStart/TurnStart校验改引用头文件常量，原参数范围不变。新增MotionStatus及MotionReadStatus一次短临界区复制动作状态、会话是否打开、VM请求与PE2输出、后端活动/结果/原因/停止掩码、CAN/RTOS/Safety故障，空指针拒绝，不发送CAN、不修改运动。读取成功不等于健康，backend结果可暂留上一段，DONE不等于已断VM。

新增docs/P1_MOTION_CALLING_GUIDE_2026-10-24.md给出单一上层的周期调用示例、等待DONE/下一段/Finish/输出关闭、拒绝与超时处理。示例未接入任何任务，未增加自动启动或绕过串口保护；动作表继续复用现有接口，不重复新建调度器，不新增正式任务流程或后退。四种模式隔离完整构建均0错0警告，日志motion_interface_*_build.log；快照当前无人调用可被链接器裁剪，编译不算实机快照验证。证据motion_interface_review.json。原文件GBK/CRLF保持，差异检查通过。

本轮没有下载、开COM或发车；板上仍上轮两段短前进实测版本，源码默认四段表未变。忙时拒绝、独立调用方、新TurnStart及本轮快照运行未单独实测。保留已有源码和产物修改，仅本地完成未提交推送；fetch确认与origin/main均54bb617、无新提交。下一步可按此头文件接入确定的上层动作，无需重新全测已通过路线；正式自主入口、距离/XY与异常验证后续。核R RUN-01/PWR-01与E LOC-03/RUN-06/MOT-01/02，官方Q不变。

## 2026-10-24最新实车：新单动作接口两段前进衔接通过

用户确认摆好并可观察，发车前明确提醒；隔离构建motion_api_smoke_20261024仅将动作表设为两段前进15RPM各2000ms，不修改实际工程默认四段表。完整编译0错0警告，17:10命令行烧录Verify OK/Application running，先只读确认新58字段/sequence_count2/健康空闲/VM0，再一次ARM/START执行。当前板上已变为该两段短前进测试版，原54bb617四段程序已被替换；程序已完成，不自动再次发车。

软件software_complete，完成2段、总流程8895ms（含准备/停车/间隔）；段间四轮0且VM请求/PE2均1，最终四轮0、stopmask15、VM请求/PE2均0。CAN/RTOS/Safety/backend cause均0，遥测最大角度误差0.598度，当前段固件峰值0.422度；两者范围不同。用户确认两次前进、无明显摆动、段间灯亮、最终停稳熄灯。新MoveStart经动作表调用的衔接及MotionFinish关电通过本轮低速验证，不扩大为直接独立调用、忙时拒绝或新TurnStart已实测。

证据同日motion_api_smoke_build.log/download.log、api_smoke_prestart和api_smoke_run原始/JSONL/summary、motion_api_smoke_review.json。DAP开口仍见旧缓存，启动跳变及1无效帧保留，不把summary累计sequence_gaps73认定运行期丢帧。主机summary的baseline字段仍旧5f132ad，实际源基线54bb617加本地修改，已在review纠正说明。29项主机检查此前通过；本轮未上传GitHub。下一步按需要接入上层路线调用，未单测忙时拒绝/独立调用/新转向路径仍明确保留；距离/XY按用户决定后续再做，不测后退。

核R RUN-01/PWR-01与E LOC-03/RUN-06/MOT-01/02，官方Q不变；串口仅开发验证。

## 2026-10-24最新本地：单动作RPM接口与理论速度显示，无后退

按用户确认新增MoveStart(方向,RPM,毫秒)、TurnStart(方向,正角度,最大轮RPM)、MotionGetState/Stop/Finish，复用原CAN任务；动作表调用同一接口。忙时拒绝覆盖、动作正常DONE保VM，整组最后显式Finish关闭VM，故障/取消/失联和停止保护保持。直接动作与动作表互斥，沿用复位新会话边界，不清故障续跑。接口当前仅模式3启用。默认路线与转向减速不变，后退排除。

RPM继续作为控制单位，cm/s仅理论速度显示：77mm直连轮15RPM约6.05cm/s，不当作实际横移速度。组合TM1增加到58字段，主机兼容48/51/55/56/58；转向速度显示不适用。详细范围和调用时序见工创赛/docs/P1_MOTION_API_2026-10-24.md。

最终源码与四种隔离构建逐字节核对，均0错0警告，29项主机测试通过；编码与CRLF检查通过。证据同日motion_api_*_build.log和motion_api_review.json。主机测试不是MCU运行验证。没有下载、开COM或发车，板上仍上次实测通过基线；所有本轮改动仅本地未提交推送，最后上传54bb617。续作fetch成功，HEAD与origin/main无差异，原构建产物/布局及其他既有改动保留。

下一步只安排单动作接口相关最小回归（已通过的平移、忙时拒绝、DONE保VM、Finish关VM），不重新全测组合、不测后退；实车开始前告知用户。距离/XY后续再说。核R RUN-01/PWR-01与E LOC-03/RUN-06/MOT-01/02，官方Q不变。

## 2026-10-24最新本地开发：固定四段改编译期动作表，未下载/未实测/未上传

开发前fetch确认main/origin main均54bb617，无队友新提交，保留已跟踪Objects/Listings/Keil本机布局修改。用户确认下一步把写死路线改动作列表。本轮新增USER/motion_action_plan.h，默认仍前进15RPM/8秒→左移15RPM/8秒→逆时针90度→前进15RPM/8秒；支持1至8项前/左/右平移或相对转向，仅改表即可改顺序/次数/参数，不新增任务。平移3..15RPM、1..8000ms、转向±5..±90度，错误字段/越界/容量/明显超过整组预算的表在动作前拒绝。

前端按当前项选择模式/速度/时长，后端BeginTranslationTimed锁存参数并按时长+1秒独立兜底，纯轮速函数AtSpeed叠加原±2纠偏；旧函数保留包装，旧独立模式原时长上界10秒保留，动作表自身限8秒。整组持续上电、首次准备一次、段间停稳与状态复核、最终/异常断VM、故障锁存/取消/失联/看门狗不变。重复转向仅在上一段正常完成且无trip后重建局部会话，避免原每次上电只能转一次导致列表第二个转向失败。数组容量8，结束判断按表数量而非写死下标3。

新组合TM1为56字段，新增sequence_count，主机兼容48/51/55/56并按实际段数等待完成；动作表不可通过串口改写，原私有启动许可保留。27项主机测试通过，含变长1/3/5/8段/非法数量及旧协议。默认组合、原单段/无线/手动与一段负90、五段混合不同速度时长/连续转向、八段边界共7个完整隔离构建均0错0警告，证据action_plan_*_build.log和review.json。固件运行时表校验、参数化命令和重复转向仅源码审查/编译，尚未执行MCU仿真或实车，不能用主机测试代替。新文件GBK/CRLF，原文件编码换行保留。

板上仍54bb617对应实测成功的四段固件；本轮没有下载、开COM或发车。默认路线不变不等于新二进制已实测。当前源码模式3动作表版，仅本地完成未提交推送，最后GitHub提交仍54bb617。操作见docs/P1_MOTION_ACTION_PLAN_2026-10-24.md。后续安排实测时先核新主机协议和实际表，再告知现场动作并确认摆位；距离/XY仍按用户决定后续再做。核R RUN-01/PWR-01、E LOC-03/RUN-06/MOT-01/02，官方Q不变。


<!-- 合并保留：以下为队友同期机械臂进度；本地底盘最新实测见上方或当前根交接。 -->

> 2026-11-02整车汇合：本聊天135°固定抓放已获用户口述确认；源105000下降/104000预升、车载28000下降/27000预升后回零。完整机械臂源码在工创赛/handoff/p2_fixed_cycle_20261102/firmware，与根3.stm32f407工程的最新电控版尚未合并。下一项对接底盘停稳→机械臂单事务→物料留台/空爪回顶→运输门禁。最新交接见P2_HANDOFF_CURRENT_2026-11-02.md。雷达归属按用户2026-10-28/29决定仍为F407 USART3，Orange Pi负责视觉；保留下方历史归属描述供追溯。本地旧文已归档到archive/P1_LOCAL_PLAN_BEFORE_SYNC_2026-11-02.md，不丢弃本地计划；R/E/Q以当前competition_requirements_2027.md为准。

## 2026-10-24上传交付：无线、左右横移和四段组合实测基线

用户明确要求“上传”，本次交付自5f132ad后全部经审查的固件、串口主机脚本、当前模式指南及成功/失败实测证据，作为新回退基线；不是只提交最后一个改动。源码当前APP_TEST_MODE3、组合转向30秒、段间持续VM；板上同版已完成一轮，默认不会再发车。无线双向7/7、左右移各8008ms、组合4段56304ms及用户观察确认分别记录，不扩大为距离/XY/所有异常工况通过。

排除仍已跟踪但本机修改的Objects/Listings、uvoptx/uvguix布局和被忽略tmp缓存，不删除磁盘成果，也不把旧构建产物当最新固件。操作指南已更新为最终配置；历史阶段描述保留作证据。构建与实际通过版本源一致、25项主机检查通过；提交前再次fetch核远程，提交/推送结果以最终本地和origin/main哈希核验为准。具体提交号可由此记录对应Git提交查到，原回退点5f132ad仍保留。后续需精确距离/XY、载荷高速和系统故障注入验证；本次不修改已通过代码、不下载或发车。R RUN-01/PWR-01与E LOC-03/RUN-06/MOT-01/02已核，Q不变。


## 2026-10-24最新实车通过：四段组合完成，段间保电，90度到位，最终自动关VM

用户确认重新摆好，源与combined_turn30s_combo_20261024构建一致；命令行实际下载Verify OK/Application running（15:36:05），先只读确认新会话mode3/健康闲置/VM0。发车前明确提醒后执行单次run-once。结果software_complete，sequence_state3/index3/completed4，elapsed56304ms（包含准备、停车和段间），backend DONE/result1/cause0/active0，stopmask15，四轮反馈0，VM请求0/PE2控制0，CAN/RTOS/Safety故障0，link_lost0，UART接收错误0。用户随后确认“都符合”：前进→左移→逆时针转90度→沿新车头前进，段间VM灯保持亮，最后自动停稳并熄灭。本轮四段低速组合及保电衔接可接受，不扩大为距离/精确XY路径或全部故障测试通过。

逐段完成采样：前进目标88.692/yaw88.181/控制误差+0.510度；左移88.187/88.401/-0.214度；转向178.401/177.484/+0.922度；最终前进177.484/177.138/+0.340度。控制误差与最新yaw来自不同更新时刻，不强行视作同步。三个段间各采到10帧VM请求1/PE21，连续供电流程实测生效。转向起始至段完成采样约24.3秒（含末尾停稳），在30秒会话上限内；5/3/1RPM减速真实出现，25/8度分界和1度到位容差不变。用户询问最后几度变慢，已按实际日志解释这是细调设计。平移段遥测峰值0.725/0.560/0.670度；末段固件高频峰值0.714度，不与100ms采样混称。整组90度峰值是转向起始目标差，不当作平移漂移。

576有效遥测/0无效，sequence_gaps484含启动前旧DAP积压跳变，不作为运行期484丢帧结论。ARM1/START1/KEEP246/STOP0，共248条；主控accepted247/rejected1，结束边界在途KEEP被终态拒绝，未导致重复启动或运动故障。RX余栈303字、最小堆9784字节仅本轮观测。程序已结束并释放COM，不自动再发车。当前板和源码为模式3、转向30秒保电版，已完成的一次会话；后续再测需新会话和摆位确认。GitHub未再次上传，本地新增证据和交接。

证据combined_turn30s_download.log、combined_four_stage_pass.raw/jsonl/summary.json、combined_four_stage_observation.json。完整组合/兼容构建及25项主机测试此前通过，本轮为独立实车证据。精确距离/XY、高速/载荷、系统故障注入与自动下载偶发连接失败原因仍未闭环。核R RUN-01/PWR-01、E LOC-03/RUN-06/MOT-01/02，官方Q不变；已上传回退点仍5f132ad。


## 2026-10-24最新：整组保电已实测生效；前进/左移成功，90度受旧20秒截止停止；30秒版本仅编译

用户指出不应每段断电并要求变通，现按整组调整。模式3首次ChassisAuto_BeginSequence只在健康闲置时取得整组许可，首段上电并完成一次原失能/使能/四轮准备。正常段结束保留VM和使能，仍发送零速、收齐停帧并核停稳；段间至少1秒，成功归属会话才可ReleaseCompleted，下一段跳到VERIFY_ON只复核新状态/零速，不重复失能/使能，不重新上电。最后一段前端明确请求VM0，等待PE2关闭才整组成功。故障/取消/失联/段间意外失去请求/健康均停止并断电，不清故障。原单段及无线模式保留原终态断电行为。

整组保电版四模式完整构建0错0警告：combo47964/664/1876/29284、single42748/664/1836/29156、link25862/462/1816/27392、manual39352/608/1768/27424；主机25项回归测试通过，新增段间VM1持续KEEP及最终VM0才完成。MCU逻辑不能凭主机测试宣布通过。用户确认重新摆好后实际命令行下载成功（15:27:56），先只读核空闲健康；明确发车提醒后一次组合启动。

实际前进→左移两段均正常完成，completed2、段间VM请求/PE2为1，未再出现换段拒绝或CAN0x20。随后按5/3/1RPM逆时针转向，误差持续下降，在原20秒会话上限到达时还差约3.724度，组合state4/cause5/index2停止并VM0、四轮反馈0，最后前进未执行。无CAN/RTOS/Safety故障、无链路失联，已触发既有限时保护，不作为整组通过。用户明确现场“都符合”：前进→左移→逆时针转动，段间VM灯亮、最后停稳并熄灭。具体误差和动作路径不等价精确XY定位。

依据该实测耗时，仅将APP_HEADING_RUN_MS改为模式3的30000ms，其他模式20000ms；这也同步影响模式3使用该宏的安全兜底，不伪称仅单个计时器变化。保留35秒单段/120秒整组上限、到位容差1度、停稳条件、停止2.5秒和后端独立兜底等保护，速度未提升。新配置combo/single完整构建均0错0警告，25项主机检查通过；30秒版本未下载/未实测，当前板仍20秒保电版，组合失败/可能复位状态不得假定空闲可再次启动。需重新摆回中央并恢复原朝向后再测试，不直接接着走。

证据combined_keep_vm_*_build.log及download.log、combined_keep_vm_turn_timeout.raw/jsonl/summary.json、combined_keep_vm_review.json，30秒版combined_turn30s_combo/single_build.log。数据统计含启动前DAP旧帧跳变，不把sequence_gaps436等同运行期丢包；整组峰值90度含转向初始目标误差，不当平移误差。新源码/交接仅本地未推送，已上传回退点5f132ad。核R RUN-01/PWR-01、E LOC-03/RUN-06/MOT-01/02，Q不变。


## 2026-10-24最新：组合第二次完成首段，第二段准备CAN故障；本次上电新状态就绪修复已编译未上板

用户再次要求尝试，5秒只读恢复实时55前版本51字段组合遥测（monitor_received，mode3/idle/selftests1/VM0）；发车前提醒后执行run-once一次。第一段前进正常停止，sequence_completed1，段间state2/VM0；下一段index1准备时触发g_safety_fault0x20（CAN状态数据保护），sequence state4/cause3，backend idle0/result0，轮速0、VM0，没有执行左移或转向。主机STOP无重试；首因保留修复生效，STOP没有把state4/cause3覆盖为取消。单段误差峰值约0.703度（遥测采样）、反馈峰值21/24/19/20，无失联和UART接收错误。随后只读先见故障旧缓存、再见tick回到新会话/故障0；其间出现复位，来源未确认，本机未发送复位/清故障，不把后来的故障0当原故障未发生。

源审查Safety_CanGuardUpdate发现：本次VM再次请求后，可将稳定期之前的有效状态计入s_can_was_ready，随后少一轮状态会立即触发0x20，而不是继续首次就绪等待；与本次准备时失败相符，但此前未遥测guard原因/异常轮位，无法确定具体节点或排除真实通信波动。本轮修复就绪判定只接受本次VM请求稳定期（现1秒）之后采集的四轮新鲜状态，原protect位仍按原fresh独立检查、3秒首次就绪截止/1秒新鲜度/故障锁存/停止/看门狗均保留，不清故障或放宽阈值。

组合遥测从51扩展至55字段，新增can_guard_cause/can_guard_bad_nodes/power_wait_state/post_settle_mask；主机兼容原48/51及55，24项主机协议/状态机测试通过。这些主机测试不验证MCU安全就绪逻辑；MCU仅源码审查/构建，修复还未烧录/实测。三个完整隔离构建日志combined_can_epoch_combo/single/link_build.log，均须核实际0错0警告后使用。当前源码模式3，板上仍上一版换段修复，当前运行复位来源待现场核对，不自动再次发车。

证据combined_second_run_can_not_ready.raw/jsonl/summary.json、combined_second_run_monitor、combined_can_epoch_review.json。组合仍未通过：只验证一次首段成功和合法断电间隔，第二段/转向90/最终前进未完成。下一次需用户先将车重新放回空地中央，再下载新就绪判定版，先监听健康新会话并明确发车提醒后验证。仅本地代码/证据/交接未推送，R RUN-01/PWR-01与E LOC-03/RUN-06/MOT-01/02核对，Q不变。


## 2026-10-24最新：换段修复版已下载校验，复位后无遥测，未重新发车

用户确认已重新摆好。修复版combined_handoff_fix第一次命令行下载Erase Failed/Cannot access Memory；只读仅得到旧取消态缓存，未发送控制字节。检查PC端DAP和COM17仍枚举OK，未发现活动UV4进程；单次重试下载返回Erase Done/Programming Done/Verify OK/Application running（15:12:24）。修复版此时已真实烧录，不能称未下载。

随后COM17连续只读3秒及8秒均opened=true、valid_frames0/invalid_frames0、sent_commands0；没有新遥测，无法确认主控正常运行或无线链路状态，未发送ARM/START、未重新运动。不把成功下载等同程序已正常调度，也不把PC枚举正常等同车端无线正常。已询问用户摆车后主控/DAP供电、SWD/串口接触和无线灯状态，等待物理信息；停止运动测试，不重复盲目烧录。下次先恢复只读健康新会话，再提醒并进行动作，不绕过保护。

证据combined_handoff_fix_download.log（失败）、combined_handoff_fix_download_retry.log（校验成功）和handoff_fix_after_flash_*采集。当前源码模式3组合，板上为换段修复版但运行未确认，整组仍未通过。仅本地记录未上传；核R RUN-01/PWR-01、E RUN-06/LOC-03，Q不变。


## 2026-10-24最新实测：组合首段正常停止后误拒换段，已修复未重试，需重摆起点

用户确认3米空地已准备，按发车前提醒执行组合。原组合只支持Watch，本轮接入既有RC1的ARM/START/KEEP/STOP与1秒失联保护，新增TM1组合51字段（原48+sequence_state/index/completed），主机mode3明确核组合方向，只有整组完成4段才退出，避免第一段完成误判全组成功。23项主机测试通过（含组合首段不退出、段间失联取消、51字段解码），组合47312/664/1856/29264及兼容单段42700/664/1836/29156完整构建0错0警告。命令行真实烧录Verify OK/Application running（15:09:33），只读核新会话健康再提醒发车，单次启动。

实测第一段前进后，backend DONE/result1/cause0、stopmask15、轮速0/VM0；组合phase4/cause4、index0/completed0，主机发送STOP并停止，没有继续左移或转向。正常底层DONE会设置g_chassis_auto_cut_vm=1作为断电请求（chassis_auto_test.c完成分支），组合完成检查原错误将该标志当故障，导致立即拒绝。这是组合判断实现错误，不是单段控制失败，不能签组合通过。主机后续STOP又将首因覆盖成取消；只读确认remote3/result3/轮速0/VM0，source正常，CAN/RTOS/Safety无故障。

已修复：正常完成检查核stateDONE/result1/cause0/停帧15/原会话归属，不再将正常cut_vm请求当故障；继续等待PE2实际控制关闭并间隔1秒，ReleaseCompleted仍要求s_vm_requested0/s_used0/cut_vm0、无故障，不绕过保护。STOP对已终态不覆盖首因或原trip，整组终态计时锁存。修复后47404/664/1860/29268完整构建0错0警告，23项主机测试通过；主机测试不覆盖MCU换段执行，实际修复尚未烧录/运行。当前板仍是失败后的旧组合固件取消态，未自动重试。

证据combined_remote_combo/single_build.log、combined_remote_download.log、combined_first_run_handoff_reject.raw/jsonl/summary.json、combined_first_run_stop_confirm、combined_handoff_fix_build.log/review.json。用户现场方向/摆动暂未确认；只完成一次前进，后续应先将车放回空地中央，再下载修复版并明确发车提醒后验证，防止重复前进压缩空间。90度和组合后续段仍未实测，当前源APP_TEST_MODE3、单段方向选择2（不决定组合私有顺序），接口与schema更新仅本地未提交推送。核R RUN-01/PWR-01、E LOC-03/RUN-06/MOT-01/02，Q不变。


## 2026-10-24最新实测：右横移串口单次验证通过，用户确认现场均符合

按用户要求发车前通知并观察，源码APP_TRANSLATION_HOLD_MODE由1改2，APP_TEST_MODE仍2。独立目录right_serial_run_20261024完整Rebuild42700/664/1836/29156，0错0警告，命令行DAP下载Verify OK/Application running（15:04:26），真实自动复位运行，无需用户Debug。随后仅监听核mode2及自检/源健康、零故障、VM0，再明确提醒右移15RPM/8秒，执行run-once(mode2)一次。

结束software_complete，elapsed8008ms，phase3/trip0、backend result1/cause0/active0、stopmask15，四轮反馈0，VM请求/PE2均0，CAN/RTOS/Safety故障0，失联0。用户回复“都符合”：向右横移、无明显摆动、自动停稳、VM灯自行熄灭。固件及100ms遥测峰值误差0.604度，完成记录+0.120度，纠偏峰值1RPM、非零提交样本12、最后-1，主机采到2个非零帧（不同采样粒度）。观测四轮RPM峰值20/19/19/20，最大源样本间隔11ms，RX余栈309字、最小堆9784字节。120有效帧、1无效、sequence_gaps52含启动前旧缓存到新帧跳变，不当作已证实运动期52丢帧或无丢帧；本轮无失联、拒绝和遥测丢弃计数。START一次、ARM一次、KEEP45次、STOP0，主控accepted47与47条主机控制帧一致，仅一次运动启动。

证据right_serial_run_build/download.log、right_serial_run_pass.raw/jsonl/summary.json及right_serial_run_observation.json。左右横移的本轮低速8秒方向/停稳/关VM均已现场确认；精确X/Y轨迹/距离、速度高反馈原因、故障注入、组合动作90度仍未验证。当前板是mode2右移已完成会话，不自动重复运行；下一步组合待用户明确安排，不擅自开始。原工程Objects/AXF未覆盖；本地代码/证据/交接更新未提交推送，GitHub回退点仍5f132ad。核R RUN-01/PWR-01、E LOC-03/RUN-06/MOT-01/02，Q不变。


## 2026-10-24最新实测：命令行编译/下载/自动复位运行已打通，无需手动Debug

用户说明下载完成芯片会自动复位，授权验证自动下载后运行。本机复制当前APP_TEST_MODE2/左移固件和原uvoptx CMSIS_AGDI配置至tmp/auto_flash_probe_20261024，未修改下载器设置；隔离Rebuild42700/664/1836/29156，0错0警告。随后UV4.exe -f <隔离工程> -o <下载日志> -j0实际下载，返回Erase Done.Programming Done.Verify OK.Application running，Flash Load finished15:02:48。用户无需进入Debug或按F5；已真实重写该板固件并运行，不能再称本轮“未烧录”。原工作工程Objects/AXF未覆盖。

下载后COM17只读3秒，sent_commands0，先读到旧完成态缓存，随后新会话seq106/tick10700，phase0/trip0/backend0/remote_state0/accepted0/source_valid1/selftest1/protocol_selftest1，VM请求/PE2/四轮RPM均0，CAN/RTOS/Safety故障0。新会话和连续遥测支持下载后程序确实运行，不只凭下载日志。默认空闲，不自行发车。本轮未进行运动或改变测试方向，板当前左移模式2、新空闲会话。

证据auto_flash_probe_build.log、auto_flash_probe_download.log、auto_flash_probe_monitor.summary.json/jsonl。后续可由Codex直接编译、下载、监听；每次实际发车前告知用户方向/速度/时长及需现场观察的动态，沿用用户已确认的准备条件，不无故要求用户手动Debug。Keil活动调试若占用DAP，须按实际错误处理，不能保证任何占用状态都能并行下载。旧串口缓存仍由主机时序门禁处理。仅本地交接/证据未提交推送；核R RUN-01/PWR-01、E LOC-03/RUN-06，Q不变。


## 2026-10-24最新实测：串口左横移一次完成，用户确认方向/停稳/VM均符合

用户复位运行后，执行COM17 run-once/mode1一次。先有旧remote_state3缓存，随后读到复位后的健康帧，主机时序观察窗口通过，ARM一次、START一次、KEEP45次、STOP0次，主控接受46次。运行elapsed8008ms，结束phase3/trip0，backend DONE/result1/cause0/active0，stop_mask15，四轮反馈0，VM请求0/PE2控制0，CAN/RTOS/Safety故障0，link_lost0，UART接收错误0。用户随后明确“都符合”：实际向左横移、无明显摆动、自动停稳、VM灯自行熄灭。该轮左横移与串口单次启动/保活/遥测正常完成可接受，不扩大到精确距离/轨迹、移动长距离无线可靠性或故障注入通过。

固件高频记录航向峰值0.521度，完成记录error=-0.335度；最新yaw1.966度、目标1.620度对应差+0.346度，两者来自不同更新时刻，不强行当同步误差。100ms遥测采样峰值0.395度小于固件峰值，应优先用固件高频峰值。固件修正非零提交样本10、最大1RPM、最后+1；主机只采到1帧非零，采样粒度不同，不能把10当10个独立动作或宣称已证明因果改善。观测四轮RPM峰值19/20/21/20，低于原30反馈限，高反馈原因仍未闭环。最大源序列采样间隔11ms，RX余栈309字、最小堆9784字节，仅本轮观测。

122有效遥测、1帧无效、统计seq gaps0；启动初仍可读到DAP旧缓存，不把全部122帧解释为本轮实时帧或无线零丢包证明。修复后这一次ARM/START成功，无重试启动。证据left_serial_run_pass.raw/jsonl/summary.json及left_serial_run_observation.json；数据保存本地未推送，代码未再修改或烧录。当前板仍APP_TEST_MODE2左移已完成的一次会话，不能直接重复启动；右移和组合90度仍未实测，后续按用户选择切换并重建下载/复位，不自动执行。R RUN-01/PWR-01、E LOC-03/RUN-06/MOT-01/02已核，Q不变。


## 2026-10-24最新实测：左移启动被旧遥测门禁拒绝，未发START，主机时序检查已修复

用户明确“开始左移”后实际执行run-once。收到旧缓冲seq210/tick21100起积压帧，ARM引用seq212/tick21300，随后当前帧seq526/tick52700报告remote_result5过期拒绝。发送总共2条，实际为ARM和STOP，没有START；运行phase0，四轮反馈0、VM请求/PE2为0，未进入本次运动。301序号跳变及同一主机时刻密集旧帧支持串口/DAP链路补传旧缓存判断，不将其归因运动控制故障。后续只读遥测确认remote_state3/result3/accepted1，主控已接受STOP，仍空闲VM0/轮速0。

主机新增Win32 PurgeComm仅清RX（不复位设备、不发命令），并要求至少3健康帧及150ms真实主机观察窗口，设备tick与主机时间推进偏差超过150ms则重置观察计数，防止旧帧突发被当就绪。20项主机回归测试通过，新增旧帧突发不ARM、旧到新跳变重建窗口。实测只读复查表明RX purge后DAP仍可补传旧帧，故不宣称单靠清缓存解决，真实节奏门禁仍必要。该软件修复尚未实际重试ARM验证，未修改固件，未再次发车。

本次串口已关闭，取消状态需用户复位并F5后开始新会话，无需重新烧录；不清故障/不自动重试。三次实测summary/JSONL保存同日left_start_*。下一步等待复位后再核新会话并执行原用户授权的一次左移，实际方向/停车仍待现场观察。仅本地主机代码/交接修改未推送，未更改原STM32保护阈值。核R RUN-01/PWR-01及E RUN-06/LOC-03，Q不变。


## 2026-10-24最新：无线收发通过后，源码切到串口左横移待下载

用户确认继续后，APP_TEST_MODE由1改2，APP_TRANSLATION_HOLD_MODE仍1，基础15RPM/8秒。隔离完整构建42700/664/1836/29156，0错0警告，证据left_serial_ready_rebuild.log。原工程AXF未重建，未代下载、未发串口发车指令；板上目前仍为用户已运行的LINK测试固件，不能把本次源码切换当已切板上模式。下一步用户Rebuild/Download/F5，再由电脑先monitor验证TM1健康状态，确认现场可左移后run-once一次。组合90度模式3仍未启用。原保护保留；仅本地修改未推送。


## 2026-10-24最新实测：COM17专用无线串口双向收发通过

用户先说明未下载最新程序，首次COM17打开成功但8秒无数据，sent0；随后按指导下载APP_TEST_MODE=1并回复“已运行”。本机实际执行test_wireless_uart.ps1 -Port COM17 -WaitSeconds 8，115200/8N1，收到专用LINK心跳，7条指令全部正确匹配，status=bidirectional_pass、opened=true、heartbeat=true、sent=matched=7，无I/O错误。随机串PING/PONG及ECHO匹配，SET569/GET569正确，INFO确认UART1/115200/8N1/VM0，SET1001被ERR,RANGE拒绝后GET仍569。证明本次PC下发→板端处理→回传双向链路可用，不是仅识别端口或本地回显。

全程只发无运动测试指令，无发车、横移或VM开启命令；VM0为专用测试模式声明，不是电压测量。串口已关闭释放。证据evidence/p1-foundation/2026-10-24/wireless_uart_bidirectional_pass.summary.json及.jsonl。无线物理路径/车端接线依用户现场准备，此回包试验未另行隔离有线/无线对照，不宣称长距离、移动中抗干扰、长期可靠性或串口控制运动已验收。下一步可切APP_TEST_MODE=2重建下载串口单次横移固件，再先监听健康数据、具备原运动条件后明确发起一次横移；本轮未切模式、未自动接续发车。组合动作/90度仍未实测。仅本地更新证据/交接，未提交推送。核R RUN-01/PWR-01、E LOC-03/RUN-06，Q不变。


## 2026-10-24最新：组合转向改为+90度，轮速仍分段5/3/1RPM

按用户要求组合第三段由+30度改为+90度，未改其他段15RPM/8秒；main只在APP_TEST_MODE=3允许+90度启动，HeadingSession仅模式3把幅度限45度放至90度，其他模式保留原范围及入口。默认仍APP_TEST_MODE=1，未启用组合。转向误差大于25度时各轮命令幅值5RPM，大于8度至25度为3RPM，余下为1RPM；进入1度容差后请求零速并沿用角速度/稳定及原后端停车核验。这是电机轮速，不是车体每秒转角，实际车体转速未测。转向原会话20秒上限保留，超时失败停整组，不能假定90度一定在期限内完成。

组合90度和默认无线两配置隔离完整构建均0错0警告：43780/608/1788/27524与25822/462/1816/27392，日志combined_combo90_rebuild.log、combined_link90_rebuild.log。未烧录、未实际旋转；本轮只编译与源码审查，实际90度耗时/误差待后续共同实测。GBK主文件与UTF8会话源分别保留原字节编码和换行。核R RUN-01/PWR-01及E RUN-06/MOT-01/02，Q不变。仅本地完成未上传。


## 2026-10-24最新：组合动作测试函数已写，默认关闭，未上板

用户要求预写前进/横移/转向组合测试，后续一起验证。本次新增App_CombinedTestStep，复用现有CAN任务周期调用，不增加任务，不在函数内等待；仅APP_TEST_MODE=3编译启用。当前APP_TEST_MODE仍1，无线收发专用配置及串口左移模式2、手动左移模式0保留。组合模式要求保留APP_HEADING_SESSION_TEST=1、APP_FORWARD_HOLD_TEST=1，测试不能随意改其他编译模式。

固定四段：前进8秒→正常停车关VM→至少间隔1秒→左移8秒→停车关VM→间隔→相对转向+30度→停车关VM→间隔→前进8秒→停车关VM。基础平移15RPM和原航向PD/反馈保护不变，转向用既有HeadingSession的角度闭环与速度限制。每次平移重新锁定当时车头方向，故最后前进沿转向后的车头；本测试是分段自动衔接，不是不停顿轨迹或同步平移旋转（E MOT-01/02后续待实现）。

组合仅通过g_combined_test.request=1启动一次，2取消；PC13也取消整组，包含段间间隔。state0空闲1执行2间隔3成功4失败5取消，index0至3，completed为成功段数。每段存phase/session/elapsed_ms/final_yaw/final_error，elapsed含准备/停车全过程。独立Safety任务监督取消、姿态/平台/故障及120秒整组上限；每段35秒上限，原单段VM准备/运行/停止截止仍保留。任一段失败/取消终止整组，不重试，不执行后续段。

为解决原单段每次上电只运行一次的限制，新增仅模式3可用ChassisAuto_ReleaseCompleted：只接受匹配会话、backend DONE/result1/cause0/stop_mask15、VM请求与PE2均关闭且无故障的正常成功结果；不调用ChassisAuto_Init，不清故障、不重置看门狗。成功结果先存到组合诊断，再释放下一段入口；平移前端仅在无trip且上一段成功后重建局部状态。等待CAN的单段Step在临界区之外，Safety可抢占取消。正常模式保持原一次启动限制。

四配置完整隔离构建均0错0警告：组合43756/608/1788/27524，无线25822/462/1816/27392，串口左移42700/664/1836/29156，手动左移39296/608/1768/27424。构建副本与实际修改四源码SHA/配置一致，GBK及既有换行保留，diff格式检查通过。证据combined_*_rebuild.log及combined_test_review.json；未做MCU运行仿真、烧录或实车动作衔接/取消/资源验收，原工程AXF未重建，无线/横移依然待实测。指导docs/P1_COMBINED_MOTION_TEST_2026-10-24.md。核R RUN-01/PWR-01、E LOC-03/RUN-06/MOT-01/02，官方Q不变。仅本地完成未提交推送，已上传回退点仍5f132ad。


## 2026-10-24最新：串口自动横移接入完成，单一开关切换，当前仍无线收发测试

用户要求无线收发通过后方便切换自动横移。复用已有CRC串口控制草稿与BeginTranslationHold，不另写运动闭环；新增统一APP_TEST_MODE：1为专用无线收发（当前默认），2为串口单次平移自动测试，0为原手动平移；APP_UART_LINK_TEST与APP_SERIAL_MOTION_CONTROL均由此派生，不再分别修改。APP_TRANSLATION_HOLD_MODE当前1左移，改2右移；每轮基础15RPM、8秒、修正±2、反馈保护30RPM及原停止/看门狗保持。

主机p1_serial_motion.py默认仅监听，预期方向改为1左移；明确run-once后等3帧健康数据、核对编译方向/速度、自检/姿态/故障/VM状态，ARM确认后仅START一次。主控继续执行原VM上电至少1秒及四轮状态就绪检查，完成准备后自动横移；STM32本地PD闭环，不由电脑逐帧算轮速。遥测100ms，主机KEEP200ms，主控1秒失联取消、主机600ms无新遥测尝试STOP；到时自动停车、停帧核对并关闭VM。不重试启动、不清故障。原无线模块测试task/function切换保留，不需要增加另一套任务。

主机自动保存raw、RX/TX JSONL和summary，每秒输出关键状态；默认监听或收到LINK测试协议均不发车。run-once的含义是本次明确请求自动执行一轮，不是固件上电自动发车。脚本只核模式，不能远程把左移改右移。四轮转速是缓存反馈，PE2是控制命令，软件完成不替代实车轨迹和停车观察。正式比赛入口与该开发测试接口须分离（R RUN-01）；R PWR-01、E LOC-03/RUN-06已核，官方Q不变。

最终四个完整隔离构建0错0警告：link25822/462/1816/27392；serial left/right42700/664/1836/29156；manual left39296/608/1768/27424。18项主机测试通过，含左右移等待准备/运行/完成、方向不符拒绝、LINK协议不触发启动、丢包/失联/取消及不重发；原19项无线主机协议检查通过。这些是主机软件模拟与编译证据，未执行MCU运行模拟、未烧录、未打开COM或发车；无线与横移仍待实测。原工程AXF保留。证据lateral_serial_*_rebuild.log和lateral_serial_review.json；操作见docs/P1_TEST_MODE_SWITCH_2026-10-24.md。当前本地完成未提交推送，已上传回退点仍5f132ad。


## 2026-10-24最新：无线收发独立测试入口，暂停串口发车

当前按用户要求先验证小模块，采取最小测试入口：简单模块优先由现有任务调用测试函数，需要独立周期/等待时才创建任务；测完关闭调用或编译开关，不固定要求每模块建任务。本轮已实现UartTest替代TestA（不重复创建业务任务），复用原UartRx进行PING/PONG、ECHO、SET/GET；UartTest每10ms解析HWT并上报真实APP任务进展，不运行扫码显示/运动门控。LED、按键、安全与CAN内部回环监督保留；CAN测试分支不进行外部电机交易，VM由已有Power/Safety测试分支强制关闭，看门狗不绕过。原自动发车草稿保留但APP_SERIAL_MOTION_CONTROL=0，不参与当前测试。

当前APP_UART_LINK_TEST=1，APP_SERIAL_MOTION_CONTROL=0；未启用的手动运动选择APP_TRANSLATION_HOLD_MODE=1（左横移），每轮基础15RPM、8秒及原保护不变。串口115200/8N1，每秒LINK,HB后追加LINK,YAW,<毫度>,<valid_mask>,<age_ms>；无效角度数值哨兵-2147483648，valid_mask及age必须一起检查，不以数据存在代表姿态健康。消息共用原RX任务发送入口，回复优先、忙时重试；原主机测试脚本忽略额外YAW行，仍可使用。专用测试不接受发车指令。测试结束仅改APP_UART_LINK_TEST=0，保持SERIAL_MOTION_CONTROL=0，重新编译下载即可恢复手动左横移流程；不要手改Watch切换测试模式。

隔离测试完整构建Code25822/RO462/RW1816/ZI27392，0错误0警告，日志uart_isolated_rebuild.log；恢复手动左移构建结果见uart_isolated_manual_rebuild.log。主机原19项协议自检通过，未打开COM、未发字节。固件运行/无线双向链路、横移方向和姿态纠偏仍未实际验证，未烧录，未覆盖原工程AXF。R核RUN-01/PWR-01、E核LOC-03/RUN-06，Q不变。本轮仅本地完成，未提交或推送；GitHub回退点仍5f132ad。后续先独立确认无线收发，再继续手动横移，不把两项同时改成自动发车。


## 2026-10-24最新：回退快照已上传，后续串口控制/自动遥测仅本地完成

按用户“先git以便回退”要求，已将94个经审查的累积源码/工程/文档/证据提交推送main，提交5f132adc946ff926bb1613216dc1e408d7ee3c32，上传后本地和远程哈希一致，后续再次ls-remote确认仍一致。未上传本轮Objects/Listings、本机调试布局和tmp缓存改动，磁盘文件保留。该提交是专用串口测试版（VM0），无线实际能力尚未由本机试验确认。快照之后的所有新串口控制代码未提交/未推送，不混称已上GitHub。

用户随后明确目前不能验证无线，先完成代码。新增APP_SERIAL_MOTION_CONTROL=1、UART_LINK_TEST=0，当前前进模式0、15RPM/8秒和原PD/30RPM反馈保护不变。TM1每100ms带CRC发送48字段运行快照；RC1只支持ARM/START/KEEP/STOP，500ms近期遥测引用+2秒解锁+一次START，200ms主机保活、1秒主控失联取消；序号/令牌核验，重复包不重新运行，不允许从串口写任意变量或清故障。默认打开COM只接收，不自动发车；STM32仍负责PD和原源/故障/停止/VM保护，串口不代替实时运动闭环。START交接窗口也支持PC13取消。纯手写Watch请求在串口模式不能绕过私有许可。

电脑脚本scripts/p1_serial_motion.py基于Windows原生COM、无需第三方包，默认monitor、显式run-once才两步启动，stop可取消；逐帧保存raw/jsonl及汇总，每秒输出关键变量。主机无新遥测600ms/握手超时/故障/会话变化会尝试STOP且不重试启动。四轮缓存反馈并非同步新样本，遥测峰值和主机完成不作为实际轨迹/停车证明。输入/输出有界，发送中断非阻塞、忙时丢帧计数，原看门狗仍监督真实任务进展。

最终四配置完整隔离构建均0错0警告：串口前进42700/664/1748/29244，串口左移42700/664/1836/29156，手动39296/608/1680/27512，专用UART测试31608/608/1724/27516。主机16项单元测试通过；固件只有编译与源码核对，MCU命令解析/截止、自检未运行模拟或上板。原工程AXF和板上固件未改，没通过串口发车。COM17当前不存在，只做监听预检sent0。无线收发/链路延迟/发车取消/资源和失效实测、横移与速度高反馈原因及X/Y未闭环。

新指导docs/P1_SERIAL_MOTION_CONTROL_2026-10-24.md、字段schema和同日serial_motion_build_review.json/四日志。保留5f132ad作为回退点，建议需要时独立worktree恢复源码，再重建/烧录；不强制覆盖当前修改。用户目前无需实际验证，本轮不要求立刻接线、编译或运动。核R RUN-01/PWR-01，E LOC-03/05/07、RUN-06、MOT-01/02/04，官方Q不变。快照已同步GitHub；新版本只在本地完成，未再次上传。


## 2026-10-24最新：无线串口双向专用测试代码完成，VM强制关闭，未烧录

用户当前明确要求先写串口收发测试，让Codex通过COM接收/下发测试数据。本轮不是恢复先前运动遥测或远程发车。新增APP_UART_LINK_TEST=1，保持既有运动模式参数但CAN路径跳过外部交易、按键不发运动请求；Safety撤销VM且Power_Enable编译期拒绝开PE2，任务真实进展和原看门狗/故障监督保留。固件USART1 PA9/TX、PA10/RX，115200/8N1；1秒LINK心跳，PING/PONG、ECHO、INFO以及仅独立测试数字的SET/GET，数字限0..1000；无运动或任意变量写接口。64字节发送缓存保留，有界输入/待回包和忙时重试，不阻塞任务。

电脑端新增scripts/test_wireless_uart.ps1：先确认专用VM0心跳，再以随机串和测试数字执行7个匹配事务，超时停止、不重发，DTR/RTS关闭，保存JSONL及汇总；ListenOnly只接收，SelfTest不打开COM。19项主机协议/命令构造检查通过。模式1完整隔离构建31608/RO608/RW1724/ZI27516；模式0恢复前进39296/608/1680/27512，均0错误0警告。开始前fetch确认main/origin main0/0，既有成果保留；三源码GBK/CRLF及构建SHA匹配，差异格式检查通过。未烧录、未实测硬件，原工程AXF未重建。

当前本机仅COM3/6/12，COM17不存在；脚本预检opened=false、sent0/matched0，无字节下发。下一步外部VM关闭，在实际工程Rebuild核31608并下载，连续Run，按DAP RX→PA9、TX→PA10及共地接线，插入电脑端DAP后确认实际COM号，再由Codex执行收发脚本。不凭编译或端口发现宣布无线链路通过。结束测试须宏改0并重建下载才能恢复运动；串口测试不签横移/PD/速度问题解决。

指导docs/P1_WIRELESS_UART_LINK_TEST_2026-10-24.md；证据evidence/p1-foundation/2026-10-24/uart_link_test_review.json及两个构建日志，实际端口预检日志在tmp/uart_link_captures。核R RUN-01/PWR-01，E LOC-03/RUN-06，官方Q不变。仅本地完成，未提交推送。


## 2026-10-24最新：15RPM左右横移航向保持代码完成，默认仍前进，未上板

用户确认最近一轮15RPM前进无明显摆动、自动停稳、VM灯自行熄灭。两张汇总图非零提交样本54/56、最大1RPM、最后+1，结束误差约0.643/0.626度，峰值0.736/0.780度，均phase3/trip0/运行8008ms；保留作当前前进开发基线，不宣称精确路径或PD因果改善。用户随后要求先写横移代码并明确每轮基础15RPM。

新增APP_TRANSLATION_HOLD_MODE（0前进/1左移/2右移），当前实际源码默认0以保留原入口；左右横移独立15RPM/8秒、纠偏±2、命令幅值13至17、反馈限30。按既有逆运动学生成横移向量，执行器起始锁存模式、每轮核归属，动态纠偏不能切换方向；方向检查按实际命令符号，纠偏不会重置运行计时或反向保护。原前进函数保留包装，原地转向模式保持可编译。复用原VM准备、源/提交100ms、10度偏角、9秒独立兜底、500ms停帧、稳定核验/故障锁存/看门狗，不接串口、不增加远程发车。

新增纯数学启动自检与g_translation_hold_selftest（0未运行/1通过/2失败），三方向五级修正对照既有逆解并核非法输入；只计算，不发CAN，自检未通过禁止平移。该函数目前仅编译，未运行仿真或上板执行。Watch沿用g_forward_hold并新增motion_mode，纠偏汇总保留。

开始前fetch确认main/origin main0/0，既有未提交成果保留。最终四个完整隔离构建均0错0警告：前进39296/608/1680/27512；左/右各39296/608/1768/27424；原转角39244/608/1680/27512。六个源码与最终构建SHA/配置差异一致，GBK/CRLF保留、源码格式检查通过。证据evidence/p1-foundation/2026-10-24/lateral_hold_build_review.json及四个日志；指导docs/P1_LATERAL_HEADING_HOLD_2026-10-24.md。原工程AXF及板上程序未改，未烧录或实测横移；只有本地代码完成，未提交推送。

下一步由后续实测任务明确左移/右移模式再重建下载，先核健康闲置、motion_mode和自检1，之后才按原完整装机/空间/VM条件安排一次测试；本次先写代码不安排动作。横移实际方向/PD/负载和异常停车、速度高反馈原因及X/Y仍未闭环。核R RUN-01/PWR-01；E LOC-03/05/07、RUN-06、MOT-01/02/04，官方Q不变。


## 2026-10-23最新：新增纠偏汇总，免逐条翻73条过程记录

用户表示过程记录过多难查看；新增g_forward_hold三个只观察字段：correction_nonzero_samples为已接受的非零纠偏提交样本数，correction_peak_abs_rpm为最大修正绝对值，correction_last_nonzero_rpm为最后一次非零修正（带方向）。只在新HWT帧且ChassisAuto_UpdateForwardHold接受后更新，停车/中止不清除，每次开机零初始化且现入口只运行一次。样本数不是CAN实际应用次数；非零只能证明提交过命令，不代表纠偏效果。现有旧运行无法追溯填充新字段，需要下一次使用新固件运行后才有值。

保留15RPM/8秒/反馈限30RPM及PD、VM、故障和停止保护；不接串口、不加远程发车。本轮仅改main.c的诊断字段/统计。开发前fetch确认main/origin main0/0；GBK/CRLF保留、源SHA与隔离构建匹配、源码差异格式检查通过。完整隔离Rebuild Code38572/RO608/RW1672/ZI27512，0错误0警告。证据同日correction_summary_rebuild.log及correction_summary_build_review.json。未烧录/实测、未改原工程AXF；仅本地完成，未提交推送。

下一步仅需原工程Rebuild核日志并下载新诊断版，再按既有健康门禁和一次手动request1运行，结束观察上述三个汇总即可，不逐条翻数组。若nonzero_samples=0则没提交非零修正；>0需结合误差和实车表现判断，不能仅据计数宣称闭环通过。此前用户截图新一轮phase3/trip0、result1/cause0、elapsed8008ms、结束误差约0.494度/峰值0.736度，但全部修正值未知，不将用户单独“1”当纠偏已触发。PD效果、稳定轮速/X/Y仍未闭环。核R RUN-01，E LOC-03、MOT-02/04；官方Q不变。


## 2026-10-23最新：按用户决定撤回串口半成品，恢复已实测手动版本

用户明确暂不接入串口调试、先手动调，随后同意撤回未完成串口代码。本轮仅撤回main.c的FH1遥测块/任务调用、app_config遥测宏及USART1发送缓存扩容；恢复64字节缓存。串口远程发车没有实现，也没有通过串口发车或操作硬件。主工程887个C/H/汇编文件已逐字节核对，与串口开发前forward15_limit30_build_20261023基线完全一致；保留此前所有供电/CAN/PD/诊断/证据修改，不使用git重置。三个新建串口采集/解析/测试工具移至tmp/serial_telemetry_shelved_20261023保留，不放在当前scripts入口。串口草稿、旧失败构建和临时采集保留作历史，不作为当前可用交付。

当前仍基础15RPM、运行8秒、PD修正±2RPM、命令13至17RPM、前进反馈限30RPM；原源/停止/VM/故障/看门狗保护均保留，默认不开VM，手动g_forward_hold.request1一次触发。用户最新确认本轮15RPM无明显摆动、自动停稳且VM灯自行熄灭；图phase3/trip0、elapsed8008ms、final_valid1、结束误差0.532836914度、峰值0.741577148度、新帧观察最大间隔11ms、73条过程记录。仅基本流程及用户观察通过；全程PD是否参与、稳定轮速及此前高反馈原因未闭环，不宣称精确路径或高速恢复性能。证据forward15_limit30_user_motion_result.json及可用原图。

开始前fetch确认main/origin main0/0，原有未提交成果保留。恢复版隔离完整Rebuild Code38512/RO608/RW1672/ZI27496，0错误0警告，源码差异格式检查通过；详见同日manual_restore_rebuild.log、manual_restore_review.json。原工程AXF和板上程序均未改，无需为本次恢复再次烧录。下一步继续Keil Watch手动调试，串口开发暂停；X/Y、PD恢复能力和稳定轮速/速度解析仍待后续。核R RUN-01/PWR-01，E LOC-03、MOT-02/04；官方Q不变。仅本地完成，未提交推送。


## 2026-10-23最新：用户要求将前进反馈超速阈值独立提高到30RPM

15RPM首次测试未正常完成：phase4/trip5、后端result2/cause8、前端elapsed418ms、stop_mask[2]=0、VM请求0；四轮保留反馈21/-24/18/-28RPM，均超过原17RPM阈值且方向符号符合原映射。此次是异常中止，不能记录为正常停车或8秒完成；实际速度高于15RPM命令的原因（瞬态/驱动/单位解析等）尚未定位。用户明确要求提高超速阈值，本轮新增APP_FORWARD_HOLD_FEEDBACK_LIMIT_RPM=30，只在前进保持模式使用；目标15RPM、纠偏±2、命令最大17RPM、8秒、原转向限值及源/停止/VM/故障/看门狗保护均不变。阈值与命令上限分开并增加编译期一致性检查，不将调高阈值描述为解决速度异常。

开发前fetch确认main/origin main0/0，既有修改保留。实际两源码GBK/CRLF保留、与隔离构建SHA一致，差异格式检查通过。隔离完整Rebuild Code38512/RO608/RW1672/ZI27496，0错误0警告；本次未新增运行时或硬件验证、未烧录、未重建原工程AXF。证据evidence/p1-foundation/2026-10-23/forward15_limit30_build_review.json及对应日志。

下一步外部VM关闭退出Debug，在实际工程Rebuild核对新配置后再下载/健康闲置观察；保留本次失败证据，不清故障伪造通过。新阈值下运动停车和反馈速度问题未闭环，X/Y及PD恢复能力仍未闭环。核R RUN-01/PWR-01，E MOT-02/04；官方Q不变。仅本地完成，未提交推送。


## 2026-10-23最新：按用户要求准备15RPM/8秒直接加速测试

用户确认此前5RPM运行约8008ms，向前、无明显摆动、自动停稳且VM灯自行熄灭；Watch phase3/trip0、结束误差约0.417度、峰值0.461度、最大新帧间隔11ms、73条过程记录。用户后续表示过程无±1/±2纠偏，不将这次当PD恢复能力验收。结果存forward5_user_motion_result.json。用户明确不做小角度目标阶跃，改为直接加速，最终指定15RPM；8RPM仅是被替代的中间候选，未上板测试。

当前APP_FORWARD_HOLD_BASE_RPM=15，运行8秒，PD Kp/Kd、±2RPM限幅、滤波/斜率/迟滞不变；各轮逻辑13至17RPM，统一BASE+2命令上限。前进执行器反馈超速检测同步为绝对速度>17RPM，只在forward_hold_mode生效，其他模式原7RPM/静态转向10RPM门限不改。其余VM门禁、100ms源/提交、10度偏角、9秒运行兜底、500ms停帧、停止稳定/故障锁存/看门狗均保留。提高速度并不自动证明PD足以应对更大偏航。

开发前fetch确认main/origin main0/0，保留既有修改。最终隔离完整构建前进38512/RO608/RW1672/ZI27496，原转角回归39080/608/1672/27496，均0错误0警告。8个源与最终前进构建SHA一致、编码/CRLF保留，源码差异检查通过。证据evidence/p1-foundation/2026-10-23/forward15_build_review.json和两个forward15日志。未烧录、未实测15RPM、未重建原AXF，未提交推送。

下一步外部VM关闭退出Debug，实际工程Rebuild核日志后下载观察健康闲置，再在完整装机载物/机构固定及足够行程空间条件下仅一次request1，检查前进、摆动、自动停稳和VM灯及响应记录。15RPM/8秒名义行程约为前轮5RPM的3倍，不以HWT推断精准位移。PD动态恢复、提高速度后的制动/失效、X/Y未闭环。详见当前前进说明；核R RUN-01/PWR-01，E LOC-03/05/07、RUN-06、MOT-01/02/04，官方Q不变。仅本地完成。


## 2026-10-23最新：用户指定5RPM，前进纠偏改为受限PD，待实测

本轮开始fetch确认main/origin main0/0，既有修改全部保留。用户指出短时慢速固定修正不足以判断动态响应并授权改动，随后明确基础速度5RPM。当前APP_FORWARD_HOLD_RUN_MS=8000；PD Kp0.8/Kd0.25、角速度低通80ms、修正限±2RPM、连续参考限10RPM/s、整数命令迟滞，四轮逻辑3至7RPM，不放宽原>7RPM反馈保护。源码沿用HWT yaw/wz同向，参数仅工程候选，未证明正常或更高速度效果。基础启动沿用原执行器，停车与故障中止不受纠偏斜率拖延。

保留100ms源/新帧续约和提交保护、10度偏角中止、VM门禁/看门狗/故障锁存及500ms停帧兜底；独立运行时限随8秒观察期改为9秒。新增新帧间隔/误差与角速度峰值和最多96条约100ms响应记录，记录为提交值不是实际CAN落地或物理轮速。旧正反30度控制参数不变，不追加旧转角重复试验。

最终隔离完整构建：前进38496/RO608/RW1672/ZI27496，旧转角回归39064/608/1672/27496，均0错0警告。8个本次源SHA与前进构建一致、CRLF及逐文件编码保留，源码差异检查通过；无运行时仿真、烧录或实车操作。实际工程AXF未更新。下一步外部VM关闭退出Debug，在实际工程Rebuild核对日志，再下载验证健康闲置；之后才进行5RPM/8秒受控前进响应观察。动态效果、参数调校、异常停机实测及X/Y未闭环，不承诺更快速度可用或精准距离。

详细配置和Watch见工创赛/docs/P1_FORWARD_HEADING_HOLD_2026-10-23.md；本轮证据evidence/p1-foundation/2026-10-23/forward_pd_build_review.json及两个PD日志。先前同日forward_hold_*是旧版记录。核R RUN-01/PWR-01；E LOC-03/05/07、RUN-06、MOT-01/02/04，官方Q不变。本轮只在本地完成，未提交推送。


## 2026-10-23最新：低速前进与HWT航向保持已本地实现，待板上验证

用户同意转入下一项后，新增APP_FORWARD_HOLD_TEST=1（SESSION仍1，其余测试0）：3RPM前进、锁存HWT目标、±1度死区及±1RPM纠偏，四轮逻辑速度2至4RPM且不反向。四轮首次整组启动完成后5秒请求停车，复用原四轮准备/状态速度反馈/停止核验与VM关断。只用新HWT帧续约，保留100ms源和提交保护、偏航超过10度中止、独立6秒执行器运行兜底及500ms停止帧保护，PC13/request2取消，默认不开VM、不自动运行。旧正反30度功能及用户认可结果保留，FORWARD_HOLD设0恢复原入口；不追加旧转角重复试验。

本轮开始已fetch确认main与origin/main均dc5207f、ahead/behind0/0；保留全部既有未提交源码、证据、Keil设置及构建产物。ARMCC5隔离完整Rebuild：前进37532/RO588/RW1664/ZI25920，原转向回归39064/588/1664/25920，均0错误0警告。8个本次源码SHA256与前进构建一致且CRLF保留；源码差异检查通过。全仓diff检查还包含既有构建产物空白，不将其当本次源码问题或擅自清理。没有运行时仿真、烧录或实车操作，实际工程AXF未重建，禁止把旧AXF当本功能下载。

下一步外部VM关闭、退出Debug，在实际工程Rebuild并核日志，再下载验证健康闲置；板上使用g_forward_hold作为新入口，旧heading请求不再触发当前模式。后续受控前进仅request1一次，核向前/纠偏/自动停稳/VM灯及终态。前进方向和纠偏、异常失效实车验证、X/Y仍未闭环；不承诺精准距离，不把PE2软件命令当实测VM电压。详见工创赛/docs/P1_FORWARD_HEADING_HOLD_2026-10-23.md及evidence/p1-foundation/2026-10-23/forward_hold_build_review.json和两个日志。核R RUN-01/PWR-01；E LOC-03/05/07、RUN-06、MOT-01/02/04，官方Q不变。仅本地完成，未提交或推送GitHub。


## 2026-10-22用户决定：保留当前转角方案，结束本轮重复性测试

用户已确认本轮-30同样反向转动后自动停稳、VM灯自行熄灭；该实物确认补齐先前heading_tolerance1_negative30_result.json的待确认项。随后用户明确“不用测了很可以了这个下一步”，因此不再安排原地转角重复性测试，保持当前1度会话候选及5/3/1RPM/700ms稳定、30度入口和全部保护。接受本阶段正反向单次软件及用户观察证据用于继续开发，不追加宣称统计重复性、独立物理精度或正式整车安全验收。

当前建议的下一开发项为低速平移时HWT航向保持：接现有vx/vy/wz逆运动学，让车前进或横移时保持锁存朝向，利用航向误差生成受限旋转修正，保留反馈和异常停机。现有数学/编码接口已具备，当前实时航向执行器仅有原地转向接口，须设计平移执行和安全状态机，不能直接把现有HeadingCommands当完整平移控制。建议尚未实施，本轮未改固件。

HWT只能提供航向，不能单独给出平移距离或X/Y；本阶段不把定时平移当精确距离，后续实际导航仍需冻结并接入X/Y来源。用户本轮不用编码器估算航向的决定保留，不能偷换为所有定位传感器已不需要。下一步不直接宣称可到指定坐标或跑整场路线。

本次只更新本地交接/当前计划，未编译下载或操作硬件、未提交推送。R RUN-01/PWR-01；E LOC-03/05/07、RUN-06、MOT-01/02/04已核，官方Q不变。


> 2026-10-22电控组1.3复测未通过：用户报VM短亮后关闭、不能判断转动。图phase4/trip6、backend cause1/io8/address1，trace result8/phase3/length5/rx1/约1ms/ESR0；实际代码表明准备控制收到非02应答后失败，尚未到速度阶段。原始request和last_rx折叠，不直接沿用上次E2结论。当前唯一下一项松开外部VM开关、保留失败不复位不重发，展开g_can_ctrl_trace_request与g_can_ctrl_trace_last_rx（含Data）截图；暂不放大角度。证据evidence/p1-foundation/2026-10-22/heading_positive5_failed.json及两PNG。核R RUN-01/PWR-01，E LOC-03/05/07、RUN-06、MOT-02/04；根因、真实转角停车/X/Y及官方Q仍未闭环，未改实际固件。

> 2026-10-21用户表示可满足完整装机载物地面/断电机构收拢固定条件。当前唯一下一项按这些条件摆好，当前37760会话固件无需改码/下载，VM外部开关按下保持，连续Run预热≥3秒且源/闲置门禁满足后仅relative+5/exec_request1一次，不手改VM；观察约18秒，异常外部松开VM，不Stop替代停车、不重复请求，实际停稳/失败后保留Watch1/2与首因。证据heading_after_static_ready_for_positive5.json；试验结果待回传，原E2/转角停车/X/Y及官方Q仍未闭环。

> 2026-10-21恢复会话闲置截图通过：源1/帧龄17与8ms，VM/两fault/phase/trip/request/首次锁存均0，CAN guard0/valid0为VM0正常等待；证据heading_restore_after_F3_idle_pass.json/png。当前唯一下一项复测前确认完整装机载物地面条件及断电机构运输姿态可靠固定、不下滑/伸出/拖地；机构VM前序仍断开，未确认前不开VM/不发heading请求、不擅自恢复机构供电。之后才按一次+5度受控复测取证；原E2/运动停车/X/Y及官方Q未闭环。

> 2026-10-21航向模式恢复Rebuild37760/588/1604/25492、0错0警告/3秒，actual main/can/config SHA与此前已核诊断会话一致。当前唯一下一项VM外部松开，下载复位、F5连续Run约5秒，不按PC13或写请求，截图源valid/两帧龄及VM/fault与phase/trip/request/首次锁存，预期默认健康闲置。机构VM继续按静态隔离状态，本步不动，运动前须核完整装机载物/运输姿态。证据heading_restore_after_static_F3_actual_rebuild_pass.json；原E2/会话转角停车/X/Y及官方Q未闭环。

> 2026-10-21稳定VM下1号失能通过：result1/F3 02 6B，poll flags02/03/03/03，首次锁存0、ESR0、约1ms。证据can_disable_ID1_stable_VM_pass.json/png。当前唯一下一项结束供电VM0/松开开关，退出Debug仅恢复APP_HEADING_SESSION_TEST=1，其余KEY/STATIC/TURN0，完整Rebuild回传，本轮不下载/发请求。实际main/can未变，恢复配置哈希与此前已核会话一致，不重复新增测试。原E2在该静态条件未复现，不能唯一认定原根因；会话+5度/转角停车/X/Y及官方Q未闭环。

> 2026-10-21接轮VM22V/灯持续亮用户确认，结合valid0F/四flags03/guard ready1/两fault0，本次静态供电局部通过。当前唯一下一项按P1_DISABLE_ID1_AFTER_STABLE_VM_2026-10-21.md，仅通用模式稳定VM下发1号失能一次，读取result/原F3应答及flags[0]使能位；仍不发速度/不恢复SESSION，完成后VM0/松开开关。证据vm_four_wheel_22V_steady_user_pass.json；原E2与真实失能/运动/X/Y和官方Q未闭环，源码不改。

> 2026-10-21四轮静态截图：VM请求1、两fault0、Safety/CAN guard ready1、guard cause0、valid0F、四flags03。软件门禁/四轮状态在该时刻通过，证据vm_four_wheel_static_guard_ready.png/json；03不是底盘位姿到位。当前唯一下一项结束本次供电（Run中VM请求0并松开开关），补报刚才VM实测电压及灯是否持续亮，不重发请求/不试转动。持续供电实物结果、原F3 E2、运动/X/Y及官方Q未闭环。

> 2026-10-21用户明确22V测点为VM输出，短时空载开启通过。当前唯一下一项四宏全0固件不改，全断电接回四轮原供电/CAN插头，机构继续隔离，健康默认状态后一次VM请求1做约5秒静态供电/3A轮询观察，记录VM电压/持续灯和CAN guard/poll及故障；不发动作请求/不按PC13，结束请求0/松开VM，异常松开且不重试。证据vm_output22V_point_confirmed_user.json；空载通路局部通过，带轮持续供电/F3 E2/运动/X/Y及官方Q仍待闭环。

> 2026-10-21用户更正：之前静态0V时忘按VM开关，按下重做后灯短亮、测得22V。此前已要求换测VM_EN，当前唯一下一项先确认这22V的红表测点究竟是VM输出还是VM_EN，不猜；开关松开、负载继续断开，不再请求/接回。VM输出22V可支持短时空载开启通过，VM_EN22V则与3.3V允许预期不同须先核点/线路。更正证据vm_static_request1_corrected22V_user.json，先前静态0V错误条件保留历史，不将其当功率级故障证据；R/Q不变，原E2/带载/运动/X/Y待验。

> 2026-10-21静态请求1仍实测VM0V、灯未亮，证据vm_static_request1_measured0_user.json。当前唯一下一项将表移到电源板VM_EN（H2第6脚/R13控制线一端），在全部VM负载断开/复位恢复五项健康默认状态后，一次受监督request1读最初1～2秒，再request0/松开开关；不Stop维持供电、不改3秒CAN保护/代码，不猜脚序。先区分允许信号未到达与下游功率级，尚不能认定Q2坏或E2唯一低压原因；实际VM_EN/VM根因及运动/X/Y/Q未闭环。

> 2026-10-21用户实测request0时VM0V、确认全部VM负载断开。当前唯一下一项在通用模式健康默认状态下，一次受监督短时request1，先摆好GND-VM电压表并按下VM开关，读最初1～2秒后请求0/松开开关；不Stop延长供电/不发动作请求。缺CAN节点约3秒保护及后续IWDG复位可能发生，不放宽或绕过保护、不凭晚读0V判断从未开启，不反复请求。证据vm_static_request0_measured0_user.json和vm_all_loads_disconnected_user.json；开启VM/E2/运动/X/Y与官方Q未闭环。

> 2026-10-21板上通用模式闲置通过：VM0/RTOS0/Safety fault0/ready1/CAN test1，旧heading变量移除符合配置。当前唯一下一项全断电含USB、隔离VM输出所接电机/机构供电插头，先确认接口不重新上电/发请求。后续静态测量需保留缺CAN节点VM请求后3秒保护，不放宽/强写GPIO。证据vm_static_mode_idle_pass.png/json；实际VM/E2/运动/X/Y及官方Q未闭环。

> 2026-10-21通用模式Rebuild38976/588/1496/25368、0错0警告/3秒通过，actual四宏全0、config与候选字节一致、main/can SHA未变。当前唯一下一项VM外部关闭下载复位，连续Run约5秒不写请求/不按PC13，截图VM请求/RTOS故障/Safety故障/Safety ready/CAN test，预期0/0/0/1/1；本轮不开VM，后续再安排负载隔离及静态电压检查。证据vm_static_mode_actual_rebuild_pass.json；VM/E2/运动/X/Y与官方Q未闭环。

> 2026-10-21用户确认试验开关一直按下，不能归因松开档。当前唯一下一项按P1_VM_STATIC_MODE_PREPARATION_2026-10-21.md仅手改实际app_config.h的SESSION1→0（KEY/STATIC/TURN保持0），完整Rebuild后发日志，暂不下载/发请求；原SESSION闲置/失败主动清VM，先临时返回原Safety监督下通用模式，再安排静态VM测量。实际main/can候选ARMCC无诊断、真实源未改，证据vm_static_mode_preparation.json。通用模式仍有手动动作接口，不把它当纯电源固件；不强写PE2、不删除保护。VM_EN/VM、E2根因与运动/X/Y及官方Q未闭环。

> 2026-10-21VM排查更新：用户Q2输入侧22V；断电实测开关按下1-2通、松开2-3通，按现有图确认为按下允许PE2控制、松开强制关闭VM。当前唯一下一项确认此前+5度试验时开关是否持续处于按下档，不重发请求；不能未确认便认定开关为唯一E2根因。证据can_F3_E2_upstream_22V_user.json及vm_switch_pressed12_released23_user.json。实际VM/保护与转角停车/X/Y未闭环，R/官方Q不变。

> 2026-10-21首次诊断实测：1号F3 AB 00 00 6B在约1ms内收到匹配F3 E2 6B，result8/ESR0、PE2前后1，exec phase4/trip6、backend cause1/io8，用户VM灯未亮且车未转。本次非超时；厂家手册43页E2为参数/范围或条件不满足，低压等只是候选。当前停止重复运动，保留失败并关闭外部VM开关，先拍左后ID1电机当前显示/错误、不改菜单清故障，再据证据安排无运动VM诊断；PE2不代替电压实测。不修改等待/保护、不接受E2为成功。证据can_trace_F3_E2_failure.json/png，实际原因/转角停车/X/Y与官方Q未闭环。

> 2026-10-21电控组1.2接续：实际can.c诊断修改已完成，与已验证候选代码一致；本机最新完整构建37760/588/1604/25492、0错0警告/3秒。无需重复手改/编译。当前唯一下一项按P1_CAN_CONTROL_TRACE_CAPTURE_2026-10-21.md下载诊断固件并一次+5度取证，记录首次控制失败原帧/PE2/时长；不得强开VM、清锁存或扩大保护。证据can_control_trace_actual_build_verified.json；仅只读核对已有构建，未操作硬件，CAN/VM根因、转角停车和X/Y及官方Q仍未闭环。

> 2026-10-21+5度再次未通过：用户VM没亮，源有效/15与6ms，两fault0；exec phase4/trip3/state4/cause2，backend result2/cause1/io5/last_address1，旋转mask0。定位为1号失能准备命令在当前窗口未取得匹配回复，不再归因HWT或仅凭灯判电源坏。当前唯一下一项按P1_CAN_FIRST_CONTROL_FAILURE_TRACE_2026-10-21.md仅手改实际can.c：加诊断变量、整段替换ExchangeSingle和ControlSingle，记录首次控制失败的命令/原始RX/阶段时长/PE2 ODR/ESR，失败后冻结不被轮询覆盖；不改帧/等待/供电/100ms保护。GCC模拟和ARMCC完整can候选通过，实际源未改。用户完整Rebuild后发日志，本轮不下载或再试运动。证据heading_after50Hz_can1_prepare_timeout.json、can_control_trace_preparation.json；实际VM/CAN根因与运动/X/Y未验，R/Q不变。

> 2026-10-21恢复会话模式后30秒闲置观察通过：用户明确未看到valid跳0，图valid1、角度/角速度帧年龄17/18ms，VM/RTOS/Safety fault与三种接收错误均0，phase/trip/request0、yaw2.65869141/rate0；CAN余栈325字/1300字节、最小堆11320。仅有限Live Watch观察，不签全部最大间隔/动态资源。下一项原完整装机载物地面条件下一次相对+5度复测，无需改码/下载/重设50Hz；打开VM外部开关，先F5连续Run≥3秒恢复新鲜源，再Run中request1一次，relative5.0，观察约18秒，实际停止/失败后Stop记录Watch1/2及VM/动作/停稳现象。异常外部关VM，不用Stop替代停车/不反复请求。Watch2保留phase/trip/request并加入目标/误差/状态/后端结果字段。证据heading_after50Hz_idle30s_user_pass.json，R/Q与运动/X/Y边界不变。

> 2026-10-21恢复会话模式实际Rebuild通过：37208/588/1540/25476、0错0警告/3秒，实际四宏和main/hwt与已核候选一致，默认heading/rate50请求0。下一项保持VM外部关闭下载复位，连续F5先运行约5秒完成初始化/自动版本读取，再观察30秒，不写任何请求；Watch1保持13项，Watch2仅加exec_phase/exec_trip/exec_request。期望预热后source_valid保持1、两帧龄期<100ms，VM/RTOS/Safety故障及三项exec均0，接收错误不新增。初始化前几秒source_valid0不作为本观察失败。观察后Stop截图并报告是否看到跳0；一次截图不证明全部间隔上界，尚未放行转动。证据heading_mode_after50Hz_actual_rebuild_pass.json；R/Q与实物运动/X/Y边界不变。

> 2026-10-21下一项恢复目标航向会话模式：用户仅在实际USER/app_config.h设置KEY0/STATIC0/TURN0/SESSION1，其余代码及115200/100ms/CAN20/30保持；退出Debug、VM外部关闭后手改并完整Rebuild，本轮不下载/不设置请求。当前main与新增HWT服务按新模式隔离ARMCC编译无诊断，实际文件SHA未变，证据heading_mode_restore_after50Hz_preparation.json。原Watch13项先保留。HWT50Hz已设置/测频并按用户断电步骤读回8，无需重复设置；构建核对后先做VM关闭的新模式源状态观察，再决定+5度复测，不能把平均50Hz当最大间隔/物理运动通过。R RUN-01/PWR-01，E LOC-03/05/07、RUN-06、MOT-02/04，官方Q/X/Y等边界不变。

> 2026-10-21用户按断电保持步骤后读回RRATE8：result1/raw8/count1、帧55 5F 08 00 06 00 00 00 00 00 C2校验正确，设置服务active/result/state/command/raw/request均0，VM/两fault0、源有效/年龄13/14ms。按用户执行该断电流程接受本次参数保持，不把截图当独立断电测量。证据hwt_rate50_powercycle_readback8.json。用户要求清理Watch，当前先按P1_WATCH_CURRENT_MINIMAL_2026-10-21.md只留13项健康/源/航向/错误/资源，删除所有重复及已完成RRATE/50Hz/版本/帧计数和暂不用会话项；只删Watch行，不删源码/不改值。固件不改/不重设速率。后续恢复SESSION模式先核100ms源连续性再受控+5度，实际运动/停车/X/Y与Q仍待闭环。

> 2026-10-21实际平均接收约50Hz通过：两图同组上方计数1166→2022、tick58950→76080ms，856/17.13≈49.97Hz；同组下方约49.97～50.03Hz，重复Live Watch行不同步不混配。checksum0/overflow0、UART错误1均无增量。仅验证区间平均频率，不签最大间隔/100ms源连续性。下一项保持VM外部关闭，对主控与HWT真实断电（含可能USB回供，不只RST），等约5秒再按原方式上电，沿用当前固件，不发rate50设置；Run≥5秒版本1/state0时仅rrate_request1一次，等约2秒Stop，截图RRATE result/raw/reply_count/原帧和VM/fault，预期raw8，另确认rate50_request/result未自行设置。证据hwt_rate50_measured_frequency.json；断电保持/运动停车/X/Y与官方Q仍待闭环。

> 2026-10-21固定50Hz流程已板上读回8：用户图active0/result1/state0/command3/raw8，帧55 5F 08 00 06 00 00 00 00 00 C2校验正确，VM及两fault0；接受本次设置流程与即时读回，原rrate_raw6为旧快照。两张图angle/rate_count均0x404=1028、update_tick均0xDB56=56150ms，不能据此计算频率。下一项保持VM外部关闭、不再request设置/运动、无需改码复位，F5连续Run中两次相隔约10秒截图angle_count/rate_count/checksum_errors/uart_errors/overflows/update_tick；用实际tick差估频，预期约50Hz但尚未验证。断电保持、最大间隔/100ms源连续性及目标转角停车/X/Y随后另验，官方Q不变。证据hwt_rate50_readback8_user.png/json。

> 2026-10-21固定50Hz服务实际Rebuild通过：40296/600/1524/25452、0错0警告/3秒，实际hwt.c/h与候选token完全相同、RRATE声明已补齐、main及STATIC1/SESSION0/TURN0/KEY1保持。下一项VM外部开关保持关闭，下载复位后连续Run≥5秒，确认版本1/state0/VM与两fault0；Run中先rrate_request1读取，看到result1/raw6后再等2秒，仅此条件下rate50_request1一次，连续Run约5秒后Stop截图rate50 active/result/state/command/raw及展开reply，VM/fault。若原读值非6或读取失败则不触发设置、先保留截图。成功候选active0/result1/raw8，原rrate_raw6仍是旧快照；频率/最大间隔/断电保持另验，不开启运动。证据hwt_rate50_actual_rebuild_pass.json，R/Q与物理闭环/X/Y边界不变。

> 2026-10-21RRATE板上读取通过：result1/raw6/reply_count1，帧55 5F 06 00 06 00 00 00 00 00 C0校验正确、VM及两fault0，和约10Hz实测相符。已取得厂家HWT101专用协议及厂商编写的同名协议PDF解锁/保存说明；只采用69/03/00/27四帧固定序列，不使用76（归零）。下一项按P1_HWT_RATE50_MANUAL_2026-10-21.md，用户仅改hwt101.c/h，补只读/50Hz声明、新静态一次性50Hz服务、两处互斥/IRQ owner3/更新调用，完整Rebuild；STATIC1等配置/main不变，本轮不下载或发请求。新服务默认不自动写，只有已读6/静态VM0/事务互斥等满足后受理，保存并读回8仍须真实频率和断电验证。GCC模拟及ARMCC完整hwt候选无诊断，实际源未改。证据hwt_rrate6_read_user_pass.json、hwt_rate50_preparation.json，R/Q及物理闭环/X/Y边界不变。

> 2026-10-21RRATE只读实际Rebuild通过：39292/580/1476/25444、0错0警告/3秒，实际hwt101.c与已测候选token一致，STATIC1/SESSION0/TURN0/KEY1、main未变。hwt101.h尚缺3行RRATE extern，对本文件内定义及当前Watch读取无影响，记为后续跨模块接入前补齐，不为此重复构建。下一项VM外部开关保持关闭，下载静态固件并复位，F5连续Run约5秒，version_result1/state0及VM/两fault0时，在Run中rrate_request1一次，再连续Run约2秒后Stop，截图RRATE result/raw/reply_count及展开11字节reply，连同version及VM/fault。禁止请求运动/校准/A7或手改结果。raw6与10Hz相符但5F无地址回显，只算候选读取证据，不提前写50Hz。证据hwt_rrate_read_actual_rebuild_pass.json；R/Q及物理闭环/X/Y边界不变。

> 2026-10-21板内RRATE只读下一步已准备：按P1_HWT_RRATE_READ_MANUAL_2026-10-21.md，用户将配置切STATIC1/SESSION0/TURN0/KEY1，只在实际hwt101.h加声明、hwt101.c加RRATE变量并整段替换Hwt_VersionUpdate，再完整Rebuild；main/硬件/CAN不动，本轮不下载。只发27读取03，不解锁/写入/保存；RRATE独立结果、静态VM0、事务互斥/1500ms5F隔离及有限截止，不污染版本/A7。GCC模拟及ARMCC hwt候选/main静态候选无诊断，实际源未改。5F无地址回显，result1仍为候选回复，后续需原帧+实测核对；未写50Hz。证据hwt_rrate_read_preparation.json，R/Q与物理闭环/X/Y边界不变。

> 2026-10-21用户补充HWT101焊在主控板且未引出IO：取消前轮USB-TTL接线/接口照片前置，不拆焊、不飞线、不改成品PCB。当前改走已有USART2 PD5 TX/PD6 RX的STM32配置路径；此前版本5F回复成功证明过该双向通道，不要求用户接第二路TX。下一项准备用户手动加入受限速率读取/设置流程，保持VM外部关闭与静态互斥，先核对RRATE再按匹配协议设置50Hz工程候选并读回/测频，不能把发送成功当参数生效或自动写未知解锁/保存。此轮仅确认路径，尚未新增配置代码、编译或发串口；100ms保护保持，实物配置/运动/X/Y及官方Q未闭环。

> 2026-10-21用户已确认有USB转TTL但未连接：采用上位机配置路径，暂不写STM32设置代码。当前唯一下一项先断整车电源/关VM，取得USB-TTL正反面针序/电平跳帽和HWT101当前端子照片，核实接口和实际TTL电平后再逐线接线，不猜针序、不并联STM32与适配器两个TX、不混供电。50Hz仍是待设置/读回/帧率验证的工程候选，100ms门槛不变，实物配置/运动和官方Q未闭环。

> 2026-10-21计数取证完成：角度/角速度各从593到744增加151，HWT更新59360→74460ms，约10Hz；校验0/溢出0、UART错误1均未增加。结合首次角速度100ms拒绝，确认当前平均周期与100ms源边界无余量。厂家HWT101页面默认10Hz可调参数仅作旁证，不代替当前RRATE读回；50Hz为工程候选。下一项保持VM关闭/不运动，确认用户现有维特上位机连接或仅STM32路径，再选择型号匹配的速率配置方式；不猜解锁/保存、不改100ms安全期限或波特率。详见P1_HWT_RATE10HZ_AND_NEXT_2026-10-21.md；原图/计算hwt_measured_rate10Hz.json。实际配置/时效与物理闭环/X/Y仍待验，R/Q不变。

> 2026-10-21首次源触发证据已取得：用户观察valid在0/1变化；首次fault_phase1、flags0x10、sample_age10ms、angle_age99ms、rate_age100ms、HWT_update_age10ms、tick0x9524=38180ms，backend0，确认等待阶段角速度帧到达100ms拒绝边界，非该时刻HWT任务长时间未更新。其下层原因（设备输出频率/无线延迟/缺帧/串口错误）尚未确认，不直接把它写成10Hz或硬件故障。当前唯一下一项保持VM外部开关关闭、不再发运动请求，连续Run中两次相隔约10秒记录HWT angle_count/rate_count/checksum_errors/uart_errors/overflows/update_tick；期间不Stop/复位/改参数，先估算实际接收频率并看错误增量。不改100ms门槛/固件，不重复运动。证据heading_source_first_rate100_user.png/json；R RUN-01/PWR-01，E LOC-03/05/07、RUN-06、MOT-02/04，官方Q不变。

> 2026-10-21首次失败快照实际Rebuild通过：36000/564/1472/25456、0错0警告/3秒，实际main与已测诊断候选token一致，CAN20/30候选及模式未变。下一项下载诊断固件并复位，在既定完整装机载物地面条件下只复现一次+5度请求：外部VM关闭时下载，打开开关后连续Run≥3秒，确认phase0/trip0/源有效，在Run中request1一次、relative5.0；观察约18秒、实际停止或失败后Stop，不重复请求/不复位抹掉首次快照，不手改VM。回传exec_trip及fault_phase/source_flags/sample_age/angle_age/rate_age/hwt_update_age/tick，连同实际VM/动作现象。若发生动作异常立即外部关VM，不用Debug暂停替代停车。只为获取首次触发证据，未宣称修复source trip2；R RUN-01/PWR-01，E LOC-03/05/07、RUN-06、MOT-02/04与官方Q边界不变。证据heading_first_fault_actual_rebuild_pass.json。

> 2026-10-21期限修正后用户仍报告VM未亮/车未转；最新图改为phase4/trip2、session state0/cause0/backend_session0及后端全0，说明本次在执行器会话建立前源判定失效，不能沿用旧trip3归因。后续source_valid1、年龄38/49ms不代表触发时状态。当前下一项按[首次源失败快照手动指南](P1_HEADING_SOURCE_FIRST_FAULT_MANUAL_2026-10-21.md)，只替换实际main的App_HeadingRuntimeTrip完整函数为含7个诊断变量的新样本，首次记录源条件位/各龄期/阶段/时刻；不改判定或100ms门槛/CAN20/30/VM动作。实际controller/session联动mock及首次原因锁存/恢复不覆盖/WAIT与RUN/配置位用例通过，候选main ARMCC无诊断，真实源未改。用户完整Rebuild，先不下载或重复运动。启动前VM0V为默认预期，触发后是否曾实际供电未测。证据heading_after_canfix_source_trip2.json、heading_first_fault_preparation.json；R/Q不变，根因/物理闭环/X/Y待验。

> 2026-10-21CAN期限位置已修正，实际Rebuild通过：35744/564/1444/25452、0错0警告/3秒，实际can.c与已测候选token完全一致，回环TX100、新SESSION交易TX20/RX30、旧交易100/200、提交监督100ms、main与模式开关均已核对。下一项一次+5度受控复测：VM外部开关关闭时下载并复位新固件，完整装机载物地面条件；打开VM外部开关后连续Run≥3秒，在Run状态只写exec_request1一次，relative5.0，不手改VM，持续约18秒，停稳或已失败后Stop保留现有Watch与实际动作/VM现象。仍有异常立即外部关VM，不重写请求。若失败优先读auto_cause/io_result/last_address与exec_trip，不默认修复成功。证据heading_can_deadline_actual_rebuild_pass.json；R RUN-01/PWR-01，E LOC-03/05/07、RUN-06、MOT-02/04，实物VM/转角/停车/X/Y及Q仍待闭环。

> 2026-10-21CAN期限手改Rebuild虽0错0警告/3秒，但实际TX宏改在回环函数：can.c第152行CAN1_LoopbackTest需恢复pdMS_TO_TICKS(100U)，第269行CAN1_ExchangeSingle才应使用pdMS_TO_TICKS(CAN1_SINGLE_TX_WAIT_MS)。RX宏及其他改动正确。当前唯一下一项用户修正这两处并完整Rebuild，先不下载；修正隔离副本与已测CAN候选token一致、实际配置ARMCC无诊断，真实源未改。证据heading_can_deadline_placement_review.json；实车故障仍未复测，R/Q不变。

> 2026-10-21后端失败截图：会话编号1，state10/result2/cause3/io_result0/start_mask2为0，exec_cause2，确认接受过执行器会话但未记录成功旋转启动。源码发现确定时限冲突：唯一CAN任务产生提交，却可等待TX100ms/RX200ms，超过独立提交100ms监督；原IO可能尚未写回便被监督先作废。该机制与截图一致，尚未唯一锁定实机首次原因。下一项按[CAN期限协调手动指南](P1_HEADING_CAN_DEADLINE_FIX_MANUAL_2026-10-21.md)仅改实际SYSTEM/CAN/can.c：包含app_config、SESSION专用TX20/RX30宏，替换ExchangeSingle两处参数；旧模式100/200、帧/速率/所有100ms安全门禁保持。实际Exchange函数主机新旧模式/回绕/边界及ARMCC完整can模块候选编译通过，实际源未改。用户手改后完整Rebuild，本轮不下载/不复测运动。原实车测试未通过；源/调度、物理VM与目标角/停车/X/Y仍待闭环，R/Q不变。

> 2026-10-21首次+5度未通过：用户报告VM没亮，截图phase4/trip3/state4/request0/rpm0/VM0、两fault0、source_valid1、两帧龄期0x61/62=97/98ms、CAN最小余栈245字=980字节/最小堆11328。实际main仍与已测候选一致。trip3复用于多个平台/互斥检查，截图不能锁定首次失败位置，也不能把最终源龄期当触发时龄期或认定电源损坏。当前唯一下一项保留失败状态，不复位/重写request/强开VM；读取exec_cause/backend_session及chassis_auto_state/result/cause/io_result/start_mask[2]截图，先区分后端执行失败和入口条件，再决定针对修复。证据heading_positive5_vm_dark_trip3.png/json。实车+5度未验收，R/Q不变。

> 2026-10-21新模式默认Watch通过：用户截图phase/request/trip/state/rpm/VM请求/两fault均0、source_valid1、角度/角速度年龄6/7ms、CAN最小余栈0x145=325字=1300字节、最小堆0x2C40=11328字节。只覆盖闲置路径和该时刻，不称物理VM/动态资源或闭环通过。下一项按[首次+5度受控试验](P1_HEADING_FIRST_POSITIVE5_TRIAL_2026-10-21.md)：保持既定完整装机载物地面条件，VM开关可关闭；打开外部VM开关后连续Run≥3秒，在Run状态只写exec_request1一次，relative保持5.0，持续约18秒，记录实际逆时针小幅转动/自动停稳及终态。不要暂停排队运动/转动中Stop/同次重复请求或扩大角度。助手未操作硬件，实车结果仍待回传。证据heading_runtime_idle_user_pass.png/json；R RUN-01/PWR-01，E LOC-03/05/07、RUN-06、MOT-02/04，官方Q不变。

> 2026-10-21真实会话接入整工程Rebuild通过：35744/564/1444/25452、0错0警告/3秒，实际main与已测候选token一致，SESSION1/STATIC0/TURN0/KEY0及ReadStatus=s_mode已核对。代码较旧静态预览缩小符合模式切换移除未引用代码的范围，不要求重复构建。下一项VM外部开关关闭、车不动，下载本次新模式，连续Run约3秒再Stop；不按PC13、不设置任何请求，记录exec_phase/request/trip/state/rpm、VM请求、两fault、source_valid、两帧龄期及CAN余栈/最小堆。预期exec闲置/无请求/无trip/VM0/fault0，源有效且两帧年龄<100ms。只是新模式默认不启动检查，不放行转动；未测物理VM、连续时效/失效响应、装机载物±5度/停车与X/Y。证据heading_runtime_actual_rebuild_pass.json；R RUN-01/PWR-01，E LOC-03/05/07、RUN-06、MOT-02/04，官方Q仍待技术答疑确认。

> 2026-10-21会话构建报HEADING_SOURCE_FRESH_MS未定义（2错0警告）：新附件与实际main一致，两处外层条件已修正，但源快照外层if之后直接typedef，遗漏1kHz检查和100ms宏。下一项仅在该typedef之前恢复`#if configTICK_RATE_HZ != 1000U`/`#error Heading_source_requires_1000Hz_tick`/`#endif`/`#define HEADING_SOURCE_FRESH_MS 100U`四行；保持外层if不动，其余源码和配置不动，再完整Rebuild，不下载。恢复后与此前已测main的token完全一致，实际配置ARMCC隔离编译无诊断，实际固件未改；证据heading_runtime_missing_define_review.json。真实构建/实机仍待闭环，R/Q不变。

> 2026-10-21会话接入粘贴审阅：两附件SHA相同，与实际保存main的代码token一致；仅相邻两处外层编译条件与已测候选不同。用户将g_heading_bus变量块扩展到新模式，却把真实源快照块保留为旧条件。当前唯一下一项：main第718行旧邮箱块恢复STATIC条件、第726行源快照块扩展SESSION或STATIC条件，详见接入指南顶部；其余代码/config保持，完整Rebuild后发日志，不下载。隔离修正后与已测候选token完全一致，实际配置ARMCC编译无诊断，真实main哈希未变。证据heading_runtime_paste_review.json。整工程链接/默认VM0/实物闭环/X/Y尚待验，R/Q不变。

> 2026-10-21模式字段修正实际Rebuild通过：38908/576/1456/25432、0错0警告/2秒，实际ReadStatus已取s_mode，不重复该项。下一项按[目标航向真实任务接入指南](P1_HEADING_RUNTIME_BINDING_MANUAL_2026-10-21.md)由用户修改实际main/app_config：独立SESSION模式、源快照跨模式发布、CAN唯一会话/动作及归属检查、旧手动交易隔离、Key只取消、独立Safety监督；默认request0/VM0，首轮目标只允许±5度。隔离新模式和旧静态main均ARMCC无诊断编译，实际controller/session链接mock执行器的联调用例通过，实际源码SHA前后未变。用户手改后完整Rebuild，本轮不下载/不设置请求。软件mock不签RTOS/CAN/物理闭环，实机默认VM0、失效响应、装机载物±5度/停车与X/Y仍待验；R RUN-01/PWR-01，E LOC-03/05/07、RUN-06、MOT-02/04，官方Q不变。

> 2026-10-21执行器快照接口用户完整Rebuild通过：38908/576/1456/25432、0错0警告/3秒。实际接口与上一版样本一致，但复核发现助手样本将dry_run误读g_chassis_auto_dry_run请求参数；Begin实际锁存并供执行使用的是s_mode。下一项仅由用户在ChassisAuto_ReadStatus中改为`status->dry_run = s_mode;`，保存并完整Rebuild，其他源码/main/静态配置不动、本轮不下载。更正样本已通过请求与锁存值相反的主机用例，实际模块隔离候选ARMCC编译无诊断；旧mock未覆盖该不一致，不作正确性证据。详见执行器状态快照指南和heading_backend_snapshot_mode_correction.json。真实会话/动作/Safety及物理闭环/X/Y未完成，R/Q不变。

> 2026-10-21真实源静态Watch通过：用户截图valid1、sequence0x2B8=696、发布/角度/角速度年龄1/4/5ms、yaw0.0384521484度、rate0、VM请求及两fault0。仅接受本观察时刻的数据交接，不签连续时效/断流或物理VM。下一项按[执行器状态快照手动指南](P1_HEADING_BACKEND_SNAPSHOT_MANUAL_2026-10-21.md)只在实际chassis_auto_test.h/c增加ChassisAutoStatus和ReadStatus，在同一临界区复制会话号/航向模式/演练标志/active/result/旋转stop_mask等，用户完整Rebuild；main与静态配置保持，本轮不下载。GCC字段/空指针/旋转段索引/mock临界区检查通过，不代替板上并发或物理停止。源截图/JSON和backend准备检查已归档；会话/动作/Safety绑定、真实运动停车及X/Y未闭环。R RUN-01/PWR-01，E LOC-03/05、RUN-06、MOT-02/04，官方Q仍待技术答疑确认。

> 2026-10-21真实航向源快照实际Rebuild通过：38908/576/1456/25432、0错0警告/3秒。只读实际外部工程确认快照代码与已测样本去注释/空白后一致，TaskA在HWT更新后唯一发布、CAN任务唯一观察，静态模式1/旧限时转向0。下一项关闭VM外部开关后下载静态固件，保持车不动，连续Run约3秒再Stop，记录source_valid/sequence/三个age/yaw/rate及VM请求、RTOS/Safety fault；不设置启动/转动/校准请求。一次暂停快照只证明该时刻，不证明连续100ms时效或真实会话/运动/停车/X/Y通过。证据heading_source_snapshot_rebuild_pass.json；R RUN-01/PWR-01，E LOC-03/05、RUN-06、MOT-02/04，官方Q保持待技术答疑确认。

> 2026-10-21新聊天接续：只读实际外部工程和Objects构建日志，确认heading_session.c已编译、38380/576/1424/25408、0错0警告，main尚未调用会话。下一项先按[真实航向源快照手动指南](P1_HEADING_SOURCE_SNAPSHOT_MANUAL_2026-10-21.md)在实际main增加完整源快照发布及CAN任务静态观察；用户自行修改、完整Rebuild后回传日志，本轮不下载。新快照分开保存发布/角度帧/角速度帧时间与发布序号，以100ms严格边界拒绝过期，不沿用原静态预览500ms作为行车许可。GCC软件用例及实际插入位置检查通过，实际源码SHA未变；未执行ARMCC/整工程构建或板上验证。源快照尚未接会话/执行器/Safety，真实运动/停车/X/Y未闭环。R RUN-01/PWR-01，E LOC-03/05、RUN-06、MOT-02/04；官方Q保持待技术答疑确认。

> 2026-10-21会话模块实际Rebuild通过：38380/576/1424/25408、0错0警告/3秒，日志含heading_session.c。实际USER/heading_session.h/c去除注释空白后与已测样本一致，uvprojx引用实际USER文件，main尚无HeadingSession调用；程序大小未变符合未引用函数被移除的范围，不据此要求重做构建。本项完成。下一项为会话与真实任务/源快照及执行动作的绑定，需保证输入真实健康/新鲜且序号明确、运行入口互斥、停止动作反馈和独立Safety监督。当前静态配置不改、本轮不下载；真实会话整合/运动/物理停车/X/Y仍待验，R/Q不变。

> 2026-10-21目标航向会话模块已准备：[手动加入会话模块指南](P1_HEADING_SESSION_MODULE_MANUAL_2026-10-21.md)。用户在实际外部工程USER新建heading_session.h/c，复制docs/code_samples对应样本，c加入USER组后完整Rebuild；main/静态配置不改、本轮不下载。模块区分临时SETTLING零速与最终停车，只有停止请求接受且执行器成功/非活动/mask0F才DONE；重复/过期/回退源、9秒未到位、停车2.5秒未确认、请求拒绝/取消及反向等均安全失败锁存。实际controller链接的GCC用例及ARMCC样本编译通过。此为纯会话判定层，尚未绑定真实任务/硬件，首轮候选±5度且源健康/互斥/Safety整合与物理闭环仍未放行，R/Q不变。

> 2026-10-21独立提交超时接口实际Rebuild通过：38380/576/1424/25408、0错0警告/3秒。实际源只读核对提交时间的Init/Begin清零、BeginHeading首次设定、Update合法提交更新及Auto_CheckLocked独立监督；去除注释/空白后与此前通过mock/ARMCC的候选一致。不再重复此接口构建。下一项为真实目标航向受控会话整合：源sequence/HWT健康绑定、互斥调试入口、到位/SETTLING/取消/故障区别、超时协调及停车反馈/VM兜底。当前仍静态模式，不下载未接入的接口、不运动；提交超时的实机效果、真实闭环/物理停车/X/Y待验，R/Q不变。

> 2026-10-21会话整合第一步：[独立指令提交超时手动指南](P1_HEADING_COMMAND_WATCHDOG_MANUAL_2026-10-21.md)已准备。用户只改实际外部工程chassis_auto_test.c：新增合法提交时间，在Init/Begin清零、BeginHeading/Update合法提交时更新，由现有Auto_CheckLocked的Safety路径检查新会话100ms未提交则故障关VM；正常停车改由原截止/反馈接管。mock状态机回归及真实宏头文件ARMCC候选编译通过；用户手改后完整Rebuild，main/h/静态配置保持、本轮不下载。此步不是完整会话；源sequence/HWT绑定、故障日志/整合、超时协调及实物闭环仍待完成，官方Q不变。

> 2026-10-21动态轮速接口用户修正后已完整Rebuild：38316/576/1420/25404、0错0警告/3秒。实际外部工程只读确认direction_age声明、Init单组清零、Begin新会话清零均正确，s_run_tick=now仅在首次START分支内，不再有无条件重复行；h中UpdateHeading声明已存在。此前两份相同粘贴代码中的4处问题已修正，不重复要求该步。下一项为目标航向会话与唯一CAN任务的受控整合，必须包含HWT/源指令新鲜度与独立Safety失效监督、正常到位与临时SETTLING停转的区别、故障/取消锁存与停车反馈/VM兜底；旧20秒控制器超时与执行器10秒上界尚须明确协调，同会话反向仍拒绝。不应仅把新接口串到静态预览便放行运动；当前保持静态模式，无需下载未接入的接口。本次未改实际固件或操纵硬件，物理闭环/停车/X/Y待验，R/Q不变。

> 2026-10-21单次航向入口实际构建通过：38044/576/1400/25408、0错0警告/4秒，实际源Init清零和BeginHeading已核。下一项按[整组动态更新手动指南](P1_HEADING_DYNAMIC_UPDATE_MANUAL_2026-10-21.md)，只在实际外部工程h/c加会话隔离、pending整组更新、方向反馈检查起点及UpdateHeading后完整Rebuild；main不接新接口，不下载、静态模式保持。mock状态机回归及实际宏/头文件ARMCC候选编译通过；同向更新/零速允许，停车优先，同会话反向/越界暂故障拒绝。10秒上界未放宽。源时效/会话整合、反向纠偏和真实动态运动/停车/X/Y仍待验；MOT-02加速度轨迹未完成，R/Q不变。

> 2026-10-21正常停车接口实际Rebuild已通过：38028/576/1400/25408、0错0警告/5秒；实际源已核。唯一下一项按[单次旋转入口手动指南](P1_HEADING_SINGLE_SESSION_MANUAL_2026-10-21.md)，在实际外部工程补Init缺少的两行finish状态清零，添加BeginHeading声明/实现后完整Rebuild；main/静态配置保持、本轮不下载。新入口复用原四轮准备、只选旋转段并使用传入正负轮速，原10秒上界保留。隔离mock及ARMCC候选编译通过，不签动态更新/真实闭环或物理停车通过；动态更新及会话整合仍是后续未完成工作，R/Q不变。

> 2026-10-21最新跨任务交接及正常停车准备：用户实际Rebuild37952/576/1392/25408、0错0警告，截图确认发布1/bus_valid1/rpm5/stop0，取消后state5/cause4及bus_rpm0/stop1。下一项按[正常停车接口手动指南](P1_HEADING_ORDERLY_STOP_MANUAL_2026-10-21.md)在实际外部工程chassis_auto_test.h/c加入RequestStop及首次截止、结束序列标志，用户完整Rebuild；当前不改main/不下载，静态模式保留。候选实际状态机mock-CAN检查及真实头文件/宏ARMCC隔离编译通过；实际动态调速、目标角闭环、物理停车/X/Y仍待验。接口完成只表示停车流程结束，不代表目标角成功，R/Q不变。

> 2026-10-21实际工程路径纠正：用户当前Keil为`D:/新建文件夹4/工创赛/代码/stm32/3.stm32f407工程/gongxun-f407.uvprojx`，本机只读确认实际Publish/Read已保存且构建37688/564/1372/25404、0错0警告/5秒。此前项目内副本缺少接口是副本差异，不回退用户进度、不自动同步覆盖。唯一下一项按[任务间静态预览接入](P1_HEADING_MAILBOX_TASK_PREVIEW_MANUAL_2026-10-21.md)，用户只改实际main：TaskA发布、CAN任务读快照到观察变量，再完整Rebuild；保持静态模式、暂不下载。数据交接不代表许可或物理停车，真实动态执行/X/Y仍待验，R/Q不变。

> 2026-10-21最新：用户实际Rebuild37688/564/1352/25408、0错0警告；真实输入到四轮预览已截图确认正向方向全1/速度5、取消后全0速、反向方向全0/速度5，encode_result1，不重复该组检查。下一项按[CAN指令交接接口手动指南](P1_HEADING_CAN_MAILBOX_MANUAL_2026-10-21.md)，用户在chassis_auto_test.h/c加Publish/Read数据接口后完整Rebuild；不改main、不下载、不接CAN。隔离1ms/10ms tick数据及过期检查、ARMCC候选编译通过，真实调度和执行层尚未整合。R RUN-01/PWR-01、E LOC-03/05、RUN-06、MOT-02/04已检查，官方Q保持待技术答疑确认，实际闭环/停车/X/Y未验。

> 2026-10-20执行层第一小步已准备：[四轮指令转换手动指南](P1_HEADING_MOTOR_ADAPTER_MANUAL_2026-10-20.md)。用户在chassis_kinematics.h/c加入Chassis_HeadingCommands后完整Rebuild，当前不改main、不接CAN、保持静态模式且无需下载。复用实际0/1/0/1方向表，±5RPM内构造纯旋转，越界拒绝并清零。隔离GCC实际C运行和ARMCC候选编译通过；真实工程由用户手改，实际运动仍未放行。

> 2026-10-20最新板上进度：虚拟角度自检及真实HWT静态预览的0度到位、+45/-45度计算方向、取消、连续运行超时均已由用户截图确认。真实预览构建37488/564/1348/25388、0错0警告。最新超时state4/cause3/command0/stop1，VM请求及两fault0；此前cause2保留为独立失效记录，不冒充超时通过。下一项准备目标航向执行层手动接入指南及隔离检查，不重复静态用例，不直接放行电机；实际固件仍由用户手改。详见[P1静态预览结果](P1_HEADING_LIVE_PREVIEW_RESULTS_2026-10-20.md)。真实闭环、停止和实时X/Y仍待验，R/Q不变。

> 2026-10-20用户明确更正：此后代码和Keil工程由用户自己修改，助手给文件/位置/代码/解释/验证，不再直接改实际工程。用户已修ErrorDeg分别归一化并Rebuild通过：35604/452/1332/25284，0错0警告/4秒；只读源核对正确。最新唯一下一项用户按P1_HEADING_DRY_SELFTEST_MANUAL_2026-10-20.md第1～4节手动切现有静态模式并在main加一次性虚拟角度自检，然后Rebuild日志。指南同段代码隔离main/控制器ARMCC通过，虚拟result1/case0/正5,3,1,0,0/反-5,-3,-1,0,0/跨界2度，当前控制器58项边界检查通过；实际源/工程未改。板上自检及真实目标角闭环仍待验，不把旧53项或旧构建当成当前物理通过。

> 2026-10-20最新：远程origin/main仍为158e3df，无队友新提交。当前真实进度按10月19日交接：四轮受限运动、双向HWT航向随动与自动停VM已过，不回退到10月15日控制接口准备。已新增纯计算`heading_controller.c/h`并加入Keil工程，未接main/CAN/VM；5/3/1RPM分段、±180°跨界、连续稳定、HWT失效、超时/取消共53项主机检查通过，完整Rebuild 0错0警告，尺寸仍35604/452/1332/25284。当前唯一下一项是建立与旧限时转向互斥的目标航向干运行模式，复用现有准备/停止/取消/VM兜底，先离线验证+45°/-45°及跨界，再完整装机载物低速实测；实时X/Y仍未闭环。

> 2026-10-19后续开发增加[决赛现场调试交付](FINALS_FIELD_DEBUG_PLAN_2026-10-19.md)：集中底盘/定位/路线/雷达参数与中文说明，读回有效值和版本，分模块自检/有界测试、停止/故障诊断与稳定版恢复。沿现有USART1与唯一CAN服务逐步接入，实际调试协议尚待设计，不以当前Watch变量当正式现场入口。保留下面控制接口手动修改/Rebuild这一唯一下一项，不自动改源或放行运动。

> **2026-10-18最新：按用户明确要求假定兼容，先做受限A6试验**：不再把版本确认当本轮前置。hwt101增加显式request1静止确认后1s+前60s记录+A6 01采10s+A6 04退出+静置1s+后60s记录；VM持续0，PC13不运动。只试A6，不改A7/归零/保存/死区/出厂参数。失败时尽力有界退出，发送完成与模块接受/漂移改善分开。2109项模拟检查通过，Keil Code33896/RO448/RW1208/ZI25272、0错0警告。下一项烧写后放稳，version_result1/valid_mask3时cal_request1，F5持续约2分15秒，不碰车不暂停，再报告cal_result/before_deg/after_deg。兼容仍是用户试验假设，不写成厂商确认；实物试验/效果待验。详见P1_HWT101_A6_STATIC_TRIAL_2026-10-18.md。R RUN-01/PWR-01、E LOC-03/05/RUN-06/MOT-04，Q不变。

> **2026-10-18版本回复实测成功，兼容性仍待核对**：用户新版截图result1/tx5/valid_mask3，展开帧55 5F E2 27 0A 00 00 00 00 00 C7，校验正确，接受连续版本请求及双向串口本次交易通过。公开HWT101协议将2E定义16位版本，按此字段为0x27E2=10210；程序raw0x000A27E2只是前四payload字节拼接，不能据此解读为新文档1003X.1.8或宣布A6/A7支持。用户新截图为另一32位版本布局，需维特上位机的明确设备版本或厂商对应说明确认；不凭数值大小判新旧，不发送尚未确认的校准/配置命令。证据hwt101_version_reply_user_pass.json/png，保留静态VM0。R RUN-01/PWR-01、E LOC-03/05/RUN-06，官方Q不变，实际版本格式/校准兼容属工程待核。

> **2026-10-18最新：版本超时，改连续发送后复测**：用户截图tx_bytes5/version_result3/raw0/reply全0、valid_mask3，证明字节已写USART数据寄存器而角度接收正常，尚不能证明PD5物理发送线、模块接受或固件不支持。审查发现原TXE任务轮询可能使帧内字节间隔达10ms；已改为USART2 TXE中断连续发五字节、与RX同IRQ分别服务，保留200ms发送/TC和1000ms回复截止，UART错误立即停剩余发送；不重置早到回复基线。2072项模拟检查通过，Keil Code32388/RO448/RW1112/ZI25272、0错0警告。此发送改进排除一个软件变量，不宣称确定是原实物超时根因。下一项下载后F5运行约3秒，再看version_result/raw/reply；仍超时再结合PD5→HWT RX物理路径排查，保持VM0，不写校准配置。

> **2026-10-18最新：HWT101先读实际固件版本**：用户补齐HWT101协议/A6/A7与AGV版本门槛截图，明确开始实施。已启用USART2 PD5 TX并增加一次上电只读版本FF AA 27 2E 00及校验5F回复，2062项模拟检查通过，Keil Code32220/RO448/RW1108/ZI25276、0错0警告。保持APP_HWT_STATIC_TEST1、VM关闭、PC13不运动；未发送A6/A7/归零/保存。下一项烧写后F5运行约3秒，看version_result/raw及展开reply，确认模块RX实际接PD5；拿到实物版本后才选择兼容校准流程，不把示例版本当实物。详见P1_HWT101_VERSION_READ_2026-10-18.md。HWT已由用户验证角度有效与正反符号/转回，不重复验。R RUN-01/PWR-01、E LOC-01/02/03/05/07、RUN-06/MOT-04核对，Q不变。

> **2026-10-18最新：HWT101只读静态验收固件已构建**：用户确认HWT101型号及已装接线，新增USART2 PD6 Rx/115200只读接收和0x52/53校验解析，不写模块参数；2036项模拟检查通过，Keil Code31616/RO440/RW1060/ZI25244、0错0警告。当前APP_HWT_STATIC_TEST=1强制VM请求0、忽略PC13启动，覆盖此前按键即可运动步骤。下一项有线Watch看rx_bytes/angle_count/valid_mask/yaw_deg，再VM0手转车核角度变化；实物接线/波特率/实际数据和方向均待证据。此前用户已确认5RPM三方向及VM0V，不重做该旧项。细节P1_HWT101_STATIC_READ_2026-10-18.md。R RUN-01/PWR-01、E LOC-01/02/03/05/07、RUN-06/MOT-04核对，Q不变。

> **2026-10-18实车接续**：用户确认本次PC13脱机5RPM三段运行正常，前进/左移/逆时针三个方向均正确，结束VM实测回到0V；接受本项有限方向及结束断电测试通过，不推断精确速度/距离/角度、闭环定位或整车功能通过。用户另确认WT姿态传感器已安装接线。既有接口计划为HWT101/USART2 PD5 TX、PD6 RX，当前尚未实现该串口姿态读取，实物型号/实际接线/波特率仍须核对；下一步先静态采集和航向方向/新鲜度检查，不触发PC13运动。证据chassis_5rpm_three_directions_user_pass.json。R RUN-01/PWR-01及E LOC-03/05、RUN-06/MOT-04，Q不变。

> **2026-10-18最新用户调整：按下直接启动、5RPM**：取消前版5秒倒计时和人工等2秒/首次释放要求，开发测试模式检测PC13稳定按下20ms即请求启动，上电已按住也接受一次。仍先经Safety/CAN就绪与四轮失能/使能/零速检查才发运动帧，不承诺按下同一时刻转动；前进、左移、逆时针各约10秒，每轮目标5RPM（实测整数速度绝对值>7RPM中止的开发候选阈值）。一次上电仅一次，运行中再次按下取消，结束关VM。140项模拟检查通过，Keil Code30792/RO440/RW1012/ZI23132、0错0警告，原扫码模式main/key均ARMCC编译通过。无线烧写后可直接按键，无需Debug/Watch。先前1RPM、5秒延时和人工等待步骤均被本条替代；新脱机实转仍待现场验证。R RUN-01/PWR-01，E RUN-06/MOT-04已核对，Q不变。

> **2026-10-18最新：PC13脱机三段测试**：用户已提交演练state9/result1/cause0截图并确认VM实测0V，默认演练通过；实际三段车体运动尚待验证。用户确认PC13可按，已新增独立开发模式APP_CHASSIS_KEY_TEST_ENABLE=1：每次上电只接受一次PC13启动，5秒延时后请求VM，最多等5秒健康，自动三段各1RPM/10秒、停车并关VM；倒计时/等待/运行中再按取消，结果后再按不重启。设该编译开关0恢复原扫码按键路径。无线烧写后脱机按键运行，不依赖Debug；仅需Watch排查时再接有线。124项模拟检查（原103+新21）通过，Keil Code30808/RO440/RW1012/ZI23132，0错0警告；原扫码模式ARMCC编译通过。尚未下载或操纵硬件，脱机实转待用户确认。详见[P1_PC13_OFFLINE_CHASSIS_TEST_2026-10-18.md](P1_PC13_OFFLINE_CHASSIS_TEST_2026-10-18.md)。

> **2026-10-18当前唯一现场下一项（三段固件已构建，默认演练待现场）**：源码/引用/AXF及10月17日Keil日志已核对，自动三段执行已接入，103项整合检查过，Code30268/0错0警告，无需重复编译。重新载入工程下载后供电健康，默认g_chassis_auto_dry_run1，只设一次g_chassis_auto_request1，运行约40秒；预期result1/state9/cause0/cut1并表测VM0。尚无新三段演练/实转证据。上次文档写入因额度未执行，现已补齐；下方“尚未接入”均历史。详见P1_CHASSIS_AUTO_SEQUENCE_2026-10-18.md。

> **2026-10-17当前下一项（轮速适配已编译，三段执行尚未接入）**：用户运动学Rebuild含新c且0错0警告；按减少Watch手动核算的要求，已增加整数RPM/方向/反馈适配及三基本指令准备。实际模块4451+228检查过，本机已自动Keil Build，Code26016/RO440/RW904/ZI22992、0错0警告，不要求用户重复编译。下一项接有限三段执行/真实反馈/每段和总截止/故障后不自动续跑，当前jog仍四轮正向10秒；未下载/发新动作，VM保持0。详见P1_CHASSIS_COMMAND_ADAPTER_2026-10-17.md。下方待登记/待编译等均为历史。

> **2026-10-17最新下一项（重新加载工程登记）**：用户Rebuild0错0警告但新模块未编译；实际uvprojx无c/h登记、lnp/map无该对象。已重新只加入USER两项并验证XML/路径。现在重新打开磁盘工程，避免保存旧Keil列表覆盖，再Rebuild确认chassis_kinematics.c编译行。VM保持0，新数学模块源码和已通过检查未改。

> **2026-10-17最新下一项（X形计算模块待完整构建）**：用户已确认轮径77mm、轮距/轴距各约210mm及X形，尺寸确认阶段完成输入收集，精标仍待。新增USER/chassis_kinematics.c/h及Keil组引用，只做纯逻辑轮速正逆解/同比限幅、异常清零，无驱动动作；隔离ARMCC及实际模块4451项检查过。现在VM0完整Rebuild，确认新模块被编译；之后统一逻辑符号/反馈/量化和运动安全再接车体运动。下方要求再提供X形/尺寸是历史，不重复。

## 当前下一项（2026-10-17，四轮修正正向已实测通过）

用户确认四轮方向映射0/1/0/1正确，已完成前一项复验，不重做Rebuild/相同轮向测试。现在确认实际轮胎外径、左右轮中心距、前后轮中心距（mm）和麦轮滚子排布/车头方向，再建立统一轮速与反馈符号、麦轮正逆解及限幅/正式反馈安全。仍在阶段3底盘与定位，正式运动许可、几何与实际RPM精度、HWT/实时X/Y未完成。下方要求再次复验方向的段落均为历史。


## 现在唯一下一项（2026-10-17，方向映射已修正）

先保持VM请求0、外部VM关闭，完整Rebuild当前main，然后以原1RPM/10秒四轮流程复验逻辑正向。方向表为1左后0、2右后1、3左前0、4右前1。该版隔离编译及实际C主机109检查过，未完整链接/下载/实车复验。刚才截图四启动/四停止mask0F、state4/cut1/cause0仅为旧方向候选的本次运行结果；2/4方向按用户实测报告修正，不能把旧截图当新映射已过。

### 复验后沿交接继续

1. 冻结四轮逻辑正向，测量实际轮径、左右轮中心距、前后轮中心距并确认滚子排布。
2. 建立逻辑有符号轮速/反馈符号、麦轮正逆解及统一限幅，补正式运行反馈时效/失效停车/重新允许；现有调试动作不等于正式许可。
3. 受控验证前后、横移、原地旋转及组合动作，再接HWT101航向与实时X/Y定位；无有效X/Y不放行位姿跟踪。

仍处于交接阶段3“底盘与定位”，不是退回基础CAN，也不提前跳雷达或整场路线。当前已过基础、应答与调试动作不重复；正式运动和安全未闭环项继续保留。


> **2026-10-17当前下一项（四轮版本地完整构建已核对）**：实际build_log为Code26012/RO424/RW904/ZI22992、0错0警告，当前map/AXF含新增四轮符号；用户粘贴25492大小为旧日志。现在下载当前四轮产物并确认Watch数组/掩码后再触发，四轮实测尚未完成。

> **2026-10-17当前唯一下一项（四轮共同运行版待构建）**：用户确认1号反向正确，并明确选择四轮同时1RPM约10秒验证正向。已自动四失能/确认→四使能/确认→四F6方向0→四FE，准备5秒/运行10秒/截止10.5秒，默认dry1；主机109项实际C检查及main/Safety隔离编译通过，未完整链接/下载/四轮实测。现在VM0完整Rebuild给日志。方向0仅1号已确认正向，后三轮需观察；分笔CAN有启动差，不宣称精密同步。详见P1_FOUR_WHEEL_FORWARD_TEST_2026-10-17.md，下方旧单轮说明仅历史。

> **2026-10-17当前唯一下一项（1号反向版待构建）**：用户报告1号方向0正向转约10秒/停止/关VM，2～4无主动转动。按要求仅将main方向改1，速度raw1、10秒/10.5秒截止、自动四失能及仅1使能流程不变；ARMCC隔离main编译通过，未完整构建/下载/实测反向。现在保持VM0完整Rebuild给日志，随后同入口测反向。默认dry1保留，正式运动许可0。下方旧下一项仅历史。

> **2026-10-17当前唯一下一项（10秒自动准备版待构建）**：旧1秒默认演练state4/cut1/cause0与VM实测0V已过，用户随后报告见1号转动，要求延长10秒并自动先全部失能。已实现自动四失能→读回→仅1使能→读回→1RPM约10秒→停止，10.5秒关VM，准备5秒独立截止；默认dry1不发速度/停止帧，但自动F3准备真实执行。main/Safety隔离编译及实际C主机94检查通过，未完整链接/下载/实测新版。现在只保持VM0完整Rebuild给日志，后续新版演练不需手动四轮准备。下方1秒/手动准备说明仅为历史。正式运动许可仍0。

> **2026-10-17当前唯一下一项（1号1RPM短时测试版待构建）**：用户实测四轮失能再使能均通过。已加固定ID1/方向0/raw1约1秒及Safety独立1.5秒关VM；默认dry_run1不发速度/停止帧，先验证新截止实际关电再另行实转。main/Safety隔离编译及实际C主机80项检查通过，尚未链接/下载/实物演练。现在仅保持VM0完整Rebuild给日志。正式运动许可仍0，实际轮向/转速/制动及新截止硬件验证未完成。详见P1_WHEEL1_SHORT_JOG_2026-10-17.md。下方旧下一项保留历史。

> **2026-10-17当前唯一下一项（直接使能修复待构建）**：实测失能后F6返回E2导致旧使能前置阻塞。按用户要求main改为直接单次F3使能，删除enable_zero_result/reply；所有原条件/优先级保留。隔离ARMCC及实际分支主机69项检查通过，未链接/下载/实测。先撤VM关电，然后完整Rebuild给日志，再测失能后选址使能。历史零速前置版说明由此覆盖；非零运动仍未开放。

> **2026-10-17当前唯一下一项（锁轴入口待完整构建）**：失能版完整Rebuild0错0警告，用户报告四轮实测松轴均通过并完成关电步骤。main已新增g_can_enable_*，一次请求先发固定零速，成功后复查条件才使能；隔离编译及实际函数/分支主机106检查通过，尚未链接/下载/实测。现在只做完整Rebuild给日志，VM请求保持0；随后单轮锁轴/再失能。正式运动许可固定0，不开放非零速度。下方旧下一项保留历史。

> **2026-10-17当前唯一下一项（失能入口待完整构建）**：main已新增g_can_disable_*单次静态失能入口，固定enabled=0；ARMCC隔离编译与真实分支主机48项检查通过，未链接/下载/实测。现在只做Keil完整Rebuild给日志，VM请求保持0；之后按地址1验证F3应答及3A状态bit0=0。下方旧下一项保留历史；不开放使能或非零速度。

> **2026-10-17当前下一项（四台缩放现值已报告）**：用户现场报告四台S_Vel_IS全Disable，按Emm raw1=1RPM记录，不改菜单/不发4F；尚无菜单原图或非零精度验证。先结束查看菜单的VM供电，撤请求运行并实测0V，再准备单节点使能/失能静态入口及3A状态核对。非零速度、正式运动许可/运动时效限值和真实制动尚未验收，不用协议上限当安全速度。

> **2026-10-17当前唯一下一项（两种关断静态分支已过）**：停止发送失败软件注入cause1/cut1及用户实测VM0V、保持锁存、明确确认复位均通过；fail计数1→0为用户明确手动清零，非源码自动变化。超时cause2和正常四轮应答也已过，不重复。下一项只读核对四台驱动“缩小10倍输入”等速度缩放现值，不改配置、不发4F；其后设计使能/失能与受限非零测试。当前运动许可固定0、实际运动/制动和完整正式恢复尚未验收。

> **2026-10-17当前唯一下一项（发送失败注入版待构建）**：main已加入默认0的一次性自动停止发送失败模拟及计数，注入时不发帧、返回4并走原cause1关VM分支。隔离ARMCC和实际C主机82断言通过，未链接/下载/实物验收此版。现在用户完整Rebuild给日志，随后按健康启动后设关VM断点的步骤静态注入并实测。超时关断/恢复已过不重复，运动许可仍0。

> **2026-10-17当前下一项（超时关断已过）**：静态自动停止超时state3/cause2/cut1、用户实测VM0V，以及撤请求后保持关断、明确确认后state0/cut0/锁存0/result1已通过；最终VM请求0、故障0、运动许可0。下一项准备停止失败cause1的软件注入，先隔离检查再用户构建/静态实物关断验证，不重复超时case；真实运动制动/正式运动许可和速度限值仍未验。

> **2026-10-17当前唯一下一项（超时注入版待构建）**：静态自动停止四result1/ack0F/state2、恢复不重复触发、VM0明确确认复位均过，用户实测0V。main已加默认0的g_motion_diag_hold_stop_service，暂停自动停止领取而保留采集/状态保护，用于后续验证500ms候选超时实际关VM；隔离编译过。现在只做完整Rebuild给日志；其后按先健康启动、后设Power_Disable关断后断点的顺序静态注入并实测VM，未测前不签失败/超时兜底或运动停车通过。

> **2026-10-17当前唯一下一项（自动停止版待构建）**：诊断锁存/恢复不自动清除/VM1确认拒绝/VM0明确确认静态检查已过，用户实测VM0V。现已接入Safety发布四轮停止、CAN任务执行、失败或500ms静态候选超时追加关VM条件；运动许可仍0。ARMCC隔离两单元及真实C函数主机63断言通过，未整工程链接/下载/实物验收此版。现在只要求用户完整Rebuild给日志，后续静态验证自动FE接收；不能声称物理停车或正式运动恢复完成。

> **2026-10-17当前下一项（诊断锁存版待构建）**：Safety已加入反馈诊断锁存10/20、禁止原因40及VM请求关闭后的一次性确认接口；旧反馈恢复不自动清诊断，确认不清原Safety故障/不允许运动，复位会清RAM记录。Safety/main隔离编译过，尚未链接/下载/实测此版。唯一下一项用户完整Rebuild日志，再逐项静态检查默认、hold过期锁存、反馈恢复仍锁存、VM1确认拒绝和VM0确认；正式运动许可/停止联动/重新供电仍未完成。

> **2026-10-17最新验收后下一项（过期/恢复已过）**：静态旧快照自然过期与恢复通过：report178冻结、Safety两fresh0/reason38；恢复发布report1CF/两fresh0F/reason08，故障0/两个ready1/allowed0，用户最后撤VM实测0V。下一唯一开发项是默认禁止的运动故障锁存及明确确认诊断层，先隔离验证，再用户构建/静态检查；正式CAN停止联动、故障重新供电/重新允许、运动时效和非零速度仍未实现。不要重复旧快照检查或把新鲜恢复说成正式恢复已过。

> **2026-10-17当前唯一下一项（冻结发布诊断版）**：时效诊断完整构建及3D→38→08→38静态转换已过，用户撤VM实测0V。main已新增默认0的快照发布暂停开关及发布计数，隔离编译过；现在请用户完整Rebuild给日志，再验证旧采样在Safety自然过期。此时不重做旧四轮控制应答，不放行运动；正式许可、运动阈值、失效停车/重新允许仍未完成。

> **2026-10-17最新下一项**：首版运动禁止诊断Rebuild及VM0的0D/VM1就绪的08显示已过。现已加入速度/位置采样时间的Safety独立时效诊断（仍为静态1000ms，allowed固定0），隔离编译通过；唯一下一项为此版本完整Rebuild给日志。新预期VM0健康reason3D，VM1反馈关闭reason38，两类四轮反馈新鲜reason08。正式运动时效、失效停车、许可/恢复仍未闭环。

> **2026-10-17接手更新（优先于下方10月15日旧快照）**：CAN控制接口已真实构建，四轮静态停止/零速接收应答及随后RPM0读取已完成；撤VM用户实测0V，关闭请求下两个测试入口result9。当前开始运动许可第一阶段诊断，Safety新增allowed固定0及禁止原因位，尚未实现放行/恢复。**唯一下一项：诊断版完整Rebuild给日志**，其后只观察VM0默认allowed0/reason0x0D。助手隔离编译Safety/main已过，未链接/下载该诊断版；不能把静态应答或诊断当完整运动安全。细节以执行记录2026-10-17条及当天证据为准。下方旧的“接口尚未改/尚未构建”只作历史。

> 2026-10-15用户要求进入下一步，已确认整车机构装齐、可载物测试（仅装配状态报告）。当前唯一下一项按P1_CAN_MOTOR_CONTROL_INTERFACES_MANUAL_2026-10-15.md第2～5节手改can.c/can.h并Rebuild：新增使能/立即停止/Emm原始速度接口，保留四种读取与现有main。厂家42/43/50/53/60/81页已核对；实际速度缩放设置未核实，非02应答返回8保留原始帧。隔离ARMCC can/main编译及101项主机模拟断言通过，真实源未改、未链接/烧录/发CAN。正式构建后再接停止/零速度验证及运动门禁，不直接发转动指令。

基线docs/competition_requirements_2027.md；沿用用户旧编号12374568，搬运机构由视觉组并行，电控主线按新阶段1→3→4→5→6→7→8。用户手动改F407，SPL+FreeRTOS原生API、F103模块风格、中文注释；成品PCB不改，PA1软件选档跳过。以下为工程安排（E），不改正式规则。

## 当前已验证与仍待完成

截至2026-10-15，用户实物/日志已覆盖：8MHz外部时钟与168MHz基础运行、SPL＋原生FreeRTOS、LED/蜂鸣、USART1收发/独立RX、PC13自恢复按键消抖/释放重接、任务监督/RTOS故障关VM、IWDG正常与停喂复位/调试冻结行为、PE2默认VM0及独立硬件关断。UART4原屏完整15字符任务码、UART5实际扫描/多次换码、PC13启动请求锁码均已有反馈；正式结构化业务与完整READY/重新允许尚未完成，赛后抓放统计按用户决定延后。

四台X42S_V2.0/ZDT_CAN_V1.2/FW_Emm/CAN1_MAP/500k/固定6B，地址1左后/2右后/3左前/4右前，单独及共总线版本/状态、四台速度/位置单次读取均已过。持续采集以50ms交易后间隔通过，现已移入独立CAN任务，扫码/屏幕TaskA10ms，调试串口短临界区包装递归错误已修并正式构建/启动。分离后两图count18/18/17/17→39/39/39/39，两mask0F/error0、两fault0/两ready1，用户确认换码屏幕正常更新；CAN查询负载余栈297字、APP最低169字、监督230字、heap11320B。缺失ID1静态fault20/原因2/坏节点1及VM实测0、接回复位平台恢复且VM仍0均已通过。详见2026-10-15证据与执行日志，不把旧10ms超时认定为唯一频率原因。

上述是已接模块的基础/静态门禁，不等于整车安全或运动完成。用户最新报告机构完整装好、可载物测试，后续实际运动仍遵守完整装机载物低速；实际载荷、轮向/几何与安全速度限值待量测。连续数据失效到运动停止、正式就绪/重新允许、运动时效、HWT及实时X/Y等尚未完成。

## 阶段与交付

| 执行序号／旧编号 | 工作 | 验收重点 |
|---|---|---|
| 1／1 F407基础收尾 | 现有RTOS/通信框架保留，补公共就绪状态、默认禁止输出、急停/故障监督及屏幕/扫码；PA1软件不做 | 每个必需模块自检/超时明确；当前核心测试已过，不签整车安全完成 |
| 2／2 搬运机构（视觉组并行） | 夹爪、270°三槽盘、搬运轴/回零、机械臂动作组、库存与臂盘互锁；场地原料转盘上表面80～100mm集中配置 | 电控提供公共驱动/底盘状态，真实高度与视觉标定版本一致；取放实物证据，载物姿态就绪后才联调实际运动 |
| 3／3 底盘与定位 | 电机CAN硬件/固件/地址/状态→四轮反馈；轮径/轮距/滚子排布与方向；正逆解、同步/限幅、HWT101航向、实时X/Y与视觉纠偏 | 先静态/软件和安全前置，实际从完整装机载物低速；四向/旋转/组合误差，反馈丢失停车；X/Y未闭环不放行位姿跟踪 |
| 4／7 路线与曲线基础 | 已知受控路线，位置/朝向联合到位、连续途经点、S曲线及速度/加速度/加加速度约束 | 轮速和车体扫掠包络、载物稳定、终点停稳；做受约束耗时优化，不用未验收雷达图驱动 |
| 5／4＋6前置 雷达机构及验收 | 先180°舵机升降、ULN2003转向、零点/高度/安装固定/停稳，再实际标定及完整14圈扫描 | 两台各自配置/误差/黑障完整性；不用仅改ID硬并TTL TX，机构改变重标定 |
| 6／5 雷达协议 | F407中断/DMA收字节，任务处理COBS/CRC、握手、选区、心跳、scan_sequence与结果、超时/过期 | ACK不等完成；坏帧/缺包/旧结果拒绝，接收与安全任务不被等待阻塞 |
| 7／6剩余 自动扫描联动 | 启动条件满足后按合规时机升降→转向→停稳→扫描→完整结果→准备状态/提示；次数配置沿用 | 开局尺寸与出发后展开约束；失败不报全系统成功，不仅凭几个Init完或一次ACK触发 |
| 8／8 视觉整合及整场 | 真实障碍地图接入路线规划；电控与视觉统一坐标/时间/高度/标定版本，结构化目标和动作回执，载台库存/运输互锁 | 单件抓放后全任务，六色/四段码/显示，双抽签起点返回、车道包络、故障恢复与续航；必须有实车/视频/同步日志 |

顺序未重新改为雷达优先；新阶段5内部仍机构先于实际扫描。视觉接口提前约定，完整整合集中阶段8。开局图可靠且障碍未变可用本轮图，行驶中定位和必要感知仍持续。PA1跳过不删除FIELD-02抽签起点身份，start_id输入配置后续实现，不能假定雷达已自动确认它。

## 历史下一项（2026-10-15，已由当前进度覆盖）

用户手动按[使能/停止/速度接口指南](P1_CAN_MOTOR_CONTROL_INTERFACES_MANUAL_2026-10-15.md)第2～5节，只改SYSTEM/CAN/can.c和can.h，Rebuild并提交日志。main保持当前已验独立CAN/扫码/监督版本，不增加动作请求入口。通用单帧收发承接旧初始化/发送100ms/接收200ms及回帧校验，旧四种读取函数体不变；新增控制地址限1～4、同步位固定0，控制应答02仅表示接收正确，非02原始回帧保留返回8。速度接口明确使用原始命令值，四台缩放10倍设置未核实前不当作RPM换算。协议上限3000不是整车允许速度。

准备证据：厂家手册V1.0.5第42/43/50/53/60/81页和渲染图；隔离副本ARMCC5.06编译can.c及现有main，0错误警告输出；实际拟议函数体在主机模拟CAN/RTOS环境执行101项断言，含帧内容、无效参数不发帧、非02应答、过滤、超时、tick回绕及旧查询解码。真实can.c/h/main哈希前后不变、无链接/烧录/外部CAN动作；私有目录清理状态在证据中记录。此证据不替代用户构建和实际控制应答。

后续先接立即停止/零速度的可审查入口与实际回帧，再补正式运动允许、连续反馈失效到停止、较快运动时效/超时及有限动作窗口、载物低速限值、轮向/几何/正逆解/同步和HWT/运行X/Y。当前1秒数据候选只用于静态，不能据此宣称可以安全行车。原八阶段顺序、视觉并行工作包、抓放统计延后及用户手改中文SPL/FreeRTOS不变；不重复已过读版本/状态及原基础采集测试，后续仅针对新通用收发的真实读取回归和新控制路径验收。

## 规则检查与未闭环

R：FIELD-01/02/06、OBS-01/04、RUN-01/02/03/04、PWR-01、MECH-01/02、QR-01、DSP-01/02。
E：LOC-01～07、MOT-01～04、RUN-05/06、CAL-05、VIS-09/12/14/15。
Q：FIELD-05、OBS-03、QR-04继续待技术答疑确认，不因工程选择而自动闭环。
工程未闭环：用户本轮控制接口正式构建/修改后实际读取回归/控制应答、四台速度输入缩放设置、运动反馈与失效策略/正式停止重新允许、实际载荷/轮向/几何/限值、HWT/实时X/Y、雷达机构/完整扫描、搬运视觉高度/动作/回执、赛后抓放真实统计/触发/字体实测及整场。基础IWDG、真实扫描显示/锁码、独立持续采集/扫码并行、缺失ID1静态关断和接回复位VM0不回退成待验；不能将它们扩大为运行中断线/实际运动或完整阶段通过。官方Q条目继续待技术答疑确认。助手未改真实固件或执行硬件操作。
