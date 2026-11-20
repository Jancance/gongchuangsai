# 下一阶段：值输入地图与反馈适配合同

日期：2026-11-18。本阶段 root 已提交并冻结基线 `d023c37a6c51e5a454b828cf1e557643ee31c4dd`，以本阶段 preparation 回执为准。上一阶段 `fusion_reference.c/.h` 已包含在该本地提交，最终源码/验证见 `../../2026-11-17/action-fusion-reference/REPORT.md`。本规划只写证据，不修改产品或访问硬件/网络。

## 1. 本阶段最小交付

只新增 `CHASSIS/USER/fusion_adapter.c/.h`，采用已经复制好的值输入。适配有效 `AppMapPose` 和调用方显式 F/mm 有向折线身份，构造已有 `FusionObservation`，为后续实际调用 `FusionReference_Step` 做准备；反馈仅实现明确标记的地图差分 ESTIMATED 分支。

本阶段不增加运行 caller、任务、CAN 查询、停车策略、雷达业务/回调、机械/视觉、电源/运动许可，不改旧 map50% overlay、reference 两源或工程目标引用。真正雷达完整 bundle 发布和同 CAN owner 运行接入分别是后阶段。root 后续独立决定工程引用、构建、交接和本地保存范围。

实现前先在本机 MATLAB 验证对应估速、时间/身份、缓存、缺帧和停止证据合同，实际通过后再写两源。MATLAB、实际 C 和物理停稳是三类证据。

## 2. 实际源码证据和不能越过的边界

| 现有入口/字段 | 实际含义 | 本阶段处理 |
|---|---|---|
| `app_odometry.c:98` `AppOdometry_ReadMap` | 临界区复制 `AppMapPose`；run/start/map、raw/corrected、state/based/valid/reasons/continuity 一致 | 使用 caller 已复制值，不再读 volatile 分散全局；保留 getter 既有 stale→同run GAP 行为，不新增另一永久锁 |
| `AppMapPose.pose.sample_ms` | 轮位置及 HWT 历史能共同插值的 pose 时刻 | 是本适配 Observation 的 source key，绝不用 `now_ms` 或 publication tick 冒充 |
| `AppMapPose.wheel_ms[4]/angle_ms` | 各源最近被接收/接纳的帧时刻，未必等于 pose common sample | 保留审计，不把四个最新源时刻当同采样；本阶段不自行重插值拼帧 |
| `AppMapPose.raw_x_mm/raw_y_mm`、correction | raw 是轮/HWT里程计积分，仍含既有1.0218254等基础尺度；corrected 是已有残差 overlay 累加一次 | 跟踪位置读 corrected；估速仅用未含map50% overlay的raw差分，不能称未标定原始物理量，不能把overlay跳变当实际速度 |
| `heading_source.c:24/35` `App_HeadingSourceRead/Publish` | 原子 TaskA 发布；yaw_deg 单位度、rate_dps 度/秒；angle_ms/rate_ms 是各真实接收帧代理时间，sample_ms 是本次发布tick | 本阶段无 live HWT 分支，不改 owner；后续可用真实 rate，但必须做源时刻/skew 校核 |
| `chassis_odometry.c:56/171` | 地图 heading = wrap(raw HWT yaw + 初始化 heading_offset) | 直接用 map.pose.heading_deg；角速度差分 wrap 两个地图角。不能用 raw yaw 替换场地 heading 或重复加一次90度 |
| `ChassisAuto_ReadWheelLog` | 原子复制 driver 有符号 deci-RPM、各回复sample_ms；speed mask 需 active、非dryrun、fresh | DONE/停机 mask=0，不能当零速；不能据此构造通用停稳测量支路 |
| `CAN1_ReadSpeed` / `CAN1_ReadEmmSystemSnapshot` | 都会实际发起总线交易 | 不是只读 cache getter，本阶段禁止调用 |
| `App_CanFeedbackStep` 的 rpm/result/tick | owner 条件轮询生成的诊断缓存，可能停用/清除；当前无独立公开原子 speed snapshot | 不暴露分散全局冒充一致测量；后续同owner添加最小源发布再接，不在本stage写悬空 measured 接口 |
| `Chassis_DriverFeedbackToLogical` | 输入是 driver RPM；物理前向映射为 0/1/0/1，地址2/4反号 | 仅记录未来 measured 分支换算；deci-RPM先除10，不能直接当RPM/mm/s |
| `Chassis_Forward` | logical wheel RPM → body F/L mm/s、CCW rad/s 的运动学估计 | 未来 wheel 测量不等于地面速度真值；变世界速度仍要实际 map heading，wz rad/s与HWT度/秒不可混用 |
| `RadarObserver_GetPath/GetStatus` | 各自临界区复制，但两次调用不是一个快照；path仅scan/request/start/goal/waypoints | 没有完整占用图、layout/G-F/run，不能称当前接口已够完整路径适配 |
| `radar_observer.c:on_grid` | 仅保存 scan_sequence，未保存完整 grid_ids；当前只支持 Zone2 owner | 不从path反推障碍；不另注册 callback 抢owner，不伪装支持两起点整场 |
| `RuntimeRoutePrepared.local_targets` | 旧局部名义轨迹；不是连续 F 地图实际位姿 | 不直接转成世界实际路径，不以安装成功当执行成功 |
| `FusionReference_Step` | 同 source sample 返回完整缓存；同timestamp下任何 pose/feedback/stopped 内容变化会拒绝，只有now可变 | adapter 必须对每个 map sample 锁存完整 Observation，不能每次塞最新异步速度导致缓存冲突 |

现有源码中不存在可不加事务、普遍覆盖 DONE/待机的现成公开原子四轮速度 getter。本阶段采用 raw 地图差分，是可落地的软件模型交付，不伪称已接实际四轮/HWT速度反馈。

## 3. 最小值接口

沿用已有 `FusionIdentity/FusionPlan/FusionObservation`，避免再复制一套路径/配置结构。API名可按两源现风格调整，语义如下：

```c
void FusionAdapter_Reset(FusionAdapterState *state);
uint32_t FusionAdapter_Observe(const FusionPlan *plan,
    FusionAdapterState *state, const FusionAdapterInput *input,
    FusionObservation *observation, FusionAdapterReport *report);
```

`FusionAdapterInput` 是 caller 已复制的 `AppMapPose`、调用时 `now_ms`、显式绑定的完整 `FusionIdentity`，以及最小停止证据 `stop_confirmed/stop_sample_ms/stop_run_id`。接口不调用任何 getter，也不自动 `RequestMapRun/MapActionAccepted`。停止证据只是来自 caller 既有停车确认的事实，不是运动许可。

输入 F 有向折线直接复用已验证 `FusionInput/FusionReference_Prepare`，不增加薄的重复 Build 接口。run/start/map 在 `AppMapPose` 中有实际字段并与 plan 一致；layout/path/direction 在本stage由 caller 显式提供并绑定计划，不能写成地图或雷达回显。方向相反是另一条身份和路径，不反转数组后复用旧执行状态。

`FusionAdapterState` 只保存前一 fresh raw pose/common time/地图heading、身份绑定及当前sample的完整 Observation 缓存。不得复制整个 FusionPlan 或保存无限历史。`FusionAdapterReport` 只记录可用性/原因、估计窗口、source类型、estimated_stationary和所读 map校准覆盖；不写map、不重新算修正、不生成deployment许可。

首次或重新绑定时仅建立基准，输出 WAIT/不可用，不把未知速度填0并当有效。失败输出清零并明确不可用，调用方不能复用上次有效输出当 fresh。本适配不新增停车/永久故障锁；输入恢复后，按下面的新窗口规则可重新建立基准。

## 4. ESTIMATED 反馈、原子性与重复样本

使用同run/start/map及完整路径绑定下的两次连续 fresh map 样本。两者必须 based、TRACKING、pose.valid、reasons=0、continuity_lost=0，且源时刻不早于本轮 begin、未在未来、age符合 plan 现有 max_age；保留既有源健康口径，不用 adapter 放宽。

```text
dt = unsigned_delta(current.pose.sample_ms, previous.pose.sample_ms) / 1000
vx_est = (current.raw_x_mm - previous.raw_x_mm) / dt
vy_est = (current.raw_y_mm - previous.raw_y_mm) / dt
omega_est = wrap(current.map_heading - previous.map_heading) / dt
observation.position = current.corrected map XY
observation.heading = current.map_heading
observation.sample_ms = current.pose.sample_ms
observation.feedback_source = FUSION_ESTIMATED_FEEDBACK
```

合法 dt 使用显式模型窗口 `0<dt_min<=dt_max`，不跨已有 max_age/连续性允许范围；整数时间按已用的uint32差值处理回绕并拒绝未来/倒退（有效间隔小于半周期）。短于dt_min时保留原有效anchor等待，不能消费anchor或输出伪零速度；长于dt_max、gap/stale/连续性断裂时丢弃旧窗口，以当前合法样本重新prime，再等第二个fresh样本。窗口不是新增比赛停车条件，只决定估计是否可用。77mm轮的每计数量级π×77/65536≈0.00369mm，1ms差分约3.69mm/s，已大于2mm/s量级margin；MATLAB必须扫量化/窗口确认dt_min，不能通过盲放speed margin掩盖。输入来自轮/HWT接收时刻代理，不称传感器硬件同步采样。

同map sample再次调用：先检查当前now/身份/健康/内容一致，再返回已缓存的完整Observation，仅更新now；不得把0位移/重复sample变成0速度，也不得把新停止证据塞进同sample改变下游signature。若当前输入stale或gap，即使有缓存也返回不可用，不能把缓存fresh化。

同timestamp下 raw/corrected/heading/身份等核心pose内容冲突应拒绝，不消费新基准；发布tick/age、各wheel_ms/angle_ms、action_epoch/calibration_flags等metadata在共同pose不更新时可自然变化，不能hash整个AppMapPose制造不必要拒绝。停止新证据同sample只报告待绑定，不视核心冲突。当前metadata健康/age每次重验，报告可并列当前metadata和cached样本metadata，Observation保持缓存。旧timestamp不倒退基准；begin变更、local_origin异常或updates回退也不能跨旧基准差分。真正gap/连续性断裂清除估速窗口，恢复必须从两个新fresh有效样本重新建立，不能跨缺口求差分制造平均低速。

overlay变化只影响corrected跟踪坐标，raw速度不受影响；同时完整保留map的raw、correction和calibration_scope/flags/source/action_epoch等审计信息。不同本地动作/旧local reset不等于新run，不能无依据清整个map或把corr归零。

## 5. stopped 与 estimated_stationary 分开

`estimated_stationary` 可依据已成立差分窗口的估计线/角速度与 plan stop限值报告，只是模型状态。以下均不能单独令 `Observation.stopped=1`：首样本、重复sample、map差分为0、`speed_valid_mask=0`、软件DONE、STOP ACK、VM关闭或desired速度0。

本stage root 明确选择：`Observation.stopped` 只由 caller 已有 `stop_confirmed` 证据给出，且 source timestamp有效、未在未来/过期、不早于当前run begin、stop_run_id匹配。缺证据置0，仍可输出有效ESTIMATED速度。不能为了模型能DONE自动伪造一位确认。

已有 `FusionReference_Step` 仍同时检查速度和heading；stop证据不绕过这些条件。此stage无真实owner caller，所以不声称能自动完成整车起步/折点/DONE。未来同CAN owner复用既有真实停车反馈自动生成证据，不新增人工确认或永久锁，不把历史运行stop mask复用到新run。

停止确认更新但map sample未更新时，可以报告有新证据待绑定；要到下一fresh map sample才进入新的Observation。此行为与reference现有同sample缓存合同一致。

## 6. 小混合纠偏与四向50%覆盖：本stage只报告

既有map50%按 `MapActionAccepted` 锁存 direction，并在首有效共同sample冻结参考角，以raw净主向进度一次投影主/旁修正。边界/unsupported/停车尾是raw-only并有flags，历史修正保留。adapter只读这些事实，不调用接受入口、不开始新scope、不再给position/速度叠补一次。

后续保留主向候选的推荐判据：世界路径切向相对锁存的 planned hold heading 为纯±F/±L时，保留对应0前/1左/2右/3后主方向身份；actual yaw小误差或有界横移让body两个分量非零，记录MIXED_UNVERIFIED，不用第二分量再套另一方向系数，也不静默全禁主向候选。

例如target90、actual88、world+Y400，body约F399.76/L13.96，仍是前向主路线含未验混合；原料→粗加工CW中途heading+X、world向左时主方向应是后退3，不能因地图看起来向左套left系数。真正相对planned hold的斜段无已验证纯四向scope，继续明确unsupported。

此判据只是后阶段实施建议，不新增一套可生效系数。本stage报告“来源map目前覆盖”与“未来desired是mixed”两件事实，不能把两者合成“mixed已校准”。未来需要MATLAB先确定主向raw净投影的冻结参考，使纯四向回归原式、小横移保持主向一次候选、旁向运动留raw且不重复累计；再决定是否改现overlay。当前raw/历史corr/50%/trim/距离倍率完全保留。

## 7. 有向路径后阶段接口依赖

本stage caller显式F折线及完整身份可用于名义验证，始终model_only/deployment_allowed0。不从未知设备格布局伪造F坐标，不把粗加工/暂存源占位作为已确认停车点。

真正雷达adapter后续最小增量应在现有radar owner内保存同事务bundle：已确认node/zone、local scan epoch、scan_sequence、完整grid占用、匹配request_sequence/start/goal/status/waypoints、layout/G-F版本及发布epoch，一次原子复制。PATH不含起点含终点；压缩跨格轴段要保留沿途格，不能按蛇形编号差乘480。成功空列表只对应start==goal。单独ACK/path_valid或历史Flash图不能代替本场完整数据。

该后续工作不改变本stage pure adapter API，不另抢callback，不因为完成两源就声称雷达地图/两起点业务已接通。

## 8. MATLAB前置和实际C验收矩阵

| 场景 | 必须验证 |
|---|---|
| 两fresh样本、四向、非零mapheading、跨±180 | raw速度和wrapped角速单位正确；corrected位置保真；ESTIMATED明确 |
| raw匀速而corr增大/跳变 | 速度只来自raw，不把导航修正当加速；pose保留已有修正一次 |
| rawyaw与mapheading相差90/其他offset | worldheading使用map值；omega差分不重复offset；不把yaw=0当场地+X |
| 首样本、短dt保anchor、长dt重新prime、量化窗口、同sample重复 | 基准不伪速度；缓存Observation完整一致；重复不会stopped或推进reference；模型冻结可用dt_min而不盲放margin |
| 同timestamp核心冲突、metadata自然更新、begin/local_origin/updates异常 | 核心冲突拒绝，metadata不误拒；不跨重基准/回退估速；当前健康仍每次验证 |
| 同map样本下HWT/stop异步更新 | 不改变已绑定Observation的速度/停止事实，下一fresh样本才绑定新事实 |
| stale、gap、continuity、倒退/未来、uint32回绕、异常dt | 旧缓存不fresh化；不跨gap估速；恢复从新窗口，失败不推进基准 |
| run/start/map/layout/path/direction逐字段错误 | 实际map字段和caller绑定分开校核；错身份不继承速度/停止证据 |
| stop缺失/旧run/过期/未来；valid stop但速度仍大 | obs.stopped口径正确；Reference仍不误DONE；estimated_stationary不冒充物理停稳 |
| 无stop证据而差分速度0、软件阶段完成 | stopped仍为0；本stage没有轮速mask/ACK等输入branch，不为这些另造接口或测试分支；源码检查不调用CAN交易 |
| map50四向、source boundary、unsupported、tail、MIXED | scope/flags/corr按源保真，无双补、无改map；不签mixed校准 |
| 显式F有向polyline及反向/错path | 原端点/剩余路程不重写；复用Reference，不假称radar路径来自设备 |
| NaN/Inf、坏指针合同、容量/坏plan | 按两源实际接口所需最小检查，失败零输出，不引入额外通用框架 |

MATLAB保存实际版本/开始结束/退出码/脚本hash/断言和对照向量；不得用Python替代前置。C编译实际两源并用对应向量复算，联合现真实FusionReference做重复sample/首基准/stop及恢复集成检查。RTOS getter和真正owner不在本阶段，明确纯值模型范围；旧受保护源hash保持。无新bug不扩大无关回归。

## 9. 随后A/J执行阶段

先在MATLAB以当前真实或明示估计的world/body v/a、源延迟与j限制验证向量执行，再写C；不能用desired cap直接驱轮。需要同时检查heading角速/角加速/角jerk和四轮RPM/加速/jerk，旋转实际heading参与世界/车体变换的导数耦合也要计入。

停止距离须覆盖当前a非零、jerk建制动、观测延迟/持令和wheel reserve，不复用仅a_hint的sqrt cap签真实刹停；无法恢复的状态输出既有恢复策略所需信息，不新增长期锁。之后接唯一CAN owner、既有Stop/故障和自动停车证据，最后再做非零过角blend。硬件保持暂停，新动作另外按明确版本/动作/次数/现场条件授权。

## 10. R / E / Q、未闭环及当前状态

R 已核 FIELD-01/02/06、OBS-01/04、RUN-01/02/03/04/07、PWR-01、MECH-01/02；E LOC-02/03/05/06/07、MOT-01/02/03/04、CAL-02及本合同估计/速度/覆盖建议。Q FIELD-05 / OBS-03 / QR-04 保持待技术答疑确认。累计轮/HWT估计不冒充无漂移定位或毫米物理验收。

工程未闭环：真实停车点/运输包络/G-F布局、完整雷达bundle、停机仍可用实际反馈发布、真实source skew/时延、mixed候选有效性、A/J制动、栈余量、业务等待及双起点整场。它们不混称官方Q。

本文件完成时只做源码/上阶段报告核对与规划，尚无本stage MATLAB/C通过结论；未改产品、未网络同步、未访问硬件/下载/reset/发车。本stage明确缺CAN实际速度、真实HWT瞬时角速度、完整雷达占用/路径snapshot及runtime caller，不能称已实时接通整车。最终接口、计数和验证状态以本stage编码及独立审查实际回执为准。
