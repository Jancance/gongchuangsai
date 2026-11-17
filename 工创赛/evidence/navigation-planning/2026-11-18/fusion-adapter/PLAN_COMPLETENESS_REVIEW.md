# fusion_adapter 前置完整性规划审查

日期：2026-11-18。只读规划审查；核对 HEAD `70b230c481b03c4fd30f0079c26f9a3f2d003701`，参考层功能基线 `d023c37a`。本审查只新增本文件，不修改产品、模型、Git 索引或已有交接；不 fetch/push，不访问、枚举或轮询硬件，不下载/reset/发车。

## 结论

`PLAN.md` 的纯值、ESTIMATED、无执行器范围可继续。现有 MATLAB 回执真实为 exit 0、521 检查/120 向量，脚本 SHA256 `ec94f0f2bc5b922ce5d65ddfade1e38244ad9559934085052a4dc73e1d9b2278`；但它是初验，尚不足以开始最终产品 C。必须先补齐下列身份、模时间、计划有效性、源健康、状态顺序与停稳缓存反例，并保存本机 MATLAB 新回执；不能拿前阶段 9034 或本阶段 521 代替。

已核源：`app_odometry.h` 的实际 `AppMapPose`，`chassis_odometry.h` 的实际 `ChassisOdometryStatus`，`app_odometry.c:98-129,175-267`，`fusion_reference.h` 全文和 `fusion_reference.c` 全文。两参考源当前 SHA 与 preparation.json 相同：C `07a9439f0435cb8ce35fc045b0a74f17e07e0362549336859760ae5921d02fb2`；H `8e0d855ae91a5ae6bc856aaf6af2300cd24ccdf8d91d251294087466fa0ab961`。当前六个 tracked dirty 名单未增加产品变更；历史 untracked 不在本任务清理范围。

## 一、初验的必要缺口

| 缺口 | 当前证据 | 最小补齐 |
|---|---|---|
| 完整身份 | verify.m:72/76/80 只有一个 `id`，未区分 plan、caller、map | 按实际六字段建 plan/caller 身份；map 只回显 run/start/map；各字段分别错一次，合法新绑定不继承旧估速/cache/stop |
| uint32 模时间 | verify.m:77/81/85/95 用普通 double 大小和减法 | 用 `mod(a-b,2^32)`；有效先后差严格小于 `2^31`；包含零时间戳、正常回绕、未来、倒退和半周期歧义 |
| 计划错误 | 当前模型没有 prepared plan 输入 | 补实际被 adapter 消费的 plan 合同；坏 plan 不变状态、零 Observation；不重写 Reference 的几何准备器 |
| 完整 map 健康 | `valid/gap/local` 把实际若干字段压缩 | based、TRACKING、pose.valid、reasons、continuity、local_origin 分别覆盖；每次调用重验 common sample 及四轮/angle 的时效与 run begin |
| 缓存完整性 | 120 组检查 `isequal(o,old)`，只变 flags/epoch | 新输出类型须检查完整 Observation（除 now）；每个核心浮点字段单独冲突，source/corr/scope/nav/源时刻等 metadata 自然更新单独放行且只影响 report |
| 状态检查顺序 | verify.m:82 先处理 updates 回退，:85 才拒绝旧 sample | 增加“旧 sample + 更小 updates”反例；同绑定/同 begin 先拒绝旧时间戳，不能据此倒退 anchor |
| 时间窗边界 | 已有短窗/长 gap 基本路径，但未全验边界与组合 | 19/20/100/101ms、短窗多次累积、短窗后重复冲突/旧 sample、长 gap 后两新样本、失效后两样本恢复 |
| stop 缓存 | 有新 stop pending 和几类坏 stop，缺 cache=true 的失效路径 | 原绑定 stop 过期、撤销、错 run、当前无效证据及新证据不得续命旧 cache；同 sample 不允许 stopped 翻转 |
| Reference 联合验收 | 当前脚本未实际调用 Reference 模型或 C | 在产品 C 验收使用真实 Reference：首窗不可调用为 fresh、重复不推进、零差分无 stop 不 DONE、valid stop 但速度仍大不 DONE、恢复无跨 gap 速度 |
| NaN/Inf 与错误上界 | 初验只测 raw_x=NaN，且用 norm(v)>10000 | 覆盖实际消费浮点与差分结果；速度范围沿用 Reference 的分量 +/-10000、omega +/-3600，不新增一个更严的合速度门禁 |

以上是本阶段所需覆盖，不要求重新跑全部上阶段几何集合，也不提前加入 A/J、雷达 bundle 或真实轮速分支。

## 二、供编码冻结的最小合同

### 1. 输入、身份与计划

- API 采用 `FusionAdapter_Reset` 与 `FusionAdapter_Observe(plan,state,input,observation,report)`；state 单 owner，plan 必须由 `FusionReference_Prepare` 成功得到且在绑定期间不可变。只复制 caller 已取得的 `AppMapPose`，不调用 getter、CAN、RTOS、运动入口或 Reference Step。
- `FusionAdapterInput` 只需 copied map、caller now、显式 `FusionIdentity`、`stop_confirmed/stop_sample_ms/stop_run_id`。`FusionIdentity.direction` 是路径有向身份 0/1，不是 map.calibration_direction 的前/左/右/后 0..3。
- caller 六字段全部等于 plan.identity；map.run_id/start_id/map_version 另行等于计划对应字段。layout/path/direction 是 caller 明确绑定的事实，不能声称 map 或雷达回显。
- adapter 只核它真正消费的 prepared-plan 外壳与字段：非空、valid==1、非零 signature、model_only==1、count 2..32、box_count<=32、合法六字段身份、max_age 100..1000ms、有限且符合既有 Reference 范围的 stop_speed 0..5、stop_omega 0..2。不扫描路径几何、不重算 hash，不用重复 Prepare 或整份 plan 拷贝掩盖 immutable 前提。几何等未消费字段的完整有效性仍由 Prepare 保证。
- 固定估速窗口 20..100ms，本阶段不增加可配置项。要求 max_dt(100)<=plan.max_age；plan.max_age<100 视为本 adapter 的 BAD_PLAN，不静默放大 max_age 或缩窗。不新增运动停车或永久锁条件。
- 合法新 plan signature 或完整身份绑定变化，使用当前合法样本 prime，输出 WAIT；合法 begin 变化同理。不能继承旧 cache 或 stop。identity 不匹配的输入是拒绝，不是合法重绑。
- 坏指针合同只检查 NULL（非 NULL 地址和输出/输入不别名由 caller 保证）；能安全写的 observation 先清零。NULL 行为在真实 C 验收，不用 MATLAB 模拟非法内存地址。report 不把 unavailable 的旧 Observation 作为 fresh 返回。

### 2. 时间和健康

- 所有 ms 都按 uint32 模差；MATLAB 的 uint32 直接相减会饱和，应先转 double 再 mod 或用等价 helper。`delta(a,b)=mod(a-b,2^32)`；有效“a 不早于 b”须 delta<2^31；恰好 2^31 拒绝。时间戳 0 不是通用无效值。
- 每次调用重新检查：based==1、state==APP_MAP_TRACKING、pose.valid==1、reasons==0、continuity_lost==0、local_origin==0；raw/corrected XY、map heading、审计 correction 浮点有限；heading 使用 Reference 已有 +/-3600 范围。
- common sample、四个 wheel_ms、angle_ms 均不早于本轮 begin、不在 caller now 未来；common pose age<=plan.max_age。逐源健康保留 getter 原口径：wheel age<400ms、angle age<100ms，边界相等即 stale。四轮/angle 可以新于 common sample，不能强求相等或拿最新源时间替换 common sample。
- `pose.age_ms` 与 map.now_ms 是快照 metadata，不代替用本次 caller now 计算的时效；合法自然更新不算核心冲突。
- 对同绑定、同 begin：先拒绝倒退 sample（state 不变），再处理同/新 sample 的 updates 回退（prime）。updates 是计数，不按 ms 模时间解释；自然回退也只重新 prime，不作永久故障。
- 明确 map 失效（不 based/非 TRACKING/invalid/reasons/continuity/local_origin、未来或 stale、非有限数据）清估速窗口，零输出。恢复后第一合法 fresh 值只 prime，下一足窗 fresh 值才可用。此处是 pure 值模型的窗口恢复，不承诺实际 getter 同 run GAP 会自行恢复；getter 的既有 run GAP 机制完全保留。

### 3. 估速、缓存和状态变化

- 可用 Observation 的位置=map.pose.x/y，heading=map.pose.heading_deg，sample=map.pose.sample_ms，identity=绑定身份，valid=1、continuity_lost=0、reasons=0、feedback_source=FUSION_ESTIMATED_FEEDBACK。world vx/vy 仅来自 raw_x/y 差分；omega 仅来自 wrapped 地图 heading 差分。raw 仍含既有距离基础标定，不叫真地速、瞬时速度或未标定物理 raw。
- 首样本 WAIT，Observation 全零。0<dt<20ms 保留估速 anchor，记录已接受的 latest 核心/updates 以识别后续重复和倒退；不得给有效零速度。dt=20..100ms 成窗，输出并把本次样本作为下一 anchor。dt>100ms 以本次合法样本重新 prime、WAIT，不跨长 gap 求平均。
- 核心是 common sample、begin、raw XY、corrected XY、地图 heading、完整身份。新 plan/begin epoch 先按上述合法重绑处理；其余同 sample 核心改变拒绝且 state 不变。raw/corrected/heading 的逐字段比较不 hash 整个 `AppMapPose`。
- correction_x/y、source、calibration_scope、nav_applied、calibration_flags/direction、action_epoch、map.now/age、wheel_ms/angle_ms 是当前审计 metadata；可自然变化，但仍要通过当前健康检查。corrected XY 已被核心比较保护，不另外把 corr 变化提升为冲突；不额外增加 corrected==raw+corr 的新浮点约束。
- 同 sample 已 ready：先验当前 plan/身份/健康/核心/stop 证据条件，再复读完整 cached Observation，仅输出 now 可变，state 不变。只 prime 或短窗的同 sample 仍 WAIT，不重复差分。
- BAD_PLAN、身份错误、旧 sample、同 sample 合法核心冲突：零输出且 state 不变。明确 map 健康失效清窗。合法重绑/begin 变化/非倒退 samples 的 updates 回退/长 gap：prime WAIT。这些优先级须落实为测试，不能统一写成“错误就 Reset”。
- 差分结果须有限，并符合 Reference 接受的分量/角速范围。失效只表示 Observation 不可用，不新增 executor 停车策略。
- report 记录当前原始审计值、估速窗口、ESTIMATED 来源、原因和 estimated_stationary 即可；不必在 state 再保存全份 map 或复制整个 plan。不得二次加 correction、重算四向系数、重置 raw/corr 或写 map。

### 4. stop 事实

- 新 fresh Observation 上，仅当 stop_confirmed==1、stop_run_id 匹配、stop 时间在当前 run begin..now、age<=plan.max_age（均使用模时间）时可绑定 stopped=1。缺失、错 run、未来、过期、早于 begin 的证据只令 stopped=0；不妨碍合格 ESTIMATED 速度输出。
- `estimated_stationary = norm(v)<=stop_speed && abs(omega)<=stop_omega` 只是 report。首窗、短窗、重复样本、raw 差分 0 不能自行生成 stopped。无 wheel mask、ACK、DONE、VM、desired 输入分支，不能为了测试这些词增加接口。
- 同 sample cache.stopped==0，新 stop 仅 report pending，Observation 保持 0。cache.stopped==1：原绑定 stop timestamp 必须按当前 now 仍 fresh，且当前 stop 确认未撤销/无效；否则整份 Observation 不可用且 state/cache 不变，不能把 stopped 改 0，也不能用新的 stop timestamp 延长原 cache 寿命。新的合法证据到下一 fresh map sample 再绑定。
- valid stop 且估计速度仍大时允许 Observation.stopped==1，但 Reference 的 stopped() 仍要求速度/角速度均过限值；必须用真实 Reference 集成证明不能误推进/DONE。

## 三、最小可测试验收门槛

| 组 | 至少包含的边界/反例 | 通过标准 |
|---|---|---|
| M1 身份/计划 | caller 六字段逐个错；map 三字段逐个错；非法 identity；prepared 外壳错误；max_age=99/100/1000/1001；stop 阈值 NaN/Inf/越界；合法新签名/反向 plan | 坏计划/错身份零输出且原状态不变；合法新绑定第一样本 WAIT，不继承旧速度、stop、cache |
| M2 模时间 | sample FFFFFFF0->00000004 恰20ms；同窗20/100；now/stop/source/begin跨零；未来1ms、旧1ms、恰半周期；时间戳0 | 模结果与手工 uint32 期望相同，正常回绕不被误判；future/倒退/歧义不产生可用值 |
| M3 当前健康 | 每个 map 健康位单错；common age=max_age/max_age+1；四轮逐个399/400ms、angle99/100ms；逐源未来/早于begin；伪造 age=0 | 失效清窗且 cache 不 fresh 化；当前合法 metadata 更新放行；不能靠 snapshot age 绕过 caller now |
| M4 完整缓存 | 重复 ready 与重复 WAIT；五个核心数值逐个变化；metadata 每类自然更新；旧sample+updates降低；同/新sample updates回退；begin变更 | 除 now 外 Observation 完全相同且 Reference state 不推进；冲突/旧样本 state 原样；合法 epoch 重建而不求跨源差分 |
| M5 窗口/数值 | dt=0/1/19/20/100/101；多次短窗；短窗后旧/冲突；长gap/各健康失效后恢复两样本；正反向与 +/-180双向wrap；NaN/Inf及除法结果界 | 短窗保 anchor、长 gap 只 prime；20..100内单位/符号正确；无跨 gap 假低速；失败 Observation 零 |
| M6 stop | 缺失、旧run、早于begin、future、age边界、跨零；cache false新stop pending；cache true原证据过期/撤销/新证据不得续命；valid stop+非零速 | 同sample stopped永不翻转，失效cache整体不可用，下一fresh才绑定；零差分无stop仍非物理停稳 |
| M7 估速与审计 | raw匀速/correction跳变；已有120向量保留；20ms量化界；实际raw轨迹形成的减速均速滞后；map四向/BOUNDARY/UNSUPPORTED/TAIL/source/scope | corrected保真一次、速度仅raw、地图heading不重复offset；20ms是估计候选而非物理噪声/瞬时制动证书；审计值保真且未写map |
| C1 真实C | 同最终 MATLAB 向量实际编译 adapter+Reference；NULL合同、逐字状态不变、plan/map不被修改、Reference首窗/重复/stop/恢复集成；静态无硬件符号 | 最终同SHA真实ARM结果；坏输入零输出、无重复推进/误DONE；无 getter/CAN/运行caller/执行器；不替代本机MATLAB前置 |

M1..M7 必须在产品 C 编写前完成适用 MATLAB 前置，回执含实际 MATLAB 版本/命令、开始结束、退出码、脚本 SHA、各组断言和可复放序列。MATLAB 无需模拟 NULL 内存或 RTOS。C1 是之后独立的真实 C 交叉证据；root 再完成唯一工程双目标编译与最终交接。不要只累计总检查数而没有分组/反例可追溯记录。

## 四、R / E / Q 与边界

- 已逐项核对 R：FIELD-01/02/06、OBS-01/04、RUN-01/02/03/04/07、PWR-01、MECH-01/02。此 adapter 不改变其规则含义、不提供物理验收证据；特别保持 start_id、F/mm、同区返回身份及无运行遥控假象。
- 直接相关 E：LOC-02/03/05/06/07、MOT-01/02/03/04、CAL-02；同时核对 RUN-05/06 的有限恢复与安全出口原则。20..100ms、原子 copied 值、估计速度和缓存策略均是工程合同，不写成官方规则。不增加比赛人工确认或永久锁。
- Q：FIELD-05、OBS-03、QR-04 继续“待技术答疑确认”；本审查无官方新答复，不改基线。
- 未闭环工程项保持 PLAN 原范围：真实停车点/运输包络/G-F布局、完整雷达bundle、停机仍可用实际轮速发布、真实 source skew/时延、mixed 校准、A/J制动、RTOS余栈及双起点整场。本阶段既无 measured 分支，也无 desired mixed 分类依据；map覆盖按源报告，Reference/future stage 的 mixed 不冒充校准通过。

审查结论：完成 M1..M7 并确认实际 MATLAB 通过后可按上述最小合同编码；当前不签署“adapter MATLAB完整前置通过”、产品C通过、运行接通或实车通过。
