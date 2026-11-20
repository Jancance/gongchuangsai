# 动作融合下一阶段：独立路径参考最小合同

日期：2026-11-17。源码基线：`9fce01db0bb68d8a6698788f2a74560d2e5afbf2`。本文件是源码核对后的规划，不是本阶段 MATLAB/C/整机验证通过记录。用户决定以 `../map-nav-calibration/INTERVIEW_DECISIONS.md` 为准；本轮 root 已限定先做到独立 pure reference 模块。

## 1. 本阶段交付边界

新增两源 `fusion_reference.h/.c`，需要时才拆出几何文件。模块只接收值类型输入和输出参考，不调用 RTOS、CAN、电源、运动许可、机械/视觉、雷达命令或旧 `MotionAction` 安装入口。MATLAB 对应模型先在本机实际运行通过，再编写对应 C；C 与同输入模型交叉检查。

本阶段完成以下最小闭环：固定合法有向折线 + 实际场地位姿 → 当前段 signed 进度/横偏/原路径剩余路程 → 路段速度上限及有界横移 desired world/body/wheel 参考；同时生成独立的起终点绝对 heading 计划。既有地图 50% overlay、四向控制倍率、trim、航向 PD 均不修改。

同向共线节点可非零通过。改变平移方向的节点先降到零；角格可输入并验证固定 heading 斜段，但斜段的两个折点仍不能无几何过渡地非零通过。曲线 blend、运行中 A/J 有界执行器和完整不停顿角格通过留到下一阶段。不能把此模块称为整车连续运动已接入。

## 2. 已核源码及其含义

| 当前证据 | 本阶段应遵守的边界 |
|---|---|
| `CHASSIS/USER/app_odometry.h` 的 `AppMapPose` 含 run/start/map、源 sample/valid/reasons、真实 heading 和 raw/corrected | 由后续 adapter 原子读取；本模块输入保留这些身份/健康事实，不建立第二套里程计，不写 pose |
| `chassis_odometry.h` 声明位姿参考为底盘几何中心、世界 X/Y、车前/左/CCW 为正 | body 矩形和所有停车点须使用同一个参考点；heading=0 沿 +X，90 沿 +Y |
| `field_map.c` 采用 480 mm 蛇形名义格，`AxisClear` 仅轴向静态矩形 | 不能复用为斜行、随机障碍或旋转扫掠证明；设备真实格边界与 G/F 外参仍待核 |
| `motion_action_plan.h` 容量 8，表达四向平移或独立转向 | 不把新参考强塞旧表，不扩大旧容量冒充融合执行能力 |
| `route_plan.h` 的速度试验入口为 1.21..20 cm/s；`runtime_route.c:75..81` 进一步限制平移 100..2100 mm、整数 13..200 mm/s | 分层报告实际范围。baseline40 的 400 mm/s 仅特定 tuple，不是通用 route 能力；本模块扫描 400 也不等于底盘已支持 |
| `motion_profile.h` 明确静止到静止、固定方向，未支持非零边界、运动中重规划 | 不用每次 `Prepare`/时间采样冒充基于地图剩余距离的实时制动 |
| `route_heading_reference.h` 已有同 run 成功目标继承，90 度 residual helper 仅 87..93 度 | 继承计划目标，保留真实欠/过转；不把 helper 外推成任意角恢复器 |
| `chassis_kinematics.c` 的 `Chassis_Inverse` 按四轮整体缩放 | 纯算术可复用，须输出缩放后实际可表示参考和 scale；不把原请求当已兑现命令 |
| `RADAR_V6/radar_client.h` PATH 最多 24 项、不含起点 | 本阶段输入是已展开的 F 坐标折线；协议/缓存/有向路径 adapter 后续实现，不倒序冒充反向请求 |

旧 `radar_path_corner_rules_20261112.m` 含旧包络假设、均分格和近工位放宽检查。本阶段可参考数学方法，不能复用旧通过结论、`stop_relax` 或旧尺寸声明作为部署依据。

## 3. 最小数据和接口

下列为合同字段，不要求逐字采用命名，不引入通用插件、回调或配置层。

```c
uint32_t FusionReference_Prepare(FusionPlan *plan,
    const FusionInput *input, FusionCheck *check);
void FusionReference_Reset(FusionState *state);
uint32_t FusionReference_Step(const FusionPlan *plan, FusionState *state,
    const FusionObservation *observation, FusionOutput *output);
```

`FusionInput` 只包含：路径身份（run/start/map/path）、显式 `model_only`；含固定首尾的 F/mm 折线；各段速度上限和停止点；固定原计划起始/终点 heading；调用方障碍盒；以底盘中心为参考的 body 长宽、位置及 heading 误差预算；所需横移增益/限速和轮速限值。root 已冻结容量为 32 points / 32 caller obstacle boxes；超容量拒绝，不截断。圆障碍可由同版本地图以包含其直径和定位误差的矩形盒保守表达，不新增第二套通用图元接口。调用方在 Prepare 后保持计划不变。

2400 mm 场界与四块黄色区域由本模块固定纳入几何检查，不能依靠调用方可漏传的障碍列表提供。额外障碍盒是本场地图输入，不替代固有禁止区域。

`FusionObservation` 只包含：同身份的实际 x/y/heading、sample/now、valid/continuity；实际或明确模型提供的世界速度/角速度/停稳状态。输入源必须标明，不能由前一帧 desired 命令冒充实测。单次缺输入、身份错误或 stale 结果只是不提供可用运动参考，不在此模块新增永久锁存或改变原故障恢复策略。

`FusionState` 只保存当前 edge、heading 窗口/方向和必要的源样本记号。`FusionOutput` 包含：状态及原因、当前 edge、signed along/cross、remaining、固定 absolute heading target、desired world/body/wheel、speed cap、wheel scale、model_only。输出不包含运动许可；无效输出不可被调用者当上一帧新鲜命令重用。

未确定的粗加工/暂存停车坐标、真实设备布局和运输轮廓可以显式名义参数进入模型；输出保留 model_only，不允许因数学通过自动变成可部署。当前既有已确认朝向为启停区/扫码/原料 +Y、粗加工 -Y、暂存 -X。

## 4. 固定路径投影与进度

对当前有向段 `A→B`：`L=norm(B-A)`、`t=(B-A)/L`、`n=(-t.y,t.x)`；实际位姿 `p`：

```text
q = dot(p-A,t)                    signed 当前段进度，保留负值/超调
e = dot(p-A,n)                    左侧为正的横偏
signed_s = prefix_length + q      诊断值，不单调钳位掩盖后退
remaining = L-min(q,L) + later_lengths
terminal_residual = L-q
```

remaining 来自原合法路径的弧长。`q<0` 时包含入口欠距；`q>L` 时不把超额直线投影计成下一拐段进度。最终超调使 remaining=0 也不能假定到位。端点位置、横偏、heading 和真实停稳须共同满足后续完成条件。

只在当前段及下一段做有向、带横偏约束的换段判断，不全局取最近点跳过障碍或自交路径。连续同向共线可直接移到下一 edge；方向变化时必须由调用者提供停稳并通过窗口后换段。重复 sample 不累加进度，不能按调用次数积分；恢复/新 run 由调用方显式重新开始，不自动把当前位置变成新终点。

终点、路径和 heading 目标始终来自原计划。当前 pose 偏离合法入口/可恢复走廊时返回明确恢复结果，不自行直连工位穿黄区，也不无限追超调。

## 5. 横移、限速与轮端参考

```text
v_lat = clamp(-K_cross*e, -lateral_cap, lateral_cap)
v_world_desired = v_along*t + v_lat*n
F = cos(theta_actual)*vX + sin(theta_actual)*vY
L = -sin(theta_actual)*vX + cos(theta_actual)*vY
heading_error = wrap(theta_planned-theta_actual)
```

横移限值须同时受当前合法走廊/误差预算和轮速余量约束；保计划 heading，不能通过旋转地图目标把横偏清零。输出 body 中的两个平移分量是 actual heading 下的结果，不能以计划 heading 代换。heading 小误差可由后续既有 HoldStep 执行；超预算先输出恢复/原地修角请求，不实现第二套 PD。

本阶段最简速度输出是 desired cap，可取段限速、接近下一速度限制/停止点的距离 cap 和 wheel cap 的最小值。合速度最大 400 mm/s 为本次数学候选；driver wheel / reserve 必须显式输入，均非硬件能力签署。若使用 `sqrt(v_exit²+2*a_hint*d)`，必须标明仅加速度理想 cap：未使用当前 acceleration、jerk 和源延迟，不构成当前状态可停车证明。不能给 v/a 递推再以最后钳位速度宣称同时满足 jerk。正式运行须后续以当前 v/a、实际源延迟和轮端余量建立 jerk-limited 执行适配。

非共线节点 `v_exit=0`；同向共线的限速变化须提前看至下一限制点，而非进入低速段后才变 cap。长直 400 mm/s、路口/靠站限速均为工程候选；输出四轮上限/scale 和受限后的等效 body/world 量，不声称动力学跟随或 3 分钟任务通过。

混合横移会离开纯四向标定工况；即使原世界路线四向，实际 yaw 欠转也会产生两车体分量，例如计划 90、实际 88 度时世界 +Y 的 400 mm/s 约为 body F399.76/L13.96。本阶段既有 map50% 不改，也不调用 `MapActionAccepted`；desired 参考只记录 MIXED/未验覆盖，不能宣称原纯四向系数在混合动作已验证，也不按两个分量各套一遍系数。未来真正执行混合动作前，必须先建对应补偿覆盖模型，明确如何保留用户已决定的主方向候选、旁向修正及 raw-only 区间，避免两套系数重叠；不能静默把所有含小纠偏的四向段改成完全不补。

## 6. 绝对 heading 计划与几何校验

以原计划 `theta_start/theta_goal` 计算变化：0 无大转；±90 在起点做；180 在起点及终点各做 90。180 先验 CW 两个窗口（负 90），若任一不合法再验 CCW 两个窗口（正 90），选后锁方向。中途固定 heading 为 `theta_start±90`，终点目标始终为原 `theta_goal`。例如原料→粗加工 CW 中途为 +X，世界向左可能变为车体后退，也保持用户 CW 优先决定，不能为速度或方向偏好自行改 CCW。

例如计划 `0→90→180`，实际第一次 88，第二目标仍 180；本模块不写实际为 90，也不生成 178。输出绝对目标/固定平移段 heading，后续执行器才按当前真实 yaw 计算受限转向。起终点只能使用显式停车点，不能默加另一转向点来规避未知空间。

固定 heading 运输段对整个线段连续扫掠校验；body 按 heading 误差预算和位置误差膨胀。采用保守世界 AABB/凸包与膨胀矩形障碍的连续交检测；平移不能全程用外接圆，因为合法起停中心距边界 150 mm 时矩形可通过、约 190 mm 外接圆会误拒。旋转窗口可单独用包络外接圆作保守检查；CW/CCW在此保守模型中可能得到相同判定，不据此声称精确方向扫掠。若未来按角度采样细化，须加入区间最大角点运动的外扩上界，单纯采样若干姿态不是完整扫掠证明。

检查场界、黄区、包含圆障碍的保守盒及原有合法走廊；触边按不放行处理。矩形 body 两组历史尺寸 283×254 / 270×250 和 0/10/20 mm 位置预算仅作敏感性假设，不直接冻结为真实运输尺寸。φ50 障碍是 R，位置/误差和完整占用来自同版本地图输入；缺完整图不能标合法部署。

## 7. 必须先运行的 MATLAB / C 对照矩阵

| 测试组 | 最小断言 |
|---|---|
| 两起点/八阶段/六类请求，名义工位 | 身份、固定首尾及朝向一致；未知 endpoint/layout 始终 model_only；不声称整场已就绪 |
| 前/后/左/右、非零 heading 37/170/-170、斜段 | signed q/e 正确；world→body 与逆变换一致；实际 heading 保真 |
| L/U 形、连续共线、反向/超调/自交 | remaining 是原path弧长，非直线终距；不跳段、不掩盖回退；折点未停不换段 |
| 0/±90/180、CW 和 CCW 几何、欠/过转 0.2/1/3 度、跨 ±180 | 绝对目标不漂移；180方向锁存；两个方向均不可行则拒绝；欠转不置名义 pose |
| 角格 0/4/20/24 四个方向输入斜段 | heading 固定；端点不漂移；折点速度零；扫掠不穿黄区/场界 |
| 400 mm 中央路口、圆障碍、触边和误差预算 | 验整车扫掠；净距不足拒绝；尺寸/位置/heading budget 增大不得变得更宽松 |
| 速度上限变化、短段、横偏正负、轮端饱和 | 减速 cap 提前；横移符号正确且有界；混合/斜行四轮上限与scale正确；不宣称 A/J 已保证 |
| 末段 q=L 但横偏/heading/速度不符、明显超调 | 不误报完成；不以remaining零伪造停稳；无无限倒车/追角 |
| stale/gap、错 run/start/map/path、重复 sample、NaN/Inf、零段、超容量 | 输出明确不可用/原因，失败不保留旧可用参考；无状态跨run污染，无静默截断 |

前置 MATLAB 回执记录版本、开始/结束、退出码、脚本 hash 和实际断言数；保留首失败再修。对应 C 以同输入 CSV/预期值做实际源检查，另外检查 float 边界及重复样本状态。无RTOS桩测试须如实标注；只有两个纯源参与构建不等于已接整车。

## 8. 接执行器的后续步骤

1. 本阶段 pure reference 通过 → 冻结输入、模型、C 对照和限制，独立审查，root 更新本地交接。
2. 新增只读 map adapter 与合法地图/有向路径 adapter → 验证原子快照、source age/identity、路径缓存和 map50% 混合覆盖；不另抢雷达/CAN owner。
3. 独立 MATLAB 验证 jerk-limited 平移/横移/heading 执行与四轮余量 → 扩展实际底盘执行入口；沿用既有 Stop/故障能力及单 owner，旧route保持原范围。
4. 对非零角格过渡引入最小几何 blend → 验切向/曲率和世界 v/a/j、扫掠及误差预算；不能把折线 velocity 突变隐藏在轮速缩放里。
5. 已授权的软件完成不等于硬件授权。新实车按明确版本/动作/次数/场地条件执行，完整装机载物低速起步，逐项 L 形、转后直线、角格、90/180、路口、扫码及双起点；首失败先保存 RAW，不自行重试。

## 9. R / E / Q 与未闭环

R 已核：FIELD-01/02/06、OBS-01/04、RUN-01/02/03/04/07、PWR-01、MECH-01/02。尤其 FIELD-06 约束主体投影，不能以中心点/模型 `stop_relax` 放宽；RUN-04/07 的15秒停滞与3分钟分别需要最终任务实证。

E 已核：FIELD-03/04/08、LOC-02/03/05/06/07、MOT-01/02/03/04、RUN-05/06、CAL-02。本文全部路径、heading、速度及容错算法属于 E，不冒充官方规则。

Q 保持：FIELD-05 / OBS-03 / QR-04 待技术答疑确认。工程未闭环另列：真实 G/F 布局与外参、粗加工/暂存停车点、当前运输包络和参考点、混合补偿、源时延/漂移、实际 A/J 制动/轮端能力、栈余量、任务等待及双起点整场。

本规划未改固件、未访问硬件、未下载/reset/发车、未操作 Git 网络；未运行本阶段 MATLAB/C。最终文件/API、测试数量及通过状态以编码阶段实际回执为准。
