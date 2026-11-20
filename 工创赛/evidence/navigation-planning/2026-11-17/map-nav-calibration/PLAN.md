# 四向导航残差接入连续地图：实施合同

> 最新用户逐项决定以[访谈决定](INTERVIEW_DECISIONS.md)为准；下文历史待回答或建议不覆盖该决定。第一阶段软件已完成，最终实现/验证状态见[交付报告](REPORT.md)。

日期：2026-11-17。规划基线：a5f2eccb7ee875968ee1773338995583fc68b1df，分支 codex/chassis-navigation-plan。本文仅规划；MATLAB、实际 C、构建和实车结论由各自实际回执提供，不在此预先填写通过。

## 范围与授权

用户要求将现有惯性导航直接用于连续地图，再规划动作融合。本阶段应让真实地图坐标使用四向50%主距和旁残差，不能只在原单段210cm调试scope中接上后称整场已用。后续动作融合、地图位置闭环、速度入口、下载、reset和发车均不属于本阶段实现。保留本地其他未提交成果，不fetch/push。

最简实现是在地图独立raw轮/HWT累计位姿之上维护一份分段修正，不另建滤波器、不改变raw积分，不修改原单段Navigation40/ICM锚/控制距离比例。地图独立默认关闭、候选显式启用；地图补偿开关依赖APP_CONTINUOUS_MAP_ENABLE，不依赖APP_BASELINE40_ACTIVE。

## 已核事实与源码入口

以下行号对应本规划基线，产品修改后应重新核对。

| 事实 | 入口 |
|---|---|
| 两个工程目标均链接活动CHASSIS/USER，不是根USER旧副本 | gongxun-f407.uvprojx:767、937、2121、2291附近的FilePath |
| 连续地图独立context、请求新run、原子getter | CHASSIS/USER/app_odometry.c:16、29、42 |
| 首次轮/HWT建基准、cache择新、同run GAP锁存、原子发布 | app_odometry.c:76–161 |
| 地图仍RAW_WHEEL_HWT，calibration_scope/nav_applied为0 | app_odometry.c:57、159；app_odometry.h:16–21 |
| 原四向strict scope排除runtime route，只接受完整debug tuple | app_odometry.c:175–200 |
| 原单段修正是reference净raw位移一次投影和max(q,0) | app_odometry.c:334–377 |
| 原始轮/HWT积分已经含前向尺度、中点角和sinc | chassis_odometry.c:158–172 |
| 成功平移、两种转向提交，以及完成/取消入口 | app_motion_runtime.c:350–383、394–420、444–469、423–441、518–540 |
| runtime route是多段路径，当前合法四向distance和转向类型 | runtime_route.c:75–82、123–145；route_plan.h:61–79 |
| 工位/起点名义坐标、实际HWT对应的地图heading offset | vehicle_tuning.h:13–35；app_odometry.c:105–108 |

## 适用范围不能等同实车标定范围

1. 地图候选按已成功接受的动作身份识别四向distance直线，不再使用旧debug计划号、单段数量、2100mm或40cm/s作为地图功能门禁。合法普通route的四向段因此可实际获得修正。
2. 与原样本完整距离、速度、方向、trim和版本条件匹配时，标为参考工况；其余合法四向直线段显式标为工程外推。参考工况也仅有已有离线候选证据，新导航未获实车精度验收。
3. 定时平移、斜行、显式转向、未来平移转向均不套经验系数；raw轮/HWT照常累计，已有修正不撤回，覆盖状态明确记录。当前无斜行/平移转向动作接口，不为本阶段扩展动作接口。
4. 首段theta0取与raw共同样本同源的实际地图heading，不把命令90度当已转到90度。显式转向或未来混合运动标签不套四向经验修正；四向段内实际航向偏离原标定条件的有效性仍未验证。本阶段不增加3度门限；若后续增加角度抑制边界，须先补对应MATLAB验证。
5. 主距/旁残差均固定50%；不拟合新权重，不叠加控制距离比例或右1%控制前馈到导航修正。

## 数学合同

地图坐标+X/+Y及逆时针heading沿用现有定义；车体前F、左L为正。段首raw地图坐标p0和实际heading theta0固定；eF=(cos(theta0),sin(theta0))，eL=(-sin(theta0),cos(theta0))。

每次从本段raw位移p_raw-p0得到净进度q：左取eL投影、右取负eL、前取eF、后取负eF，最后统一max(q,0)。不用绝对路程、逐样本正增量之和或已校准坐标作为输入；短时反向噪声不会双算。

| 动作 | 每raw净1m主方向修正 | 每raw净1m旁方向修正 |
|---|---:|---:|
| 左 | 向右3.351160mm | 向前8.952769mm |
| 右 | 向左3.171487mm | 向后5.505865mm |
| 前 | 向后1.407280mm | 向右3.767248mm |
| 后 | 向前2.286337mm | 向左3.070165mm |

以上仅为已有app_config.h:74–101常量乘0.5后的方向解释；产品复用已有常量，不再建一组四舍五入常量。

p_nav = p_raw + C_committed + C_segment(q,theta0)。当前段每次纯重算，换段只把最后有效已发布C_segment固化进C_committed。不同方向/heading的段各自旋转后累计；转向后不能拿上一段theta0修新段。不同getter调用次数、局部SetPose、ResetRecords、plan/session更换不改变结果。

## 动作身份、源时刻和关闭边界

只在真实成功的Move提交和两种Turn提交处发布原子tag：唯一epoch、begin_ms、动作方向/距离/速度语义。可用AppOdometry窄API接入，避免扩大AppMotion_Live。所有成功动作都产生新epoch，包含不支持经验补偿的定时/转向动作；失败提交完全不改tag。不能用先增加后失败的g_combined_test.index、stage_ms或请求值当已接受动作。

CAN owner消费tag并拥有全部修正状态；提交线程只写邮箱。新tag到来时先固化上一段最后有效已发布修正；首个raw共同sample_ms>=begin_ms仅建立新段基准。跨边界区间raw照常积分，经验修正跳过并显式标记，不插值伪造动作边界精度。若CAN错过多个tag、共同样本晚于关闭、或无法确定区间归属，也不把旧尾算作新段。

最终关闭合同以root当前决定为准：软件运动完成、取消、故障的可观测关闭即固化最后有效修正，不等可能长期不调用的MotionFinish。可在CAN积分前读取一次原子phase快照或在真实状态转换处接关闭hook，选择可靠且改动最少者。关闭后停车尾和idle仍保raw位移，但不再按旧方向增长经验修正；公开这个未补范围，不增加经验尾时窗。HR_DONE不证明现场物理停稳。

## 生命周期、GAP和元数据

- 真正新run：清已累计/当前修正、段身份和覆盖状态。getter等待新run时不得带上上一轮校准坐标；start0保持NO_START，不造工位原点。
- 本轮跨段、局部ResetRecords/manual SetPose及换PLAN：不清地图原点或已累计修正。
- invalid/缺包/坏源/已知供电epoch变化：沿用现有sticky GAP。同一run不自动恢复，不用补偿伪造valid。
- getter触发GAP和CAN中段被抢占时，raw与修正必须退回同一份最后发布快照；不能只退raw却留下未来corr。getter只复制/判老化，不积分修正。
- pose的heading/sample_ms/age/valid/reasons/continuity_lost和轮/HWT时刻继续来自同份raw状态；只改发布的XY和明确校准元数据。保留raw地图context供诊断，原local nav与ICM锚不改，绝不把已修正map再送入原local修正。
- source须能区分raw与带经验修正；calibration_scope能表达参考、外推、未支持或边界未补。nav_applied说明本轮已有经验修正被使用，不能把有历史修正但当前转向的地图冒称纯raw，也不能把raw有效性当校准实车验收。

## 实施顺序和验证门禁

1. 本机MATLAB先实现上述纯模型和生命周期矩阵，保留脚本/输入/起止/exitcode/检查结果。通过后才写相应产品代码。
2. 只改app_config、app_odometry及必要动作成功/关闭hook；保留GBK/CRLF和原单段行为。独立地图开关在有runtime route、无baseline40的有效整机候选中显式启用。
3. 实际ARM C按同输入交叉检查；开/关及baseline40 local nav开/关组合核对。统一工程构建真实整机配置，ICM45686_SAMPLE_ONLY=0，核来源/hash/映像。原始dirty文件保持，不下载。
4. 独立审查应复现GAP抢占、新run、动作边界和跨段累计；完成交接并停在本地软件结果，后续实车另按明确动作授权。

| MATLAB/实际C必须覆盖 | 验收 |
|---|---|
| 四向、0/小距/1500/2100/route长度和不同速度、theta=0/90/-90/±179 | 符号/旋转正确，参考与外推元数据正确，不因非debug scope失去全部接入 |
| L形、左右往返、两同向段、四向闭合、转90/180后新段 | raw连续；修正按段累积，旧corr不丢、不重复，不按名义角伪造地图 |
| 正-反-正净位移、q<0、重复sample/getter、零位移 | max净投影语义一致；无abs路程/调用次数漂移 |
| 提交前旧sample、首sample跨begin、同ms、新旧tag跳号、失败提交 | 不把旧尾算新段；边界未补有标记；失败不改变上段 |
| DONE/取消/故障、无新动作长idle、结束后raw停车尾、turn/timed/diagonal | 关闭后corr固定，raw仍连续；不支持动作无新增经验补偿 |
| new run/回绕/start0、local reset/manual pose、换plan/session | 只真正新run清地图校准；局部操作隔离 |
| stale/gap后新鲜源、getter抢占CAN计算/发布、坏源/供电epoch | raw与corr共同冻结；不恢复valid、不跨gap偷推进 |
| map nav与旧local nav分别开/关 | raw源/旧local结果不变，ICM不双加，map-only能服务普通route |

软件通过不说明速度/载荷外推有效、不说明物理停车尾已校准，也不说明整场地图位置闭环已完成。后续实测需要同版本四向新组、至少一组L形和转向后续段、双起点整场，以及真实轮/HWT时延、栈余量与累计漂移证据。

## R/E/Q与未闭环

R已检查FIELD-01/02/06、OBS-01/04、RUN-01/02/03/04/07、PWR-01、MECH-01/02。E涉及LOC-02/03/05/06/07、FIELD-08、MOT-01/02/03/04、CAL-02及本文全部补偿和元数据选择。Q FIELD-05/OBS-03/QR-04继续待技术答疑确认。地图/雷达坐标外参、跨速度/距离/载荷经验有效性、真实时间同步、停车尾和混合运动校准是工程未闭环，不能改写成官方规则。
