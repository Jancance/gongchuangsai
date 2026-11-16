# 第二趟同条件左移：窄重复计划

2026-11-16；gpt-6.1-sol xhigh。仅规划；实际复位、运行及读回由device执行、high独审、root统一推进。本文件未操作硬件。

## 目标与冻结项

用户第一趟原话：**“向前  8.5cm，左移  差2cm再测一次”**。保留原文；前向+85 mm、左移少20 mm即2080 mm只是当前解释，测量不确定度与同窗参考未知，不当作严格地面真值，不据此改补偿。用户随后明确**“已回同起点，可开始第二趟”**，同朝向、同载物；第二趟已授权，不重复询问。

- 同一候选AXF SHA256：`0d5e2b752c92db9094393723b42411cbf02dbdc2f3950ac4c4ca6847fe73242f`。
- 同一加载镜像：PT_LOAD_0，0x08000000起115340 B，SHA256 `ddc6dd63e925ec44921290f726359bb12377b78736916a1927dfd572dd2680f5`。
- PLAN9 / `left210_40` / kind5 / dir1 / 2100 mm / 400 mm/s参考 / RPM102预算 / a150 / jerk300 / HWT原纠偏 / feedback130；前进PLAN8不执行。
- 保持第一趟已审客户端、reader/decoder、采集200 Hz、terminal+500 ms及10 s/2000 POST上限。**不重MATLAB、不重构建、不重下载、不改固件/控制/补偿/KF/Mahony。**
- 本次动作预算：**download0、明确reset1、START1、左移1趟**；无自动重试、第三趟、回起点或额外方向动作。

第一趟证据已核：12:41完成执行汇总；download1/reset0/START1，software_complete、powered尾19.985 s有效，末轮零/故障0/VM1/PE2=1；RAW2032=PRE100+POST1932、quality0/drop0、双读稳定、reason1/terminal+.5完成，无地面同窗停稳证明。完整第一趟数据已落盘，因此本次允许用一次reset清除冻结capture并开始新会话。

## 顺序与就绪标准

1. **独立绑定及首记录。** 所有新脚本、回执、串口流、RAW与SHA仅写本`lateral-field-run2`目录；保留run1不动。候选文件与client/reader/decoder SHA核对第一趟已审绑定。当前无线/唯一probe只读核实；第一趟为COM17、UID6afaae24，继承为期望而非实时证明，实际不匹配时停止动作，不猜其他设备。无线与现场条件用户已确认，不再申请权限。
2. **一次明确reset。** 新wrapper限定绑定UID且独占started/receipt，检查本趟reset尚未尝试，记录调用起止与结果；只reset1，不包含下载、ARM、START或隐式二次reset。首调用失败或执行不确定先保存首状态，不能盲目再reset；read-only检查可以区分已执行/未执行，但本预算不自动扩成第二次。
3. **复位后fresh idle/PRE。** 使用本趟新串口流，清除接收残帧，不复用run1 token、ready、tick、yaw或capture身份。等待静止至少2 s、PRE100；同一新流新鲜HWT健康、angle/rate age在现客户端/固件既有限值内、idle/inactive、轮零、VM0/PE2=0且CAN/RTOS/safety fault0。读当前capture应state0/session0/POST0/quality0、终态seen清零；RAW READY2/error0、WHOAMI233/acc56/gyro40等配置匹配候选，drop0且序列更新。闲置odometry尚未建立运动参考属预期，不伪造成运动定位有效。
4. **当前候选/Flash身份。** 第一趟12:40只读读回已经核对加载Flash与上述候选一致，之后无下载/改源；本趟沿用该映像依据，核AXF与绑定SHA未漂移，**不再次完整读取Flash**。复位后只读核当前RAM/采集配置/新鲜状态及core RUNNING，禁止halt/resume/目标写入。身份矛盾则停止，不自动下载“修复”。fresh idle/capture结果必须来自本次reset之后，旧run1运行态不能替代。
5. **新PLAN9→ARM→START一次。** 初始默认PLAN8仅为复位后的预期观察态；沿同一已审客户端选择PLAN9，取得新鲜精确BF1 `(9,102,2100,400,150,300,130,12779)`、当前健康TM及匹配token后才START1。运行严核PLAN9/kind5/dir1/目标2100000 milli-mm、参考40000 milli-cm/s及102/130范围，保留现KEEP/故障STOP能力。运行失败/取消/断链或首故障保存原件，不自动补发START、不重跑。
6. **留电读回及本趟结束。** 正常软件完成后约20 s receive-only powered尾；分别记active、轮速、fault、VM/PE2及现场物理停稳。留电、不移车、不复位，用本次result_summary中的非零session绑定一次no-reset/no-halt/running-core读回；双bank及独立tail稳定双读，存原bin/decoded/SHA/receipt。PLAN9未采前进补偿tail可全零，如实标未采，不影响独立RAW完整性，不伪造mask7。安全需断电时现场立即处置。

本趟reset后的session可能也等于1；身份键必须含**本目录＋本次reset/启动证据＋session**，不能与run1数值1混配。读回expected_session须取本趟真实运行结果，禁止硬编码继承1。候选、reset、START和读回每个阶段均独立回执，由high审实际阶段证据，不以规划签名冒充执行结果。

## 完成判据与报告边界

软件完成/轮零/VM保持与物理停稳分别列。核RAW序列、quality/drop、源窗、reason、terminal_seen/terminal_tail_completed及10 s容量是否先到；第二趟数量无需等于2032，也不要求重复第一趟时间。截断只报告截断，不因此加停车或追加第三趟。数据安全保存后明确通知可断开/复位，再记录用户第二趟同点前向/左移尺量；不提前断言改善原因或地面精度。

只做操作次数、运行健康和完整性汇总，不展开新融合性能重放。第一趟+85 mm与“少2 cm”不用于本次调参，后续是否分析/改补偿按用户指令与证据决定。

已检查：R RUN-01/PWR-01/MECH-01/MECH-02；E LOC-01/02/03/05/07、MOT-02/04、RUN-06；Q FIELD-05/OBS-03/QR-04仍待技术答疑确认。沿用已完成MATLAB/软件独审，未改变适用模型或实现，因此本次重复不重做前置。未闭环项：第二趟实际运行/读回、现场地面同窗与测量不确定度。仅本地文档，不fetch/commit/push，dirty/编码换行保持。
