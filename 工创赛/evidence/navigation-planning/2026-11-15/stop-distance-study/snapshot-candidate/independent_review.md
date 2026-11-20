# 176 B末段诊断候选最终独立复核

2026-11-15。状态：**LOCAL_OBSERVATION_CANDIDATE**；**chip_cycles_measured=false，terminal_snapshots_measured=false，B0=FAIL_NOT_IDENTIFIABLE，B1_enabled=false**。

结论：最终冻结版本未发现阻断本地交付的问题。三个事件位于真实控制函数的适当位置，原控制字节、固定输入下的距离状态及专项关闭运行内容保持。它是补齐观察的候选，没有实施B1前视、停车控制或新参数；不代表地面停车效果、芯片耗时或真机终点快照已经验证。本独审只读产品/既有证据，仅新增本文；没有运行implement/会覆盖原证据的runner、重构建、Git写入、联网或硬件操作。

## 最终绑定与改动隔离

|产品源|最终SHA256|
|---|---|
|chassis_auto_test.c|`ada294ab54d230e1c791a701b99c9007dc3c15ee7711633d63fe9855b4de122b`|
|chassis_auto_test.h|`aa431918fb5b111d083647320b7d042ef60fb5d19c05c184c5d101c9882dbefe`|

本次仅这两份产品源变化。独审逐项检查所有诊断插入，按 `source_changes.json` 的精确片段反向剥除后，两源分别与保存before **字节完全一致**：auto.c before为已独审的浮点计时源 `49a1c2bf…abcd94`，header before为 `8d07cec5…6c68bd`。不是只检查输出近似相同；原S2.5控制、follow、floor1RPM、距离系数、参考、限幅、STOP/VM及故障分支都没有被重写。

原104 B浮点计时块保持，hwt101.c/.h哈希与上轮冻结一致，旧HWT三scope/28fields没有扩展。新增诊断没有控制值回写、额外CAN轮询、周期UART发送或动态内存分配。GBK roundtrip通过；auto.c保持2144 CRLF且无裸LF，header保持原混合换行（257 LF，其中原有1个CRLF），没有批量转码。

## 三个真实事件的源码落点

**入口槽0：**在 `Auto_DistanceCheckLocked` 成功建立、校准共同时间mean并采样原reference之后，原arrival判断之前。条件明确使用 `elapsed>=boundary_s[4]` 且 `mean>=target-120`，没有把匀速时的零加速度误判成减速入口。只保存第一次；若本次已经直接满足arrival，可以同一次留下入口与arrival，原控制判断不变。

**最新冻结槽1：**在 `ChassisAuto_CanStep` 已成功形成新四轮命令、写入applied base/heading并进入START之后记录，且要求入口槽已存在。只有真正采用新批才更新；正在发送其余轮、请求与已用批未变等路径不增加freeze_count。此时pending/applied base可相同，表示软件采用该命令；**不是四轮都ACK成功，也不是实测速度**。STOP仍可能中断此冻结批，槽名和parser保留 `NOT_complete_ACK`。

**arrival槽2：**在原arrival分支中、设置 `s_distance_reached / s_finish_requested / s_stop_due` 之前。此时reference已由原逻辑采样，本次follow尚未推进，故保存的是新的reference与旧follow/follow_tick；没有为观察再次推进控制器。随后仍执行原reached/STOP赋值和return。arrival槽一旦有效，其他诊断槽也冻结。

记录函数只在原锁内复制已有状态，reference_rpm为已有reference速度除原factor的一次观察换算；没有新增参考采样或控制更新。原有效baseline40接受路径才设置s_lateral_enabled并reset，helper另核active/profile/session；拒绝或session不匹配的start不擦除原记录，接受非专项目标也保留原记录及其原session。STOP/fault/idle会按原路径退出，未添加诊断清零。slot0/1/2仅由固定静态调用点使用，没有引入对外任意索引接口。

flags分清事件前profile active/done、run_started、reached/stop/finish、共同时间对齐及400ms年龄；arrival前态bits8/16/32仍为0，不把“arrival槽有效”伪装成记录前已reached。共同时间和run/follow tick保留uint32，模减跨回绕有效，没有转float保存时间戳。

## 独审发现、修正与栈开销

1. 首版Reset用176 B局部empty再复制到全局，独审在实际Integrated反汇编确认frame184 B；已要求修正。最终采用直接对静态记录做176次有界volatile字节清零，再写version/session。重新构建的callgraph确认 **Reset local/max stack=0**，没有把大临时对象隐藏为“只有176 B全局开销”。Record local/max为12 B。
2. 首版before回执错误地把tested_auto SHA写成当前候选；生成元数据已修正并重跑。当前before绑定49a1…、after绑定ada294…，与各fixture实际源相符，旧错误没有继续作为同源证明。
3. 对整镜像ZI的初始假定176 B不准确。最终map/回执单列对象与对齐差：auto.o ZI **4360→4536（+176）**，整镜像ZI **119024→119196（+172）**，差4 B为链接对齐抵消，未为凑176 B新增padding。

独立读取新旧编译器callgraph，原主要路径local/max depth保持：BeginPreparedTranslation **48/376 B**，CanStep **224/608 B**，DistanceCheck **96/192 B**。这是静态调用图证据，不能替代真机stack高水位、IRQ/FPU上下文及运行时栈验证。新记录的有界赋值和一次除法仍会占用原临界段时间，不能宣称实际调度和耗时完全零变化。

## 独立ARM固定输入重放与布局

独审核对现成fixture C/AXF哈希，直接在独立Unicorn实例初始化分配节并运行已有ARM镜像，没有重新编译或覆盖证据：

|镜像|独立执行结果|实际绑定|
|---|---|---|
|before浮点计时版|return0；**1402 cases / 16714 assertions**|auto SHA49a1…|
|after三槽诊断版|return0；**1420 cases / 16755 assertions**|auto SHAada294…|

两次实际ARM运行导出的 `distance_control_trace[32][12]` 与各自保存bin逐字节一致，前后 **384字段/1536 B完全相同**，SHA均为 `908ed2248de6fde5e7ac90d93ee6b58310a65dbdedf32187cfe53ac8b0804188`；entry F/L/C与FF观察也一致。这32个构造共同时间mean状态用于代码隔离，不是保存四趟的逐时刻重放；其时间网格止于7.75s，arrival另用构造的8.05s/2104mm状态验证。

本轮实际提取执行 `Auto_DistanceCheckLocked`、`Auto_ProfileFollowRpmLocked`、Begin及CanStep；没有用仅同名的模型代替事件落点。外部CAN/power/RTOS/source readiness、Auto_CheckLocked等仍为stub，不能扩展成板端调度或轮地验证。

实际ARM sizeof和偏移检查通过：头32 B、槽48 B、总176 B，common_mean偏移16、pending base偏移36、flags偏移44；编译期size断言及release ELF176 B符号相互核对。独审从重新执行后的ARM内存取得176 B blob，与保存 `synthetic_snapshot_from_ARM.bin` 完全一致。

该**合成**blob中arrival为now9050/run1000/common9050/follow8310，reference **14.980769 RPM**、follow **−0.973274 RPM**，flags197且reached/stop/finish前态均0，说明该落点确可保存参考与旧follow。它不是四趟实测内部状态，不能用这个构造输入把B0改成PASS。

## 三目标、RAM和新ELF双符号

三份实际构建日志均为0 Error(s)/0 Warning(s)，AXF哈希与最终构建回执匹配。独立比较所有非空SHF_ALLOC节，专项关闭目标与原浮点计时版 **全部相同，包含ZI**；启用专项目标Code各+580 B、RO/RW不变、整ZI各+172 B。

|目标|Code / RO / RW / ZI bytes|176 B快照地址|104 B计时地址|
|---|---|---|---|
|baseline40_integrated|102676 / 6572 / 2952 / 119196|`0x2001306c`|`0x20012768`|
|baseline40_offline|102028 / 6564 / 2760 / 119196|`0x20013034`|`0x20012730`|
|baseline_off_integrated|11974 / 470 / 960 / 25576|无该符号|无该专用符号|

Integrated AXF SHA256：`5e803d0def9e33155752d4ceec71b9ca93ab903311407185d5d7af3c4880a185`。

旧浮点报告里的0x200124c8/0x200123f8仅绑定旧镜像，**不能用于本新ELF**；保持代码不等于保持RAM地址。两个release getter均未链接，不能把源中声明当成硬件可调用接口。新ELF地址也不能跨下一次构建硬套。

## Host解析与未来取数边界

独审直接导入无硬件行为的parser，核验ARM合成blob、全零未初始化、uint32回绕、NaN和未知flags。回绕样例的elapsed/common age/follow age正确为21/11/6ms；无效数据不被标为记录，硬件来源始终false。valid_mask只说明槽是否被写过，不证明物理量可信或取数一致。

后续现场操作在用户最新明确的本诊断版一趟及RAM读取授权内执行，必须绑定上述镜像，在同session的DONE/backend inactive/fault0状态前后核验，完整读取104/176 B各自稳定双读并保存原件；**不halt、不reset、不改变VM、不mask IRQ、不调用未链接getter**。状态/一致性不能证明时标为unstable，不能把活跃RAM单字段拼接后称为一致快照。故障时记录本身保留，不表示本次完成态导出规程已经验证了故障态取数。

此前实际MATLAB布局/回绕/reset/hold前置验证先于诊断实现，已在B0/spec独审中核对。此次C/ELF与ARM补齐了当时尚未核验的布局和代码隔离，仍没有真实终点快照、M4耗时、ground或停车控制改善证据。B0不可辨识、B1未运行的结论继续有效；本报告不扩大用户最新的一趟及RAM读取授权，失败保留首故障，不自动重试。

已核R **RUN-01、PWR-01**；E **LOC-02/03/05/07、MOT-02/04、RUN-06**；Q **FIELD-05、OBS-03、QR-04** 继续待技术答疑确认。176 B诊断、120mm观察窗口与数据有效性口径均为E，不是竞赛规则。仅本地独审完成，Git提交/同步由父任务按用户授权另行执行。
