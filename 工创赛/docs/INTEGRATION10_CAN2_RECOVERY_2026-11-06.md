# 整合10 CAN2通信补丁与一次实机复测（2026-11-06）

用户明确要求：先核对另一AI分析、修改代码缺陷并复测一次；若仍不通，用户先检查接线，最后才用USB-CAN。按此顺序执行，不因补丁编译通过宣称通信已恢复。

## 交付与当前固件

基于前一完整单件候选b56a5070…，只修改两个固件源文件，编码/换行按原文件保留：

- `CHASSIS/SYSTEM/CAN/can.c`：CAN1硬复位前保存已分配给CAN2的过滤器14～27与相关模式/路由/启用位，复位后恢复，不覆盖CAN1新建的过滤器0，也不改变14/14归属。
- `HARDWARE/DM_CAN2/dm_can2.c`：CAN2配置未变化时复用初始化；验证/按需修复本模块拥有的过滤器14；NART关闭、ABOM开启，同时保留原30ms发送截止、取消邮箱及Bus-Off故障出口，电气恢复不恢复旧运动；新增LEC/TEC/REC/BOFF及首个可采样错误快照。

未改变运动、回零、VM、夹爪、底盘、视觉算法或保护阈值；外部正式实车工程源码未修改。当前候选入口：`tmp/integration10_can2_recovery_20261106/project/gongxun-f407.uvprojx`，在线目标仍为`Vehicle_LiftRuler`，Flash SHA256：`60daa57f712b64a601a90783acd2ee5e2193175286e7012cdf72091d880287ba`。

辅助供电只读脚本已支持明确选择该候选，并修正必须等待本轮read_cycle推进后才接受结果。正常/失败/异常出口都关闭空载诊断VM，正式持料保位流程不使用此脚本。

## 验证

实施前用户本机MATLAB模型36检查通过，检查共享过滤器保留、服务间隔影响下的截止/取消与恢复后不重发旧事务；图为模型，不是实车通信或热验收。真实CAN1初始化/过滤器备份与完整DM参数查询C代码，模拟寄存器/总线/时钟进行20项检查通过：重复读取不重置外设、CAN1复位不丢CAN2过滤器、损坏过滤器修复、短暂/持续无ACK、Bus-Off取消、显式重试、取消、错误归属拒绝、tick回绕以及首错误保留。设备模型不能替代物理总线。

在线/Offline完整构建均0错误0警告。源码对照仅上述2文件有差异，无冲突标记。现场换版前读取原b56a5070…，VM0/PE2低、空爪空闲。21:51:15 Keil下载Verify OK/Application running，随后逐段Flash比较一致。下载包含复位；未执行回零或运动。

## 一次实机只读结果

仅恢复实际VM供电请求，等待guard及1秒启动时间后提交一次节点1参数读取。结果：DMstate5/result4/read_cycle1/tx_count1/rx_count0、feedback_valid0，LEC3、TEC128、REC0、BOFF0、ESR0x00800033。MCR0x00010040验证NART0/ABOM1；硬件初始化计数1、过滤器修复计数1。tx_count1是软件提交次数，自动重传发生在硬件，不能说只有一个总线发送尝试。持续无ACK后发送邮箱被取消，原角度0无效，未伪造健康或放行运动。

测试结束finally已撤销真实VM请求，实际读回VM请求0/PE2低，`VM_off_after_test=true`；这是软件请求/引脚证据，不替代电压精测。本轮没有发送4310使能/失能、回零、旋转或35运动命令，也没有再次抓料。35之前的异常现场停止确认保持，但本次无热验收。

结论：**三类通信加固已编码、验证并下载，本次仍未解决外部无ACK。** 当前仍空爪，VM关闭，各机械参考重启后无效，载盘PWM0。按用户明确顺序停止继续软件重试，先由用户检查线缆/端子/压接/同名H-L连接，再决定USB-CAN。电阻值只能提供终端/连通线索，约60Ω不证明所有接线及信号完整；ACK错误也不能单独排除速率/引脚链路。不得采纳另一AI“已证明完全不是代码”的绝对结论。

## 未闭环与依据

独立审查另复现的“只读被升降原点门槛阻挡、旧查询失败被新事务提前消费/锁全流程”仍在原单件协调器中；本次按所给分析只改通信两文件，现场通过独立只读接口复测，未声称所有整合10状态机缺陷都已修好。这些缺陷不解释此次底层外部无ACK，后续单独处理。真实4310通信、单件转运、放料深度示教、三件与第三段仍未闭环。

R核PWR-01/RUN-01及原单件范围；E核RUN-06/LOC-03/CAL-05。本次是人工看护通信诊断，不是正式自主比赛验收。官方Q保持“待技术答疑确认”。共享资源依据：[ST双bxCAN过滤器说明](https://community.st.com/stm32-mcus-60/stm32-in-dual-can-configuration-bxcan-filter-bank-explanation-and-relation-with-can2-start-bank-parameter-136737)、[ST RM0090](https://www.st.com.cn/resource/en/reference_manual/rm0090-stm32f405415-stm32f407417-stm32f427437-and-stm32f429439-advanced-armbased-32bit-mcus-stmicroelectronics.pdf)；具体代码/寄存器行为还由本次实际C注入与实机读回交叉核对。

证据：`evidence/integration/2026-11-06/can2-recovery/`下MATLAB结果/图/日志、host-result.json、两构建日志、ready.json、download.log；一次实机完整原始结果：`evidence/integration/2026-11-06/integration10-complete/can2-recovery-single-retest-power-readonly.json`。准备/验证脚本分别为`prepare_can2_recovery_20261106.py`、`check_can2_recovery_20261106.py`、`validate_can2_recovery_matlab_20261106.m`。

团队隔离fetch仍9f4ed20，根工作区无提交/remote，未pull覆盖本地成果或AGENTS。本轮仅本地交付，未提交/上传GitHub。


## 用户要求现场再看一次灯态的复试

用户“在试试我看看”明确新增一次观察授权，已提前通知用户看VM与4310灯，执行user-observe-retest-01一次。Flash仍60daa57f…匹配、升降/运动请求0，短时VM请求1/PE2高/guard就绪后读参数；read_cycle2、仍state5/result4/rx0/LEC3/TEC128/REC0/BOFF0。硬件初始化计数仍1，没有重复重置CAN2。finally确认VM请求0/PE2低，未复位/烧录/使能/失能/运动，未自动循环。灯态由用户观察，当前待回，不编造先绿后红或VM灯已亮灭。原始记录integration10-complete/user-observe-retest-01-power-readonly.json。按此前顺序，仍先查线再决定USB-CAN。


用户随后确认本次“VM灯亮后灭，4310先绿后红”，进一步确认“VM还亮着时就变红”。排除“仅关闭VM时才变红”的解释，尚未定位触发源，不能凭灯色认定代码已发失能；本次仅发送7FF参数读取帧，未收到电机反馈，不补造电机ERR/使能状态。不得把回环成功或无ACK作为完全排除软件/速率/引脚链路的证明。当前VM已关闭，不自动增加上电次数，下一步保持原定先断电查线；独立USB-CAN在后。建议当前无电时先量4310端H-L电阻、确认端子和压接，以实数记录，不以60Ω单值宣称整条线正常。


用户再次明确“在尝试一下”，执行user-observe-retest-02一次短时只读；60daa57f…Flash匹配，未复位/换版/运动。当前本轮read_cycle1（未继承此前cycle值），仍DMstate5/result4/rx0、LEC3/TEC128/REC0/BOFF0/ESR0x800033，初始化1/过滤器修复1。供电请求及PE2在测试期间为高，finally VM关闭确认true；现场灯态本轮尚未获得新反馈，不沿用前次观察冒充本次。证据integration10-complete/user-observe-retest-02-power-readonly.json；通信仍未恢复，不自动再试。原定断电查线及最后USB-CAN的顺序保持。


用户报告4310端H-L为32kΩ（32000Ω）。此前已要求整车/USB断电，但本次接插件是否仍接整条原总线尚待确认，不能直接认定两端终端缺失或线断。已核原图纸专题P1_CAN_VM_SCHEMATIC_RECHECK_2026-10-09.md：主控CAN2 U3/TJA1050、R11=120Ω；历史图纸/读数不替代当前实装。若全线连接且断电，32kΩ说明测量点未见约120Ω的有效通路；若只量拔开的电机本体，可能仅为收发器输入，不能推出线束问题。已向用户询问测量拓扑，未加电阻/改线/再上电，VM保持关闭，代码仍60daa57f…；user-resistance-32k.json记录原始输入和不确定性。


用户补充“俩个都试过都是32K，没上电测的”，接受用户断电确认和两个接法约32kΩ，不擅自补齐三选项中所有组合。接续只安排直接测主控板CAN2接口本体H2-L2：先在断电状态拔开主控端CAN信号插头，只量板端H/L，不量拔下来的线头，不改电阻/PCB。该测点用于区分板端120Ω终端通路与外接线束/中转板；原图R11=120Ω不能代替当前实测，32kΩ也不直接判定电机损坏。当前VM关闭/未运动、固件仍60daa57f…，等主控端实际读数。


## 最新：用户重焊后同固件一次只读通信成功

用户明确“再次尝试一下 我重新焊接了一下线”，按本次新授权只读复试after-user-resolder-01一次。仍60daa57f…Flash逐段匹配，无下载/复位/源修改，短时VM1后query2。完整身份/量程/状态读取：DMstate4/result1/read_cycle1/valid_mask7/range_mask7/feedback_valid1，tx7/rx7、ESR0、LEC/TEC/REC/BOFF均0、无首错误。实读ctrl_mode1（MIT）、motor_state0（失能），原始角-123.3933487°、逻辑173.1506958°、速度-0.007326rad/s（反馈量化读数，不称绝对静止精度）。这些均为本次新鲜反馈，不是旧缓存。finally已确认VM关闭请求0/PE2低；未发使能、失能、切模式、回零、旋转或其他运动。

对照：重焊前相同60daa57f固件多次外部无ACK；用户重焊后相同固件完整7/7读取成功。证据支持连接/焊接接触为此前无ACK的重要因素，不能定位具体哪根线/焊点，也不替代持续负载通信可靠性验收。用户此前“接线未变化”作为当时现场报告保留；本次明确重焊是新的现场变更，不倒推未报告的历史拆装。

此前拟做“普通CAN1轮询与DM查询隔离”的调度模型已MATLAB48检查通过，用户中断转为重焊复试；该调度修改未实现、未下载。成功不能归因于尚未实施的调度方案。原两文件通信补丁保持，不继续无必要改动/扩大测试。

当前空爪、VM0、软件升降/伸缩参考无效、载盘PWM0。通信单次功能已恢复；实际旋转/源抓取对准/模式2及单件转运仍未验，不盲发历史35.1079407°，下一次动作需先按当前实物姿态及既有位置速度模式切换流程接续。当前ctrl1不能写成ctrl2，本轮未切模式。R PWR-01/RUN-01及E RUN-06/LOC-03/CAL-05沿前述范围，Q保持待技术答疑确认，未上传GitHub。

原始证据：integration10-complete/after-user-resolder-01-power-readonly.json；当前ready.json更新单次通信通过，模型/编译/烧录/实物读取/运动验收分开记录。
