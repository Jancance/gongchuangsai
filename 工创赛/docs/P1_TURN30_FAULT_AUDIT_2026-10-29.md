# 约30档起转前保护离线排查

2026-10-29。用户要求排查，并确认调试器USB已拔掉、先离线分析。此次不修改固件、不复位、不烧录、不重跑；保留此前首次故障RAM与串口原始记录。上轮下载镜像d195babca36de7ca37f4b5750a3f0001b3649f6c5da4f04b8d38dd2920655232，软件候选通过与本次实车失败分别记录。

## 已确认的证据

1. 本次一次START后，HeadingSession在22927ms接受，22938ms触发输入保护；最大非零命令、轮速反馈及yaw变化均为0。后端仍处于DISABLE准备阶段，未进入加速，也未达到30档。backend cause9是上层Abort的后果。不能据此评价高速过冲或30档效果。
2. 首次RAM快照heading trip6、session cause2、source_flags2：姿态源healthy=0。HWT UART错误1、ORE1，FE/NE/PE0、环形溢出0、校验错误0、error_epoch1。`hwt101.c`的USART2中断遇到ORE会增加epoch；Update观察到epoch变化便清理未消费缓存并Invalidate解析器，必须等新的完整姿态帧恢复。此链路能解释源短暂无效，保护行为应保留。
3. 原“角/率年龄7ms、源年龄8ms”不能证明故障时存在有效帧。`App_HeadingSourcePublish`直接用update_tick减去age；当age为无效哨兵0xFFFFFFFF时，无符号回绕会构造一个位于update_tick之后1ms的假时间戳。已用实际发布函数与实际解析器C复现：update_tick/sample=22930，无效age→angle_ms=22931，22938ms观察到的表面年龄恰为7ms，healthy仍0。正常计时器回绕另行检查通过。这是诊断显示问题，不是让无效数据继续运动的漏洞。
4. 启动调用链存在嵌套长临界区候选：`App_HeadingRuntimeStep`进入临界区后，执行HeadingSession_Start，再调用Auto_BeginHeading。后者又进入临界区并调用ChassisAuto_Begin；尽管该函数把Auto_PrepareBasicCommands放在自身临界区之前，它实际仍在上述两层外锁中。该准备会调用三次Chassis_BasicTestCommands，进行几何/逆解/编码浮点计算与初始化。
5. USART2中断优先级为6，FreeRTOS最大系统调用中断优先级为5；当前taskENTER_CRITICAL使用BASEPRI，因此会屏蔽USART2。115200、8N1的连续字节间隔约86.8微秒。启动计算被外锁包住会增加接收服务延迟，是ORE的重点软件嫌疑。

## 当前不能断言的内容

这次转向没有调用HWT101_DiagnosticArm，s_diagnostic未武装且全0；已有关键区计时只覆盖平移流程/动作装载，没有记录本次完整转向启动临界区。故缺少ORE事件时刻及与该临界区的重合证据。不能把当前“有ORE、存在可屏蔽接收的路径”写成已测得关中断超时，更不能据此排除接线、供电、其他中断占用。故障前后100ms串口遥测中的source_valid=1会漏掉其间短暂失效，也不能推翻锁存快照。

编译镜像使用FPU2；不能将离线软件浮点仿真的执行时间直接当作168MHz实物关中断时间。后端CAN交易本身位于ChassisAuto_CanStep的临界区外；不能笼统说“CAN等待期间一直关中断”。用户已拔下载器，新增只读复查没有连接到目标，也没有复位或读写板上状态。

## 建议修复与验证顺序

1. 给转向补齐原有HWT首错布防，以及转向启动外层临界区、后端准备的计时。保留首个故障，不把已恢复的源覆盖进故障快照。
2. 仿照平移的Prepare/Commit安排，把纯计算移出整个外层锁；仅提交短状态更新。在重新加锁后再次核对取消请求、源新鲜度、VM/安全条件及执行器归属。只改ChassisAuto_Begin内部的锁范围不够，必须检查上层实际锁深度。普通三段基础测试入口保持原行为。
3. 帧无效时用明确的有效标志保留不可用年龄，避免UINT32_MAX参与反推时间戳；修复诊断，继续拒绝无效姿态。无需删除保护，也不靠延长100ms期限掩盖接收问题。
4. 软件检查通过后，先用禁止非零轮速的受限准备流程取得ORE/关键区时序证据，再决定是否进行一次新的30档转向。当前未进行这些新硬件验证。

## 本次交付

新增离线审查脚本 `scripts/p1_turn_fault_audit.py`，只抽取发布函数和实际解析器到隔离目录编译/运行，不包含串口、探针、复位或运动API。证据 `evidence/p1-foundation/2026-10-29/turn30_fault_audit/`：unit.c、解析器副本、构建记录、测试AXF、report.json。实际C检查通过，包含有效帧、全部/部分无效帧及正常32位时钟回绕。工作固件保持与已记录候选一致；本次仅新增诊断脚本/证据/说明/交接，未上传GitHub。

核对R RUN-01/PWR-01；E LOC-03/05、MOT-02/04、RUN-06，未新增规则事实，官方Q不变。
