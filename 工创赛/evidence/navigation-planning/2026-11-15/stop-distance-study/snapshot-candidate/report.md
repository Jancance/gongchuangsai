# 三个停车尾段快照：仅观测候选本地交付

日期2026-11-15。用户授权停车过渡研究，root在浮点计时候选独立提交`b1885ec3`后明确greenlight本项三个事件快照。本项仅修改统一F407入口的`CHASSIS/USER/chassis_auto_test.c/.h`，不启用B1、不改变任何控制公式/门限/曲线/距离比例/STOP/VM。B0仍为不可唯一复原，新增诊断不是停车效果改善实现或证明。

## 产品冻结与事件含义

- auto源：`ada294ab54d230e1c791a701b99c9007dc3c15ee7711633d63fe9855b4de122b`。
- header：`aa431918fb5b111d083647320b7d042ef60fb5d19c05c184c5d101c9882dbefe`。
- 字节级before为`49a1c2…acd94`浮点仅计时版本。按`source_changes.json`逆向移除**仅本项诊断插入**后，auto和header均严格还原before字节。原中文GBK保持；auto全部CRLF，header原混合行尾保持（全文件仍仅1个CRLF）。
- 只在原baseline40的精确专项接受条件`s_lateral_enabled`成立、会话相同且原profile有效/active时记录。其它动作不会清掉上次带原session的记录，也不会冒充同次新记录。
- 进入点：原有效共同时间mean≥目标−120mm，且原elapsed≥`boundary_s[4]`，确实已进入减速，不用“acceleration≤0”替代相位条件。第一次进入保留。
- 冻结点：进入窗后，在原CanStep整批命令计算成功并实际接受新批之后保存最近一次。它是**接受批冻结，不是完整ACK批**，仍可能随后被STOP打断；不在每轮发送/重复调用时生成新冻结。
- 到位点：原arrival分支置reached/finish/stop之前抄既有状态。此时current reference已Sample，follow尚未更新，故保存的是末次follow值/rate/tick，而不是凭空补一步或重算控制。
- arrival保存后锁存至下一次真实接受的专项start；STOP/取消/故障/idle及拒绝start保持。初始化全零不代表捕获或测量。

## 布局与资源

每槽48B：4个u32毫秒（now/run_tick/common/follow_tick）；5个float（共同mean、current参考position、**参考F**、follow值/rate）；3个u32（**请求base F**、**已接受applied base F**、状态flags）。三个槽144B；版本/session/validmask/事件计数/保留共32B；总176B。F不是轮端实测或车体速度。

静态C sizeof断言、实际ARM sizeof/offset和最终ELF symbol均核实176B。`g_chassis_auto_tail_snapshots`是唯一新增静态对象。auto.o ZI增加176B；链接库零区padding由4B降为0，所以整镜像ZI**净增172B**，不能把两个数混写。

reset使用有界volatile字节清零循环，只有一次真实接受专项start时执行，没有176B局部临时结构。最终Keil静态调用图：reset local/max=0B；record local/max=12B。BeginPrepared local48/max376、DistanceCheck local96/max192、CanStep local224/max608均与before相同。它们是静态编译器调用图值，未验证中断叠加、动态任务栈水位或真实芯片耗时。

## 验证结果

前置数学：父目录`matlab_receipt.json` exit0，`snapshot_matlab_results.json`通过布局、回绕、reset/保持、有效位及相位测试。原B0没有改判PASS。

本次actual ARM提取真实Prepare/Begin/DistanceCheck/follow/CanStep等函数执行。候选1420case/16755断言，before1402case/16714断言通过。所有before/after回执绑定实际输入auto与当前header，before确为49a…，after确为ada294…。外部CAN、供电、RTOS、HWT/source就绪及Auto_Check调度接口有明确桩，不是完整RTOS/板载运行。

- 相同fixture下32组×12字段=384个原距离控制状态/输出逐字节相同，见`before_distance_control.bin`与`after_distance_control.bin`；entry F/L/C与FF观察输出也相同。
- 真实DistanceCheck进入、真实CanStep接受冻结、原arrival捕获顺序、旧follow保留、STOP/取消保持、拒绝start保持、非专项接受保持、跨session拒收、C无符号毫秒回绕与getter只读均通过。
- ARM fixture初期两项准备错误已修正：人工两点history的head应为2而非0；取消后的新接受fixture需恢复`vm_requested=1`。没有为绕过这些fixture失败修改产品控制。初始嵌套断言只报告调用行，改为保留子检查行后定位。最终before/after均重新运行。
- 三目标均0 Error/0 Warning：baseline40 Integrated Code102676/RO6572/RW2952/ZI119196；baseline40 Offline Code102028/RO6564/RW2760/ZI119196；专项关闭Integrated Code11974/RO470/RW960/ZI25576。
- 非专项关闭build的所有allocated ELF节（含ZI）地址、flags、size及内容hash与浮点before完全一致；非专项没有新snapshot符号。
- parser完成全零未捕获、真实ARM合成176B三槽、uint32回绕、非有限字段无效等离线检查。`synthetic_snapshot_from_ARM.bin`明确是ARM仿真fixture产物，不是硬件读取。

## 最终镜像和未来读取边界

Integrated AXF SHA256 `5e803d0def9e33155752d4ceec71b9ca93ab903311407185d5d7af3c4880a185`，global地址`0x2001306c`。

Offline AXF SHA256 `4099e742ec2e243122770ec25ea8acd97aeb8e9ed0f1718898adf68d1b02bb24`，global地址`0x20013034`。

两镜像只读getter均被linker移除；未来只能按**匹配ELF的global symbol**读取。需要另一次明确硬件授权，本轮没有读取/下载/复位/发车。读取前后必须同session、DONE、backend inactive、fault0，且完整176B两次读回相同，才可作为稳定快照。不得halt、reset、改变VM、屏蔽IRQ或调用getter。无法证明停止更新/一致性则标`unstable`不采信；validmask或全零本身不能证明采集有效。离线入口：`python parse_snapshot.py snapshot.bin`，只读文件，没有硬件工具依赖。

已检查R RUN-01/PWR-01/MECH-01/02，E LOC-01/02/03/05/07、MOT-02/04/RUN-06；Q FIELD-05/OBS-03/QR-04保持待技术答疑确认。下一步由root/high独审并本地commit；硬件周期、真实snapshot和地面停车效果仍未闭环。
