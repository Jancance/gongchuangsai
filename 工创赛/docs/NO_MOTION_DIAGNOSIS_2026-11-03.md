# 第1段不运动与末端停止：日志、仿真、下载记录

> 最新复测（2026-11-03）：用户确认放回启停区并按PC13。第二次候选经CMSIS-DAP下载/Verify OK，AXF SHA256为61795c2d6efb90084cf5f295148872e542309953f5038694e2c958b711f3ed1b。60秒只读采集149587字节/1065条、VM1有120条、CRC/字段错误0；最终VM1 state4/cause0、startup4，TMQ sequence_completed2/2、backend result1、STOP mask15、四轮反馈0、CAN/RTOS/Safety故障0，VM仍1保持机构。底盘两动作日志用时7800ms，前移后已进入横移并停车；不等于停车点/地面速度/重复性通过。证据endpoint/after-uart/和endpoint/live-result.json。

2026-11-03。用户报告已下载但不运动，明确要求助手下载并查看串口，随后要求每一步先做仿真。此次下载授权仅用于本轮诊断交付；串口只读，不发送启动/停止/KEEP，也不写Watch业务变量。程序启动仍由用户短按PC13。

## 第一次不运动：供电请求交接次序

COM14只读采集15秒37938字节，重解析286条，VM1有30条，CRC/字段错误0。原始数据before-uart/uart1.raw及兼容解析后的before-decoded/完整保留。

VM1：state6/cause8；TMQ：执行器result9、session0、首动作未完成、四轮转速记录0；HF3：启动提交时phase1、source_valid1、角度/角速度龄22/23ms，CAN/RTOS/Safety全局故障0。不是已完成前移或停车验收。

源码与真实完整后端复现一致：联合机构已经持VM，底盘App_MotionAccept发布g_vm_test_request=1后，可以立即到提交，不再经过旧“等上电”时间窗口；底盘内部s_vm_requested仍是Safety上轮的0，Auto_BeginPreparedBasicLocked拒绝并写result9。上一版仿真预先调用SafetyUpdate(1)，遗漏了这个交接。

修复仅在CHASSIS/USER/main.c的准入成功、g_vm_test_request=1同批调用ChassisAuto_SafetyUpdate(g_vm_test_request)。只同步已有准入请求，不写健康位、不清锁存、不发CAN，不删除安全条件。45项真实完整后端+抽取的原/新App_MotionAccept检查通过：原调用复现拒绝，新调用通过，故障/取消/错误会话继续拒绝；外设/平台/RTOS为桩。

验证副本及实际双目标0错误0警告；助手通过CMSIS-DAP下载Vehicle_Integrated，Keil返回Programming Done、Verify OK、Application running。第一次AXF SHA256：1cf5a7100fd66b1192c3973f92c24981afb754e907e58bad126e7ff4eebb2588。详flash-result.json与flash-Vehicle_Integrated.log。这是下载/Flash算法校验，不是独立全Flash读回或运动验收。

下载后after-uart/45秒日志只有待机run_id0、无故障，尚无用户启动。待机TMQ中的内置动作回显5cm/s属于阶段表尚未加载前的旧预设，不是第1段20/40已被改回；配置仍在vehicle_parameters.h。

## 用户复测：已前移，末端方向保护停机

用户确认按PC13一次；after-key-uart/18秒73043字节、460条、VM1有35条，CRC/字段错误0。用户反馈“电机只完成了前移，没有横移”，补充没看到倒退或回弹。

HF3：backend_session1、state5、cause8/detail10（方向反向）、address1，已确认目标+20个0.1RPM单位（+2RPM），新采反馈-20（-2RPM），两者时戳140712/140723ms。锁存编码器推算进度145.572mm、目标150mm、distance_reached0、sequence_completed0。原供电交接拒绝已消失；这次不是横移已执行或首段已验收。编码器推算不当作尺量地面位移。

实际反馈只有这一故障样本及用户观察，不能证明负反馈一定为误报、量化噪声或机械微反转；具体低速驱动响应/反馈与机械原因仍开放。

## 末端单帧观察候选与仿真

旧方向保护已有“末端小反向观察”，上限0.5RPM；Emm速度反馈为整数RPM，当前样本-2RPM被立即判为detail10。仅将chassis_direction_guard.h的末端单帧上限5→20个0.1RPM单位，即0.5→2RPM，作为明确工程候选，并非证明机械根因修复。

下列条件全部保留：最后10mm、目标/基速≤2RPM、曲线已进入减速/末端、不在停止或已到位状态；第二个新的反向样本仍detail11，超2RPM或非末端/较高目标立即detail10，600ms未恢复仍detail11；重复缓存不算新样本，同向ACK不延长观察截止。到位2mm、50mm跟随/轮间差、制动/通信/平台与速度上限均保持。

真实完整后端填入本次故障快照：旧版5项复现detail10，新候选39项验证单帧恢复、持续/大反向、非末端/加速段、缓存/同向ACK/超时。完整后端双段模型4053项断言，实际执行CAN后端状态机，CAN/RTOS/轮响应为模型：正常前移→横移完成；为进入末端窗口，故障注入场景对第一段假设3%响应差，单帧-2RPM恢复后完成前移并进入第二段标称横移；连续反向detail11、-3RPM单帧detail10继续停。该3%差和响应时间常数不是本车辨识值，不能据模型小误差签落点或真实CAN时序。

endpoint/contract-check.json、dynamic-check.json、dynamic-model.log保留精确模型及断言。副本/实际构建见各stage/actual日志和manifest。第二次候选下载/校验以endpoint/flash-result.json为准，不能把软件模型成功写成本轮实物已经横移。

## 串口解析兼容

当前固件使用TMQ/HF3，旧采集器只识别TM1/HF1/HF2，旧原始字节已保留。PC解析补充当前66/49字段TMQ及43字段HF3：RPM线格式按0.1RPM转一次，方向故障目标/反馈还原有符号值；CRC、字段数及范围检查保留。11项既有日志测试、4项本次真实样本/坏帧/单位/方向检查通过；只改PC解析，不改串口控制命令。

## 当前复测步骤与R/E/Q

当前速度仍前进20cm/s、横移40cm/s，轮径77mm；4310/伸缩/载物盘联合动作、夹爪145°保持均未改。由于车已前移，必须先放回PA1对应启停区、车头+Y、路线净空，再在新程序运行后短按PC13一次；不能在前移后的地点直接重新跑整段。正常完成后的再次按键是再跑，不是停机；运行中PC13取消。

已核R RUN-01/02/04、PWR-01、MECH-01/02、FIELD-02/06；E RUN-06、LOC-03/05/07、MOT-02/04、CAL-05。参数与单帧观察调整为E，不改R；FIELD-05、OBS-03、QR-04等官方Q仍待技术答疑确认。未闭环：新候选真实前移→横移→停车/持位、末端反馈物理原因、尺量距离/车道/载物稳定、故障物理响应和完整八段。全部仿真、Flash校验及用户有限观察分别记录，不相互替代。

证据根目录evidence/integration/2026-11-03/no-motion/，原件及各次SHA/日志分别保留。无Git提交/推送。

## 本轮现场复核结论与剩余测量

本次run1完成2个底盘动作，速度参考20/40cm/s，末端参考推算前移148.018mm、横移1051.193mm，四轮反馈峰值前段52/51/44/47RPM、横段104/101/102/101RPM。这是采样到的轮侧反馈与编码器推算，不是地速/地面落点误差。末端航向误差约-2.834°，没有将其签成航向精度通过。当前保持停止及机构持位，需用户反馈扫码区停车点的前后/左右偏差，再决定距离/航向调整。

初次启动拒绝已由实际session1/两段执行结果证明消失；这次末端未再触发方向故障，但日志未公开方向观察pending状态，不能声称本次一定出现过相同负帧并被正确恢复，也不能认定负反馈的机械原因已查清。单次完成不扩大为重复/载荷/故障/定位/比赛验收。
