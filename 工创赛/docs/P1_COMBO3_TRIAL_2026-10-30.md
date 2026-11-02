# 三段组合候选与首次现场记录（2026-10-30）

用户确认路线和摆放：地图左下角为原点，中心(1800,1800)mm，车头-90度，前进100cm/上限20cm/s，右移100cm/上限20cm/s，CW90度/30档，段间停稳，最终目标(800,800,-180度)。不返回起点、不做XY补偿。283×254mm按用户接受的近似外廓；理论最小边界余量458.5mm。短段曲线使平移规划峰值约14.5cm/s，20是上限不是全程地速。

基于独立外围记录候选复制tmp/combo3_candidate_20261030，只调整固定动作数、表、起始位姿/4个检查点、组截止120s，原控制器和保护保持；保守预算86055ms。根USER与CCW/CW/整圈原镜像SHA均核对不变。新客户端p1_combo3_serial.py仅显式mode3/PLAN0/三段身份启动，逐段检查100cm与CW命令符号，错误STOP不重试；最终应为转向结束。导出p1_combo3_records.py保留4点与连续OD1，终点输出位移而非“返回误差”，无效点不算完整。电脑126测试与2项导出检查通过。模型夹具适配最初引用旧文件而失败，保留models；修正后models_v2 40模型/482877断言，三段常规3/3完成、压力3/3过冲保护停止，故障/取消均停止。这里不包含完整RTOS/现场CAN上电服务，不能称全系统通过。旧单转预览工具不支持多段高速转向，preview.json拒绝保留，不将其当新候选通过证据；实际候选转换已在C调度与模型中检查。

21:35:43首次V1下载Programming/Verify OK，镜像897f4bf7fc6fc63bf80168d587cd7785144eb9d88cb5bdd30de52de196aa4434。现场确认后一次START（run_once/20261030_213614_780300）。起步等待约3秒保护，首错HF2 trip3/phase1、backend0、姿态valid1、角率龄29/30ms、UART/ORE/溢出/校验0；随后遥测safety_fault32、can_guard_cause2、四轮0、VM0，后端未开始、检查点未建立。STOP已发，无自动重试。串口后来出现tick回到约8秒的待机帧，故summary.last是复位后状态，不能误读为没有发生故障。两次pyOCD attach只读快照，未halt/reset/write；g_reset_iwdg=1确认看门狗复位。现场是否完全未动/停稳待独立确认，遥测零速不能代替现场确认。

原因：从外围候选继承的App_CanTask空闲分派，在组合state1/2直接调用ChassisAuto_OdometryPoll。第一段后端尚未Begin时，后者因无后端VM会话返回，App_CanPollStep又被跳过，导致起步必须的四轮状态轮询缺失，最终触发原3秒数据保护；压力模型和桩调度没有覆盖这一点。修复只在新副本tmp/combo3_candidate_v2_20261030增加App_ComboIdlePoll，空闲组合阶段交替状态/位置轮询，每轮最多一笔交易，非组合阶段仍状态轮询。没有放宽门限、清故障或无条件喂狗。V1证据与原候选保留。V2 Rebuild0错误0警告，实际提取调度/轮询89项通过；测试首次替身重名失败保留scheduler_v2，最终scheduler_v2_final。V2当此条写入时尚未下载/启动，现场复测仍待确认。

证据目录evidence/p1-foundation/2026-10-30/combo3/：manifest.json、geometry.json、models_v2、scheduler、scheduler_v2_final、build.log/v2_build.log、download.log、run_once、ram_first_fault.json、ram_recheck_reset.json、vehicle_review_v1.json。R FIELD-01/02/06 RUN-01/PWR-01，E LOC-03/05 MOT-02/04 RUN-06；内区本轮允许经过，Q不变。全部本地保存，未提交推送。

## V2重新下载与一次实车运行

用户确认V1完全未移动、仍在标记起点、现场就绪。21:44:44下载V2 Programming/Verify OK，AXF c46c1e35680eba51acb19b17912743ef630c49452f849b3aff5355e90763f82f；待机身份与源正常后，仅一次START，记录v2_run_once/20261030_214507_381000。正常完成3/3，总计42.185秒，无trip/CAN/RTOS/Safety故障，四轮0、VM请求/输出0、stopmask15。CW本段HWT约89.506度，少转0.494度。

保存的OP1起点/前进后/右移后/旋转后均valid1、continuity_lost0：起点(1800,1800,-90)，前进后(1791,799,-90.554)，右移后(791,805,-91.219)，最终(793,805,179.269)，单位mm/度。终点估计差(-7,+5)mm，地图航向差-0.731度；不是独立尺量结果。断VM后实时OD1因轮反馈过期冻结，valid0/reasons10/continuity_lost1，必须与停车前保存的有效终点记录区分，不能称停车后持续定位有效。v2_export包含原样XY与4点CSV和位移，不做位置补偿。此时现场停稳/灯灭/平顺与总前向/右向尺量待用户反馈。
