# 同版独立观测实测分析

2026-11-15。新候选 `0ebf9e4e…874b3` 已单次编程/校验成功；两次有效实测和一次通信失败分别留档。观察者仍只读，不参与纠偏、到位、STOP 或 VM。

|项目|half-icm-run1|half-icm-run3|
|---|---:|---:|
|用户尺量右偏|5 mm|11 mm|
|用户尺量前进多走|7.5 mm|10 mm|
|arrival共同轮程|2100.300 mm|2100.678 mm|
|arrival共同样点年龄|164 ms|142 ms|
|最终PE3轮端均值|2114.867 mm|2114.156 mm|
|PE3−arrival轮程差|14.567 mm|13.478 mm|
|完整IO1 / 有效IO1|31 / 9|30 / 9|
|行驶raw的IR1帧数|160|158|
|最后状态报告的TM drop|2|8|
|IO1自己的低优先级busy drop|40|40|
|本趟采到的ICM最小余栈|320 B|320 B|

**目前能证明辅助观测已运行，不能证明改善了实际横向精度。** 两趟都完成正常20秒保VM尾段，并在新地址 `0x20012f18` 取得同会话、mask7的176B稳定双读。尺量仍是右5/11 mm、多走7.5/10 mm；观察者没有回馈到控制，不能把两趟差别归因于它。

有效IO1的旧轮锚落后估计时刻77～180 ms。运动中辅助前向位置相对旧轮锚最多增加约50.104/54.626 mm，说明它确实在做短时延迟外推；这不是已经减少了50 mm地面误差。横向辅助量相对旧轮锚的差分别为−1.110～+1.288 mm、−1.590～+0.791 mm。没有同期地面沿程真值，不能用这些依赖编码器的输出认定侧滑已经纠正，也不能将最后IO1和晚些时候的尺量相减作为终点精度。

两趟均为运动前5帧无效（无reference或无有效轮姿态）、运动中8帧有效、arrival后到首次DONE前1帧有效，正常尾窗其余帧因原轮姿态不再可用而invalid；没有新增控制故障。`valid_io_records.json/csv`逐行保留估计/两轮锚时刻、年龄、轮端与辅助F/R、差值、预测速度/线加速度及flags/cal/gaps。最后STOP区域的负前向预测速度是模型输出，不是实车倒退或特定振动原因的证据。

IR1行驶文件自身没有时序倒退：run1导出间隔中位200、最大205 ms，序号步长40或41；run3中位200、最大300 ms，步长40/41/42/60。它们是稀疏导出，不能把这些跳号当200 Hz底层漏采。已导出记录中的skips/races/transfer_errors均为0、age最大4 ms、观测自己的gaps为0，但仍不能恢复未导出的完整轨迹。有效IO1端点间通常1000/1005 ms、发布序号增50、原始样点增200/201，支持约20 ms计算/200 Hz采样的实现已在运行；不能据此认证每一次50 Hz执行或其最坏延迟。

**preflight旧前缀单独保留：** run1同一preflight raw内部出现tick1600→800、sample305→145；run3出现6000→800、sample1185→145。不是preflight/serial拼接制造的倒退。可能是统一reset前的串口缓冲残留，但来源未证实，不能据此认定另一次MCU复位。`ir1_export_audit.json`保留原始行号和全部单调段，主统计只用各文件最后单调段，不排序、不将负差取模成数十亿跳号。

新增IO1成功输出最大空档约13.001/13.050 s；“最多2 Hz尝试”不等于全程2 Hz输出。参考旧run7的TM drop为7，新两趟为2/8，IO1 busy drop为40/40。只能描述这些不同实测记录，不能据两趟建立CPU/队列负载因果。run3文件末尾64 B未结束的IO1片段另列，完整IO1 CRC/字段错误为0；原始raw和首次简易decoder结果均未改。

half-icm-run2单列，不计精度：唯一PLAN8/ARM/START后ack_timeout，既有client发送KEEP×8及安全STOP×1。稍后的单次只读采集确认START已accepted3、RX消费67 B、link_lost1/result8；任务/喂狗/TX继续推进，当时VM/PE2为0、backend idle/session0、无RTOS/safety/CFSR/HFSR故障。后续KEEP/STOP未体现消费，具体USB/COM/USART根因未定。不能把失联前最后TM当实时状态；没有自动重试该失败趟。

完整对照：`field_comparison_summary.json`、`field_comparison_table.csv`。原件与回执哈希：`field_evidence_manifest.json`；各run保留capture_check、stop_wheel_distance、raw/jsonl、用户原话、动作回执、176B双读及IO1表。

未闭环：实际地面沿程真值、姿态/时延/杆臂完整标定、长期漂移、最坏耗时/余栈及通信故障原因。没有因本次结果修改产品/参数或授权继续发车。R RUN-01/PWR-01及相关E边界保持，Q仍待技术答疑确认。
