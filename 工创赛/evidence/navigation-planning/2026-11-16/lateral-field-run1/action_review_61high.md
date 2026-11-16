# 单次左移实采最终独审

2026-11-16，gpt-6.1-sol high：**SOFTWARE_COMPLETION_RECEIPT_PASS / READBACK_RECEIPT_PASS**。详细前置、代码、MATLAB、构建、客户端及阶段独审见同日lateral-field-run1-prep/review_61high.md。本审查者只读证据，未操作串口/SWD、下载、复位或发车。

候选AXF SHA `0d5e2b752c92db9094393723b42411cbf02dbdc2f3950ac4c4ca6847fe73242f`。一次Keil下载/验证成功，额外reset0；本次原JSONL独立计数PLAN9=1、ARM=1、START=1、KEEP=48、STOP=0。实际plan9/kind5/dir1、2100mm任务/400mm/s参考；软件return0、330合法0坏、末active0/轮零/故障0/VM1/PE2=1、199帧19.985s powered尾有效。软件轮程2103.606mm不替代地面尺量。

完整Flash115340B匹配候选；真实本次PM1/session1与冻结header和readback一致。独立检查A34020B/B33600B/tail176B两读逐字节一致，SHA与回执一致：

- A：`f02da759a794ff153f142e60802d2377a35121f1fd6faed6da93818476b4ff4c`
- B：`ff9dfcb45751ac9443021b47b86b1f15d1c00554190ab0a4d065f2db63657604`
- tail：`86d2cf5b090f43ee54d8f7c1dcf746a853951191457ff6dac96269a9d24860b9`

RAW2032=PRE100+POST1932、wheel341/HWT194/OD78，quality0、drop0、保留样本gap空。reason1且terminal_tail_completed真，本趟未截断；trigger222104→terminal231266→freeze231766ms，实际触发至冻结9662ms、终态尾500ms，RAW支持−496..9659ms。首STOP230188保留原义。tail176全零为PLAN9未采前进补偿诊断，不能冒充有效快照，也不影响独立RAW完整性。

此前据软件总elapsed10826ms推测可能撞10s采集上限不准确，已按真实时标纠正；软件begin与采集START触发起点不同。这里通过的是完整保存、软件终态尾与运行健康；同窗物理静止/地面左移及前后偏差尺量、B0/BM对照与四向真实距离准确性仍未闭环，不签惯导精度或比赛闭环。不重放旧数据、不再发车或复位；root已通知数据安全可断开/复位，尺量前不移车。

R RUN-01/PWR-01/MECH-01/02（正式场地另遵守FIELD-06），E LOC-01/02/03/05/07、MOT-02/04、RUN-06已核；Q FIELD-05/OBS-03/QR-04仍待技术答疑确认。本轮仅本地完成，无Git同步/提交/上传。
