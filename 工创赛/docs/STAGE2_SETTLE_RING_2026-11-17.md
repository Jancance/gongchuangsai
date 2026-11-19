# 第二段停车稳图与最近8条诊断记录

用户明确批准：底盘每次真实STOP15完成回到新state2/pose后，等300ms，再连续5张停车后新图，原始盘心最大两两欧氏距离≤2px才发送下一条有效纠偏；仍晃只继续观察，不新增故障停车。另修复诊断记录满8条后第9次纠偏被拒绝，改为循环保留最近8条。单步20mm及其他运动参数保持。

固件独立候选 `tmp/stage2_settle_ring_20261117/project`，基于已烧6203源码；Pi独立候选 `output/stage23_settle300_5x2_20261117`，基于原64项统一桌面包。**本轮只本地完成，未部署/烧录/清故障/连接设备或恢复当前run。** 当前现场仍以主代理最后故障状态为准，不继承旧反馈作新参考。

固件仅改 `chassis_auto_test.c`、`main.c`：count限定0..8，head指最旧物理槽，当前inflight物理索引不挪动；只回收outcome1真实完成或2已终止，不覆盖0。公开getter按最旧→最新返回；两个距离启动准入与普通启动共用可回收判断，session回绕跳过保留值0。普通basic在原真实DONE出口记录终态，不补造编码器mask；主机诊断遥测按session推进，跳过无完整端点的终态，不因物理索引回绕漏发，也不因发送失败提前推进。原真实故障/STOP/释放成功判据不变。

Pi只新增 `source_alignment_settle.py`、中文 `source-alignment-settle.json`，并在原入口state2接入：

- 起点为本次新SC2 `(run,pose,item)` state2收到的本机单调时刻，同pose重复状态不重置。300ms计时不是从发STOP命令或估算运动时长开始。
- 只用同一采集worker的严格递增sequence和read-completion timestamp；read必须在pose收到后，丢第一张postpose完成帧。计数的5张必须全部在300ms之后，每张均满足原200ms主机新鲜度；不复制/补齐图，不平均盘心掩盖晃动。
- 取现有检测器当前原始盘心，5点最大两两欧氏距离≤2px。无效fit、丢链、新pose、worker代际变化清窗口；新代际重新围栏/计时。未稳只发valid=false，无新增MCU故障或停车。
- 诊断记录新增 gate原因、pose接收时刻、worker/帧序列及实际发送wire/sendtimestamp，供实车对齐。**主机读完成时刻不等于传感器曝光时刻，不能证明USB零积压。**

原HSV、检测算法、J映射、body-profile、20mm、5mm纠偏结束、state6独立120°/抓取5帧及整个S3分支保持。S3继续暂停；本候选不能保证纠偏映射正确或收敛，也不代表抓放已通过。

本机MATLAB初343检查于 **02:44:55** 实际通过后才实现；普通basic真实终态/输入补充模型352检查于当前 `matlab-result.json` 的完成时间通过后才作对应补充。初次工具stdout为初前置证据，当前JSON是补充运行，不能冒充初次文件。实际C完整底盘650、旧距离分支391、真实MCU消费Pi wire319检查，Python真实process/新gate75检查通过。旧非SCurve夹具只补无关blend条件缺失的静态存储以编译死分支，生产构建未改该宏/路径。双Keil均 **0错误0警告**：在线AXF **18cc11eec2bf28b2642dbbb31c7c25807d651317a20464ca29997584262f293d**，Offline **bb7c2ac426d9f1aa130e9e339cf4f04bd12687ba6ab26c4548bb65e178096b0d**。

证据：`evidence/integration/2026-11-17/stage2-settle-ring/` 的candidate/checks、2051源码和66Pi清单、实际C/Python夹具与日志。保留原编码/换行；chassis旧混合17个LF没有批量转码，main原CRLF保持。包是**基于6203工程的2源码增量+双AXF及原统一Pi目录的3文件增量**，不是完整工程，也不自动安装。未来部署由主代理按现场承托/当前窗口明确安排；先切何版本、是否恢复本轮故障必须另有新鲜记录，不能因候选完成就擅自恢复已按run。

R核SRC-01/02、RUN-01/02、PWR-01、MECH-01/02，以及继承调试任务边界QR-01/05/06、DSP-01；E核RUN-06、VIS-15、CAL-02/05。300/5/2是用户工程决定E，R未改，官方Q继续待技术答疑确认。未闭环：实车相机余振、停车后稳图时延、映射/收敛、>8次连续纠偏实物、同步起动差、完整三件及100ms释放余量。仅本地交付，无GitHub提交推送。
