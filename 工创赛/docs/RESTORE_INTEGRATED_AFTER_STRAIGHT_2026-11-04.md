# 直线测距后恢复联调版

用户要求改回之前联调代码重新观察，并询问改动是否大。本轮恢复独立直线测试前备份的三个已有文件：USER/vehicle_parameters.h、CHASSIS/USER/vehicle_mission.inc、CHASSIS/USER/main.c；不是重写算法或根据四次均值加入补偿。独立测试与测量证据保留，straight_test.inc保留为未引用文件，原联调不调用它。

恢复内容：启停区1，前进15cm的斜移/扫码搜索；锁码停稳后按编码器推算X对齐1200，保留Y，逐段停稳重读到12格中心，再到14格沿−X100mm停车点(2060,1200)；到站后DM180°/伸缩70mm/夹爪180°并行准备，完成后等待视觉，不进入第3段。前向进度系数仍1.02182540，未应用候选1.01483076，也未删除保护或改变S曲线。

恢复前验证实际测试版三个文件与直线测试及调度修复的SHA一致，备份SHA与测试前原件一致。与较早dynamic-route最终证据相比，参数备份另有一处旧vehicle_drive第2行横移20cm/s、该早期证据为40cm/s；本轮保留真实测试前备份的20，不静默覆盖。当前动态第2段提前走专用生成分支，实际横移速度由VEHICLE_STAGE2_LATERAL_CM_S=40控制，该旧表项不参与当前第2段。其余关键控制源码与此前动态联调记录一致。

候选和最终实际源码同批重验80场景、1490773断言0失败；包含逐段编码器路线、准入与故障出口及原料区并行准备。设备/DM/PWM/轮系/传感器及部分调度仍为模型，不是新一轮实物通过。候选和实际Vehicle_Integrated/Vehicle_Offline均0错误0警告。实际恢复3文件，其他1865源码/工程SHA保持；没有覆盖其他同时发生的修改。

下载前只读确认板端处于停止/故障态后再烧录，非在运动中下载。下载Programming Done/Verify OK/Application running成功；镜像SHA256 `e63eb225de4857fc3300d041c306f8e163bfc72666f8bf72396b9cde5542e702`。之后只读136条、坏帧0，VM2第1段run0/state0/cause0待机，LT1独立测试帧0；未触发运动。当前已恢复联调板端，待用户从启停区1单按观察，助手未通过串口/RAM启动。

证据`evidence/integration/2026-11-04/restore-integrated/`：before-test保留直线版、manifest说明备份差异、after/actual-final整组结果、stage/actual构建、before-flash-uart/、Flash和after-flash-uart/。四次地面数据与独立测距镜像/源代码仍保留于straight210和straight-dispatch-fix目录，无Git提交/推送。

下载前末轮停止日志另见before-flash-uart：VM2任务cause8，TMQ编码器进度约1612.415mm/目标2100、sequence_completed0、backend_result2、distance_fault_reason5；HF3首时刻34900ms，backend_state10/cause3、detail8、反馈年龄450ms，航向源有效且UART错误0。距离reason5由位置/四轮一致性/里程范围等检查共用，当前不能仅根据450ms年龄定为反馈超时。该末轮与用户四次地面测量的对应关系未知，不能混入四次统计，也不能据此直接解释持续右偏。只保留事实，未调整其保护。

联调重新观察时，分别记录“车头是否转偏”“车头基本不变但车体横移”“偏差首次出现在哪一段”“是否故障撤VM及首因”。S曲线只定义速度参考，航向保持不等于地面横向位置闭环，编码器场地坐标仍为推算；当前不能断定右偏、距离波动和偶发撤VM同源。异常保留现场和电源，不先复位。

已核R RUN-01/02/04/07、FIELD-01/02/06、PWR-01、MECH-01/02、QR-01、DSP-01；E CAL-02/05、LOC-02/03/05/06/07、MOT-02/04、RUN-05/06、MECH-06。恢复/观察方案为E，R不变，FIELD-05/OBS-03/QR-04等Q仍**待技术答疑确认**。未闭环：恢复版同轮实车、右偏/重复性、末轮反馈故障与旧偶发撤VM原因、毫米视觉/抓料/整场。
