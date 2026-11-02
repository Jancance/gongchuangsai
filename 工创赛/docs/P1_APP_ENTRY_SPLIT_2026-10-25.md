# 调试与比赛入口拆分（2026-10-25）

## 2026-10-25 20:20 目标纠正为210cm，旧237cm预设禁用

用户明确纠正原237cm为口误，实际应测210cm。新增独立PLAN6/--plan forward210，仅调试mode3前进15RPM/2100mm，PLAN5保留编号但拒绝，旧forward237主机选项删除，不静默改成210cm。新forward210_debug已20:19:24下载、单次连续210cm软件完成40949ms，编码估算2101.04006mm，现场量尺及本次停车确认待回复；当前不再发车。原237cm结果保留历史，不作210cm精度结论。

普通10..100cm与30秒不变；210cm专用运行60秒、阶段65秒、后台总75秒截止，120秒组及其他故障保护不变；未加提前5mm补偿。两入口0错0警告，主机89项、C模拟162+131+44通过。证据同日forward210_review.json、forward210_test_review.json。以下237cm/PLAN5为历史实现，当前不得使用；板为新210cm专用调试版。比赛仍不放行。

## 2026-10-25 连续237cm调试专用扩展

用户指定237cm实测，新增PLAN5/--plan forward237，仅调试mode3、前进15RPM、2370mm单段可用。普通10..100cm、30秒保持；专用运行60秒/阶段65秒/后台总75秒截止，原组120秒和源/邮箱100ms、位置400ms、停止/VM/看门狗不变。遥测距离独立支持2370mm，角度范围不变。禁止其他超长距离/横移/低速借用此例外，比赛仍无许可。

debug/competition构建0错0警告，主机88项与C模拟161+128+42通过。20:11:32已烧录专用debug，单次237cm软件完成45470ms，编码估算2371.22791mm，CAN/RX最低652/992字节；现场停车及地面实际距离仍待本轮用户反馈，未加入提前停止补偿。证据同日forward237_review.json及forward237_test_review.json。后续使用forward237_debug隔离版；旧entry_split_debug版不含PLAN5。以下旧预设0..4/普通100cm限制说明需结合此专用例外阅读。

## 2026-10-25 19:40 三个单段预设实测更新

同一调试固件依次选择PLAN1左30/PLAN2前30/PLAN3右30，均只执行1段、结束关闭VM，用户逐项确认方向、停车及灯灭。无代码修改或重编，每轮回起点并复位一次性会话。CAN/RX最低剩余652/992字节，命令拒绝和运动故障均0；遥测丢弃0/2/0、捕获无效1/0/1保留。编码估算位移不是地面实测精度。证据同日entry_single_presets_review.json；初次USB掉线失败及恢复历史保留。当前板right30正常完成态，串口关闭、不再发车。比赛自主任务仍未实现且禁止发车；以下较早的“其他预设未硬测”为历史阶段。

## 2026-10-25 15:59 实测更新

新调试入口已15:58:45烧录，--plan route-a经PLAN4确认（空闲/VM0）后再ARM/START，三段完成并现场确认停车灯灭。CAN最低163 words=652字节，RX最低248 words=992字节，临界区最大32.22619us，命令拒绝0、遥测丢弃6。证据同日entry_split_test_review.json；板当前新调试入口完成态。本轮未改源码或重新编译，其他预设/比赛入口未硬测，比赛自主任务仍未实现。以下“本地未烧录”及“板上仍为旧版”是实施阶段历史。

本轮仅本地完成，未烧录、未开COM、未发车。板上仍为上一轮已实测的KEEP+CAN栈版。两个入口属于工程实现E，不代表正式比赛任务已经实现或通过验收。

## 配置与控制来源

唯一入口配置在实际工程USER/app_entry_config.h：APP_ENTRY_DEBUG=1、APP_ENTRY_COMPETITION=2，默认调试。APP_DEBUG_MODE默认3；旧0手动/1链路/2串口单段保留作兼容，当前日常不需要靠它们切换单段和组合。APP_TEST_MODE改为共享引擎的内部派生值，不再直接编辑app_config.h里的数字。入口切换仍需重新构建下载，运行中不允许切成比赛模式。

调试入口保持现有RC1/CRC、ARM/START、1秒KEEP失联停止、PC13取消、遥测及诊断。模式3可以在同一固件中选择PLAN0原默认动作表、PLAN1左30cm、PLAN2前30cm、PLAN3右30cm、PLAN4左/前/右各30cm。预设均沿用15RPM最大速度和原距离减速/终点保护，未提速。原motion_action_plan.h表内容保留；不带--plan仍使用原默认表，含原有转向动作，不能误当成当前三段A。当前测试明确用--plan route-a，不安排已取消的后两段。

选择PLAN需新的空闲遥测seq/tick，500ms及窗口5的原ARM新鲜度检查，remote_state0，姿态/独占/自检健康、前端未启动、无待办/旧运动会话、后台静止及VM关闭。选择只装载参数并递增accepted、回显预设id，不发车、不授予START许可。主机等到id及accepted递增确认，再重新取得健康窗口后ARM/START。未确认2秒超时或拒绝就不启动。已ARM、执行中、完成或故障后不得切预设；每次上电一组的原限制保持，下一轮仍需复位及回起点。

TM1模式3新增尾字段debug_plan_id（在can_stack_words后），共66字段；模式2仍49字段。旧字段位置保持，新主机兼容旧捕获，新固件需搭配新脚本。频率、CAN查询、任务优先级、时限、VM/停止和看门狗保护不变。

比赛入口复用运动API和原保护，但USART1不启用调试接收，不编入RC1发车解析器，不执行电脑ARM/START/KEEP或调试预设；主机无法运行时改入口。PC13只确认经原扫描/显示及平台门禁的任务码，按原扫描版本冻结16字节快照。AppCompetitionEntry_Submit检查四段基本格式并记录状态。完整任务/场地状态调度未实现，明确进入COMP_ENTRY_BLOCKED/COMP_REASON_PLAN_UNIMPLEMENTED，AppCompetitionEntry_MotionAllowed始终拒绝，MoveStart/TurnStart等公共API及CAN入口均检查该许可，VM保持关闭。不能把这个框架称作已具备自主比赛能力，也不把基本格式检查当成官方完整业务语法校验。

## 使用

以下命令会在现场另行确认并烧录新调试固件后才使用；本轮没有执行：

```powershell
python 'C:/Users/lenovo/Desktop/工创赛/工创赛/scripts/p1_serial_motion.py' --port COM17 --action run-once --mode 3 --plan left30 --seconds 55
python 'C:/Users/lenovo/Desktop/工创赛/工创赛/scripts/p1_serial_motion.py' --port COM17 --action run-once --mode 3 --plan route-a --seconds 55
```

单段可选left30/forward30/right30，组合route-a；default显式选择原默认表。主机默认action monitor，不会自动开始。比赛固件不接受这些命令，也没有电脑KEEP依赖。该变化不授权任何新的实车测试。

## 验证及限制

只构建两个当前入口：debug Code59900/RO848/RW2028/ZI34340，competition Code53024/RO652/RW1944/ZI29376，均0错误0警告。初次拆分出现编译期不可达/未使用告警，已集中到比赛入口许可函数处理，按受影响两个入口增量复核，无全七配置矩阵。比赛map中无App_SerialMotionCommand/App_SerialMotionByte/App_DebugSelectPlan，主启动门禁和运动接收引用比赛入口模块。原0/1/2兼容配置本轮未重建，不扩展成全部配置验证。

主机88项通过（原83+5组预设握手/拒绝/超时/兼容测试）。从实际main提取选表校验、空闲选择器和CRC命令处理，结合实际app_entry.c通过114项ARM模拟断言，包含五预设、忙/上电/旧会话/旧令牌拒绝及KEEP原回归。比赛实际模块通过38项，涵盖禁用调试计划、任务码快照不被后续覆盖、非法码/平台未就绪及运动许可始终拒绝。模拟为Cortex-M3 SoftVFP+Unicorn，RTOS/后台/电源为桩，没有F407并发/时序或实车证明。模拟源tmp/entry_split_unit_20261025，含debug.axf/competition.axf，证据同日entry_split_debug/competition_arm_checks.json。

生产源与隔离构建逐字节一致，比赛仅app_entry_config.h入口默认值不同；GBK/CRLF、Keil工程原UTF8/LF格式保留。主工程注册app_entry.c，未编辑冻结工程。检查结果和SHA见evidence/p1-foundation/2026-10-25/entry_split_review.json、两份build.log。

尚未上板验证PLAN握手、66字段遥测及新增Rx调用栈峰值；上次实测CAN水位652字节属于旧版，不能移植成新版测量。比赛尚缺任务码到场地/机械臂/P2/回区的正式任务状态机及场地输入，保持阻止启动，Q项不代答。下一步先对新调试入口单次route-a复测，再接比赛任务。

## 需求核对

R：RUN-01自主运行及禁止遥控、PWR-01电源、QR-01任务码、DSP-01原始码显示保持；E：QR-03校验、RUN-06有限状态/故障出口、LOC-03/05/07姿态与保护、MOT-02/04共用运动保护。没有变更R规则或官方Q，场地及完整任务语法答疑保留。比赛阻止发车是当前工程状态，不是规则要求机器人不得运行。
