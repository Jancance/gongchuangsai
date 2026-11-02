# 串口控制与遥测接口（2026-10-24当前版）

## 2026-10-27 专用PLAN7候选，当前禁止烧录实测

新增forward210-20/PLAN7：仅专用debug/mode3 S曲线构建允许前进210cm、参考20cm/s；基速上限52RPM含位置修正余量，反馈80。普通默认限值不变，根开关均0。主机必须显式选择，匹配回显/accepted后重新核健康（52/80/1段），再ARM/START；未指定/默认不能启动已选高档，运行形状不符单次STOP。TM1仍66字段，0.6秒失联/历史不保活保持。用新脚本，不用旧冻结host；详见P1_SPEED20_TRIAL_2026-10-27.md。代码/软件构建通过，按用户要求未烧录/发车，板上仍旧B2故障候选。

## 2026-10-26 低速S曲线候选的串口含义

候选默认关闭，若后续明确启用debug/mode3距离动作，TM1原字段数/顺序不变：base_rpm与distance_base_rpm可为0～2，理论速度按实际候选基速显示；空闲健康启动仍要求原3～15巡航配置，运行中0参考不是已到距。仅参考结束而编码未到不能成功，新增distance_fault_reason7/8/9表示跟随误差/参考异常/参考结束未到超时。后端到距仍需实际停止确认与新鲜零速才能DONE；不能把通信回包、S曲线参考结束或编码估计当真实地面到位。当前默认低速路径不变，未烧录；详见P1_DISTANCE_SCURVE_EXECUTION_2026-10-26.md。

## 2026-10-26 新增停车后的批次通信记录BT1

当前调试构建新增BT1：记录四轮速度批次生成/开始、各轮调用/返回及结束，最近16批与累计最大延迟/覆盖统计。只在VM请求输出0且后端inactive时输出，运动中不输出；主机只归档诊断、不用于控制/保活，结束后收尾采集2.5秒。原TM1及其他帧不变。生成初始准备与动态更新来源分开，通信确认不能代表机械到速。格式及边界见[P1_CONTROL_BATCH_DIAGNOSTIC_2026-10-26.md](P1_CONTROL_BATCH_DIAGNOSTIC_2026-10-26.md)。本轮仅代码/软件验证，未烧录实测。

## 2026-10-26 前进距离校准的遥测含义

新构建仅在距离任务的前进进度乘1.02182540，TM1 distance_progress_milli_mm为校准后估计，distance_target_milli_mm仍为用户目标。左右进度、原始位置及PE端点不变；主机从PE计算的translation_travel_mm仍为77mm名义轮径编码投影，不能直接当校准进度或尺量值。字段顺序、数量、单位及发送节奏不变；定时前进与速度模型不应用此系数。本次仅构建，尚未烧录或实车验证；此前记录属于无补偿版本，勿追溯套用新系数。

## 2026-10-25 20:20 目标纠正为210cm，旧237cm预设禁用

用户明确纠正原237cm为口误，实际应测210cm。新增独立PLAN6/--plan forward210，仅调试mode3前进15RPM/2100mm，PLAN5保留编号但拒绝，旧forward237主机选项删除，不静默改成210cm。新forward210_debug已20:19:24下载、单次连续210cm软件完成40949ms，编码估算2101.04006mm，现场量尺及本次停车确认待回复；当前不再发车。原237cm结果保留历史，不作210cm精度结论。

普通10..100cm与30秒不变；210cm专用运行60秒、阶段65秒、后台总75秒截止，120秒组及其他故障保护不变；未加提前5mm补偿。两入口0错0警告，主机89项、C模拟162+131+44通过。证据同日forward210_review.json、forward210_test_review.json。以下237cm/PLAN5为历史实现，当前不得使用；板为新210cm专用调试版。比赛仍不放行。

## 2026-10-25 连续237cm调试专用扩展

用户指定237cm实测，新增PLAN5/--plan forward237，仅调试mode3、前进15RPM、2370mm单段可用。普通10..100cm、30秒保持；专用运行60秒/阶段65秒/后台总75秒截止，原组120秒和源/邮箱100ms、位置400ms、停止/VM/看门狗不变。遥测距离独立支持2370mm，角度范围不变。禁止其他超长距离/横移/低速借用此例外，比赛仍无许可。

debug/competition构建0错0警告，主机88项与C模拟161+128+42通过。20:11:32已烧录专用debug，单次237cm软件完成45470ms，编码估算2371.22791mm，CAN/RX最低652/992字节；现场停车及地面实际距离仍待本轮用户反馈，未加入提前停止补偿。证据同日forward237_review.json及forward237_test_review.json。后续使用forward237_debug隔离版；旧entry_split_debug版不含PLAN5。以下旧预设0..4/普通100cm限制说明需结合此专用例外阅读。

## 2026-10-25 15:59 实测更新

新调试入口已15:58:45烧录，--plan route-a经PLAN4确认（空闲/VM0）后再ARM/START，三段完成并现场确认停车灯灭。CAN最低163 words=652字节，RX最低248 words=992字节，临界区最大32.22619us，命令拒绝0、遥测丢弃6。证据同日entry_split_test_review.json；板当前新调试入口完成态。本轮未改源码或重新编译，其他预设/比赛入口未硬测，比赛自主任务仍未实现。以下“本地未烧录”及“板上仍为旧版”是实施阶段历史。

## 2026-10-25最新入口变更（本地未烧录）

当前选择APP_ENTRY_DEBUG/APP_ENTRY_COMPETITION，配置USER/app_entry_config.h；APP_TEST_MODE为内部兼容派生，不再直接改app_config.h数字。日常debug模式3用--plan left30/forward30/right30/route-a切单段或组合，PLAN只在发车前空闲/VM0选表，不自动发车。组合TM1新66字段（CAN水位后追加debug_plan_id），配套新主机兼容旧捕获。比赛调试接收关闭，任务调度未实现故阻止发车；板仍此前已实测KEEP+CAN栈版。完整配置、验证、限制及命令见[P1入口拆分说明](P1_APP_ENTRY_SPLIT_2026-10-25.md)。下文旧APP_TEST_MODE、字段数及“当前板”说明为历史阶段，不能据此选择新版配置。

## 2026-10-25 新扩展已烧录并单次实测

15:34:57烧录新KEEP+CAN栈版，三段30cm完成并确认停车灯灭。CAN任务最低剩余163 words=652字节；运行命令拒绝0。成功终态额外单发一条同令牌KEEP，accepted144→145、rejected0、结果2保持、VM0且未重启，现场通过。未验证所有故障路径的栈最坏值，未放宽保护。证据同日completed_keep_test_review.json、completed_keep_terminal_probe.json；下节“未烧录”为实施阶段历史。

## 2026-10-25本地扩展，未烧录

成功停车终态的同令牌晚到KEEP只确认，不续期、不重启、不覆盖原成功结果；CRC/令牌和故障/停止门禁保持。TM1尾部新增CAN任务生命周期最低剩余栈can_stack_words（F407每word4字节），单段49字段、组合65字段。周期100ms不变，rx_stack_words仍是UART任务；0不构成栈安全证明。新主机兼容旧捕获，旧主机不识别新长度。实际CAN水位及结束边界修复尚待配套烧录实测。证据同日completed_keep_review.json，当前板仍旧优化版。

> 最新本地源码已改为动作表，组合遥测56字段、主机按sequence_count结束；尚未下载或实测，板上仍为54bb617的已验证四段版。当前入口及限制见[P1动作列表说明](P1_MOTION_ACTION_PLAN_2026-10-24.md)，下文55字段/固定四段描述对应此前实测基线。


当前APP_TEST_MODE=3为组合测试，模式2为单次平移，模式1为无线收发小模块。无线、左右移及整组动作已实测；具体结果与限制见P1_TEST_MODE_SWITCH_2026-10-24.md和根交接。打开COM默认仅监听，不自动发车；方向/动作模式编译锁定。

## 协议和调度

USART1 PA9/TX→DAP RX、PA10/RX←DAP TX，GND共地；115200/8N1，无流控。RC1只允许ARM/START/KEEP/STOP，不提供任意内存写、故障清除或PID修改。ASCII帧 `RC1,<seq>,<tick>,<verb>,<CRC>\n`，CRC16-CCITT-FALSE覆盖末尾逗号，4位大写十六进制。seq/tick引用主控近期遥测，令牌用于会话和重复包约束，不是加密认证。

主机3帧健康且至少150ms真实到达观察窗口后ARM；设备tick与主机节奏差过大重建窗口，防止DAP旧帧突发。主控核500ms新鲜度/近期序号和闲置门禁，再给予2秒启动窗口；主机核匹配令牌后仅START一次。重复START不重开或延长动作。模式2进入原平移前端；模式3进入整组调度。私有启动许可防止Watch绕过串口门禁。STM32负责PD、状态机和保护，电脑不逐帧算轮速。

主机200ms KEEP，主控1秒无有效KEEP取消，主机600ms无新遥测尝试STOP；原HWT/提交新鲜度、停止/后端/安全/看门狗保护均保留。PC13及STOP可取消整个当前组，包含等待/段间；已失败时STOP不覆盖首因。准备、失败、成功都有边界，不自动重试或清故障。

既有任务分工：TaskA更新姿态并100ms发遥测，UartRx解析请求，唯一CAN任务执行运动/换段，Safety独立抢占监督。CAN等待在组合临界区外；快照复制使用短临界区，格式化/CRC在外。串口TX中断非阻塞，忙时丢帧计数；串口模式TX768字节、TaskA512字栈、RX384字栈。实测RX余栈303字、最小堆9784字节只是本轮观测，不作为所有模式的资源保证。

## 遥测

TM1每100ms发送带CRC的整数CSV。普通平移48字段，早期组合51字段，当前组合55字段；主机兼容三种格式，schema见P1_SERIAL_MOTION_SCHEMA_2026-10-24.json。角度/角速度乘1000，非法浮点哨兵INT32_MIN；序号/tick以uint32回绕处理。组合追加sequence_state/index/completed、can_guard_cause/bad_nodes、power_wait_state/post_settle_mask。

组合phase描述整个组，只有completed4、正常停止mask15和VM0/PE20才完成，不能首段结束就停止保活。组合转向时correction_rpm承载转向轮速命令，非平移PD修正；peak_error和纠偏汇总来自当前平移前端，须按阶段解释。四轮RPM为缓存反馈，PE2是输出命令，误差和最新yaw可能来自不同更新时刻，不作同步位置测量或电压实测。

## 主机与记录

`scripts/p1_serial_motion.py`仅用Python标准库及Windows COM，默认monitor；run-once显式选择才启动，stop用于取消。默认预期左移--mode1；当前组合须明确--mode3，右移--mode2。例：

```powershell
D:/python/python.exe 'C:/Users/lenovo/Desktop/工创赛/工创赛/scripts/p1_serial_motion.py' --port COM17 --action monitor --mode 3 --seconds 3
D:/python/python.exe 'C:/Users/lenovo/Desktop/工创赛/工创赛/scripts/p1_serial_motion.py' --port COM17 --action run-once --mode 3 --seconds 120
```

动作命令仅在已确认现场准备且发车前提醒后执行。脚本关闭DTR/RTS，不靠开串口复位；清理RX并用时序门禁处理仍可能从DAP送来的积压，单靠PurgeComm不足。raw/逐帧RX/TX JSONL/summary默认存tmp/serial_motion_captures，关键实测已复制进evidence。每秒输出阶段/角度/轮速/VM状态。

25项主机测试通过；完整组合实测56.304秒、段间保持VM、90度到位误差约0.922度，最终停稳关VM，用户现场确认。结束边界一个KEEP被终态拒绝，不等于重复启动；旧缓存跳变不等于真实运行丢包。精确距离/XY、高速载荷、全部故障注入与无线长期可靠性仍未闭环。R RUN-01/PWR-01及E LOC-03/RUN-06/MOT-01/02已核，官方Q不变。
