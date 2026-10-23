# HWT停车前后角度分阶段诊断（2026-10-22）

> 首次板上诊断已取证：零速请求误差1.9849度，PE2关闭观察误差1.9025度，1020ms后误差2.0782度；HWT在关闭观察后反向变化约0.1758度，大部分欠角已在首次请求停车时存在。不能把全部欠角归为机械回退，也不能把HWT变化当作机械原因独立证明。证据见同日heading_stop_trace_first_result.json及两张heading_stop_trace_*_user.png；根交接最新段优先，下文未实测为构建时历史边界。

用户怀疑30度停下时回退；本轮仅记录，不调整±2度容差、5/3/1RPM、700ms稳定、20秒会话/21秒执行器、上电门槛或任何故障停机策略。前次截图的保存误差1.913度和当前差1.919度来自不同采样时刻，不能据此精确否定停车过程有回摆；此前“二者差0.006度”的比较只限那两个读数，不是整个停车过程的测量。

## 数据入口与事件

Watch新增g_heading_stop_trace，结构体复位后全0，每次上电只记录既有一次会话，终态不清除或重新排队请求。

| 子项 | 实际记录时机 |
|---|---|
| start | 航向会话和执行器开始被接受后，目标已锁存 |
| zero_request | 首次将零速提交到执行器邮箱之前；zero_submit_accepted记录该次提交是否接受 |
| reached | 控制器满足容差/角速度/稳定时间，开始请求有序STOP时 |
| completed | 后端结束、旋转停车mask0F，会话报告完成时；此时PE2可能已经关闭 |
| off_seen | 任务第一次观察到软件PE2关闭，可能早于completed；不是硬件精确边沿或真实VM电压采样 |
| after_off | 自off_seen观察时刻至少1000ms后产生的一帧新HWT角度，且源有效；2000ms截止 |

各事件有captured、valid、event_ms、angle_sample_ms、angle_age_ms、rate_age_ms、source_sequence、pe2_active、yaw_deg、error_deg、rate_dps。captured1只表示事件有记录；必须valid1才能把角度用于有效比较。每个事件只记录一次。target_deg锁存本次目标；error_deg根据同一份HWT源和目标计算最短角误差，处理±180跨界。

post_status：0等待关闭后样本，1已记录有效新帧，2两秒内未获得样本；超时不触发新动作、不延迟关VM。诊断观察在原终态提前返回前执行，所以正常完成/失败/取消后仍可收集关闭后的样本；必须保持Run才能获得这段记录。若只停住CPU而时钟暂停，所得时间不是独立墙钟测量。

这是软件请求/观察事件，不能把zero_request当作四轮实际刹车时刻、off_seen当作实际VM电压下降时刻，或把一秒后的HWT漂移完全归为机械回退。若零速后恢复过转动，zero_request仍是首次事件，需结合时间和后续事件判断。

## 判读

先检查事件captured/valid均1、zero_submit_accepted1、post_status1及各阶段时间。比较零速请求、到位、PE2关闭观察、关闭后一秒的yaw/error；相邻航向差按HeadingController_ErrorDeg(new_yaw,old_yaw)最短角计算，不能直接跨±180相减。对于+30目标，后阶段yaw下降/目标误差增大才支持该阶段存在反向变化；负向目标符号相反。即便观察到变化，仍需结合实物视频区分机械回摆、HWT漂移和现场触碰。HWT仍非独立角度参考。

若零速请求时已欠约1.9度且其后角度几乎不变，主要是当前±2度候选容差提前停；若零速/到位后出现明显反向变化，可定位到有序停车或PE2关闭后的观察区间，再讨论容差、减速或保持方式。不能先清故障、延长供电或去掉停止保护。

## 构建与下一步

实际只修改USER/main.c，保留GBK/CRLF。隔离完整Rebuild：Code38804/RO588/RW1632/ZI25872，0错误0警告/2秒。剥离新增诊断后main.c与上一版30度候选的控制源码规范化后完全一致；其余配置/控制/会话/执行器/电源/CAN/Safety文件逐字节一致。只作静态回归审查和完整编译，未主机运行模拟、未下载或实测新诊断版。

用户先松开外部VM、退出Debug，在原工程Rebuild确认上述大小，再下载并Run检查初始状态；Watch加入g_heading_stop_trace。初始核对后，在原完整装机载物、平整地面和30度留空条件下，外部VM全程允许程序控制，Run中仅发一次+30请求。观察完成后保持Run至少3秒，让关闭后新帧记录完成，再松开外部开关并暂停查看；异常立即外部断VM，取证优先级低于停机。不要为取证持续保持供电、不复位抹掉首次结果。

需要截图：原终态、g_heading_stop_trace的target_deg/zero_submit_accepted/post_status，以及zero_request/reached/completed/off_seen/after_off内的captured/valid/yaw_deg/error_deg和时间。先不改容差或保持方式。

证据：同日heading_stop_trace_rebuild.log、heading_stop_trace_build_review.json。已核R RUN-01/PWR-01；E LOC-03/05/07、RUN-06、MOT-02/04。工程诊断不改变竞赛规则，官方Q不变；机械回摆假说及精确角度未闭环。仅本地修改，未提交推送。