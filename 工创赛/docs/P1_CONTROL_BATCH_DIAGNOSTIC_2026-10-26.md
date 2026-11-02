# 控制更新批次诊断（2026-10-26）

本功能只观察已有速度批次，不能据构建/仿真宣称实车延迟、纠偏或提速已验证。本轮未烧录、未打开COM、未发车。调试和比赛完整构建均0错0警告，实际C后台286项断言和主机94项测试通过；证据见evidence/p1-foundation/2026-10-26/batch_diag_review.json。

## 记录边界

四台速度指令一批，第一轮调用前开始，第四轮有效确认返回才标成功结束。发送失败、原保护故障、停止或撤VM会保存部分记录。停止命令不等待诊断；诊断不插入CAN交易、不改变轮次与穿插查询。每轮时间包含CAN接口内部等待以及任务调度，精度为现有1ms tick，不是硬件时间戳；原协议接口只返回有效确认，不能证明车轮到速。

生成来源0是初始任务/首次START选择，可能包含准备等待；1是匹配的最近邮箱发布时间，2是直接更新API接受时刻，3是距离调速产生时刻。重复同值刷新不反复改生成时间，不新增发送批次。批次中pending变化只记数量和结束时纠偏值，当前四轮仍使用原冻结命令。后续批次取当前待发送目标，不把诊断变成新调度器。

## 容量与读取

仅调试入口分配16条循环记录；重启清空，组合换段的session区分，batch_id在本次上电内累加。保留最近16批，另有覆盖数、累计最大排队/整批/单轮接口耗时及成功/中断计数，最大整批时间包含中断批次。诊断引入约2760字节额外静态RAM，运行时开销和CAN栈余量待实测；不能声称绝对不影响控制时序。

只有VM请求0、VM实际输出0且后端不活动才导出BT1；不在运动中格式化。每次至多一条、至少100ms，全部输出后至少空闲1秒再从头重放。主机在完成/故障后收尾采集2.5秒，不发送新的控制命令；仍受原总采集时限限制。也可在停车后使用原monitor入口只读采集约4秒，不需要再次发车。诊断帧不能续保活或建立新鲜健康样本。

BT1为ASCII，35个字段加CRC16-CCITT，形式BT1,v0,...,v34,CRC\r\n；仅correction_rpm、pending_rpm带符号，其余uint32。字段如下：

| 序号（从0） | 字段 |
| --- | --- |
| 0 | `session` |
| 1 | `batch_id` |
| 2 | `segment` |
| 3 | `dry_run` |
| 4 | `generation_source` |
| 5 | `generated_ms` |
| 6 | `start_ms` |
| 7 | `end_ms` |
| 8 | `outcome` |
| 9 | `sent_mask` |
| 10 | `ack_mask` |
| 11 | `failed_address` |
| 12 | `io_result` |
| 13 | `correction_rpm` |
| 14 | `base_rpm` |
| 15 | `pending_changes` |
| 16 | `pending_rpm` |
| 17 | `send_ms1` |
| 18 | `send_ms2` |
| 19 | `send_ms3` |
| 20 | `send_ms4` |
| 21 | `reply_ms1` |
| 22 | `reply_ms2` |
| 23 | `reply_ms3` |
| 24 | `reply_ms4` |
| 25 | `result1` |
| 26 | `result2` |
| 27 | `result3` |
| 28 | `result4` |
| 29 | `overwritten` |
| 30 | `max_queue_ms` |
| 31 | `max_batch_ms` |
| 32 | `max_call_ms` |
| 33 | `total_success` |
| 34 | `total_interrupted` |

outcome=1四轮确认，2故障，3停止/撤VM；dry_run=1表示模拟，不是实际通信；result为原接口返回值，1成功，0也可能是真实失败。sent_mask表示已进入调用；ack_mask表示有效成功确认。失败地址存在时，对应result即失败返回；仅已发送但没有确认/失败地址的轮表示中断时未取得可接受返回，不能用时间0判定“没有数据”（tick可回绕到0）。会话/状态已经失效的晚返回不补写为成功。

时间差按uint32模减，最长正常批次远小于半周期。queue_ms=start-generated，batch_ms=end-start；call_ms仅对有效确认或失败地址计算reply-send，其余为None。主机明确mechanical_response_measured=False。BT1不改原TM1/PM1/PE2/CT1/CT2字段布局。

## 后续集中实测

先经现场就位确认下载当前调试构建，验证校准30cm与210cm。每次停稳VM关闭后收取BT1并核对未丢帧/覆盖数，按generation_source区分准备和更新，再比较队列、四轮接口及穿插查询导致的整批耗时。结合轮速反馈才能讨论机械响应，不从确认耗时推算到速时间。没有运行证据前，不据本功能改变查询频率、放宽保护或启用高速/新航向量化。

核R RUN-01/PWR-01；E LOC-03/05、RUN-06、MOT-02/04。官方Q保持待答疑。
