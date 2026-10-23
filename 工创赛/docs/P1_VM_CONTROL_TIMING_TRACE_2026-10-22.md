# VM开启与首条控制事务诊断（2026-10-22）

当前工程为根目录3.stm32f407工程/gongxun-f407.uvprojx。用户本轮明确授权助手修改固件，覆盖旧交接中本轮只读/手动修改限制。本轮修改power.c/h、can.c，保留GBK/CRLF。用途是诊断F3 E2，尚未修复或确认E2根因；没有增加ADC或VM电压检测。

## 实现与边界

Power_Enable首次设置PE2后记录RTOS tick；Safety循环重复调用不刷新时刻，Disable保持即时GPIO关断且不调用RTOS，历史开启时刻保留。重新开启另记一轮并增加计数。Power_ReadCommandStatus在临界区读取软件历史与PE2状态。

CAN控制包装在原首个失败锁存生效之前，每条合法控制事务开始前复制g_can_ctrl_trace_context；失败后随原g_can_ctrl_fault_latched一并保留。后台轮询和后续控制不会覆盖失败快照。非法参数沿原拒绝路径，不产生新的快照。captured=1只表示复制完成，不表示命令成功；成功事务间快照仍会更新，不是整个会话的首条命令永久记录。

| 字段 | 含义 |
|---|---|
| command_tick | 控制包装调用底层交换前的RTOS tick，不是硬件总线发送时间 |
| vm_command_active | 软件记录开启且PE2输出高；不能据此断言实际VM有电 |
| vm_enable_seen / vm_enable_count | 是否记录过开启及累计开启次数，tick为0也可有效 |
| vm_enable_age_ms | 本次PE2开启到控制包装开始的时长；未开启/无记录为0xFFFFFFFF |
| guard_ready | 发命令前的CAN就绪标志 |
| poll_result / poll_flags / poll_count | 地址1至4的既有后台状态查询结果、状态字及次数 |
| poll_age_ms | 各地址查询完成到快照时刻的龄期，未成功/未计数为0xFFFFFFFF |

RTOS暂停时tick不反映真实墙钟时间，因此不得在暂停中排队运动请求后直接当作连续时序测试。轮询快照来自现有后台3A反馈，不额外发CAN查询；不是完整总线抓包或独立电压反馈。首次失败前仅保存最新控制事务上下文。

## 验证

当前源与隔离构建副本逐文件SHA256一致；ARMCC5.06 update7完整Rebuild：Code38168/RO588/RW1620/ZI25588，0错0警告、2秒。检查保留SESSION1/KEY0/STATIC0/TURN0、20ms发送/30ms回复窗口、首次失败锁存及即时关VM路径。没有运行主机模拟，也未下载或实测修改后的固件。

为保留用户正在调试的主工程AXF和调试现场，构建在工创赛/tmp/vm_control_trace_build_20261022/完成。主工程Objects不会因为源码已改而自动更新。用户松开外部VM、退出Debug后，应在原工程完整Rebuild再下载；不能拿当前旧AXF解释新增变量。构建日志和SHA256记录见同日vm_control_trace_rebuild.log、vm_control_trace_build_review.json。

## 下一步操作

1. 外部VM保持松开，退出Debug。在原工程Rebuild，确认程序大小与上述一致、0错0警告，然后下载新构建。不得直接触发运动。
2. 进入Debug并连续Run至少3秒，先核原闲置健康条件；Watch加入g_can_ctrl_trace_context，展开结构体及四轮数组。仅添加Watch不触发动作。
3. 初始核对通过后，沿已确认的完整装机载物、低速地面和周围留空条件，外部VM全程允许程序控制，Run中仅发一次+5度请求；异常外部关VM。结束关外部开关后保留首次失败，不复位；提供原请求/回复、context与真实动作观察。当前可能仍返回E2，不承诺转动。
4. 根据vm_enable_age_ms和发送前四轮状态判断后续排查；不能仅凭一个短间隔断言欠压，不直接扩大超时、强写VM、清故障或增加任意延时伪造通过。已有稳定供电单条失能通过证据不要求重做。

已核R RUN-01/PWR-01；E LOC-03/05/07、RUN-06、MOT-02/04。此为工程诊断，不是比赛正式启动或运动/定位验收；官方Q未变。未闭环：E2根因、修改后板上时序、目标转角/停稳及X/Y。仅本地完成，未提交推送。