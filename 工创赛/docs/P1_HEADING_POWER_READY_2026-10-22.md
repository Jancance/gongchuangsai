# 航向会话上电准备门槛（2026-10-22）

> 反向对照已补齐：用户-5度终态phase3/trip0、后端result1/cause0，并确认方向与+5相反、自动停稳、VM灯自行熄灭。当前正负各一次有限测试通过；后续补重复性和独立测角，不视为精确转角或整车验收。证据同日heading_negative5_after_power_ready_user_pass.json/png；根交接最新段优先。

> 最新用户实车反馈：单次+5度phase3/trip0、后端result1/cause0，用户确认小角转动后自动停稳、VM灯自行熄灭；本次准备检查1046ms/mask0F。有限单次测试通过，独立角度精度、反向/重复性、真实电压和E2确切根因仍未闭环。详见根交接最新记录及同日heading_positive5_after_power_ready_user_pass.json/png；下文“未实测”属于候选构建时的历史边界。

## 本轮依据与改动

前版诊断固件用户Rebuild38168/588/1620/25588通过后，板上截图显示首条控制开始时PE2开启584ms，guard1，四轮状态查询结果全1、flags全02（使能位0）、次数2/3/2/2、帧龄21/10/43/32ms。命令F3 AB 00 00 6B收到ExtId0100、DLC3、F3 E2 6B，约1ms，执行终态phase4/trip6、后端result2/cause1、VM请求及两fault0。不能归因于30ms超时；重复失能或上电条件均为待排查原因，没有确认具体根因。

仅在APP_HEADING_SESSION_TEST航向模式中，原HR_WAIT阶段额外要求：PE2已经由Power_Enable实际置高至少APP_HEADING_POWER_SETTLE_MS=1000ms；随后四轮各有一次成功、无保护位、仍在原新鲜度期限内的3A状态采样，且采样完成时刻不得早于该次PE2开启加1000ms。原App_CanJogReady仍须通过，原5000ms等待截止、源100ms保护、取消/故障关VM、20/30ms交易窗口及E2返回失败均不变。非阻塞检查，不用忙等、不额外发CAN，不跳过失能/使能检查。

1000ms属于工程候选，既非厂商保证，也非VM电压检测或驱动器已经准备好执行命令的证明；本轮是加强就绪检查的待验证候选，不宣布E2修复。等待途中真实供电若被外部开关切断，软件仍无法凭PE2测出电压，应沿既有故障保护与现场停止流程处置。

## 新增观察项

```text
g_heading_power_wait_state
g_heading_power_on_age_ms
g_heading_power_post_settle_mask
g_can_ctrl_trace_context
```

wait_state：0未检查，1等待PE2开启，2尚不足1000ms，3等待四轮等待后的新帧，4该检查就绪。仅在原App_CanJogReady也满足且处于HR_WAIT时更新，不是独立实时健康监测。mask位0至3对应地址1至4，0F为四轮齐全。on_age_ms为软件PE2开启时长；未记录/未开启为FFFFFFFF。执行开始/结束后这些项保留最后检查值，不是实时供电状态。

原g_can_ctrl_trace_context保留：若发生控制失败，vm_enable_age_ms应至少1000、poll_age_ms对应采样须落在等待完成后；仍需结合本次request/last_rx判具体原因。E2不能清掉或当02成功；状态全02也不构成该命令已经被接受的证据。

## 验证和下一步

隔离完整Rebuild：Code38384/RO588/RW1632/ZI25584，0错误0警告、2秒；五个本轮累计改动源文件与隔离副本字节及SHA256一致，GBK/CRLF保留。静态检查原两个5000ms截止、原就绪条件、E2失败判定、20/30ms窗口保留，helper没有延时、CAN发送、强写VM或清故障。未做主机模拟、未下载或实测新门槛。

原工程调试会话及AXF未替换。用户先松开外部VM、退出Debug，在原3.stm32f407工程/gongxun-f407.uvprojx完整Rebuild确认大小，再下载、运行检查闲置健康状态及新Watch。之后再安排一次受控+5度，完整装机载物和地面留空条件沿既定方案，外部VM全程允许控制；异常立即外部关VM。不得暂停排队运动请求、反复发请求或复位抹掉首次失败。下一次是否动作、是否自动停稳以及原始回复须分别记录，不提前宣称通过。

证据：同日heading_584ms_*三张用户原图，heading_power_ready_rebuild.log和heading_power_ready_build_review.json。R RUN-01/PWR-01；E LOC-03/05/07、RUN-06、MOT-02/04已核。官方Q不变；E2根因、候选板上效果、目标角度/物理停稳、X/Y均未闭环。仅本地修改，未提交推送。