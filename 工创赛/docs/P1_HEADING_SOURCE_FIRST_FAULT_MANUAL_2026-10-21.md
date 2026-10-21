# 记录航向源首次失败：用户手动替换一个函数

> 最新取证已完成：首次phase1、flags0x10、发布/HWT更新10ms、角度99ms、角速度100ms，确认角速度帧达到新鲜度边界。无需再执行下方修改/触发步骤，不再发运动请求。下一项关闭VM外部开关，在连续Run状态两次相隔约10秒记录g_hwt_angle_count、g_hwt_rate_count、g_hwt_checksum_errors、g_hwt_uart_errors、g_hwt_overflows、g_hwt_update_tick；期间不暂停/复位/改参数，先核对接收频率与错误增量。下层原因尚待确认，门槛不变。证据heading_source_first_rate100_user.png/json。

> 最新：实际代码已匹配诊断候选，用户完整Rebuild36000/564/1472/25456、0错0警告/3秒；CAN等待参数/模式及原保护不变。下方手改/构建已完成。下一项下载并复位诊断版，在原受控+5度条件下仅复现一次，获取exec_trip和7个fault首次快照；不要重复写request、复位抹掉原因或手改VM。外部VM关闭时下载，打开外部开关后连续Run≥3秒再于Run中发request1，relative5.0，实际停止/失败后再Stop读快照。若发生异常运动立即外部关VM。本次仍是首次条件取证，不是已修复或物理验收通过。证据heading_first_fault_actual_rebuild_pass.json。

最新复测：用户报告VM0V、指示未亮、车未转；截图phase4/trip2、会话state0/cause0/backend_session0，执行器所有状态/结果仍0。与此前trip3/backend1不同，本次在运行桥等待阶段检查源失败，尚未建立执行器会话。最终source_valid1、帧龄期0x26/31=38/49ms只证明稍后的观测恢复，不能说明触发时未超期。启动前VM0V是默认关闭的预期，不需手改VM请求。

本步只加诊断，保留源100ms、提交100ms、CAN20/30ms、模式及全部VM动作。不能根据稍后截图继续猜电源或把期限放宽；记录真实首次条件后再决定修复。

## 当前唯一操作

关闭VM外部开关，退出Debug。打开实际工程`D:/新建文件夹4/工创赛/代码/stm32/3.stm32f407工程/USER/main.c`。

找到完整的`static void App_HeadingRuntimeTrip(uint32_t cause)`函数，从这一行开始到该函数匹配的右花括号结束，整体替换为[完整首次快照代码](code_samples/heading_first_fault_trip.c.txt)。样本包含7个新增观察变量和同名函数；不要保留第二个旧同名函数，也不要粘到别的函数内部。后面的App_HeadingRuntimeExclusive及其他函数全部保留。

保存、完整Rebuild，回传日志。本轮先不下载、不重复+5度试验，不手改故障/VM状态。助手只准备说明和隔离文件，实际源码由用户修改。

## 后续需要的读数

首次Trip时，在同一临界区保存，后续数据恢复或重复Trip不覆盖。记录的是Trip调用时的实际源快照和配置检查，不是后来Watch刷新出的年龄。

```text
g_heading_exec_trip
g_heading_fault_phase
g_heading_fault_source_flags
g_heading_fault_sample_age_ms
g_heading_fault_angle_age_ms
g_heading_fault_rate_age_ms
g_heading_fault_hwt_update_age_ms
g_heading_fault_tick_ms
```

| flags位值 | 触发当时的条件 |
|---|---|
| 0x001 | 源尚未发布 |
| 0x002 | 最近发布的源healthy不为1；发布时的内部拒绝细节尚未单独保存 |
| 0x004 | 计算快照年龄达到100ms |
| 0x008 | 角度真实帧年龄达到100ms |
| 0x010 | 角速度真实帧年龄达到100ms |
| 0x020/040/080 | 校准活动/请求或退出发送失败 |
| 0x100/200/400 | 版本事务活动/请求或A7请求 |

flags可叠加。0x002单独出现时不能直接声称无线断流；需进一步查发布时的拒绝原因。age_update记录HWT采集任务更新时间，用于辅助区分数据帧变旧与任务更新变慢。该值本身不改变判定。flags0且trip2若出现，应继续核对调用时序/源码，不能伪造非0原因。

R核对RUN-01/PWR-01；E核对LOC-03/05/07、RUN-06、MOT-02/04。官方Q继续待技术答疑确认。原CAN等待冲突已修正的构建证据保留，但本次trip2不是其修复成功或失败的充分证据。首次源失效原因、实际VM/运动/停车、动态时效及X/Y均未闭环。

软件检查：更新后的Runtime样本链接实际heading_controller/session通过原联动回归；新增首次帧龄期100ms/flags0x1C、等待与运行阶段、校准请求位0x40、后续源恢复/重复Trip保留首次原因、原Abort/VM0均通过。实际main仅隔离替换诊断函数后的候选由ARMCC无诊断编译，实际文件SHA前后未变；这些不证明板上时序或实物通过。证据heading_first_fault_preparation.json。
