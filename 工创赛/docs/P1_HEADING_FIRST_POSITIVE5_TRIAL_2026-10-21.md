# 首次目标航向+5度受控试验

> 最新现场结果：用户报告VM没亮，截图phase4/trip3/state4、VM请求0，测试未通过。不要重复以下启动步骤。当前先保留失败状态，补读exec_cause/backend_session、chassis_auto_state/result/cause/io_result/start_mask[2]再定位；不复位、不重写request1或强开VM。trip3是多个平台条件共用码，尚未确定首次失败来源。证据heading_positive5_vm_dark_trip3.png/json。

前置证据：会话固件35744/564/1444/25452、0错0警告；用户已提交新模式默认Watch截图，phase/request/trip/state/rpm、VM请求及两fault全0，源有效，角度/角速度帧年龄6/7ms，CAN最小余栈325字=1300字节，最小堆11328字节。接受观察时刻的默认软件状态；余栈当前只覆盖闲置路径，不当作活动路径通过，VM请求0也不等于物理电压实测。

本步属于工程调试（E），不是RUN-01正式比赛启动。沿既定完整装机载物、低速受控路径，首次只试+5度；不同时安排-5度或扩大角度。未签署正式整车运动许可、精确角度、连续时效或定位X/Y通过。助手不操作硬件。

## 用户操作

1. 保持之前确认的完整装机载物测试条件，把车放在平整地面，周围留出小幅旋转空间，线束不牵拉车体。固定机位记录一次动作，VM外部开关应可随时关闭。当前固件不用再修改/重复编译下载。
2. 打开VM外部开关。当前会话phase0/request0，程序本身仍应保持VM请求0，不手改g_vm_test_request。
3. 按F5保持连续Run至少3秒，使HWT恢复连续新鲜数据。不要在Stop状态先排队启动请求，也不要从截图暂停位置直接排队运动；本程序使用RTOS tick，Debug暂停不能证明真实帧持续新鲜。
4. 在程序仍保持Run时，确认g_heading_exec_relative_deg为5.0，只把g_heading_exec_request写成1一次。不要修改其他请求/状态输出。运行中若无法写Watch，保留未启动状态并反馈工具提示，不通过随意暂停/排队请求绕过。
5. 连续运行约18秒，观察车是否逆时针小幅转动后自动停稳。程序可能先等VM/CAN准备，不能把准备等待当立即故障。整个过程中不要点Stop或单步；如动作异常，关闭VM外部开关停止本次试验，不反复重写request1。PC13在本模式只提交取消。
6. 车已停止后点击Stop，提供真实动作观察/固定视频和结果Watch；不要在本次终态内继续改-5重试。若未移动/提前失败，也保留结果，不复位抹掉首次原因。

## 结果记录

```text
g_heading_exec_phase
g_heading_exec_trip
g_heading_exec_state
g_heading_exec_cause
g_heading_exec_target_deg
g_heading_exec_error_deg
g_chassis_auto_result
g_chassis_auto_cause
g_chassis_auto_stop_mask[2]
g_vm_test_request
g_rtos_fault
g_safety_fault
g_can_task_stack_words
g_heap_min_bytes
```

候选成功组合：phase3、trip0、state3、cause0，backend result1/cause0/旋转stop_mask0F、VM请求0、两fault0；到位误差在既有±2度候选容差内。目标为接受时的HWT航向+5度，target_deg不一定等于5。成功组合只能结合真实动作和自动停稳判断本次受控试验，不能证明外部角度精度或物理停车距离；误差和相同HWT源属于内部闭环证据，不是独立测角。

取消预期phase5/trip1，失败phase4并保留trip/cause。出现失败不清状态、不扩大时间/角度、不删保护，先保留源龄期、backend状态和首次原因进一步排查。+5度通常处于既有控制器1RPM档；2度/700ms与全部时限均为既有工程候选，参数未改。正常到位后切VM的实际电压、机械停稳、载荷稳定性仍要分别观察/测量。

已检查R RUN-01/PWR-01；E LOC-03/05/07、RUN-06、MOT-02/04。官方Q继续待技术答疑确认。未闭环：本次实车结果、动态资源/调度及源失效、正反角误差/物理停车、双起点与整场、实时X/Y。证据入口heading_runtime_idle_user_pass.png/json；该文件是试验操作安排，不是试验通过记录。
