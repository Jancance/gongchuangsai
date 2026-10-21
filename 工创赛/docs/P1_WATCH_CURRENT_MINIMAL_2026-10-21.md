# 当前Watch精简清单

HWT已完成RRATE8读回及区间平均约50Hz取证。用户按断电步骤后新图RRATE result1/raw8/reply_count1、完整帧校验正确，设置服务active/result/state/command/raw和request均0，没有重新设置；在用户执行该断电流程的前提下，支持本次参数保持，截图本身不独立测量断电电源。后续回目标航向模式前还要核100ms源连续性，+5度和真实停车仍待验。

## 当前Watch1仅保留13项，每项一行，不重复

```text
g_vm_test_request
g_rtos_fault
g_safety_fault
g_heading_source_valid
g_heading_source_angle_age_ms
g_heading_source_rate_age_ms
g_hwt_yaw_deg
g_hwt_wz_dps
g_hwt_checksum_errors
g_hwt_uart_errors
g_hwt_overflows
g_can_task_stack_words
g_heap_min_bytes
```

其余当前均可从Watch删除：RRATE/50Hz设置过程的所有观察项、版本读取项、angle_count/rate_count/update_tick、当前未用的chassis_auto以及旧heading_exec/fault项，所有重复行也删除。以后进入会话试验时另在Watch2集中加入会话/执行器/首次故障项，不把它们混在静态监测中。

操作仅在Watch选中行按Delete，删除的是观察条目，不删除源码变量、不清零实际值、不影响已保存的HWT参数。可直接清空Watch后按上述13项重新加入；不要双击Value手改数值。当前无需修改固件或重新Rebuild；保持VM外部开关关闭，不再发设置或运动请求。

R已检查RUN-01/PWR-01；E已检查LOC-03/05/07、RUN-06、MOT-02/04。Q继续待技术答疑确认，参数保持/频率证据不替代最大间隔、源失效响应或物理运动验收。
