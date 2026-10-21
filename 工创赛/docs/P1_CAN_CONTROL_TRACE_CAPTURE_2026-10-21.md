# CAN首次失败诊断：构建已核，下一步一次取证

2026-10-21只读核对实际工程D:/新建文件夹4/工创赛/代码/stm32/3.stm32f407工程。can.c与已验证诊断候选去注释/空白后一致；本机21:18:15完整构建日志Code37760/RO588/RW1604/ZI25492、0错0警告/3秒。KEY0/STATIC0/TURN0/SESSION1、CAN20/30ms保持。助手没有修改实际固件或操作硬件。

## 操作（工程调试E）

1. VM外部开关保持关闭，下载该次诊断构建并复位。完整装机载物地面条件保持，留出小幅旋转空间，外部VM开关随时可关。
2. 在Watch2加入下列诊断项；数组和last_rx需要展开。Watch1原13项健康/源状态保留，Watch2保留exec_phase/trip/request与后端result/cause/io_result/last_address。
3. 打开VM外部开关，F5连续Run至少3秒。确认exec_phase/trip/request均0、fault_latched0、两fault0、source_valid1且角度/角速度帧年龄均小于100ms。条件不满足时不发请求，回传状态。
4. 保持Run，relative_deg保持5.0，只把g_heading_exec_request写1一次。程序仍可能真实转动；连续观察约18秒，不中途Stop/单步，不再次写请求。异常立即外部关VM，不用调试暂停代替停车。正常停止或明确失败后Stop，保留首次快照，不复位、不清锁存。
5. 截图Watch1/2，展开request[8]和last_rx，报告VM灯是否亮、车是否转动/自动停稳。若fault_latched0，不强行制造失败；同样保留实际终态。

```text
g_can_ctrl_fault_latched
g_can_ctrl_trace_result
g_can_ctrl_trace_phase
g_can_ctrl_trace_address
g_can_ctrl_trace_length
g_can_ctrl_trace_request
g_can_ctrl_trace_pe2_start
g_can_ctrl_trace_pe2_end
g_can_ctrl_trace_tx_ms
g_can_ctrl_trace_total_ms
g_can_ctrl_trace_rx_count
g_can_ctrl_trace_last_rx
g_can_ctrl_trace_esr
g_can_ctrl_trace_flush_count
```

## 判读与验收边界

第一条准备命令预期address1、length5、F3 AB 00 00 6B。result5表示等待窗口内没有匹配回复；rx_count0为等待窗口没有取到FIFO帧，非完整总线抓包结论；rx_count>0时需核last_rx的IDE/RTR/ExtId/DLC/Data。phase2为发送等待、3为回复等待。PE2 start/end只代表软件输出允许，不等于实际VM电压；不要手改PE2/VM或扩大20/30ms与100ms保护。

R已检查RUN-01/PWR-01；E已检查LOC-03/05/07、RUN-06、MOT-02/04。FIELD-05、OBS-03等官方Q仍待技术答疑确认。证据can_control_trace_actual_build_verified.json及此前can_control_trace_preparation.json。本步是取证安排，首次CAN/VM根因、转角/物理停车、动态故障响应和实时X/Y未闭环。