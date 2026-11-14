# half-stop-run5：当前half版成功单趟与停止快照

2026-11-15 07:46（Asia/Shanghai）。用户单独“再试一次呢”授权，仅复位一次、fresh HWT +0.005°、一趟40cm/s210cm；没有下载/改固件/参数/自动重试。

- software_complete；20.032秒保VM尾窗，200个有效尾帧，结束VMreq/PE2均1、active0/state9/result1、fault0。完成后TX0；总体有效TMQ317/无效0，累计telemetry dropped6另列，不能说全通道零丢包。
- XD925/925完整、102完整批；中段稀疏OD峰5mm/3点、横向指令峰1.9RPM，非地面沿程精度。HWT首尾+0.005→+0.703°，差+0.698°，停止前相对起点左0.038°。
- 20秒尾收后立即在同一脚本链读取176B；两次完全一致SHA 0f694fd54f6415405b7dd4730002d7edf099a9804878302ae10a3506e5fb107d，session1/coreRUNNING/VM1/fault0前后不变，mask7三槽有效。无halt/reset/VM写；读取完成已明确通知用户可复位。
- arrival判定now14080ms，共同样点13976ms/年龄104ms，轮端mean2102.344238mm，参考位置2082.572266mm/15.791047RPM；首STOP ACK14086ms（+6ms）。最终PE3四轮样点15120/15131/15142/15153ms，校准mean2113.429788mm，相差11.085550mm。不同时间的轮端量，不叫真实地面滑行或机械停车距离。
- 用户尺量原话“右偏2cm多走1.25cm”：实际前进2112.5mm、前进误差+12.5mm、右偏20mm。最终PE−尺量+0.929788mm；尺量−arrival +10.155762mm，不能用编码器接近尺量否认横向20mm。
- 本次通信成功不证明run2/3/4故障修复，具体主机到MCU接收路径仍未定位。用户后续又明确授权下一趟并确认摆好，另存run6，不与本样本混合。

证据：本目录授权/复位回执、serial原始raw/jsonl、20s summary、176B双bin/receipt/tail_parsed、capture_check和stop_wheel_distance、operator_measurement。源及已下AXF3966ed37…1113d保持。R RUN-01/PWR-01；E LOC-01/02/03/05/07、MOT-02/04、RUN-06；Q FIELD-05/OBS-03/QR-04继续待技术答疑确认。控制未变，本轮无新增动力学模型/代码；未fetch/push，其他dirty保留。
