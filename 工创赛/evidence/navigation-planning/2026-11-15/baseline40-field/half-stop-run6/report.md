# half-stop-run6：同版本再次软件完成，停止快照已取

2026-11-15 07:48～07:49（Asia/Shanghai）。用户报告run5右偏20mm/多12.5mm后，再明确确认已摆好，只授权本趟。统一工具复位一次，fresh HWT +0.494°，一趟40cm/s210cm；未下载/改固件/参数或自动重试。

- software_complete；保VM尾窗20.046秒、200有效尾帧，VMreq/PE2=1，backend state9/result1/inactive/fault0。完成后TX0，XD925/925和102完整批；TM累计drop7另列，不称全通道零丢包。
- 中段稀疏OD峰6mm/3点，非地面峰。HWT首尾+0.494→+1.043°，相对差+0.549°；停车前相对起点右0.033°。不能把不同起始绝对角或稀疏OD当终点横向尺量。
- 尾窗结束同脚本链立即读176B，稳定双读SHA c489478e34ff5ec50d75db43d4e3d4dbebbb9b33f697efcc8fd62ddb632b6d41；session1/mask7三槽有效，RUNNING/VM1/fault0前后稳定，无halt/reset/VM写。已通知用户读完可手动复位。
- arrival now14060ms/common13933ms/age127ms，mean2100.468994mm，reference15.758623RPM；首STOP ACK14065ms（+5ms）。最终PE3四轮样点15099/15110/15121/15132ms，校准mean2113.418473mm；两者差12.949479mm。是不同采样时刻轮端估算，不是真实地面滑行/停距。
- 用户尺量原话“右偏1.25，多走1cm”，沿用厘米单位：实际前进2110mm、前进误差+10mm、右偏12.5mm；PE−尺量+3.418473mm，尺量−arrival +9.531006mm。本次成功不证明既往通信根因修复，接收路径仍未闭环。

10项源及已下AXF3966ed37…1113d未变。R RUN-01/PWR-01；E LOC-01/02/03/05/07、MOT-02/04、RUN-06；Q FIELD-05/OBS-03/QR-04继续待技术答疑确认。本轮无新控制实现/模型或MATLAB仿真需求，不fetch/push。证据见本目录原始raw/jsonl、授权/复位/尾读回执、176B双bin、capture_check及stop_wheel_distance。
