# 最终实施交付

2026-11-16，实际6.1-sol medium；high已给SOFTWARE_PASS_WITH_LIMITATIONS。完整说明见report_61medium.md，本文件作为root交接入口。

- 默认OFF宏APP_LEFT40_DRIFT_TRIM_ENABLE；beta=.04，t=round(beta*b)整数0..4；仅PLAN9/左1/102预算/2100目标采用[b−t−h,−b−t+h,−b−t−h,b−t+h]，h保持原±min(b,2)，轮峰108<反馈130。左参考400，Aref128.1039269423/Jref240.0810115963，MATLAB profile8.90605s。产品旧前进、LeftTrim、左距比例、停车阈值与Mahony/KF不改。
- 五文件GBK/CRLF及前后SHA见source_changes.json；受保护函数字节见protected_functions_check.json。实际修改位置可按验证manifest中的源码定位列表直接核。
- 正式ON候选AXF：`D:/Desktop/工创赛/工创赛/tmp/lateral-drift-trim-build-20261116/left210_trim4/Vehicle_Integrated.axf`；SHA256 `73b4fad295ac044af9002e93e65858d7bc87165543557cd034a7a758406224ce`。0错误0警告，固定capture RAM及tail布局保持。defaultOFF加载段与此前0d5e2b基线逐字节相同，AXF整体并非逐字节相同。
- 实际MATLAB exit0；8个trim跳变，每次1RPM；h0阈值处原轮一步1RPM、新轮一步可2RPM。连续A/J预算不等于整数台阶具有连续有限jerk，不宣称地面前偏已改善。两趟原2032RAW完整保存，不从加计双积分造truth。
- actualC ON4443/OFF4443/非baseline45断言全PASS；实际非baseline chassis_auto_test源单编译return0无warning。STOP仍原停止帧，不作STOP后补退。
- 早期defaultOFF加载段曾+8B不一致，已明确保留defaultoff_first_comparison_preserved.json；修正预处理隔离后结果见defaultoff_load_comparison.json=true。早期AXF没有另外保存，不声称初次即同或所有中间制品都在。

全部只在本地，未下载/复位/串口/发车，未fetch/commit/push，不改根交接。下一步只能由root按新增用户硬件授权安排实测；本软件候选不等于物理验证通过。
