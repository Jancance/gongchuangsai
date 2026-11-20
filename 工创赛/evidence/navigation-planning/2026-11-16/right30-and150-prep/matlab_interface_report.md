# 右移300mm MATLAB前置及现有接口边界（产品写入前）

- 时间：2026-11-16。本地基线607368de0ce5eeb087f675b9183fbee4cb761f35；本任务无fetch/push/硬件。此次只新增证据和MATLAB验证脚本，不改变产品源码、根交接，不执行Git写操作。原dirty保留。
- 权威基线已核R RUN-01/PWR-01/MECH-01/02；E LOC-02/03/05/07、MOT-02/04、RUN-06。Q FIELD-05/OBS-03/QR-04仍待技术答疑确认。本次是工程参考曲线验证，不把候选参数写为正式规则。
- inputs_before.json为基于原150prep相同keys计算的当前1931项SHA256，5个既有产品源原字节已复制source_before/。本任务MATLAB完成时1931项全部未变，见matlab_scope_check.json。这是独立后续实现的增量基线。

## 真运行结论

E:/Program Files/Polyspace/R2020a/bin/matlab.exe实际运行成功，version 9.8.0.1323502 (R2020a)，2026-11-16 21:38:50~21:38:59，进程0。见actual_matlab_receipt.json、matlab_stdout.log、matlab_diary.log、matlab_summary.json和right300/right1500/right2100_profile_trace.csv。

全部采用右移候选速度上限400mm/s、rpm预算102、f=pi*77/60=4.0317105721、effectiveA=150-f*4=133.8731577116、effectiveJ=300-f*12=251.6194731347。按实际通用motion_profile.c:87-107逐分支算tj/ta/tv/peak；没有把1500的tv>0公式套在300上。另以single运算复现分支、double逐段积分，并用C原端点容差断言；不宣称MATLAB与目标ARM浮点库逐位相同。

|距离mm|数学参考峰mm/s|tj s|ta s|tv s|总s|加2s末等|参考整数峰RPM|跟随后base上界|加航向后轮RPM上界|
|---|---|---|---|---|---|---|---|---|---|
|300|167.930894808|.532046093|.722356938|0|3.572898249|5.572898249|42|44|46|
|1500|400|.532046093|2.455856835|.230050978|7.269949022|9.269949022|99|101|103|
|2100|400|.532046093|2.455856835|1.730050978|8.769949022|10.769949022|99|101|103|

三者终点与末v/a、单调位移、连续参考A/J、速度上限、runLimit12779ms、离散RPM预算断言通过。浮点分支端点误差分别0.0000276063/0.0000155942/0.0000575946mm，均低于C容差。离散全预算309个base/heading组合均不超104RPM且低于130反馈阈值；三条实际参考轨迹加2RPM跟随仍不超102预算，加2RPM航向仍不超104。离散整数RPM阶跃不能据此声称驱动实际A/J连续或实车加速度已验，物理执行/驱动滤波/载物仍待现场。

300mm会自动降低参考峰值且无巡航；400只能称速度上限。其数学参考峰只有约167.93mm/s，不能把未来30/150/210三种长度称为同峰速实测标定。相同目标距离也不证明地面实距、滑移或位置比例。

## 只读接口结论

1. CHASSIS/USER/motion_profile.c:66-128的MotionProfile_Prepare(distance_mm,&limits)已直接接收distance及speed/acceleration/jerk三项限制，包含巡航、短程去巡航、极短程jerk-only退化。300mm此次进入去巡航且保留恒加速段分支；不需要为30cm新建一套控制器。它是数学规划器，不是方向选择或已完成比赛路线接口。
2. chassis_auto_test.c:1824-1887的ChassisAuto_PrepareRouteTranslation(mode,rpm,run_ms,mm,speed,&prepared)已具方向/距离/参考速度入口，调用MotionRoute_ShapeAllowed和MotionRoute_Build等校核。MotionRoute_Limits目前由APP_ROUTE_SPEED_TRIAL_ACTIVE门禁控制，motion_route_trial.h:19明确最高20cm/s，且<=52 baseRPM；其A/J来自已有配置和方向校准/预留，调用者不能任意传A/J。不能把它宣传成当前已支持任意方向/峰速/A/J的完整比赛接口，也不能为本次400mm/s绕过门禁。
3. motion_trial40.h当前只允许2100四向和1500右移，尚未允许300mm；本任务尚未产品实现。方向0前/1左/2右/3后及各方向已有比例/修正必须保持分别判断。后续在既有通用规划和执行器上参数化可以复用底层，但需用户授权范围、方向参数表/门禁和相应MATLAB及实际C验证，不能复制右移参数覆盖其他方向。

## 未闭环

本结果只签当前右移300/1500/2100参考数学与RPM预算，不签新PLAN实现、真实ARM C断言、固件构建、下载、板端身份、实车峰速/实距/旁偏、比例与滑移、硬件停止或物理验收。无参数拟合；download/reset/START/COM/SWD均0。后续产品改动必须晚于本次真实MATLAB成功并由root独立核对PASS后实施。
