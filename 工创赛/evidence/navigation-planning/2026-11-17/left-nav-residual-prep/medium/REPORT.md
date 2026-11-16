# 左向导航经验残差：medium MATLAB前置验证

2026-11-17，源HEAD90da4385。MATLAB正式前置通过后，root批准的4源导航候选已完成实现及medium实际C验证，见IMPLEMENTATION_REPORT.md。原始RAW/采样/控制/硬件未改，右1%软件未安装的状态不变。完整Keil与high独立审查由root协调，不据MATLAB或C桩结果宣布实车通过。

## 训练口径

只用新左210三趟。k_i=(groundF_i-modelF_i)/modelL_i，k=mean(k_i)，不按150数据重拟合，不拟合主向gamma，不丢任何一趟。modelF是已有MATLAB按真实HWT角逐段投影的轮坐标；恒等式groundF-modelF=groundF-bodyF-yawEffect证明HWT已解释量只扣一次。剩余误差不能全部归因为纯轮滑，仍含尺量、起点/测点、停止末窗等误差。

| 左210趟 | 模型F mm | bodyF mm | HWT投影影响F mm | 地面F mm | 剩余F mm | k_i |
|---|---:|---:|---:|---:|---:|---:|
| 1 | -46.2312 | -43.7170 | -2.5142 | -2.5 | +43.7312 | .0207056643465753 |
| 2 | -46.9531 | -43.3314 | -3.6218 | -12.5 | +34.4531 | .0163105736591883 |
| 3 | -47.8235 | -43.4085 | -4.4150 | -12.5 | +35.3235 | .0167003774830163 |

均值k=.017905538496259973。用户固定工程lambda=.5时有效小残差系数=.0089527692481299865，既不是统计置信度，也不将总轮距乘.5。此均值来自三趟同工况；后续每一批按用户要求保留三次及其均值，不能把不同方向/不同条件混作一批。

## 正式每source映射与生命周期

L_path=dot(rawXY-origin,reference左轴)；navXY=rawXY+reference前轴*(.5*k*max(L_path,0))。参考轴来自原始参考yaw，rawXY已含每时刻实际HWT，不再补命令角或终角，也不写回raw。所有source主向L保持原值，净左位移为负/零不校正，重复同source不重加。scope只在成功SetPose建立参考时按精确PLAN9左210/PLAN18左150锁存，暂不支持19左30；限定单LEFT_DISTANCE/102RPM/正确距离/run0/reference0/relative0/非runtime_route。DONE/active/liveplan改变不撤回。ResetRecords只令anchorfalse，原scope/frame保留至新成功参考；失败manual SetPose不清scope，成功manual清scope。invalid仍按冻结raw坐标做同一仿射映射并逐字段复制valid/reasons/sample/age/updates/continuity/local_origin及其余参考元数据，禁止invalid时退回raw造成跳变。featureoff/成功新非左参考为raw alias。

## 左150回顾性留出敏感性

k先在脚本读取150数据前冻结，150不进拟合；但150结果此前已经看过，因此这是透明回顾性留出，不宣称事前盲验证或新增实车真值。

| lambda | 旁偏表观MAE mm | 最坏绝对误差 mm | 三趟有符号误差 mm |
|---|---:|---:|---|
| 0 | 20.0601 | 23.7676 | -23.7676/-23.7582/-12.6544 |
| .5 | 7.1062 | 10.2185 | -10.2185/-10.1988/+.9012 |
| 1 | 7.0494 | 14.4569 | +3.3306/+3.3606/+14.4569 |

100%均值略小但第3趟补过且比其原绝对12.6544mm更大；50%已由用户固定为本轮候选，本模型不自动选最优/签部署。所有主距表观误差保持原14.3961/10.5509/9.1288mm，不沿用前两趟gamma或beta。

## 验证证据与限制

本机MATLAB R2020a正式合同版本exit0，最终脚本SHA3B6358D300CC6A937D13EA38364B2E1204B43F2AFE7CE885FE868EFCC9DB4FD9；运行06:49:36至06:49:46。临时合同版原样归档provisional_lifecycle_before_contract。覆盖0/.5/1、六条实际HWT分解轨迹、初始yaw37/-90/±179.8/180.2/359.9、跨±180解析轮增量圆弧、无位移、禁用/非左、新reference、重复source与invalid同仿射/逐元数据复制，以及20项精确scope与125项生命周期断言。全局轴变换最大误差9.09e-13mm，single发布最大差.0002453mm；原始yaw/主L不变，无重复补偿。数据来自先前已验证holdout_prediction/decomposition，未重新生成地面途中真值。

24项输入/当前源SHA前后一致，见input_sha256_before/after与preservation_checks.json；保留原GBK/CRLF与6dirty。运行回执matlab_receipt.json；系数frozen_candidate.json；训练training_decomposition.csv（run/modelL/modelF/groundF/bodyF/yawF/residual/k_i）；sensitivity_scores.csv（record/isTrain/lambda/modelL/modelF/groundL/groundF/mainError/rawCrossError/navF/navCrossError/correction）；汇总sensitivity_metrics.csv；matlab_summary.json。

实际消费者风险：AppOdometry_ReadBaseline40同时供ICM轮锚(app_icm_observer.c182)、原始采集(app_fusion_capture.c130/184)和前向lateral控制(chassis_auto_test.c1349)。不得直接替换现getter.pose而污染原证据或已验控制；正式API方案保留原raw getter/s_baseline40.pose/s_odometry/perimeter与采集/前控不变；新增同型ReadNavigation40，featureoff直接alias raw，独立64B nav缓存，g_chassis_odometry发布nav.pose，ICM仅换getter选nav轮锚一次，不改capture ABI。MATLAB已证明raw/nav消费者分流；真正C实现和ABI仍待后续验证。

R核RUN-01/PWR-01/MECH-01/02；E核LOC-02/03/05/07、MOT-02/04、RUN-06；Q FIELD-05/OBS-03/QR-04不变，仍待技术答疑确认。MATLAB数学通过与产品实现、独立C、Keil构建、下载及实车通过严格分开。本阶段只本地新证据，无Git写入/fetch/push、COM/探针或现场命令。