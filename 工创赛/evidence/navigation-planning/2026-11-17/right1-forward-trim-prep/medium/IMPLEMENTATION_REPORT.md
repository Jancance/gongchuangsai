# 右移1%前向候选：medium实现与软件验证

2026-11-17。产品只增5文件38行，原字节GBK/CRLF保留。新宏APP_RIGHT40_FORWARD_TRIM_ENABLE默认0；enable>1或enable1但APP_BASELINE40_ACTIVE=0会编译报错。root独立构建，本文不报告整机构建、下载或实车结果。

右向逻辑轮序LR/RR/LF/RF为[-b+t-h,b+t+h,b+t-h,-b+t+h]，t=0.01f*b；沿现Chassis_ProfileCorrectionRpm夹紧航向，仅最后Chassis_EncodeEmm量化到0.1RPM，轮速cap106RPM/反馈130RPM保持。仅PLAN10/12/13、mode2、!route且真实AppDebugPlan40Allowed匹配；实际计划对为10/2100、12/1500、13/300。右向精确tuple A/J除1.01，原左2%除1.02分支和其整数补偿字节保持；其他前/后/转向和保护未改。

MATLAB最终attempt6：R2020a exit0；脚本SHA B67617FD19AA1ACFF24BE88DCCC539FDD8FC9C84A854D2FE5E3DA4173A3CB349，回执matlab_receipt.json。17201轮速、10非法输入、2880scope组合和明确三正/反tuple、route/mode/inactive断言通过。逐轮最大量化误差0.0500000954RPM，F/L/yaw分别0.050000003/0.050000000/0.049000001RPM，均低于0.0501。零速全零；零航向b1..4前分量0、b5为0.05RPM、b6为0.1RPM，不承诺每档实际恰1%。参考A132.388009/J248.649176；300/1500/2100mm解析时长3.589837/7.303851/8.803851s，均含2s余量低于12.779s，端位置保持且终速/终加速度归零。

失败如实保留：attempt1错误将b5期望为0.1RPM；attempt2空结构赋值；attempt4新增逐轮断言漏ideal变量。attempt3/5数学通过但scope12/13反配，不能用作正确门控通过证据；attempt5_scope_invalid_12_13_reversed保存原脚本/输出并标INVALID_SCOPE。attempt6核实际app_entry.h修正12=1500、13=300并添加明确正反断言后才获root产品写许可。

实际ARM C：check_right1_c.py/actual_c_checks.json，right_on208124/right_off208124/inactive8295断言全部通过；真实chassis_kinematics/app_entry/motion_profile编译与Unicorn执行，按SHF_ALLOC执行地址加载PROGBITS并清零NOBITS。抽取实际新right选择表达式调用真实AppDebugPlan40Allowed；抽取完整Auto_PrepareTranslationEx/公开Prepare与BeginPreparedTranslation临界区前校验段。覆盖四方向三距离准备、零起步、A/J/130RPM/12779ms、profile破坏后拒绝、b0..102与h[-2,2]量化及非法/NaN/Inf/空指针。此为局部真实C与桩环境，不是完整RTOS/Begin后半状态机/真实CAN验收。

check_compile_guards.py/negative_compile_checks.json：默认0与合法enable1编译成功；enable2按Right40_forward_trim_enable_invalid报错，enable1/baseline0按Right40_forward_trim_requires_baseline40_debug报错。

source_before_manifest/source_after_manifest与source_diff记录原字节和最终5源SHA。git -c core.whitespace=cr-at-eol diff --check通过；普通diff --check会把按要求保留的CR当行尾空白，未因此转码或改换行。原6dirty与其他既有文件不在本次修改范围；最终一致性由root总快照核对。没有COM/探针/硬件、Git写入、fetch或push。

R已核RUN-01/PWR-01/MECH-01/02；E已核LOC-02/03/05/07、MOT-02/04、RUN-06；Q FIELD-05/OBS-03/QR-04仍待技术答疑确认。1%为工程候选而非正式规则或地面纠偏承诺；编码阶梯不证明连续轮加加速度或滑移/旁偏实际改善。定位算法未改，不把前馈命令直接加进XY；左坐标准确性和实车对照由后续独立证据判断。