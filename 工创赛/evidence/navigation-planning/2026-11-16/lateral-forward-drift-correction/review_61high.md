# PLAN9向后4%分量候选独审

2026-11-16，实际gpt-6.1-sol high。最终为**SOFTWARE_PASS_WITH_LIMITATIONS，未下载/未实测**。先独立读取两趟实际源码/JSONL/RAW decoded/端点，再读xhigh冻结计划与medium结果；没有新硬件/Git操作，不重放旧融合算法。

## 方案边界与独立判断

两趟用户前偏85/95mm是整体平移现象，不能改称偏航问题。4%只是基于约4.09/4.60%量级选定的首个有界前馈候选，非地面标定、最优参数或已证纠偏结果。仅改PLAN9的向后分量，先不改欠距系数、到距阈值、提前停车、HWT、Mahony或KF，可以保留可归因性。

当前到距是左向共同源四轮投影，并未误用前进均值；公共向后−t在左向投影代数抵消，无须减去“预期补偿位移”。不得搬旧LeftTrim的s_position_trim或旧spread保护。首STOP后共同源左向轮程仅再增加约2.411/2.469mm；STOP→终态1078ms含1000ms SETTLE与端点读取，不能全称滑行。详见lateral-distance-diagnosis/review_61high.md。地面欠距下直接提前切STOP无充分收益证据，本候选不同时改停车。

## 已核实际MATLAB

本机R2020a实际13:21:46.799–13:21:56.058 exit0，actual_matlab_receipt/matlab_summary/4922B脚本及trace已读。beta.04；A_reference128.10392694234162、J_reference240.08101159625565，即先总150/300除1.04、再扣完整原follow余量。duration8.9060510306721685s，+2s<12.779；400参考平台保留。

309组base0..102、heading−2/0/+2枚举证明按既有低速heading限幅后mix左向符号正确、base0全零、峰108<130；逆解forward=−trim、left=base。公共前后/旋转增量的左向投影抵消检查成立；两趟各2032RAW已实际读入，均明确不当地面真值。新增trim整数0..4，候选profile有8次1RPM trim跳变，脚本明确不声称连续有限jerk。

窄补已闭环：最终实际MATLAB回执13:24:00.591–13:24:10.425 exit0，5588B最终脚本SHA `bc6137b658f79c5d8993f8916c316e772788530b55bc9102ba7e8f347b80faef`与当前文件独立匹配。已独立复算306行adjacent_command_changes.csv和4460点integer_profile_wheels.csv：h0、base13/38/63/88处新增trim各1RPM，某轮原1RPM变化变新2RPM；全heading枚举的old/new最大相邻跳幅均2RPM（原低速heading限幅本就可产生2RPM）。候选轨迹终点全零、左向逆解保持base。这里比较整数台阶的新增项，不声称连续有限jerk。

## 实现初读

最小五文件候选为app_config.h、motion_trial40.h、chassis_kinematics.h/.c、chassis_auto_test.c。宏defaultOFF，新增独立mix函数，现profile更新分支按macro+ID9+mode1+非route+精确AppDebugPlan40Allowed选择；初始prepare通用0RPM与候选mix0RPM等价，无需另造启动分支。A/J helper只在白名单LEFT102/2100及宏ON时返回总预算/1.04，原prepare/validate继续扣follow；PLAN8返回原值。

新增调用最初仅用运行时三元表达式，存在非baseline编译仍解析未声明helper的边界问题，已改为窄`#if APP_BASELINE40_ACTIVE && APP_LEFT40_DRIFT_TRIM_ENABLE`。nonbaseline_compile.json/log证明最终真实chassis_auto_test.c在ARMCC Cortex-M4/FPv4-SP、BASELINE40=0/新宏默认0下return0、空warning日志；未以fixture的inactive案例替代该真实调用源编译。A/J helper亦改为编译边界保持OFF加载内容。

实际C最终trim_on4443/trim_off4443/inactive45，共8931断言全PASS，fixture及源SHA已核；覆盖实际mix边界/108峰/公共后向和左向投影/零速、错误base/heading/null拒绝、PLAN8原A/J及100专用混轮边界、原capture终态。此为离线ARM模拟执行，不冒充F407真实时序或地面动力学。

STOP代码段、到距Auto_DistanceCheckLocked、Chassis_CalibrateDistanceMm、Chassis_TranslationPositionMm、前进Baseline40Budget/Commands、旧LeftTrimCommands源保持；本代理按花括号独立提取其中五函数全字节一致，root最终六函数检查亦true。没有s_position_trim新扣减、spread停车条件或STOP后补退动作，原逐轮CAN1_StopMotor/SETTLE/故障路径保持。5文件GBK/CRLF与source_changes最终SHA匹配。

ON/OFF统一入口派生隔离构建均0错0警告。最终ON AXF SHA `73b4fad295ac044af9002e93e65858d7bc87165543557cd034a7a758406224ce`，加载115660B，SHA `9322b1a88fe6a3d364e2459f945f156f7a9befbf29a25c65c7738b7777ae364b`。本代理独立解析ELF并实际比较字节：defaultOFF加载115340B与两趟原0d5e2b横移基线完全一致，加载SHA同为 `ddc6dd63e925ec44921290f726359bb12377b78736916a1927dfd572dd2680f5`。最初OFF比较+8B失败已保留检查过程，最终#if修正后一致；不以AXF调试元数据不同误判控制变化。RAM bank地址/尺寸及ZI总量保持，无新增记录RAM。

原前进d7d66e…d609候选和两趟横移0d5e2b…3242f候选/RAW均保留。本候选只提供固定向后比例，不是自主前偏测量闭环；轮/HWT本次仅估计到约30.75/20.15mm前移，低于85/95mm用户尺量，ICM尚未上线平移估计。不能称当前能自主量准前移多少，不能称4%已实测纠正前偏，亦不因此签左向尺度/四向距离准确。后续硬件与效果验证仍待用户明确授权，不自动部署。

当前没有新下载、复位、发车授权；软件候选即使通过，也只能报告未下载/未实测。R RUN-01/PWR-01/MECH-01/02，E LOC-01/02/03/05/07、MOT-02/04、RUN-06已核；Q FIELD-05/OBS-03/QR-04仍待技术答疑确认。4%/108/宏/AJ均为E，不改正式规则。
