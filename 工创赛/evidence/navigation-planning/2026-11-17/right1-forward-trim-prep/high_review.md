# 2026-11-17 GPT-6.1-sol high 独立审查

状态：PASS_WITH_LIMITATIONS。右向候选软件独审完成；未下载或实测。仅允许本地软件候选，未操作 COM/SWD、下载、复位或运动。读取时 HEAD 为 `3cda6059`，现有 dirty 保留；本审查未 fetch、提交或推送。

## 范围与基线

- 已读取根与 `工创赛/AGENTS.md`、根长期记忆与最新交接，以及 `工创赛/docs/competition_requirements_2027.md` 的受影响条目。R：RUN-01、PWR-01、MECH-01/02；E：LOC-02/03/05/07、MOT-02/04、RUN-06；Q：FIELD-05、OBS-03、QR-04 保持待技术答疑确认。
- 实际工程入口核为 `3.stm32f407工程/gongxun-f407.uvprojx`，Vehicle_Integrated 引用 `CHASSIS/USER`。未把顶层 USER 或冻结 handoff 当当前底盘代码。
- 现 `chassis_kinematics.c`、`chassis_auto_test.c`、`app_config.h`、`motion_trial40.h` 均保留 CRLF；前两者读取时分别496/2106行，不编辑产品。
- 本轮新右向 1% 前馈属于工程候选，不是官方规则、真实地面反馈或惯导残差校正。应先有用户电脑 MATLAB 适用验证，再写候选实现；MATLAB/C/构建与下载及实车效果分别记录。

## 右移符号、门控与量化审查

1. 当前合同为车体前 F、左 L、逆时针 yaw；轮序 LR/RR/LF/RF，正前进驱动方向 `{0,1,0,1}`。右移本体为 `[-b,+b,+b,-b]`，正航向修正为 `[-h,+h,-h,+h]`。因此新增前向 t 必须四轮共同加正值：`[-b+t-h,+b+t+h,+b+t-h,-b+t+h]`；解析正解 F=t、L=-b、yaw=h，未见符号疑点。
2. 已有左 2% 后向项为四轮共同减 `round(.02*b)`，仅 PLAN9/18/19、mode1、非 route 且 AppDebugPlan40Allowed 精确匹配才调用。右 mode2 当前不调用左项，删除左项不会改变右移。本任务保留原左项及其它方向。
3. 新右向默认关闭宏，只在基线调试模式、精确 PLAN10/12/13、mode2、非 route、102RPM预算与对应2100/1500/300mm匹配时启用。运行时门控、A/J预算、构建显式使能仍待候选实际核查。
4. 不可直接照搬左整数 trim：`round(.01*b)` 在 b<50 时为0，30cm短程峰约43RPM会完全没有前馈。现 `Chassis_EncodeEmm` 已将最终逻辑轮目标一次量化为0.1RPM，`CAN1_SetEmmSpeedRaw` 按 raw 原样发送，已有协议能力可用。
5. root 明确候选采用 `t=.01f*b` 保留 float，只由现 EncodeEmm 对最终混合轮目标做一次0.1RPM量化。此方案无确定缺陷。每轮相对理想混合目标的量化误差不超过0.05RPM（另计 float 计算微差）；F/L/等效yaw为四轮线性组合，各分量误差也不超过0.05RPM。应以实际 C 覆盖 ties、b=0..102、有效航向±2及低速夹紧边界，不承诺每个 b 恰好1%。
6. 特别例：b=1..4、h=0的实际前馈可为0；b=5、t=.05、h=0的舍入 tie 可同时使F=.05RPM、L=-5.05RPM，主向有.05RPM量化偏差。单独 trim 再舍入不能消除最终轮量化本身。b=0要求 h夹0/t0、四轮0；b=1/2、h夹到b时，未量化微小跨零 t<=.02RPM预计最终舍入0，需 C 确认无编码反向。
7. 未量化最高幅值102+1.02+2=105.02RPM，候选专用cap106覆盖，现反馈130RPM与停止/故障保护保留。右向 A/J 按1.01缩减再沿既有profile/follow余量；此连续预算不消除整数基速、最终轮量化与四轮通信批次产生的离散变化，不可称已保证实车jerk。

## 左移坐标准确性独立复核

复核输入为 `localization-left150-crossdistance-run1/2/3/capture/decoded.json`、各 `operator_measurement.json` 与冻结 `localization-left210-field-run3/fresh_two_run_residual_candidate_frozen.json`。本审查独立直接重建轮/HWT源时间共同窗，未执行会覆写旧证据的 `verify_and_score.py`。

- 冻结 SHA256：`847956d477d463919ffc1b8b0ed618f1afd821b85a2a3eb74c315b5c7a5089a6`。γ=`0.9935232978388142`、β=`-0.018507830134159294`，只用左210前两趟；第三同距留出与新左150三趟未加入训练。
- 复算使用每轮 source_ms、HWT angle_ms解缠、实际参考航向、5ms共同网格、中点+sinc逐段旋转；前向比例只乘一次。与各现存 MATLAB 输出最大差 `3.64e-12mm`。当前 `chassis_odometry.c` 同样采用 F/L正解、实际yaw逐段投影，不使用命令90度或末角旋全程；未发现确定坐标符号错误。
- 三趟共同网格末7.740/7.785/7.795s，分别早于首terminal47/46/36ms；尺量又是最终停后读数且误差界未知。因此下表均为表观误差，不能当同刻毫米精度验收。

| 左150趟次 | 基线主向误差mm | 修正主向误差mm | 基线前向误差mm | 修正前向误差mm |
| --- | ---: | ---: | ---: | ---: |
| 1 | +14.396 | +4.594 | -23.768 | +4.242 |
| 2 | +10.551 | +0.742 | -23.758 | +4.273 |
| 3 | +9.129 | -0.678 | -12.654 | +15.369 |

主向MAE11.359→2.005mm、旁向MAE20.060→7.961mm；第三趟旁向绝对误差12.654→15.369mm变差2.714mm。结论是冻结残差存在跨距离平均收益，但不可据均值放行通用旁偏校正，更不能称惯导已准确。主距和旁距应分开判定，γ与β也不捆绑签署部署。三趟IO1有效帧数8/0/0且期末invalid，ICM又共享轮锚，不能补成独立地面证明。

当前右向4趟轮/HWT模型前偏+4.645/+2.265/+0.697/+2.792mm，而地面后偏5/40/40/10mm，未解释差不能全归HWT或轮滑；也不能把新前馈命令累计当真实位移直接写进惯导。继续保持HWT实际读数基准，不叠固定0.2度补偿。左残差保持离线冻结观察，右控制前馈与定位坐标候选分别验证。

## 尚待候选后置独审

- MATLAB实际成功回执、场景和离散量化结论。
- 候选产品diff与字节/编码/原左及其它方向不变证明。
- 实际C独立复算：全部基速、±2RPM与夹紧航向、NaN/越界失败清零、b0停车、精确计划/非route门控、106cap与130反馈、STOP/故障代码保持。
- 适当构建与最终镜像身份；本审查不获取板端身份、不执行下载或实车。

## 后置独审完成

独立ARMCC/Unicorn实际C验证通过，证据independent-review/actual_c_report.json。开关0/1各1,018,205断言、42,127组混合输入、6,440组门控组合；开启仅3个正确tuple、关闭0，原generic方向校验和均2467659381。NaN/Inf/基速越界清零、NULL拒绝、b0零速、低b无编码反向、原Left整数trim与右A/J方向组合通过。flag2和非baseline开启编译负例命中Right40 guard。

最大轮幅105.0RPM；每轮最大量化差0.050010681RPM，F/L/H最大0.040001750/0.050003052/0.049003005RPM，均在既有0.0501RPM容差内。runtime条件与完整AppDebugPlan40Allowed函数从当前源原文提取，未以自写白名单替代；SHA分别f94abe5cba86162de1068c8bc42672cc5a03224b4726fec54e310bfb07005b74和3a1541cc7ffee41825d9aef0a35b7b3588c72222d0cfa33f121ca60c8ea5743a。完整RTOS/CAN/STOP未在此纯C测试执行，相关原保护通过局部diff确认未改。

首次完整app_entry编译遇到既有空APP_ROUTE_ROWS initializer，build_attempt1_full_entry.json保留，后提取真实完整白名单函数。首仿真4亿指令未返回，actual_c_attempt2_instruction_limit.json与当时脚本保留；只提高离线仿真器预算至12亿后通过，未放宽断言/控制。新目录写权限及自动审批超时经主代理授权限定目录执行解决，未改ACL或产品。

最终MATLAB第6次R2020a真实exit0及脚本B67617FD19AA1ACFF24BE88DCCC539FDD8FC9C84A854D2FE5E3DA4173A3CB349已核，旧PLAN12/13反配对问题修正并保留早期attempt。局部产品diff为5文件纯新增38行，无删除；原左/前后/转向/定位与HWT/STOP/故障路径保持。本次C输入源哈希最终一致。

独立读取root的build_receipt.json与实际build.log，隔离Vehicle_Integrated整机重建0错误0警告，实际AXF哈希核为3043c4a98aed1b04eb0c80fd4b2ea819b2c70b52a2c0dbe60613a969bca48ad3。构建执行者为root；本代理未重建、下载或接触硬件。

计划第5.4项4份RAW反事实命令再积分未在当前MATLAB执行，不能标完成；现理想端点敏感性只作背景，真实地面改善待实车。未来残差50%可作离线工程权重，不是统计可信度或轮总距打折；本轮不改左运动或冻结γβ。

最终：PASS_WITH_LIMITATIONS。未见阻断右1%隔离候选的确定软件缺陷；左坐标符号未见确定错误，旁偏残差仍不部署。右候选未实测，地面旁偏/同步坐标精度/ICM独立有效覆盖尚未闭环。
