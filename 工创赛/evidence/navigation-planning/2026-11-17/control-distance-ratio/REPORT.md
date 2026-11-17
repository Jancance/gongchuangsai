# 四向210cm控制距离比例候选阶段交付（2026-11-17）

本阶段仅在本地完成，基线30750bd9。产品改动只有app_config.h与chassis_auto_test.c；未fetch/push、未访问硬件、未下载/reset/发车。板端仍为此前3043...8ad3版本。

## 范围与行为（E）

APP_CONTROL_DISTANCE_RATIO_ENABLE默认0，独立候选为1。只对debug plan8～11、2100mm、现有40cm/s对应方向/RPM tuple，且route_speed=0的调试单段启用。左右分别要求既有左trim、右trim匹配；其他距离/速度/plan和实际路线不应用比例。

| 方向 | 比例 | 名义2100mm内部目标mm |
|---|---:|---:|
| 前 | 0.99604743083 | 2091.69960 |
| 左 | 1.00047641734 | 2101.00048 |
| 右 | 1.00015875536 | 2100.33339 |
| 后 | 0.998573466476 | 2097.00428 |

名义距离与既有路线接口保持，比例只在MotionProfile准备和准备结果验证时换算内部目标；到位、尾段快照、末端方向检查与超范围保护随内部profile目标保持一致。raw及导航残差算法不变，原有故障处理保持。

## 验证证据

- MATLAB实际先于产品实现运行，returncode=0，6628检查；见matlab_receipt.json、matlab_summary.json和verify_control_ratio.m。
- 实际ARM C fixture：ratio_on/off各213704断言，inactive13815断言，均PASS；涵盖真实准备函数、临界区前验证、right selector和AppDebugPlan40Allowed。无RTOS/电机后端实测，见actual_c_checks.json及check_control_c.py。
- Keil Vehicle_Integrated双整机构建均0错误0警告；见build.log、build_off.log及两个receipt。on同时启用既有导航候选；off同时关闭导航候选和本比例，并非仅切本宏的整机对照。单宏对照由实际C fixture覆盖。
- 冻结frozen/ratio_on.axf、ratio_off.axf、对应工程与build_options.json。开AXF SHA256 ae46ea7f62072dfa5388cda450ae2aaf029bb1b1a42a1dcb9147d42f8add1f28；关24c9a31726acfe5f6f2fe78b16488cb6320af5733d55ad39406d9043dd768994。冻结工程是原构建输入快照，保留原输出目录配置；非新日常工程入口。
- 早期fixture/guard宏拼写失败保留attempt/build_attempt档案，最终证据来自修正后的成功运行；未删除失败证据。原六dirty SHA256保持，GBK可解码、CRLF及非ASCII字节保持，见final_checks.json。

## 已检查需求及未闭环

R：RUN-01/02/04、PWR-01、MECH-01/02已核，无正式规则变化。调试候选不等于自主比赛验收。

E：CAL-02、LOC-02/05/07、MOT-02/04、RUN-06已核；本次只实现既有210cm样本下的比例候选，不是全距离全速度标定。单距离不足以区分比例误差与固定停车偏移，载荷/地面/重复性和本候选实车效果仍待验证。编译与离线检查不替代实际位移、停止和安全验收。

Q：官方未明确事项保持待技术答疑确认，未自行转为正式规则。本阶段不关闭现有规则Q项。

下一步先完成运动函数地图与统一坐标报告，再按用户决定安排适当新距离/重复样本和受控实车对照；当前候选不自动投入比赛或执行路线。

原始工具日志及生成fixture存在工具输出尾空白/末尾空行，保留原始字节；最终cr-at-eol检查对产品、文档和其余证据通过，原始输出例外逐项列于final_checks.json，未改写验证输入以制造全绿。
