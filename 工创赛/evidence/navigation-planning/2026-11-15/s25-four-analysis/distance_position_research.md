# 四趟距离误差与位置环：源码及 GitHub 核对

日期：2026-11-15。仅研究与本地分析，不构成新的运动授权。

## 刚完成四趟的事实

尺量前进依次2105/2107.5/2105/2112.5mm，右偏8/10/7.5/10mm。到位冻结的轮端进度依次2104.655/2101.789/2102.141/2102.677mm；与尺量差分别−.345/−5.711/−2.859/−9.823mm。最终PE按既有轮径和距离系数换算约2114.703/2114.395/2113.073/2113.541mm。

因此要分别回答三个问题：轮端何时被判到位、判到位后轮子还转了多少、同一车体测点最后在地面上移动多少。三个数的采样时刻、共同时间校正、车体参考点和轮地滑移均可能不同。最终PE−冻结进度约10～13mm不全是停车后真实地面滑行；不能把它直接作为刹车补偿距离或新距离比例。详细原始绑定见本目录summary.json与input_manifest.json。

## 当前代码已经有什么位置环

活动工程：`3.stm32f407工程/gongxun-f407.uvprojx`，实际源码在CHASSIS/USER。

`Auto_ProfileFollowRpmLocked`的原结构可概括为：

```
位置误差 = 轨迹此时参考位置 - 同一参考时刻折算的轮端进度
追赶目标 = Kp * 位置误差 / 每RPM线速度系数
追赶目标限幅到±2RPM，再经现有二阶平滑状态
前进RPM = 轨迹速度前馈/系数 + 平滑后的追赶修正
```

baseline40的跟踪Kp=1、follow上限2RPM、omega=1；并非纯时间开环，也不是没有位置反馈。`Auto_DistanceCheckLocked`先对四轮位置做共同时间插值、应用原距离系数，再判断到位；未到位时用共同采样时刻的参考误差计算等效进度，速度前馈仍取当前参考时刻。到位时走原STOP链，不会为了软件进度等于目标就证明地面已停稳。

需要更正一个容易混淆的参数：`APP_DISTANCE_SCURVE_APPROACH_RPM`实际是**1RPM**；终点遥测显示base15/16不是最低接近速度常数。四趟STOP前完整批的F分量为19/20/20/18RPM，不能归因于所谓“15RPM底速”；单降1RPM底速不可能解释这些命令。

## GitHub/官方实现查到什么

查阅日为2026-11-15，均查实际源或原作者文档，没有拉取项目Git仓库。

1. ROS 2 `joint_trajectory_controller` 的速度接口适配器把轨迹速度前馈与位置/速度误差PID输出相加；其文档要求速度命令闭环具有位置与速度反馈。这说明“位置误差转速度修正”是常见结构，但这是关节轨迹控制，不能照搬其参数或宣称它解决麦轮打滑。
   - 源码：https://github.com/ros-controls/ros2_controllers/blob/master/joint_trajectory_controller/src/joint_trajectory_controller.cpp
   - 文档：https://github.com/ros-controls/ros2_controllers/blob/master/joint_trajectory_controller/doc/userdoc.rst
2. WPILib Holonomic Drive Controller面向麦轮/舵轮等全向底盘，使用场地x/y误差修正和独立航向参考，计算修正后的底盘速度；它使用当前位姿和轨迹当前时刻的目标位姿，不是只盯终点塞一个很大的P。
   - 原作者文档：https://docs.wpilib.org/en/stable/docs/software/advanced-controls/trajectories/holonomic.html
3. ROS 2 `mecanum_drive_controller` 的odometry源码从各轮速度经运动学求车体速度，并处理车体参考点偏置。它仍是轮端推算，不提供本车独立地面位移真值；同样的编码器既作进度又作检验，不会凭空观测地面侧滑。
   - 源码：https://github.com/ros-controls/ros2_controllers/blob/master/mecanum_drive_controller/src/odometry.cpp

这些资料支持梳理参考速度、位置修正、航向及测点/时刻的一致性；它们不能指出本车唯一故障原因，也不支持直接再加一层PID或放大现系数。用户不新增地面传感器的决定保持。

## 本次行动与边界

- 保留四趟S2.5基线，不凭平均右偏少.625mm就继续减S或加Ky/cap；两组各四趟不是随机配对试验。
- 位置环先不另叠：先复原原参考速度与follow在最后120mm的贡献，确认到位时非零F由哪项主导，再讨论末端速度调度。需要本机MATLAB适用验证后才可对应实现；无法复原不能用任意plant拟合造通过。
- 浮点计时单独做仅观测插桩：现有DWT临界段统计不能替代横向数学耗时。芯片周期未读到时保持`chip_cycles_measured=false`，不宣称提速，不先改定点。
- 本轮实车四趟、数据分析、后续MATLAB、候选编码、下载及新实车必须分别记录。当前没有第五趟授权。

已核R RUN-01/PWR-01；E LOC-01/02/03/05/07、MOT-02/04、RUN-06；Q FIELD-05/OBS-03/QR-04未闭环，均不因参考GitHub而改变。
