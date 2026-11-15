# robot_localization：麦轮/惯性融合方法独立核对

2026-11-15。仅主仓库 `cra-ros-pkg/robot_localization` 的 `rolling-devel` 公开源码与仓内官方文档；来源索引见 sources.json。本轮没有核得主仓库一套可直接复用、含四麦轮参数及实车验收的方案；所核为通用全向运动模型融合器。文档的 vy=0 示例明确是 nonholonomic 假想车，不是麦轮案例。无 fetch/clone/安装/第三方执行、无本车产品/硬件改动。

## 它实际融合什么

- `odometryCallback → prepareTwist` 接收已有 Odometry/Twist 的机体系线速度和角速度，不接四个原始编码器替你辨识轮径、轮位、滚子方向或滑移；麦轮 vx/vy/wz 运动学需上游先算好。[S2/S6]
- `imuCallback` 把 IMU 姿态、角速度、线加速度拆成三类，各由 15 位 mask 选择。可选择 wheel vx/vy/(wz) 加 IMU yaw 或 gyro_z，accel 可全部关闭；只含原始 gyro 的 ICM 不能冒充已提供绝对 yaw。[S2/S4]
- `Ekf::predict` 用姿态将机体系速度/加速度转为世界位移，含 v·dt 与 0.5·a·dt²，速度含 a·dt，yaw 由角速度传播；`correct` 以选中的量、其测量协方差 R 与估计 P 做 Kalman 修正及角差归一化。它**不要求先把加速度双积分成“IMU位置”再与轮位置平均**。若启用加速度，它进入加速度状态后参与预测。[S1]
- 状态是位置3、RPY3、线速3、角速3、加速度3，共15维；没有独立 gyro/accel bias 或轮滑率状态，不能因此声称自动完成零偏/滑移辨识。[S3]

## 麦轮不能照搬的配置

`forceTwoD` 实际强制 z、roll/pitch、vz、roll/pitch rate、az 为零，**不强制 vy 或 ay 为零**。官方 nonholonomic 示例的零 vy 是“车不能横移”的额外观测；真正麦轮必须保留其真实横向自由度，不能把“期望沿直线”伪装成横速测量为零。示例关闭 ay 的理由也依赖其底盘假设，不能自动套本车。[S2/S5]

## 最关键的数据前提

1. 去重：同一组轮编码器积分出的 x/y/yaw 与对应 vx/vy/wz 不是独立信息，官方示例建议此时优先用速度，避免把重复数据当独立证据。类似地，已使用 HWT 旋转得到的 wheel pose 再与同一 HWT yaw 融合，需说明相关性，不能只给小方差宣称更准。[S5；后半为对本工程的推论]
2. 坐标：pose 在 world，twist 在 base_link；使用正确 frame/安装变换、ENU与右手符号，ROS输入为 m、rad 等标准单位。本工程 F/R(mm) 与 ROS F/左(m) 不能直接混用。[S6；末句为接口推论]
3. 重力：`prepareAcceleration` 先按安装变换处理，再按参数减重力；有 IMU attitude 则用该姿态，否则当前实现可用 filter attitude。只有 yaw、而车体有 roll/pitch 变化时，不能据此声称重力投影正确；two_d_mode 也不是实际三维倾斜补偿。[S2/S7；后两句为推论]
4. 时间：`processMeasurement` 按 measurement timestamp 的 delta 预测，不以串口打印/收到时间冒充测量时刻。官方提供 lagged-data 回退重放及 history_length，但这不自动修复错误的四轮时间戳、错会话或样点缺失。[S8/S7；末句为推论]
5. 协方差：真实反映各量不确定度，mask 才是关闭无效量的方式；不要填零方差或虚小方差。不同 yaw 源和由 gyro 积分的 yaw/gyro 同源相关性需辨清，不应机械堆输入。[S6/S5；末句为推论]

## 能否纠正累计地面滑移

官方区分连续但会漂移的 odom 与可用地标/GPS等修正的 map；也明确只有速度、没有绝对 pose 时位置不确定度会持续长大。因此，轮速+gyro 可改进航向/短时预测；若横向地面滑移没有形成可信、独立的位移/位置观测，普通 EKF 不会凭算法名称恢复已丢失的真实累计侧移。可选 accel 能增加短时约束，但偏置、重力和振动仍须处理，不能替代沿程真值验收。这个“不保证恢复累计滑移”是基于模型/输入和官方漂移说明的工程推论，不是源码给本车的效果承诺。[S4/S7/S3]

## 对本 F407/HWT/ICM 的适用判断（E，尚不实施）

值得借鉴的是“轮端真实 vx/vy＋独立 yaw/gyro 的选择性、带不确定度、按测量时刻融合”，而非先升级加速度双积分。本车已有 HWT 主 yaw，当前 ICM 短窗 observer 只补时间差，不能把同源结果再喂为独立地面位置。若日后研究 gyro 辅助，先核符号、时间、bias与关联、实测噪声，再决定是否启用；当前稀疏 IO1 没有 u/C/D/E，不能凭前向多出约50mm就证明 accel 更准。加速度仍先作诊断/短窗研究，现阶段不建议直接把本包搬入控制。[本地已审事实与方法推论]

本包依赖 ROS2 rclcpp/lifecycle、tf2、Eigen、消息与参数/队列基础设施，并使用动态矩阵/标准容器；不是可拷入本 F407 C 工程的裸机单文件库。这里只借方法，不设计新 EKF，不安装或移植。[S9/S1/S2]

已核现有 R RUN-01/PWR-01、E LOC-01/02/03/05/07、MOT-02/04、RUN-06；外部库方法不改变竞赛规则，Q FIELD-05/OBS-03/QR-04仍待答疑。尚无该方案在本车的 MATLAB、固件、下载或实车结果。

## 主源导航

- S1 `src/ekf.cpp` — Ekf::predict / Ekf::correct。`https://github.com/cra-ros-pkg/robot_localization/blob/rolling-devel/src/ekf.cpp`
- S2 `src/ros_filter.cpp` — odometryCallback / imuCallback / prepareTwist / prepareAcceleration / forceTwoD / enqueueMeasurement。`https://github.com/cra-ros-pkg/robot_localization/blob/rolling-devel/src/ros_filter.cpp`
- S3 `include/robot_localization/filter_common.hpp` — StateMembers / STATE_SIZE。`https://github.com/cra-ros-pkg/robot_localization/blob/rolling-devel/include/robot_localization/filter_common.hpp`
- S4 `params/ekf.yaml` — 15-element config masks / example topics / gravity switch。`https://github.com/cra-ros-pkg/robot_localization/blob/rolling-devel/params/ekf.yaml`
- S5 `doc/configuring_robot_localization.rst` — Nonholonomic example / duplicate wheel-derived data / vy=0 / differential。`https://github.com/cra-ros-pkg/robot_localization/blob/rolling-devel/doc/configuring_robot_localization.rst`
- S6 `doc/preparing_sensor_data.rst` — Frames / ENU / signs / covariance / odometry preprocessing。`https://github.com/cra-ros-pkg/robot_localization/blob/rolling-devel/doc/preparing_sensor_data.rst`
- S7 `doc/state_estimation_nodes.rst` — Optional acceleration / gravity / lag smoothing / drift without pose measurements。`https://github.com/cra-ros-pkg/robot_localization/blob/rolling-devel/doc/state_estimation_nodes.rst`
- S8 `src/filter_base.cpp` — FilterBase::processMeasurement。`https://github.com/cra-ros-pkg/robot_localization/blob/rolling-devel/src/filter_base.cpp`
- S9 `package.xml` — ROS2 / Eigen / tf2 / lifecycle / messages dependencies。`https://github.com/cra-ros-pkg/robot_localization/blob/rolling-devel/package.xml`

版本边界：web读取的是可变化 rolling-devel，未声称锁定不可变 commit 或某个最新发行版；docs.ros.org 页面被拒后直接核仓内官方 rst，没有借第三方教程补结论。
