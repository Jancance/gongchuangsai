# ICM45686＋编码器融合研究接续（未完成）

状态：用户要求开新对话，已停止继续调研。仅完成固定提交一手源码复核和最小方向整理；没有实现、MATLAB新模型、硬件操作或精度结论。实时web搜索及打开官方文档均返回HTTP400“请求被拒绝”，以下来源为本地已缓存的一手源码，未核最新上游版本。

已有事实：本次三趟真实MATLAB同窗结果中，BM比编码器＋HWT路径少119.99/107.71/121.63mm；B0相对轮路径有正有负。原B采用tau=.3s、轮区间.2s、历史.5s；BM只增加Mahony Kp=.5/Ki=0，未加入bias状态。比较窗早于DONE、手量不确定度未知，不能按单终点谁更贴尺量决定权重。详细数值见forward_three_way.md。

## 已核一手来源

1. [WPILib Odometry.java，固定提交6e9a27c](https://github.com/wpilibsuite/allwpilib/blob/6e9a27c1bc737ba239d91011d389013d55835272/wpimath/src/main/java/org/wpilib/math/kinematics/Odometry.java)：110–117行用轮位置差算twist，以gyro角差替换dtheta，再推进位姿；无acc积分。[PoseEstimator.java同提交](https://github.com/wpilibsuite/allwpilib/blob/6e9a27c1bc737ba239d91011d389013d55835272/wpimath/src/main/java/org/wpilib/math/estimator/PoseEstimator.java)378行起仍先更新轮/gyro里程，另有带时间戳的视觉校正。证明一种可核基线结构，不证明本车轮程无滑移。
2. [RtJinxuWang ESKF示例，固定提交9b7c3d9](https://github.com/RtJinxuWang/eskf_imu_wheel_ros/blob/9b7c3d99d2c688ef957bda43cda8bf12907d4e6d/include/eskf_iwo/eskf.hpp)：245–290行以R*(acc-ba)+g推进p/v并传协方差，296–328行轮速更新。它确有加速度预测＋轮速约束；但将车体观测写成(average_vel,0,0)，不能照搬麦轮真实横移，Q/R、18态和Jacobian均不是本车规格。

缓存位于2026-11-15/mecanum-imu-github-research/wpilib及smartcar-inertial/refs_root/wheel_eskf；对应source_metadata.json已有固定提交和SHA。本轮直接读了上述实现，不以INS/EKF名字推断功能。

## 可供新对话评估的最简方向（E，尚未实施）

先保留轮位置增量＋HWT路径为独立基线，不因加入ICM默认更准。若目标包含ICM加速度参与平移，先在MATLAB试一个小状态[p,v,b_eff]：加速度经已核坐标/杆臂/重力处理后推进p/v，b_eff为该轴有效残余加速度偏差，轮区间平均速度作有噪声约束。b_eff不冒称可分离物理加计bias和姿态重力泄漏；先固定已核航向和前端，不直接上整套18态ESKF。位置只积分速度，不把同一编码器衍生的位置/速度当独立观测重复计权；保留原轮区间与延迟语义，不能将区间平均速度假作瞬时末速度。若小模型仍不能解释姿态/转动，再判断是否需要姿态耦合EKF，而非按名称选模型。

局限：倾角误差会把重力投影为水平加速度；持续平移加速度也会污染基于重力假设的Mahony倾角反馈。现有旧MATLAB合成已显示此风险。只有轮和IMU时，持续匀速共同滑移不一定产生可识别加速度创新，轮尺度/滑移/姿态/偏差也可混淆；不能承诺识别所有滑移，或以创新变小证明地面更准。需要独立地面参考才可验证实际改进。

Q/R应由静态原始噪声、动态模型误差及同步地面速度/位移残差标定，并考虑轮源时延、区间相关性、振动与载荷；保留独立验证趟。没有动态真值时只能给假设范围/敏感性，不能从单个最终尺量反推最优权重或滤波通过。

## 下一步与授权边界

用户最新确定后续左移2100mm、速度上限400mm/s（不是300mm/100mm/s）；没有授权立即下载、复位或发车。Mahony平移处理明确延后到横移阶段，不在本研究悄改。

新对话先核最新长期记忆/交接和Git，只读理解已保存输入；再冻结MATLAB候选与评价标准。必要最小比较为轮＋HWT基线、原B、BM及小状态候选，使用完整约200Hz输入和同一时间窗口；在已知真值模型中分开检验静止、真实加减速、倾角/零偏、轮时延与滑移，量化位置/速度误差及不可观测情况。现有三趟可查行为一致性，不替代新模型真值或实车验收。完成MATLAB前置及独审后才决定实现；横移现场动作另按版本/条件授权执行。

本研究尚未完成：状态/区间观测矩阵、噪声辨识、可观测性和模型测试/验收标准。无运行中的命令或MATLAB进程由本研究启动。根文档由主代理统一交接。

规则：已核R RUN-01/PWR-01/MECH-01/02；研究结构属E LOC-01/02/03/05/07、MOT-02/04、RUN-06。Q FIELD-05/OBS-03/QR-04继续待技术答疑确认。本轮仅本地文档，不改规则、固件、控制、采样或Git。
