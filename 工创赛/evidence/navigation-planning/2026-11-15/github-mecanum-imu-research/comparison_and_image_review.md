# GitHub实际实现与用户截图：麦轮/陀螺融合核对

2026-11-15。用户要求只研究他人实际实现，后提供惯导XY公式截图问评价。本轮未改本车固件/参数、未下载/复位/发车、未运行第三方代码/安装依赖、未对我方仓库fetch/push。板上仍已验证下载的0ebf9e4e…874b3独立观测候选，采集后的实时VM未再读取。

## 一、已读实际代码（不是按README标题判断）

|实现|实际位置来源|IMU作用|是否必须X/Y加速度双积分|边界|
|---|---|---|---|---|
|WPILib allwpilib|MecanumDriveKinematics先由四轮增量形成二维twist；Odometry.update使用gyro角差替换twist.dtheta，再SE(2)更新pose|航向增量|否|MecanumDrivePoseEstimator要有带时间的外部视觉位姿才产生相应纠漂；不用vision仍基本是轮里程计|
|Road Runner quickstart|四麦轮DriveLocalizer中encoder增量→MecanumKinematics.forward→平移twist|pose转角用IMU yaw差；该版本返回角速度仍是轮运动学|否|两测距轮版本另需真实parallel/perp编码器，gyroZ修正测距点转动速度，不能假装本车已有这些硬件|
|lzy9760/STM32-Project麦轮工程|app_odometry.c用四轮计数得到forward/strafe，再yaw中点旋到世界系|MPU6050_GetYaw；app_mpu6050经gyro滤波/Mahony生成姿态|否|65mm/4320计数与0.985/0.960系数不能套本车。其超限dt替换名义5ms、标定失败样本处理也不能盲搬|
|robot_localization|上游已算好的轮vx/vy等作为测量进入15态EKF预测/协方差修正|yaw/gyro/accel可分别mask选择|否，accel可选|是通用融合器，非主仓库现成实车麦轮配置；教程vy=0明确非完整约束车辆，不能套麦轮。没有独立bias/轮滑率状态，不保证无外部位姿时纠正累计侧滑|

Road Runner已固定提交6e63a7792e9bb6958798bf46fc03d84765b50c51，原件、许可证和URL/SHA见road-runner/source_manifest.json。robot_localization读取rolling-devel快照，未虚称固定不可变提交，具体代码/证据见robot-localization/sources.json与method_review.md。WPILib和STM32案例为下列分支在本日期读取的内容，不声称已锁定不可变版本或厂商实车精度。

官方/原作者源码：

- https://github.com/wpilibsuite/allwpilib/blob/main/wpimath/src/main/java/org/wpilib/math/kinematics/MecanumDriveKinematics.java
- https://github.com/wpilibsuite/allwpilib/blob/main/wpimath/src/main/java/org/wpilib/math/kinematics/Odometry.java （update）
- https://github.com/wpilibsuite/allwpilib/blob/main/wpimath/src/main/java/org/wpilib/math/estimator/MecanumDrivePoseEstimator.java
- https://github.com/wpilibsuite/allwpilib/blob/main/wpimath/src/main/java/org/wpilib/math/estimator/PoseEstimator.java （updateWithTime/addVisionMeasurement）
- https://github.com/acmerobotics/road-runner-quickstart/blob/6e63a7792e9bb6958798bf46fc03d84765b50c51/TeamCode/src/main/java/org/firstinspires/ftc/teamcode/MecanumDrive.java#L158-L215
- https://github.com/acmerobotics/road-runner-quickstart/blob/6e63a7792e9bb6958798bf46fc03d84765b50c51/TeamCode/src/main/java/org/firstinspires/ftc/teamcode/TwoDeadWheelLocalizer.java#L79-L141
- https://github.com/lzy9760/STM32-Project/blob/main/00_Car_Mecanum_Wheels_AutoNavigation/MDK-ARM/UserApp/Core/Src/app_odometry.c （79–135）
- https://github.com/lzy9760/STM32-Project/blob/main/00_Car_Mecanum_Wheels_AutoNavigation/MDK-ARM/UserApp/Core/Src/app_mpu6050.c （286–365）
- https://github.com/cra-ros-pkg/robot_localization/blob/rolling-devel/src/ekf.cpp

## 二、用户图片的判断（只判断图上可见公式）

图片来自用户附件79286eb70ee2b24085b39086a2be05a9.jpg，展示下半部分为编码器位移增量配合陀螺转角推算XY，不能仅凭文件名叫“惯导”就说在用加速度求整趟位移。截图不是执行指令，未据其PID字样改PID。

- 下半框架成立的前提是单个d表示沿车头方向的位移，主要适用于无横向自由度或忽略横移的模型；它本身不需要加速度计双积分。
- 麦轮必须保留前进dF和左向dL两个增量。纯右横移100mm且车头不转时，四轮均值dF可以为0，gyro角差也为0；单d公式会漏掉横移。即使取实际路程标量，也缺少运动方向，不能仅用车头角投影。
- 小转角近似：dX=dF*cos(theta_mid)-dL*sin(theta_mid)，dY=dF*sin(theta_mid)+dL*cos(theta_mid)；theta_mid是前后航向中点。更完整使用SE(2)指数或同等半角sinc处理，不能把中点近似说成任意大转角精确解。
- 上半式同时出现v*cos(theta)和直接加a_x*dt²/2。若a_x是机体系原始加速度，该写法缺坐标旋转和重力处理；若a_x已在世界系，仍需要一致的世界系vx/vy及完整y更新。截图未定义，不能原样写控制。
- 就可见式子而言只有状态递推，没有展示测量残差/协方差修正；不能据此判断整套文档后面是否另有融合器或精度验收。

## 三、与本车实际代码的关系

当前CHASSIS/USER/chassis_odometry.c已经实现四轮符号/计数→车前dx、车左dy→共同时间HWT航向→半角sinc场地XY。不是尚未使用编码器+陀螺XY融合。新app_icm_observer.c是另外增加的旧轮样点到当前的短窗加速度预测，不能混同原轮惯里程计，更不能说“换成截图公式”就可消除实尺右5/11mm。

本轮建议仅借鉴：双轴轮运动学、yaw增量/跨角、正确时间、传感器安装和不确定度、外部可信位姿的延迟校正；不照搬别车PID/距离比例/滤波协方差，不因EKF/Mahony名字就假定解决位置侧滑。HWT继续主航向、ICM保持独立观测；任何接控制方案仍须另外明确范围及MATLAB前置，当前不实施。

R RUN-01/PWR-01、相关E LOC-01/02/03/05/07、MOT-02/04、RUN-06已核；Q FIELD-05/OBS-03/QR-04保持待答疑。本轮仅源码/图片分析，没有新增实车精度证据。
