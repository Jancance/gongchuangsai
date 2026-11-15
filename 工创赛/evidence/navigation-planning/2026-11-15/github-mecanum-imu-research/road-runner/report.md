# Road Runner：麦轮编码器与IMU实际融合路径

2026-11-15只读公开源码研究。只核 `acmerobotics/road-runner-quickstart`，未运行上游代码、安装库、我方Git fetch/clone、改产品或连接硬件。

## 版本与可复查入口

- 分支：`master`；官方GitHub源码页内`currentOid`：`6e63a7792e9bb6958798bf46fc03d84765b50c51`。
- 页面：`https://github.com/acmerobotics/road-runner-quickstart/blob/6e63a7792e9bb6958798bf46fc03d84765b50c51/TeamCode/src/main/java/org/firstinspires/ftc/teamcode/MecanumDrive.java`。
- 同目录的 `TwoDeadWheelLocalizer.java`。已保存两份master原始字节和两份commit固定原始字节，逐份SHA相同；许可证另存。GitHub匿名API限流，提交号改从官方blob页面元数据取得并用固定commit raw复核；没有编造提交时间。
- 本报告使用下载原件**1-based物理行号**。网页文本提取会折叠空行，工具显示行号不能直接当GitHub `#L` 行号。

## 四驱动轮版本：位移来自编码器，航向增量来自IMU

`MecanumDrive.java` L123–215的`DriveLocalizer`：

1. L158–168依次读取四轮`PositionVelocityPair`和IMU yaw，yaw明确取弧度。L170–181第一次只保存四轮位置和航向，返回零速度，不把历史累计encoder读数算成本次位移。
2. L183–201用四个encoder位置差及轮速度做麦轮正运动学，乘`inPerTick`；平移来自所得`twist.line`。L210–213合成位姿时，角增量换成`heading.minus(lastHeading)`所得IMU yaw增量。这是按不同可观测量分工，不是这段代码内的EKF或线加速度双积分。
3. 容易漏看：L215直接返回原`twist.velocity()`，因此此版DriveLocalizer返回的角速度仍来自轮正运动学，不能笼统宣称其角速度也已替换为gyro。
4. L66–68有`inPerTick/lateralInPerTick/trackWidthTicks`，L97把独立横向比例放入运动学。长度口径为英寸，角为弧度；默认1/0是待调模板，不能复制成本车参数。`kA`、加速度约束属于运动命令/前馈，不是IMU线加速度观测。

这两个定位类没有读取IMU linear acceleration，也没有把其X/Y加速度整趟双积分成位置；不能据此推广成“所有Road Runner或所有开源项目都不用加速度”。

## 两测距轮版本：用gyro扣除转动带来的偏置分量

`TwoDeadWheelLocalizer.java`：

- L30–31、48–64、79–90依赖两路独立`par/perp`编码器输入及IMU，安装参数以encoder tick为单位。这是另一套测距硬件/输入结构；使用`DcMotorEx`接口是读FTC编码器端口，不代表可以把本车任意两只麦轮直接当两只测距轮。
- L95–103用IMU yaw增量，同时取得gyro Z角速度。角速度先用度接口读取后转弧度；跳变偏置处理在源码注明针对FTC SDK历史问题，不能当本车通用校准/滤波照搬。
- L115–132的平行/垂直位移与速度分别减去对应安装参数乘yaw增量/gyro rate，随后乘`inPerTick`。L140–141再更新pose并返回速度。其价值是校正测距点不在旋转中心时的运动分量，不是识别所有地面滑移，也没有线加速度双积分。

## 时序、坐标与初始化边界

- 四轮顺序为LF/LB/RB/RF；本车原序为LR/RR/LF/RF，方向与比例必须逐项映射，不能按数组下标照搬。
- 上游使用SDK已经转换到robot语义的yaw与Road Runner的局部twist/pose组合，IMU安装方向由`RevHubOrientationOnRobot`配置（Mecanum L60–63、243–244）。本次未跨仓库追读依赖库内部的`MecanumKinematics/Rotation2d/Pose2d`定义；其完整左右正号、复合/跨界细节不冒称已独立核实。类内明确用`Rotation2d.exp`和`minus`，没有直接把两个包角double硬相减。
- L147–149的`setPose`只改pose，不清encoder/heading基线。首次update的yaw差分隐含相对初始pose的角偏移；这里没有调用`resetYaw()`。不能把运行中SetPose误当硬件编码器清零。
- Mecanum L223启用FTC Hub AUTO bulk cache，但定位类本身没有按四轮/IMU各自时间戳插值到共同采样时刻。TwoDead保留angular acquisitionTime作对象/记录；其update内没有用该时间做异步对齐。SDK可能另有缓存语义，本次不声称已经验证其总硬件同步精度或更新频率。

## 与本车当前实现的关系

实际本车 `3.stm32f407工程/CHASSIS/USER/chassis_odometry.c`：L25–41历史插值，L97–113 HWT yaw展开，L115–141取最旧新鲜共同样点并对四轮/HWT插值；L158–166以四轮增量给车前/车左平移，用HWT yaw增量的半角/sinc方式累加全局位姿。L50–58显式初始位姿/heading offset，L152–155丢连续性后不悄悄拼接。

因此本车**原里程计已经是轮增量平移＋HWT航向增量的同类分工**；新ICM模块只是额外短时延迟观测，不是原车此前完全没有陀螺融合。上游四轮方案比我们少了类内显式共同时间插值，不能假定简单移植会解决本车22ms串行反馈时序。

工程推论：yaw能约束转角和旋转坐标投影，却不能唯一观测“姿态几乎不变时额外横滑了多少”。两测距轮提供不同的位移测量来源，但本身也不是无漂移全局定位。此源码没有以外部地面/视觉绝对观测纠正所有累计滑移的机制，不能据此签署本车右偏已解决。

可借鉴的是传感器分工、先建立差分基线、独立横向比例、角度类型和测距点旋转补偿思想。不可直接照搬FTC Java/SDK接口、英寸/弧度参数、轮序方向、默认标定值、SDK gyro workaround和未核实的时序假设；没有新增C或模型。

已核相关R RUN-01/PWR-01及E LOC-01/02/03/05/07、MOT-02/04、RUN-06；本次只有研究判断，不改规则或控制。Q保持待技术答疑确认。未闭环：依赖库轴约定/角跨界内部实现、FTC实际硬件时间同步、上游实车误差，以及本车独立地面观测方案。
