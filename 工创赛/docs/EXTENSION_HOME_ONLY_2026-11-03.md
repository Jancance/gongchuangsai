# 当前单项：伸缩回零，夹爪上电145°

> 当前已转[联合启动](STARTUP_GROUP_2026-11-03.md)：VEHICLE_STARTUP_GROUP_ONLY=1，两个旧单项开关=0。4310、伸缩、载物盘联合，夹爪145°保持；以下保留原单项/八段交接。三个模式全部为0才进入原八段。

> 后续当前入口：[4310单项](DM_STARTUP_ONLY_2026-11-03.md)。伸缩已获用户单次接受，不重跑；如需切回本页，VEHICLE_EXTENSION_HOME_ONLY=1、VEHICLE_DM_STARTUP_ONLY=0；两个开关都为0才进入原八段。以下保留伸缩单项交付与实物反馈。

> 最新反馈（2026-11-03，E）：用户在本次步骤后回复“没有问题”，按本次口述接受伸缩单次回零与夹爪上电145°的单项功能，不重复该项。未提供状态遥测、触挡细节或板端镜像独立证据，不补写8/1/1、不扩大为机械重复原点、失电恢复或整车验收。当前VEHICLE_EXTENSION_HOME_ONLY仍为1，保持仅回零模式。证据：evidence/integration/2026-11-03/extension-home-only/user-observation.json。

2026-11-03，来源为用户在整合1上传后明确“我现在只需要伸缩机构回零即可”，并追加“夹爪初始化为145°”。这是本轮现场调试E选择，替代当前先读手摆参考再跑第一段的安排；原八段与并行固定目标实现保留为可切回分支，不删除历史证据。

## 使用

实际工程仍为`D:/新建文件夹4/工创赛/代码/stm32/3.stm32f407工程/gongxun-f407.uvprojx`，选择Vehicle_Integrated。参数仍只在USER/vehicle_parameters.h、Keil分组00_现场参数。

- STARTUP_GRIPPER_ANGLE_DEG=145.0f：上电夹爪初始化。动作组闭合135°不改。
- VEHICLE_EXTENSION_HOME_ONLY=1：本轮仅伸缩回零，优先于VEHICLE_TEST_STAGE等八段参数；设0可切回原整车入口，需要重新构建与下载。
- 固定目标的STARTUP_EXTENSION_REFERENCE_VALID仍为0，不需要靠填有效位才能回零，也不将驱动完成自动变成机械原点验收。

先空爪、支承升降/机械臂、确认缩回通路及止挡，下载本次Vehicle_Integrated后运行。夹爪上电到145°；短按PC13一次，执行节点6单次向缩回端碰撞回零。按键前不自动回零。完成后保持供电，不进入底盘路线、不触发4310/升降/载盘动作、不登记搬运次数。运行或保持时再按PC13取消并撤销VM，故障/取消不自动重试；VM共用，停机前保持重力机构支承。

原平台、CAN健康、看门狗、任务时效及故障锁存保护保留。回零前只读核对方向0、30RPM、驱动10000ms、检测300RPM/800mA/60ms及上电自动回零0；不匹配拒绝，不写驱动菜单或改变阈值。准备2.5秒、启动后主控11秒、服务间隔150ms；启动ACK丢失也按可能已启动处理，失败/取消由唯一CAN任务尝试退出回零与停止，停止未确认继续锁定。

## 观察与证据

只读查看g_extension_home_state/result/reported：8/1/1代表本轮驱动完成报告，结合实际缩回/触挡后停止观察确认；不能仅凭该数值签机械重复性或绝对位置。g_startup_extension_current_deg为完成后的本轮驱动角，不写回驱动零点，不自动恢复失电后的机械参考。

故障result：3超时、4通信/选项、5驱动状态、6平台互锁/健康、7回零参数不符、8服务间隔。g_vehicle_state=4在本模式仅表示回零单项结束，g_vehicle_stage_mask保持0，不表示第1段跑完。

改动6文件：USER/vehicle_parameters.h、CHASSIS/USER/startup_axes.c、extension_home_only.inc、vehicle_mission.inc、main.c及CHASSIS/SYSTEM/CAN/can.c。其余1856个既有源码/工程文件SHA保持；原件差分备份及安装SHA在evidence/integration/2026-11-03/extension-home-only/manifest.json与before-install/。实际构建以manifest和actual-*.log为准，未烧录/复位/发硬件命令。

软件桩检查：真实回零控制器55项+离线2项，真实CAN传输/会话捕获18项+离线9项，真实按键流程7项，合计91项通过；涵盖不自动启动、仅节点6、参数不符、旧通知/错节点/坏帧、三帧顺序、准备/运行超时、ACK丢失、取消及停止未确认锁定、完成后不进入路线。正常模式六组软件检查和比赛profile防误用检查通过；PWM初始化已按145°检查。桩检查不等于硬件闭环、碰撞止挡承载/停止或机械原点通过。

## R/E/Q

已核R：RUN-01/04、PWR-01、MECH-01/02；E：RUN-06、LOC-03、CAL-05。本轮为独立工程调试，不作为出发前自动展开或正式自主运行验收。本次单项功能已获用户“没有问题”反馈；触挡细节/机械重复性与原点独立证据、失电后参考、板端镜像与遥测独立核对、原整车八段实测仍未闭环；FIELD-05/OBS-03/QR-04等官方Q继续**待技术答疑确认**。无Git提交或推送，之前远端5bf4995不包含本轮新改动。
