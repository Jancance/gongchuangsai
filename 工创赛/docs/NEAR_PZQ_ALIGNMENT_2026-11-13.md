# 第二段固定近侧取料接续（2026-11-13）

用户当前决定覆盖旧三位置调试方案：底盘定位并保持，近侧4310为180°、伸缩为0；用户让场地原料盘转动，视觉按任务码判断当前目标，再触发板端pzq单件连续抓放。车载槽盘和场地原料盘是不同机构。当前车载黄/红在1/2号槽已由用户接受；黑料未完成。

## 现场定位

- 当前实际固件仍为a93760f0。用户撤承托后，prepare-once-20261113-162738.json恢复180°/65mm观察姿态，各步软件完成。
- 新图source-observe-20261113-162852：15帧，盘面稳定，近黑实际存在，远侧两个位置已空。本机MATLAB compute_near_body_alignment_20261113沿用已验尺度及图像中心TCP/回零半径161mm模型，近侧偏差约[5.209,4.721]mm；后一个轴尚未绑定车前后，不作已验底盘指令。
- align-20261113-163159.json仅发一次右移5mm。板端同会话1、DONE、result1、停止mask[0,0,15]。旧主机读取另一套g_motion_auto_stop_ack_mask=0导致断言失败并发STOP；已按实际MotionReadStatus来源改读g_chassis_auto_stop_mask[2]，163337只读核对通过。没有重复右移。
- align-20261113-163411.json前移3mm未被接纳，命令序号仍1，无第二次移动。新鲜只读s_motion_session_open0、s_align_chain1，原因是完成后SourceBodyStep_WheelWindow提前退出，导致轮子guard关闭并使API会话终止。没有清状态绕过。
- 右移后新图source-observe-20261113-163522：15帧盘面稳定。6个固定特征相关性0.960～0.978、位移约[-14,+8～9]px，确认实际图像变化有横向分量；不能拿理想单轴5mm当物理纯右移。下一步用修订入口继续有界前移及新图，依据实际轴响应计算剩余平移，不用伸缩5.14mm补偿。

## 新隔离候选

工程tmp/source_near_pzq_20261113/project/gongxun-f407.uvprojx，在线目标Vehicle_LiftRuler，基线a93760f0。不覆盖团队工程或外部实车工程。

仅4个源文件变化：

- source_body_step.inc：纠偏链在DONE等下一请求期间保留WheelWindow，直到显式finish；实际健康、STOP、边界、同会话DONE检查保留。
- integration13_immediate.inc：原连续owner增加固定近侧单件模式。旧16号三件模式保留原180/195/165及0/156/160；新模式恒180/0，只完成指定槽的一件，最后回180/0。原140°闭爪接回升、37°转盘放料、118800/7200和50ms释放衔接不变。
- pzq_near_cycle.inc：独立单件触发模块；PZQNear_TryStart(slot,sequence,now)，slot1～3、非零新序号；忙/重复请求软拒绝，实际故障/取消仍STOP。g_pzq_near_request=1只请求一次，9取消；state1运行、2仅命令链完成、9故障。无物理抓牢/放稳造假，不更新正式库存。
- integration10_complete.inc：include和service接线。未往main增加新流程。

本机MATLAB34项前置检查通过；实际C旧三件owner、近侧三个槽单件owner、49/50ms、忙/重复/旧DONE、故障/取消通过；实际纠偏owner33断言通过，新增等待阶段guard保持检查。在线/Offline双构建0错误0警告。底层vehicle_source_parallel.inc、lift_ruler_calibration.c、safety.c逐字节与基线一致。源码原编码/换行保留，未关真实保护。

用户可靠承托并明确允许换版后，新候选4a612a34已于16:46:58烧录Verify OK；align-20261113-164722.json完整Flash读回匹配、RUNNING、IT待机、无安全/RTOS/IT故障、VM及PE2为0。未自动展开或移动，等待承托撤出。视觉到该单件入口的在线收件/触发尚未接通；提供入口不等于动态视觉闭环或第二段完成。原料转盘目前仍需静止，待底盘定位及触发入口就绪后再通知用户转动。

证据在evidence/integration/2026-11-13/near-pzq/及source-resume/body-align/，现场动作日志在2026-11-12/source-align/目录，文件时间戳20261113。本轮R核SRC-01/02、QR-01/05/06、RUN-01/02/04、FIELD-06、PWR-01、MECH-01/02；E为固定近侧选择、相机/TCP模型和MECH-06。人工调试不等于正式自主，动态窗口/抓牢/放稳和底盘实测误差未闭环；官方Q不变。隔离fetch核21522db，根目录无提交/remote，未合并或推送。
