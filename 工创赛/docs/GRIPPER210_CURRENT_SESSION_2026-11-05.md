# 本轮待机夹爪单次打开210°

用户明确要求先打开夹爪到210°。本轮新鲜无复位只读核对当前Flash148728字节与旧AXF完全一致、RUNNING/run0/stage1/state0，夹爪PWM1574us对应名义145°；CAN/RTOS/安全故障、任务/轴/按键请求均无异常，PA0为TIM2_CH1。

先在用户本机MATLAB R2020a完成本次145→210的PWM信号验证，8项检查0失败：83个五次缓动参考步、每步PWM变化不超过11us、最终2056us。它只验证信号序列，不等于舵机动力学、物理角度、电流或机械干涉通过。MATLAB通过后才编写本次执行脚本。

随后再次attach并逐字节核对Flash，在每个参考步前后核对待机/故障/请求/PWM和引脚状态，仅将TIM2_CCR1(0x40000034)从1574缓动到2056us；未重复写相同PWM，实际写入75次，参考步最小间隔30ms（USB读回使实际间隔更长）。结束连续三次读回2056us且run0/stage1/state0和其他受检状态保持，总调整与保持观察约9.516秒。

不写固件、不复位/暂停/恢复CPU、不改业务变量、不触发路线、不发DM/伸缩/升降/载盘命令。本次为用户明确授权的现场调试E，不是正式自主动作。g_source_prepare_grip_deg仍为启动历史命令145，不篡改该字段，也不将其当作本次舵机反馈。当前开爪临时命令以CCR1=2056及execute-result.json为准；物理开度/干涉仍待用户观察。

原料区永久210°源码状态沿用SOURCE_GRIP210_PERSISTENT_2026-11-05.md，尚未烧录；此次不改变旧固件重启后的初始化行为。当前底盘可能位移，新原图需求和旧纠偏不可执行边界保持。

证据：evidence/integration/2026-11-05/gripper210-current-session/下readonly-sync-report.json、matlab-signal-plan.json/.mat/.log、execute-result.json与final-manifest.json；脚本为scripts/validate_gripper210_current_matlab_20261105.m和scripts/gripper210_current_session_once_20261105.py。

已核R RUN-01、PWR-01、MECH-01/02；E RUN-06、VIS-15。命令与当前运行记录属E，R不变，原官方Q继续待技术答疑确认。未闭环：准确210°的物理反馈、现场干涉、永久版烧录以及后续视觉/三位置抓放。
