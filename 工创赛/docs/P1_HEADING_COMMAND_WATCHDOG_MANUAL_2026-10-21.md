# 会话整合前置：独立指令提交超时保护

实际工程：`D:/新建文件夹4/工创赛/代码/stm32/3.stm32f407工程/gongxun-f407.uvprojx`。本轮只手改USER/chassis_auto_test.c。原生FreeRTOS/SPL和原编码保持；助手未修改实际源。

当前已核对用户Rebuild38316/576/1420/25404、0错0警告，动态接口4处问题均已修正。本步不重复之前的数据交接用例。

目的：在目标航向会话中，独立Safety路径发现上层停止提交指令时请求切VM，避免一直保持旧轮速至10秒。它监督任务提交时效，不能单独证明HWT数据新鲜，也不是完整会话。正式会话接入前不调用BeginHeading，不直接开放运动。

## 1. 增加提交时间变量

退出Debug。在`static TickType_t s_heading_update_tick = 0U;`后添加：

```c
/* 最近一次合法轮速提交时间，与实际四轮更新起点分开。 */
static TickType_t s_heading_command_tick = 0U;
```

## 2. 两处初始化各补一行

ChassisAuto_Init和ChassisAuto_Begin内各有一处`s_heading_update_tick = 0U;`。在两处的这一行后分别添加，保持对应缩进：

```c
    s_heading_command_tick = 0U;
```

不要在文件顶部的static定义后重复添加，也不修改ChassisAuto_CanStep里的`s_heading_update_tick = now;`。

## 3. 新航向会话记下首次提交时间

在ChassisAuto_BeginHeading成功分支内，紧接：

```c
        s_heading_update_tick = xTaskGetTickCount();
```

添加：

```c
        s_heading_command_tick = s_heading_update_tick;
```

## 4. 合法的新指令更新提交时间

在ChassisAuto_UpdateHeading接受分支内，紧接：

```c
            s_heading_pending_rpm = command_rpm;
```

添加：

```c
            s_heading_command_tick = xTaskGetTickCount();
```

这与s_heading_update_tick含义不同：同值但来自新采样的合法提交，也更新command_tick；四轮实际切组才更新update_tick。非法、反向或停车后被拒绝的提交不续期。

## 5. 加入独立监督检查

在Auto_CheckLocked内，紧接已有这行后：

```c
    if (!Auto_BaseHealthy()) { Auto_CutLocked(2U); return; }
```

添加：

```c
    /* 任务提交中断时撤销会话；正常停车继续由原截止和反馈监督。 */
    if ((s_heading_session == 1U) && (s_finish_requested == 0U) &&
        (s_stop_due == 0U) && (state <= CHASSIS_AUTO_RUN) &&
        ((TickType_t)(now - s_heading_command_tick) >=
         pdMS_TO_TICKS(CHASSIS_HEADING_MAILBOX_MAX_AGE_MS)))
    { Auto_CutLocked(3U); return; }
```

保留后面的总时限、准备时限和全部原保护。沿用CHASSIS_HEADING_MAILBOX_MAX_AGE_MS的100ms工程候选值，不是官方时限。检查范围包含新航向会话的准备和运行，因此未来BeginHeading后必须持续提交新指令；没有持续新指令时允许安全拒绝启动。旧三段模式s_heading_session=0不受此检查影响。

请求正常停车后不再依赖新调速提交，继续使用原500ms停帧截止和SETTLE零速反馈/2秒截止，不把正常停车误判成发布超时。运行异常直接锁存原故障3并请求切VM；源恢复不能用Update重新允许已故障会话。cause3同时用于现有反馈失效，当前不能仅靠该值区分具体根因，独立首故障日志仍需后续补齐。

## 6. 本轮只验收构建

保存此c文件，完整Rebuild，发送日志。main和h都不改，APP_HWT_STATIC_TEST=1、APP_HWT_TURN_TEST=0保持。本轮无需下载或开启VM。不要手动周期刷新command_tick来制造通过。

## 后续会话的强制使用条件

UpdateHeading必须基于新的、HWT有效的源快照，不能重复读取缓存就续期；未来会话需检查sequence变化、源快照到龄、HWT当前健康以及CAN/Safety条件。恒定目标速度也应由新真实采样续期。此接口自身没有源sequence参数，重放拒绝仍是未接入的上层责任。

100ms是任务提交超时阈值；Safety任务名义每20ms检查，实际调度/收发延迟及物理切电时间尚待测量，不能宣称100ms内车体已经停止。Safety路径已只读确认：main的Safety任务调用Safety_Update，safety.c内部调用ChassisAuto_SafetyUpdate，后者调用Auto_CheckLocked并通过cut_vm决定Power_Enable/Disable，不需新增硬件或Safety源码修改。

## 隔离证据及未闭环

脚本tmp/check_heading_command_watchdog_20261021.py；候选tmp/heading_command_watchdog_20261021/chassis_auto_test_candidate.c。实际候选状态机mock测试99ms有效/100ms超时（准备、运行），过期后不再发新速度、合法发布不能重启故障会话，正常停车跨100ms仍能完成，tick回绕及旧三段模式不受影响。连续提供新命令时复跑此前动态组切换、停车/故障、单次入口和旧三段回归，GCC C99 Wall/Wextra/Werror通过。实际头文件/宏下ARMCC V5候选编译通过，无诊断。

mock采用模拟tick和临界区计数，只证明候选状态处理，不签真实调度/HWT物理断线/切VM延迟或机械刹停。用户本次手改后整工程链接尚待日志。完整会话尚需源健康、sequence/时效绑定、启动互斥、到位与临时停转区分、取消/故障、10秒执行器与20秒控制器超时协调、反向互锁及停止确认等整合。

R检查RUN-01/PWR-01；E检查LOC-03/05、RUN-06、MOT-02/04。官方Q保持待技术答疑确认。真实目标角闭环、物理停车、实时X/Y未闭环。
