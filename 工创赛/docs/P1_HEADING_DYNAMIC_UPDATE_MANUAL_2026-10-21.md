# 航向执行层：整组动态轮速更新（用户手动修改）

实际工程固定为`D:/新建文件夹4/工创赛/代码/stm32/3.stm32f407工程/gongxun-f407.uvprojx`。只改USER/chassis_auto_test.h/c，助手没有改实际源或工程。保持文件原编码。

最新实际构建38044/576/1400/25408、0错0警告/4秒已核对，Init状态清零及BeginHeading已保存。本步沿用现有准备、监测、停止和总时限，不自动下载或运动。

## 1. 添加接口声明

退出Debug。在chassis_auto_test.h的ChassisAuto_BeginHeading声明后添加：

```c
/* 同一转向的下一组轮速更新；1接受排队，0拒绝，不表示已发完。 */
uint32_t ChassisAuto_UpdateHeading(int32_t command_rpm);
```

## 2. 添加私有变量

在chassis_auto_test.c的`static TickType_t s_finish_tick = 0U;`后添加：

```c
/* 仅BeginHeading会话使用，旧三段测试不走动态更新。 */
static uint32_t s_heading_session = 0U;
static int32_t s_heading_initial_rpm = 0;
static int32_t s_heading_pending_rpm = 0, s_heading_applied_rpm = 0;
static TickType_t s_heading_update_tick = 0U;
```

## 3. 两处清零与一处设置

在ChassisAuto_Init内，紧接已有`s_finish_tick = 0U;`后添加：

```c
    s_heading_session = 0U;
    s_heading_initial_rpm = s_heading_pending_rpm = s_heading_applied_rpm = 0;
    s_heading_update_tick = 0U;
```

在ChassisAuto_Begin成功分支内，紧接该处已有`s_finish_tick = 0U;`后也添加同样三行，使用原分支缩进。这样旧启动入口始终清除航向模式，不受新接口影响。

在ChassisAuto_BeginHeading成功分支内，紧接`g_chassis_auto_segment = 2U;`后添加：

```c
        s_heading_session = 1U;
        s_heading_initial_rpm = command_rpm;
        s_heading_pending_rpm = s_heading_applied_rpm = command_rpm;
        s_heading_update_tick = xTaskGetTickCount();
```

该处分支末尾复制commands到s_commands的原for循环保留。

## 4. CAN任务在整组边界读取新目标

在ChassisAuto_CanStep内找到：

```c
    state = g_chassis_auto_state; session = g_chassis_auto_session;
```

紧接其后、原s_stop_due判断之前插入：

```c
    /* 仅在整组边界接纳新目标，避免一组四轮混入两个不同轮速。 */
    if ((s_heading_session == 1U) && (s_stop_due == 0U) &&
        (s_finish_requested == 0U) &&
        ((state == CHASSIS_AUTO_RUN) ||
         ((state == CHASSIS_AUTO_START) && (s_motor_index == 0U) &&
          (s_monitor_between == 0U))) &&
        (s_heading_pending_rpm != s_heading_applied_rpm))
    {
        if (Chassis_HeadingCommands(s_heading_pending_rpm, s_commands[2]) != 1U)
        {
            Auto_CutLocked(8U);
            taskEXIT_CRITICAL();
            return;
        }
        s_heading_applied_rpm = s_heading_pending_rpm;
        s_heading_update_tick = now;
        g_chassis_auto_start_mask[2] = 0U;
        Auto_EnterLocked(CHASSIS_AUTO_START, now);
        state = CHASSIS_AUTO_START;
    }
```

一次START最多发当前组的四台目标，中途新目标只存pending，下一次整组边界再用。批内四台仍依原总线逐台交易，不宣称四台物理同步。重复提交相同值不会重复启动一组。停车请求或原截止一旦生效，不接受新调速覆盖它。

## 5. 首组启动时记录方向反馈检查起点

在ChassisAuto_CanStep内，将已有的：

```c
    if ((state == CHASSIS_AUTO_START) && !s_run_started)
    { s_run_tick = now; s_run_started = 1U; }
```

替换为：

```c
    if ((state == CHASSIS_AUTO_START) && !s_run_started)
    {
        s_run_tick = now;
        s_run_started = 1U;
        if (s_heading_session == 1U) s_heading_update_tick = now;
    }
```

s_run_tick仍只在首次启动时设定，后续调速不重置10秒硬上界。

## 6. 更新后的方向反馈判定

在Auto_CheckLocked速度检查的for循环内，紧接`int32_t raw = g_chassis_auto_driver_rpm[i];`后添加：

```c
            TickType_t direction_age = (s_heading_session == 1U) ?
                (TickType_t)(now - s_heading_update_tick) : run_age;
```

随后找到原有的方向检查开头：

```c
            if (((TickType_t)(now - s_speed_tick[i]) <= run_age - pdMS_TO_TICKS(500U)) &&
```

只将这一行替换成以下三行，后面原方向条件及Auto_CutLocked(8U)保留：

```c
            if ((s_commands[g_chassis_auto_segment][i].speed_raw != 0U) &&
                (direction_age >= pdMS_TO_TICKS(500U)) &&
                ((TickType_t)(now - s_speed_tick[i]) <= direction_age - pdMS_TO_TICKS(500U)) &&
```

零速目标不判断方向；非零目标仍检查来自当前组更新后500ms的新反馈。原绝对超速、状态故障和反馈新鲜度检查全部保留。这里是原速度检查模式的延续，不构成已验证的速度跟踪容差或制动性能。

## 7. 文件末尾追加提交接口

在chassis_auto_test.c现有所有函数结束后添加：

```c
/* 仅提交下一组轮速；CAN任务在整组边界取用，不在这里发帧。 */
uint32_t ChassisAuto_UpdateHeading(int32_t command_rpm)
{
    uint32_t accepted = 0U;
    taskENTER_CRITICAL();
    if ((s_heading_session == 1U) && (g_chassis_auto_active == 1U) &&
        (g_chassis_auto_cut_vm == 0U) && (s_finish_requested == 0U) &&
        (s_stop_due == 0U) &&
        (g_chassis_auto_state >= CHASSIS_AUTO_DISABLE) &&
        (g_chassis_auto_state <= CHASSIS_AUTO_RUN))
    {
        /* 当前会话禁止反向：后续完成零速反向互锁后再扩展。 */
        if ((command_rpm < -5) || (command_rpm > 5) ||
            ((s_heading_initial_rpm > 0) && (command_rpm < 0)) ||
            ((s_heading_initial_rpm < 0) && (command_rpm > 0)))
            Auto_CutLocked(8U);
        else
        {
            s_heading_pending_rpm = command_rpm;
            accepted = 1U;
        }
    }
    taskEXIT_CRITICAL();
    return accepted;
}
```

只接受已开始的BeginHeading会话，准备阶段可先更新待用目标，旧三段/非活动会话拒绝。正常停车/停止阶段拒绝调速。0RPM表示会话内暂时停转，不结束会话；最终成功/取消/故障须由后续上层区分后走RequestStop或Abort/VM兜底。接口返回1是接受下一组目标，不是四台都已更新成功。

本阶段同会话反向或超范围指令会故障8并请求关VM，不能忽略返回0重试绕过故障。控制器超调后输出反向时也会安全失败，尚不支持完整超调纠偏。将来须完成真实零速确认、反向过渡与截止设计/测试后再扩展，不能简单去掉保护。

## 8. 当前唯一验收动作

保存h/c，完整Rebuild并发送日志。main不调用新接口，APP_HWT_STATIC_TEST=1、APP_HWT_TURN_TEST=0保持。本轮无需下载，不要用Watch开启VM。后续还要接入受控会话/源指令时效与HWT失效、最终停车，再做对应实物验证；不可在现阶段直接发g_heading_live_motor数组。

## 验证证据及范围

候选由当前实际源生成，位置tmp/heading_dynamic_20261021/chassis_auto_test_candidate.c，脚本tmp/check_heading_dynamic_20261021.py。GCC C99 Wall/Wextra/Werror最终构建及运行通过；第一次检查程序的缩进警告已在隔离检查代码中修正，不涉及实际源码。mock CAN/tick执行候选状态机验证：5到3的四轮组完成前再提交1，上一组仍全3、下一组全1；重复值不重发；零速保持后同向恢复；停车优先；负方向更新；准备期间更新；反向/越界故障；旧会话拒绝；调速不重置10秒上界。此前正常停车、应答失败/超时/反馈过期、单次旋转和原三段流程回归也通过。实际头文件/宏下ARMCC V5候选编译成功，无诊断。

测试没有真实RTOS并发或硬件，不签实际四轮同步/速度精度/刹停通过；没有在实际工程本次手改后链接或下载。动态API本身不判断源数据时间，后续会话必须从Read快照验证新鲜度、HWT状态和最终安全条件并锁存故障；不得把API接受等同于安全许可。MOT-02的加速度/加加速度约束仍未实现，本步只提供离散目标更新。

R检查RUN-01/PWR-01。E检查LOC-03/05、RUN-06、MOT-02/04。官方Q继续待技术答疑确认。未闭环：动态目标角会话及源时效/失效整合、超调反向互锁、完整闭环与物理停车、实时X/Y。
