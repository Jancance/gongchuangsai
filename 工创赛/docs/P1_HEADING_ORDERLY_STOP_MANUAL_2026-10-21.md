# 目标航向执行层：正常停车接口（用户手动修改）

实际工程：`D:/新建文件夹4/工创赛/代码/stm32/3.stm32f407工程/gongxun-f407.uvprojx`。只修改该工程USER/chassis_auto_test.h和.c，不修改项目内旧副本。助手只生成说明与隔离候选，未修改实际固件。

前置证据：用户完整Rebuild37952/576/1392/25408、0错0警告；本聊天截图确认跨任务+5RPM交接及取消后bus_valid1/rpm0/stop1、state5/cause4。这些用例已完成，不重复测试。

本步补足正常结束路径：请求后由原CAN状态机逐台停止，沿用原反馈采集及零速检查，完成后结束整个序列并请求关VM。异常仍使用原Auto_CutLocked兜底。当前不改main、不接入自动调用，不放行运动。

## 1. 头文件声明

退出Debug。在USER/chassis_auto_test.h已有`void ChassisAuto_Abort(void);`后添加：

```c
/* 1接受正常停车请求，0当前阶段不接受；并非物理停车完成。 */
uint32_t ChassisAuto_RequestStop(void);
```

## 2. 私有状态

在USER/chassis_auto_test.c找到：

```c
static ChassisMotorCommand s_commands[3][4];
```

在其后添加：

```c
/* 正常停车后结束本次序列，记录首次请求时刻。 */
static uint32_t s_finish_requested = 0U;
static TickType_t s_finish_tick = 0U;
```

## 3. 初始化和新会话都清除新状态

在ChassisAuto_Init内，紧接`s_vm_requested = s_used = s_mode = 0U;`后添加：

```c
    s_finish_requested = 0U;
    s_finish_tick = 0U;
```

在ChassisAuto_Begin成功分支中，紧接`s_total_tick = xTaskGetTickCount();`后也添加同样两行（保持原分支缩进）：

```c
            s_finish_requested = 0U;
            s_finish_tick = 0U;
```

## 4. 首次请求的停车截止

在Auto_CheckLocked内找到以下已有行，保留原行：

```c
    if (!s_run_started || (state == CHASSIS_AUTO_SETTLE)) return;
```

紧接其前添加：

```c
    /* 从首次正常停车请求计时；停帧未齐则走原VM兜底。 */
    if ((s_finish_requested != 0U) &&
        ((TickType_t)(now - s_finish_tick) >=
         pdMS_TO_TICKS(CHASSIS_AUTO_STOP_MS)) &&
        (g_chassis_auto_stop_mask[g_chassis_auto_segment] != 0x0FU))
    { Auto_CutLocked(6U); return; }
```

这里检查的是四台停止交易完成标志：500ms内未齐则请求故障关VM。它不是500ms物理刹停承诺；之后仍须真实新鲜零速反馈及原SETTLE时限检查。沿用原CHASSIS_AUTO_STOP_MS工程候选值，没有放宽原反馈/故障保护。

## 5. 正常停车后不再进入下一运动段

在ChassisAuto_CanStep的`if (state == CHASSIS_AUTO_SETTLE)`分支中，将唯一的：

```c
            if (g_chassis_auto_segment == 2U)
```

替换为：

```c
            if ((s_finish_requested != 0U) || (g_chassis_auto_segment == 2U))
```

其余DONE/result/active/cut_vm及else里的旧逻辑保留。没有正常停车请求时，原三段流程照旧。

## 6. 文件末尾追加接口

在chassis_auto_test.c现有所有函数结束后，追加：

```c
/* 请求有序结束本次序列：只置标志，由唯一CAN任务逐台停车。 */
uint32_t ChassisAuto_RequestStop(void)
{
    uint32_t accepted = 0U;
    taskENTER_CRITICAL();
    if ((g_chassis_auto_active == 1U) &&
        (g_chassis_auto_cut_vm == 0U) &&
        (g_chassis_auto_state >= CHASSIS_AUTO_START) &&
        (g_chassis_auto_state <= CHASSIS_AUTO_SETTLE))
    {
        /* 重复请求不能延长第一次请求的停车截止。 */
        if (s_finish_requested == 0U)
        {
            s_finish_requested = 1U;
            s_finish_tick = xTaskGetTickCount();
        }
        s_stop_due = 1U;
        accepted = 1U;
    }
    taskEXIT_CRITICAL();
    return accepted;
}
```

该函数只置标志，不操作CAN、不等待；允许在START/RUN/STOP/SETTLE阶段请求。准备阶段和结束后返回0，不能忽略返回值；未来会话在准备阶段取消或故障时走原Abort/VM兜底，不强行当作正常完成。重复请求不会延长首次截止。

原CanStep已有s_stop_due优先进入STOP，因此无需新增发送分支。请求发生在一次速度交易正在进行时，最多还有该笔在途交易；下一轮会转STOP，不保证已进入收发函数的交易瞬时撤销。首个正常请求的截止仍由独立Safety检查。

重要语义：本接口完成后的g_chassis_auto_result=1仅表示该执行序列的停车/反馈流程完成，不等于目标角已到位。未来会话必须结合heading_controller的DONE/FAULT/CANCELLED等结果判定任务；控制器SETTLING阶段的临时停转不能直接调用本接口结束会话。本次新增API尚无main调用。

## 7. 当前只验收构建

保存两个文件，完整Rebuild，把完整日志发来。保持APP_HWT_STATIC_TEST=1、APP_HWT_TURN_TEST=0。当前不修改main、不设置VM请求、不下载。后续才加入动态调速及受控目标角会话，不能把仅有停车API写成闭环已完成。

## 验证证据及未闭环

隔离候选位于tmp/heading_stop_20261021/chassis_auto_test_candidate.c，由当前真实源生成，仅在候选加入上述改动；测试脚本tmp/check_heading_stop_20261021.py。主机测试用mock CAN/tick/临界区，实际执行候选原状态机函数及现有麦轮转换：

- 第一段提前正常停车后四台停止且不再发后续段速度；最终stop_mask0F。
- 首笔速度交易中收到请求后停止，后续不再发其余启动速度。
- 重复请求不刷新截止；首次请求499ms尚可等待、500ms停帧未齐即故障6并请求切VM。
- 停止应答失败故障5、持续非零速度故障7、反馈过期故障3，均请求切VM。
- VM请求撤销后不再继续CAN动作。
- 无新请求时原三段仍完成12次速度和12次停止交易。
- 所有mock CAN调用均确认不在临界区内。

GCC C99 Wall/Wextra/Werror构建并运行通过。使用实际工程IncludePath和宏定义，ARMCC V5 Cortex-M4候选chassis_auto_test.c编译退出0，无诊断。未完成实际工程本次用户修改后的链接、RTOS并发或硬件验收；mock的零速和关VM标志不是物理证据。

R已检查RUN-01/PWR-01；E已检查LOC-03/05、RUN-06、MOT-02/04。官方Q保持待技术答疑确认。未闭环：动态调速、真实目标角会话整合、实时执行时效/反馈与物理停车、转角精度和X/Y。
