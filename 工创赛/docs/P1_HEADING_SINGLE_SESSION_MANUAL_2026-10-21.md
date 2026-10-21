# 单次航向转动入口：用户手动修改

实际工程：`D:/新建文件夹4/工创赛/代码/stm32/3.stm32f407工程/gongxun-f407.uvprojx`。退出Debug后只改USER/chassis_auto_test.h/c，保持原编码；不改main或配置，不下载。

## 1. 补齐上一项初始化

只读实际源确认新会话ChassisAuto_Begin已清除s_finish_requested/s_finish_tick，但ChassisAuto_Init内尚缺这两行。在ChassisAuto_Init中，紧接`s_vm_requested = s_used = s_mode = 0U;`后添加：

```c
    s_finish_requested = 0U;
    s_finish_tick = 0U;
```

保留Begin内已有的两行，不重复添加。静态初值和新会话清零使现有上电流程有保护，但Init仍应完整恢复本模块的新状态。

## 2. 头文件声明

在chassis_auto_test.h的ChassisAuto_RequestStop声明后添加：

```c
/* 单次旋转：1接受，0拒绝；不等于目标角已到位。 */
uint32_t ChassisAuto_BeginHeading(uint32_t dry_run, int32_t command_rpm);
```

## 3. 文件末尾追加实现

在chassis_auto_test.c现有全部函数结束后追加：

```c
/* 单次旋转入口：只选旋转段，沿用原四轮准备和停车保护。 */
uint32_t ChassisAuto_BeginHeading(uint32_t dry_run, int32_t command_rpm)
{
    ChassisMotorCommand commands[CHASSIS_WHEEL_COUNT] = {0};
    uint32_t i, accepted;
    /* 零速不启动；应由上层直接判断已到位或继续保持停止。 */
    if ((dry_run > 1U) || (command_rpm == 0) ||
        (Chassis_HeadingCommands(command_rpm, commands) != 1U)) return 0U;

    taskENTER_CRITICAL();
    /* 拒绝重复启动，不覆盖正在运行的序列结果。 */
    if (g_chassis_auto_active != 0U)
    {
        taskEXIT_CRITICAL();
        return 0U;
    }
    accepted = ChassisAuto_Begin(dry_run);
    if (accepted == 1U)
    {
        g_chassis_auto_segment = 2U;
        for (i = 0U; i < CHASSIS_WHEEL_COUNT; i++)
            s_commands[2][i] = commands[i];
    }
    taskEXIT_CRITICAL();
    return accepted;
}
```

该函数通过原ChassisAuto_Begin保留VM请求、健康门禁、单次使用、四轮失能/使能和零速反馈准备。临界区内选定段2并复制完整四轮目标，CAN任务不能在复制前启动旧段0。原生FreeRTOS任务临界区允许嵌套；临界区不执行CAN交易，只操作内存。

这里的command_rpm仍是每轮逻辑旋转目标，正逆时针、负顺时针。直接使用已验证的Chassis_HeadingCommands，不读取旧APP_HWT_TURN_CLOCKWISE来翻转新目标。输入0、超范围或非法dry_run拒绝且不启动；运行中的再次启动被拒绝，原会话结果不被覆盖。以后调用方必须处理返回0，不能当已进入运行。

本步只建立固定初始轮速的单次旋转入口，后续仍要补运行中的动态更新、源指令时效、反向切换、HWT故障/取消和到位停车的会话控制。当前保留原10秒运行上界和准备/停止/总时限，未放宽。上界触发后的执行序列完成不表示达到目标角，也不等于20秒heading_controller超时；未来上层必须结合两者结果处理，不能仅凭auto_result1判定目标成功。

ChassisAuto_BeginHeading中的dry_run=1沿用原实现：跳过速度/停止写帧，但仍有真实使能/失能和状态/速度读取。因此不得把它描述成完全无CAN或无使能；当前不加任何新调用，保持静态模式。以后只有满足物理调试前置条件才调用该接口。

## 4. 本轮验收

保存两个文件，完整Rebuild，发送日志。APP_HWT_STATIC_TEST=1、APP_HWT_TURN_TEST=0保持，main不新增调用，本轮无需下载。助手未修改真实源码/工程，未发送硬件命令。

## 隔离验证与范围

当前实际正常停车接口构建已确认38028/576/1400/25408、0错0警告、5秒。隔离候选与脚本：tmp/heading_begin_20261021/chassis_auto_test_candidate.c、tmp/check_heading_begin_20261021.py。GCC C99 Wall/Wextra/Werror实际候选状态机mock测试通过：±5/±3/±1单次旋转方向和幅值、只发一段四台速度/停止、正常提前结束、活动会话拒绝重复启动、VM/零速/范围/模式门禁、演练语义及原10秒上界；复跑停车截止、停止应答失败、非零反馈/过期故障与原三段流程回归。实际工程头文件/宏下ARMCC V5候选编译退出0，无诊断。mock不代表实际并发、CAN或机械停止通过；本次用户手改后的完整构建尚待日志。

R已检查RUN-01/PWR-01。E已检查LOC-03/05、RUN-06、MOT-02/04。官方Q保持待技术答疑确认。动态执行整合、物理转角闭环、停车与X/Y尚未闭环。
