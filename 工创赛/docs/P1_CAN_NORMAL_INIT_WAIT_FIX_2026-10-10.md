# CAN正常初始化退出等待补丁（2026-10-10）

依据用户失败后的寄存器截图：MCR.INRQ=0、MSR.INAK=0、PD0 RX输入=1，而g_can_version_result仍3。当前代码读版本前CAN_Init失败就立即return3，滤波器及发送未执行；实际SPL退出初始化用固定循环次数等待，硬件可在软件返回后才完成同步。0/0/1说明观察时已退出初始化、RX高，不足以确定最初超时的全部物理原因，不据此更改电机ID/校验/端接。

最小补丁位于用户原工程SYSTEM/CAN/can.c，用户自行修改，其他500k/GPIO/过滤器/版本命令保持。当前 helper 只在FreeRTOS任务里调用，允许用原生节拍有限等待；不在调度器启动前调用。

## 1. 加一个局部变量

在CAN1_InitMode()开头，原CAN_FilterInitTypeDef filter之后添加：

```c
TickType_t init_start_tick;
```

## 2. 替换原初始化失败分支

原来的CAN_Init失败立即return3这一整段替换为：

```c
if (CAN_Init(CAN1, &can) != CAN_InitStatus_Success)
{
    /* 进入初始化未成功，不接受尚未完成配置的状态 */
    if ((CAN1->MCR & CAN_MCR_INRQ) != 0U)
    {
        return 3U;
    }

    /* 已请求退出：继续等硬件同步，最多100毫秒 */
    init_start_tick = xTaskGetTickCount();
    while ((CAN1->MSR & CAN_MSR_INAK) != 0U)
    {
        if ((TickType_t)(xTaskGetTickCount() - init_start_tick) >=
            pdMS_TO_TICKS(100U))
        {
            return 3U;
        }
        vTaskDelay(pdMS_TO_TICKS(1U));
    }
}
```

保留此段之后的过滤器初始化，不放进if内部或提前return1。标准库进入初始化失败保留失败，不跳过配置；已经配置并请求退出时才允许补等INAK清零。每毫秒让出CPU，100ms超时仍返回3，不强制改CAN状态位，也不新增自动重试。

## 3. 用户验证

保存Rebuild、下载。原请求复位后默认为0：先g_vm_test_request=1/F5，等电机屏幕完成启动，再g_can_version_request=1/F5，约1秒后Stop看g_can_version_result、g_can_version_reply、g_can_version_esr。原始版本回复ID0x100/DLC6/1F+FW16+HW16+6B仍为成功条件；进展到4/5/6不当成成功，按实际阶段排查。测试只读，无运动命令，测试结束请求0/F5撤销VM。

R检查PWR-01；E检查LOC-03/RUN-06/MOT-04，官方Q不变。此为有限初始化等待的最小实现修正，原源/PCB由助手保持不变；寄存器事实与迟到同步推断分别记录，实际改后初始化/版本结果待用户验证，完整电气与运动可靠性尚未验。
