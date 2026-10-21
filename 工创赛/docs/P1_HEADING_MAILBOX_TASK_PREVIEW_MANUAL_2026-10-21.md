# 航向指令跨任务交接：用户手动接入静态观察

## 当前实际工程（用户截图与只读文件核对）

工程：`D:/新建文件夹4/工创赛/代码/stm32/3.stm32f407工程/gongxun-f407.uvprojx`。

此前助手读取的是D:/Codex/工创赛/3.stm32f407工程副本，不能以副本缺少代码否定用户已完成修改。用户实际USER/chassis_auto_test.h/c均已存在Publish/Read；实际本地构建日志与用户一致：Code37688/RO564/RW1372/ZI25404、0错0警告、5秒。后续实际固件只读核对使用本节工程；用户手动编辑，不自动同步覆盖任何副本。

## 本轮只修改实际工程USER/main.c

保持APP_HWT_STATIC_TEST=1、APP_HWT_TURN_TEST=0。以下是数据预览接入，没有新增CAN发送或VM控制。退出Debug后修改，保留原文件编码。

### 1. 在App_CanTask定义前添加观察变量

找到`static void App_CanTask(void *argument)`，在其前面添加：

```c
#if APP_HWT_STATIC_TEST && !APP_HWT_TURN_TEST
/* 以下仅显示任务间数据，不作为运动允许信号。 */
volatile uint32_t g_heading_bus_valid = 0U;
volatile int32_t g_heading_bus_rpm = 0;
volatile uint32_t g_heading_bus_stop = 1U;
volatile uint32_t g_heading_bus_sequence = 0U;
volatile uint32_t g_heading_publish_result = 0U;
#endif
```

### 2. CAN任务每轮读取快照

在App_CanTask的while循环中，紧接`g_can_task_count++;`后、现有`if (g_can_test_result == 1U)`前添加：

```c
#if APP_HWT_STATIC_TEST && !APP_HWT_TURN_TEST
        {
            ChassisHeadingRequest heading_request = {0, 1U, 0U};
            uint32_t valid = ChassisAuto_HeadingRead(&heading_request);
            /* 只观察：不会把这里的轮速发给电机。 */
            g_heading_bus_rpm = heading_request.command_rpm;
            g_heading_bus_stop = heading_request.stop_required;
            g_heading_bus_sequence = heading_request.sequence;
            g_heading_bus_valid = valid;
        }
#endif
```

保留原有CAN任务的全部分支，不替换App_CanTask，不把上述代码放在函数外。仅观察字段不是供其他任务读取的原子协议，未来执行层必须使用Read返回的局部快照。

### 3. TaskA每轮发布计算结果

在App_HeadingLiveStep函数内，找到末尾已有的：

```c
    for (i = 0U; i < CHASSIS_WHEEL_COUNT; i++)
    {
        g_heading_live_motor[i] = commands[i];
    }
```

在此for循环结束后、App_HeadingLiveStep函数最后的大括号前添加：

```c
    /* TaskA发布完整的一轮计算结果；CAN任务只读取预览。 */
    g_heading_publish_result = ChassisAuto_HeadingPublish(
        output.command_rpm, output.stop_required);
```

不放到一次性的App_HeadingDryRun里，也不放在while之外。数据接口成功只表示参数合法且尚未到龄，不代表HWT有效、会话许可或电机已停；控制器故障/取消等终态仍以state/cause判断。

## 当前验收动作

保存main.c，完整Rebuild，发送完整日志。本轮先不下载。新函数将真正被调用，不能用旧未调用版本的程序大小判断本次结果。助手未修改实际main、源码或工程。

构建核对后的下一轮才下载：VM关闭、HWT正常时发+45度，观察发布结果1、bus_valid1、bus_rpm5、bus_stop0及sequence随运行更新；取消后应读到rpm0/stop1。序号可能跳过中间发布，这是最新值交接而非事件队列。暂停恢复可能影响时效，快照0不能直接归因为物理传感器故障。不要求重复此前四轮编码用例。

## 规则、依据与范围

已检查R RUN-01/PWR-01，E LOC-03/05、RUN-06、MOT-04。官方Q保持待技术答疑确认。依据是实际文件、用户Rebuild与已完成的四轮静态编码截图。本轮代码尚待用户手改及完整构建；未做新增整工程编译/烧录或真实跨任务验收。旧接口隔离检查不代替本步实机结果。实际执行层、动态四轮更新、物理停车与X/Y仍待闭环。
