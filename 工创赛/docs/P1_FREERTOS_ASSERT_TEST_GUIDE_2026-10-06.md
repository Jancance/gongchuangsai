# F407断言故障钩子：手动临时注入

日期2026-10-06。已通过当前两个测试任务的调度、栈堆余量及名义1ms配置检查。此次仅验证断言诊断入口，未验证硬件急停、输出关断或任务饥饿监督；保持当前无执行器动作工程。检查LOC-03、RUN-06（E），R/官方Q不变。

最新结果：临时注入已通过（开关1、fault3、assert_line0x21=33，进入App_RtosStop）；源码清理/构建0错误0警告及恢复运行也已通过（新截图Idle/PSP，A521/B105/tick52000，fault/line/returned均0）。本项闭环，进入PE7状态灯验证，不再重复注入。Watch遗留cannot evaluate变量行可删除，不影响运行。

实际源文件由用户手动修改：D:/新建文件夹4/工创赛/代码/stm32/3.stm32f407工程/USER/main.c。不改FreeRTOS内核或port。

1. 退出调试。在已有测试全局变量后加入：

```c
/* 临时故障测试：正常为0，调试时改为1触发断言 */
volatile uint32_t g_test_force_assert = 0U;
```

2. App_TestTaskA的while(1)内部，在g_test_count_a++之前加入：

```c
/* 验证断言记录与停止路径，测试完成后删除此行 */
configASSERT(g_test_force_assert == 0U);
```

3. 保存main，Rebuild零错误后下载并重新进入调试、F5正常运行。Stop暂停后Watch新增g_test_force_assert、g_rtos_assert_line。把g_test_force_assert的Value从0改为1；不要修改故障码，不在Watch直接调用函数。

4. F5运行约1秒后Stop。期望g_rtos_fault=3、g_rtos_assert_line对应main中新断言行；PC在freertos_hooks.c的App_RtosStop诊断循环（优化时可看反汇编/Call Stack），此后任务计数和tick停止推进。截Watch和当前PC提交；若不符合提供实际结果，不删除原configASSERT来绕过。

5. 故障循环不返回，只把开关改回0后F5不能恢复。复位后C初始化使开关及故障码归0，再F5才能正常运行。测试结果记录后删除本次临时全局和任务中的断言两段，重新编译下载；保留FreeRTOSConfig.h里的configASSERT和原hooks。

依据为实际main、FreeRTOSConfig.h、freertos_hooks.c/h、用户板上注入截图及清理恢复截图，见evidence/p1-foundation/2026-10-06/rtos_assert_injection_pass.json/png及rtos_assert_cleanup_running_pass.json/png。助手未自动修改/编译用户固件。本项结果不外推为malloc失败/栈溢出检测或整车安全验收。
