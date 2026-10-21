# 板内USART2只读回传速率：用户手动修改

> 最新：实际读取c代码与已测候选一致，用户完整构建39292/580/1476/25444、0错0警告/3秒，静态配置及main已核。头文件3行RRATE extern尚未加入，当前仅Watch/本文件读取不依赖它，不阻塞本次测试，后续跨模块接入前补齐。无需为此再构建。下一项VM外部开关关闭，下载复位后F5连续Run约5秒；version_result1/state0、VM/fault0时Run中rrate_request1一次，再Run约2秒Stop，读取下文列出的RRATE/版本/VM及故障字段、展开reply11字节。只读不配置，不触发任何运动或校准。证据hwt_rrate_read_actual_rebuild_pass.json。

实际工程：`D:/新建文件夹4/工创赛/代码/stm32/3.stm32f407工程/gongxun-f407.uvprojx`。HWT101焊在板上，不拆焊、不飞线、不连接USB-TTL，继续原USART2 PD5发送/PD6接收。助手不改实际固件或工程。

本轮只增加手动读取RRATE的入口，发送`FF AA 27 03 00`；不写50Hz、不解锁、不保存、不归零、不校准、不改波特率。RRATE地址0x03及50Hz编码由厂家通用标准协议提供；在本机使用已验证的0x27读取方式作只读验证，不据此提前保证本机所有配置命令/版本兼容。参考：https://wit-motion.gitbook.io/witmotion-sdk/wit-standard-protocol/wit-standard-communication-protocol 。

## 1. 切到既有静态模式

关闭VM外部开关，退出Debug。在实际USER/app_config.h将四项设为：

```c
#define APP_CHASSIS_KEY_TEST_ENABLE 1U
#define APP_HWT_STATIC_TEST 1U
#define APP_HWT_TURN_TEST 0U
#define APP_HEADING_SESSION_TEST 0U
```

其余值/宏保留。STATIC模式优先于KEY分支，按键只计数，不启动三段动作；Safety每轮维持VM请求0。本轮不修改main、不触发旧预览或任何CAN/机构请求。

## 2. hwt101.h添加声明

在实际USER/hwt101.h最后的`#endif`之前粘贴[完整声明代码](code_samples/hwt_rrate_read_header.c.txt)，保留原声明和最后#endif。

## 3. hwt101.c增加变量和只读命令

找到实际USER/hwt101.c中的：

```c
static uint32_t s_read_a7 = 0U;
```

在这行之前粘贴[完整变量代码](code_samples/hwt_rrate_read_globals.c.txt)。原s_read_a7及其他变量全部保留，不改现有版本、A7或校准命令数组。

## 4. hwt101.c替换读取服务函数

找到完整的：

```c
static void Hwt_VersionUpdate(uint32_t now)
```

从这行开始，到这个函数匹配的结束右花括号为止，整体替换为[完整读取函数](code_samples/hwt_rrate_read_update.c.txt)。只替换这个函数；后面的Hwt_AngleDelta、所有校准函数、HWT101_Init/Update及中断代码保持。

新函数沿用原TXE连续发5字节、发送/TC各200ms和回复1000ms期限；新增RRATE只在静态、VM请求0、无RTOS/Safety故障及无校准请求时受理。RRATE进行中条件变坏也会结束本次读取，保持原运动保护。

版本/ A7优先，冲突的RRATE请求返回9并清除，不排队。版本必须已成功，且距最近5F回复至少1500ms才受理RRATE；本次候选回复要求计数推进、时间在事务窗口。5F不回显地址，静默隔离只能减少旧回复混入，不能成为地址回显的替代；必须保留原帧并结合当前10Hz实测判断，不能只看result1就宣布RRATE身份无歧义。

## 5. 本轮只保存、完整Rebuild

把完整构建日志发来，先不下载、不设置请求。更改涉及app_config.h、hwt101.h和hwt101.c三个文件，不增Keil文件或任务。保留现有main、CAN20/30ms和全部100ms安全期限，不覆盖此前已完成模块。

本轮切回静态后g_heading_exec等新模式变量可能因条件编译不存在，属预期；不用为此改main或重新添加变量。真实读取操作在用户构建核对后安排，不直接从Watch调用函数。

## 后续观察字段和解释（本轮先不操作）

```text
g_hwt_rrate_request
g_hwt_rrate_result
g_hwt_rrate_raw
g_hwt_rrate_reply_count
g_hwt_rrate_reply
g_hwt_version_result
g_hwt_version_state
g_vm_test_request
g_rtos_fault
g_safety_fault
```

request默认0，未来手动1读取一次；result0未完成、1收到校验有效候选5F、2发送/TC超时、3回复超时、4UART错误或读取中失去静态条件、9条件不满足/事务冲突/静默隔离未满。raw是前两个payload字节小端拼成16位；按通用协议，0x0006与10Hz相符。并列保存reply原帧及计数，不用raw覆盖原version_raw或a7_raw。result1不是参数写入/50Hz生效，不重跑自动校准/旧运动试验。

## 软件检查与范围

相同函数样本通过GCC C99 Wall/Wextra/Werror编译/模拟，覆盖只读03包、候选有效回复/校验/原版本与A7信息隔离、发送与回复截止、UART错误、VM改变、校准冲突、1500ms隔离边界、并发版本/A7回归、旧时间回复拒绝和忙时重复请求不排队。实际hwt101.c隔离候选及实际main的静态配置候选均通过ARMCC Cortex-M4.fp编译，无诊断；没有链接整套候选固件或执行真实中断/串口。

证据evidence/p1-foundation/2026-10-21/hwt_rrate_read_preparation.json，隔离文件tmp/hwt_rrate_read_20261021/。本轮只准备说明/样本/隔离代码，实际固件未改。用户手改后的完整构建、板上RRATE身份/值、50Hz设置/读回/持久化和帧间隔验证均待完成。

R核对RUN-01/PWR-01；E核对LOC-03/05/07、RUN-06、MOT-02/04。参数读取和50Hz候选均非竞赛正式规则。官方Q继续待技术答疑确认；实物VM/转角/停车与X/Y未闭环。
