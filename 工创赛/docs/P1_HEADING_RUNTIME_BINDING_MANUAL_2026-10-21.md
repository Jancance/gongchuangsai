# 目标航向会话接入真实任务：用户手动修改，先构建

> 最新：实际main已完整匹配已测候选，用户整工程35744/564/1444/25452、0错0警告、3秒；SESSION1/STATIC0/TURN0/KEY0和后端s_mode已核对。下方所有修改/构建要求已完成，不再重复。当前下一项仅验证新模式默认不启动：VM外部开关关闭、车静止，下载后连续Run约3秒再Stop，不按PC13或设置请求；记录exec_phase/request/trip/state/rpm、VM请求及两fault（均预期0）、source_valid（预期1）、两帧龄期（各<100ms）、g_can_task_stack_words及g_heap_min_bytes。此为默认状态观察，不能代替物理VM、连续时效、目标角运动或停车验收。证据heading_runtime_actual_rebuild_pass.json。

> 最新：两处外层条件已改对。用户最新Rebuild的两处HEADING_SOURCE_FRESH_MS未定义来自源快照块中遗漏的原四行。应在源快照外层if之后、typedef之前恢复1kHz检查及`#define HEADING_SOURCE_FRESH_MS 100U`，不要再更改外层条件。恢复后隔离ARMCC编译无诊断；实际固件未改。详见第3节完整代码块及heading_runtime_missing_define_review.json，用户补回后完整Rebuild、本轮不下载。

> 最新粘贴审阅：两份附件内容相同，且与实际保存main一致。除两处相邻外层条件外，其余代码与已测候选去注释/空白后相同。当前main第718行、g_heading_bus_valid等旧邮箱变量之前应为`#if APP_HWT_STATIC_TEST && !APP_HWT_TURN_TEST`；第726行、含HEADING_SOURCE_FRESH_MS的源快照段最外层应为`#if APP_HEADING_SESSION_TEST || (APP_HWT_STATIC_TEST && !APP_HWT_TURN_TEST)`。用户仅修正这两行后完整Rebuild，其余保持，不下载。已隔离纠正并用实际配置ARMCC编译，无诊断；真实文件未改。证据heading_runtime_paste_review.json。

当前实际工程：`D:/新建文件夹4/工创赛/代码/stm32/3.stm32f407工程/gongxun-f407.uvprojx`。源快照静态截图已记录，ReadStatus中的dry_run已改为s_mode，用户完整Rebuild为38908/576/1456/25432、0错误0警告、2秒。本项不重复。

本轮把已有会话、真实HWT源、执行器反馈与Safety接起来，采用独立开发模式；默认请求为0，不自动上VM。用户手改实际源码，助手只准备说明/样本/隔离候选。本轮只完整Rebuild，不下载、不按启动键、不设置Watch请求。后续先检查新固件默认关闭，再另行安排完整装机载物的正负5度低速测试。

## 1. app_config.h选择独立模式

退出Debug。在实际USER/app_config.h设置以下四项；新项只定义一次：

```c
#define APP_CHASSIS_KEY_TEST_ENABLE 0U
#define APP_HWT_STATIC_TEST 0U
#define APP_HWT_TURN_TEST 0U
#define APP_HEADING_SESSION_TEST 1U
```

保留其他参数，包括APP_HWT_TURN_CLOCKWISE。旧方向开关不作用于本次BeginHeading的有符号轮速。

在文件最后的`#endif`之前新增：

```c
#if APP_HEADING_SESSION_TEST && (APP_HWT_STATIC_TEST || APP_HWT_TURN_TEST || APP_CHASSIS_KEY_TEST_ENABLE)
#error Heading_session_mode_must_be_exclusive
#endif
```

该模式是工程调试入口，不是正式比赛启动模式。回退旧静态模式时，设置SESSION=0、STATIC=1、TURN=0、KEY=1；不要同时启用多种模式。

## 2. main.c增加头文件，并处理旧扫码条件

在`#include "heading_controller.h"`之后增加：

```c
#include "heading_session.h"
```

main.c中两处完全相同的：

```c
#if !APP_CHASSIS_KEY_TEST_ENABLE
```

均改为：

```c
#if !APP_CHASSIS_KEY_TEST_ENABLE && !APP_HEADING_SESSION_TEST
```

两处分别在s_start_gate_revision定义前及App_StartGateUpdate内的赋值前。新模式不走扫码启动，因此不保留仅赋值未使用的旧变量；正式扫码模式原行为保留。

## 3. 扩展真实源快照的编译范围

找到先前增加的源快照代码：它包含`#define HEADING_SOURCE_FRESH_MS 100U`、AppHeadingSource结构、App_HeadingSourcePublish和App_HeadingSourceObserve。

仅把**这整段最外层**原来的：

```c
#if APP_HWT_STATIC_TEST && !APP_HWT_TURN_TEST
```

改为：

```c
#if APP_HEADING_SESSION_TEST || (APP_HWT_STATIC_TEST && !APP_HWT_TURN_TEST)
```

不要全局替换其他同名条件；原g_heading_bus等观察变量和App_HeadingLiveStep仍只属于旧静态预览。

这两个相邻代码块的开头最终应分别为：

```c
/* 旧邮箱观察变量：保持旧条件，不扩展。 */
#if APP_HWT_STATIC_TEST && !APP_HWT_TURN_TEST
/* 原注释保留 */
volatile uint32_t g_heading_bus_valid = 0U;
/* 其余原邮箱变量与原#endif均保留 */
```

```c
/* 真正的源快照块：扩展这一处最外层条件。 */
#if APP_HEADING_SESSION_TEST || (APP_HWT_STATIC_TEST && !APP_HWT_TURN_TEST)
/* 原注释保留 */
#if configTICK_RATE_HZ != 1000U
#error Heading_source_requires_1000Hz_tick
#endif
#define HEADING_SOURCE_FRESH_MS 100U
/* 后续原源快照代码全部保留 */
```

以上仅用于辨认位置，不要把省略说明替换成代码，也不要删掉任何原变量/函数。

## 4. 在源快照之后加入会话运行代码

找到`static void App_CanTask(void *argument)`函数定义。在它之前、源快照段的`#endif`之后，粘贴[完整会话接入代码](code_samples/heading_runtime_main.c.txt)。保留已有函数和变量。代码必须在App_CanTask之前，不能贴进任何函数体内。

这段代码包含独立的g_heading_exec观察/请求变量、CAN任务唯一持有的会话对象、状态机、动作接受报告及Safety检查。不要把它贴进heading_session.c；它需要main.c现有的私有调试/源快照状态。

## 5. CAN任务接入及旧交易入口互斥

### 5.1 循环入口

在App_CanTask循环的`g_can_task_count++;`之后添加：

```c
#if APP_HEADING_SESSION_TEST
        App_HeadingRuntimeStep();
#endif
```

### 5.2 包住原手动交易分支

在`if (g_can_test_result == 1U)`内部，找到原第一条：

```c
        if (g_chassis_auto_active == 1U)
```

紧接它之前插入：

```c
#if APP_HEADING_SESSION_TEST
        if (g_chassis_auto_active == 0U) App_CanPollStep();
#endif
        if (APP_HEADING_SESSION_TEST == 0U)
        {
```

向下找到这组原交易分支之后、Safety报告之前的：

```c
        App_CanPollRefresh();
```

紧接此行之前添加一个右花括号：

```c
        } /* 原调试交易只在非目标航向模式执行 */
```

即：整个原三段/单轮/手动CAN交易if-else链被包在`if (APP_HEADING_SESSION_TEST == 0U)`内；App_CanPollRefresh、App_CanFeedbackRefresh以及Safety报告仍在外面。新模式下，活动会话只由Runtime调用原ChassisAuto_CanStep一次；闲置/等上电阶段继续原只读状态轮询。新模式的手动CAN请求被拒绝，不能绕过互斥入口。

### 5.3 CAN循环周期

将App_CanTask末尾原来的：

```c
        vTaskDelay(pdMS_TO_TICKS(50U));
```

改为：

```c
        vTaskDelay(pdMS_TO_TICKS(APP_HEADING_SESSION_TEST ? 10U : 50U));
```

原执行器一次CAN等待仍有自己的上界，新模式不在活动执行器之后叠加第二次轮询交易。10ms是软件候选周期，不代表实测调度时延或最终行车时效已通过。

## 6. TaskA继续生产真实源，但不运行旧控制器预览

找到HWT101_Update之后原来的：

```c
        #if APP_HWT_STATIC_TEST && !APP_HWT_TURN_TEST
        App_HeadingSourcePublish();
        App_HeadingLiveStep();
        #endif
```

替换为：

```c
        #if APP_HEADING_SESSION_TEST || (APP_HWT_STATIC_TEST && !APP_HWT_TURN_TEST)
        App_HeadingSourcePublish();
        #endif
        #if APP_HWT_STATIC_TEST && !APP_HWT_TURN_TEST
        App_HeadingLiveStep();
        #endif
```

HWT101_Update调用保留在这些行之前。新模式由TaskA只发布带真实帧时间的源快照，CAN任务负责唯一会话计算与执行；不再用旧纯RPM邮箱驱动新模式。

## 7. Key任务仅提供取消

找到App_KeyTask循环中原来的：

```c
#if APP_HWT_STATIC_TEST
        if (Key_GetEvent() != 0U) g_start_key_count++;
```

替换为：

```c
#if APP_HEADING_SESSION_TEST
        if (Key_GetEvent() != 0U)
        {
            g_start_key_count++;
            /* PC13只取消，不用按键启动本次开发试验。 */
            taskENTER_CRITICAL();
            if ((g_heading_exec_phase == HR_WAIT) || (g_heading_exec_phase == HR_RUN))
                g_heading_exec_request = 2U;
            taskEXIT_CRITICAL();
        }
#elif APP_HWT_STATIC_TEST
        if (Key_GetEvent() != 0U) g_start_key_count++;
```

后面的原elif/else/endif全部保留。PC13不启动本模式；运行中的按键事件只提交取消请求。Key任务不直接修改会话对象。

## 8. 独立Safety监督

在App_SafetyTask中找到原来的：

```c
#if APP_HWT_STATIC_TEST
        g_vm_test_request = 0U;
```

将这两行替换为以下内容，原行末注释可保留：

```c
#if APP_HEADING_SESSION_TEST
        App_HeadingRuntimeSafety();
#elif APP_HWT_STATIC_TEST
        g_vm_test_request = 0U;
```

原`Safety_Update(g_vm_test_request, g_can_test_result);`继续在这些条件之后执行。RuntimeSafety在Safety任务内检查真实源帧龄期、配置请求互斥、平台健康、归属、等待期限和总期限；它只锁存首次trip、Abort和撤销VM，不操作会话对象、不等待CAN。原Safety_Update还独立执行100ms指令提交监督及原电机反馈/截止检查。

## 9. 本轮只保存、完整Rebuild

把完整日志发来；先不下载、不设置request。预计新代码会进入链接，大小通常增加，但验收依据是实际源码和构建日志，不以某个固定Code数值判断正确。

后续下载前先核对新模式的默认VM0/请求0、无自动动作、故障/取消锁存和观察变量。正式运动许可、±5度真实误差和物理停止仍未放行。

## 接入行为和限制

| 字段 | 含义 |
|---|---|
| g_heading_exec_phase | 0闲置、1等平台、2会话运行/停车、3完成、4失败、5取消 |
| g_heading_exec_request | 上电默认0；未来受控试验才使用1开始/2取消；本轮不设置 |
| g_heading_exec_relative_deg | 默认+5度；首轮只接受+5/-5，运行目标在实际接受时固定 |
| g_heading_exec_trip | 首次原因锁存：1取消、2源/配置、3平台/互斥、4等待超时、5会话归属、6控制/提交失败、7总期限 |
| g_heading_exec_state/cause | 原会话模块状态/原因；开始前被拒绝时可能仍为HS_IDLE，应结合phase/trip判断 |
| g_heading_exec_backend_session | 本次BeginHeading接受后取得的实际执行器会话号 |

上电不调用循环Init恢复状态，一次成功或失败后保持终态，再写request1不排队。等待平台最长5秒；执行器准备和运行共用会话9秒候选期限，到位后最多2.5秒停车确认，独立监督还限制接受后的总上界。当前2度/700ms稳定阈值和5/3/1RPM属于既有工程候选；对±5度通常先输出1RPM，不声称达到5度实测精度。

BEGIN/UPDATE/STOP均报告实际接口接受结果。SETTLING只UPDATE零速；连续稳定才RequestStop。完成检查本次真实执行模式、会话号、旋转段2、STOP已接受、result1、非活动及stop_mask0F；正常结束引起guard下降不误判故障。取消或故障即Abort并撤VM，不冒充正常停车。异常切VM可能比完整停车反馈更早，实际减速/停稳/供电及负载行为必须单独实测。源过期100ms是判定边界，Safety20ms调度加执行时间仍有响应延迟，不保证恰好100ms切电。

当前同会话反向纠偏仍拒绝；运动中超调需要反向时故障退出，不删保护制造到位。此开发入口依赖Watch发起，明确与RUN-01正式比赛自主启动隔离。禁止同时发校准/配置或其他手动CAN请求。

## 检查证据、未闭环项和需求

隔离候选main在ARMCC Cortex-M4.fp下分别以新模式和旧静态模式编译，无诊断；实际main/config/backend/session源码SHA256前后未变。运行样本链接实际heading_controller.c/heading_session.c进行主机模拟，覆盖默认无动作、正负5度、同序号不重复Begin/Update、临时零速/700ms连续稳定、正常停止/正确归属及终态保持、源过期、取消、错会话/演练模式、配置请求冲突、BEGIN/UPDATE/STOP拒绝、5秒等待/9秒会话截止、互斥、非法角度和时间回绕。初版主机用例一次跳过700ms导致按设计触发100ms源序号过期，改为10ms持续发布/计算后通过，未放宽生产代码期限。

这些检查使用mock执行器/CAN，不代表实际RTOS并发、CAN等待、物理动作或Safety时延通过；未完整链接新固件、未下载。证据：heading_runtime_compile_preparation.json、heading_runtime_host_preparation.json；复现脚本tmp/prepare_heading_runtime_20261021.py和tmp/check_heading_runtime_20261021.py。后续用户真实手改/Rebuild、新模式默认VM0、实机调度/源失效、完整装机载物±5度/到位误差/真实停车及X/Y仍待验证。

R核对RUN-01/PWR-01；E核对LOC-03/05/07、RUN-06、MOT-02/04。本步骤未修改R或新增官方规则；官方Q保持待技术答疑确认，FIELD-05/OBS-03等不因软件构建通过而关闭。
