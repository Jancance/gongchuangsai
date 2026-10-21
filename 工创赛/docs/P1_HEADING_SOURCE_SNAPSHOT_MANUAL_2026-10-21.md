# 真实航向源快照：手动接入静态观察

接续依据：当前下一步记录中，会话模块已完整Rebuild，38380/576/1424/25408、0错误0警告。2026-10-21只读实际工程确认heading_session.c/h存在，main尚未调用会话；静态模式1、旧限时转向模式0。实际工程为`D:/新建文件夹4/工创赛/代码/stm32/3.stm32f407工程/gongxun-f407.uvprojx`，不得拿项目内旧副本覆盖。助手不修改实际固件或工程。

本步只准备会话所需的真实姿态快照；不调用BeginHeading/UpdateHeading/RequestStop，不改VM、Safety或现有预览行为。现有预览500ms窗口继续仅用于原静态预览；新快照以100ms为候选拒绝边界，尚未批准为正式行车指标。

## 1. main.c增加快照代码

退出Debug。在实际工程USER/main.c找到`static void App_CanTask(void *argument)`的函数定义，在这行之前粘贴[完整快照代码](code_samples/heading_source_snapshot_main.c.txt)。这段应位于已有g_heading_bus等观察变量之后、App_CanTask之前，不要粘到函数体内。保持原文件编码。

代码独立包在现有静态模式条件中；不增加新模式开关或Keil文件。已核对USER/FreeRTOSConfig.h为1000Hz，所以当前HWT tick与毫秒一致；以后更改频率会触发编译错误，须统一修改时间换算，不能直接删检查。

## 2. TaskA发布完整快照

在App_TestTaskA循环中找到：

```c
        HWT101_Update();
        #if APP_HWT_STATIC_TEST && !APP_HWT_TURN_TEST
        App_HeadingLiveStep();
        #endif
```

改为：

```c
        HWT101_Update();
        #if APP_HWT_STATIC_TEST && !APP_HWT_TURN_TEST
        App_HeadingSourcePublish();
        App_HeadingLiveStep();
        #endif
```

TaskA是唯一发布者，HWT101_Update解析真实帧后发布。sequence是完整快照序号，不是HWT帧计数；即使重复读到同一真实帧，真实angle_ms/rate_ms也不会刷新。无新帧持续到100ms时新快照会失效。

## 3. CAN任务只观察

在App_CanTask循环中找到原静态预览块开头：

```c
        #if APP_HWT_STATIC_TEST && !APP_HWT_TURN_TEST
```

紧接此行、原`{ ChassisHeadingRequest ... }`块之前添加：

```c
        App_HeadingSourceObserve();
```

其余原预览和CAN调度保留。临界区复制完整快照；消费者按读取时刻重新检查发布年龄和两类真实帧年龄。Observe不递增sequence；未发布、任一年龄达到100ms、数据非法或生产时校准/版本请求冲突均不认为有效。过期时保留角度数值供排查，valid=0的角度不能供后续会话使用。

## 4. 本轮验收

保存后完整Rebuild，回传完整日志。APP_HWT_STATIC_TEST保持1、APP_HWT_TURN_TEST保持0；本步先不下载。

后续构建核对后，才安排VM外部关闭下的软件观察：g_heading_source_valid、sequence、age_ms、angle_age_ms、rate_age_ms、yaw_deg、rate_dps，连同VM请求和两fault一起记录。不要修改这些输出伪造有效，也不重复做已经通过的旧控制器用例。

此代码只建立静态生产/观察路径：尚未把输入传给HeadingSession，也未绑定backend一致快照、platform_ok、动作接受报告或独立Safety源失效监督。生产后出现的配置请求/故障还需在将来的会话入口和Safety再次检查；不能把本快照作为跨任务供电许可。临界区主机mock不能证明板上并发/中断优先级配置。100ms严格边界可能暴露实际帧率或调度不足，应先取证，不能静默放宽。

本机软件检查：同一代码样本经GCC C99 Wall/Wextra/Werror编译运行，验证未发布、99/100ms严格边界、发布时已过期、重复发布不刷新真实帧时间、非法角度/角速度、配置请求互斥、tick/sequence回绕；非1000Hz配置确实编译拒绝。实际main的三个插入位置已匹配，并生成隔离候选；实际main/app_config/session文件SHA256前后相同。没有运行ARMCC或完整Keil构建，没有板上并发/时效证据。检查脚本为tmp/check_heading_source_20261021.py，记录为evidence/p1-foundation/2026-10-21/heading_source_snapshot_preparation.json。

规则检查：R RUN-01/PWR-01；E LOC-03/05、RUN-06、MOT-02/04。官方Q保持待技术答疑确认；真实会话整合、运动/停车、定位X/Y未闭环。证据为本轮实际源只读核对和独立软件检查；用户手改后的完整构建与板上时效尚待验证。
