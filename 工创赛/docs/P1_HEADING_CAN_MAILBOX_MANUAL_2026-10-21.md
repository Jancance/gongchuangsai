# 目标航向CAN接入第一步：指令交接接口

本步承接10月21日用户截图：+45度四轮direction全1/speed5、取消后speed全0、-45度direction全0/speed5均已确认，encode_result1。真实工程仍由用户手动修改。

## 当前只改两个文件

退出Debug。保持app_config.h静态开关原值，不改main，不下载。打开当前工程的USER/chassis_auto_test.h，在最后的`#endif`前添加以下代码：

```c
/* TaskA到CAN任务的纯数据交接；1有效，0无效/过期。 */
#define CHASSIS_HEADING_MAILBOX_MAX_AGE_MS 100U
typedef struct
{
    int32_t command_rpm;
    uint32_t stop_required;
    uint32_t sequence;
} ChassisHeadingRequest;

uint32_t ChassisAuto_HeadingPublish(int32_t command_rpm, uint32_t stop_required);
uint32_t ChassisAuto_HeadingRead(ChassisHeadingRequest *request);
```

然后打开USER/chassis_auto_test.c，在文件末尾、ChassisAuto_CanStep函数结束的大括号后，追加以下代码。原文件已有task.h、stddef.h和临界区所需头文件，不需增加工程文件。

```c
/* 此处仅交接数据，不发CAN、不修改VM或原自动测试状态。 */
static ChassisHeadingRequest s_heading_request = {0, 1U, 0U};
static TickType_t s_heading_request_tick = 0U;
static uint32_t s_heading_request_valid = 0U;

uint32_t ChassisAuto_HeadingPublish(int32_t command_rpm, uint32_t stop_required)
{
    uint32_t valid = ((command_rpm >= -5) && (command_rpm <= 5) &&
        (stop_required <= 1U) &&
        ((stop_required == 0U) || (command_rpm == 0))) ? 1U : 0U;

    taskENTER_CRITICAL();
    /* 非法发布也覆盖旧指令，不能继续使用上一次非零轮速。 */
    s_heading_request.command_rpm = valid ? command_rpm : 0;
    s_heading_request.stop_required = valid ? stop_required : 1U;
    s_heading_request.sequence++;
    s_heading_request_tick = xTaskGetTickCount();
    s_heading_request_valid = valid;
    taskEXIT_CRITICAL();
    return valid;
}

uint32_t ChassisAuto_HeadingRead(ChassisHeadingRequest *request)
{
    uint32_t valid;
    TickType_t now;
    if (request == NULL) return 0U;

    taskENTER_CRITICAL();
    now = xTaskGetTickCount();
    valid = s_heading_request_valid;
    if ((valid != 0U) &&
        ((TickType_t)(now - s_heading_request_tick) >=
         pdMS_TO_TICKS(CHASSIS_HEADING_MAILBOX_MAX_AGE_MS)))
    {
        s_heading_request_valid = 0U;
        valid = 0U;
    }
    *request = s_heading_request;
    if (valid == 0U)
    {
        request->command_rpm = 0;
        request->stop_required = 1U;
    }
    taskEXIT_CRITICAL();
    return valid;
}
```

保存两文件，完整Rebuild，把日志发来。本轮不添加任何新调用，现有运动流程保持原样；新增函数可能暂被链接器移除，程序大小不变不代表没有保存。

## 为什么先做这个接口

TaskA负责计算并发布，唯一CAN任务以后读取快照并处理四轮更新。临界区使轮速、停止请求、序号和时间一起交接；临界区里不等待CAN。当前两个函数只是数据接口，没有发送帧或改变VM的代码。只有任务上下文可调用，不能从ISR调用。

Publish接收-5至+5RPM和0/1停止请求；停止请求为1时轮速必须为0。非法发布覆盖旧目标并置为无效，避免继续沿用上一次非零目标。Read未收到发布或指令到龄时返回0，并在输出中给出轮速0、停止请求1；有效时返回1。有效的stop_required1也返回1，其含义是收到有效的停车目标，调用方仍须检查停止字段。

Read不会刷新指令时间。100ms为本阶段待整合验证的工程候选时效，不是官方规则，也不是已验收的端到端制动时间。使用pdMS_TO_TICKS，不把RTOS tick直接当毫秒。sequence只供识别更新，无需手动修改。

该接口不是会话状态机，也不是安全许可：失效后再次收到合法发布可以恢复数据有效；未来执行器必须锁存会话故障，不能因数据恢复自动重启。将来停止/取消/传感器失效都要清除未发送目标，并走真实停止反馈与VM兜底，不能把本接口输出0当成电机已经停止。

后续将接入受控目标角会话和唯一CAN服务：固定目标、准备及反馈检查、只在允许运行阶段发送速度、更新周期和四轮顺序、变向互锁、最终停车与故障截止。旧固定10秒序列不能直接当作动态目标角闭环。目前保持静态模式，不把g_heading_live_motor数组直接发CAN。

## 已做的隔离检查与限度

候选代码及检查文件在tmp/heading_mailbox_*20261021*。GCC以C99、Wall/Wextra/Werror构建后，分别按1ms和10ms tick运行通过：初始无发布、空指针、正负目标、连续读取不续期、100ms边界失效、非法指令覆盖旧非零目标、停止请求一致性、tick回绕。ARMCC V5 Cortex-M4候选检查单元编译成功，无诊断。

这些检查使用模拟tick和空临界区宏，只验证数据处理和过期逻辑，不能证明真实RTOS并发或整个工程已构建。用户实际文件尚待手改/Rebuild。助手未改实际固件或Keil工程，未烧录、发送CAN或操作硬件。

R已检查：RUN-01/PWR-01。E已检查：LOC-03/05、RUN-06、MOT-02/04。Q：官方未决项继续待技术答疑确认。未闭环：目标航向执行层整合、真实调速/停车与失效保护、物理转角精度、实时X/Y。
