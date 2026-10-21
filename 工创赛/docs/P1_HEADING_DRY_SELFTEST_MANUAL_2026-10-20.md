# 目标航向虚拟角度自检：用户手动修改

2026-10-20。用户已提交完整Rebuild：Code35604/RO452/RW1332/ZI25284，0错误0警告，4秒。只读实际源确认ErrorDeg已先分别归一化目标角和当前角。本指南由用户手动改代码；助手只修改说明和隔离检查文件，不改实际固件或工程。

本步在F407上执行一次纯计算自检，输入为代码给定的虚拟角度/时间。它检查目标+45°/-45°、最短跨界、减速、到位、失效和超时，不能称为真实HWT/电机闭环通过。没有电机/CAN动作调用，不读取真实姿态作为自检输入。

## 1. 切到现有静态模式

打开当前工程 `D:/Codex/工创赛/3.stm32f407工程/gongxun-f407.uvprojx`。

在USER/app_config.h，只将以下两项改为：

```c
#define APP_HWT_STATIC_TEST 1U
#define APP_HWT_TURN_TEST 0U
```

其余开关和数值保留。现有Safety任务在APP_HWT_STATIC_TEST为1时每轮把VM请求保持0，按键也不会进入底盘测试。不要触发HWT校准/配置请求。当前APP_HWT_TURN_CLOCKWISE仍可保留1，该方向配置在本静态模式不生效。

## 2. main.c增加头文件

在 `#include "chassis_auto_test.h"` 后添加：

```c
#include "heading_controller.h"
```

## 3. main.c增加一次性自检函数

在 `static void App_TestTaskA(void *argument)` 定义之前，粘贴以下整段。保留原App_TestTaskA及其他函数，保持文件原有编码。

```c
/* 仅虚拟数据的软件自检：0未执行，1通过，2失败。case非0定位失败组。 */
volatile uint32_t g_heading_dry_result = 0U;
volatile uint32_t g_heading_dry_case = 0U;
volatile int32_t g_heading_dry_ccw[5] = {0};
volatile int32_t g_heading_dry_cw[5] = {0};
volatile float g_heading_dry_cross_error = 0.0f;

static void App_HeadingDryRun(void)
{
    HeadingController controller = {0};
    HeadingControllerOutput output = {0};
    const HeadingControllerConfig *config = &g_heading_controller_default_config;
    static const float yaw[2][5] = {
        {0.0f, 25.0f, 39.0f, 44.0f, 44.0f},
        {0.0f, -25.0f, -39.0f, -44.0f, -44.0f}
    };
    static const uint32_t tick[5] = {0U, 100U, 200U, 300U, 1000U};
    static const int32_t expected[5] = {5, 3, 1, 0, 0};
    uint32_t direction, i, expected_state;
    int32_t expected_command;
    float error;

    g_heading_dry_result = 2U;
    /* 两组各5次计算，虚拟tick不会让任务真实等待这些时间。 */
    for (direction = 0U; direction < 2U; direction++)
    {
        g_heading_dry_case = direction + 1U;
        if (HeadingController_Start(&controller, config,
            (direction == 0U) ? 45.0f : -45.0f, 0.0f, 0U) != 1U) return;
        for (i = 0U; i < 5U; i++)
        {
            if (HeadingController_Step(&controller, tick[i], yaw[direction][i],
                0.0f, 1U, &output) != 1U) return;
            expected_command = (direction == 0U) ? expected[i] : -expected[i];
            if (direction == 0U) g_heading_dry_ccw[i] = output.command_rpm;
            else g_heading_dry_cw[i] = output.command_rpm;
            expected_state = (i < 3U) ? HEADING_CTRL_TURNING :
                ((i == 3U) ? HEADING_CTRL_SETTLING : HEADING_CTRL_DONE);
            if ((output.command_rpm != expected_command) ||
                (output.state != expected_state) ||
                (output.cause != HEADING_CAUSE_NONE) ||
                (output.stop_required != ((i < 3U) ? 0U : 1U))) return;
        }
    }

    /* +179到-179最短误差为+2度，在当前2度候选容差内。 */
    g_heading_dry_case = 3U;
    if (HeadingController_Start(&controller, config, -179.0f, 179.0f, 0U) != 1U)
        return;
    if (HeadingController_Step(&controller, 0U, 179.0f, 0.0f, 1U, &output) != 1U)
        return;
    g_heading_dry_cross_error = output.error_deg;
    if ((output.error_deg < 1.999f) || (output.error_deg > 2.001f) ||
        (output.state != HEADING_CTRL_SETTLING) || (output.command_rpm != 0)) return;
    if (HeadingController_Step(&controller, 700U, -179.0f, 0.0f, 1U, &output) != 1U)
        return;
    if ((output.state != HEADING_CTRL_DONE) || (output.stop_required != 1U)) return;

    g_heading_dry_case = 4U;
    if (HeadingController_Start(&controller, config, 45.0f, 0.0f, 0U) != 1U) return;
    if (HeadingController_Step(&controller, 1U, 0.0f, 0.0f, 0U, &output) != 1U)
        return;
    if ((output.state != HEADING_CTRL_FAULT) ||
        (output.cause != HEADING_CAUSE_SENSOR_INVALID) ||
        (output.command_rpm != 0) || (output.stop_required != 1U)) return;

    g_heading_dry_case = 5U;
    if (HeadingController_Start(&controller, config, 45.0f, 0.0f, 0U) != 1U) return;
    if (HeadingController_Step(&controller, config->timeout_ms,
        0.0f, 0.0f, 1U, &output) != 1U) return;
    if ((output.state != HEADING_CTRL_FAULT) ||
        (output.cause != HEADING_CAUSE_TIMEOUT) ||
        (output.command_rpm != 0) || (output.stop_required != 1U)) return;

    g_heading_dry_case = 6U;
    if (HeadingController_Start(&controller, config, 45.0f, 0.0f, 0U) != 1U) return;
    HeadingController_Cancel(&controller, &output);
    if ((output.state != HEADING_CTRL_CANCELLED) ||
        (output.cause != HEADING_CAUSE_CANCELLED) ||
        (output.command_rpm != 0) || (output.stop_required != 1U)) return;

    /* 验证刚修正的两角分别归一化，不允许误返回0。 */
    g_heading_dry_case = 7U;
    error = HeadingController_ErrorDeg(179.0f, -3500.0f);
    if ((error < 78.999f) || (error > 79.001f)) return;
    error = HeadingController_ErrorDeg(-179.0f, 3500.0f);
    if ((error < -79.001f) || (error > -78.999f)) return;

    g_heading_dry_case = 0U;
    g_heading_dry_result = 1U;
}
```

case1/2表示正反45度分段/到位，3跨界，4传感器无效，5超时，6取消，7分别归一化回归；首次失败直接return，保留result2/case以定位问题。结果和数组仅用于观察，不作为电机指令。

## 4. TaskA启动部分调用一次

找到App_TestTaskA开始处的：

```c
    Buzzer_Start(APP_INIT_BEEP_COUNT);
```

在其后、`while (1)`之前添加：

```c
    /* 一次性虚拟角度自检，不驱动小车。 */
    App_HeadingDryRun();
```

不要放进while循环，不改CAN任务、Safety或现有测试状态机，不需要把浮点轮速接入CAN。

## 5. 本轮先保存、Rebuild

当前只完成1～4节并Rebuild，发送完整构建日志；构建确认后再下载观察。新日志不得用来替代真实转角实测。

指南中同一段自检代码已在隔离副本执行：result1/case0，正转{5,3,1,0,0}、反转{-5,-3,-1,0,0}、跨界2.000度。当前实际控制器另有58项边界检查通过。按本指南模拟修改的main及当前控制器均由ARMCC编译通过，无错误/警告输出；没有整工程链接或烧录这个新候选。实际main/app_config/控制器/工程哈希在检查前后保持一致。记录见heading_dry_manual_preparation.json。此证据不能代替用户手动修改后的完整Rebuild。

## 6. 构建核对后才进行的软件自检观察

在VM外部电源关闭条件下下载新静态固件，Run约2秒后Stop。不设置VM请求、转动请求或HWT校准请求。Watch观察：

```text
g_heading_dry_result          = 1
g_heading_dry_case            = 0
g_heading_dry_ccw             = {5,3,1,0,0}
g_heading_dry_cw              = {-5,-3,-1,0,0}
g_heading_dry_cross_error     = 2.0（允许微小浮点误差）
g_vm_test_request             = 0
g_rtos_fault                 = 0
g_safety_fault               = 0
```

同时记录APP余栈和最小堆。结果0通常表示调用尚未执行，结果2查看case定位；不要手改result造通过。自检只运行一次，复位才重跑。当前参数为默认候选，手动调参数后预期值也需要重新审阅。

## 范围和依据

R：RUN-01/PWR-01；E：LOC-03/05、RUN-06、MOT-04；官方Q未变。用户本轮Rebuild及实际ErrorDeg修正已核对；后续虚拟自检、运行路径集成、真实停止、到位精度和实时X/Y均待相应实测。助手没有修改本次实际main/app_config/heading源码或工程文件。此说明中的代码会由用户手动加入。
