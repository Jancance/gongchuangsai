#ifndef ICM45686_SAMPLE_ONLY
#if APP_BASELINE40_ENABLE
#define ICM45686_SAMPLE_ONLY 0U
#else
#define ICM45686_SAMPLE_ONLY 1U
#endif
#endif
#if (ICM45686_SAMPLE_ONLY != 0U) && (ICM45686_SAMPLE_ONLY != 1U)
#error ICM45686_SAMPLE_ONLY must be 0 or 1
#endif

#include "stm32f4xx.h"
#include "FreeRTOS.h"
#include "task.h"
#include "freertos_hooks.h"
#include "led.h"
#include "Buzzer.h"
#include "app_config.h"
#include "startup_pose.h"
#include "startup_axes.h"
#include "vehicle_arm.h"
#include "vehicle_mission.h"
#include "route_plan.h"
extern volatile RouteCheck g_route_check;
#include "dm_can2.h"
#include "radar_observer.h"
#include "radar_servo.h"
#include "radar_lift.h"
#include "radar_zone_switch.h"
#include "fine_rpm_telemetry.h"
#include "app_entry.h"
#include "usart.h"
#include "Key.h"
#include "power.h"
#include "can.h"
#include "chassis_kinematics.h"
#include "chassis_auto_test.h"
#include "app_odometry.h"
#include "app_icm_observer.h"
#include "app_can_diagnostics.h"
#include "app_heading_runtime.h"
#include "app_translation_runtime.h"
#include "app_motion_internal.h"
#include "app_motion_runtime.h"
#include "app_runtime_route.h"
#include "app_serial_runtime.h"
#include "app_debug_runtime.h"
#include "app_input_runtime.h"
#include "app_chassis_service.h"
#include "hwt101.h"
#include "heading_source.h"
#include "safety.h"
#include "watchdog.h"
#include "screen.h"
#include "saoma.h"
#include <stdio.h>
#include <string.h>
#include "heading_controller.h"
#include "heading_session.h"
#include "route_heading_reference.h"
#include "motion_action_plan.h"
#include "motion_control.h"
#include "runtime_route.h"
#include "motion_distance_limits.h"
#include "vehicle_log_config.h"
#include "field_map.h"
#include "icm45686_raw.h"

/* 临时静态供电测试：0关闭，1请求开启 */
volatile uint32_t g_vm_test_request = 0U;
/* 测试状态，可在Watch中观察 */
volatile uint32_t g_test_count_a = 0U;
volatile uint32_t g_test_count_b = 0U;
volatile TickType_t g_test_tick = 0U;
volatile BaseType_t g_test_create_a = pdFAIL;
volatile BaseType_t g_test_create_b = pdFAIL;
volatile uint32_t g_test_scheduler_returned = 0U;
/* 1表示发送请求被接受，不代表电脑已经收到 */
volatile uint32_t g_serial_send_ok = 0U;
/* 已处理的接收字节数与最后一个字节 */
volatile uint32_t g_serial_rx_count = 0U;
volatile uint32_t g_serial_rx_last = 0U;
/* 接收任务创建结果、栈余量及本轮序列错误数 */
volatile BaseType_t g_rx_task_create = pdFAIL;
volatile UBaseType_t g_rx_stack_words = 0U;
/* 启动按键任务创建结果及栈余量 */
volatile BaseType_t g_key_task_create = pdFAIL;
volatile UBaseType_t g_key_stack_words = 0U;
/* 看门狗初始化结果，成功后才启动应用任务 */
volatile uint32_t g_watchdog_init_result = 0U;
volatile uint32_t g_stage1_stop_requested=0U;
volatile uint32_t g_stage1_motion_skipped=0U;
/* CAN独立采集任务的创建结果、循环次数和历史最小余栈 */
volatile BaseType_t g_can_task_create = pdFAIL;
volatile uint32_t g_can_task_count = 0U;
volatile UBaseType_t g_can_task_stack_words = 0U;

/* 监督任务创建结果及历史最小剩余栈 */
volatile BaseType_t g_safety_task_create = pdFAIL;
volatile UBaseType_t g_safety_stack_words = 0U;

/* 栈余量单位为字，每字4字节 */
volatile UBaseType_t g_stack_a_words = 0U;
volatile UBaseType_t g_stack_b_words = 0U;
/* 堆余量单位为字节 */
volatile size_t g_heap_free_bytes = 0U;
volatile size_t g_heap_min_bytes = 0U;

#if APP_BASELINE40_ACTIVE && ICM45686_SAMPLE_ONLY
#error Baseline40_cannot_use_sample_only
#endif
#if !ICM45686_SAMPLE_ONLY
static void App_CanTask(void *argument)
{
    (void)argument;
    AppChassis_CanInit();
    while (1)
    {
        g_can_task_count++;
        AppChassis_CanStep();
        g_can_task_stack_words = uxTaskGetStackHighWaterMark(NULL);
        vTaskDelay(pdMS_TO_TICKS(APP_UART_LINK_TEST ? 50U : (APP_HEADING_SESSION_TEST ? 10U : 50U)));
    }
}

/* 任务A：每100毫秒计数一次 */
/* 独立串口测试任务：替代业务任务A，不执行扫码显示或运动门控。 */
static void App_UartLinkTestTask(void *argument)
{
    (void)argument;
    if (APP_IS_DEBUG) USART1_EnableReceive();
    HWT101_EnableReceive();
    Buzzer_Start(APP_INIT_BEEP_COUNT);
    while (1)
    {
        HWT101_Update();
        Buzzer_Update();
        g_test_count_a++;
        g_test_tick = xTaskGetTickCount();
        g_stack_a_words = uxTaskGetStackHighWaterMark(NULL);
        g_heap_free_bytes = xPortGetFreeHeapSize();
        g_heap_min_bytes = xPortGetMinimumEverFreeHeapSize();
        /* 完成真实接收解析后报告进展，原安全监督和看门狗不变。 */
        Safety_Report(SAFETY_TASK_APP);
        vTaskDelay(pdMS_TO_TICKS(10U));
    }
}

static void App_TestTaskA(void *argument)
{
    TickType_t screen_start_tick = xTaskGetTickCount();
    (void)argument;
    /* FreeRTOS运行后才开启接收中断 */
    if (APP_IS_DEBUG) USART1_EnableReceive();
    /* 启动时只请求发送一次基础平台就绪信息 */
    if (!APP_BASELINE40_ACTIVE) SAOMA_EnableReceive();
    HWT101_EnableReceive();
    g_serial_send_ok = App_DebugSendString("F407 FreeRTOS READY\r\n");
    /* 已验收的基础平台就绪后，只请求一次提示音 */
    Buzzer_Start(APP_INIT_BEEP_COUNT);
        /* 一次性虚拟角度自检，不驱动小车。 */
    App_HeadingDryRun();
#if APP_SERIAL_MOTION_CONTROL
    AppSerial_Init();
#endif
#if APP_HEADING_SESSION_TEST && APP_FORWARD_HOLD_TEST
    /* 只运行数学自检，失败禁止平移启动，默认不发动作。 */
    g_translation_hold_selftest = Chassis_TranslationHoldSelfTest() ? 1U : 2U;
#if APP_TEST_MODE == 3U
    AppMotion_LoadDefaultPlan();
#endif
#endif
    if (!APP_BASELINE40_ACTIVE) AppMotion_InitStage1Plan();
    while (1)
    {
        g_test_count_a++;
        
        /* 每100毫秒更新响停状态 */
        Buzzer_Update();
        HWT101_Update();
        #if APP_HEADING_SESSION_TEST || (APP_HWT_STATIC_TEST && !APP_HWT_TURN_TEST)
        App_HeadingSourcePublish();
        #endif
        #if APP_HWT_STATIC_TEST && !APP_HWT_TURN_TEST
        App_HeadingLiveStep();
        #endif
        /* 一轮最多处理64字节；只有任务A交接并更新屏幕 */
        if (!APP_BASELINE40_ACTIVE) SAOMA_Update();
        if(APP_BASELINE40_ACTIVE) { /* Dedicated chassis branch; no mission dispatch. */ }
        else if(VEHICLE_INTEGRATED_ENABLE){AppMotion_MissionService();AppMotion_MissionTelemetry();}
        else if(VEHICLE_STAGE1_ENABLE)
        {
            AppMotion_Stage1Service();
            AppMotion_Stage1Display((TickType_t)(xTaskGetTickCount()-screen_start_tick)>=pdMS_TO_TICKS(1000U));
        }
        else
        {
            AppInput_ScanDisplay(screen_start_tick);
        }

        if (!APP_BASELINE40_ACTIVE) App_StartGateUpdate();
#if APP_SERIAL_MOTION_CONTROL
        AppSerial_Telemetry();
#endif
        g_test_tick = xTaskGetTickCount();
        /* 查询当前任务的最小剩余栈及堆余量 */
        g_stack_a_words = uxTaskGetStackHighWaterMark(NULL);
        g_heap_free_bytes = xPortGetFreeHeapSize();
        g_heap_min_bytes = xPortGetMinimumEverFreeHeapSize();
        /* 一轮查询和平台工作结束后，报告真实进展 */
        Safety_Report(SAFETY_TASK_APP);
        
        
        vTaskDelay(pdMS_TO_TICKS(10U));
    }
}

/* 任务B：每500毫秒计数一次 */
static void App_TestTaskB(void *argument)
{
    (void)argument;

    while (1)
    {
        g_test_count_b++;
        /* 每500毫秒翻转一次，完整亮灭周期约1秒 */
        LED_Turn();
        /* 查询任务B自己的最小剩余栈 */
        g_stack_b_words = uxTaskGetStackHighWaterMark(NULL);
        vTaskDelay(pdMS_TO_TICKS(500U));
    }
}

/* 独立接收任务：有数据立即处理，无数据进入等待 */
static void App_UartRxTask(void *argument)
{
    uint8_t data;

    (void)argument;

    while (1)
    {
        if (USART1_WaitByte(&data, 100U) != 0U)
        {
            g_serial_rx_count++;
            g_serial_rx_last = data;
#if APP_UART_LINK_TEST
            App_UartLinkByte(data);
#elif APP_SERIAL_MOTION_CONTROL
            App_SerialMotionByte(data);
#endif
        }
        else
        {
            /* 空闲时记录历史最小剩余栈，单位为字 */
            g_rx_stack_words = uxTaskGetStackHighWaterMark(NULL);
        }
#if APP_UART_LINK_TEST
        App_UartLinkPoll();
#endif
        /* 有数据或正常空闲等待返回，均表示任务仍在运行 */
        Safety_Report(SAFETY_TASK_RX);
    }
}

static void App_KeyTask(void *argument)
{
    uint32_t diagnostic_count = 0U;

    (void)argument;

    while (1)
    {
        AppInput_KeyStep();
        diagnostic_count++;
        if (diagnostic_count >= 100U)
        {
            g_key_stack_words = uxTaskGetStackHighWaterMark(NULL);
            diagnostic_count = 0U;
        }
        Safety_Report(SAFETY_TASK_KEY);
        vTaskDelay(pdMS_TO_TICKS(1U));
    }
}

/* 监督不等待CAN或串口，检查后立即让出CPU */
static void App_SafetyTask(void *argument)
{
    (void)argument;

    while (1)
    {
        AppChassis_SafetyCheck();
        Safety_Update(g_vm_test_request, g_can_test_result);
        g_safety_stack_words = uxTaskGetStackHighWaterMark(NULL);
        vTaskDelay(pdMS_TO_TICKS(20U));
    }
}

/* 单次观察入口：结果0待处理/1接受/2拒绝，拒绝后不自动重试。 */
volatile uint32_t g_radar_app_config_result = 0U;
volatile uint32_t g_radar_app_calibration_result = 0U;
volatile uint32_t g_radar_app_scan_result = 0U;

/* 本次上电仅校准一次；成功并收拢后仅扫描、规划一次，不接底盘。 */
#if !APP_BASELINE40_ACTIVE
static void App_RadarTask(void *argument)
{
    uint32_t calibration_attempted = 0U;
    uint32_t scan_attempted = 0U;
    RadarObserverStatus radar_status;
    /* 用户2026-11-11临时授权measured=1，仅供校准和路径观察；
     * 8559约70mm为单样本比例候选，未双向标定；等待时间为工程选择，非实测。 */
    const RadarZone2Config cfg = {
        1U, 1U, 8559U, 1000U, 500U, 1000U, 60000U, 1U
    };

    (void)argument;
    g_radar_app_config_result = RadarObserver_Configure(&cfg) ? 1U : 2U;
    for (;;)
    {
        if (g_radar_app_config_result == 1U)
        {
            RadarObserver_Service();
            RadarObserver_GetStatus(&radar_status);
            if (radar_status.stage != 99U && radar_status.error == 0U)
            {
                if (calibration_attempted == 0U && RadarClient_IsReady())
                {
                    /* 仅入队一次；本次人工确认顶部、180度收拢起点。 */
                    calibration_attempted = 1U;
                    /* 默认沿用已保存校准；恢复本行则等待本轮校准成功。 */
                    /* g_radar_app_calibration_result = RadarCalibration_StartZone2() ? 1U : 2U; */
                }
                else if (calibration_attempted != 0U && scan_attempted == 0U &&
                         (g_radar_app_calibration_result == 0U ||
                          (g_radar_app_calibration_result == 1U &&
                         radar_status.stage == 8U && radar_status.calibration_complete == 1U)))
                {
                    /* 先观察到校准最终成功，再请求本场扫描和0到24路径。 */
                    scan_attempted = 1U;
                    g_radar_app_scan_result = RadarObserver_StartZone2(0, 24, true) ? 1U : 2U;
                }
            }
        }
        vTaskDelay(pdMS_TO_TICKS(2U));
    }
}

#endif /* !APP_BASELINE40_ACTIVE */
#endif /* !ICM45686_SAMPLE_ONLY */

#if ICM45686_RAW_ENABLE || ICM45686_SAMPLE_ONLY || APP_BASELINE40_ACTIVE
/* 可选RAW观察，不加入整车安全门槛。创建失败仅保留结果。 */
volatile BaseType_t g_icm_task_create = pdFAIL;
volatile IcmRawStatus g_icm_raw_status;
volatile uint32_t g_icm_publish_sequence = 0U;
volatile UBaseType_t g_icm_stack_words = 0U;
uint32_t AppIcm_ReadPublished(IcmRawStatus *out, uint32_t *publication)
{
    uint32_t before, after, attempt;
    if (!out || !publication) return 0U;
    for (attempt=0U;attempt<3U;attempt++) {
        before=g_icm_publish_sequence;
        if (before&1U) continue;
        __DMB();
        *out=g_icm_raw_status;
        __DMB();
        after=g_icm_publish_sequence;
        if (before==after && !(after&1U)) { *publication=after;return 1U; }
    }
    return 0U;
}
static void App_IcmRawTask(void *argument)
{
    IcmRawStatus snapshot;
    (void)argument;
    (void)IcmRaw_Init();
    for (;;)
    {
        IcmRaw_Service();
        IcmRaw_GetStatus(&snapshot);
#if APP_BASELINE40_ACTIVE
        AppIcmObserver_Step(&snapshot);
#endif
        g_icm_publish_sequence++;
        __DMB();
        g_icm_raw_status = snapshot;
        __DMB();
        g_icm_publish_sequence++;
        g_icm_stack_words = uxTaskGetStackHighWaterMark(NULL);
        vTaskDelay(pdMS_TO_TICKS(1U));
    }
}
#endif

int main(void)
{
    /* 更新时钟，将4位优先级全部用于抢占优先级 */
    SystemCoreClockUpdate();
    NVIC_PriorityGroupConfig(NVIC_PriorityGroup_4);
#if ICM45686_SAMPLE_ONLY
    Power_Init();
    g_icm_task_create = xTaskCreate(App_IcmRawTask, "IcmRaw", 256U, NULL, 3U, NULL);
    configASSERT(g_icm_task_create == pdPASS);
#else
    /* 调度启动前初始化状态灯 */
    LED_Init();
    /* 调度前初始化蜂鸣器，保持关闭 */
    Buzzer_Init();
    /* 初始化调试串口，本轮仅发送 */
    USART1_Config();
    /* 调度前初始化启动按键 */
    /* UART4仅初始化，实际发送放到任务A */
    Screen_Init();
    Key_Init();
    /* 初始化PE2，默认关闭VM允许请求 */
    Power_Init();
#if !APP_BASELINE40_ACTIVE
    StartupPose_Init();
    RadarZoneSwitch_Init();
    (void)RadarServo_Init();
#endif
    Safety_Init();
    if (!APP_BASELINE40_ACTIVE) SAOMA_Init();
    HWT101_Init();
    g_watchdog_init_result = Watchdog_Init();
    configASSERT(g_watchdog_init_result == 1U);
    
    /* 每个任务栈为256个字，即1024字节 */
    /* 编译常量选择任务，只创建一方；关闭测试即可恢复业务。 */
    if (APP_UART_LINK_TEST != 0U)
        g_test_create_a = xTaskCreate(App_UartLinkTestTask, "UartTest",
                                     256U, NULL, 2U, NULL);
    else
        g_test_create_a = xTaskCreate(App_TestTaskA, "TestA",
                                     (APP_SERIAL_MOTION_CONTROL ? 512U : 256U), NULL, 2U, NULL);
    configASSERT(g_test_create_a == pdPASS);
    g_test_create_b = xTaskCreate(App_TestTaskB, "TestB",
                                 256U, NULL, 1U, NULL);
    configASSERT(g_test_create_b == pdPASS);
    /* 本轮接收任务栈256字，优先级3 */
    g_rx_task_create = xTaskCreate(App_UartRxTask, "UartRx",
                                   (APP_SERIAL_MOTION_CONTROL ? 384U : 256U), NULL, 3U, NULL);
    configASSERT(g_rx_task_create == pdPASS);
    /* 本轮按键扫描任务，栈256字、优先级2 */
    g_key_task_create = xTaskCreate(App_KeyTask, "Key",
                                    256U, NULL, 2U, NULL);
    configASSERT(g_key_task_create == pdPASS);
    /* 优先级4，高于当前接收3、任务A/按键2；无阻塞外设操作 */
    g_safety_task_create = xTaskCreate(App_SafetyTask, "Safety",
                                      256U, NULL, 4U, NULL);
    configASSERT(g_safety_task_create == pdPASS);
    
    
    
    /* CAN栈384字，优先级3；监督仍为4 */
    g_can_task_create = xTaskCreate(App_CanTask, "CanPoll",
                                      384U, NULL, 3U, NULL);
    configASSERT(g_can_task_create == pdPASS);
    /* 启动后由任务运行 */
    /* 雷达仅本次校准/扫描/路径观察；上电自动运行一次，不接底盘。 */
#if !APP_BASELINE40_ACTIVE
    configASSERT(xTaskCreate(App_RadarTask, "RadarObs", 768U, NULL, 2U, NULL) == pdPASS);
#endif
#if ICM45686_RAW_ENABLE || APP_BASELINE40_ACTIVE
    g_icm_task_create = xTaskCreate(App_IcmRawTask, "IcmRaw", 256U, NULL, 3U, NULL);
    configASSERT(g_icm_task_create == pdPASS);
#endif
#endif /* ICM45686_SAMPLE_ONLY */
    vTaskStartScheduler();

    /* 调度器异常返回时记录并停止 */
    g_test_scheduler_returned = 1U;
    configASSERT(0);

    while (1)
    {
    }
}
