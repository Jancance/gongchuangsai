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
#include "radar_zone_config.h"
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
#include "app_fusion_capture.h"
#include "app_can_diagnostics.h"
#include "app_heading_runtime.h"
#include "app_translation_runtime.h"
#include "app_motion_internal.h"
#include "app_motion_runtime.h"
#include "app_runtime_route.h"
#include "app_serial_runtime.h"
#include "app_debug_runtime.h"
#include "app_input_runtime.h"
#include "vehicle_tuning.h"
#include "app_chassis_service.h"
#if APP_ZONE2_MISSION_ENABLE
#include "app_zone2.h"
#endif
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

#if APP_VM_CALIBRATION_ONLY_ENABLE
/* Standalone calibration supply: no scheduler, CAN, Safety or watchdog start. */
volatile uint32_t g_vm_calibration_only_cancelled, g_vm_calibration_only_active;
volatile uint32_t g_vm_calibration_only_option_control;
/* Sampling is inactive; dormant telemetry receives unavailable. */
uint32_t AppIcm_ReadPublished(IcmRawStatus *out,uint32_t *publication)
{
    (void)out;(void)publication;return 0U;
}
static void App_VmCalibrationPowerInit(void)
{
    GPIO_InitTypeDef gpio;
    SystemCoreClockUpdate();
    g_vm_calibration_only_option_control=FLASH->OPTCR;
    RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOE|RCC_AHB1Periph_GPIOC,ENABLE);
    GPIO_ResetBits(GPIOE,GPIO_Pin_2);
    GPIO_StructInit(&gpio);gpio.GPIO_Pin=GPIO_Pin_2;gpio.GPIO_Mode=GPIO_Mode_OUT;
    gpio.GPIO_OType=GPIO_OType_PP;gpio.GPIO_Speed=GPIO_Speed_2MHz;gpio.GPIO_PuPd=GPIO_PuPd_NOPULL;
    GPIO_Init(GPIOE,&gpio);
    GPIO_StructInit(&gpio);gpio.GPIO_Pin=GPIO_Pin_13;gpio.GPIO_Mode=GPIO_Mode_IN;
    gpio.GPIO_PuPd=GPIO_PuPd_UP;GPIO_Init(GPIOC,&gpio);
    if(GPIO_ReadInputDataBit(GPIOC,GPIO_Pin_13)==Bit_RESET)g_vm_calibration_only_cancelled=1U;
    else {GPIO_SetBits(GPIOE,GPIO_Pin_2);g_vm_calibration_only_active=1U;}
}
static void App_VmCalibrationPowerStep(void)
{
    if(GPIO_ReadInputDataBit(GPIOC,GPIO_Pin_13)==Bit_RESET) {
        g_vm_calibration_only_cancelled=1U;g_vm_calibration_only_active=0U;
        GPIO_ResetBits(GPIOE,GPIO_Pin_2);
    }
}
int main(void)
{
    App_VmCalibrationPowerInit();
    for(;;)App_VmCalibrationPowerStep();
}
#else
#if APP_BASELINE40_ACTIVE && ICM45686_SAMPLE_ONLY
#error Baseline40_cannot_use_sample_only
#endif
#if APP_ZONE2_NAV_ENABLE && ICM45686_SAMPLE_ONLY
#error Zone2_nav_requires_explicit_ICM45686_SAMPLE_ONLY_0
#endif
#if !ICM45686_SAMPLE_ONLY
#if !APP_ZONE2_RADAR_CALIBRATION_TEST
static void App_CanTask(void *argument)
{
#if APP_ZONE2_NAV_ENABLE
    const TickType_t period=pdMS_TO_TICKS(10U);
    TickType_t slot;
    uint32_t phase=0U;
#endif
    (void)argument;
    AppChassis_CanInit();
#if APP_ZONE2_NAV_ENABLE
    slot=xTaskGetTickCount();
#endif
    while (1)
    {
        g_can_task_count++;
#if APP_ZONE2_NAV_ENABLE
        TickType_t missed=(xTaskGetTickCount()-slot)/period;
        slot+=missed*period;
        phase=(phase+missed)%8U;
        AppChassis_Zone2CanStep((uint32_t)slot*portTICK_PERIOD_MS,phase);
        phase=(phase+1U)%8U;
#else
        AppChassis_CanStep();
#endif
        g_can_task_stack_words = uxTaskGetStackHighWaterMark(NULL);
#if APP_ZONE2_NAV_ENABLE
        vTaskDelayUntil(&slot,period);
#else
        vTaskDelay(pdMS_TO_TICKS(APP_UART_LINK_TEST ? 50U : (APP_HEADING_SESSION_TEST ? 10U : 50U)));
#endif
    }
}
#endif

/* 任务A：每100毫秒计数一次 */
/* 独立串口测试任务：替代业务任务A，不执行扫码显示或运动门控。 */
static void App_UartLinkTestTask(void *argument)
{
    (void)argument;
    if (APP_IS_DEBUG) USART1_EnableReceive();
    HWT101_EnableReceive();
#if !APP_ZONE2_NAV_ENABLE
    Buzzer_Start(APP_INIT_BEEP_COUNT);
#endif
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

#if APP_ZONE2_RADAR_CALIBRATION_TEST
#define ZONE2_RADAR_TEST_RUN_ID 1U
static volatile uint32_t s_zone2_radar_test_beep_ms;
#endif

static void App_TestTaskA(void *argument)
{
    TickType_t screen_start_tick = xTaskGetTickCount();
#if APP_ZONE2_RADAR_CALIBRATION_TEST
    uint32_t radar_test_announced = 0U;
#endif
    (void)argument;
    /* FreeRTOS运行后才开启接收中断 */
    if (APP_IS_DEBUG) USART1_EnableReceive();
    /* 启动时只请求发送一次基础平台就绪信息 */
    if (APP_ZONE2_MISSION_ENABLE || (!APP_BASELINE40_ACTIVE && !APP_ZONE2_NAV_ENABLE)) SAOMA_EnableReceive();
#if APP_ZONE2_MISSION_ENABLE
    USART6_VisionEnableReceive();
#endif
    HWT101_EnableReceive();
    g_serial_send_ok = App_DebugSendString("F407 FreeRTOS READY\r\n");
#if !APP_ZONE2_RADAR_CALIBRATION_TEST
    /* 已验收的基础平台就绪后，只请求一次提示音 */
#if !APP_ZONE2_NAV_ENABLE
    Buzzer_Start(APP_INIT_BEEP_COUNT);
#endif
#endif
        /* 一次性虚拟角度自检，不驱动小车。 */
    if (!APP_ZONE2_NAV_ENABLE) App_HeadingDryRun();
#if APP_SERIAL_MOTION_CONTROL && !APP_ZONE2_NAV_ENABLE
    AppSerial_Init();
#endif
#if APP_HEADING_SESSION_TEST && APP_FORWARD_HOLD_TEST && !APP_ZONE2_NAV_ENABLE
    /* 只运行数学自检，失败禁止平移启动，默认不发动作。 */
    g_translation_hold_selftest = Chassis_TranslationHoldSelfTest() ? 1U : 2U;
#if APP_TEST_MODE == 3U
    AppMotion_LoadDefaultPlan();
#endif
#endif
    if (!APP_BASELINE40_ACTIVE && !APP_ZONE2_NAV_ENABLE) AppMotion_InitStage1Plan();
    while (1)
    {
        g_test_count_a++;
        
        /* 每100毫秒更新响停状态 */
#if APP_ZONE2_RADAR_CALIBRATION_TEST
        if (g_zone2_radar_calibration_requested && s_zone2_radar_test_beep_ms && !radar_test_announced)
        {
            radar_test_announced = 1U;
            Buzzer_StartPulse(s_zone2_radar_test_beep_ms);
        }
#endif
        Buzzer_Update();
        HWT101_Update();
        #if APP_HEADING_SESSION_TEST || (APP_HWT_STATIC_TEST && !APP_HWT_TURN_TEST)
        App_HeadingSourcePublish();
        #endif
        #if APP_HWT_STATIC_TEST && !APP_HWT_TURN_TEST
        App_HeadingLiveStep();
        #endif
        /* 一轮最多处理64字节；只有任务A交接并更新屏幕 */
        if (APP_ZONE2_MISSION_ENABLE || (!APP_BASELINE40_ACTIVE && !APP_ZONE2_NAV_ENABLE)) SAOMA_Update();
        if(APP_BASELINE40_ACTIVE || APP_ZONE2_NAV_ENABLE) { /* Dedicated chassis branch; no mission dispatch. */ }
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

        if (!APP_BASELINE40_ACTIVE && !APP_ZONE2_NAV_ENABLE) App_StartGateUpdate();
#if APP_SERIAL_MOTION_CONTROL && !APP_ZONE2_NAV_ENABLE
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
#if APP_ZONE2_MISSION_ENABLE
        AppZone2_DisplayTaskCode();
#endif
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
#elif APP_SERIAL_MOTION_CONTROL && !APP_ZONE2_NAV_ENABLE
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

/* 校准测试由PC13触发，只校准、扫描一次。 */
#if !APP_BASELINE40_ACTIVE
#if APP_ZONE1_RADAR_LIFT_TEST
volatile uint32_t g_radar_zone1_link_ready;
#endif
static void App_RadarTask(void *argument)
{
#if APP_ZONE1_RADAR_LIFT_TEST
    (void)argument;
    for (;;) {
        g_radar_zone1_link_ready=RadarObserver_PollZone1Link();
        vTaskDelay(pdMS_TO_TICKS(2U));
    }
#else
#if !APP_ZONE2_NAV_ENABLE || APP_ZONE2_RADAR_CALIBRATION_TEST
    uint32_t calibration_attempted = 0U;
    uint32_t scan_attempted = 0U;
    RadarObserverStatus radar_status;
#endif
    /* 用户2026-11-11临时授权measured=1，仅供校准和路径观察；
     * 自动扫描下行和回升均用4000半步；等待时间为工程选择，非实测。 */
    RadarZone2Config cfg = {
        RADAR_ZONE2_NODE_ID, 1U, RADAR_ZONE_LIFT_STEPS, 1500U, 500U, 1000U, 60000U, 1U, 2U
    };

    (void)argument;
#if APP_ZONE2_RADAR_CALIBRATION_TEST
    g_can_test_result = CAN1_LoopbackTest();
#endif
#if APP_ZONE2_NAV_ENABLE && !APP_ZONE2_RADAR_CALIBRATION_TEST && APP_ZONE1_RADAR_LIFT_CONFIRMED && !APP_ZONE1_RADAR_SERVO_TEST
    while (!g_radar_zone_ready) {
        RadarObserver_Service();
        vTaskDelay(pdMS_TO_TICKS(2U));
    }
    cfg.zone=(uint8_t)g_radar_zone_selected;
    cfg.node_id=RADAR_NODE_FOR_ZONE(cfg.zone);
    if (cfg.zone==1U) cfg.down_direction=RADAR_ZONE1_DOWN_DIRECTION;
#endif
    g_radar_app_config_result = RadarObserver_Configure(&cfg) ? 1U : 2U;
#if APP_ZONE2_RADAR_CALIBRATION_TEST
    if (g_radar_app_config_result == 2U) s_zone2_radar_test_beep_ms = 3000U;
#endif
    for (;;)
    {
        if (!g_radar_zone_locked && g_radar_zone_ready &&
            (g_radar_zone_selected==1U || g_radar_zone_selected==2U) &&
            cfg.zone!=(uint8_t)g_radar_zone_selected) {
            RadarZone2Config next_cfg=cfg;
            next_cfg.zone=(uint8_t)g_radar_zone_selected;
            next_cfg.node_id=RADAR_NODE_FOR_ZONE(next_cfg.zone);
            next_cfg.down_direction=next_cfg.zone==1U?RADAR_ZONE1_DOWN_DIRECTION:1U;
            if (RadarObserver_Configure(&next_cfg)) cfg=next_cfg;
        }        if (g_radar_app_config_result == 1U)
        {
            RadarObserver_Service();
#if !APP_ZONE2_NAV_ENABLE || APP_ZONE2_RADAR_CALIBRATION_TEST
            RadarObserver_GetStatus(&radar_status);
            if (radar_status.stage != 99U && radar_status.error == 0U)
            {
                if (calibration_attempted == 0U &&
#if APP_ZONE2_RADAR_CALIBRATION_TEST
                    g_zone2_radar_calibration_requested && g_safety_ready && !g_rtos_fault &&
                    RadarObserver_IsScanReady())
#else
                    RadarClient_IsReady())
#endif
                {
                    /* 仅入队一次；本次人工确认顶部、180度收拢起点。 */
                    calibration_attempted = 1U;
#if APP_ZONE2_RADAR_CALIBRATION_TEST
                    g_radar_app_calibration_result =
                        (RadarServo_Init() && RadarCalibration_StartZone2()) ? 1U : 2U;
#else
                    /* 默认沿用已保存校准；恢复本行则等待本轮校准成功。 */
                    /* g_radar_app_calibration_result = RadarCalibration_StartZone2() ? 1U : 2U; */
#endif
                }
                else if (calibration_attempted != 0U && scan_attempted == 0U &&
#if APP_ZONE2_RADAR_CALIBRATION_TEST
                         RadarObserver_IsScanReady() &&
#endif
                         (
#if !APP_ZONE2_RADAR_CALIBRATION_TEST
                          g_radar_app_calibration_result == 0U ||
#endif
                          (g_radar_app_calibration_result == 1U &&
                           radar_status.stage == 8U && radar_status.calibration_complete == 1U)))
                {
                    scan_attempted = 1U;
#if APP_ZONE2_RADAR_CALIBRATION_TEST
                    g_radar_app_scan_result = RadarObserver_StartScan(ZONE2_RADAR_TEST_RUN_ID, 2U, VEHICLE_MAP_VERSION) ? 1U : 2U;
#else
                    g_radar_app_scan_result = RadarObserver_StartZone2(0, 24, true) ? 1U : 2U;
#endif
                }
            }
#if APP_ZONE2_RADAR_CALIBRATION_TEST
            if (!s_zone2_radar_test_beep_ms)
            {
                if (g_radar_app_calibration_result == 2U || g_radar_app_scan_result == 2U ||
                    (radar_status.stage == 99U && radar_status.error && radar_status.recovered))
                    s_zone2_radar_test_beep_ms = 3000U;
                else if (g_radar_app_scan_result == 1U && !radar_status.error &&
                         radar_status.stage == 8U && radar_status.map_valid)
                {
                    RadarObserverMap map;
                    if (RadarObserver_GetMap(ZONE2_RADAR_TEST_RUN_ID, &map)) s_zone2_radar_test_beep_ms = APP_BEEP_ON_MS;
                }
            }
#endif
#endif
        }
        vTaskDelay(pdMS_TO_TICKS(2U));
    }
#endif
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
        AppFusionCapture_Step(&snapshot);
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
#if APP_ZONE2_MISSION_ENABLE
    USART6_VisionConfig();
#endif
    /* 调度前初始化启动按键 */
    /* UART4仅初始化，实际发送放到任务A */
    Screen_Init();
    Key_Init();
    /* 初始化PE2，默认关闭VM允许请求 */
    Power_Init();
#if !APP_BASELINE40_ACTIVE
    RadarZoneSwitch_Init();
#if !APP_ZONE2_RADAR_CALIBRATION_TEST
    (void)RadarServo_Init();
#endif
    if (!APP_ZONE2_NAV_ENABLE || APP_ZONE2_MISSION_ENABLE) StartupPose_Init();
#endif
    Safety_Init();
    if (APP_ZONE2_MISSION_ENABLE || (!APP_BASELINE40_ACTIVE && !APP_ZONE2_NAV_ENABLE)) SAOMA_Init();
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
    
    
    
#if !APP_ZONE2_RADAR_CALIBRATION_TEST
    /* CAN栈384字，优先级3；监督仍为4 */
    g_can_task_create = xTaskCreate(App_CanTask, "CanPoll",
                                      (APP_ZONE2_NAV_MOTION_ENABLE ? 1024U : 384U), NULL, 3U, NULL);
    configASSERT(g_can_task_create == pdPASS);
#endif
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
#endif /* APP_VM_CALIBRATION_ONLY_ENABLE */
