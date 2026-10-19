#include "stm32f4xx.h"
#include "FreeRTOS.h"
#include "task.h"
#include "freertos_hooks.h"
#include "led.h"
#include "Buzzer.h"
#include "app_config.h"
#include "usart.h"
#include "Key.h"
#include "power.h"
#include "can.h"
#include "safety.h"
#include "watchdog.h"
#include "screen.h"
#include "saoma.h"
#include <stdio.h>

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
/* 启动按键事件数、任务创建结果及栈余量 */
volatile uint32_t g_start_key_count = 0U;
volatile BaseType_t g_key_task_create = pdFAIL;
volatile UBaseType_t g_key_stack_words = 0U;
/* CAN1回环结果：0未完成，1通过 */
volatile uint32_t g_can_test_result = 0U;
/* 查询地址：1左后、2右后、3左前、4右前 */
volatile uint32_t g_can_version_address = 1U;
/* 实时位置只读，不清零；原始幅值与符号分别保留 */
volatile uint32_t g_can_position_address = 1U;
volatile uint32_t g_can_position_request = 0U;
volatile uint32_t g_can_position_result = 0U;
volatile uint32_t g_can_position_sign = 0U;
volatile uint32_t g_can_position_raw = 0U;
volatile uint32_t g_can_position_esr = 0U;
CanRxMsg g_can_position_reply;


/* 只读转速请求：地址1左后、2右后、3左前、4右前 */
volatile uint32_t g_can_speed_address = 1U;
volatile uint32_t g_can_speed_request = 0U;
volatile uint32_t g_can_speed_result = 0U;
volatile int32_t g_can_speed_rpm = 0;
volatile uint32_t g_can_speed_esr = 0U;
CanRxMsg g_can_speed_reply;
/* 版本读取仅由Watch手动触发一次 */
volatile uint32_t g_can_version_request = 0U;
volatile uint32_t g_can_version_result = 0U;
volatile uint32_t g_can_version_esr = 0U;
CanRxMsg g_can_version_reply;
/* 状态查询沿用g_can_version_address选择目标 */
volatile uint32_t g_can_status_request = 0U;
volatile uint32_t g_can_status_result = 0U;
volatile uint32_t g_can_status_esr = 0U;
volatile uint32_t g_can_status_flags = 0U;
volatile TickType_t g_can_status_tick = 0U;
CanRxMsg g_can_status_reply;
/* 默认开启必要采集；VM关闭或平台故障时仍不发送查询 */
volatile uint32_t g_can_poll_enable = 1U;
/* 数组下标0/1/2/3对应地址1/2/3/4 */
volatile uint32_t g_can_poll_result[4] = {0U};
volatile uint32_t g_can_poll_flags[4] = {0U};
volatile TickType_t g_can_poll_tick[4] = {0U};
volatile uint32_t g_can_poll_count[4] = {0U};
volatile uint32_t g_can_poll_age_ticks[4] = {0U};
volatile uint32_t g_can_poll_valid_mask = 0U;
volatile uint32_t g_can_poll_protect_mask = 0U;
volatile uint32_t g_can_poll_esr = 0U;
CanRxMsg g_can_poll_reply;
/* 看门狗初始化结果，成功后才启动应用任务 */
volatile uint32_t g_watchdog_init_result = 0U;
/* 本步只显示实际扫码结果，不自动显示固定任务码 */
volatile uint32_t g_scan_display_result = 0U;
volatile uint32_t g_scan_display_count = 0U;
volatile uint32_t g_scan_log_send_ok = 0U;
/* 当前已接模块的启动请求门禁，尚未驱动底盘 */
volatile uint32_t g_start_gate_ready = 0U;
volatile uint32_t g_start_request = 0U;
volatile uint32_t g_start_accept_count = 0U;
volatile uint32_t g_start_deny_count = 0U;
char g_start_task_code[16] = {0};
static uint32_t s_start_gate_revision = 0U;
static uint32_t s_start_pending = 0U;
static uint32_t s_start_pending_revision = 0U;

static uint32_t s_can_poll_index = 0U;
static uint32_t s_can_poll_active = 0U;
/* 本步100ms查一台，名义四台约400ms，候选新鲜度1秒 */
#define CAN_POLL_VALID_MS 1000U
/* 台架持续反馈：默认0，用户静态供电健康后设1 */
volatile uint32_t g_can_feedback_enable = 0U;
volatile uint32_t g_can_feedback_speed_result[4] = {0U};
volatile int32_t g_can_feedback_speed_rpm[4] = {0};
volatile uint32_t g_can_feedback_position_result[4] = {0U};
volatile uint32_t g_can_feedback_position_sign[4] = {0U};
volatile uint32_t g_can_feedback_position_raw[4] = {0U};
volatile TickType_t g_can_feedback_speed_tick[4] = {0U};
volatile TickType_t g_can_feedback_position_tick[4] = {0U};
volatile uint32_t g_can_feedback_speed_age_ticks[4] = {
    0xFFFFFFFFU, 0xFFFFFFFFU, 0xFFFFFFFFU, 0xFFFFFFFFU};
volatile uint32_t g_can_feedback_position_age_ticks[4] = {
    0xFFFFFFFFU, 0xFFFFFFFFU, 0xFFFFFFFFU, 0xFFFFFFFFU};
volatile uint32_t g_can_feedback_count[4] = {0U};
volatile uint32_t g_can_feedback_speed_valid_mask = 0U;
volatile uint32_t g_can_feedback_position_valid_mask = 0U;
volatile uint32_t g_can_feedback_error_count = 0U;
static uint32_t s_feedback_phase = 0U;
static uint32_t s_feedback_index = 0U;
/* 当前静态候选阈值，运动控制前重新验证更快的时效 */
#define CAN_FEEDBACK_VALID_MS 1000U
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

/* 两个任务共用调试发送缓冲区，短临界区内只复制，不等待串口 */
static uint8_t App_DebugSendString(const char *text)
{
    uint8_t result;
    taskENTER_CRITICAL();
    result = USART1_SendString(text);
    taskEXIT_CRITICAL();
    return result;
}
/* 每轮只采一台，保持成功计数/时间戳；关闭时清除本会话快照 */
static void App_CanPollStep(void)
{
    uint32_t i;
    uint32_t index;
    uint32_t result;

    if ((g_can_poll_enable != 1U) ||
        (g_vm_test_request != 1U) || (g_safety_ready != 1U))
    {
        CAN1_ResetSession();
        s_can_poll_active = 0U;
        s_can_poll_index = 0U;
        g_can_poll_valid_mask = 0U;
        g_can_poll_protect_mask = 0U;
        for (i = 0U; i < 4U; i++)
        {
            g_can_poll_result[i] = 0U;
            g_can_poll_flags[i] = 0U;
            g_can_poll_tick[i] = 0U;
            g_can_poll_count[i] = 0U;
            g_can_poll_age_ticks[i] = 0xFFFFFFFFU;
        }
        return;
    }

    if (s_can_poll_active == 0U)
    {
        CAN1_ResetSession();
        s_can_poll_index = 0U;
        s_can_poll_active = 1U;
    }

    index = s_can_poll_index;
    result = CAN1_ReadStatus((uint8_t)(index + 1U),
                            &g_can_poll_reply);
    g_can_poll_result[index] = result;
    g_can_poll_esr = CAN1->ESR;
    if (result == 1U)
    {
        g_can_poll_flags[index] = g_can_poll_reply.Data[1];
        g_can_poll_tick[index] = xTaskGetTickCount();
        g_can_poll_count[index]++;
    }
    else
    {
        /* 本节点失败后撤销其快照，不能继续使用旧状态 */
        g_can_poll_flags[index] = 0U;
        g_can_poll_tick[index] = 0U;
    }
    s_can_poll_index = (index + 1U) % 4U;
}

/* 每轮都计算新鲜度，手动查询占用总线时也不能冻结旧有效标志 */
static void App_CanPollRefresh(void)
{
    TickType_t now = xTaskGetTickCount();
    TickType_t age;
    uint32_t i;
    uint32_t valid = 0U;
    uint32_t protect = 0U;

    for (i = 0U; i < 4U; i++)
    {
        g_can_poll_age_ticks[i] = 0xFFFFFFFFU;
        if ((g_can_poll_enable == 1U) &&
            (g_vm_test_request == 1U) && (g_safety_ready == 1U) &&
            (g_can_poll_result[i] == 1U) &&
            (s_can_poll_active == 1U))
        {
            /* 无符号相减，允许tick正常回绕 */
            age = (TickType_t)(now - g_can_poll_tick[i]);
            g_can_poll_age_ticks[i] = (uint32_t)age;
            if (age <= pdMS_TO_TICKS(CAN_POLL_VALID_MS))
            {
                valid |= 1UL << i;
                if ((g_can_poll_flags[i] & 0x0CU) != 0U)
                {
                    protect |= 1UL << i;
                }
            }
        }
    }
    g_can_poll_valid_mask = valid;
    g_can_poll_protect_mask = protect;
}

/* 关闭反馈时撤销有效性，原状态保护轮询继续 */
static void App_CanFeedbackClear(void)
{
    uint32_t i;
    s_feedback_phase = 0U;
    s_feedback_index = 0U;
    g_can_feedback_speed_valid_mask = 0U;
    g_can_feedback_position_valid_mask = 0U;
    g_can_feedback_error_count = 0U;
    for (i = 0U; i < 4U; i++)
    {
        g_can_feedback_speed_result[i] = 0U;
        g_can_feedback_position_result[i] = 0U;
        g_can_feedback_speed_tick[i] = 0U;
        g_can_feedback_position_tick[i] = 0U;
        g_can_feedback_speed_age_ticks[i] = 0xFFFFFFFFU;
        g_can_feedback_position_age_ticks[i] = 0xFFFFFFFFU;
        g_can_feedback_count[i] = 0U;
    }
}

/* 只由任务A调用，一轮一个请求：状态、转速、位置再换节点 */
static void App_CanFeedbackStep(void)
{
    CanRxMsg reply;
    uint32_t result;
    int32_t rpm = 0;
    uint8_t sign = 0U;
    uint32_t raw = 0U;
    uint32_t index;

    if (g_can_feedback_enable != 1U || g_can_poll_enable != 1U ||
        g_vm_test_request != 1U || g_safety_ready != 1U)
    {
        App_CanFeedbackClear();
        App_CanPollStep();
        return;
    }
    if (s_feedback_phase == 0U)
    {
        /* 原状态函数首次启动会从节点0开始 */
        s_feedback_index = (s_can_poll_active == 0U) ? 0U : s_can_poll_index;
        App_CanPollStep();
        if (g_can_poll_result[s_feedback_index] != 1U)
        {
            g_can_feedback_error_count++;
        }
        s_feedback_phase = 1U;
        return;
    }
    index = s_feedback_index;
    if (s_feedback_phase == 1U)
    {
        result = CAN1_ReadSpeed((uint8_t)(index + 1U), &reply, &rpm);
        g_can_feedback_speed_result[index] = result;
        if (result == 1U)
        {
            g_can_feedback_speed_rpm[index] = rpm;
            g_can_feedback_speed_tick[index] = xTaskGetTickCount();
        }
        else
        {
            g_can_feedback_speed_tick[index] = 0U;
            g_can_feedback_error_count++;
        }
        s_feedback_phase = 2U;
    }
    else
    {
        result = CAN1_ReadPosition((uint8_t)(index + 1U), &reply, &sign, &raw);
        g_can_feedback_position_result[index] = result;
        if (result == 1U)
        {
            g_can_feedback_position_sign[index] = sign;
            g_can_feedback_position_raw[index] = raw;
            g_can_feedback_position_tick[index] = xTaskGetTickCount();
            if (g_can_poll_result[index] == 1U &&
                g_can_feedback_speed_result[index] == 1U)
            {
                g_can_feedback_count[index]++;
            }
        }
        else
        {
            g_can_feedback_position_tick[index] = 0U;
            g_can_feedback_error_count++;
        }
        s_feedback_phase = 0U;
    }
}

/* 手动查询占用总线时，依然每轮计算反馈年龄并撤销过期数据 */
static void App_CanFeedbackRefresh(void)
{
    TickType_t now = xTaskGetTickCount();
    TickType_t age;
    uint32_t i;
    uint32_t speed_mask = 0U;
    uint32_t position_mask = 0U;
    for (i = 0U; i < 4U; i++)
    {
        g_can_feedback_speed_age_ticks[i] = 0xFFFFFFFFU;
        g_can_feedback_position_age_ticks[i] = 0xFFFFFFFFU;
        if (g_can_feedback_enable == 1U && g_can_poll_enable == 1U &&
            g_vm_test_request == 1U && g_safety_ready == 1U)
        {
            if (g_can_feedback_speed_result[i] == 1U)
            {
                age = (TickType_t)(now - g_can_feedback_speed_tick[i]);
                g_can_feedback_speed_age_ticks[i] = (uint32_t)age;
                if (age <= pdMS_TO_TICKS(CAN_FEEDBACK_VALID_MS))
                    speed_mask |= 1UL << i;
            }
            if (g_can_feedback_position_result[i] == 1U)
            {
                age = (TickType_t)(now - g_can_feedback_position_tick[i]);
                g_can_feedback_position_age_ticks[i] = (uint32_t)age;
                if (age <= pdMS_TO_TICKS(CAN_FEEDBACK_VALID_MS))
                    position_mask |= 1UL << i;
            }
        }
    }
    g_can_feedback_speed_valid_mask = speed_mask;
    g_can_feedback_position_valid_mask = position_mask;
}


/* 只有任务A调用；先检查当前平台，再确认按键对应的扫码版本 */
static void App_StartGateUpdate(void)
{
    uint32_t platform_ok;
    uint32_t code_ok;
    uint32_t accepted = 0U;
    uint32_t i;
    char log[24];

    taskENTER_CRITICAL();
    platform_ok = (g_rtos_fault == 0U && g_safety_fault == 0U &&
                   g_safety_ready == 1U) ? 1U : 0U;
    if (platform_ok == 0U)
    {
        g_start_request = 0U;
    }
    code_ok = (g_saoma_last_result == 1U &&
               g_saoma_accept_count != 0U &&
               g_scan_display_result == SCREEN_OK) ? 1U : 0U;
    g_start_gate_ready = (platform_ok != 0U && code_ok != 0U &&
                          g_start_request == 0U) ? 1U : 0U;
    s_start_gate_revision = g_saoma_accept_count;
    if (s_start_pending != 0U)
    {
        s_start_pending = 0U;
        if (g_start_gate_ready != 0U &&
            s_start_pending_revision == g_saoma_accept_count)
        {
            for (i = 0U; i < sizeof(g_start_task_code); i++)
            {
                g_start_task_code[i] = SAOMA_Data[i];
            }
            g_start_request = 1U;
            g_start_gate_ready = 0U;
            g_start_accept_count++;
            accepted = 1U;
        }
        else
        {
            g_start_deny_count++;
        }
    }
    taskEXIT_CRITICAL();
    if (accepted != 0U)
    {
        sprintf(log, "START:%s\r\n", g_start_task_code);
        (void)App_DebugSendString(log);
    }
}
/* CAN1只由本任务初始化和查询；每轮交易结束后至少等待50毫秒 */
static void App_CanTask(void *argument)
{
    (void)argument;
    g_can_test_result = CAN1_LoopbackTest();
    while (1)
    {
        g_can_task_count++;
        if (g_can_test_result == 1U)
        {
                    /* 同一轮只执行一个查询，位置请求优先于转速/版本/状态 */
        if ((g_can_position_request == 1U) && (g_vm_test_request == 1U) &&
            (g_safety_ready == 1U) && (g_can_guard_ready == 1U))
        {
            uint32_t address = g_can_position_address;
            uint8_t sign = 0U;
            uint32_t raw = 0U;
            char log[64];
            g_can_position_request = 0U;
            g_can_position_result = 0U;
            g_can_position_sign = 0U;
            g_can_position_raw = 0U;
            if (address >= 1U && address <= 4U)
            {
                g_can_position_result = CAN1_ReadPosition(
                    (uint8_t)address, &g_can_position_reply, &sign, &raw);
            }
            else
            {
                g_can_position_result = 7U;
            }
            g_can_position_esr = CAN1->ESR;
            if (g_can_position_result == 1U)
            {
                g_can_position_sign = sign;
                g_can_position_raw = raw;
            }
            /* 结果非1时SIGN/RAW字段只是无效占位 */
            sprintf(log, "POS:A%lu R%lu SIGN%lu RAW%lu\r\n",
                    (unsigned long)address,
                    (unsigned long)g_can_position_result,
                    (unsigned long)g_can_position_sign,
                    (unsigned long)g_can_position_raw);
            (void)App_DebugSendString(log);
        }
        /* 已有四台基本状态就绪后，每轮只发一个只读转速请求 */
        else if ((g_can_speed_request == 1U) && (g_vm_test_request == 1U) &&
            (g_safety_ready == 1U) && (g_can_guard_ready == 1U))
        {
            uint32_t address = g_can_speed_address;
            int32_t rpm = 0;
            char log[48];
            g_can_speed_request = 0U;
            g_can_speed_result = 0U;
            g_can_speed_rpm = 0;
            if (address >= 1U && address <= 4U)
            {
                g_can_speed_result = CAN1_ReadSpeed((uint8_t)address,
                                                  &g_can_speed_reply, &rpm);
            }
            else
            {
                g_can_speed_result = 7U;
            }
            g_can_speed_esr = CAN1->ESR;
            if (g_can_speed_result == 1U) g_can_speed_rpm = rpm;
            /* 结果非1时RPM字段不作为有效反馈 */
            sprintf(log, "SPEED:A%lu R%lu RPM%ld\r\n",
                    (unsigned long)address,
                    (unsigned long)g_can_speed_result,
                    (long)g_can_speed_rpm);
            (void)App_DebugSendString(log);
        }
        /* VM已请求开启后，读取指定电机版本 */
        else if ((g_can_version_request == 1U) &&
            (g_vm_test_request == 1U))
        {
            uint32_t address = g_can_version_address;

            g_can_version_request = 0U;
            g_can_version_result = 0U;

            if ((address >= 1U) && (address <= 4U))
            {
                g_can_version_result =
                    CAN1_ReadVersion((uint8_t)address,
                                     &g_can_version_reply);
            }
            else
            {
                /* 地址无效，本次不发送 */
                g_can_version_result = 7U;
            }

            g_can_version_esr = CAN1->ESR;
        }
        /* VM请求开启后，手动读取一次电机状态 */
        else if ((g_can_status_request == 1U) &&
            (g_vm_test_request == 1U))
        {
            uint32_t address = g_can_version_address;

            g_can_status_request = 0U;
            g_can_status_result = 0U;
            g_can_status_flags = 0U;
            g_can_status_tick = 0U;

            if ((address >= 1U) && (address <= 4U))
            {
                g_can_status_result =
                    CAN1_ReadStatus((uint8_t)address,
                                    &g_can_status_reply);
            }
            else
            {
                g_can_status_result = 7U;
            }

            g_can_status_esr = CAN1->ESR;

            /* 只在查询成功时保存状态字和采样时间 */
            if (g_can_status_result == 1U)
            {
                g_can_status_flags = g_can_status_reply.Data[1];
                g_can_status_tick = xTaskGetTickCount();
            }
        }
            else
            {
                /* 无手动请求时，按地址1到4循环采状态 */
                 App_CanFeedbackStep();
            }
        App_CanPollRefresh();
        App_CanFeedbackRefresh();
        /* 临时故障注入，正式监督与正常轮询仍继续运行 */
        Safety_CanReport(g_can_poll_enable, g_can_poll_valid_mask,
        g_can_poll_protect_mask, g_can_poll_tick);
        }
        else
        {
            /* 回环失败时不发电机查询，监督沿用原启动故障判定 */
            Safety_CanReport(0U, 0U, 0U, g_can_poll_tick);
        }
        g_can_task_stack_words = uxTaskGetStackHighWaterMark(NULL);
        vTaskDelay(pdMS_TO_TICKS(50U));
    }
}

/* 任务A：每100毫秒计数一次 */
static void App_TestTaskA(void *argument)
{
    TickType_t screen_start_tick = xTaskGetTickCount();
    (void)argument;
    /* FreeRTOS运行后才开启接收中断 */
    USART1_EnableReceive();
    /* 启动时只请求发送一次基础平台就绪信息 */
    SAOMA_EnableReceive();
    g_serial_send_ok = App_DebugSendString("F407 FreeRTOS READY\r\n");
    /* 已验收的基础平台就绪后，只请求一次提示音 */
    Buzzer_Start(APP_INIT_BEEP_COUNT);
    
    while (1)
    {
        g_test_count_a++;
        
        /* 每100毫秒更新响停状态 */
        Buzzer_Update();
        /* 一轮最多处理64字节；只有任务A交接并更新屏幕 */
        SAOMA_Update();
        /* 确认启动请求后，保留本轮码和屏幕，不替换任务对象 */
        if (g_start_request != 0U)
        {
            SAOMA_DataReady = 0U;
        }
        if (SAOMA_DataReady != 0U &&
            (TickType_t)(xTaskGetTickCount() - screen_start_tick) >=
            pdMS_TO_TICKS(1000U))
        {
            char scan_log[24];
            SAOMA_DataReady = 0U;
            g_scan_display_result = Screen_ShowTaskCode(SAOMA_Data);
            if (g_scan_display_result == SCREEN_OK)
            {
                g_scan_display_count++;
            }
            /* 诊断发送接受不等于PC已经收到；忙时返回0 */
            sprintf(scan_log, "SCAN:%s\r\n", SAOMA_Data);
            g_scan_log_send_ok = App_DebugSendString(scan_log);
        }
        App_StartGateUpdate();
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
        }
        else
        {
            /* 空闲时记录历史最小剩余栈，单位为字 */
            g_rx_stack_words = uxTaskGetStackHighWaterMark(NULL);
        }
        /* 有数据或正常空闲等待返回，均表示任务仍在运行 */
        Safety_Report(SAFETY_TASK_RX);
    }
}

/* 每1毫秒扫描，只记录启动按键事件 */
static void App_KeyTask(void *argument)
{
    uint32_t diagnostic_count = 0U;

    (void)argument;

    while (1)
    {
        if (Key_GetEvent() != 0U)
        {
            g_start_key_count++;
            /* 未就绪的按键当场拒绝，不留到扫码后自动启动 */
            taskENTER_CRITICAL();
            if (g_start_gate_ready != 0U && g_start_request == 0U)
            {
                s_start_pending_revision = s_start_gate_revision;
                s_start_pending = 1U;
            }
            else
            {
                g_start_deny_count++;
            }
            taskEXIT_CRITICAL();
        }

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
        Safety_Update(g_vm_test_request, g_can_test_result);
        g_safety_stack_words = uxTaskGetStackHighWaterMark(NULL);
        vTaskDelay(pdMS_TO_TICKS(20U));
    }
}

int main(void)
{
    /* 更新时钟，将4位优先级全部用于抢占优先级 */
    SystemCoreClockUpdate();
    NVIC_PriorityGroupConfig(NVIC_PriorityGroup_4);
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
    Safety_Init();
    SAOMA_Init();
    g_watchdog_init_result = Watchdog_Init();
    configASSERT(g_watchdog_init_result == 1U);
    
    /* 每个任务栈为256个字，即1024字节 */
    g_test_create_a = xTaskCreate(App_TestTaskA, "TestA",
                                 256U, NULL, 2U, NULL);
    configASSERT(g_test_create_a == pdPASS);
    g_test_create_b = xTaskCreate(App_TestTaskB, "TestB",
                                 256U, NULL, 1U, NULL);
    configASSERT(g_test_create_b == pdPASS);
    /* 本轮接收任务栈256字，优先级3 */
    g_rx_task_create = xTaskCreate(App_UartRxTask, "UartRx",
                                   256U, NULL, 3U, NULL);
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
    vTaskStartScheduler();

    /* 调度器异常返回时记录并停止 */
    g_test_scheduler_returned = 1U;
    configASSERT(0);

    while (1)
    {
    }
}
