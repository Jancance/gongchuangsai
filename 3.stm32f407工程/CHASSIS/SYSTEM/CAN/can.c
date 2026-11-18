#include "can.h"
#include "stm32f4xx_can.h"
#include "FreeRTOS.h"
#include "task.h"
#include <string.h>
#include "app_config.h"
#include "startup_axes.h"
#include "power.h"
#include "safety.h"

/* 新目标航向模式不允许单次交易长于指令提交监督期限。 */
#if APP_HEADING_SESSION_TEST
#define CAN1_SINGLE_TX_WAIT_MS 20U
#define CAN1_SINGLE_REPLY_WAIT_MS 30U
#else
#define CAN1_SINGLE_TX_WAIT_MS 100U
#define CAN1_SINGLE_REPLY_WAIT_MS 200U
#endif
/* 首次控制交易失败快照；仅读取硬件状态，不改帧或等待期限。 */
volatile uint32_t g_can_ctrl_fault_latched = 0U;
volatile uint32_t g_can_ctrl_trace_result = 0U, g_can_ctrl_trace_phase = 0U;
volatile uint32_t g_can_ctrl_trace_address = 0U, g_can_ctrl_trace_length = 0U;
volatile uint8_t g_can_ctrl_trace_request[8] = {0U};
volatile uint32_t g_can_ctrl_trace_pe2_start = 0U, g_can_ctrl_trace_pe2_end = 0U;
volatile uint32_t g_can_ctrl_trace_esr = 0U;
volatile uint32_t g_can_ctrl_trace_rx_count = 0U, g_can_ctrl_trace_flush_count = 0U;
volatile uint32_t g_can_ctrl_trace_tx_ms = 0U, g_can_ctrl_trace_total_ms = 0U;
volatile CanRxMsg g_can_ctrl_trace_last_rx = {0};
/* 与原首次失败诊断共用锁存，失败后不覆盖。 */
typedef struct
{
    uint32_t captured;
    uint32_t command_tick;
    uint32_t vm_command_active;
    uint32_t vm_enable_seen;
    uint32_t vm_enable_count;
    uint32_t vm_enable_age_ms;
    uint32_t guard_ready;
    uint32_t poll_result[4];
    uint32_t poll_flags[4];
    uint32_t poll_count[4];
    uint32_t poll_age_ms[4];
} CanControlTraceContext;
volatile CanControlTraceContext g_can_ctrl_trace_context = {0};
extern volatile uint32_t g_can_poll_result[4];
extern volatile uint32_t g_can_poll_flags[4];
extern volatile uint32_t g_can_poll_count[4];
extern volatile TickType_t g_can_poll_tick[4];

static uint32_t s_can_ctrl_trace_active = 0U;
static TickType_t s_can_ctrl_trace_tick = 0U;

/* 只读源码确认项，不写驱动参数；前端据此在申请VM之前拒绝启动。 */
uint32_t CAN1_EmmSettingsConfirmed(void)
{ return ZDT_EMM_SETTINGS_CONFIRMED && !APP_MOTOR_OFFLINE_ONLY; }

/* 当前所有CAN调用都来自任务A，不允许其他任务并发使用 */
static void StartupHome_Capture(const CanRxMsg *p)
{
    if(g_startup_extension_home_active && p->IDE==CAN_Id_Extended && p->RTR==CAN_RTR_Data &&
       p->ExtId==0x600U && p->DLC==3U && p->Data[0]==0x9aU && p->Data[1]==0x9fU && p->Data[2]==0x6bU)
        g_startup_extension_home_notice=1U;
}
static uint32_t s_can1_normal_ready = 0U;
/* 标记下一次查询需重新初始化，不在这里发送或切换硬件模式 */
void CAN1_ResetSession(void)
{
    s_can1_normal_ready = 0U;
}
/* 成功返回1，时钟不匹配返回2，初始化失败返回3 */
static uint32_t CAN1_InitMode(uint8_t mode)
{
    TickType_t init_start_tick;
    RCC_ClocksTypeDef clocks;
    GPIO_InitTypeDef gpio;
    CAN_InitTypeDef can;
    CAN_FilterInitTypeDef filter;

    RCC_GetClocksFreq(&clocks);
    if (clocks.PCLK1_Frequency != 42000000U)
    {
        return 2U;
    }

    RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOD, ENABLE);
    RCC_APB1PeriphClockCmd(RCC_APB1Periph_CAN1, ENABLE);
    CAN_DeInit(CAN1);

    /* 成品PCB：PD0接收，PD1发送 */
    GPIO_PinAFConfig(GPIOD, GPIO_PinSource0, GPIO_AF_CAN1);
    GPIO_PinAFConfig(GPIOD, GPIO_PinSource1, GPIO_AF_CAN1);

    GPIO_StructInit(&gpio);
    gpio.GPIO_Pin = GPIO_Pin_0 | GPIO_Pin_1;
    gpio.GPIO_Mode = GPIO_Mode_AF;
    gpio.GPIO_OType = GPIO_OType_PP;
    gpio.GPIO_Speed = GPIO_Speed_25MHz;
    gpio.GPIO_PuPd = GPIO_PuPd_NOPULL;
    GPIO_Init(GPIOD, &gpio);

    CAN_StructInit(&can);

    /* 由调用者选择回环或正常通信模式 */
    can.CAN_Mode = mode;
    can.CAN_SJW = CAN_SJW_1tq;
    can.CAN_BS1 = CAN_BS1_11tq;
    can.CAN_BS2 = CAN_BS2_2tq;

    /* 42MHz / 6 / (1 + 11 + 2) = 500kbit/s */
    can.CAN_Prescaler = 6U;
    can.CAN_NART = ENABLE;

    if (CAN_Init(CAN1, &can) != CAN_InitStatus_Success)
    {
        /* 进入初始化未成功，保留失败 */
        if ((CAN1->MCR & CAN_MCR_INRQ) != 0U)
        {
            return 3U;
        }

        /* 已请求退出，等待硬件同步完成，最多100毫秒 */
        init_start_tick = xTaskGetTickCount();

        while ((CAN1->MSR & CAN_MSR_INAK) != 0U)
        {
            if ((TickType_t)(xTaskGetTickCount() - init_start_tick) >=
                pdMS_TO_TICKS(100U))
            {
                return 3U;
            }

            vTaskDelay(pdMS_TO_TICKS(1U));
        }
    }
    /* 过滤器0接收全部帧，送入FIFO0 */
    filter.CAN_FilterNumber = 0U;
    filter.CAN_FilterMode = CAN_FilterMode_IdMask;
    filter.CAN_FilterScale = CAN_FilterScale_32bit;
    filter.CAN_FilterIdHigh = 0U;
    filter.CAN_FilterIdLow = 0U;
    filter.CAN_FilterMaskIdHigh = 0U;
    filter.CAN_FilterMaskIdLow = 0U;
    filter.CAN_FilterFIFOAssignment = CAN_FIFO0;
    filter.CAN_FilterActivation = ENABLE;
    CAN_FilterInit(&filter);

    return 1U;
}

/* 一次完整回环测试，不开启VM，不使用电机协议 */
uint32_t CAN1_LoopbackTest(void)
{
    CanTxMsg tx;
    CanRxMsg rx;
    TickType_t start_tick;
    uint32_t result;
    uint8_t mailbox;
    uint8_t status;
    uint8_t i;
    
    CAN1_ResetSession();
    result = CAN1_InitMode(CAN_Mode_Silent_LoopBack);
    if (result != 1U)
    {
        return result;
    }

    tx.StdId = 0U;
    tx.ExtId = 0x12345U;
    tx.IDE = CAN_Id_Extended;
    tx.RTR = CAN_RTR_Data;
    tx.DLC = 8U;

    for (i = 0U; i < 8U; i++)
    {
        tx.Data[i] = (uint8_t)(0x10U + i);
    }

    mailbox = CAN_Transmit(CAN1, &tx);
    if (mailbox == CAN_TxStatus_NoMailBox)
    {
        return 4U;
    }

    start_tick = xTaskGetTickCount();

    while (1)
    {
        status = CAN_TransmitStatus(CAN1, mailbox);

        if (status == CAN_TxStatus_Ok)
        {
            break;
        }

        if ((status == CAN_TxStatus_Failed) ||
            ((TickType_t)(xTaskGetTickCount() - start_tick) >=
             pdMS_TO_TICKS(100U)))
        {
            CAN_CancelTransmit(CAN1, mailbox);
            return 4U;
        }

        vTaskDelay(pdMS_TO_TICKS(1U));
    }

    start_tick = xTaskGetTickCount();

    while (CAN_MessagePending(CAN1, CAN_FIFO0) == 0U)
    {
        if ((TickType_t)(xTaskGetTickCount() - start_tick) >=
            pdMS_TO_TICKS(100U))
        {
            return 5U;
        }

        vTaskDelay(pdMS_TO_TICKS(1U));
    }

    CAN_Receive(CAN1, CAN_FIFO0, &rx);StartupHome_Capture(&rx);

    if ((rx.IDE != CAN_Id_Extended) ||
        (rx.RTR != CAN_RTR_Data) ||
        (rx.ExtId != tx.ExtId) ||
        (rx.DLC != tx.DLC))
    {
        return 6U;
    }

    for (i = 0U; i < 8U; i++)
    {
        if (rx.Data[i] != tx.Data[i])
        {
            return 6U;
        }
    }

    return 1U;
}

/* 只读版本：1成功，4发送失败，5无回复，6格式异常 */
/* 共用短帧查询，供版本和状态读取使用 */
/* 通用单帧收发：只允许独立CAN任务串行调用 */
static uint32_t CAN1_NowMs(void)
{
    return (uint32_t)xTaskGetTickCount() * portTICK_PERIOD_MS;
}

/* 清理FIFO后，在同一临界区检查剩余帧和绝对时限，再提交邮箱。 */
static uint32_t CAN1_SubmitWindow(CanTxMsg *tx, uint32_t slot_ms,
                                  uint8_t *mailbox, uint32_t *request_ms)
{
    uint32_t now_ms;
    taskENTER_CRITICAL();
    if (CAN_MessagePending(CAN1, CAN_FIFO0))
    {
        taskEXIT_CRITICAL();
        return CAN1_TIMING_WINDOW;
    }
    now_ms = CAN1_NowMs();
    if (now_ms-slot_ms > CAN1_NAV_ADMISSION_MS)
    {
        taskEXIT_CRITICAL();
        return CAN1_TIMING_WINDOW;
    }
    *mailbox = CAN_Transmit(CAN1, tx);
    taskEXIT_CRITICAL();
    if (*mailbox == CAN_TxStatus_NoMailBox) return 4U;
    *request_ms = now_ms;
    return 1U;
}

static uint32_t CAN1_ExchangeSingleWindow(uint8_t address,
                                    const uint8_t *request,
                                    uint8_t request_length,
                                    uint8_t reply_length,
                                    CanRxMsg *reply, uint32_t slot_ms,
                                    CAN1_EmmSpeedReceipt *receipt)
{
    CanTxMsg tx;
    CanRxMsg rx;
    TickType_t start_tick;
    uint32_t result;
    uint8_t mailbox;
    uint8_t status;
    
    uint8_t flush_count;
    /* 请求和回复均限制在单个CAN帧内 */
    if ((address == 0U) || (reply == NULL) || (request == NULL) ||
        (request_length < 2U) || (request_length > 8U) ||
        (reply_length < 2U) || (reply_length > 8U))
    {
        return 7U;
    }

    memset(reply, 0, sizeof(*reply));

    /* 同一连续会话只初始化一次，避免每条请求反复重置CAN */
    if (s_can1_normal_ready == 0U)
    {
        if (receipt) return CAN1_TIMING_WINDOW;
        result = CAN1_InitMode(CAN_Mode_Normal);
        if (result != 1U)
        {
            return result;
        }
        s_can1_normal_ready = 1U;
    }

    /* FIFO0最多容纳3帧，有限清理前一请求遗留的数据 */
    for (flush_count = 0U; flush_count < 3U; flush_count++)
    {
        if (CAN_MessagePending(CAN1, CAN_FIFO0) == 0U)
        {
            break;
        }
        CAN_Receive(CAN1, CAN_FIFO0, &rx);StartupHome_Capture(&rx);
        if (s_can_ctrl_trace_active) g_can_ctrl_trace_flush_count++;
    }

    if(address==6U && request_length==4U && request[0]==0x9aU)
        g_startup_extension_home_notice=0U;
    memset(&tx, 0, sizeof(tx));
    tx.ExtId = (uint32_t)address << 8;
    tx.IDE = CAN_Id_Extended;
    tx.RTR = CAN_RTR_Data;
    /* 按实际请求长度复制，包括功能码和末尾校验字节 */
    tx.DLC = request_length;
    memcpy(tx.Data, request, request_length);

    if (s_can_ctrl_trace_active) g_can_ctrl_trace_phase = 2U;
    if (receipt)
    {
        result = CAN1_SubmitWindow(&tx, slot_ms, &mailbox, &receipt->tx_ms);
        if (result != 1U) return result;
        receipt->mailbox_accepted = 1U;
    }
    else mailbox = CAN_Transmit(CAN1, &tx);
    if (mailbox == CAN_TxStatus_NoMailBox)
    {
        return 4U;
    }

    start_tick = xTaskGetTickCount();

    while (1)
    {
        status = CAN_TransmitStatus(CAN1, mailbox);
        if (status == CAN_TxStatus_Ok)
        {
            if (receipt) receipt->tx_complete = 1U;
            if (s_can_ctrl_trace_active) g_can_ctrl_trace_tx_ms =
                (uint32_t)(xTaskGetTickCount() - s_can_ctrl_trace_tick);
            break;
        }

        if (receipt && CAN1_NowMs()-slot_ms >= CAN1_NAV_COMPLETION_MS &&
            status != CAN_TxStatus_Failed)
        {
            CAN_CancelTransmit(CAN1, mailbox);
            return CAN1_TIMING_WINDOW;
        }
        if ((status == CAN_TxStatus_Failed) ||
            ((TickType_t)(xTaskGetTickCount() - start_tick) >=
             pdMS_TO_TICKS(CAN1_SINGLE_TX_WAIT_MS)))
        {
            CAN_CancelTransmit(CAN1, mailbox);
            return 4U;
        }

        vTaskDelay(pdMS_TO_TICKS(1U));
    }

    start_tick = xTaskGetTickCount();

    if (s_can_ctrl_trace_active) g_can_ctrl_trace_phase = 3U;
    while ((TickType_t)(xTaskGetTickCount() - start_tick) <
           pdMS_TO_TICKS(CAN1_SINGLE_REPLY_WAIT_MS))
    {
        uint8_t remaining = receipt ? 3U : 1U;
        if (receipt && CAN1_NowMs()-slot_ms > CAN1_NAV_COMPLETION_MS)
            return CAN1_TIMING_WINDOW;
        /* Check the FIFO0 entries without waiting a tick between frames. */
        while (remaining-- && CAN_MessagePending(CAN1, CAN_FIFO0) != 0U)
        {
            uint32_t received_ms=0U;
            if (receipt && CAN1_NowMs()-slot_ms > CAN1_NAV_COMPLETION_MS)
                return CAN1_TIMING_WINDOW;
            CAN_Receive(CAN1, CAN_FIFO0, &rx);
            if (receipt) received_ms=CAN1_NowMs();
            StartupHome_Capture(&rx);
            if (s_can_ctrl_trace_active)
            {
                g_can_ctrl_trace_rx_count++;
                g_can_ctrl_trace_last_rx = rx;
            }

            if ((rx.IDE == CAN_Id_Extended) &&
                (rx.RTR == CAN_RTR_Data) &&
                (rx.ExtId == tx.ExtId))
            {
                /* 保存原始回复，异常时也能查看 */
                *reply = rx;

                /* 回复功能码必须与本次请求一致 */
                if ((rx.DLC != 0U) && (rx.Data[0] == request[0]))
                {
                    if (receipt)
                    {
                        receipt->reply_ms=received_ms;
                        if (CAN1_NowMs()-slot_ms > CAN1_NAV_COMPLETION_MS)
                            return CAN1_TIMING_WINDOW;
                    }
                    if ((rx.DLC == reply_length) &&
                        (rx.Data[reply_length - 1U] == 0x6BU))
                    {
                        return 1U;
                    }

                    return 6U;
                }
            }
        }

        if (receipt && CAN1_NowMs()-slot_ms >= CAN1_NAV_COMPLETION_MS)
            return CAN1_TIMING_WINDOW;
        vTaskDelay(pdMS_TO_TICKS(1U));
    }

    return 5U;
}

static uint32_t CAN1_ExchangeSingle(uint8_t address, const uint8_t *request,
                                   uint8_t request_length, uint8_t reply_length,
                                   CanRxMsg *reply)
{
    return CAN1_ExchangeSingleWindow(address,request,request_length,reply_length,reply,0U,NULL);
}

/* 保留原查询入口，四种读取函数继续使用原来的调用方式 */
static uint32_t CAN1_ReadParameter(uint8_t address,
                                  uint8_t command,
                                  uint8_t reply_length,
                                  CanRxMsg *reply)
{
    uint8_t request[2] = {command, 0x6BU};

    return CAN1_ExchangeSingle(address, request, 2U,
                               reply_length, reply);
}

/* 读取版本，失败后下一次查询重新建立正常模式会话 */
uint32_t CAN1_ReadVersion(uint8_t address, CanRxMsg *reply)
{
    uint32_t result = CAN1_ReadParameter(address, 0x1FU, 6U, reply);
    if (result != 1U)
    {
        CAN1_ResetSession();
    }
    return result;
}

/* 读取状态，不改变驱动器的使能或运动状态 */
uint32_t CAN1_ReadStatus(uint8_t address, CanRxMsg *reply)
{
    uint32_t result = CAN1_ReadParameter(address, 0x3AU, 3U, reply);
    if (result != 1U)
    {
        CAN1_ResetSession();
    }
    return result;
}
uint32_t CAN1_ReadStatusBounded(uint8_t address, uint32_t slot_ms,
                                CanRxMsg *reply, uint32_t *reply_ms)
{
    const uint8_t request[2] = {0x3AU, 0x6BU};
    CAN1_EmmSpeedReceipt receipt = {0};
    uint32_t result;

    if (reply_ms) *reply_ms = 0U;
    if (!reply || !reply_ms) return 7U;
    memset(reply, 0, sizeof(*reply));
    if (address < 1U || address > 4U) return 7U;
    if (CAN1_NowMs()-slot_ms > CAN1_NAV_ADMISSION_MS) return CAN1_TIMING_WINDOW;

    result = CAN1_ExchangeSingleWindow(address, request, 2U, 3U, reply, slot_ms, &receipt);
    if (result == 1U)
    {
        taskENTER_CRITICAL();
        if (CAN1_NowMs()-slot_ms > CAN1_NAV_COMPLETION_MS) result=CAN1_TIMING_WINDOW;
        else *reply_ms = receipt.reply_ms;
        taskEXIT_CRITICAL();
    }
    return result;
}

/* Emm实时转速为整数RPM，输入缩放不影响反馈单位。 */
uint32_t CAN1_ReadSpeed(uint8_t address, CanRxMsg *reply, float *rpm)
{
 uint32_t result;
 if(!rpm || !reply)return 7U;*rpm=0.0f;
 result=CAN1_ReadParameter(address,0x35U,5U,reply);
 if(result==1U && !ZdtEmm_DecodeSpeed(reply->Data,reply->DLC,rpm))result=6U;
 if(result!=1U)CAN1_ResetSession();return result;
}
/* Emm实时位置为65536计数/圈，符号与幅值分开。 */
uint32_t CAN1_ReadPosition(uint8_t address,CanRxMsg *reply,uint8_t *sign,uint32_t *raw)
{
 uint32_t result;
 if(!sign || !raw || !reply)return 7U;*sign=0U;*raw=0U;
 result=CAN1_ReadParameter(address,0x36U,7U,reply);
 if(result==1U && !ZdtEmm_DecodePosition(reply->Data,reply->DLC,sign,raw))result=6U;
 if(result!=1U)CAN1_ResetSession();return result;
}
/* 控制应答统一处理：只接受02，其他应答保留并返回8 */
static uint32_t CAN1_ControlSingleWindow(uint8_t address,
                                   const uint8_t *request,
                                   uint8_t request_length,
                                   CanRxMsg *reply, uint32_t slot_ms,
                                   CAN1_EmmSpeedReceipt *receipt)
{
    uint32_t result, trace, i;
    TickType_t context_tick;
    PowerCommandStatus power_status;
    CanRxMsg empty = {0};

    /* 只允许四轮地址，不允许广播 */
    if ((address < 1U) || (address > 4U) || (reply == NULL))
    {
        return 7U;
    }

#if APP_MOTOR_OFFLINE_ONLY
    /* 离线候选拒绝启用驱动和所有非零速度；STOP和读取不受影响。 */
    if(request && ((request_length==5U && request[0]==0xF3U && request[2]) ||
       (request_length==7U && request[0]==0xF6U && (request[2] || request[3]))))return 7U;
#endif
    trace = ((g_can_ctrl_fault_latched == 0U) && (request != NULL) &&
        (request_length >= 2U) && (request_length <= 8U)) ? 1U : 0U;
    if (trace)
    {
        g_can_ctrl_trace_address = address;
        g_can_ctrl_trace_length = request_length;
        for (i = 0U; i < 8U; i++) g_can_ctrl_trace_request[i] =
            (i < request_length) ? request[i] : 0U;
        g_can_ctrl_trace_result = 0U; g_can_ctrl_trace_phase = 1U;
        g_can_ctrl_trace_rx_count = g_can_ctrl_trace_flush_count = 0U;
        g_can_ctrl_trace_tx_ms = g_can_ctrl_trace_total_ms = 0U;
        g_can_ctrl_trace_last_rx = empty;
        g_can_ctrl_trace_pe2_start = (GPIOE->ODR & GPIO_Pin_2) ? 1U : 0U;
        /* 临界区内只拷贝快照，不发CAN、不改电源。 */
        taskENTER_CRITICAL();
        g_can_ctrl_trace_context.captured = 0U;
        Power_ReadCommandStatus(&power_status);
        context_tick = xTaskGetTickCount();
        g_can_ctrl_trace_context.command_tick = (uint32_t)context_tick;
        g_can_ctrl_trace_context.vm_command_active = power_status.active;
        g_can_ctrl_trace_context.vm_enable_seen = power_status.enable_seen;
        g_can_ctrl_trace_context.vm_enable_count = power_status.enable_count;
        g_can_ctrl_trace_context.vm_enable_age_ms =
            ((power_status.active != 0U) && (power_status.enable_seen != 0U)) ?
            (uint32_t)((TickType_t)(context_tick - (TickType_t)power_status.enable_tick) *
                portTICK_PERIOD_MS) : 0xFFFFFFFFU;
        g_can_ctrl_trace_context.guard_ready = g_can_guard_ready;
        for (i = 0U; i < 4U; i++)
        {
            g_can_ctrl_trace_context.poll_result[i] = g_can_poll_result[i];
            g_can_ctrl_trace_context.poll_flags[i] = g_can_poll_flags[i];
            g_can_ctrl_trace_context.poll_count[i] = g_can_poll_count[i];
            g_can_ctrl_trace_context.poll_age_ms[i] =
                ((g_can_poll_result[i] == 1U) && (g_can_poll_count[i] != 0U)) ?
                (uint32_t)((TickType_t)(context_tick - g_can_poll_tick[i]) *
                    portTICK_PERIOD_MS) : 0xFFFFFFFFU;
        }
        g_can_ctrl_trace_context.captured = 1U;
        s_can_ctrl_trace_tick = context_tick;
        taskEXIT_CRITICAL();
        s_can_ctrl_trace_active = 1U;
    }
    result = CAN1_ExchangeSingleWindow(address, request,
                                      request_length, 3U, reply, slot_ms, receipt);

    if ((result == 1U) && !ZdtEmm_DecodeAck(request[0],reply->Data,reply->DLC))
    {
        /* 非02不能作为成功确认，原始回复仍保留 */
        result = 8U;
    }
    if (receipt && result == 1U)
    {
        taskENTER_CRITICAL();
        if (CAN1_NowMs()-slot_ms <= CAN1_NAV_COMPLETION_MS) receipt->ack_valid=1U;
        else result=CAN1_TIMING_WINDOW;
        taskEXIT_CRITICAL();
    }

    if (trace)
    {
        g_can_ctrl_trace_total_ms = (uint32_t)(xTaskGetTickCount() - s_can_ctrl_trace_tick);
        g_can_ctrl_trace_pe2_end = (GPIOE->ODR & GPIO_Pin_2) ? 1U : 0U;
        g_can_ctrl_trace_esr = CAN1->ESR;
        g_can_ctrl_trace_result = result;
        s_can_ctrl_trace_active = 0U;
        if (result != 1U && result != CAN1_TIMING_WINDOW) g_can_ctrl_fault_latched = 1U;
    }
    if (result != 1U && result != CAN1_TIMING_WINDOW)
    {
        CAN1_ResetSession();
    }

    return result;
}

static uint32_t CAN1_ControlSingle(uint8_t address, const uint8_t *request,
                                  uint8_t request_length, CanRxMsg *reply)
{
    return CAN1_ControlSingleWindow(address,request,request_length,reply,0U,NULL);
}

/* 使能控制：0松轴，1锁轴；同步位固定0，立即执行 */
uint32_t CAN1_SetMotorEnable(uint8_t address, uint8_t enabled,
                             CanRxMsg *reply)
{
    uint8_t request[5] = {0xF3U, 0xABU, 0U, 0U, 0x6BU};

    if (enabled > 1U)
    {
        return 7U;
    }

    if(enabled) {
#if !ZDT_EMM_SETTINGS_CONFIRMED
        return 9U;
#else
        CanRxMsg check_reply;float checked_rpm;uint32_t checked;
        checked=CAN1_ReadEmmSystemSnapshot(address,&check_reply,&checked_rpm);
        if(checked!=1U)return checked;
#endif
    }

    request[2] = enabled;

    return CAN1_ControlSingle(address, request, 5U, reply);
}

/* 立即停止：同步位固定0，不缓存停止命令 */
uint32_t CAN1_StopMotor(uint8_t address, CanRxMsg *reply)
{
    uint8_t request[4] = {0xFEU, 0x98U, 0U, 0x6BU};

    uint32_t result=CAN1_ControlSingle(address, request, 4U, reply);
    ChassisLoad_RecordCommand(address,0,CHASSIS_LOAD_STOP_ACK,result,(uint32_t)xTaskGetTickCount()*portTICK_PERIOD_MS);
    return result;
}

/* Emm速度控制：方向0为CW、1为CCW，输入缩小10倍。 */
/* CW/CCW与整车前后方向的对应关系仍需实测 */
static uint32_t CAN1_EmmSpeed(uint8_t address,uint8_t direction,uint16_t speed_raw,
    uint16_t acceleration,uint8_t sync,CanRxMsg *reply,uint32_t slot_ms,CAN1_EmmSpeedReceipt *receipt)
{
 uint8_t request[7];
 if(speed_raw && !ZDT_EMM_SETTINGS_CONFIRMED)return 9U;
 if(speed_raw>ZDT_EMM_APP_MAX_SPEED_RAW || !ZdtEmm_EncodeSpeed(direction,speed_raw,acceleration,sync,request))return 7U;
 { uint32_t result;int32_t target=direction?-(int32_t)speed_raw:(int32_t)speed_raw;
   ChassisLoad_RecordCommand(address,target,CHASSIS_LOAD_TX,0U,(uint32_t)xTaskGetTickCount()*portTICK_PERIOD_MS);
   result=CAN1_ControlSingleWindow(address,request,7U,reply,slot_ms,receipt);
   ChassisLoad_RecordCommand(address,target,sync?CHASSIS_LOAD_STAGED_ACK:CHASSIS_LOAD_ACK,result,(uint32_t)xTaskGetTickCount()*portTICK_PERIOD_MS);
   return result; }
}
uint32_t CAN1_SetEmmSpeedRaw(uint8_t address,uint8_t direction,uint16_t speed_raw,
    uint16_t acceleration,CanRxMsg *reply)
{ return CAN1_EmmSpeed(address,direction,speed_raw,acceleration,0U,reply,0U,NULL); }
uint32_t CAN1_StageEmmSpeedRaw(uint8_t address,uint8_t direction,uint16_t speed_raw,
    uint16_t acceleration,CanRxMsg *reply)
{ return CAN1_EmmSpeed(address,direction,speed_raw,acceleration,1U,reply,0U,NULL); }

uint32_t CAN1_SetEmmSpeedRawBounded(uint8_t address,uint8_t direction,uint16_t speed_raw,
    uint16_t acceleration,uint32_t slot_ms,CanRxMsg *reply,CAN1_EmmSpeedReceipt *receipt)
{
    if (!receipt) return 7U;
    memset(receipt,0,sizeof(*receipt));
    if (!reply) return 7U;
    memset(reply,0,sizeof(*reply));
    if (CAN1_NowMs()-slot_ms > CAN1_NAV_ADMISSION_MS) return CAN1_TIMING_WINDOW;
    return CAN1_EmmSpeed(address,direction,speed_raw,acceleration,0U,reply,slot_ms,receipt);
}

/* 复用速度采样时隙：仅一条读取请求，有界接收五包系统状态回复。 */
static uint32_t CAN1_ReadEmmSystemSnapshotImpl(uint8_t address, CanRxMsg *reply,
                                                float *rpm, CAN1_EmmSystemFeedback *feedback,
                                                uint8_t bounded, uint32_t slot_ms)
{
    CanTxMsg tx;CanRxMsg rx;ChassisSystemSnapshot sample={0};
    TickType_t start;uint32_t result=5U;uint8_t packet=0U,length=0U,i,offset,mailbox,status;
    uint8_t data[30]={0};ZdtEmmSystemData decoded;
    uint32_t request_ms = 0U, receive_ms = 0U, elapsed_ms;
    if (feedback) memset(feedback, 0, sizeof(*feedback));
    if(address<1U || address>4U || !reply || !rpm)return 7U;
    *rpm=0.0f;memset(reply,0,sizeof(*reply));
    if (bounded && (!s_can1_normal_ready || CAN1_NowMs()-slot_ms > CAN1_NAV_ADMISSION_MS))
        return CAN1_TIMING_WINDOW;
    if(!s_can1_normal_ready){result=CAN1_InitMode(CAN_Mode_Normal);if(result!=1U)goto finished;s_can1_normal_ready=1U;}
    for(i=0U;i<3U && CAN_MessagePending(CAN1,CAN_FIFO0);i++){CAN_Receive(CAN1,CAN_FIFO0,&rx);StartupHome_Capture(&rx);}
    memset(&tx,0,sizeof(tx));tx.ExtId=(uint32_t)address<<8;tx.IDE=CAN_Id_Extended;tx.RTR=CAN_RTR_Data;
    tx.DLC=3U;tx.Data[0]=0x43U;tx.Data[1]=0x7AU;tx.Data[2]=0x6BU;
    if (bounded)
    {
        result = CAN1_SubmitWindow(&tx, slot_ms, &mailbox, &request_ms);
        if (result != 1U) goto finished;
    }
    else
    {
        if (feedback) request_ms = CAN1_NowMs();
        mailbox=CAN_Transmit(CAN1,&tx);if(mailbox==CAN_TxStatus_NoMailBox){result=4U;goto finished;}
    }
    start=xTaskGetTickCount();
    do {
        if (bounded) elapsed_ms = CAN1_NowMs()-slot_ms;
        status=CAN_TransmitStatus(CAN1,mailbox);
        if (bounded && status == CAN_TxStatus_Pending && elapsed_ms >= CAN1_NAV_COMPLETION_MS)
        {
            CAN_CancelTransmit(CAN1, mailbox);
            goto timing_window;
        }
        if(status==CAN_TxStatus_Failed || (TickType_t)(xTaskGetTickCount()-start)>=pdMS_TO_TICKS(CAN1_SINGLE_TX_WAIT_MS)){
            CAN_CancelTransmit(CAN1,mailbox);result=4U;goto finished;}
    }while(status!=CAN_TxStatus_Ok);
    start=xTaskGetTickCount();result=5U;
    while((TickType_t)(xTaskGetTickCount()-start)<pdMS_TO_TICKS(CAN1_SINGLE_REPLY_WAIT_MS)){
        if (bounded)
        {
            elapsed_ms = CAN1_NowMs()-slot_ms;
            if (elapsed_ms > CAN1_NAV_COMPLETION_MS) goto timing_window;
        }
        if(!CAN_MessagePending(CAN1,CAN_FIFO0))
        {
            if (bounded && elapsed_ms >= CAN1_NAV_COMPLETION_MS) goto timing_window;
            continue;
        }
        CAN_Receive(CAN1,CAN_FIFO0,&rx);
        if (feedback) receive_ms = CAN1_NowMs();
        StartupHome_Capture(&rx);
        if (bounded && receive_ms-slot_ms > CAN1_NAV_COMPLETION_MS) goto timing_window;
        if(rx.IDE!=CAN_Id_Extended || rx.RTR!=CAN_RTR_Data || (rx.ExtId>>8)!=address)continue;
        if(rx.DLC==0U || rx.Data[0]!=0x43U)continue;
        if (!bounded && (rx.DLC<2U || rx.DLC>8U)) continue;
        *reply=rx;
        if((rx.ExtId&0xFFU)!=packet){result=6U;break;}
        /* 30个载荷字节，续帧重复功能码：前四帧8字节，末帧2字节。 */
        if (bounded && (packet>=5U || rx.DLC!=(packet<4U ? 8U : 2U)))
        {
            result=6U;
            break;
        }
        offset=packet?1U:0U;
        if((uint32_t)length+rx.DLC-offset>30U){result=6U;break;}
        for(i=offset;i<rx.DLC;i++)data[length++]=rx.Data[i];
        packet++;
        if(length==30U){
            uint8_t valid = ZdtEmm_DecodeSystem(data,length,&decoded);
            if (bounded && CAN1_NowMs()-slot_ms > CAN1_NAV_COMPLETION_MS) goto timing_window;
            if(!valid){result=6U;break;}
            sample.speed_decirpm=decoded.speed_decirpm;
            sample.position_deci_deg=decoded.position_deci_deg;sample.position_sign=decoded.position_sign;
            sample.phase_ma=decoded.phase_ma;sample.bus_mv=decoded.bus_mv;
            sample.home_flags=decoded.home_flags;sample.motor_flags=decoded.motor_flags;
            if (feedback)
            {
                feedback->request_ms = request_ms;
                feedback->reply_ms = receive_ms;
                feedback->position_counts = decoded.position_counts;
                feedback->speed_decirpm = decoded.speed_decirpm;
                feedback->position_sign = decoded.position_sign;
                feedback->motor_flags = decoded.motor_flags;
                feedback->home_flags = decoded.home_flags;
                feedback->address = address;
                feedback->valid = 1U;
            }
            *rpm=(float)sample.speed_decirpm*0.1f;result=1U;break;
        }
    }
    goto finished;
timing_window:
    result = CAN1_TIMING_WINDOW;
finished:
    ChassisLoad_RecordSnapshot(address,result,&sample,(uint32_t)xTaskGetTickCount()*portTICK_PERIOD_MS);
    if(result!=1U && !bounded)CAN1_ResetSession();return result;
}

uint32_t CAN1_ReadEmmSystemSnapshot(uint8_t address, CanRxMsg *reply, float *rpm)
{
    return CAN1_ReadEmmSystemSnapshotImpl(address, reply, rpm, NULL, 0U, 0U);
}

uint32_t CAN1_ReadEmmSystemFeedback(uint8_t address, CanRxMsg *reply,
                                   CAN1_EmmSystemFeedback *feedback)
{
    float rpm;
    if (!feedback) return 7U;
    return CAN1_ReadEmmSystemSnapshotImpl(address, reply, &rpm, feedback, 0U, 0U);
}

uint32_t CAN1_ReadEmmSystemFeedbackBounded(uint8_t address, uint32_t slot_ms,
                                          CanRxMsg *reply, CAN1_EmmSystemFeedback *feedback)
{
    CAN1_EmmSystemFeedback decoded;
    uint32_t result;
    float rpm;
    if (!feedback) return 7U;
    memset(feedback, 0, sizeof(*feedback));
    result=CAN1_ReadEmmSystemSnapshotImpl(address, reply, &rpm, &decoded, 1U, slot_ms);
    if (result == 1U)
    {
        /* 诊断记录也占用本槽时间，发布有效数据前复查截止时刻。 */
        taskENTER_CRITICAL();
        if (CAN1_NowMs()-slot_ms > CAN1_NAV_COMPLETION_MS) result=CAN1_TIMING_WINDOW;
        else *feedback = decoded;
        taskEXIT_CRITICAL();
    }
    return result;
}

/* 启动并行伸缩专用节点6，不扩展四轮控制接口；唯一CAN任务调用。 */
uint32_t VehicleArm_Exchange(uint8_t address,const uint8_t *q,uint8_t length,uint8_t reply_length,void *out)
{
    CanRxMsg *reply=(CanRxMsg*)out;
#if !APP_MOTOR_OFFLINE_ONLY
    CanRxMsg rx;CanTxMsg tx;uint8_t packet=0U,offset=1U,chunk,mailbox,status,flush;
    TickType_t start;uint32_t r;
#endif
    if(!q || !out || (address!=5U && address!=6U))return 7U;
    if(address==6U && length==3U && q[0]==0x9c && q[1]==0x48 && q[2]==0x6b)
        return CAN1_ExchangeSingle(address,q,length,3U,reply);
    if(length==2U && q[1]==0x6b &&
       ((q[0]==0x1a && reply_length==4U) || (q[0]==0x36 && reply_length==7U) ||
        ((q[0]==0x3a || (address==6U && q[0]==0x3b)) && reply_length==3U) || (q[0]==0x35 && reply_length==5U)))
        return CAN1_ExchangeSingle(address,q,length,reply_length,reply);
    if(length==4U && q[0]==0xfe && q[1]==0x98 && q[2]==0 && q[3]==0x6b)
        return CAN1_ExchangeSingle(address,q,length,3U,reply);
#if APP_MOTOR_OFFLINE_ONLY
    return 9U;
#else
    if(!g_startup_axes_vm_request || g_startup_axes_state<2U || g_startup_axes_state>4U ||
       g_safety_fault || g_rtos_fault || g_can_ctrl_fault_latched)return 9U;
    if(address==6U && g_startup_axes_state==2U && length==4U &&
       q[0]==0x9a && q[1]==2U && q[2]==0U && q[3]==0x6b && reply_length==3U)
        return CAN1_ExchangeSingle(address,q,length,reply_length,reply);
    if(length==5U && q[0]==0xf3 && q[1]==0xab && q[2]==1 && q[3]==0 && q[4]==0x6b)
        return CAN1_ExchangeSingle(address,q,length,3U,reply);
    if(length!=15U || q[0]!=0xfd || q[1]>1 || q[12]!=2 || q[13]!=0 || q[14]!=0x6b || reply_length!=3U)return 7U;
    {
        uint32_t speed=((uint32_t)q[6]<<8)|q[7];
        if(!speed || speed>(address==5U?18464U:1060U) || (!q[2] && !q[3]) || (!q[4] && !q[5]))return 7U;
    }
    if(!s_can1_normal_ready){r=CAN1_InitMode(CAN_Mode_Normal);if(r!=1U)return r;s_can1_normal_ready=1U;}
    for(flush=0;flush<3 && CAN_MessagePending(CAN1,CAN_FIFO0);flush++){CAN_Receive(CAN1,CAN_FIFO0,&rx);StartupHome_Capture(&rx);}
    while(offset<length) {
        memset(&tx,0,sizeof(tx));chunk=length-offset;if(chunk>7)chunk=7;
        tx.ExtId=((uint32_t)address<<8)|packet;tx.IDE=CAN_Id_Extended;tx.RTR=CAN_RTR_Data;tx.DLC=chunk+1;tx.Data[0]=q[0];
        memcpy(&tx.Data[1],&q[offset],chunk);mailbox=CAN_Transmit(CAN1,&tx);if(mailbox>2)return 4U;
        start=xTaskGetTickCount();
        for(;;){status=CAN_TransmitStatus(CAN1,mailbox);if(status==CAN_TxStatus_Ok)break;
            if(status==CAN_TxStatus_Failed || xTaskGetTickCount()-start>=pdMS_TO_TICKS(20U)){CAN_CancelTransmit(CAN1,mailbox);return 4U;}
            vTaskDelay(pdMS_TO_TICKS(1U));}
        offset+=chunk;packet++;
    }
    start=xTaskGetTickCount();
    while(xTaskGetTickCount()-start<pdMS_TO_TICKS(30U)) {
        if(CAN_MessagePending(CAN1,CAN_FIFO0)) {
            CAN_Receive(CAN1,CAN_FIFO0,&rx);StartupHome_Capture(&rx);
            if(rx.IDE==CAN_Id_Extended && rx.RTR==CAN_RTR_Data && rx.ExtId==((uint32_t)address<<8) && rx.Data[0]==0xfd)
            { *reply=rx;return rx.DLC==3 && rx.Data[2]==0x6b ? 1U:6U; }
        }
        vTaskDelay(pdMS_TO_TICKS(1U));
    }return 5U;
#endif
}

uint32_t StartupAxis_Exchange(const uint8_t *q,uint8_t n,uint8_t reply_n,void *out)
{return VehicleArm_Exchange(6U,q,n,reply_n,out);}

uint32_t StartupAxis_ReadHomeParameters(uint8_t out[17])
{
#if APP_MOTOR_OFFLINE_ONLY
    (void)out;return 9U;
#else
    CanRxMsg replies[3];
    CanTxMsg tx;
    CanRxMsg rx;
    TickType_t started;
    uint32_t result;
    uint8_t mailbox, status, flush, packet = 0U, offset = 1U, chunk;

    if (out == NULL) return 7U;
    memset(out, 0, 17U);
    memset(replies, 0, 3U * sizeof(CanRxMsg));
    if (s_can1_normal_ready == 0U)
    {
        result = CAN1_InitMode(CAN_Mode_Normal);
        if (result != 1U) return result;
        s_can1_normal_ready = 1U;
    }
    for (flush = 0U; flush < 3U && CAN_MessagePending(CAN1, CAN_FIFO0); flush++)
    { CAN_Receive(CAN1, CAN_FIFO0, &rx);StartupHome_Capture(&rx); }
    memset(&tx, 0, sizeof(tx));
    tx.ExtId = 0x0600U;
    tx.IDE = CAN_Id_Extended; tx.RTR = CAN_RTR_Data;
    tx.DLC = 2U; tx.Data[0] = 0x22U; tx.Data[1] = 0x6BU;
    mailbox = CAN_Transmit(CAN1, &tx);
    if (mailbox == CAN_TxStatus_NoMailBox) return 4U;
    started = xTaskGetTickCount();
    for (;;)
    {
        status = CAN_TransmitStatus(CAN1, mailbox);
        if (status == CAN_TxStatus_Ok) break;
        if (status == CAN_TxStatus_Failed ||
            (TickType_t)(xTaskGetTickCount() - started) >= pdMS_TO_TICKS(CAN1_SINGLE_TX_WAIT_MS))
        {
            CAN_CancelTransmit(CAN1, mailbox);
            return 4U;
        }
        vTaskDelay(pdMS_TO_TICKS(1U));
    }
    started = xTaskGetTickCount();
    out[0] = 0x22U;
    while ((TickType_t)(xTaskGetTickCount() - started) < pdMS_TO_TICKS(CAN1_SINGLE_REPLY_WAIT_MS))
    {
        if (CAN_MessagePending(CAN1, CAN_FIFO0))
        {
            CAN_Receive(CAN1, CAN_FIFO0, &rx);StartupHome_Capture(&rx);
            if (rx.IDE == CAN_Id_Extended && rx.RTR == CAN_RTR_Data &&
                (rx.ExtId >> 8) == 6U && rx.DLC != 0U && rx.Data[0] == 0x22U)
            {
                replies[packet] = rx;
                chunk = (uint8_t)(17U - offset);
                if (chunk > 7U) chunk = 7U;
                if (rx.ExtId != (0x0600U | packet) || rx.DLC != (uint8_t)(chunk + 1U))
                    return 6U;
                memcpy(&out[offset], &rx.Data[1], chunk);
                offset = (uint8_t)(offset + chunk);
                if (offset == 17U) return out[16] == 0x6BU ? 1U : 6U;
                packet++;
            }
        }
        vTaskDelay(pdMS_TO_TICKS(1U));
    }
    return 5U;

#endif
}
