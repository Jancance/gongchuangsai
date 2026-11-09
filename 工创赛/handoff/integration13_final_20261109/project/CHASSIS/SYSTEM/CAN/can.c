#include "can.h"
#include "stm32f4xx_can.h"
#include "FreeRTOS.h"
#include "task.h"
#include <string.h>
#include "app_config.h"
#include "startup_axes.h"
#include "power.h"
#include "safety.h"
static void StartupHome_Receive(CanRxMsg *rx);


/* 新目标航向模式不允许单次交易长于指令提交监督期限。 */
#if APP_HEADING_SESSION_TEST
#define CAN1_SINGLE_TX_WAIT_MS 20U
/* 2026-11-08: node4 answers in ~17ms while 1/2/3 answer in 6-8ms; the old 30ms
 * budget plus bus contention made it time out intermittently, which expired the
 * wheel guard and latched 0x20 during unrelated mechanism work. */
#define CAN1_SINGLE_REPLY_WAIT_MS 60U
/* 2026-11-08: the lift driver answers 0x22 in three frames and frame 3 arrives at
 * ~89ms on hardware, so the multi-frame parameter read needs its own longer
 * window. Kept separate so ordinary single-frame polls stay fast. */
#define CAN1_PARAM_REPLY_WAIT_MS 250U
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
static uint32_t s_can1_normal_ready = 0U;
/* CAN1硬件只在上电后首次初始化一次；重复CAN_DeInit(CAN1)会连CAN2共享过滤器域一起清掉。 */
static uint32_t s_can1_hw_initialized = 0U;
/* 标记下一次查询需重新初始化，不在这里发送或切换硬件模式 */
void CAN1_ResetSession(void)
{
    s_can1_normal_ready = 0U;
}
/* 成功返回1，时钟不匹配返回2，初始化失败返回3 */
/* CAN1/CAN2共享过滤器。仅保存/恢复原14..27号CAN2资源，
 * 所有初始化仍由唯一CAN任务串行调用；不改CAN1刚重建的过滤器0。 */
typedef struct {
 uint32_t valid,fmr,fm1r,fs1r,ffa1r,fa1r,fr[14][2];
} CAN2FilterBackup;
static void CAN2_FiltersSave(CAN2FilterBackup *b)
{
 uint32_t i;memset(b,0,sizeof(*b));
 if(!(RCC->APB1ENR&RCC_APB1Periph_CAN2) || ((CAN1->FMR>>8)&63U)!=14U)return;
 b->valid=1U;b->fmr=CAN1->FMR;b->fm1r=CAN1->FM1R;b->fs1r=CAN1->FS1R;
 b->ffa1r=CAN1->FFA1R;b->fa1r=CAN1->FA1R;
 for(i=0;i<14U;i++){b->fr[i][0]=CAN1->sFilterRegister[i+14U].FR1;b->fr[i][1]=CAN1->sFilterRegister[i+14U].FR2;}
}
static void CAN2_FiltersRestore(const CAN2FilterBackup *b)
{
 const uint32_t mask=0x0fffc000U;uint32_t i;
 if(!b->valid)return;
 CAN1->FMR|=1U;
 CAN1->FMR=(CAN1->FMR&~0x3f00U)|(b->fmr&0x3f00U)|1U;
 CAN1->FA1R&=~mask;
 for(i=0;i<14U;i++){CAN1->sFilterRegister[i+14U].FR1=b->fr[i][0];CAN1->sFilterRegister[i+14U].FR2=b->fr[i][1];}
 CAN1->FM1R=(CAN1->FM1R&~mask)|(b->fm1r&mask);
 CAN1->FS1R=(CAN1->FS1R&~mask)|(b->fs1r&mask);
 CAN1->FFA1R=(CAN1->FFA1R&~mask)|(b->ffa1r&mask);
 CAN1->FA1R=(CAN1->FA1R&~mask)|(b->fa1r&mask);
 CAN1->FMR=(CAN1->FMR&~1U)|(b->fmr&1U);
}
static uint32_t CAN1_InitMode(uint8_t mode)
{
    TickType_t init_start_tick;
    CAN2FilterBackup can2_filters;
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
    if (!s_can1_hw_initialized)
    {
        CAN2_FiltersSave(&can2_filters);
        CAN_DeInit(CAN1);
        CAN2_FiltersRestore(&can2_filters);
    }

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
    /* 不再重复CAN_DeInit(CAN1)，必须允许硬件自动离线恢复，否则CAN1进Bus-Off后无法自愈。 */
    can.CAN_ABOM = ENABLE;

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
    if (!s_can1_hw_initialized)
    {
        CAN_FilterInit(&filter);
        s_can1_hw_initialized = 1U;
    }

    return 1U;
}

static uint32_t CAN1_LoopbackTestInner(void)
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

    StartupHome_Receive(&rx);

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

/* Restore the real bus mode after the power-on self test. Without this
 * the controller stays in Silent_LoopBack until the first normal
 * exchange, so any lift/arm command issued before that sees its own
 * frames echoed back and fails with LRC_CAUSE_COMM. */
uint32_t CAN1_LoopbackTest(void)
{
    uint32_t r = CAN1_LoopbackTestInner();
    (void)CAN1_InitMode(CAN_Mode_Normal);
    return r;
}

/* 只读版本：1成功，4发送失败，5无回复，6格式异常 */
/* 共用短帧查询，供版本和状态读取使用 */
/* 通用单帧收发：只允许独立CAN任务串行调用 */
/* 仅捕获本轮节点6的完成通知；旧会话/错节点不作为成功。 */
static void StartupHome_Receive(CanRxMsg *rx)
{
    CAN_Receive(CAN1,CAN_FIFO0,rx);
#if (VEHICLE_EXTENSION_HOME_ONLY || VEHICLE_STARTUP_GROUP_ENABLE)
    extern volatile uint32_t g_extension_home_capture,g_extension_home_done_notice;
    if(g_extension_home_capture && rx->IDE==CAN_Id_Extended && rx->RTR==CAN_RTR_Data &&
       rx->ExtId==0x600U && rx->DLC==3U && rx->Data[0]==0x9aU && rx->Data[1]==0x9fU && rx->Data[2]==0x6bU)
        g_extension_home_done_notice=1U;
#endif
}
static uint32_t CAN1_ExchangeSingle(uint8_t address,
                                    const uint8_t *request,
                                    uint8_t request_length,
                                    uint8_t reply_length,
                                    CanRxMsg *reply)
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
        StartupHome_Receive(&rx);
        if (s_can_ctrl_trace_active) g_can_ctrl_trace_flush_count++;
    }

#if (VEHICLE_EXTENSION_HOME_ONLY || VEHICLE_STARTUP_GROUP_ENABLE)
    if(address==6U && request_length==4U && request[0]==0x9aU){
        extern volatile uint32_t g_extension_home_capture,g_extension_home_done_notice;
        g_extension_home_done_notice=0U;g_extension_home_capture=1U;
    }
#endif
    memset(&tx, 0, sizeof(tx));
    tx.ExtId = (uint32_t)address << 8;
    tx.IDE = CAN_Id_Extended;
    tx.RTR = CAN_RTR_Data;
    /* 按实际请求长度复制，包括功能码和末尾校验字节 */
    tx.DLC = request_length;
    memcpy(tx.Data, request, request_length);

    if (s_can_ctrl_trace_active) g_can_ctrl_trace_phase = 2U;
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
            if (s_can_ctrl_trace_active) g_can_ctrl_trace_tx_ms =
                (uint32_t)(xTaskGetTickCount() - s_can_ctrl_trace_tick);
            break;
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
        if (CAN_MessagePending(CAN1, CAN_FIFO0) != 0U)
        {
            StartupHome_Receive(&rx);
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
                    if ((rx.DLC == reply_length) &&
                        (rx.Data[reply_length - 1U] == 0x6BU))
                    {
                        return 1U;
                    }

                    return 6U;
                }
            }
        }

        vTaskDelay(pdMS_TO_TICKS(1U));
    }

    return 5U;
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
static uint32_t CAN1_ControlSingle(uint8_t address,
                                   const uint8_t *request,
                                   uint8_t request_length,
                                   CanRxMsg *reply)
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
    result = CAN1_ExchangeSingle(address, request,
                                 request_length, 3U, reply);

    if ((result == 1U) && !ZdtEmm_DecodeAck(request[0],reply->Data,reply->DLC))
    {
        /* 非02不能作为成功确认，原始回复仍保留 */
        result = 8U;
    }

    if (trace)
    {
        g_can_ctrl_trace_total_ms = (uint32_t)(xTaskGetTickCount() - s_can_ctrl_trace_tick);
        g_can_ctrl_trace_pe2_end = (GPIOE->ODR & GPIO_Pin_2) ? 1U : 0U;
        g_can_ctrl_trace_esr = CAN1->ESR;
        g_can_ctrl_trace_result = result;
        s_can_ctrl_trace_active = 0U;
        if (result != 1U) g_can_ctrl_fault_latched = 1U;
    }
    if (result != 1U)
    {
        CAN1_ResetSession();
    }

    return result;
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
    uint16_t acceleration,uint8_t sync,CanRxMsg *reply)
{
 uint8_t request[7];
 if(speed_raw && !ZDT_EMM_SETTINGS_CONFIRMED)return 9U;
 if(speed_raw>ZDT_EMM_APP_MAX_SPEED_RAW || !ZdtEmm_EncodeSpeed(direction,speed_raw,acceleration,sync,request))return 7U;
 { uint32_t result;int32_t target=direction?-(int32_t)speed_raw:(int32_t)speed_raw;
   ChassisLoad_RecordCommand(address,target,CHASSIS_LOAD_TX,0U,(uint32_t)xTaskGetTickCount()*portTICK_PERIOD_MS);
   result=CAN1_ControlSingle(address,request,7U,reply);
   ChassisLoad_RecordCommand(address,target,sync?CHASSIS_LOAD_STAGED_ACK:CHASSIS_LOAD_ACK,result,(uint32_t)xTaskGetTickCount()*portTICK_PERIOD_MS);
   return result; }
}
uint32_t CAN1_SetEmmSpeedRaw(uint8_t address,uint8_t direction,uint16_t speed_raw,
    uint16_t acceleration,CanRxMsg *reply)
{ return CAN1_EmmSpeed(address,direction,speed_raw,acceleration,0U,reply); }
uint32_t CAN1_StageEmmSpeedRaw(uint8_t address,uint8_t direction,uint16_t speed_raw,
    uint16_t acceleration,CanRxMsg *reply)
{ return CAN1_EmmSpeed(address,direction,speed_raw,acceleration,1U,reply); }

/* 复用速度采样时隙：仅一条读取请求，有界接收五包系统状态回复。 */
uint32_t CAN1_ReadEmmSystemSnapshot(uint8_t address,CanRxMsg *reply,float *rpm)
{
    CanTxMsg tx;CanRxMsg rx;ChassisSystemSnapshot sample={0};
    TickType_t start;uint32_t result=5U;uint8_t packet=0U,length=0U,i,offset,mailbox,status;
    uint8_t data[30]={0};ZdtEmmSystemData decoded;
    if(address<1U || address>4U || !reply || !rpm)return 7U;
    *rpm=0.0f;memset(reply,0,sizeof(*reply));
    if(!s_can1_normal_ready){result=CAN1_InitMode(CAN_Mode_Normal);if(result!=1U)goto finished;s_can1_normal_ready=1U;}
    for(i=0U;i<3U && CAN_MessagePending(CAN1,CAN_FIFO0);i++)StartupHome_Receive(&rx);
    memset(&tx,0,sizeof(tx));tx.ExtId=(uint32_t)address<<8;tx.IDE=CAN_Id_Extended;tx.RTR=CAN_RTR_Data;
    tx.DLC=3U;tx.Data[0]=0x43U;tx.Data[1]=0x7AU;tx.Data[2]=0x6BU;
    mailbox=CAN_Transmit(CAN1,&tx);if(mailbox==CAN_TxStatus_NoMailBox){result=4U;goto finished;}
    start=xTaskGetTickCount();
    do {
        status=CAN_TransmitStatus(CAN1,mailbox);
        if(status==CAN_TxStatus_Failed || (TickType_t)(xTaskGetTickCount()-start)>=pdMS_TO_TICKS(CAN1_SINGLE_TX_WAIT_MS)){
            CAN_CancelTransmit(CAN1,mailbox);result=4U;goto finished;}
    }while(status!=CAN_TxStatus_Ok);
    start=xTaskGetTickCount();result=5U;
    while((TickType_t)(xTaskGetTickCount()-start)<pdMS_TO_TICKS(CAN1_SINGLE_REPLY_WAIT_MS)){
        if(!CAN_MessagePending(CAN1,CAN_FIFO0))continue;
        StartupHome_Receive(&rx);
        if(rx.IDE!=CAN_Id_Extended || rx.RTR!=CAN_RTR_Data || (rx.ExtId>>8)!=address)continue;
        if(rx.DLC<2U || rx.DLC>8U || rx.Data[0]!=0x43U)continue;
        *reply=rx;
        if((rx.ExtId&0xFFU)!=packet){result=6U;break;}
        offset=packet?1U:0U;
        if((uint32_t)length+rx.DLC-offset>30U){result=6U;break;}
        for(i=offset;i<rx.DLC;i++)data[length++]=rx.Data[i];
        packet++;
        if(length==30U){
            if(!ZdtEmm_DecodeSystem(data,length,&decoded)){result=6U;break;}
            sample.speed_decirpm=decoded.speed_decirpm;
            sample.position_deci_deg=decoded.position_deci_deg;sample.position_sign=decoded.position_sign;
            sample.phase_ma=decoded.phase_ma;sample.bus_mv=decoded.bus_mv;
            sample.home_flags=decoded.home_flags;sample.motor_flags=decoded.motor_flags;
            *rpm=(float)sample.speed_decirpm*0.1f;result=1U;break;
        }
    }
finished:
    ChassisLoad_RecordSnapshot(address,result,&sample,(uint32_t)xTaskGetTickCount()*portTICK_PERIOD_MS);
    if(result!=1U)CAN1_ResetSession();return result;
}

/* 启动并行伸缩专用节点6，不扩展四轮控制接口；唯一CAN任务调用。 */
#include "lift_ruler_adapter.h"
uint32_t VehicleArm_Exchange(uint8_t address,const uint8_t *q,uint8_t length,uint8_t reply_length,void *out)
{
    CanRxMsg *reply=(CanRxMsg*)out;
#if !APP_MOTOR_OFFLINE_ONLY
    CanRxMsg rx;CanTxMsg tx;uint8_t packet=0U,offset=1U,chunk,mailbox,status,flush;
    TickType_t start;uint32_t r;
#endif
    if(!q || !out || (address!=5U && address!=6U))return 7U;
    if(VEHICLE_LIFT_RULER_ONLY && address!=5U && !(VEHICLE_EXTENSION_HOME_ONLY && address==6U))return 9U;
    if(length==2U && q[1]==0x6b &&
       (((q[0]==0x1a || q[0]==0x31) && reply_length==4U) || ((q[0]==0x36 || q[0]==0x33) && reply_length==7U) ||
        ((q[0]==0x3f && address==5U && reply_length==4U) || (q[0]==0x3a && reply_length==3U)) || (q[0]==0x35 && reply_length==5U)))
        return CAN1_ExchangeSingle(address,q,length,reply_length,reply);
#if (VEHICLE_EXTENSION_HOME_ONLY || VEHICLE_STARTUP_GROUP_ENABLE)
    if(address==6U && length==2U && q[0]==0x3bU && q[1]==0x6bU && reply_length==3U)
        return CAN1_ExchangeSingle(6U,q,length,3U,reply);
    if(address==6U && length==3U && q[0]==0x9cU && q[1]==0x48U && q[2]==0x6bU && reply_length==3U)
        return CAN1_ExchangeSingle(6U,q,length,3U,reply);
#if !APP_MOTOR_OFFLINE_ONLY
    if(address==6U && length==4U && q[0]==0x9aU && q[1]==2U && q[2]==0U && q[3]==0x6bU && reply_length==3U){
        if(!g_startup_axes_vm_request || g_startup_axes_state!=2U || g_safety_fault || g_rtos_fault || g_can_ctrl_fault_latched)return 9U;
        return CAN1_ExchangeSingle(6U,q,length,3U,reply);
    }
#endif
#endif
    if(LIFT_RULER_SERVICE_ENABLED && address==5U && reply_length==3U &&
       ((length==2U && q[0]==0x3bU && q[1]==0x6bU) ||
        (length==3U && q[0]==0x9cU && q[1]==0x48U && q[2]==0x6bU)))
        return CAN1_ExchangeSingle(5U,q,length,3U,reply);
    if(length==4U && q[0]==0xfe && q[1]==0x98 && q[2]==0 && q[3]==0x6b)
        return CAN1_ExchangeSingle(address,q,length,3U,reply);
#if APP_MOTOR_OFFLINE_ONLY
    return 9U;
#else
    if(g_safety_fault || g_rtos_fault || g_can_ctrl_fault_latched ||
       (LIFT_RULER_SERVICE_ENABLED && address==5U ? !LiftRuler_DebugCanAllowed(address,q,length) :
        (!g_startup_axes_vm_request || g_startup_axes_state<2U || g_startup_axes_state>4U)))return 9U;
    if(LIFT_RULER_SERVICE_ENABLED && address==5U && length==4U && reply_length==3U &&
       q[0]==0x9aU && q[1]==2U && q[2]==0U && q[3]==0x6bU)
        return CAN1_ExchangeSingle(5U,q,length,3U,reply);
    if(length==5U && q[0]==0xf3 && q[1]==0xab && q[2]==1 && q[3]==0 && q[4]==0x6b)
        return CAN1_ExchangeSingle(address,q,length,3U,reply);
    /* 2026-11-08: these steppers run Emm_V5/X firmware. Its 0xFD position command is
     * 11 bytes: [0xFD,dir,acc,speedH,speedL,p3,p2,p1,p0,sync,0x6B], distance in
     * pulses (3200/rev). The 15-byte frame below is the legacy ZDT layout and is kept
     * for node5 (lift ruler). */
    if(length==11U && q[0]==0xfdU && q[1]<=1U && q[10]==0x6bU && reply_length==3U)
    {
        uint32_t spd=((uint32_t)q[3]<<8)|q[4];
        if(!spd || spd>3000U || (!q[5] && !q[6] && !q[7] && !q[8]))return 7U;
    }
    else if(length!=15U || q[0]!=0xfd || q[1]>1 || q[12]!=2 || q[13]!=0 || q[14]!=0x6b || reply_length!=3U)return 7U;
    else
    {
        uint32_t speed=((uint32_t)q[6]<<8)|q[7];
        if(!speed || speed>(address==5U?18464U:1060U) || (!q[2] && !q[3]) || (!q[4] && !q[5]))return 7U;
    }
    if(!s_can1_normal_ready){r=CAN1_InitMode(CAN_Mode_Normal);if(r!=1U)return r;s_can1_normal_ready=1U;}
    for(flush=0;flush<3 && CAN_MessagePending(CAN1,CAN_FIFO0);flush++)StartupHome_Receive(&rx);
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
    /* 2026-11-08: 30ms was too tight for the Emm_V5 11-byte frame ack; the drive
     * answers after it has parsed the second frame. */
    while(xTaskGetTickCount()-start<pdMS_TO_TICKS(80U)) {
        if(CAN_MessagePending(CAN1,CAN_FIFO0)) {
            StartupHome_Receive(&rx);
            if(rx.IDE==CAN_Id_Extended && rx.RTR==CAN_RTR_Data && rx.ExtId==((uint32_t)address<<8) && rx.Data[0]==0xfd)
            { *reply=rx;return rx.DLC==3 && rx.Data[2]==0x6b ? 1U:6U; }
        }
        vTaskDelay(pdMS_TO_TICKS(1U));
    }return 5U;
#endif
}

uint32_t StartupAxis_Exchange(const uint8_t *q,uint8_t n,uint8_t reply_n,void *out)
{return VehicleArm_Exchange(6U,q,n,reply_n,out);}

#if (VEHICLE_EXTENSION_HOME_ONLY || VEHICLE_STARTUP_GROUP_ENABLE) && !APP_MOTOR_OFFLINE_ONLY
/* 复用原17字节三帧顺序读参，不写菜单参数。 */
uint32_t StartupAxis_ReadHomeParameters(uint8_t out[17])
{
    CanTxMsg tx;
    CanRxMsg rx;
    TickType_t started;
    uint32_t result;
    uint8_t mailbox, status, flush, packet = 0U, offset = 1U, chunk;
    if(!g_startup_axes_vm_request || g_startup_axes_state!=2U || g_safety_fault || g_rtos_fault || g_can_ctrl_fault_latched)return 9U;
    if (out == NULL) return 7U;
    memset(out, 0, 17U);

    if (s_can1_normal_ready == 0U)
    {
        result = CAN1_InitMode(CAN_Mode_Normal);
        if (result != 1U) return result;
        s_can1_normal_ready = 1U;
    }
    for (flush = 0U; flush < 3U && CAN_MessagePending(CAN1, CAN_FIFO0); flush++)
        StartupHome_Receive(&rx);
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
            StartupHome_Receive(&rx);
            if (rx.IDE == CAN_Id_Extended && rx.RTR == CAN_RTR_Data &&
                (rx.ExtId >> 8) == 6U && rx.DLC != 0U && rx.Data[0] == 0x22U)
            {

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
}

#endif

#if !APP_MOTOR_OFFLINE_ONLY
uint32_t LiftRuler_ReadHomeParameters(uint8_t out[17])
{
    CanTxMsg tx;
    CanRxMsg rx;
    TickType_t started;
    uint32_t result;
    uint8_t mailbox, status, flush, packet = 0U, offset = 1U, chunk;
    if(!LIFT_RULER_SERVICE_ENABLED || !g_lift_ruler_controller.vm_request || g_safety_fault || g_rtos_fault || g_can_ctrl_fault_latched)return 9U;
    if (out == NULL) return 7U;
    memset(out, 0, 17U);

    if (s_can1_normal_ready == 0U)
    {
        result = CAN1_InitMode(CAN_Mode_Normal);
        if (result != 1U) return result;
        s_can1_normal_ready = 1U;
    }
    for (flush = 0U; flush < 3U && CAN_MessagePending(CAN1, CAN_FIFO0); flush++)
        StartupHome_Receive(&rx);
    memset(&tx, 0, sizeof(tx));
    tx.ExtId = 0x0500U;
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
    while ((TickType_t)(xTaskGetTickCount() - started) < pdMS_TO_TICKS(CAN1_PARAM_REPLY_WAIT_MS))
    {
        if (CAN_MessagePending(CAN1, CAN_FIFO0))
        {
            StartupHome_Receive(&rx);
            if (rx.IDE == CAN_Id_Extended && rx.RTR == CAN_RTR_Data &&
                (rx.ExtId >> 8) == 5U && rx.DLC != 0U && rx.Data[0] == 0x22U)
            {

                chunk = (uint8_t)(17U - offset);
                if (chunk > 7U) chunk = 7U;
                if (rx.ExtId != (0x0500U | packet) || rx.DLC != (uint8_t)(chunk + 1U))
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
}
#else
uint32_t LiftRuler_ReadHomeParameters(uint8_t out[17]){(void)out;return 9U;}
#endif
