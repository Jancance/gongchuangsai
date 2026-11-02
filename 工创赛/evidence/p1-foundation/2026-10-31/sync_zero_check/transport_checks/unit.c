#include <stdint.h>
#include <string.h>
typedef uint32_t TickType_t;
typedef struct{uint32_t StdId,ExtId;uint8_t IDE,RTR,DLC,Data[8];} CanRxMsg;
typedef CanRxMsg CanTxMsg;
typedef struct{uint32_t ODR;} GpioMock;
typedef struct{uint32_t ESR;} CanMock;
typedef struct{uint32_t active,enable_seen,enable_tick,enable_count;} PowerCommandStatus;
static GpioMock gpio={4};static CanMock can={0};
#define GPIOE (&gpio)
#define CAN1 (&can)
#define GPIO_Pin_2 4U
#define CAN_Id_Extended 1U
#define CAN_RTR_Data 0U
#define CAN_Mode_Normal 0U
#define CAN_FIFO0 0U
#define CAN_TxStatus_NoMailBox 4U
#define CAN_TxStatus_Ok 1U
#define CAN_TxStatus_Failed 2U
#define CAN1_SINGLE_TX_WAIT_MS 20U
#define CAN1_SINGLE_REPLY_WAIT_MS 30U
#define portTICK_PERIOD_MS 1U
#define pdMS_TO_TICKS(x) (x)
static uint32_t tick,depth,checks,failure,permit,claim_calls,tx_calls,rx_pending,rx_mode,tx_mode;
static uint32_t claim_session,claim_generation;
static CanTxMsg last_tx;static CanRxMsg next_rx;
volatile uint32_t g_can_guard_ready=1;
volatile uint32_t g_can_poll_result[4],g_can_poll_flags[4],g_can_poll_count[4];
volatile TickType_t g_can_poll_tick[4];
#define CHECK(x) do{checks++;if(!(x)&&!failure)failure=__LINE__;}while(0)
#define taskENTER_CRITICAL() (++depth)
#define taskEXIT_CRITICAL() (--depth)
static TickType_t xTaskGetTickCount(void){return tick;}
static void vTaskDelay(uint32_t n){CHECK(!depth);tick+=n;}
static uint32_t CAN1_InitMode(uint8_t mode){(void)mode;return 1;}
static void Power_ReadCommandStatus(PowerCommandStatus *s){memset(s,0,sizeof(*s));s->active=s->enable_seen=1;}
static uint8_t CAN_Transmit(CanMock *c,CanTxMsg *t){
 (void)c;tx_calls++;last_tx=*t;
 CHECK(depth==(t->ExtId==0?1U:0U));
 memset(&next_rx,0,sizeof(next_rx));next_rx.IDE=1;next_rx.ExtId=t->ExtId?t->ExtId:0x100U;
 next_rx.DLC=3;next_rx.Data[0]=t->Data[0];next_rx.Data[1]=2;next_rx.Data[2]=0x6b;
 if(rx_mode==1)next_rx.ExtId=0x200;
 if(rx_mode==2)next_rx.Data[2]=0;
 if(rx_mode==3)next_rx.Data[1]=0xe2;
 if(rx_mode==4)next_rx.DLC=2;
 if(rx_mode==5)next_rx.ExtId=0;
 rx_pending=rx_mode==6?0:1;
 return tx_mode==2?4:0;
}
static uint8_t CAN_TransmitStatus(CanMock *c,uint8_t mailbox){(void)c;(void)mailbox;CHECK(!depth);return tx_mode==1?2:1;}
static uint32_t CAN_MessagePending(CanMock *c,uint8_t fifo){(void)c;(void)fifo;return rx_pending;}
static void CAN_Receive(CanMock *c,uint8_t fifo,CanRxMsg *r){(void)c;(void)fifo;*r=next_rx;rx_pending=0;}
static void CAN_CancelTransmit(CanMock *c,uint8_t mailbox){(void)c;(void)mailbox;CHECK(!depth);}
static uint32_t ChassisAuto_SyncTriggerClaim(uint32_t session,uint32_t generation){
 CHECK(depth==1);claim_calls++;claim_session=session;claim_generation=generation;return permit;
}
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

/* 当前所有CAN调用都来自任务A，不允许其他任务并发使用 */
static uint32_t s_can1_normal_ready = 0U;
/* 标记下一次查询需重新初始化，不在这里发送或切换硬件模式 */

static uint32_t s_sync_trigger_session,s_sync_trigger_generation;

void CAN1_ResetSession(void)
{
    s_can1_normal_ready = 0U;
}
static uint32_t CAN1_ExchangeDirected(uint8_t address,uint8_t reply_address,
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
    if ((reply_address == 0U) || (reply == NULL) || (request == NULL) ||
        (address==0U && !(reply_address==1U && request_length==3U &&
          request[0]==0xFFU && request[1]==0x66U && request[2]==0x6BU)) ||
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
        CAN_Receive(CAN1, CAN_FIFO0, &rx);
        if (s_can_ctrl_trace_active) g_can_ctrl_trace_flush_count++;
    }

    memset(&tx, 0, sizeof(tx));
    tx.ExtId = (uint32_t)address << 8;
    tx.IDE = CAN_Id_Extended;
    tx.RTR = CAN_RTR_Data;
    /* 按实际请求长度复制，包括功能码和末尾校验字节 */
    tx.DLC = request_length;
    memcpy(tx.Data, request, request_length);

    if (s_can_ctrl_trace_active) g_can_ctrl_trace_phase = 2U;
    if (address==0U) {
        /* 只把一次短邮箱提交放进临界区；等待应答仍可被Safety抢占。 */
        taskENTER_CRITICAL();
        if(!ChassisAuto_SyncTriggerClaim(s_sync_trigger_session,s_sync_trigger_generation)) {
            taskEXIT_CRITICAL();return 9U;
        }
        mailbox=CAN_Transmit(CAN1,&tx);
        taskEXIT_CRITICAL();
    } else mailbox = CAN_Transmit(CAN1, &tx);
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
            CAN_Receive(CAN1, CAN_FIFO0, &rx);
            if (s_can_ctrl_trace_active)
            {
                g_can_ctrl_trace_rx_count++;
                g_can_ctrl_trace_last_rx = rx;
            }

            if ((rx.IDE == CAN_Id_Extended) &&
                (rx.RTR == CAN_RTR_Data) &&
                (rx.ExtId == ((uint32_t)reply_address << 8)))
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
static uint32_t CAN1_ExchangeSingle(uint8_t address,const uint8_t *request,
    uint8_t request_length,uint8_t reply_length,CanRxMsg *reply)
{
    if(!address)return 7U;
    return CAN1_ExchangeDirected(address,address,request,request_length,reply_length,reply);
}
static uint32_t CAN1_ControlSingle(uint8_t address,
                                   const uint8_t *request,
                                   uint8_t request_length,
                                   CanRxMsg *reply)
{
    uint32_t result, trace, i;
    /* 零速专用镜像：在公共控制出口硬拒绝任何非零F6。 */
    if(request && request_length==7U && request[0]==0xF6U &&
       (request[2]!=0U || request[3]!=0U))return 7U;
    TickType_t context_tick;
    PowerCommandStatus power_status;
    CanRxMsg empty = {0};

    /* 只允许四轮地址，不允许广播 */
    if ((reply == NULL) ||
        (((address < 1U) || (address > 4U)) && !(address==0U && request &&
          request_length==3U && request[0]==0xFFU && request[1]==0x66U && request[2]==0x6BU)))
    {
        return 7U;
    }

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
    result = CAN1_ExchangeDirected(address,address ? address : 1U,request,
                                 request_length,3U,reply);

    if ((result == 1U) && (reply->Data[1] != 0x02U))
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
uint32_t CAN1_StageEmmSpeedRaw(uint8_t address,uint8_t direction,
    uint16_t speed_raw,uint8_t acceleration,CanRxMsg *reply)
{
    uint8_t request[7]={0xF6U,0U,0U,0U,0U,1U,0x6BU};
    if(direction>1U || speed_raw>3000U)return 7U;
    request[1]=direction;request[2]=(uint8_t)(speed_raw>>8);
    request[3]=(uint8_t)speed_raw;request[4]=acceleration;
    return CAN1_ControlSingle(address,request,7U,reply);
}
uint32_t CAN1_TriggerSync(uint32_t session,uint32_t generation,CanRxMsg *reply)
{
    uint8_t request[3]={0xFFU,0x66U,0x6BU};
    s_sync_trigger_session=session;s_sync_trigger_generation=generation;
    return CAN1_ControlSingle(0U,request,3U,reply);
}
uint32_t CAN1_StopMotor(uint8_t address, CanRxMsg *reply)
{
    uint8_t request[4] = {0xFEU, 0x98U, 0U, 0x6BU};

    return CAN1_ControlSingle(address, request, 4U, reply);
}
uint32_t CAN1_SetEmmSpeedRaw(uint8_t address, uint8_t direction,
                             uint16_t speed_raw, uint8_t acceleration,
                             CanRxMsg *reply)
{
    uint8_t request[7] = {
        0xF6U, 0U, 0U, 0U, 0U, 0U, 0x6BU
    };

    /* 3000是协议输入上限，不是整车允许的安全速度 */
    if ((direction > 1U) || (speed_raw > 3000U))
    {
        return 7U;
    }

    request[1] = direction;
    request[2] = (uint8_t)(speed_raw >> 8);
    request[3] = (uint8_t)speed_raw;
    request[4] = acceleration;

    /* 加速度0有直接启动含义，实际运动参数后续单独确定 */
    return CAN1_ControlSingle(address, request, 7U, reply);
}
static void reset(void){
 tick=100;depth=tx_calls=claim_calls=rx_pending=rx_mode=tx_mode=0;permit=1;
 g_can_ctrl_fault_latched=0;s_can_ctrl_trace_active=0;s_can1_normal_ready=1;
}
uint32_t test_entry(void){
 uint32_t a,m;CanRxMsg reply;
 for(a=1;a<=4;a++) {
  reset();CHECK(CAN1_StageEmmSpeedRaw(a,a&1U,0,0,&reply)==1);
  CHECK(tx_calls==1&&!claim_calls);CHECK(last_tx.ExtId==(a<<8));CHECK(last_tx.DLC==7);
  CHECK(last_tx.Data[0]==0xf6&&last_tx.Data[1]==(a&1U)&&last_tx.Data[2]==0&&last_tx.Data[3]==0&&last_tx.Data[4]==0&&last_tx.Data[5]==1&&last_tx.Data[6]==0x6b);
 }
 reset();CHECK(CAN1_StageEmmSpeedRaw(0,0,1,0,&reply)==7);CHECK(!tx_calls);
 reset();CHECK(CAN1_StageEmmSpeedRaw(1,2,1,0,&reply)==7);CHECK(!tx_calls);
 reset();CHECK(CAN1_StageEmmSpeedRaw(1,0,3001,0,&reply)==7);CHECK(!tx_calls);
 reset();CHECK(CAN1_TriggerSync(17,23,&reply)==1);CHECK(tx_calls==1&&claim_calls==1&&claim_session==17&&claim_generation==23);
 CHECK(last_tx.ExtId==0&&last_tx.DLC==3&&last_tx.Data[0]==0xff&&last_tx.Data[1]==0x66&&last_tx.Data[2]==0x6b);
 reset();permit=0;CHECK(CAN1_TriggerSync(17,23,&reply)==9);CHECK(!tx_calls&&claim_calls==1&&g_can_ctrl_fault_latched);
 for(m=1;m<=6;m++){
  reset();rx_mode=m;CHECK(CAN1_TriggerSync(17,23,&reply)!=1);CHECK(tx_calls==1&&g_can_ctrl_fault_latched);
 }
 reset();tx_mode=1;CHECK(CAN1_TriggerSync(17,23,&reply)==4);CHECK(tx_calls==1);
 reset();tx_mode=2;CHECK(CAN1_TriggerSync(17,23,&reply)==4);CHECK(tx_calls==1);
 reset();CHECK(CAN1_StopMotor(2,&reply)==1);CHECK(last_tx.ExtId==0x200&&last_tx.Data[2]==0&&!claim_calls);
 reset();CHECK(CAN1_SetEmmSpeedRaw(3,0,51,0,&reply)==7);CHECK(!tx_calls);
 reset();CHECK(CAN1_StageEmmSpeedRaw(1,0,1,0,&reply)==7);CHECK(!tx_calls);
 reset();CHECK(CAN1_SetEmmSpeedRaw(3,0,0,0,&reply)==1);CHECK(last_tx.Data[5]==0);
 CHECK(!depth);
 return failure?(0x80000000U|failure):checks;
}
