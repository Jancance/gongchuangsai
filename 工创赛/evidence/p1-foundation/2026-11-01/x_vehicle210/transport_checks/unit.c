#include <stdint.h>
#define CHASSIS_WHEEL_COUNT 4U
#include <string.h>
#include "src/zdt_x_protocol.h"
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
static uint32_t native_speed=127,native_sign=0,native_position=3600;
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
 if(t->Data[0]==0x35U) {
  next_rx.DLC=5;next_rx.Data[1]=native_sign;next_rx.Data[2]=(uint8_t)(native_speed>>8);next_rx.Data[3]=(uint8_t)native_speed;next_rx.Data[4]=0x6b;
 }
 if(t->Data[0]==0x36U) {
  next_rx.DLC=7;next_rx.Data[1]=native_sign;next_rx.Data[2]=(uint8_t)(native_position>>24);next_rx.Data[3]=(uint8_t)(native_position>>16);next_rx.Data[4]=(uint8_t)(native_position>>8);next_rx.Data[5]=(uint8_t)native_position;next_rx.Data[6]=0x6b;
 }
 if(rx_mode==1)next_rx.ExtId=0x200;
 if(rx_mode==2)next_rx.Data[next_rx.DLC-1]=0;
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
            CAN_Receive(CAN1, CAN_FIFO0, &rx);
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

static uint32_t CAN1_ReadParameter(uint8_t address,
                                  uint8_t command,
                                  uint8_t reply_length,
                                  CanRxMsg *reply)
{
    uint8_t request[2] = {command, 0x6BU};

    return CAN1_ExchangeSingle(address, request, 2U,
                               reply_length, reply);
}

uint32_t CAN1_ReadSpeed(uint8_t address, CanRxMsg *reply, float *rpm)
{
 uint32_t result;
 if(!rpm || !reply)return 7U;*rpm=0.0f;
 result=CAN1_ReadParameter(address,0x35U,5U,reply);
 if(result==1U && !ZdtX_DecodeSpeed(reply->Data,reply->DLC,rpm))result=6U;
 if(result!=1U)CAN1_ResetSession();return result;
}

uint32_t CAN1_ReadPosition(uint8_t address,CanRxMsg *reply,uint8_t *sign,uint32_t *raw)
{
 uint32_t result;
 if(!sign || !raw || !reply)return 7U;*sign=0U;*raw=0U;
 result=CAN1_ReadParameter(address,0x36U,7U,reply);
 if(result==1U && !ZdtX_DecodePosition(reply->Data,reply->DLC,sign,raw))result=6U;
 if(result!=1U)CAN1_ResetSession();return result;
}

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

#if APP_X_OFFLINE_ONLY
    /* 离线候选拒绝启用驱动和所有非零速度；STOP和读取不受影响。 */
    if(request && ((request_length==5U && request[0]==0xF3U && request[2]) ||
       (request_length==8U && request[0]==0xF6U && (request[4] || request[5]))))return 7U;
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

    if ((result == 1U) && !ZdtX_DecodeAck(request[0],reply->Data,reply->DLC))
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

uint32_t CAN1_SetMotorEnable(uint8_t address, uint8_t enabled,
                             CanRxMsg *reply)
{
    uint8_t request[5] = {0xF3U, 0xABU, 0U, 0U, 0x6BU};

    if (enabled > 1U)
    {
        return 7U;
    }

    request[2] = enabled;

    return CAN1_ControlSingle(address, request, 5U, reply);
}

uint32_t CAN1_StopMotor(uint8_t address, CanRxMsg *reply)
{
    uint8_t request[4] = {0xFEU, 0x98U, 0U, 0x6BU};

    return CAN1_ControlSingle(address, request, 4U, reply);
}

static uint32_t CAN1_XSpeed(uint8_t address,uint8_t direction,uint16_t speed_raw,
    uint16_t acceleration,uint8_t sync,CanRxMsg *reply)
{
 uint8_t request[8];
 if(speed_raw>ZDT_X_APP_MAX_SPEED_RAW || !ZdtX_EncodeSpeed(direction,speed_raw,acceleration,sync,request))return 7U;
 return CAN1_ControlSingle(address,request,8U,reply);
}

uint32_t CAN1_SetXSpeedRaw(uint8_t address,uint8_t direction,uint16_t speed_raw,
    uint16_t acceleration,CanRxMsg *reply)
{ return CAN1_XSpeed(address,direction,speed_raw,acceleration,0U,reply); }

uint32_t CAN1_StageXSpeedRaw(uint8_t address,uint8_t direction,uint16_t speed_raw,
    uint16_t acceleration,CanRxMsg *reply)
{ return CAN1_XSpeed(address,direction,speed_raw,acceleration,1U,reply); }
volatile float g_chassis_auto_driver_rpm[4];
static uint32_t Auto_ZeroSpeedLocked(void)
{
    uint32_t i;
    for (i = 0U; i < 4U; i++) if (g_chassis_auto_driver_rpm[i] != 0) return 0U;
    return 1U;
}
#if APP_X_OFFLINE_ONLY
static void Power_Disable(void){gpio.ODR &= ~4U;}
void Power_Enable(void)
{
#if APP_X_OFFLINE_ONLY
    Power_Disable();return;
#else
#if APP_UART_LINK_TEST
    /* 串口测试不能通过手改请求或接收字符串开启VM。 */
    Power_Disable();
    return;
#else
    taskENTER_CRITICAL();
    if ((s_vm_command_active == 0U) || ((GPIOE->ODR & GPIO_Pin_2) == 0U))
    {
        GPIO_SetBits(GPIOE, GPIO_Pin_2);
        s_vm_enable_tick = xTaskGetTickCount();
        s_vm_enable_seen = 1U;
        s_vm_enable_count++;
        s_vm_command_active = 1U;
    }
    else GPIO_SetBits(GPIOE, GPIO_Pin_2);
    taskEXIT_CRITICAL();
#endif
#endif
}
static uint32_t App_SerialIdleReady(uint32_t now)
{
#if APP_X_OFFLINE_ONLY
    (void)now;return 0U;
#else
    ChassisAutoStatus status;
    PowerCommandStatus power;
    (void)ChassisAuto_ReadStatus(&status);
    Power_ReadCommandStatus(&power);
#if APP_TEST_MODE == 3U
    if ((g_combined_test.state != 0U) || (g_combined_test.request != 0U) ||
        (g_motion_plan_valid != 1U)) return 0U;
#endif
    return ((g_forward_hold.phase == HR_IDLE) && (g_forward_hold.request == 0U) &&
        (g_vm_test_request == 0U) && (power.active == 0U) &&
        (status.active == 0U) && (status.result == 0U) && (status.cut_vm == 0U) &&
        (g_translation_hold_selftest == 1U) && (s_serial_proto_ok == 1U) &&
        App_HeadingRuntimeSource(now) && App_HeadingRuntimeExclusive() &&
        (g_can_ctrl_fault_latched == 0U) &&
        (s_heading_source.rate_dps >= -3.0f && s_heading_source.rate_dps <= 3.0f)) ? 1U : 0U;
#endif
}
#endif
static void fresh(void){tick=depth=rx_mode=tx_mode=rx_pending=0;tx_calls=0;g_can_ctrl_fault_latched=0;gpio.ODR=4;native_speed=127;native_sign=0;native_position=3600;CAN1_ResetSession();}
static float ab(float x){return x<0?-x:x;}
uint32_t test_entry(void){
 uint8_t bytes[8],sign=0;uint32_t counts=0,j;float rpm=0;CanRxMsg reply;
 CHECK(ZdtX_EncodeSpeed(1,123,1000,0,bytes));
 CHECK(bytes[0]==0xf6 && bytes[1]==1 && bytes[2]==3 && bytes[3]==0xe8 && bytes[4]==0 && bytes[5]==123 && bytes[6]==0 && bytes[7]==0x6b);
 CHECK(ZdtX_EncodeSpeed(0,30000,65535,1,bytes));CHECK(bytes[2]==255 && bytes[3]==255 && bytes[4]==0x75 && bytes[5]==0x30 && bytes[6]==1);
 CHECK(!ZdtX_EncodeSpeed(2,1,0,0,bytes));CHECK(!ZdtX_EncodeSpeed(0,30001,0,0,bytes));CHECK(!ZdtX_EncodeSpeed(0,1,0,2,bytes));CHECK(!ZdtX_EncodeSpeed(0,1,0,0,0));
 CHECK(ZdtX_EncodeSyncTrigger(bytes));CHECK(bytes[0]==0xff && bytes[1]==0x66 && bytes[2]==0x6b);
 for(j=0;j<256;j++){uint8_t ack[3]={0xf6,(uint8_t)j,0x6b};CHECK(ZdtX_DecodeAck(0xf6,ack,3)==(j==2));}
 fresh();CHECK(CAN1_ReadSpeed(1,&reply,&rpm)==1);CHECK(ab(rpm-12.7f)<.0001f);native_sign=1;CHECK(CAN1_ReadSpeed(1,&reply,&rpm)==1);CHECK(ab(rpm+12.7f)<.0001f);
 native_speed=1;CHECK(CAN1_ReadSpeed(1,&reply,&rpm)==1);CHECK(ab(rpm+.1f)<.0001f);native_sign=2;CHECK(CAN1_ReadSpeed(1,&reply,&rpm)==6);CHECK(rpm==0);
 fresh();CHECK(CAN1_ReadPosition(1,&reply,&sign,&counts)==1);CHECK(sign==0 && counts==65536);
 native_sign=1;native_position=1800;CHECK(CAN1_ReadPosition(1,&reply,&sign,&counts)==1);CHECK(sign==1 && counts==32768);
 native_position=0xffffffffU;CHECK(CAN1_ReadPosition(1,&reply,&sign,&counts)==6);CHECK(counts==0);
 fresh();rx_mode=2;CHECK(CAN1_ReadSpeed(1,&reply,&rpm)!=1);
 fresh();rx_mode=1;CHECK(CAN1_ReadSpeed(1,&reply,&rpm)!=1);
 fresh();rx_mode=6;CHECK(CAN1_ReadSpeed(1,&reply,&rpm)!=1);
 fresh();CHECK(CAN1_SetXSpeedRaw(0,0,10,0,&reply)==7);CHECK(tx_calls==0);CHECK(CAN1_SetXSpeedRaw(1,0,3001,0,&reply)==7);CHECK(tx_calls==0);
#if APP_X_OFFLINE_ONLY
 fresh();CHECK(CAN1_SetMotorEnable(1,1,&reply)==7);CHECK(tx_calls==0);
 CHECK(CAN1_SetXSpeedRaw(1,0,1,0,&reply)==7);CHECK(tx_calls==0);
 CHECK(CAN1_StageXSpeedRaw(1,0,1,0,&reply)==7);CHECK(tx_calls==0);
 CHECK(CAN1_SetXSpeedRaw(1,0,0,0,&reply)==1);CHECK(last_tx.DLC==8);
#else
 fresh();CHECK(CAN1_SetMotorEnable(1,1,&reply)==1);CHECK(last_tx.DLC==5 && last_tx.Data[2]==1);
 fresh();CHECK(CAN1_SetXSpeedRaw(2,1,123,1000,&reply)==1);CHECK(last_tx.DLC==8 && last_tx.ExtId==0x200 && last_tx.Data[2]==3 && last_tx.Data[3]==0xe8 && last_tx.Data[5]==123 && last_tx.Data[6]==0);
 fresh();CHECK(CAN1_StageXSpeedRaw(4,0,75,600,&reply)==1);CHECK(last_tx.DLC==8 && last_tx.Data[2]==2 && last_tx.Data[3]==0x58 && last_tx.Data[5]==75 && last_tx.Data[6]==1);
 fresh();rx_mode=3;CHECK(CAN1_SetXSpeedRaw(1,0,10,0,&reply)==8);CHECK(g_can_ctrl_fault_latched);
 rx_mode=0;CHECK(CAN1_StopMotor(1,&reply)==1);CHECK(g_can_ctrl_fault_latched);
#endif
 fresh();CHECK(CAN1_StopMotor(3,&reply)==1);CHECK(last_tx.DLC==4 && last_tx.Data[0]==0xfe && last_tx.Data[1]==0x98 && last_tx.Data[2]==0);
 CHECK(CAN1_SetMotorEnable(3,0,&reply)==1);CHECK(last_tx.Data[2]==0);
 CHECK(Auto_ZeroSpeedLocked());g_chassis_auto_driver_rpm[2]=.1f;CHECK(!Auto_ZeroSpeedLocked());g_chassis_auto_driver_rpm[2]=-.1f;CHECK(!Auto_ZeroSpeedLocked());g_chassis_auto_driver_rpm[2]=0;CHECK(Auto_ZeroSpeedLocked());
#if APP_X_OFFLINE_ONLY
 gpio.ODR=4;Power_Enable();CHECK(!(gpio.ODR&4));CHECK(!App_SerialIdleReady(0));
#endif
 CHECK(ZdtX_RpmDeci(5.1f)==51);CHECK(ZdtX_RpmDeci(-.1f)==-1);
 return failure?(0x80000000U|failure):checks;
}
