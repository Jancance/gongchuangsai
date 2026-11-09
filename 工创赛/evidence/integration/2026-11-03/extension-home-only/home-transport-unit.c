
#include <stdint.h>
#include <stdio.h>
#include <string.h>
#define VEHICLE_EXTENSION_HOME_ONLY 1
#define CAN1 1
#define CAN_FIFO0 0
#define CAN_Id_Extended 1
#define CAN_RTR_Data 0
#define CAN_Mode_Normal 0
#define CAN_TxStatus_NoMailBox 4
#define CAN_TxStatus_Ok 1
#define CAN_TxStatus_Failed 2
#define CAN1_SINGLE_TX_WAIT_MS 20
#define CAN1_SINGLE_REPLY_WAIT_MS 30
#define pdMS_TO_TICKS(x) (x)
typedef uint32_t TickType_t;
typedef struct {uint32_t ExtId,IDE,RTR;uint8_t DLC,Data[8];} CanRxMsg;
typedef CanRxMsg CanTxMsg;
uint32_t tick,s_can1_normal_ready,s_can_ctrl_trace_active,s_can_ctrl_trace_tick;
uint32_t g_can_ctrl_trace_flush_count,g_can_ctrl_trace_phase,g_can_ctrl_trace_tx_ms,g_can_ctrl_trace_rx_count;
CanRxMsg g_can_ctrl_trace_last_rx;
uint32_t g_startup_axes_vm_request=1,g_startup_axes_state=2,g_safety_fault,g_rtos_fault,g_can_ctrl_fault_latched;
volatile uint32_t g_extension_home_capture,g_extension_home_done_notice;
static CanRxMsg queue[20];static unsigned used,cursor,tx_count,packet_fault,command;
uint32_t xTaskGetTickCount(void){return tick++;}
void vTaskDelay(uint32_t n){tick+=n;}
uint32_t CAN1_InitMode(uint32_t mode){(void)mode;return 1;}
uint32_t CAN_MessagePending(uint32_t can,uint32_t fifo){(void)can;(void)fifo;return used-cursor;}
void CAN_Receive(uint32_t can,uint32_t fifo,CanRxMsg *p){(void)can;(void)fifo;*p=queue[cursor++];}
void CAN_CancelTransmit(uint32_t can,uint8_t box){(void)can;(void)box;}
uint8_t CAN_TransmitStatus(uint32_t can,uint8_t box){(void)can;(void)box;return CAN_TxStatus_Ok;}
static void notice(uint32_t id,uint32_t dlc,uint32_t tail){
 CanRxMsg p={0};p.ExtId=id;p.IDE=1;p.DLC=dlc;p.Data[0]=0x9a;p.Data[1]=0x9f;p.Data[2]=tail;queue[used++]=p;
}
uint8_t CAN_Transmit(uint32_t can,CanTxMsg *q){
 CanRxMsg p={0};(void)can;tx_count++;command=q->Data[0];
 if(command==0x22){
  uint8_t data[17]={0x22,0,0,0,30,0,0,0x27,0x10,1,0x2c,3,0x20,0,60,0,0x6b};unsigned offset=1;
  for(unsigned packet=0;packet<3;packet++){
   unsigned chunk=17-offset;if(chunk>7)chunk=7;
   memset(&p,0,sizeof(p));p.ExtId=0x600+packet;p.IDE=1;p.DLC=chunk+1;p.Data[0]=0x22;
   memcpy(p.Data+1,data+offset,chunk);offset+=chunk;
   if(packet_fault==1 && packet==1)p.ExtId=0x600;
   if(packet_fault==2 && packet==2)p.Data[chunk]=0;
   if(packet_fault==3 && packet==1)p.DLC--;
   queue[used++]=p;
  }
 }else{
  p.ExtId=q->ExtId;p.IDE=1;p.DLC=3;p.Data[0]=command;p.Data[1]=2;p.Data[2]=0x6b;
  if(command==0x9a && packet_fault==4){p.Data[1]=0x9f;}
  queue[used++]=p;
 }return 0;
}
static void StartupHome_Receive(CanRxMsg *rx)
{
    CAN_Receive(CAN1,CAN_FIFO0,rx);
#if VEHICLE_EXTENSION_HOME_ONLY
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
    
    if ((address == 0U) || (reply == NULL) || (request == NULL) ||
        (request_length < 2U) || (request_length > 8U) ||
        (reply_length < 2U) || (reply_length > 8U))
    {
        return 7U;
    }

    memset(reply, 0, sizeof(*reply));

    
    if (s_can1_normal_ready == 0U)
    {
        result = CAN1_InitMode(CAN_Mode_Normal);
        if (result != 1U)
        {
            return result;
        }
        s_can1_normal_ready = 1U;
    }

    
    for (flush_count = 0U; flush_count < 3U; flush_count++)
    {
        if (CAN_MessagePending(CAN1, CAN_FIFO0) == 0U)
        {
            break;
        }
        StartupHome_Receive(&rx);
        if (s_can_ctrl_trace_active) g_can_ctrl_trace_flush_count++;
    }

#if VEHICLE_EXTENSION_HOME_ONLY
    if(address==6U && request_length==4U && request[0]==0x9aU){
        extern volatile uint32_t g_extension_home_capture,g_extension_home_done_notice;
        g_extension_home_done_notice=0U;g_extension_home_capture=1U;
    }
#endif
    memset(&tx, 0, sizeof(tx));
    tx.ExtId = (uint32_t)address << 8;
    tx.IDE = CAN_Id_Extended;
    tx.RTR = CAN_RTR_Data;
    
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
                
                *reply = rx;

                
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
uint32_t VehicleArm_Exchange(uint8_t address,const uint8_t *q,uint8_t length,uint8_t reply_length,void *out)
{
    CanRxMsg *reply=(CanRxMsg*)out;
#if !APP_MOTOR_OFFLINE_ONLY
    CanRxMsg rx;CanTxMsg tx;uint8_t packet=0U,offset=1U,chunk,mailbox,status,flush;
    TickType_t start;uint32_t r;
#endif
    if(!q || !out || (address!=5U && address!=6U))return 7U;
    if(length==2U && q[1]==0x6b &&
       ((q[0]==0x1a && reply_length==4U) || (q[0]==0x36 && reply_length==7U) ||
        (q[0]==0x3a && reply_length==3U) || (q[0]==0x35 && reply_length==5U)))
        return CAN1_ExchangeSingle(address,q,length,reply_length,reply);
#if VEHICLE_EXTENSION_HOME_ONLY
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
    if(length==4U && q[0]==0xfe && q[1]==0x98 && q[2]==0 && q[3]==0x6b)
        return CAN1_ExchangeSingle(address,q,length,3U,reply);
#if APP_MOTOR_OFFLINE_ONLY
    return 9U;
#else
    if(!g_startup_axes_vm_request || g_startup_axes_state<2U || g_startup_axes_state>4U ||
       g_safety_fault || g_rtos_fault || g_can_ctrl_fault_latched)return 9U;
    if(length==5U && q[0]==0xf3 && q[1]==0xab && q[2]==1 && q[3]==0 && q[4]==0x6b)
        return CAN1_ExchangeSingle(address,q,length,3U,reply);
    if(length!=15U || q[0]!=0xfd || q[1]>1 || q[12]!=2 || q[13]!=0 || q[14]!=0x6b || reply_length!=3U)return 7U;
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
    while(xTaskGetTickCount()-start<pdMS_TO_TICKS(30U)) {
        if(CAN_MessagePending(CAN1,CAN_FIFO0)) {
            StartupHome_Receive(&rx);
            if(rx.IDE==CAN_Id_Extended && rx.RTR==CAN_RTR_Data && rx.ExtId==((uint32_t)address<<8) && rx.Data[0]==0xfd)
            { *reply=rx;return rx.DLC==3 && rx.Data[2]==0x6b ? 1U:6U; }
        }
        vTaskDelay(pdMS_TO_TICKS(1U));
    }return 5U;
#endif
}
#if !APP_MOTOR_OFFLINE_ONLY
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

static unsigned checks,failures;
static void ck(unsigned ok){checks++;if(!ok){failures++;printf("failed check %u\n",checks);}}
static void reset(void){used=cursor=tx_count=packet_fault=tick=0;g_extension_home_capture=g_extension_home_done_notice=0;g_startup_axes_vm_request=1;g_startup_axes_state=2;g_safety_fault=0;}
int main(void){
 CanRxMsg reply;uint8_t out[17],home[4]={0x9a,2,0,0x6b},abort[3]={0x9c,0x48,0x6b};
#if !APP_MOTOR_OFFLINE_ONLY
 reset();ck(StartupAxis_ReadHomeParameters(out)==1 && out[4]==30 && out[16]==0x6b);
 for(unsigned i=1;i<=3;i++){reset();packet_fault=i;ck(StartupAxis_ReadHomeParameters(out)==6);}
 reset();g_startup_axes_vm_request=0;ck(StartupAxis_ReadHomeParameters(out)==9 && !tx_count);
 reset();notice(0x600,3,0x6b);ck(VehicleArm_Exchange(6,home,4,3,&reply)==1);
 ck(!g_extension_home_done_notice && g_extension_home_capture && tx_count==1);
 reset();packet_fault=4;ck(VehicleArm_Exchange(6,home,4,3,&reply)==1 && g_extension_home_done_notice);
 reset();g_startup_axes_vm_request=0;ck(VehicleArm_Exchange(6,home,4,3,&reply)==9 && !tx_count);
 reset();g_safety_fault=1;ck(VehicleArm_Exchange(6,home,4,3,&reply)==9 && !tx_count);
#else
 reset();ck(VehicleArm_Exchange(6,home,4,3,&reply)!=1 && !tx_count);
#endif
 reset();ck(VehicleArm_Exchange(5,home,4,3,&reply)!=1 && !tx_count);
 ck(VehicleArm_Exchange(1,home,4,3,&reply)!=1 && !tx_count);
 home[1]=1;ck(VehicleArm_Exchange(6,home,4,3,&reply)!=1 && !tx_count);home[1]=2;
 reset();g_startup_axes_vm_request=0;ck(VehicleArm_Exchange(6,abort,3,3,&reply)==1);
 for(unsigned i=0;i<4;i++){
  reset();g_extension_home_capture=1;notice(i==0?0x500:0x600,i==1?2:3,i==2?0:0x6b);
  StartupHome_Receive(&reply);ck(g_extension_home_done_notice==(i==3));
 }
 printf("home transport checks=%u failures=%u\n",checks,failures);return failures?1:0;
}
