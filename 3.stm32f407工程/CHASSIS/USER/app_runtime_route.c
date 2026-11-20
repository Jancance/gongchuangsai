#include "app_runtime_route.h"
#include "app_motion_runtime.h"
#include "app_serial_internal.h"
#include "app_translation_runtime.h"
#include "app_can_diagnostics.h"
#include "FreeRTOS.h"
#include "task.h"
#include <stdio.h>
#if APP_SERIAL_MOTION_CONTROL
static RuntimeRoute s_runtime_route;
static RuntimeRoutePrepared s_runtime_prepared;
static uint32_t s_route_ack_pending,s_route_ack_id,s_route_ack_op,s_route_ack_result,s_route_ack_generation;
/* 路线上传不启动电机；只能初始待机或正常完成且断VM后修改。 */
uint32_t App_RuntimeRouteEditable(uint32_t now)
{
    if(g_serial_motion_state==0U && App_SerialIdleReady(now))return 1U;
    return App_SerialCompletedStopped() && !(*AppMotion_Live()->session_open) &&
        !g_heading_exec_trip && !g_forward_hold.trip &&
        App_HeadingRuntimeSource(now) && App_HeadingRuntimeExclusive() &&
        !g_serial_motion_link_lost && !g_can_ctrl_fault_latched;
}
void App_RuntimeRouteCommand(const char *line)
{
    RuntimeRouteMessage m;uint32_t result,now=(uint32_t)xTaskGetTickCount();
    result=RuntimeRoute_Decode(line,&m);
    if(result==RQ_OK) {
        taskENTER_CRITICAL();
        if(m.op==RQ_GO) {
            if(m.seq!=(*AppSerial_Live()->token_seq) || m.tick!=(*AppSerial_Live()->token_tick))result=RQ_STALE;
            else if((g_serial_motion_state==2U || App_SerialCompletedStopped()) &&
                    s_runtime_route.consumed && s_runtime_route.committed.id==m.id &&
                    s_runtime_route.committed.digest==m.digest)result=RQ_DUPLICATE;
            else if(g_serial_motion_state!=1U || (uint32_t)(now-(*AppSerial_Live()->arm_ms))>=2000U ||
                    !App_SerialIdleReady(now))result=RQ_BUSY;
            else {
                result=RuntimeRoute_Start(&s_runtime_route,m.id,m.digest);
                if(result==RQ_OK) {
                    AppMotion_SetStartPermit(1U);AppSerial_SetKeepalive(now);
                    g_combined_test.request=1U;
                    g_serial_motion_state=2U;g_serial_motion_result=2U;
                    g_serial_motion_accepted++;
                }
            }
        } else if(!(*AppSerial_Live()->tx_valid) || !(*AppSerial_Live()->tx_seq) ||
                  (uint32_t)((*AppSerial_Live()->tx_seq)-1U-m.seq)>5U || (uint32_t)(now-m.tick)>=500U)
            result=RQ_STALE;
        else if(!App_RuntimeRouteEditable(now))result=RQ_BUSY;
        else if(m.op!=RQ_COMMIT)result=RuntimeRoute_Stage(&s_runtime_route,&m,now);
        taskEXIT_CRITICAL();
        if(m.op==RQ_COMMIT && result==RQ_OK) {
            /* 浮点轨迹转换在临界区外完成，不阻塞独立安全任务。 */
            result=RuntimeRoute_Prepare(&s_runtime_route,&m,now,&s_runtime_prepared);
            if(result==RQ_OK) {
                taskENTER_CRITICAL();now=(uint32_t)xTaskGetTickCount();
                if(!s_runtime_route.staging || m.id!=s_runtime_route.stage_id ||
                   (uint32_t)(now-s_runtime_route.stage_ms)>=RQ_UPLOAD_MS)result=RQ_EXPIRED;
                else if(!App_RuntimeRouteInstall(&s_runtime_prepared,now))result=RQ_BUSY;
                taskEXIT_CRITICAL();
            }
        }
    }
    taskENTER_CRITICAL();
    s_runtime_route.last_result=result;
    s_route_ack_id=m.id;s_route_ack_op=m.op;s_route_ack_result=result;s_route_ack_pending=1U;s_route_ack_generation++;
    taskEXIT_CRITICAL();
}
/* RA1/RS1只反馈路线上传和执行，不充当ARM/GO/KEEP健康令牌。 */
void App_RuntimeRouteTelemetry(void)
{
    static uint32_t last;static char frame[256];
    uint32_t now=(uint32_t)xTaskGetTickCount(),used,values[12],ack,generation,i;int count;
    int32_t progress=0;uint16_t crc;ChassisAutoStatus status;
    taskENTER_CRITICAL();RuntimeRoute_Tick(&s_runtime_route,now);taskEXIT_CRITICAL();
    if(!App_SerialDiagnosticWindow(now,20U))return;
    taskENTER_CRITICAL();
    ack=s_route_ack_pending;generation=s_route_ack_generation;
    if(ack) {
        values[0]=s_route_ack_id;values[1]=s_route_ack_op;values[2]=s_route_ack_result;
        values[3]=s_runtime_route.stage_mask;values[4]=s_runtime_route.committed.id;
        values[5]=s_runtime_route.committed.digest;values[6]=s_runtime_route.committed.count;
        values[7]=s_runtime_route.ready;values[8]=s_runtime_route.consumed;
    } else {
        if((uint32_t)(now-last)<200U || !s_runtime_route.committed.id) {
            taskEXIT_CRITICAL();return;
        }
        values[0]=s_runtime_route.committed.id;values[1]=s_runtime_route.committed.digest;
        values[2]=s_runtime_route.staging?6U:(g_combined_test.state?g_combined_test.state:
            (s_runtime_route.ready?1U:0U));
        if(g_combined_test.state==1U || g_combined_test.state==2U)values[2]=2U;
        values[3]=g_combined_test.index;values[4]=g_combined_test.completed;
        values[5]=s_runtime_route.committed.count;
        values[6]=(g_combined_test.state==1U)?now-(*AppMotion_Live()->stage_ms):0U;
        (void)ChassisAuto_ReadStatus(&status);
        if(!(*AppMotion_Live()->turn) && g_combined_test.state && status.distance_mode)
            progress=App_SerialDistanceMilli(status.distance_progress_mm);
        values[7]=0U;values[8]=g_combined_test.cause;values[9]=s_runtime_route.stage_id;
        values[10]=s_runtime_route.stage_mask;values[11]=s_runtime_route.last_result;
    }
    taskEXIT_CRITICAL();
    /* Format and enqueue outside the control critical section. */
    count=snprintf(frame,sizeof(frame),ack?"RA1,":"RS1,");
    if(count<0)return;
    used=(uint32_t)count;
    for(i=0U;i<(ack?9U:12U);i++) {
        if(!ack && i==7U)count=snprintf(frame+used,sizeof(frame)-used,"%ld,",(long)progress);
        else count=snprintf(frame+used,sizeof(frame)-used,"%lu,",(unsigned long)values[i]);
        if(count<0 || (uint32_t)count>=sizeof(frame)-used-6U)return;
        used+=(uint32_t)count;
    }
    crc=App_SerialCrc(frame,used);
    (void)snprintf(frame+used,sizeof(frame)-used,"%04X\r\n",crc);
    if(App_DebugSendString(frame)==1U) {
        if(ack) {
            taskENTER_CRITICAL();
            if(s_route_ack_generation==generation)s_route_ack_pending=0U;
            taskEXIT_CRITICAL();
        } else last=now;
    }
}
const RuntimeRoute *AppRuntimeRoute_Live(void) { return &s_runtime_route; }
void AppRuntimeRoute_Init(void) { RuntimeRoute_Init(&s_runtime_route); }
void AppRuntimeRoute_Install(const RuntimeRoutePrepared *prepared) { RuntimeRoute_Install(&s_runtime_route,prepared); }
#endif
