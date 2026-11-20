#include "radar_observer.h"
#include "app_config.h"
#include "radar_zone_config.h"
#include "radar_uart_port.h"
#include "radar_servo.h"
#include "radar_servo_config.h"
#include "radar_lift.h"
#include "FreeRTOS.h"
#include "task.h"
#include "stm32f4xx.h"
#include <string.h>
/* 状态数值见公开头文件；步数输出完成和等待结束均不是实际到位反馈。 */
static RadarZone2Config config,pending_config;
static RadarObserverStatus status,published_status;
static RadarObserverPath published_path;
static RadarObserverMap nav_map,published_map;
static RadarObserverPathReply nav_reply,published_reply;
static uint32_t scan_run,scan_layout,map_generation,link_generation;
static uint8_t link_ready,published_ready,lift_claimed,scan_only;
#if APP_ZONE2_NAV_MOTION_ENABLE && !APP_ZONE2_RADAR_CALIBRATION_TEST
static uint8_t idle_bad_event_ignored;
#endif
enum { CMD_SCAN=5U,CMD_PATH=6U };
static volatile uint8_t owner_busy;
static volatile uint8_t stop_pending;
static RadarObserverPath path;
static volatile uint8_t command,configure_pending,cancel_pending;
/* 独立机构单次邮箱：先填等待/预算，最后写request=1下或2上；0默认不动作。 */
volatile uint32_t g_radar_mechanism_request=0U,g_radar_mechanism_wait_ms=0U;
volatile uint32_t g_radar_mechanism_budget_ms=0U,g_radar_mechanism_interval_us=RADAR_LIFT_INTERVAL_US;
volatile uint32_t g_radar_mechanism_result=0U;
/* Zone 2 GRID_SCAN diagnostics, retained until the next scan. */
volatile int32_t g_radar_scan_submit_result=-1;
volatile radar_client_error_t g_radar_scan_first_error;
static uint8_t request_start,request_goal,calibrating,initialized,mechanism;
static uint32_t mechanism_wait,mechanism_budget,mechanism_interval;
static uint32_t leg_steps,leg_direction,leg_budget,leg_wait;
static uint32_t entered,lift_sequence,scan_sequence;
static uint8_t grid_done,calibration_done,path_done;
#if APP_ZONE2_RADAR_CALIBRATION_TEST
/* Keep the first calibration error through recovery for debugger readback. */
volatile int32_t g_radar_calibration_submit_result=-1;
volatile radar_client_error_t g_radar_calibration_first_error;
static uint8_t recovering;
#endif
static uint32_t now(void) { return (uint32_t)(xTaskGetTickCount()*portTICK_PERIOD_MS); }
static bool configured(const RadarZone2Config *c)
{
    return c && c->measured==1U && c->node_id>=1U && c->node_id<=254U &&
        (c->zone==0U || c->zone==2U ||
         (c->zone==1U && APP_ZONE1_RADAR_LIFT_CONFIRMED && c->node_id==RADAR_ZONE1_NODE_ID)) &&
        c->steps>=1U && c->steps<=RADAR_LIFT_LONG_MAX_STEPS && c->down_direction<=1U &&
        c->deploy_ms>0U && c->settle_ms>0U && c->stow_ms>0U &&
        c->stage_timeout_ms>c->deploy_ms && c->stage_timeout_ms>c->settle_ms &&
        c->stage_timeout_ms>c->stow_ms &&
        c->stage_timeout_ms>(uint32_t)(((uint64_t)c->steps*RADAR_LIFT_INTERVAL_US+999U)/1000U)+250U && c->stage_timeout_ms<0x80000000U;
}
/* 纯候选计算：H填实测整数半步/圈；不发动作、不生成measured配置。 */
bool RadarObserver_FourTurnCandidate(uint32_t half_steps_per_turn,RadarFourTurnCandidate *candidate)
{
    if(!candidate || half_steps_per_turn<1U || half_steps_per_turn>RADAR_LIFT_LONG_MAX_STEPS/4U)return false;
    candidate->steps=(uint16_t)(4U*half_steps_per_turn);
    candidate->nominal_leg_ms=(uint32_t)(((uint64_t)candidate->steps*RADAR_LIFT_INTERVAL_US+999U)/1000U);
    candidate->estimated=1U;candidate->measured=0U;return true;
}
static void invalidate(void)
{
    status.map_valid=0U;status.path_valid=0U;status.calibration_complete=0U;
    nav_map.valid=0U;
}
static bool same_map(const RadarObserverMapIdentity *a,const RadarObserverMapIdentity *b)
{
    return a->run_id==b->run_id && a->map_generation==b->map_generation &&
        a->layout_id==b->layout_id && a->link_generation==b->link_generation &&
        a->scan_sequence==b->scan_sequence && a->start_id==b->start_id &&
        a->node_id==b->node_id && a->zone==b->zone;
}
static void on_state(const radar_client_state_t *s)
{
    bool ready=s->ready && s->connected && s->selected_zone==config.zone && s->node_id==config.node_id;
    if(ready && !link_ready)link_generation++;
    if(!ready && link_ready && !mechanism && status.stage>0U && status.stage<8U && !status.error)
        status.error=5U;
    link_ready=ready;
    stop_pending=s->operation==RADAR_CLIENT_OPERATION_STOP;
    if(!ready)invalidate();
}
static void on_error(const radar_client_error_t *e)
{
#if APP_ZONE2_NAV_MOTION_ENABLE && !APP_ZONE2_RADAR_CALIBRATION_TEST
    if(scan_only && nav_map.valid && published_map.valid && status.stage==8U &&
       !owner_busy && link_ready && !idle_bad_event_ignored &&
       e->kind==RADAR_CLIENT_ERROR_BAD_EVENT && e->operation==RADAR_CLIENT_OPERATION_NONE) {
        idle_bad_event_ignored=1U;
        return;
    }
#endif
    if(scan_only && status.stage>0U && status.stage<7U && !g_radar_scan_first_error.kind)
        g_radar_scan_first_error=*e;
#if APP_ZONE2_RADAR_CALIBRATION_TEST
    if(calibrating && !recovering && status.stage>0U && status.stage<=4U &&
       !g_radar_calibration_first_error.kind)
        g_radar_calibration_first_error=*e;
#endif
    invalidate();
    if(!mechanism && status.stage>0U && status.stage<8U && !status.error)status.error=3U;
}
static void on_grid(const radar_client_grid_result_t *g)
{
    uint8_t i;uint32_t mask=0U;
    if(status.stage!=4U || calibrating || grid_done)return;
    if(g->count>RADAR_CLIENT_MAX_GRID_IDS){status.error=3U;return;}
    for(i=0U;i<g->count;i++) {
        if(g->grid_ids[i]>24U){status.error=3U;return;}
        mask|=1UL<<g->grid_ids[i];
    }
    nav_map.occupied_mask=mask;
    scan_sequence=g->scan_sequence;grid_done=1U;
}
static void on_calibration(const radar_client_calibration_t *c)
{
    if(status.stage==4U && calibrating && c->pose.zone==2U) calibration_done=1U;
}
static void on_path(const radar_client_path_result_t *p)
{
    if(status.stage!=7U || !status.map_valid || path_done)return;
    if(nav_map.valid) {
        /* Client has matched the PLAN_PATH wire sequence and ACK. */
        if(p->start!=request_start || p->goal!=request_goal || p->status!=0U ||
           p->waypoint_count>RADAR_CLIENT_MAX_WAYPOINTS ||
           (p->waypoint_count?p->waypoints[p->waypoint_count-1U]!=p->goal:p->start!=p->goal))return;
        nav_reply.identity=nav_map.identity;
        nav_reply.path=*p;
    }
    path.scan_sequence=scan_sequence;path.request_sequence=p->request_sequence;
    path.start=p->start;path.goal=p->goal;path.status=p->status;
    path.waypoint_count=p->waypoint_count;memcpy(path.waypoints,p->waypoints,p->waypoint_count);
    path_done=1U;
}
static void configure_uart(void)
{
    GPIO_InitTypeDef g;USART_InitTypeDef u;NVIC_InitTypeDef n;
    RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOD,ENABLE);
    RCC_APB1PeriphClockCmd(RCC_APB1Periph_USART3,ENABLE);
    GPIO_PinAFConfig(GPIOD,GPIO_PinSource8,GPIO_AF_USART3);
    GPIO_PinAFConfig(GPIOD,GPIO_PinSource9,GPIO_AF_USART3);
    GPIO_StructInit(&g);g.GPIO_Pin=GPIO_Pin_8|GPIO_Pin_9;g.GPIO_Mode=GPIO_Mode_AF;
    g.GPIO_Speed=GPIO_Speed_50MHz;g.GPIO_PuPd=GPIO_PuPd_UP;GPIO_Init(GPIOD,&g);
    USART_StructInit(&u);u.USART_BaudRate=115200U;u.USART_Mode=USART_Mode_Rx|USART_Mode_Tx;
    USART_Init(USART3,&u);USART_ITConfig(USART3,USART_IT_RXNE,DISABLE);
    USART_ITConfig(USART3,USART_IT_TXE,DISABLE);USART_ITConfig(USART3,USART_IT_TC,DISABLE);
    n.NVIC_IRQChannel=USART3_IRQn;n.NVIC_IRQChannelPreemptionPriority=6U;
    n.NVIC_IRQChannelSubPriority=0U;n.NVIC_IRQChannelCmd=ENABLE;NVIC_Init(&n);USART_Cmd(USART3,ENABLE);
}
#if APP_ZONE1_RADAR_LIFT_TEST
bool RadarObserver_PollZone1Link(void)
{
    static bool started;
    radar_client_state_t client;
    if(!started) {
        configure_uart();
        RadarClient_Init(NULL,RADAR_ZONE1_NODE_ID);
        (void)RadarClient_RequestSelectZone(1U);
        started=true;
    }
    RadarClient_Poll();
    RadarClient_GetState(&client);
    return client.connected && client.ready && client.node_id==RADAR_ZONE1_NODE_ID &&
        client.selected_zone==1U;
}
#endif

bool RadarObserver_Configure(const RadarZone2Config *c)
{
    uint32_t key;bool ok=false;if(!configured(c))return false;
    key=RadarUartPort_EnterCritical();
    if(!configure_pending && !command && !owner_busy && !g_radar_mechanism_request && !cancel_pending && !stop_pending) {
        pending_config=*c;
        if(!pending_config.zone) pending_config.zone=2U;
        configure_pending=1U;ok=true;
    }
    RadarUartPort_ExitCritical(key);return ok;
}
static bool submit(uint8_t kind,uint8_t start,uint8_t goal)
{
    uint32_t key=RadarUartPort_EnterCritical();bool ok=false;
    if(initialized && config.zone==2U && !configure_pending && !command && !owner_busy &&
       !g_radar_mechanism_request && !cancel_pending && !stop_pending) {
        request_start=start;request_goal=goal;owner_busy=1U;command=kind;ok=true;
    }
    RadarUartPort_ExitCritical(key);return ok;
}
bool RadarObserver_StartZone2(uint8_t start,uint8_t goal,bool reference)
{ return reference && start<=24U && goal<=24U && submit(1U,start,goal); }
bool RadarCalibration_StartZone2(void) { return submit(2U,0U,0U); }
void RadarObserver_Cancel(void)
{
    uint32_t key=RadarUartPort_EnterCritical();
    cancel_pending=1U;
    published_map.valid=0U;
    RadarUartPort_ExitCritical(key);
}
static bool scan_ready(void)
{
    return published_ready && !configure_pending && !command && !owner_busy &&
        !g_radar_mechanism_request && !cancel_pending;
}
bool RadarObserver_IsScanReady(void)
{
    uint32_t key=RadarUartPort_EnterCritical();bool ready=scan_ready();
    RadarUartPort_ExitCritical(key);return ready;
}
bool RadarObserver_StartScan(uint32_t run_id,uint8_t start_id,uint32_t layout_id)
{
    uint32_t key;bool ok=false;
    if(!run_id || !layout_id || (start_id!=2U && (start_id!=1U || !APP_ZONE1_RADAR_LIFT_CONFIRMED)))return false;
    key=RadarUartPort_EnterCritical();
    if(scan_ready() && start_id==config.zone && run_id!=scan_run) {
        scan_run=run_id;scan_layout=layout_id;owner_busy=1U;command=CMD_SCAN;
        published_map.valid=0U;ok=true;
    }
    RadarUartPort_ExitCritical(key);return ok;
}
bool RadarObserver_GetMap(uint32_t run_id,RadarObserverMap *out)
{
    uint32_t key;bool ok;
    if(!out)return false;
    key=RadarUartPort_EnterCritical();
    ok=published_map.valid && published_map.identity.run_id==run_id &&
        !configure_pending && (!command || command==CMD_PATH) && !cancel_pending;
    if(ok)*out=published_map;
    RadarUartPort_ExitCritical(key);return ok;
}
bool RadarObserver_RequestPath(const RadarObserverMapIdentity *expected,uint8_t start,uint8_t goal)
{
    uint32_t key;bool ok=false;
    if(!expected || start>24U || goal>24U)return false;
    key=RadarUartPort_EnterCritical();
    if(scan_ready() && published_map.valid && same_map(expected,&published_map.identity)) {
        request_start=start;request_goal=goal;owner_busy=1U;command=CMD_PATH;
        published_reply.status.path_valid=0U;ok=true;
    }
    RadarUartPort_ExitCritical(key);return ok;
}
bool RadarObserver_GetPathReply(const RadarObserverMapIdentity *expected,uint8_t start,uint8_t goal,RadarObserverPathReply *out)
{
    uint32_t key;bool ok;
    if(!expected || !out)return false;
    key=RadarUartPort_EnterCritical();
    ok=published_map.valid && published_reply.status.path_valid && !configure_pending &&
        !command && !cancel_pending && same_map(expected,&published_reply.identity) &&
        published_reply.path.start==start && published_reply.path.goal==goal;
    if(ok)*out=published_reply;
    RadarUartPort_ExitCritical(key);return ok;
}
bool RadarObserver_GetPath(RadarObserverPath *p)
{
    bool valid;uint32_t key;if(!p)return false;key=RadarUartPort_EnterCritical();
    valid=published_status.path_valid!=0U && !command && !cancel_pending;
    if(valid)*p=published_path;RadarUartPort_ExitCritical(key);return valid;
}
void RadarObserver_GetStatus(RadarObserverStatus *s)
{ uint32_t key;if(!s)return;key=RadarUartPort_EnterCritical();*s=published_status;RadarUartPort_ExitCritical(key); }
static void stage(uint32_t value) { status.stage=value;entered=now(); }
static bool begin_up(uint32_t t)
{
    g_radar_lift_interval_us=RADAR_LIFT_INTERVAL_US;
    if(!RadarLift_Enable(t) || !RadarLift_BeginBounded(leg_steps,leg_direction^1U,leg_budget,&lift_sequence))
        return false;
    stage(5U);return true;
}
static void fail(uint32_t error)
{
    if(!status.error)status.error=error;
    invalidate();stage(99U);
    if(lift_claimed)RadarLift_Cancel();
    if(mechanism)g_radar_mechanism_result=3U;
    if(initialized && !mechanism
#if APP_ZONE2_RADAR_CALIBRATION_TEST
       && !recovering
#endif
      )(void)RadarClient_RequestStop();
#if APP_ZONE2_RADAR_CALIBRATION_TEST
    recovering=0U;status.recovered=0U;
#endif
}
#if APP_ZONE2_RADAR_CALIBRATION_TEST
static bool recover_radar_fault(uint32_t t,uint32_t error)
{
    RadarLiftSnapshot lift;
    if(mechanism || (!calibrating && !scan_only) || !lift_claimed ||
       status.stage<3U || status.stage>6U)return false;
    if(!status.error)status.error=error;
    if(status.stage<=4U) {
        RadarLift_GetSnapshot(&lift);
        if(lift.sequence!=lift_sequence || lift.state!=2U || lift.result!=1U || lift.done!=leg_steps ||
           !begin_up(t))return false;
    }
    recovering=1U;
    invalidate();
    (void)RadarClient_RequestStop();
    return true;
}
#endif
/* 单一雷达任务服务入口，测试也执行此实际代码；不调用底盘接口。 */
static void service_owner(void)
{
    uint8_t cmd,zone;bool maintenance,cancel;uint32_t key,t=now();radar_client_state_t client;RadarLiftSnapshot lift;
    key=RadarUartPort_EnterCritical();
    if(configure_pending) {
        config=pending_config;configure_pending=0U;initialized=0U;link_ready=0U;
        published_ready=0U;published_map.valid=0U;invalidate();
#if APP_ZONE2_RADAR_CALIBRATION_TEST
        recovering=0U;status.recovered=0U;
#endif
    }
    cmd=command;command=0U;
    if(g_radar_mechanism_request) {
        uint32_t request=g_radar_mechanism_request;g_radar_mechanism_request=0U;
        if(!owner_busy && !cmd && !configure_pending && !stop_pending && request<=2U &&
           g_radar_mechanism_wait_ms>0U && g_radar_mechanism_wait_ms<0x80000000U &&
           g_radar_mechanism_interval_us>=733U && g_radar_mechanism_interval_us<=4000U &&
           g_radar_mechanism_budget_ms>=(uint32_t)(((uint64_t)RADAR_ZONE_LIFT_STEPS*g_radar_mechanism_interval_us+999U)/1000U) &&
           g_radar_mechanism_budget_ms>g_radar_mechanism_wait_ms && g_radar_mechanism_budget_ms<0x80000000U) {
            mechanism_wait=g_radar_mechanism_wait_ms;mechanism_budget=g_radar_mechanism_budget_ms;
            mechanism_interval=g_radar_mechanism_interval_us;cmd=(uint8_t)(request+2U);owner_busy=1U;
            g_radar_mechanism_result=1U;
        } else g_radar_mechanism_result=2U;
    }
    RadarUartPort_ExitCritical(key);
    maintenance=cmd==3U || cmd==4U;
    zone=(maintenance || mechanism)?2U:config.zone;
    if(!maintenance && !mechanism && configured(&config) && !initialized) {
        radar_client_callbacks_t cb;memset(&cb,0,sizeof(cb));
        cb.state=on_state;cb.error=on_error;cb.grid=on_grid;cb.path=on_path;cb.calibration=on_calibration;
        configure_uart();RadarClient_Init(&cb,config.node_id);
        (void)RadarClient_RequestSelectZone(config.zone);initialized=1U;
    }
    if(initialized && !mechanism && !maintenance)RadarClient_Poll();
    key=RadarUartPort_EnterCritical();
    cancel=cancel_pending;
    if(cancel) { owner_busy=1U;cancel_pending=0U;command=0U; }
    RadarUartPort_ExitCritical(key);
    if(cancel) {
#if APP_ZONE2_RADAR_CALIBRATION_TEST
        status.recovered=0U;
#endif
        if(cmd){mechanism=maintenance;status.sequence++;status.error=0U;fail(4U);}
        else if((status.stage>0U && status.stage<8U) || (scan_only && status.stage==8U))fail(4U);
        return;
    }
    if(cmd==CMD_PATH) {
        status.sequence++;status.error=0U;status.path_valid=0U;path_done=0U;
        if(!nav_map.valid || !link_ready ||
           RadarClient_RequestPlanPath(request_start,request_goal)!=RADAR_CLIENT_REQUEST_ACCEPTED){fail(3U);return;}
        stage(7U);
    }
    if(cmd && cmd!=CMD_PATH) {
        if(cmd==CMD_SCAN) {
            g_radar_scan_submit_result=-1;
            g_radar_scan_first_error=(radar_client_error_t){(radar_client_error_kind_t)0, RADAR_CLIENT_OPERATION_NONE, 0U, 0U};
        }
        invalidate();status.sequence++;status.error=0U;status.calibration_complete=0U;
#if APP_ZONE2_RADAR_CALIBRATION_TEST
        recovering=0U;status.recovered=0U;
        if(cmd==2U) {
            g_radar_calibration_submit_result=-1;
            g_radar_calibration_first_error=(radar_client_error_t){(radar_client_error_kind_t)0, RADAR_CLIENT_OPERATION_NONE, 0U, 0U};
        }
#endif
        mechanism=maintenance;
        if((!mechanism && (!configured(&config) || !RadarClient_IsReady() || RadarClient_IsBusy())) ||
           (cmd==CMD_SCAN && !link_ready) ||
           !RadarLift_SafeIdle(t) || !RadarLift_Claim(zone)) {
            status.error=1U;if(mechanism)g_radar_mechanism_result=3U;stage(99U);return;
        }
        lift_claimed=1U;scan_only=cmd==CMD_SCAN;
        calibrating=cmd==2U;grid_done=calibration_done=path_done=0U;
        leg_steps=mechanism?RADAR_ZONE_LIFT_STEPS:config.steps;
        leg_direction=mechanism?1U:config.down_direction;
        leg_budget=mechanism?mechanism_budget:config.stage_timeout_ms-250U;
        leg_wait=mechanism?mechanism_wait:config.deploy_ms;
        /* 扫描配置按RADAR_LIFT_INTERVAL_US名义基线准入；机构邮箱按本次档位。 */
        g_radar_lift_interval_us=mechanism?mechanism_interval:RADAR_LIFT_INTERVAL_US;
        if(cmd==4U) {
            if(!RadarLift_Enable(t) || !RadarLift_BeginBounded(leg_steps,0U,leg_budget,&lift_sequence)){fail(2U);return;}
            stage(5U);
        } else {
            if(RadarServo_SetAngle(zone,zone==1U?RADAR_SERVO1_DEPLOY_DEG:RADAR_SERVO2_DEPLOY_DEG)!=1U){fail(2U);return;}stage(1U);
        }
    }
    if(status.stage==0U || status.stage>=8U)return;
    t=now();
    if(!mechanism)RadarClient_GetState(&client);
#if APP_ZONE2_RADAR_CALIBRATION_TEST
    if(!mechanism && (calibrating || scan_only)) {
        bool timed_out=(uint32_t)(t-entered)>config.stage_timeout_ms;
        if(!RadarLift_SafeIdle(t) || (timed_out && (status.stage<3U || status.stage>4U))) {fail(5U);return;}
        if(!recovering && (status.error || !client.ready || client.selected_zone!=2U || timed_out)) {
            uint32_t error=status.error?status.error:5U;
            if(!recover_radar_fault(t,error)){fail(error);return;}
            t=now();
        }
    } else
#endif
    {
    if(status.error){fail(status.error);return;}
    if((!mechanism && (!client.ready || client.selected_zone!=zone)) || (lift_claimed && !RadarLift_SafeIdle(t)) ||
       (uint32_t)(t-entered)>(mechanism?mechanism_budget:config.stage_timeout_ms)) { fail(5U);return; }
    }
    switch(status.stage) {
    case 1U:
        if((uint32_t)(t-entered)>=leg_wait) {
            g_radar_lift_interval_us=mechanism?mechanism_interval:RADAR_LIFT_INTERVAL_US;
            if(!RadarLift_Enable(t) || !RadarLift_BeginBounded(leg_steps,leg_direction,leg_budget,&lift_sequence)){fail(2U);break;}stage(2U);
        }break;
    case 2U:case 5U:
        RadarLift_GetSnapshot(&lift);
        if(lift.sequence!=lift_sequence){if(lift.sequence+1U!=lift_sequence)fail(2U);break;}
        if(lift.state==3U || lift.result!=1U){fail(2U);break;}
        if(lift.state==2U && lift.done==leg_steps) {
            if(status.stage==2U) {
                if(!RadarLift_Release(lift_sequence)){fail(2U);break;}
                if(mechanism){g_radar_mechanism_result=4U;stage(8U);}else stage(3U);
            }
            else {
                if(!mechanism && !calibrating && !RadarLift_Release(lift_sequence)){fail(2U);break;}
                if(RadarServo_SetAngle(zone,zone==1U?RADAR_SERVO1_INIT_DEG:RADAR_SERVO2_INIT_DEG)!=1U){fail(2U);break;}stage(6U);
            }
        }break;
    case 3U:
        if((uint32_t)(t-entered)>=config.settle_ms) {
            radar_client_request_result_t result=calibrating?RadarClient_RequestCalibrate(2U):RadarClient_RequestGridScan();
            if(scan_only)g_radar_scan_submit_result=result;
#if APP_ZONE2_RADAR_CALIBRATION_TEST
            if(calibrating)g_radar_calibration_submit_result=result;
#endif
            if(result!=RADAR_CLIENT_REQUEST_ACCEPTED){
#if APP_ZONE2_RADAR_CALIBRATION_TEST
                if(recover_radar_fault(t,3U))break;
#endif
                fail(3U);break;
            }stage(4U);
        }break;
    case 4U:
        if((calibrating?calibration_done:grid_done) && !RadarClient_IsBusy()) {
            if(calibrating)status.calibration_complete=1U;
            else if(!scan_only)status.map_valid=1U;
            if(!begin_up(t)){fail(2U);break;}
        }break;
    case 6U:
        if((uint32_t)(t-entered)>= (mechanism?mechanism_wait:config.stow_ms)) {
            if((mechanism || calibrating) && !RadarLift_Release(lift_sequence)){fail(2U);break;}
#if APP_ZONE2_RADAR_CALIBRATION_TEST
            if(recovering) {
                RadarLift_GetSnapshot(&lift);
                if(lift.sequence!=lift_sequence || lift.state!=2U || lift.result!=1U || lift.done!=leg_steps){
                    fail(2U);break;
                }
                recovering=0U;status.recovered=1U;stage(99U);break;
            }
#endif
            if(mechanism){g_radar_mechanism_result=4U;stage(8U);break;}
            if(calibrating){stage(8U);break;}
            status.map_valid=1U;
            if(scan_only) {
                nav_map.identity.run_id=scan_run;
                nav_map.identity.map_generation=++map_generation;
                nav_map.identity.layout_id=scan_layout;
                nav_map.identity.link_generation=link_generation;
                nav_map.identity.scan_sequence=scan_sequence;
                nav_map.identity.start_id=zone;
                nav_map.identity.node_id=config.node_id;
                nav_map.identity.zone=zone;
#if APP_ZONE2_NAV_MOTION_ENABLE && !APP_ZONE2_RADAR_CALIBRATION_TEST
                idle_bad_event_ignored=0U;
#endif
                nav_map.valid=1U;stage(8U);break;
            }
            if(!status.map_valid || RadarClient_RequestPlanPath(request_start,request_goal)!=RADAR_CLIENT_REQUEST_ACCEPTED){fail(3U);break;}stage(7U);
        }break;
    case 7U:if(path_done && !RadarClient_IsBusy()){status.path_valid=1U;stage(8U);}break;
    default:break;
    }
}
void RadarObserver_Service(void)
{
    uint32_t key;service_owner();key=RadarUartPort_EnterCritical();
    if(!command && (status.stage==0U || status.stage>=8U)) {
        if(lift_claimed)RadarLift_Unclaim();
        lift_claimed=0U;owner_busy=0U;mechanism=0U;
    }
    if(cancel_pending && scan_only)invalidate();
    published_ready=initialized && link_ready && !RadarClient_IsBusy();
    published_status=status;if(status.path_valid)published_path=path;
    nav_map.status=nav_reply.status=status;
    published_map=nav_map;published_reply=nav_reply;
    RadarUartPort_ExitCritical(key);
}
