#include "radar_observer.h"
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
static volatile uint8_t owner_busy;
static RadarObserverPath path;
static volatile uint8_t command,configure_pending,cancel_pending;
/* 独立机构单次邮箱：先填等待/预算，最后写request=1下或2上；0默认不动作。 */
volatile uint32_t g_radar_mechanism_request=0U,g_radar_mechanism_wait_ms=0U;
volatile uint32_t g_radar_mechanism_budget_ms=0U,g_radar_mechanism_interval_us=RADAR_LIFT_INTERVAL_US;
volatile uint32_t g_radar_mechanism_result=0U;
static uint8_t request_start,request_goal,calibrating,initialized,mechanism;
static uint32_t mechanism_wait,mechanism_budget,mechanism_interval;
static uint32_t leg_steps,leg_direction,leg_budget,leg_wait;
static uint32_t entered,lift_sequence,scan_sequence;
static uint8_t grid_done,calibration_done,path_done;
static uint32_t now(void) { return (uint32_t)(xTaskGetTickCount()*portTICK_PERIOD_MS); }
static bool configured(const RadarZone2Config *c)
{
    return c && c->measured==1U && c->node_id>=1U && c->node_id<=254U &&
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
static void invalidate(void) { status.map_valid=0U;status.path_valid=0U;status.calibration_complete=0U; }
static void on_state(const radar_client_state_t *s)
{
    if (!s->ready || s->selected_zone!=2U) invalidate();
}
static void on_error(const radar_client_error_t *e)
{
    (void)e;invalidate();
    if(!mechanism && status.stage>0U && status.stage<8U) status.error=3U;
}
static void on_grid(const radar_client_grid_result_t *g)
{
    if(status.stage==4U && !calibrating) { scan_sequence=g->scan_sequence;grid_done=1U; }
}
static void on_calibration(const radar_client_calibration_t *c)
{
    if(status.stage==4U && calibrating && c->pose.zone==2U) calibration_done=1U;
}
static void on_path(const radar_client_path_result_t *p)
{
    if(status.stage!=7U || !status.map_valid) return;
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
bool RadarObserver_Configure(const RadarZone2Config *c)
{
    uint32_t key;bool ok=false;if(!configured(c))return false;
    key=RadarUartPort_EnterCritical();
    if(!configure_pending && !command && !owner_busy && !g_radar_mechanism_request) {
        pending_config=*c;configure_pending=1U;ok=true;
    }
    RadarUartPort_ExitCritical(key);return ok;
}
static bool submit(uint8_t kind,uint8_t start,uint8_t goal)
{
    uint32_t key=RadarUartPort_EnterCritical();bool ok=false;
    if(initialized && !configure_pending && !command && !owner_busy && !g_radar_mechanism_request) {
        request_start=start;request_goal=goal;owner_busy=1U;command=kind;ok=true;
    }
    RadarUartPort_ExitCritical(key);return ok;
}
bool RadarObserver_StartZone2(uint8_t start,uint8_t goal,bool reference)
{ return reference && start<=24U && goal<=24U && submit(1U,start,goal); }
bool RadarCalibration_StartZone2(void) { return submit(2U,0U,0U); }
void RadarObserver_Cancel(void) { cancel_pending=1U; }
bool RadarObserver_GetPath(RadarObserverPath *p)
{
    bool valid;uint32_t key;if(!p)return false;key=RadarUartPort_EnterCritical();
    valid=published_status.path_valid!=0U;if(valid)*p=published_path;RadarUartPort_ExitCritical(key);return valid;
}
void RadarObserver_GetStatus(RadarObserverStatus *s)
{ uint32_t key;if(!s)return;key=RadarUartPort_EnterCritical();*s=published_status;RadarUartPort_ExitCritical(key); }
static void stage(uint32_t value) { status.stage=value;entered=now(); }
static void fail(uint32_t error)
{
    status.error=error;invalidate();stage(99U);RadarLift_Cancel();
    if(mechanism)g_radar_mechanism_result=3U;
    if(initialized && !mechanism)(void)RadarClient_RequestStop();
}
/* 单一雷达任务服务入口，测试也执行此实际代码；不调用底盘接口。 */
static void service_owner(void)
{
    uint8_t cmd;uint32_t key,t=now();radar_client_state_t client;RadarLiftSnapshot lift;
    key=RadarUartPort_EnterCritical();
    if(configure_pending) {
        config=pending_config;configure_pending=0U;initialized=0U;invalidate();
    }
    cmd=command;command=0U;
    if(g_radar_mechanism_request) {
        uint32_t request=g_radar_mechanism_request;g_radar_mechanism_request=0U;
        if(!owner_busy && !cmd && !configure_pending && request<=2U &&
           g_radar_mechanism_wait_ms>0U && g_radar_mechanism_wait_ms<0x80000000U &&
           g_radar_mechanism_interval_us>=733U && g_radar_mechanism_interval_us<=4000U &&
           g_radar_mechanism_budget_ms>=(uint32_t)(((uint64_t)8559U*g_radar_mechanism_interval_us+999U)/1000U) &&
           g_radar_mechanism_budget_ms>g_radar_mechanism_wait_ms && g_radar_mechanism_budget_ms<0x80000000U) {
            mechanism_wait=g_radar_mechanism_wait_ms;mechanism_budget=g_radar_mechanism_budget_ms;
            mechanism_interval=g_radar_mechanism_interval_us;cmd=(uint8_t)(request+2U);owner_busy=1U;
            g_radar_mechanism_result=1U;
        } else g_radar_mechanism_result=2U;
    }
    RadarUartPort_ExitCritical(key);
    if(cmd<3U && !mechanism && configured(&config) && !initialized) {
        radar_client_callbacks_t cb;memset(&cb,0,sizeof(cb));
        cb.state=on_state;cb.error=on_error;cb.grid=on_grid;cb.path=on_path;cb.calibration=on_calibration;
        configure_uart();RadarClient_Init(&cb,config.node_id);
        (void)RadarClient_RequestSelectZone(2U);initialized=1U;
    }
    if(initialized && !mechanism && cmd<3U)RadarClient_Poll();
    if(cancel_pending) {
        cancel_pending=0U;command=0U;
        if(cmd){mechanism=cmd>=3U;status.sequence++;invalidate();fail(4U);}
        else if(status.stage>0U && status.stage<8U)fail(4U);
        return;
    }
    if(cmd) {
        invalidate();status.sequence++;status.error=0U;status.calibration_complete=0U;
        mechanism=cmd>=3U;
        if((!mechanism && (!configured(&config) || !RadarClient_IsReady() || RadarClient_IsBusy())) ||
           !RadarLift_SafeIdle(t) || !RadarLift_Claim()) {
            status.error=1U;if(mechanism)g_radar_mechanism_result=3U;stage(99U);return;
        }
        calibrating=cmd==2U;grid_done=calibration_done=path_done=0U;
        leg_steps=mechanism?8559U:config.steps;
        leg_direction=mechanism?1U:config.down_direction;
        leg_budget=mechanism?mechanism_budget:config.stage_timeout_ms-250U;
        leg_wait=mechanism?mechanism_wait:config.deploy_ms;
        /* 扫描配置按733us名义基线准入；机构邮箱按本次档位。 */
        g_radar_lift_interval_us=mechanism?mechanism_interval:RADAR_LIFT_INTERVAL_US;
        if(cmd==4U) {
            if(!RadarLift_Enable(t) || !RadarLift_BeginBounded(leg_steps,0U,leg_budget,&lift_sequence)){fail(2U);return;}
            stage(5U);
        } else {
            if(RadarServo_SetAngle(2U,RADAR_SERVO2_DEPLOY_DEG)!=1U){fail(2U);return;}stage(1U);
        }
    }
    if(status.stage==0U || status.stage>=8U)return;
    if(!mechanism)RadarClient_GetState(&client);
    if(status.error){fail(status.error);return;}
    if((!mechanism && (!client.ready || client.selected_zone!=2U)) || !RadarLift_SafeIdle(t) ||
       (uint32_t)(t-entered)>(mechanism?mechanism_budget:config.stage_timeout_ms)) { fail(5U);return; }
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
            else {if(RadarServo_SetAngle(2U,180.0f)!=1U){fail(2U);break;}stage(6U);}
        }break;
    case 3U:
        if((uint32_t)(t-entered)>=config.settle_ms) {
            if((calibrating?RadarClient_RequestCalibrate(2U):RadarClient_RequestGridScan())!=RADAR_CLIENT_REQUEST_ACCEPTED){fail(3U);break;}stage(4U);
        }break;
    case 4U:
        if((calibrating?calibration_done:grid_done) && !RadarClient_IsBusy()) {
            if(!calibrating)status.map_valid=1U;else status.calibration_complete=1U;
            g_radar_lift_interval_us=RADAR_LIFT_INTERVAL_US;
            if(!RadarLift_Enable(t) || !RadarLift_BeginBounded(leg_steps,leg_direction^1U,leg_budget,&lift_sequence)){fail(2U);break;}stage(5U);
        }break;
    case 6U:
        if((uint32_t)(t-entered)>= (mechanism?mechanism_wait:config.stow_ms)) {
            if(!RadarLift_Release(lift_sequence)){fail(2U);break;}
            if(mechanism){g_radar_mechanism_result=4U;stage(8U);break;}
            if(calibrating){stage(8U);break;}
            if(!status.map_valid || RadarClient_RequestPlanPath(request_start,request_goal)!=RADAR_CLIENT_REQUEST_ACCEPTED){fail(3U);break;}stage(7U);
        }break;
    case 7U:if(path_done && !RadarClient_IsBusy()){status.path_valid=1U;stage(8U);}break;
    default:break;
    }
}
void RadarObserver_Service(void)
{
    uint32_t key;service_owner();key=RadarUartPort_EnterCritical();
    published_status=status;if(status.path_valid)published_path=path;
    if(!command && (status.stage==0U || status.stage>=8U)){if(owner_busy)RadarLift_Unclaim();owner_busy=0U;mechanism=0U;}
    RadarUartPort_ExitCritical(key);
}
