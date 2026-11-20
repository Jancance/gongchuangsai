#include "radar_lift.h"
#include "stm32f4xx.h"
#include "zdt_emm_config.h"
#include "app_entry_config.h"
#include "app_config.h"
volatile uint32_t g_radar_lift_interval_us=RADAR_LIFT_INTERVAL_US;
volatile uint32_t g_radar_lift_enable=0U,g_radar_lift_request=0U;
volatile uint32_t g_radar_lift_steps=32U,g_radar_lift_direction=0U;
volatile uint32_t g_radar_lift_state=0U,g_radar_lift_result=0U,g_radar_lift_sequence=0U;
volatile uint32_t g_radar_lift_done=0U,g_radar_lift_phase_bits=0U;
static uint32_t pending_long,pending_steps,pending_direction,pending_budget,pending_interval,claimed;
static volatile uint32_t last_service,safe_service;
static uint32_t nominal_ms(uint32_t steps,uint32_t interval)
{ return (uint32_t)(((uint64_t)steps*interval+999U)/1000U); }
static uint32_t valid_interval(uint32_t interval) { return interval>=733U && interval<=4000U; }
#if APP_IS_DEBUG && !APP_MOTOR_OFFLINE_ONLY
static uint32_t initialized=0U,active_zone=2U,start_tick,active_budget,active_interval;
static volatile uint32_t phase=7U,total,direction;
static volatile uint32_t timer_active,terminal_reason,safe_cycle;
static const uint8_t phases[8]={1U,3U,2U,6U,4U,12U,8U,9U};
static void setup(void)
{
    GPIO_InitTypeDef gpio;
    if (initialized==active_zone) return;
    GPIO_StructInit(&gpio);gpio.GPIO_Mode=GPIO_Mode_OUT;
    gpio.GPIO_OType=GPIO_OType_PP;gpio.GPIO_PuPd=GPIO_PuPd_DOWN;
    gpio.GPIO_Speed=GPIO_Speed_2MHz;
    if (active_zone==1U) {
        RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOE|RCC_AHB1Periph_GPIOD,ENABLE);
        GPIO_ResetBits(GPIOE,GPIO_Pin_11|GPIO_Pin_9);
        GPIO_ResetBits(GPIOD,GPIO_Pin_4|GPIO_Pin_3);
        gpio.GPIO_Pin=GPIO_Pin_11|GPIO_Pin_9;GPIO_Init(GPIOE,&gpio);
        gpio.GPIO_Pin=GPIO_Pin_4|GPIO_Pin_3;GPIO_Init(GPIOD,&gpio);
    } else {
        RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOB|RCC_AHB1Periph_GPIOD,ENABLE);
        GPIO_ResetBits(GPIOD,GPIO_Pin_10|GPIO_Pin_11|GPIO_Pin_12);
        GPIO_ResetBits(GPIOB,GPIO_Pin_8);
        gpio.GPIO_Pin=GPIO_Pin_10|GPIO_Pin_11|GPIO_Pin_12;GPIO_Init(GPIOD,&gpio);
        gpio.GPIO_Pin=GPIO_Pin_8;GPIO_Init(GPIOB,&gpio);
    }
    phase=7U;initialized=active_zone;
}
static void apply(uint32_t bits)
{
    /* Each bank uses one BSRR write; no read-modify-write of unrelated pins. */
    if (active_zone==1U) {
        uint32_t pe=((bits&1U)<<11)|((bits&2U)<<8);
        uint32_t pd=((bits&4U)<<2)|(bits&8U);
        (*(volatile uint32_t *)&GPIOE->BSRRL)=pe|(((GPIO_Pin_11|GPIO_Pin_9)&~pe)<<16);
        (*(volatile uint32_t *)&GPIOD->BSRRL)=pd|(((GPIO_Pin_4|GPIO_Pin_3)&~pd)<<16);
    } else {
        uint32_t pd=(bits&7U)<<10;
        (*(volatile uint32_t *)&GPIOD->BSRRL)=pd|(((GPIO_Pin_10|GPIO_Pin_11|GPIO_Pin_12)&~pd)<<16);
        (*(volatile uint32_t *)&GPIOB->BSRRL)=(bits&8U)?GPIO_Pin_8:((uint32_t)GPIO_Pin_8<<16);
    }
    g_radar_lift_phase_bits=bits;
}

static void timer_stop(uint32_t reason)
{
    TIM7->DIER=0U;TIM7->CR1=0U;TIM7->SR=0U;NVIC_ClearPendingIRQ(TIM7_IRQn);
    timer_active=0U;if(reason && !terminal_reason)terminal_reason=reason;
}
/* Shared HWT counter: enable only, never reset CYCCNT. */
static uint32_t cycle_ready(void)
{
    uint32_t before;
    CoreDebug->DEMCR|=CoreDebug_DEMCR_TRCENA_Msk;
    if(DWT->CTRL&DWT_CTRL_NOCYCCNT_Msk)return 0U;
    DWT->CTRL|=DWT_CTRL_CYCCNTENA_Msk;before=DWT->CYCCNT;
    __NOP();__NOP();__NOP();__NOP();
    return DWT->CYCCNT!=before && SystemCoreClock>=10U;
}
static uint32_t timer_start(void)
{
    RCC_ClocksTypeDef clocks;NVIC_InitTypeDef n;uint32_t hz,divider;
    RCC_GetClocksFreq(&clocks);hz=clocks.PCLK1_Frequency;
    if((RCC->CFGR&RCC_CFGR_PPRE1)!=0U) {if(hz>0x7fffffffU)return 0U;hz*=2U;}
    if(hz<1000000U || hz%1000000U)return 0U;
    divider=hz/1000000U;if(divider>65536U)return 0U;
    RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM7,ENABLE);
    timer_stop(0U);TIM7->PSC=divider-1U;TIM7->ARR=active_interval-1U;
    TIM7->CNT=0U;TIM7->EGR=TIM_EGR_UG;TIM7->SR=0U;NVIC_ClearPendingIRQ(TIM7_IRQn);
    DBGMCU->APB1FZ|=DBGMCU_APB1_FZ_DBG_TIM7_STOP;
    n.NVIC_IRQChannel=TIM7_IRQn;n.NVIC_IRQChannelPreemptionPriority=4U;
    n.NVIC_IRQChannelSubPriority=0U;n.NVIC_IRQChannelCmd=ENABLE;NVIC_Init(&n);
    terminal_reason=0U;timer_active=1U;TIM7->DIER=TIM_DIER_UIE;TIM7->CR1=TIM_CR1_CEN;
    return 1U;
}
void TIM7_IRQHandler(void)
{
    if(!(TIM7->SR&TIM_SR_UIF))return;
    TIM7->SR=0U;
    if(!timer_active)return;
    if(g_radar_lift_request==2U){timer_stop(5U);return;}
    if(g_radar_lift_enable!=1U || safe_service!=1U ||
       !(CoreDebug->DEMCR&CoreDebug_DEMCR_TRCENA_Msk) ||
       !(DWT->CTRL&DWT_CTRL_CYCCNTENA_Msk) || (DWT->CTRL&DWT_CTRL_NOCYCCNT_Msk) ||
       (uint32_t)(DWT->CYCCNT-safe_cycle)>SystemCoreClock/10U){timer_stop(2U);return;}
    phase=(phase+(direction?7U:1U))&7U;apply(phases[phase]);g_radar_lift_done++;
    if(g_radar_lift_done>=total)timer_stop(1U);
}
#endif
static uint32_t safe_owner(uint32_t now_ms)
{ return safe_service==1U && (uint32_t)(now_ms-last_service)<=100U; }
uint32_t RadarLift_SafeIdle(uint32_t now_ms)
{
    uint32_t key=__get_PRIMASK(),ok;__disable_irq();ok=safe_owner(now_ms);
    if(!key)__enable_irq();return ok;
}
static uint32_t available_owner(void)
{
#if APP_IS_DEBUG && !APP_MOTOR_OFFLINE_ONLY
    return g_radar_lift_enable==1U && !g_radar_lift_request && g_radar_lift_state!=1U && !pending_long;
#else
    return 0U;
#endif
}
uint32_t RadarLift_Available(void)
{
    uint32_t key=__get_PRIMASK(),ok;__disable_irq();ok=available_owner();
    if(!key)__enable_irq();return ok;
}
uint32_t RadarLift_Begin(uint32_t steps,uint32_t dir,uint32_t *sequence)
{
    uint32_t key=__get_PRIMASK(),ok=0U;__disable_irq();
    if(sequence && !claimed && available_owner() && valid_interval(g_radar_lift_interval_us) &&
       steps>=1U && steps<=RADAR_LIFT_MAX_STEPS && dir<=1U) {
        pending_steps=g_radar_lift_steps=steps;pending_direction=g_radar_lift_direction=dir;
        pending_interval=g_radar_lift_interval_us;pending_budget=nominal_ms(steps,pending_interval)+250U;
        *sequence=g_radar_lift_sequence+1U;g_radar_lift_request=1U;ok=1U;
    }
    if(!key)__enable_irq();return ok;
}
uint32_t RadarLift_BeginBounded(uint32_t steps,uint32_t dir,uint32_t budget_ms,uint32_t *sequence)
{
    uint32_t key=__get_PRIMASK(),ok=0U;__disable_irq();
    if(!APP_ZONE1_RADAR_LIFT_TEST && sequence && available_owner() && steps>=1U &&
       steps<=RADAR_LIFT_LONG_MAX_STEPS && dir<=1U &&
       valid_interval(g_radar_lift_interval_us) && budget_ms>=nominal_ms(steps,g_radar_lift_interval_us) && budget_ms<0x80000000U) {
        pending_steps=g_radar_lift_steps=steps;pending_direction=g_radar_lift_direction=dir;
        pending_budget=budget_ms;pending_long=1U;pending_interval=g_radar_lift_interval_us;
        *sequence=g_radar_lift_sequence+1U;g_radar_lift_request=1U;ok=1U;
    }
    if(!key)__enable_irq();return ok;
}
uint32_t RadarLift_Claim(uint32_t zone)
{
    uint32_t key=__get_PRIMASK(),ok=0U;__disable_irq();
#if APP_IS_DEBUG && !APP_MOTOR_OFFLINE_ONLY
    if(!APP_ZONE1_RADAR_LIFT_TEST && (zone==2U || (zone==1U && APP_ZONE1_RADAR_LIFT_CONFIRMED)) &&
       !claimed && !pending_long && !g_radar_lift_request && g_radar_lift_state!=1U &&
       (!g_radar_lift_phase_bits || zone==active_zone)){claimed=zone;ok=1U;}
#else
    (void)zone;
#endif
    if(!key)__enable_irq();return ok;
}
void RadarLift_Unclaim(void) { uint32_t key=__get_PRIMASK();__disable_irq();claimed=0U;if(!key)__enable_irq(); }
uint32_t RadarLift_Enable(uint32_t now_ms)
{
    uint32_t key=__get_PRIMASK(),ok=0U;__disable_irq();
#if APP_IS_DEBUG && !APP_MOTOR_OFFLINE_ONLY
    if(claimed && safe_owner(now_ms) && !pending_long && !g_radar_lift_request &&
       g_radar_lift_state!=1U){g_radar_lift_enable=1U;ok=1U;}
#else
    (void)now_ms;
#endif
    if(!key)__enable_irq();return ok;
}
uint32_t RadarLift_Release(uint32_t sequence)
{
    uint32_t key=__get_PRIMASK(),ok=0U;__disable_irq();
#if APP_IS_DEBUG && !APP_MOTOR_OFFLINE_ONLY
    if(sequence==g_radar_lift_sequence && g_radar_lift_state==2U && g_radar_lift_result==1U &&
       !g_radar_lift_request && !pending_long && !timer_active){apply(0U);g_radar_lift_enable=0U;ok=1U;}
#else
    (void)sequence;
#endif
    if(!key)__enable_irq();return ok;
}
void RadarLift_Cancel(void)
{
    uint32_t key=__get_PRIMASK();__disable_irq();pending_long=0U;pending_interval=0U;g_radar_lift_request=2U;
#if APP_IS_DEBUG && !APP_MOTOR_OFFLINE_ONLY
    if(timer_active)timer_stop(5U);
#endif
    if(!key)__enable_irq();
}
static void service_owner(uint32_t now_ms,uint32_t safe_idle)
{
    uint32_t request=g_radar_lift_request,long_request=pending_long;
    uint32_t queued=pending_interval!=0U;
    uint32_t request_interval=queued?pending_interval:g_radar_lift_interval_us;
    uint32_t request_steps=queued?pending_steps:g_radar_lift_steps;
    uint32_t request_direction=queued?pending_direction:g_radar_lift_direction;
    pending_interval=0U;pending_long=0U;last_service=now_ms;safe_service=safe_idle;
    g_radar_lift_request=0U;
#if APP_ZONE1_RADAR_LIFT_TEST
    if (request==1U && (!queued || long_request || request_steps!=32U || request_interval!=4000U)) request=0U;
#elif APP_ZONE2_NAV_ENABLE
    if (request==1U && (!long_request || !claimed)) request=0U;
#endif
#if APP_IS_DEBUG && !APP_MOTOR_OFFLINE_ONLY
    {
        uint32_t was_moving=g_radar_lift_state==1U;
        if(timer_active && (!(CoreDebug->DEMCR&CoreDebug_DEMCR_TRCENA_Msk) ||
           !(DWT->CTRL&DWT_CTRL_CYCCNTENA_Msk) || (DWT->CTRL&DWT_CTRL_NOCYCCNT_Msk) ||
           (uint32_t)(DWT->CYCCNT-safe_cycle)>SystemCoreClock/10U))timer_stop(2U);
        if(!cycle_ready())safe_service=0U;
        safe_cycle=DWT->CYCCNT;
        if(was_moving && (safe_service!=1U || g_radar_lift_enable!=1U))timer_stop(2U);
        if(was_moving && timer_active && (uint32_t)(now_ms-start_tick)>active_budget)timer_stop(2U);
        if(was_moving && terminal_reason) {
            g_radar_lift_state=terminal_reason==1U?2U:3U;g_radar_lift_result=terminal_reason;
        }
        if(!request)return;
        if(request==1U && (was_moving || (claimed && !long_request)))return;
        g_radar_lift_sequence++;
        if(request==2U) {
            timer_stop(5U);if(g_radar_lift_state==1U)g_radar_lift_state=3U;
            if(!(terminal_reason==2U && g_radar_lift_state==3U && g_radar_lift_result==2U))g_radar_lift_result=5U;return;
        }
        if(request!=1U || !valid_interval(request_interval) || request_steps<1U ||
           request_steps>(long_request?RADAR_LIFT_LONG_MAX_STEPS:RADAR_LIFT_MAX_STEPS) || request_direction>1U)
            {g_radar_lift_result=3U;return;}
        if(safe_service!=1U || g_radar_lift_enable!=1U){g_radar_lift_result=2U;return;}
        total=request_steps;direction=request_direction;active_interval=request_interval;
        active_budget=queued?pending_budget:nominal_ms(total,active_interval)+250U;
        active_zone=claimed?claimed:(APP_ZONE1_RADAR_LIFT_TEST?1U:2U);
        setup();g_radar_lift_done=0U;start_tick=now_ms;
        g_radar_lift_state=1U;g_radar_lift_result=1U;
        if(!timer_start()){g_radar_lift_state=3U;g_radar_lift_result=2U;}
    }
#else
    (void)now_ms;(void)safe_idle;(void)long_request;(void)request_interval;(void)request_steps;
    (void)request_direction;(void)pending_budget;
    if(request){g_radar_lift_sequence++;g_radar_lift_result=2U;}
#endif
}
void RadarLift_Service(uint32_t now_ms,uint32_t safe_idle)
{
    uint32_t key=__get_PRIMASK();__disable_irq();service_owner(now_ms,safe_idle);if(!key)__enable_irq();
}
void RadarLift_GetSnapshot(RadarLiftSnapshot *s)
{
    uint32_t key=__get_PRIMASK();__disable_irq();
    if(s){s->sequence=g_radar_lift_sequence;s->state=g_radar_lift_state;s->result=g_radar_lift_result;s->done=g_radar_lift_done;}
    if(!key)__enable_irq();
}
