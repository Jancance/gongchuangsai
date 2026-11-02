#include "radar_lift.h"
#include "stm32f4xx.h"
#include "zdt_x_config.h"
#include "app_entry_config.h"
volatile uint32_t g_radar_lift_enable=0U,g_radar_lift_request=0U;
volatile uint32_t g_radar_lift_steps=32U,g_radar_lift_direction=0U;
volatile uint32_t g_radar_lift_state=0U,g_radar_lift_result=0U,g_radar_lift_sequence=0U;
volatile uint32_t g_radar_lift_done=0U,g_radar_lift_phase_bits=0U;
#if APP_IS_DEBUG && !APP_X_OFFLINE_ONLY
static uint32_t initialized=0U,phase=7U,last_tick,start_tick,total,direction;
static const uint8_t phases[8]={1U,3U,2U,6U,4U,12U,8U,9U};
static void setup(void)
{
    GPIO_InitTypeDef gpio;
    if (initialized) return;
    RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOB|RCC_AHB1Periph_GPIOD,ENABLE);
    GPIO_ResetBits(GPIOD,GPIO_Pin_10|GPIO_Pin_11|GPIO_Pin_12);
    GPIO_ResetBits(GPIOB,GPIO_Pin_8);
    GPIO_StructInit(&gpio);gpio.GPIO_Mode=GPIO_Mode_OUT;
    gpio.GPIO_OType=GPIO_OType_PP;gpio.GPIO_PuPd=GPIO_PuPd_DOWN;
    gpio.GPIO_Speed=GPIO_Speed_2MHz;
    gpio.GPIO_Pin=GPIO_Pin_10|GPIO_Pin_11|GPIO_Pin_12;GPIO_Init(GPIOD,&gpio);
    gpio.GPIO_Pin=GPIO_Pin_8;GPIO_Init(GPIOB,&gpio);phase=7U;initialized=1U;
}
static void apply(uint32_t bits)
{
    uint16_t pd=(uint16_t)((bits&7U)<<10);
    /* Each bank uses one BSRR write; no read-modify-write of unrelated pins. */
    (*(volatile uint32_t *)&GPIOD->BSRRL)=(uint32_t)pd|((uint32_t)((GPIO_Pin_10|GPIO_Pin_11|GPIO_Pin_12)&~pd)<<16);
    (*(volatile uint32_t *)&GPIOB->BSRRL)=(bits&8U)?GPIO_Pin_8:((uint32_t)GPIO_Pin_8<<16);
    g_radar_lift_phase_bits=bits;
}
#endif
void RadarLift_Service(uint32_t now_ms,uint32_t safe_idle)
{
    uint32_t request=g_radar_lift_request;
    g_radar_lift_request=0U;
#if APP_IS_DEBUG && !APP_X_OFFLINE_ONLY
    if (g_radar_lift_state==1U && (safe_idle!=1U || g_radar_lift_enable!=1U)) {
        g_radar_lift_state=3U;g_radar_lift_result=2U;
    }
    if (request!=0U) {
        g_radar_lift_sequence++;
        if (request==2U) {
            if (g_radar_lift_state==1U) g_radar_lift_state=3U;
            g_radar_lift_result=5U;return;
        }
        if (request!=1U || g_radar_lift_steps<1U || g_radar_lift_steps>RADAR_LIFT_MAX_STEPS || g_radar_lift_direction>1U)
            {g_radar_lift_result=3U;return;}
        if (safe_idle!=1U || g_radar_lift_enable!=1U)
            {g_radar_lift_result=2U;return;}
        if (g_radar_lift_state==1U) {g_radar_lift_result=4U;return;}
        total=g_radar_lift_steps;direction=g_radar_lift_direction;
        setup();g_radar_lift_done=0U;last_tick=start_tick=now_ms;
        g_radar_lift_state=1U;g_radar_lift_result=1U;
    }
    if (g_radar_lift_state!=1U) return;
    if ((uint32_t)(now_ms-start_tick)>total*RADAR_LIFT_INTERVAL_MS+250U) {
        g_radar_lift_state=3U;g_radar_lift_result=2U;return;
    }
    if ((uint32_t)(now_ms-last_tick)<RADAR_LIFT_INTERVAL_MS) return;
    last_tick=now_ms; /* No catch-up burst after a delayed task. */
    phase=(phase+(direction?7U:1U))&7U;apply(phases[phase]);
    g_radar_lift_done++;
    if (g_radar_lift_done>=total) g_radar_lift_state=2U;
#else
    (void)now_ms;(void)safe_idle;
    if (request) {g_radar_lift_sequence++;g_radar_lift_result=2U;}
#endif
}
