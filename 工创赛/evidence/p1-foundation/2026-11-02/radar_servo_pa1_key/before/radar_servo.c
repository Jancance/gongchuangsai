#include "radar_servo.h"
#include "radar_servo_logic.h"
#include "stm32f4xx.h"
#include "stm32f4xx_tim.h"
#include "zdt_x_config.h"
#include "app_entry_config.h"
volatile uint32_t g_radar_servo_state=0U;
volatile float g_radar_servo_angle_deg[2]={RADAR_SERVO1_INIT_DEG,RADAR_SERVO2_INIT_DEG};
volatile uint32_t g_radar_servo_departure_done=0U,g_radar_servo_departure_id=0U;
volatile uint32_t g_radar_servo_pulse_us[2]={0U,0U};
uint32_t RadarServo_Init(void)
{
#if APP_X_OFFLINE_ONLY
    /* 离线构建不初始化GPIO/TIM，不产生舵机动作。 */
    g_radar_servo_state=2U;return 0U;
#else
    RCC_ClocksTypeDef clocks;
    GPIO_InitTypeDef gpio;
    TIM_TimeBaseInitTypeDef timer;
    TIM_OCInitTypeDef channel;
    uint32_t timer_hz;
    uint16_t first_pulse,second_pulse;
    if (g_radar_servo_state == 1U) return 1U;
    if (!RadarServo_Encode(1U,RADAR_SERVO1_INIT_DEG,&first_pulse) ||
        !RadarServo_Encode(2U,RADAR_SERVO2_INIT_DEG,&second_pulse)) {g_radar_servo_state=4U;return 0U;}
    RCC_GetClocksFreq(&clocks);timer_hz=clocks.PCLK1_Frequency;
    if ((RCC->CFGR & RCC_CFGR_PPRE1) != 0U) timer_hz*=2U;
    if (timer_hz<1000000U || timer_hz%1000000U || timer_hz/1000000U>65536U) {
        g_radar_servo_state=3U;return 0U;
    }
    RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOB,ENABLE);
    GPIO_ResetBits(GPIOB,GPIO_Pin_6|GPIO_Pin_7);
    GPIO_StructInit(&gpio);gpio.GPIO_Pin=GPIO_Pin_6|GPIO_Pin_7;
    gpio.GPIO_Mode=GPIO_Mode_OUT;gpio.GPIO_OType=GPIO_OType_PP;
    gpio.GPIO_PuPd=GPIO_PuPd_DOWN;gpio.GPIO_Speed=GPIO_Speed_50MHz;
    GPIO_Init(GPIOB,&gpio);
    RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM4,ENABLE);
    /* TIM4由双雷达PWM统一拥有；禁止其他模块另改公共计数周期。 */
    TIM_Cmd(TIM4,DISABLE);
    TIM_TimeBaseStructInit(&timer);timer.TIM_Prescaler=(uint16_t)(timer_hz/1000000U-1U);
    timer.TIM_Period=RADAR_SERVO_PERIOD_US-1U;timer.TIM_CounterMode=TIM_CounterMode_Up;
    TIM_TimeBaseInit(TIM4,&timer);
    TIM_OCStructInit(&channel);channel.TIM_OCMode=TIM_OCMode_PWM1;
    channel.TIM_OutputState=TIM_OutputState_Enable;channel.TIM_OCPolarity=TIM_OCPolarity_High;
    channel.TIM_Pulse=first_pulse;TIM_OC1Init(TIM4,&channel);
    channel.TIM_Pulse=second_pulse;TIM_OC2Init(TIM4,&channel);
    TIM_OC1PreloadConfig(TIM4,TIM_OCPreload_Enable);TIM_OC2PreloadConfig(TIM4,TIM_OCPreload_Enable);
    TIM_ARRPreloadConfig(TIM4,ENABLE);TIM_SetCounter(TIM4,0U);
    TIM_GenerateEvent(TIM4,TIM_EventSource_Update);
    GPIO_PinAFConfig(GPIOB,GPIO_PinSource6,GPIO_AF_TIM4);
    GPIO_PinAFConfig(GPIOB,GPIO_PinSource7,GPIO_AF_TIM4);
    gpio.GPIO_Mode=GPIO_Mode_AF;GPIO_Init(GPIOB,&gpio);TIM_Cmd(TIM4,ENABLE);
    g_radar_servo_angle_deg[0]=RADAR_SERVO1_INIT_DEG;g_radar_servo_angle_deg[1]=RADAR_SERVO2_INIT_DEG;
    g_radar_servo_pulse_us[0]=first_pulse;g_radar_servo_pulse_us[1]=second_pulse;
    g_radar_servo_departure_done=g_radar_servo_departure_id=0U;
    g_radar_servo_state=1U;return 1U;
#endif
}
uint32_t RadarServo_SetAngle(uint32_t id, float angle_deg)
{
    uint16_t pulse;
    if (g_radar_servo_state!=1U || !RadarServo_Encode(id,angle_deg,&pulse)) return 0U;
#if !APP_X_OFFLINE_ONLY
    if (id==1U) TIM_SetCompare1(TIM4,pulse);else TIM_SetCompare2(TIM4,pulse);
    g_radar_servo_angle_deg[id-1U]=angle_deg;g_radar_servo_pulse_us[id-1U]=pulse;
    return 1U;
#else
    return 0U;
#endif
}

volatile uint32_t g_radar_servo_test_request=0U,g_radar_servo_test_id=1U;
volatile uint32_t g_radar_servo_test_result=0U,g_radar_servo_test_sequence=0U;
volatile float g_radar_servo_test_angle_deg=0.0f;
void RadarServo_DebugService(uint32_t safe_idle)
{
    uint32_t request=g_radar_servo_test_request,id;
    float angle;
    if (request==0U) return;
    id=g_radar_servo_test_id;angle=g_radar_servo_test_angle_deg;
    g_radar_servo_test_request=0U;
#if APP_IS_DEBUG && !APP_X_OFFLINE_ONLY
    if (safe_idle!=1U || g_radar_servo_state!=1U) g_radar_servo_test_result=2U;
    else if (request!=1U || id<1U || id>2U || !(angle>=0.0f && angle<=RADAR_SERVO_TEST_MAX_DEG))
        g_radar_servo_test_result=3U;
    else g_radar_servo_test_result=RadarServo_SetAngle(id,angle)?1U:2U;
#else
    (void)safe_idle;(void)id;(void)angle;
    g_radar_servo_test_result=2U;
#endif
    g_radar_servo_test_sequence++;
}

/* 仅由已获准且实际进入RUN的首段调用；每轮上电只展开锁定的一套。 */
uint32_t RadarServo_OnDeparture(uint32_t zone)
{
    float target;
    if (zone<1U || zone>2U || g_radar_servo_state!=1U) return 0U;
    if (g_radar_servo_departure_done) return g_radar_servo_departure_id==zone?1U:0U;
    target=zone==1U?RADAR_SERVO1_DEPLOY_DEG:RADAR_SERVO2_DEPLOY_DEG;
    if (!RadarServo_SetAngle(zone,target)) return 0U;
    g_radar_servo_departure_id=zone;g_radar_servo_departure_done=1U;return 1U;
}
