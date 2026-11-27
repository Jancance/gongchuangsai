#include "radar_zone_switch.h"
#include "radar_servo_logic.h"
#include "stm32f4xx.h"
#include "FreeRTOS.h"
#include "task.h"
volatile uint32_t g_radar_zone_ready=0U,g_radar_zone_selected=0U,g_radar_zone_locked=0U;
static RadarZoneFilter s_filter;
void RadarZoneSwitch_Init(void)
{
    GPIO_InitTypeDef gpio;
    RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOA,ENABLE);
    GPIO_StructInit(&gpio);gpio.GPIO_Pin=GPIO_Pin_1;gpio.GPIO_Mode=GPIO_Mode_IN;
    gpio.GPIO_PuPd=GPIO_PuPd_NOPULL;GPIO_Init(GPIOA,&gpio);
    RadarZoneFilter_Init(&s_filter);
    g_radar_zone_ready=g_radar_zone_selected=g_radar_zone_locked=0U;
}
void RadarZoneSwitch_Service(uint32_t now_ms)
{
    taskENTER_CRITICAL();
    RadarZoneFilter_Sample(&s_filter,GPIO_ReadInputDataBit(GPIOA,GPIO_Pin_1),now_ms);
    g_radar_zone_ready=s_filter.ready;
    g_radar_zone_selected=s_filter.ready?s_filter.zone:0U;
    g_radar_zone_locked=s_filter.locked_zone;
    taskEXIT_CRITICAL();
}
uint32_t RadarZoneSwitch_Lock(void)
{
    uint32_t zone;
    taskENTER_CRITICAL();
    zone=RadarZoneFilter_Lock(&s_filter);g_radar_zone_locked=zone;
    taskEXIT_CRITICAL();
    return zone;
}
void RadarZoneSwitch_Unlock(void)
{
    taskENTER_CRITICAL();
    RadarZoneFilter_Unlock(&s_filter);g_radar_zone_locked=0U;
    taskEXIT_CRITICAL();
}
