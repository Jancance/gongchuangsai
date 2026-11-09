#include <stdint.h>
#include <assert.h>
#include <math.h>
#include <stdio.h>
#define JIAZHUA_ANGLE_MAX_DEG 270.0f
#define JIAZHUA_PULSE_MIN_US 500U
#define JIAZHUA_PULSE_MAX_US 2500U
#define TIM2 0
#define ENABLE 1
#define GPIO_Mode_AF 2
#define TIM_EventSource_Update 1
static uint8_t s_ready=1,s_running=1;
static uint32_t pulse=1833,updates=0;
void TIM_SetCompare1(int t,uint32_t p){pulse=p;}
void TIM_GenerateEvent(int t,int e){updates++;}
void TIM_Cmd(int t,int e){}
void Gripper_PinMode(int m){}
uint8_t jiazhua(float angle_deg)
{
    uint32_t pulse_us;

    /* 此写法同时拒绝越界、NaN和无穷值，不把错误角度夹到端点。 */
    if ((s_ready == 0U) ||
        !(angle_deg >= 0.0f && angle_deg <= JIAZHUA_ANGLE_MAX_DEG))
    {
        return 0U;
    }
    pulse_us = JIAZHUA_PULSE_MIN_US + (uint32_t)(
        angle_deg * (float)(JIAZHUA_PULSE_MAX_US - JIAZHUA_PULSE_MIN_US)
        / JIAZHUA_ANGLE_MAX_DEG + 0.5f);
    TIM_SetCompare1(TIM2, pulse_us);
    if (s_running == 0U)
    {
        TIM_GenerateEvent(TIM2, TIM_EventSource_Update);
        TIM_Cmd(TIM2, ENABLE);
        Gripper_PinMode(GPIO_Mode_AF);
        s_running = 1U;
    }
    /* 已运行时在下一PWM周期更新，不反复清零计数器。 */
    return 1U;
}


int main(void){unsigned checks=0;assert(jiazhua(180));assert(pulse==1833);checks+=2;for(int i=1;i<=20;i++){float a=180+30.0f*i/20;assert(jiazhua(a));assert(pulse>=1833 && pulse<=2056);checks+=2;}assert(pulse==2056 && updates==0);checks++;for(int i=0;i<5;i++){float a[]={-1,271,NAN,INFINITY,-INFINITY};assert(!jiazhua(a[i]));assert(pulse==2056);checks+=2;}s_ready=0;assert(!jiazhua(210));assert(pulse==2056);checks+=2;printf("checks=%u failures=0 pulse=%u\n",checks,pulse);}
