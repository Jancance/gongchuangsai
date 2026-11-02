"""Install two standard-library radar PWM channels and a debounced start-zone input.
No probe, serial, download or hardware commands.
"""
from pathlib import Path
import hashlib,json
ROOT=Path(__file__).resolve().parents[2];FW=ROOT/'3.stm32f407工程'
OUT=ROOT/'工创赛/evidence/p1-foundation/2026-11-02/dual_radar_servo'
OUT.mkdir(parents=True,exist_ok=True)
paths=['USER/main.c','gongxun-f407.uvprojx','gongxun-f407-offline.uvprojx']
for name in paths:
 p=OUT/'before'/name;p.parent.mkdir(parents=True,exist_ok=True)
 if not p.exists():p.write_bytes((FW/name).read_bytes())
def hashes():
 return {p.relative_to(FW).as_posix():hashlib.sha256(p.read_bytes()).hexdigest() for p in FW.rglob('*') if p.is_file() and p.suffix in ('.c','.h','.s','.uvprojx')}
snapshot=OUT/'before_hashes.json'
if snapshot.exists():before=json.loads(snapshot.read_text(encoding='utf8'))
else:
 before=hashes();snapshot.write_text(json.dumps(before,indent=2),encoding='utf8')
files={
'radar_servo_config.h':'''#ifndef RADAR_SERVO_CONFIG_H
#define RADAR_SERVO_CONFIG_H
/* 双180度PWM舵机。当前为待实物确认的标称映射，不是机械零点标定。 */
#define RADAR_SERVO_COUNT 2U
#define RADAR_SERVO_MAX_DEG 180.0f
#define RADAR_SERVO_PERIOD_US 20000U
#define RADAR_SERVO_ZERO_US 500U
#define RADAR_SERVO_180_US 2500U
#define RADAR_ZONE_DEBOUNCE_MS 50U
/* PA1外部10k上拉：接地档为启停区1，悬空上拉档为启停区2。 */
#define RADAR_ZONE_LOW_ID 1U
#if RADAR_SERVO_ZERO_US < 400U || RADAR_SERVO_180_US > 2600U || RADAR_SERVO_ZERO_US >= RADAR_SERVO_180_US
#error Invalid_radar_servo_pulse_range
#endif
#endif
''',
'radar_servo_logic.h':'''#ifndef RADAR_SERVO_LOGIC_H
#define RADAR_SERVO_LOGIC_H
#include <stdint.h>
#include "radar_servo_config.h"
typedef struct {
    uint32_t sample_started, candidate_level, candidate_ms, ready, zone;
    uint32_t locked, locked_zone;
} RadarZoneFilter;
uint32_t RadarServo_Encode(uint32_t id, float angle_deg, uint16_t *pulse_us);
void RadarZoneFilter_Init(RadarZoneFilter *filter);
void RadarZoneFilter_Sample(RadarZoneFilter *filter, uint32_t level, uint32_t now_ms);
uint32_t RadarZoneFilter_Lock(RadarZoneFilter *filter);
/* 仅在上层确认本轮结束、机构停稳后解锁，不在运行中跟随开关。 */
void RadarZoneFilter_Unlock(RadarZoneFilter *filter);
#endif
''',
'radar_servo_logic.c':'''#include "radar_servo_logic.h"
#include <stddef.h>
uint32_t RadarServo_Encode(uint32_t id, float angle_deg, uint16_t *pulse_us)
{
    float pulse;
    if (pulse_us == NULL || id < 1U || id > RADAR_SERVO_COUNT ||
        !(angle_deg >= 0.0f && angle_deg <= RADAR_SERVO_MAX_DEG)) return 0U;
    pulse = (float)RADAR_SERVO_ZERO_US + angle_deg *
        (float)(RADAR_SERVO_180_US-RADAR_SERVO_ZERO_US)/RADAR_SERVO_MAX_DEG;
    *pulse_us = (uint16_t)(pulse + 0.5f);
    return 1U;
}
void RadarZoneFilter_Init(RadarZoneFilter *f)
{
    if (f != NULL) *f = (RadarZoneFilter){0};
}
void RadarZoneFilter_Sample(RadarZoneFilter *f, uint32_t level, uint32_t now_ms)
{
    if (f == NULL || level > 1U) return;
    if (!f->sample_started || level != f->candidate_level) {
        f->sample_started=1U;f->candidate_level=level;f->candidate_ms=now_ms;
        f->ready=0U;return;
    }
    if ((uint32_t)(now_ms-f->candidate_ms) >= RADAR_ZONE_DEBOUNCE_MS) {
        f->zone = level == 0U ? RADAR_ZONE_LOW_ID : 3U-RADAR_ZONE_LOW_ID;
        f->ready=1U;
    }
}
uint32_t RadarZoneFilter_Lock(RadarZoneFilter *f)
{
    if (f == NULL) return 0U;
    if (f->locked) return f->locked_zone;
    if (!f->ready) return 0U;
    f->locked=1U;f->locked_zone=f->zone;return f->locked_zone;
}
void RadarZoneFilter_Unlock(RadarZoneFilter *f)
{
    if (f != NULL) {f->locked=0U;f->locked_zone=0U;}
}
''',
'radar_servo.h':'''#ifndef RADAR_SERVO_H
#define RADAR_SERVO_H
#include <stdint.h>
/* 状态0未初始化、1PWM就绪、2强制离线不输出、3时钟配置不适用。 */
extern volatile uint32_t g_radar_servo_state;
extern volatile float g_radar_servo_angle_deg[2];
extern volatile uint32_t g_radar_servo_pulse_us[2];
uint32_t RadarServo_Init(void);
/* id为1或2；只更新对应通道，返回1仅表示PWM命令写入，不是到位反馈。 */
uint32_t RadarServo_SetAngle(uint32_t id, float angle_deg);
#endif
''',
'radar_servo.c':'''#include "radar_servo.h"
#include "radar_servo_logic.h"
#include "stm32f4xx.h"
#include "stm32f4xx_tim.h"
#include "zdt_x_config.h"
volatile uint32_t g_radar_servo_state=0U;
volatile float g_radar_servo_angle_deg[2]={0.0f,0.0f};
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
    if (g_radar_servo_state == 1U) return 1U;
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
    channel.TIM_Pulse=RADAR_SERVO_ZERO_US;
    TIM_OC1Init(TIM4,&channel);TIM_OC2Init(TIM4,&channel);
    TIM_OC1PreloadConfig(TIM4,TIM_OCPreload_Enable);TIM_OC2PreloadConfig(TIM4,TIM_OCPreload_Enable);
    TIM_ARRPreloadConfig(TIM4,ENABLE);TIM_SetCounter(TIM4,0U);
    TIM_GenerateEvent(TIM4,TIM_EventSource_Update);
    GPIO_PinAFConfig(GPIOB,GPIO_PinSource6,GPIO_AF_TIM4);
    GPIO_PinAFConfig(GPIOB,GPIO_PinSource7,GPIO_AF_TIM4);
    gpio.GPIO_Mode=GPIO_Mode_AF;GPIO_Init(GPIOB,&gpio);TIM_Cmd(TIM4,ENABLE);
    g_radar_servo_angle_deg[0]=g_radar_servo_angle_deg[1]=0.0f;
    g_radar_servo_pulse_us[0]=g_radar_servo_pulse_us[1]=RADAR_SERVO_ZERO_US;
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
''',
'radar_zone_switch.h':'''#ifndef RADAR_ZONE_SWITCH_H
#define RADAR_ZONE_SWITCH_H
#include <stdint.h>
extern volatile uint32_t g_radar_zone_ready,g_radar_zone_selected,g_radar_zone_locked;
void RadarZoneSwitch_Init(void);
void RadarZoneSwitch_Service(uint32_t now_ms);
uint32_t RadarZoneSwitch_Lock(void);
void RadarZoneSwitch_Unlock(void);
#endif
''',
'radar_zone_switch.c':'''#include "radar_zone_switch.h"
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
'''}
dest=FW/'HARDWARE/RADAR_SERVO';dest.mkdir(exist_ok=True)
for name,text in files.items():
 p=dest/name;data=text.replace('\n','\r\n').encode('gbk')
 if p.exists():assert p.read_bytes()==data,'Refuse overwrite changed new module'
 else:p.write_bytes(data)
p=FW/'USER/main.c';raw=(OUT/'before/USER/main.c').read_bytes();s=raw.decode('gbk');nl='\r\r\n' if b'\r\r\n' in raw else '\r\n'
assert s.count('#include "app_config.h"')==1 and s.count('    Power_Init();')==1
s=s.replace('#include "app_config.h"','#include "app_config.h"'+nl+'#include "radar_servo.h"'+nl+'#include "radar_zone_switch.h"')
s=s.replace('    Power_Init();','    Power_Init();'+nl+'    RadarZoneSwitch_Init();'+nl+'    (void)RadarServo_Init();')
pos=s.index('if (APP_UART_LINK_TEST',s.index('static void App_KeyTask('))
s=s[:pos]+'        RadarZoneSwitch_Service((uint32_t)(xTaskGetTickCount()*portTICK_PERIOD_MS));'+nl+s[pos:]
data=s.encode('gbk');assert p.read_bytes() in (raw,data),'Main changed concurrently';p.write_bytes(data)
for name in paths[1:]:
 p=FW/name;b=(OUT/'before'/name).read_bytes();old=b;nl=b'\r\n' if b'\r\n' in b else b'\n'
 assert b'.\\HARDWARE\\RADAR_SERVO' not in b
 needle=b'.\\HARDWARE\\SAOMA</IncludePath>';assert b.count(needle)==1
 b=b.replace(needle,b'.\\HARDWARE\\SAOMA;.\\HARDWARE\\RADAR_SERVO</IncludePath>')
 group=['          <Group>','            <GroupName>RADAR_SERVO</GroupName>','            <Files>']
 for source in ['radar_servo.c','radar_servo_logic.c','radar_zone_switch.c']:
  group+=['              <File>','                <FileName>'+source+'</FileName>','                <FileType>1</FileType>','                <FilePath>.\\HARDWARE\\RADAR_SERVO\\'+source+'</FilePath>','              </File>']
 group+=['              <File>','                <FileName>stm32f4xx_tim.c</FileName>','                <FileType>1</FileType>','                <FilePath>.\\StdPeriph_Driver\\src\\stm32f4xx_tim.c</FilePath>','              </File>']
 group+=['            </Files>','          </Group>']
 anchor=b'</Groups>';assert b.count(anchor)==1
 index=b.index(anchor);begin=b.rfind(b'\n',0,index)+1;indent=b[begin:index]
 b=b[:begin]+nl.join(x.encode() for x in group)+nl+indent+anchor+b[index+len(anchor):]
 assert p.read_bytes() in (old,b),'Project changed concurrently';p.write_bytes(b)
after=hashes();changed=[k for k in before if before[k]!=after[k]]
assert set(changed)==set(paths),changed
(OUT/'manifest.json').write_text(json.dumps({'changed_existing':changed,'added':list(files),'GPIO_PWM':['PB6/TIM4_CH1','PB7/TIM4_CH2'],'initial_degrees':[0,0],'nominal_pulse_us':[500,2500],'pulse_calibration':'not physically verified','PA1_debounce_ms':50,'offline_PWM_blocked':True,'other_existing_sources_unchanged':True,'downloaded':False,'github_pulled':False,'after_hashes':after},ensure_ascii=False,indent=2),encoding='utf8')
print(json.dumps({'changed':changed,'added':list(files),'hardware_accessed':False},ensure_ascii=False))
