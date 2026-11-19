"""Prepare an isolated MCU-cycle candidate after the local MATLAB result."""
import json
from pathlib import Path
import shutil

R=Path(__file__).resolve().parents[1]
E=R/'evidence/integration/2026-11-15/source-mcu-cycle'
BASE=R/'tmp/source_top_window10_yaw165_20261115/project'
P=R/'tmp/source_mcu_cycle_20261115/project'

def replace(data,old,new):
    assert data.count(old)==1,old[:90]
    return data.replace(old,new,1)

def configure_key_and_buzzer():
    # These two original files use GBK comments and LF; preserve their bytes.
    h=BASE/'CHASSIS/HARDWARE/BUZZER/buzzer.h';d=h.read_bytes()
    d=replace(d,b'void Buzzer_Update(void);',b'void Buzzer_Update(void);\nvoid Buzzer_StartTimed(uint32_t count,uint32_t on_ms,uint32_t gap_ms);\nuint32_t Buzzer_IsBusy(void);')
    (P/h.relative_to(BASE)).write_bytes(d)
    c=BASE/'CHASSIS/HARDWARE/BUZZER/buzzer.c';d=c.read_bytes()
    d=replace(d,b'static TickType_t g_buzzer_phase_tick = 0U;',b'static TickType_t g_buzzer_phase_tick = 0U;\nstatic uint32_t s_buzzer_on_ms=APP_BEEP_ON_MS,s_buzzer_gap_ms=APP_BEEP_GAP_MS;')
    d=replace(d,b'void Buzzer_Start(uint32_t count)\n{',b'uint32_t Buzzer_IsBusy(void){return g_buzzer_remaining!=0U;}\nvoid Buzzer_Start(uint32_t count){Buzzer_StartTimed(count,APP_BEEP_ON_MS,APP_BEEP_GAP_MS);}\nvoid Buzzer_StartTimed(uint32_t count,uint32_t on_ms,uint32_t gap_ms)\n{\n    taskENTER_CRITICAL();\n    g_buzzer_remaining=0U;\n    s_buzzer_on_ms=on_ms;s_buzzer_gap_ms=gap_ms;')
    d=replace(d,b'APP_BEEP_ON_MS : APP_BEEP_GAP_MS);',b's_buzzer_on_ms : s_buzzer_gap_ms);')
    d=replace(d,b'        GPIO_SetBits(GPIOB, GPIO_Pin_0);\n    }\n}',b'        GPIO_SetBits(GPIOB, GPIO_Pin_0);\n    }\n    taskEXIT_CRITICAL();\n}')
    (P/c.relative_to(BASE)).write_bytes(d)
    p=P/'CHASSIS/USER/vehicle_arm.h';d=(BASE/p.relative_to(P)).read_bytes()
    d=replace(d,b'void Integration10_Service(void);',b'void Integration10_Service(void);\r\nvoid SourceCycle_KeyEvent(void);');p.write_bytes(d)
    p=P/'CHASSIS/USER/main.c';d=(BASE/p.relative_to(P)).read_bytes()
    old=b'        if(VEHICLE_INTEGRATED_ENABLE)\r\n        {if(Key_GetEvent()!=0U){taskENTER_CRITICAL();g_vehicle_key=1U;taskEXIT_CRITICAL();}}'
    new=b'        if(SOURCE_MCU_CYCLE_ENABLE)\r\n        {if(Key_GetEvent()!=0U){g_start_key_count++;SourceCycle_KeyEvent();}}\r\n        else if(VEHICLE_INTEGRATED_ENABLE)\r\n        {if(Key_GetEvent()!=0U){taskENTER_CRITICAL();g_vehicle_key=1U;taskEXIT_CRITICAL();}}'
    p.write_bytes(replace(d,old,new))

def main():
    m=json.loads((E/'matlab-result.json').read_text())
    assert m['passed'] and m['local_matlab'] and m['extension_arrival_mm']==5
    assert not P.exists(),'Preserve existing candidates'
    shutil.copytree(BASE,P,ignore=shutil.ignore_patterns('Objects','Listings'))
    p=P/'USER/vehicle_parameters.h';d=p.read_bytes()
    lines=d.splitlines(keepends=True)
    found=[i for i,x in enumerate(lines) if x.startswith(b'#define SOURCE_EXTENSION_DONE_DEG ')]
    assert len(found)==1
    lines[found[0]]=b'#define SOURCE_EXTENSION_DONE_DEG (5.0f * VEHICLE_EXTENSION_PULSES_PER_MM * 360.0f / 3200.0f) /* User2026-11-15: normal arrival5mm; zero unchanged. */\r\n'
    d=b''.join(lines)
    d=replace(d,b'#define VEHICLE_INTEGRATED_ENABLE 1U',b'#define SOURCE_MCU_CYCLE_ENABLE 1U /* PC13 owns this isolated source cycle. */\r\n#define VEHICLE_INTEGRATED_ENABLE 1U')
    p.write_bytes(d)
    p=P/'CHASSIS/USER/source_mcu_cycle.inc';p.write_bytes((R/'output/source_mcu_cycle_20261115.inc').read_text(encoding='utf-8').replace('\n','\r\n').encode('utf-8'))
    p=P/'CHASSIS/USER/integration10_complete.inc';d=p.read_bytes()
    declarations=b'static uint32_t SourceCycle_SharedByte(uint8_t,uint32_t,uint32_t);\r\nstatic void SourceCycle_Telemetry(uint32_t);\r\n'
    d=replace(d,b'#include "source_body_step.h"\r\n',b'#include "source_body_step.h"\r\n'+declarations)
    d=replace(d,b'if(!SourceObserver_SharedByte(b,(uint32_t)xTaskGetTickCount(),vis_len!=0U))Vision_Byte(b);',b'if(!SourceCycle_SharedByte(b,(uint32_t)xTaskGetTickCount(),vis_len!=0U) &&\r\n     !SourceObserver_SharedByte(b,(uint32_t)xTaskGetTickCount(),vis_len!=0U))Vision_Byte(b);')
    d=replace(d,b' (void)SourceObserver_TrySend(USART6_VisionSend);',b' SourceCycle_Telemetry((uint32_t)xTaskGetTickCount());\r\n (void)SourceObserver_TrySend(USART6_VisionSend);')
    d=replace(d,b'#include "pzq_commit_overlap.inc"\r\n',b'#include "pzq_commit_overlap.inc"\r\n#include "source_mcu_cycle.inc"\r\n')
    d=replace(d,b' Low_Service(now);\r\n',b' Low_Service(now);\r\n SourceCycle_Service((uint32_t)xTaskGetTickCount());\r\n')
    p.write_bytes(d)
    configure_key_and_buzzer()
    print('Prepared isolated MCU cycle, normal arrival5mm; no hardware access')

if __name__=='__main__':main()
