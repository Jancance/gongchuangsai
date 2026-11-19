"""Isolated third-region candidate; requires the local MATLAB prevalidation."""
from pathlib import Path
import json, shutil
R=Path(__file__).resolve().parents[1]
BASE=R/'tmp/source_real_qr_20261116/project'
P=R/'tmp/stage3_dispatch_20261116/project'
E=R/'evidence/integration/2026-11-16/stage3'

def edit(name, pairs):
    p=P/name
    data=p.read_bytes()
    for old,new in pairs:
        old=old.encode('ascii');new=new.encode('ascii')
        assert data.count(old)==1,(name,old)
        data=data.replace(old,new,1)
    p.write_bytes(data)

def main():
    proof=json.loads((E/'matlab-result.json').read_text(encoding='utf-8'))
    assert proof['status']=='PASS_PREIMPLEMENTATION_DISCRETE_MODEL_ONLY'
    assert len(proof['checks'])==12
    assert not P.exists(),'Preserve existing candidate'
    shutil.copytree(BASE,P,ignore=shutil.ignore_patterns('Objects','Listings'))
    for name in ['stage3_cycle.h','stage3_cycle.inc','region_dispatch.h','region_dispatch.inc']:
        p=R/'src/stage3_20261116'/name
        (P/'CHASSIS/USER'/name).write_bytes(p.read_bytes().replace(b'\r\n',b'\n').replace(b'\n',b'\r\n'))
    edit('CHASSIS/USER/main.c',[
      ('#include "source_body_step.h"','#include "source_body_step.h"\r\n#include "stage3_cycle.h"\r\n#include "region_dispatch.h"'),
      ('SourceCycle_ScanService();','RegionDispatch_ScanService();'),
      ('g_start_key_count++;SourceCycle_KeyEvent();','g_start_key_count++;RegionDispatch_KeyEvent();'),
      ('((s_heading_exec_relative <= -5.0f) && (s_heading_exec_relative >= -90.0f))))) ||',
       '((s_heading_exec_relative <= -5.0f) && (s_heading_exec_relative >= -90.0f)))) &&\r\n             !(Stage3_FineTurnAllowed() && fabsf(s_heading_exec_relative)>1.0f && fabsf(s_heading_exec_relative)<=10.0f)) ||'),
      ('prepared_ok=s_route_heading_turn_active ?',
       'prepared_ok=Stage3_FineTurnAllowed() ?\r\n                HeadingSession_StartFine(&s_heading_prepared,&input,s_heading_exec_relative,&output) :\r\n                s_route_heading_turn_active ?'),
      ('uint32_t TurnStart(uint32_t direction, float angle_deg, uint32_t max_rpm)\r\n{\r\n    uint32_t accepted = 0U;\r\n    if (direction > TURN_CCW || !(angle_deg >= MOTION_TURN_MIN_DEG && angle_deg <= MOTION_TURN_MAX_DEG) ||',
       'uint32_t TurnStart(uint32_t direction, float angle_deg, uint32_t max_rpm)\r\n{\r\n    uint32_t accepted = 0U;\r\n    if (direction > TURN_CCW || !((angle_deg >= MOTION_TURN_MIN_DEG && angle_deg <= MOTION_TURN_MAX_DEG) ||\r\n        (Stage3_FineTurnAllowed() && angle_deg>1.0f && angle_deg<=10.0f)) ||')])
    edit('CHASSIS/USER/source_body_step.inc',[
      ('fabsf(g_source_align_turn_deg)>=5.0f && fabsf(g_source_align_turn_deg)<=10.0f',
       '(fabsf(g_source_align_turn_deg)>=5.0f || (Stage3_FineTurnAllowed() && fabsf(g_source_align_turn_deg)>1.0f)) && fabsf(g_source_align_turn_deg)<=10.0f')])
    edit('CHASSIS/USER/heading_session.h',[
      ('void HeadingSession_Step(', 'uint32_t HeadingSession_StartFine(HeadingSession*,const HeadingSessionInput*,float,HeadingSessionOutput*);\nvoid HeadingSession_Step(')])
    # Pure heading-session implementation keeps ordinary admission unchanged.
    edit('CHASSIS/USER/heading_session.c',[
      ('#include <stddef.h>','#include <stddef.h>\n#include <math.h>'),
      ('uint32_t HeadingSession_Start(HeadingSession *s, const HeadingSessionInput *in,\n    float relative_deg, HeadingSessionOutput *out)',
       'static uint32_t HS_Start(HeadingSession *s, const HeadingSessionInput *in,\n    float relative_deg, HeadingSessionOutput *out, uint32_t fine)'),
      ('!(((relative_deg >= 5.0f) && (relative_deg <= ((APP_TEST_MODE == 3U) ? 90.0f : 45.0f))) ||\n          ((relative_deg <= -5.0f) && (relative_deg >= ((APP_TEST_MODE == 3U) ? -90.0f : -45.0f))))) return 0U;',
       '!((!fine && (((relative_deg >= 5.0f) && (relative_deg <= ((APP_TEST_MODE == 3U) ? 90.0f : 45.0f))) ||\n          ((relative_deg <= -5.0f) && (relative_deg >= ((APP_TEST_MODE == 3U) ? -90.0f : -45.0f))))) ||\n          (fine && APP_TEST_MODE==3U && fabsf(relative_deg)>1.0f && fabsf(relative_deg)<=10.0f))) return 0U;'),
      ('uint32_t HeadingSession_StartReference(',
       'uint32_t HeadingSession_Start(HeadingSession *s,const HeadingSessionInput *in,float relative_deg,HeadingSessionOutput *out)\n{return HS_Start(s,in,relative_deg,out,0U);}\n\nuint32_t HeadingSession_StartFine(HeadingSession *s,const HeadingSessionInput *in,float relative_deg,HeadingSessionOutput *out)\n{return HS_Start(s,in,relative_deg,out,1U);}\n\nuint32_t HeadingSession_StartReference(')])
    edit('CHASSIS/USER/integration10_complete.inc',[
      ('static uint32_t SourceCycle_SharedByte(uint8_t,uint32_t,uint32_t);',
       '#include "stage3_cycle.h"\r\n#include "region_dispatch.h"\r\nstatic uint32_t RegionDispatch_SharedByte(uint8_t,uint32_t,uint32_t);\r\nstatic void RegionDispatch_Telemetry(uint32_t);\r\nstatic void RegionDispatch_Service(uint32_t);\r\nstatic uint32_t SourceCycle_SharedByte(uint8_t,uint32_t,uint32_t);'),
      ('!SourceCycle_SharedByte(b,','!RegionDispatch_SharedByte(b,'),
      (' SourceCycle_Telemetry((uint32_t)xTaskGetTickCount());',' RegionDispatch_Telemetry((uint32_t)xTaskGetTickCount());'),
      (' (void)SourceObserver_TrySend(USART6_VisionSend);',' if(g_region_dispatch_area!=3U)(void)SourceObserver_TrySend(USART6_VisionSend);'),
      ('#include "source_mcu_cycle.inc"','#include "source_mcu_cycle.inc"\r\n#include "stage3_cycle.inc"\r\n#include "region_dispatch.inc"'),
      (' SourceCycle_Service((uint32_t)xTaskGetTickCount());',' RegionDispatch_Service((uint32_t)xTaskGetTickCount());'),
      ('if(g_source_parallel_state==3U){it_done();}',
       'if(g_source_parallel_state==3U){\r\n#if !APP_MOTOR_OFFLINE_ONLY\r\n   if(Stage3_Owns() && (it_command.axes_mask&SPF_LIFT)){\r\n    g_lift_ruler_controller.position_after_raw=(int64_t)(vsp_axis[1].current*(float)g_lift_ruler_controller.top_scale);\r\n    g_lift_ruler_controller.backoff_valid=0U;g_lift_ruler_controller.operation=2U;g_lift_ruler_controller.samples++;\r\n   }\r\n#endif\r\n   it_done();}')])
    # Give the shared wheel-hold service an owner-neutral API. Existing Low ABI remains.
    edit('CHASSIS/USER/pzq_commit_overlap.inc',[
      ('static void Low_WheelHoldService(void)','static void WheelHold_Service(void)'),
      ('Low_ReadonlyDiag();Low_WheelHoldService();','Low_ReadonlyDiag();WheelHold_Service();'),
      ('  g_low_wheel_hold_request=0U;','  g_low_wheel_hold_request=0U;\r\n  g_low_wheel_hold_ready=0U;g_low_wheel_hold_stage=0U;')])
    assert (P/'CHASSIS/USER/source_mcu_cycle.inc').read_bytes()==(BASE/'CHASSIS/USER/source_mcu_cycle.inc').read_bytes()
    print(P)
if __name__=='__main__':main()
