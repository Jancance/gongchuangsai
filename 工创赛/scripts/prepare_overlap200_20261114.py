"""Prepare only the user-requested overlapping single-commit revision."""
from pathlib import Path
import json,shutil

R=Path(__file__).resolve().parents[1]
B=R/'tmp/source_top_pick_pb9_20261114/project'
P=R/'tmp/source_top_overlap200_20261114/project'
E=R/'evidence/integration/2026-11-14/overlap200-commit'

def edit(rel,pairs):
    p=P/rel;b=p.read_bytes()
    for a,c in pairs:
        a=a.encode('ascii');c=c.encode('ascii');assert a in b,(rel,a);b=b.replace(a,c)
    p.write_bytes(b)

def main():
    assert json.loads((E/'matlab-result.json').read_text())['passed']
    assert json.loads((E/'body-registration-matlab.json').read_text())['passed']
    assert not P.exists(),'Preserve existing candidate'
    shutil.copytree(B,P,ignore=shutil.ignore_patterns('Objects','Listings'))
    edit('USER/vehicle_parameters.h',[
      ('#define VEHICLE_EXTENSION_RPM 106.0f','#define VEHICLE_EXTENSION_RPM 200.0f'),
      ('#define STARTUP_EXTENSION_RPM 106.0f','#define STARTUP_EXTENSION_RPM 200.0f')])
    p=P/'USER/vehicle_parameters.h';b=p.read_bytes();at=b.rfind(b'#endif')
    extra='''
/* 2026-11-14 用户要求：一次2帧触发整件；X/Z重叠，先收臂再转槽。 */
#define SOURCE_OVERLAP_HOVER_PULSES 15600U /* 原39mm/80mm净空点，按新400p/mm换算 */
#define SOURCE_LOADED_PARTIAL_RETRACT_MM 60.0f /* MATLAB轨迹段，非新增停车门限 */
#define SOURCE_RELEASE_DWELL_MS 50U /* 松爪脉冲生效后计时，用户指定 */
#define SOURCE_OBSERVE_PULSES 600U /* 回到原框选的顶部1.5mm退出位 */
#define SOURCE_EXTENSION_ZERO_DONE_DEG 17.75f /* 仅新动作：原2度放宽约0.5mm */
#define SOURCE_WHEEL_HOLD_SPEED_RAW 37U /* 已有15mm/s低速档折算；Emm输入速度/10 */
'''.replace('\n','\r\n').encode('utf-8')
    p.write_bytes(b[:at]+extra+b[at:])
    edit('CHASSIS/USER/source_parallel_flow.h',[
      ('#define SPF_EXT_RPM 106.0f','#define SPF_EXT_RPM 200.0f')])
    edit('CHASSIS/USER/integration10_complete.inc',[
      ('g_it_ext_rpm_test:106.0f','g_it_ext_rpm_test:SPF_EXT_RPM'),
      ('Low_Service(now);Region_Service(now);PZQNear_Service(now);request=g_it_request;',
       'Low_Service(now);\r\n if(Low_Busy()){\r\n  uint32_t pending=g_it_request;g_it_request=0U;g_src_now_ms=now;Vision_Poll();\r\n  if(pending==9U)Low_Fail(2U);\r\n  else if(pending)g_it_rejected=1U;\r\n  return;\r\n }\r\n Region_Service(now);PZQNear_Service(now);request=g_it_request;')])
    edit('CHASSIS/USER/vehicle_arm.c',[
      ('static uint32_t Low_ObserveReturnSettling(void);',
       'static uint32_t Low_Busy(void);\r\nstatic uint32_t Low_ObserveReturnSettling(void);')])
    edit('CHASSIS/USER/vehicle_source_parallel.inc',[
      ('zero_return?2.0f:(address==6U?',
       'zero_return?(Low_Busy()?SOURCE_EXTENSION_ZERO_DONE_DEG:2.0f):(address==6U?')])
    edit('CHASSIS/SYSTEM/CAN/can.c',[
      ('address==5U?18464U:1060U','address==5U?18464U:(uint32_t)(VEHICLE_EXTENSION_RPM*10.0f)')])
    # gripper.c is UTF-8/LF; preserve it. No reset of shared TIM4 channels1/2.
    edit('HARDWARE/GRIPPER/gripper.c',[
      ('    TIM_SetCompare4(TIM4, pulse_us);',
       '    TIM_SetCompare4(TIM4, pulse_us);\n    TIM_ClearFlag(TIM4, TIM_FLAG_Update);')])
    p=P/'HARDWARE/GRIPPER/gripper.c';p.write_bytes(p.read_bytes()+b'\nuint8_t Gripper_PulseApplied(void)\n{ return s_running && TIM_GetFlagStatus(TIM4, TIM_FLAG_Update) != RESET; }\n')
    edit('HARDWARE/GRIPPER/gripper.h',[
      ('void Gripper_Stop(void);','void Gripper_Stop(void);\r\nuint8_t Gripper_PulseApplied(void);')])
    print(P)

if __name__=='__main__':main()
