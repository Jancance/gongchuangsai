"""Isolate current pzq50; add only supervised one-left10mm commissioning."""
from pathlib import Path
import hashlib,json,shutil
R=Path(__file__).resolve().parents[1];B=R/'tmp/pzq_release50_20261111/project'
P=R/'tmp/source_body_step_20261111/project';E=R/'evidence/integration/2026-11-11/source-body-step'
def edit(relative,old,new):
    p=P/relative;d=p.read_bytes();a=old.encode('ascii');b=new.encode('ascii')
    assert d.count(a)==1,(relative,old[:100],d.count(a));p.write_bytes(d.replace(a,b,1))
def main():
    assert json.loads((E/'matlab-result.json').read_text())['passed']
    assert not P.exists(),'Preserve existing candidate'
    shutil.copytree(B,P,ignore=shutil.ignore_patterns('Objects','Listings'))
    U='CHASSIS/USER/'
    for name in ['source_body_step.h','source_body_step.inc']:
        shutil.copy2(R/'software_interfaces/source_station'/name,P/U/name)
    # Include declarations in both normal main and the existing mechanical TU.
    for f in ['main.c','integration10_complete.inc','source_stage2_body_port.inc','chassis_auto_test.c']:
        path=P/U/f;d=path.read_bytes();path.write_bytes(b'#include "source_body_step.h"\r\n'+d)
    path=P/'CHASSIS/SYSTEM/SAFETY/safety.c';path.write_bytes(b'#include "source_body_step.h"\r\n'+path.read_bytes())
    edit(U+'main.c','        if(VEHICLE_INTEGRATED_ENABLE){/* Left-pick bench image: do not start the old mission. */ VehicleMission_Telemetry();}',
         '        if(VEHICLE_INTEGRATED_ENABLE){SourceBodyStep_Service((uint32_t)xTaskGetTickCount()); VehicleMission_Telemetry();}')
    edit(U+'main.c','#include "vehicle_mission.inc"','#include "vehicle_mission.inc"\r\n#include "source_body_step.inc"')
    edit(U+'main.c','!ChassisAuto_BeginSequence()) return 0U;',
         '!(SourceBodyStep_WheelWindow()?ChassisAuto_BeginHeldSequence():ChassisAuto_BeginSequence())) return 0U;')
    edit(U+'source_stage2_body_port.inc','{return Stage2Body_Owns() && StartupAxes_Holding() && !g_safety_fault && !g_rtos_fault && !g_can_ctrl_fault_latched;}',
         '{return SourceBodyStep_KeepsPower() || (Stage2Body_Owns() && StartupAxes_Holding() && !g_safety_fault && !g_rtos_fault && !g_can_ctrl_fault_latched);}')
    edit('CHASSIS/SYSTEM/SAFETY/safety.c','    if (g_chassis_auto_request == 0U &&',
         '    if (!SourceBodyStep_WheelWindow() && g_chassis_auto_request == 0U &&')
    edit(U+'chassis_auto_test.h','uint32_t ChassisAuto_BeginSequence(void);',
         'uint32_t ChassisAuto_BeginSequence(void);\r\nuint32_t ChassisAuto_BeginHeldSequence(void);')
    held=r'''
/* Explicit mechanism-held commissioning entry; original normal entry unchanged.
 * Real wheel guard, power and original backend preparation are still required.
 * No fault/result/status is cleared here. No CAN frame or Power_Disable call. */
uint32_t ChassisAuto_BeginHeldSequence(void)
{
 uint32_t accepted=0U;PowerCommandStatus power;
 taskENTER_CRITICAL();Power_ReadCommandStatus(&power);
 if(SourceBodyStep_WheelWindow() && StartupAxes_Holding() && power.active==1U &&
    !s_sequence_hold && !s_used && Auto_BaseHealthy() &&
    !g_chassis_auto_active && !g_chassis_auto_cut_vm && !g_chassis_auto_result &&
    !g_chassis_auto_cause && !g_can_ctrl_fault_latched &&
    g_chassis_auto_state==CHASSIS_AUTO_IDLE) {
  s_vm_requested=1U;s_sequence_hold=1U;s_sequence_continue=0U;accepted=1U;
 }
 taskEXIT_CRITICAL();return accepted;
}
'''
    edit(U+'chassis_auto_test.c','uint32_t ChassisAuto_BeginSequence(void)',held+'\r\nuint32_t ChassisAuto_BeginSequence(void)')
    main=P/U/'main.c';data=main.read_bytes()
    old=b'VEHICLE_STAGE1_BLEND_ENABLE && g_vehicle_stage==1U && (g_vehicle_scan_search_state!=1U && g_vehicle_scan_search_state!=2U)'
    assert data.count(old)==2
    data=data.replace(old,b'!SourceBodyStep_WheelWindow() && '+old)
    main.write_bytes(data)
    edit(U+'main.c','g_vm_test_request = (g_forward_hold.phase == HR_DONE) ? App_StageDoneVm() : 0U;',
         'g_vm_test_request = (g_forward_hold.phase == HR_DONE) ? App_StageDoneVm() : (SourceBodyStep_KeepsPower()?1U:0U);')
    edit(U+'main.c','      g_vm_test_request = 0U; taskEXIT_CRITICAL(); return; }',
         '      if(!SourceBodyStep_KeepsPower())g_vm_test_request = 0U; taskEXIT_CRITICAL(); return; }')
    edit(U+'integration10_complete.inc',' g_it_request=0U;\r\n /*',
         ' g_it_request=0U;\r\n if(SourceBodyStep_WheelWindow() && request){\r\n'
         '  if(request==9U)g_source_body_step_request=9U;\r\n'
         '  else {g_it_rejected=13U;request=0U;}\r\n }\r\n /*')
    changes={}
    for f in P.rglob('*'):
        if not f.is_file():continue
        rel=f.relative_to(P);old=B/rel
        if not old.exists() or old.read_bytes()!=f.read_bytes():changes[rel.as_posix()]=hashlib.sha256(f.read_bytes()).hexdigest()
    assert (P/U/'integration13_immediate.inc').read_bytes()==(B/U/'integration13_immediate.inc').read_bytes()
    (E/'candidate.json').write_text(json.dumps(dict(base=B.relative_to(R).as_posix(),changes=changes,hardware_access=False,
        flashed=False,step_direction='left',step_distance_mm=10,reference_speed_mm_s=15,repeat_automatically=False,
        mechanical_pzq50_scheduler_unchanged=True),indent=2)+'\n')
    print('Prepared isolated body-step candidate; pzq50 scheduler unchanged.',len(changes),'changed/added files')
if __name__=='__main__':main()
