"""Isolated offline source station: parked alignment -> manualQR -> pzq.
No flash/probe/serial/camera access. Preserve the accepted pzq snapshot.
"""
from pathlib import Path
import hashlib,json,shutil
R=Path(__file__).resolve().parents[1];B=R/'handoff/pzq/project'
PACKAGE=R/'tmp/source_station_offline_20261109';P=PACKAGE/'project';U=P/'CHASSIS/USER'
E=R/'evidence/integration/2026-11-09/source-station-offline'
def edit(name,old,new):
 p=U/name;b=p.read_bytes();a=old.encode();assert b.count(a)==1,(name,old[:90]);p.write_bytes(b.replace(a,new.encode(),1))
def main():
 assert json.loads((E/'matlab-result.json').read_text())['passed']
 assert not P.exists(),'Preserve candidate';shutil.copytree(B,P)
 for p in (R/'software_interfaces/source_station').iterdir():
  if p.is_file():shutil.copy2(p,U/p.name)
 edit('source_stage2_body_port.inc','    limits->body_frame_verified!=1U || limits->route3_profile_verified!=1U ||',
      '    limits->body_frame_verified!=1U || limits->route3_profile_verified>1U ||')
 edit('source_stage2_body_port.inc',' if(!s2bp.configured){taskEXIT_CRITICAL();return 2U;}',
 ' if(!s2bp.configured || (command->kind==SPF_CMD_ROUTE_AND_STOW && s2bp.limits.route3_profile_verified!=1U)){taskEXIT_CRITICAL();return 2U;}')
 edit('source_stage2_body_port.inc','{return Stage2Body_Owns() && StartupAxes_Holding()',
 '{return (Stage2Body_Owns() || SourceStation_HoldsBody()) && StartupAxes_Holding()')
 edit('source_stage2_body_port.inc','#include "source_observer.h"',
 '#include "source_observer.h"\n#include "source_station.h"')
 edit('source_stage2_body_port.h','void Stage2Body_Cancel(void);',
 'void Stage2Body_Cancel(void);\nuint32_t Stage2Body_BeginAlignment(uint32_t run,uint32_t sequence,uint32_t now);\n'
 'uint32_t Stage2Body_ReadAlignment(uint32_t run,uint32_t sequence,S2_Event *out);')
 p=U/'source_stage2_body_port.inc';p.write_bytes(p.read_bytes()+b'''\nuint32_t Stage2Body_BeginAlignment(uint32_t run,uint32_t sequence,uint32_t now)
{
 SPF_Command c;memset(&c,0,sizeof c);c.run_id=run;c.sequence=sequence;c.kind=SPF_CMD_ALIGN;
 return run && sequence && S2BP_Submit(&c,now)==1U;
}
uint32_t Stage2Body_ReadAlignment(uint32_t run,uint32_t sequence,S2_Event *out)
{
 SPF_Command c;memset(&c,0,sizeof c);c.run_id=run;c.sequence=sequence;c.kind=SPF_CMD_ALIGN;
 return S2BP_Poll(&c,out);
}
''')
 edit('vehicle_mission.inc','#include "source_stage2_body_port.inc"\r\n',
 '#include "source_stage2_body_port.inc"\r\n#include "source_station_mission.inc"\r\n')
 edit('vehicle_mission.inc','        if(now-s_vehicle_stage_start>=VEHICLE_STAGE_TIMEOUT_MS ||',
 '        if((!SourceStation_ManualWait() && now-s_vehicle_stage_start>=VEHICLE_STAGE_TIMEOUT_MS) ||')
 edit('vehicle_mission.inc','    if(Stage2Body_Service(now))return;',
 '    if(Stage2Body_Service(now)){(void)SourceStation_Service(now);return;}\r\n'
 '    if(SourceStation_Service(now))return;')
 edit('vehicle_mission.inc','        g_vehicle_scan_bypassed=VEHICLE_SCAN_BYPASS;s_vehicle_code_valid=s_vehicle_display_done=0U;',
 '        /* Temporary supervised flow: original-area alignment, THEN a real new scanner frame. */\r\n'
 '        g_vehicle_scan_bypassed=(SOURCE_STATION_ENABLE && SOURCE_STATION_MANUAL_QR_AFTER_ALIGNMENT)?1U:VEHICLE_SCAN_BYPASS;\r\n'
 '        s_vehicle_code_valid=s_vehicle_display_done=0U;')
 edit('vehicle_mission.inc','        SAOMA_EnableReceive();\r\n        s_vehicle_scan_baseline=',
 '        if(SOURCE_STATION_ENABLE && SOURCE_STATION_MANUAL_QR_AFTER_ALIGNMENT)SAOMA_DisableReceive();\r\n'
 '        else SAOMA_EnableReceive();\r\n        s_vehicle_scan_baseline=')
 edit('vehicle_mission.inc','    Stage2Body_Cancel();MotionStop();VehicleArm_Cancel();StartupAxes_Cancel();',
 '    SourceStation_Cancel(cause);Stage2Body_Cancel();MotionStop();VehicleArm_Cancel();StartupAxes_Cancel();')
 edit('vehicle_mission.inc',' uint32_t valid=0U;\r\n if(!code || !run || !batch)return 0U;\r\n taskENTER_CRITICAL();',
 '#if !SOURCE_STATION_ENABLE\r\n uint32_t valid=0U;\r\n#endif\r\n'
 ' if(!code || !run || !batch)return 0U;\r\n'
 '#if SOURCE_STATION_ENABLE\r\n return SourceStation_ReadTask(code,run,batch);\r\n#else\r\n taskENTER_CRITICAL();')
 edit('vehicle_mission.inc',' taskEXIT_CRITICAL();return valid;\r\n}\r\n\r\nuint32_t VehicleMission_ReadAlignmentRequest',
 ' taskEXIT_CRITICAL();return valid;\r\n#endif\r\n}\r\n\r\nuint32_t VehicleMission_ReadAlignmentRequest')
 # Only after arrival: build/validate real lift/extension references separately.
 # Do not send the legacy70mm source deployment on this new station candidate.
 edit('vehicle_mission.inc','#if VEHICLE_SOURCE_PREP_ENABLE\r\n                {',
 '#if SOURCE_STATION_ENABLE\r\n                g_vehicle_alignment_state=1U;g_vehicle_state=8U;return;\r\n'
 '#elif VEHICLE_SOURCE_PREP_ENABLE\r\n                {')
 edit('source_vision_runtime.h','#include <stdint.h>',
 '#include <stdint.h>\n#include "source_station.h"')
 edit('main.c','#include "source_stage2_body_port.h"',
 '#include "source_stage2_body_port.h"\r\n#include "source_station.h"\r\n#include "source_station_site.h"')
 edit('main.c','    if(!CAN1_EmmSettingsConfirmed())mask|=1U;',
 '    if(SourceStation_HoldsBody())mask|=512U; /* parked vision/scan/pick owns the station */\r\n'
 '    if(!CAN1_EmmSettingsConfirmed())mask|=1U;')
 edit('main.c','        if (status.active || status.result || status.cut_vm || (power.active && !StartupAxes_Holding()) ||',
 '        if (SourceStation_HoldsBody() || status.active || status.result || status.cut_vm || (power.active && !StartupAxes_Holding()) ||')
 edit('main.c','    { g_forward_hold.request = g_heading_exec_request = 0U;\r\n      g_vm_test_request = 0U; taskEXIT_CRITICAL(); return; }',
 '    { g_forward_hold.request = g_heading_exec_request = 0U;\r\n'
 '      if(!Stage2Body_KeepsPower())g_vm_test_request = 0U; taskEXIT_CRITICAL(); return; }')
 # Frozen pzq has a commissioning-only PC13 home switch. Restore mission
 # progression in THIS candidate; no start is issued by boot or preparation.
 q=P/'USER/vehicle_parameters.h';data=q.read_bytes();old=b'#define VEHICLE_EXTENSION_HOME_ONLY 1U';assert data.count(old)==1;q.write_bytes(data.replace(old,b'#define VEHICLE_EXTENSION_HOME_ONLY 0U',1))
 edit('startup_axes.c','#include "app_config.h"',
 '#include "app_config.h"\r\n#include "source_station_site.h"')
 edit('startup_axes.c','#elif VEHICLE_EXTENSION_HOME_ONLY',
 '#elif VEHICLE_EXTENSION_HOME_ONLY || SOURCE_STATION_ENABLE')
 can=P/'CHASSIS/SYSTEM/CAN/can.c';d=can.read_bytes()
 d=d.replace(b'#include "app_config.h"',b'#include "app_config.h"\r\n#include "source_station_site.h"',1)
 assert b'(VEHICLE_EXTENSION_HOME_ONLY || VEHICLE_STARTUP_GROUP_ENABLE)' in d
 d=d.replace(b'(VEHICLE_EXTENSION_HOME_ONLY || VEHICLE_STARTUP_GROUP_ENABLE)',
 b'(VEHICLE_EXTENSION_HOME_ONLY || VEHICLE_STARTUP_GROUP_ENABLE || SOURCE_STATION_ENABLE)');can.write_bytes(d)
 edit('integration13_immediate.inc','static uint32_t it_auto_begin,it_auto_phase_begin,it_auto_sequence;',
 'static uint32_t it_auto_begin,it_auto_phase_begin,it_auto_sequence;\n'
 'volatile uint32_t g_it_auto_vision_mode,g_it_auto_region;\n'
 'static uint32_t it_auto_take_vision(uint32_t now);\n'
 'static uint32_t it_auto_target_usable(uint32_t now);')
 edit('integration13_immediate.inc',' case 1U:\n',
 ' case 1U:\n'
 '  if(g_it_auto_vision_mode && !it_auto_target_usable(now)){\n'
 '   (void)Vision_RejectUnstarted(g_vis_task_run_id,g_vis_pending_seq);\n'
 '   g_it_auto_phase=7U;it_auto_phase_begin=now;g_vis_command_request=2U;return;\n'
 '  }\n')
 edit('integration13_immediate.inc',' static const float yaw[3]={180.0f,195.0f,165.0f};',
 ' static const float yaw[3]={180.0f,195.0f,165.0f};\n'
 ' if(g_it_auto_vision_mode){g_it_auto_phase=7U;it_auto_phase_begin=now;g_vis_command_request=2U;return;}')
 edit('integration13_immediate.inc',' if(g_it_auto_phase==6U){',
 ' if(g_it_auto_phase==7U){(void)it_auto_take_vision(now);return;}\n if(g_it_auto_phase==6U){')
 edit('integration13_immediate.inc','  if(g_it_auto_phase_ms<200U)return;',
 '  if(g_it_auto_phase_ms<200U)return;\n'
 '  if(g_it_auto_vision_mode && g_vis_pending_seq)return; /* await actual placement provider */')
 edit('integration13_immediate.inc','  g_it_extension_mm=g_it_auto_item==2U?160.0f:(g_it_auto_item==1U?156.0f:0.0f);g_it_lift_pulses=118800.0f;',
 '  g_it_extension_mm=g_it_auto_vision_mode?(g_it_auto_region==0U?156.0f:(g_it_auto_region==1U?160.0f:0.0f)):\n'
 '   (g_it_auto_item==2U?160.0f:(g_it_auto_item==1U?156.0f:0.0f));g_it_lift_pulses=118800.0f;')
 edit('integration10_complete.inc','#include "stage2_fast_cycle.h"',
 '#include "stage2_fast_cycle.h"\r\n#include "source_station.h"\r\n#include "source_station_site.h"')
 edit('integration10_complete.inc',' if(run!=g_vis_task_run_id || !g_vis_pending_seq || sequence!=g_vis_pending_seq || source_clear!=1U || held!=1U ||',
 ' if(g_it_auto_vision_mode && g_it_auto_phase!=3U && g_it_auto_phase!=6U)return 0U;\r\n'
 ' if(run!=g_vis_task_run_id || !g_vis_pending_seq || sequence!=g_vis_pending_seq || source_clear!=1U || held!=1U ||')
 edit('integration10_complete.inc',' if(run!=g_vis_task_run_id || !g_vis_pending_seq || sequence!=g_vis_pending_seq || slot!=g_vis_pending_slot ||',
 ' if(g_it_auto_vision_mode && g_it_auto_phase!=6U)return 0U;\r\n'
 ' if(run!=g_vis_task_run_id || !g_vis_pending_seq || sequence!=g_vis_pending_seq || slot!=g_vis_pending_slot ||')
 edit('integration10_complete.inc','#include "integration13_immediate.inc"',
 '#include "integration13_immediate.inc"\r\n#include "source_station_vision.inc"')
 edit('integration10_complete.inc','void Integration10_Service(void)\r\n',
 'uint32_t SourceStation_MechanicsReady(void)\r\n'
 '{return SingleTeach_ReferencesReady() && it_at_top() && it_dm_fresh() && !VehicleArm_Busy() &&\r\n'
 ' !g_lift_ruler_request && !g_lift_ruler_stop_pending && g_it_state!=1U && g_it_state!=9U;}\r\n'
 'void Integration10_Service(void)\r\n')
 edit('integration10_complete.inc',' Vision_Poll();\r\n g_it_request=0U;',
 ' Vision_Poll();\r\n g_it_request=0U;\r\n SourceStation_VisionService(now);')
 edit('integration10_complete.inc',' if(request==16U){\r\n  if(!it_auto_start(now))',
 ' if(request==16U){\r\n  if(SOURCE_STATION_ENABLE){g_it_rejected=13U;request=0U;}\r\n  else if(!it_auto_start(now))')
 report={'base':'handoff/pzq','source':'offline only','hardware_access':False,'calibration_enabled':False,
  'route3_enabled':False,'changed':{}}
 for p in P.rglob('*'):
  if p.is_file():
   old=B/p.relative_to(P)
   if not old.exists() or old.read_bytes()!=p.read_bytes():report['changed'][p.relative_to(P).as_posix()]=hashlib.sha256(p.read_bytes()).hexdigest()
 (E/'candidate.json').write_text(json.dumps(report,indent=2)+'\n');print('Prepared',P,flush=True)
if __name__=='__main__':main()
