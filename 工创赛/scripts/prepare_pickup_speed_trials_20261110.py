"""Prepare paired speed trials and optional lift S-reference, no hardware."""
from pathlib import Path
import hashlib,json,shutil
R=Path(__file__).resolve().parents[1];B=R/'handoff/pzq/project'
PACK=R/'tmp/pickup_speed_trials_20261110';P=PACK/'project';U=P/'CHASSIS/USER'
E=R/'evidence/integration/2026-11-10/speed-trials'
def replace(data,old,new):
 a=old.encode();assert data.count(a)==1,old[:100];return data.replace(a,new.encode(),1)
def edit(name,old,new):
 p=U/name;p.write_bytes(replace(p.read_bytes(),old,new))
def main():
 assert json.loads((E/'timing-model.json').read_text())['passed']
 assert not P.exists(),'Preserve candidates';shutil.copytree(B,P)
 for p in (R/'software_interfaces/pickup_speed_trials').iterdir():
  if p.is_file():shutil.copy2(p,U/p.name)
 edit('vehicle_arm.c','#include "app_config.h"',
 '#include "app_config.h"\r\n#include "pickup_trial_profile.h"\r\n'
 'volatile uint32_t g_pick_trial_profile=0U,g_pick_trial_requested=0U,g_pick_trial_scurve=0U;\r\n'
 'volatile uint32_t g_pick_trial_scurve_requested=0U;')
 edit('source_parallel_flow.h','#define SPF_EXT_RPM 106.0f',
 '#include "pickup_trial_profile.h"\r\n#define SPF_EXT_RPM PickupTrial_ExtRpm()')
 edit('source_parallel_flow.h','#define SPF_LIFT_RPM 1846.4f','#define SPF_LIFT_RPM PickupTrial_LiftRpm()')
 edit('source_parallel_flow.h','#define SPF_STEPPER_ACCEL 600U','#define SPF_STEPPER_ACCEL PickupTrial_ExtAccel()')
 edit('source_parallel_flow.c',' if(!g || !c || !io || (g->state && g->state!=3) || verified!=1 ||',
 ' if(!g || !c || !io || g_pick_trial_profile>3U || (g->state && g->state!=3) || verified!=1 ||')
 edit('integration10_complete.inc','it_command.extension_rpm=g_it_ext_rpm_test?(float)g_it_ext_rpm_test:106.0f;it_command.lift_rpm=1846.4f;',
 'it_command.extension_rpm=SPF_EXT_RPM;it_command.lift_rpm=SPF_LIFT_RPM;')
 edit('integration10_complete.inc','it_command.stepper_accel_rpm_s=600;it_command.keep_vm_on=1U;',
 'it_command.stepper_accel_rpm_s=SPF_STEPPER_ACCEL;it_command.keep_vm_on=1U;')
 edit('integration10_complete.inc',' if(request==16U){',
 ''' if(request==17U){
  if(g_it_auto_state==1U || g_it_state==1U || g_it_state==9U || !it_platform() ||
   VehicleArm_Busy() || g_pick_trial_requested>3U || g_pick_trial_scurve_requested>1U ||
   !SingleTeach_ReferencesReady() || !it_at_top())g_it_rejected=14U;
  else {
   g_pick_trial_profile=g_pick_trial_requested;
   g_pick_trial_scurve=g_pick_trial_profile?g_pick_trial_scurve_requested:0U;
   g_it_action=17U;g_it_rejected=0U;g_it_state=2U;
  }
  request=0U;
 }
 if(request==16U){''')
 edit('vehicle_source_parallel.inc','static uint32_t VSP_Prepare(void *context,uint32_t bit,const SPF_Command *c)',
 '#include "pickup_s_curve_adapter.inc"\r\nstatic uint32_t VSP_Prepare(void *context,uint32_t bit,const SPF_Command *c)')
 edit('vehicle_source_parallel.inc','  a->options=s_rx.Data[2];',
 '  a->options=s_rx.Data[2];\n'
 '  if(g_pick_trial_profile && address==6U && !(a->options&0x80U))return 2; /* preserve taught input scale */')
 edit('vehicle_source_parallel.inc',' if(fabsf(a->goal-a->before)<=.2f)return 1;',
 ' if(fabsf(a->goal-a->before)<=.2f)return 1;\r\n'
 ' if(address==5U && g_pick_trial_scurve)return VSP_SCurveStart((uint32_t)xTaskGetTickCount());')
 edit('vehicle_source_parallel.inc',' SPF_GroupService(&vsp_group);g_source_parallel_state=vsp_group.state;',
 ' if(!VSP_SCurveService())SPF_GroupCancel(&vsp_group,9U);\r\n'
 ' SPF_GroupService(&vsp_group);g_source_parallel_state=vsp_group.state;')
 edit('vehicle_source_parallel.inc',' memset(vsp_axis,0,sizeof(vsp_axis));vsp_dm_posted=0;vsp_fast_lift=0U;vsp_owned=1;',
 ' memset(vsp_axis,0,sizeof(vsp_axis));vsp_dm_posted=0;vsp_fast_lift=0U;vsp_owned=1;\r\n'
 ' vsp_scurve_active=vsp_scurve_issued=0U;')
 edit('vehicle_source_parallel.inc',' if(n==5U && q[0]==0xf3',
 ' if(address==5U && n==11U && q[0]==0xfbU && g_pick_trial_scurve && vsp_scurve_active &&\r\n'
 '  vsp_group.state<4U && (vsp_group.started&SPF_LIFT) && !memcmp(q,vsp_scurve_frame,11U))return 1U;\r\n'
 ' if(n==5U && q[0]==0xf3')
 can=P/'CHASSIS/SYSTEM/CAN/can.c';data=can.read_bytes()
 data=replace(data,'#include "app_config.h"','#include "app_config.h"\r\n#include "pickup_trial_profile.h"')
 data=replace(data,'speed>(address==5U?18464U:1060U)',
 'speed>(uint32_t)((address==5U?PickupTrial_LiftRpm():PickupTrial_ExtRpm())*10.0f+.5f)')
 # FB is permitted only for the exact current lift reference by the existing
 # LiftRuler_DebugCanAllowed -> VSP whitelist, before entering this sender.
 data=replace(data,'    if(length==11U && q[0]==0xfdU',
 '''    if(address==5U && length==11U && q[0]==0xfbU && q[1]<=1U &&
       q[8]==1U && q[9]==0U && q[10]==0x6bU && reply_length==3U){
        uint32_t speed=((uint32_t)q[2]<<8)|q[3];
        if(!g_pick_trial_scurve || !speed || speed>(uint32_t)(PickupTrial_LiftRpm()*10.0f+.5f))return 7U;
    }
    else if(length==11U && q[0]==0xfdU''')
 data=replace(data,'rx.Data[0]==0xfd)', 'rx.Data[0]==q[0])')
 can.write_bytes(data)
 report={'base':'handoff/pzq','hardware_access':False,'profiles':[
  {'index':1,'lift_rpm':2100,'extension_rpm':200,'extension_accel_rpm_s':600},
  {'index':2,'lift_rpm':2500,'extension_rpm':300,'extension_accel_rpm_s':1200},
  {'index':3,'lift_rpm':2900,'extension_rpm':400,'extension_accel_rpm_s':2000}],
  'lift_accel_rpm_s':45055,'extension_accel_rpm_s':[600,1200,2000],'s_curve_reference_optional':True,
  's_curve_plant_tracking_verified':False,'actual_tests_run':0,'changed':{}}
 for p in P.rglob('*'):
  if p.is_file():
   old=B/p.relative_to(P)
   if not old.exists() or old.read_bytes()!=p.read_bytes():report['changed'][p.relative_to(P).as_posix()]=hashlib.sha256(p.read_bytes()).hexdigest()
 (E/'candidate.json').write_text(json.dumps(report,indent=2)+'\n')
 # Reuse the previously exercised bounded host runner, adapting only identity,
 # trial selection and per-sample speed/reference logging. Still default CHECK.
 src=(R/'scripts/integration13_empty_overlap_live_20261109.py').read_text(encoding='utf-8')
 src=src.replace("E = R / 'evidence/integration/2026-11-07/stage2-mainline'", "E = R / 'evidence/integration/2026-11-10/speed-trials'")
 src=src.replace('stage2-fast-candidate-20261109.json','candidate.json').replace('tmp/stage2_sub20_20261109','tmp/pickup_speed_trials_20261110')
 src=src.replace("p = R / 'tmp/pickup_speed_trials_20261110/project/CHASSIS/USER' / name", "p = R / 'tmp/pickup_speed_trials_20261110/project' / name")
 src=src.replace("                if immediate:\n", "                if immediate:\n                    cmd('select_trial',17,(('g_pick_trial_requested',TRIAL_INDEX),('g_pick_trial_scurve_requested',S_CURVE)))\n")
 src=src.replace("'g_vsp_prepare_wait_address') if k in symbols]", "'g_vsp_prepare_wait_address','g_pick_trial_profile','g_pick_trial_scurve','g_pick_scurve_frames','g_pick_scurve_last_gap_ms','g_pick_scurve_max_gap_ms') if k in symbols]")
 src=src.replace("    args=parser.parse_args()", "    parser.add_argument('--trial',type=int,choices=(1,2,3),default=1)\n    parser.add_argument('--s-curve',action='store_true')\n    args=parser.parse_args()\n    TRIAL_INDEX=args.trial\n    S_CURVE=1 if args.s_curve else 0")
 (R/'scripts/pickup_speed_trials_live_20261110.py').write_text(src,encoding='utf-8')
 shutil.copy2(R/'evidence/integration/2026-11-07/stage2-mainline/integration13-three-direct-matlab-20261109.json',E/'integration13-three-direct-matlab-20261109.json')
 print('Prepared three paired speed profiles andoptional S-shaped reference candidate; no device access.')
if __name__=='__main__':main()
