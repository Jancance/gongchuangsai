"""Offline C regressions and dual Keil build for minimal teach/task candidate."""
from pathlib import Path
import hashlib,json,subprocess
import build_stage3_teach_mode_20261116 as common
import build_stage3_preclose160_20261116 as preclose
from prepare_stage3_offline_ready_20261116 import R,BASE,P,E

def owner_fixture():
 text=(R/'tests/stage3_teach_mode_20261116.c').read_text()
 text=text.replace('g_vehicle_lift_feedback_valid=1,g_startup_extension_feedback_valid=1;', 'g_vehicle_lift_feedback_valid=1;')
 text=text.replace('volatile uint8_t g_lift_diag_raw[6][8];','volatile uint32_t g_lift_diag_raw[6][8];')
 text=text.replace('static uint32_t g_dm_can2_read_cycle,g_dm_can2_read_request;',
  'static uint32_t g_dm_can2_read_cycle,g_dm_can2_read_request,g_dm_can2_feedback_tick,g_dm_posvel_holding,g_dm_can2_state;')
 text=text.replace('g_stage3_carrier_yaw=36.5f;',
  'g_stage3_carrier_yaw=36.5f;s3_teach_pending=0;g_stage3_teach_result=0;g_stage3_teach_measured_x=-1;g_dm_posvel_holding=0;for(unsigned k=0;k<3;k++)g_stage3_slot_colors[k]=(unsigned[]){2,1,5}[k];')
 text=text.replace('static unsigned motions,commands,grips,releases,slots,dm_moves,x_moves,z_moves;',
  'static unsigned motions,commands,grips,releases,slots,dm_moves,x_moves,z_moves,precloses;')
 text=text.replace(' if(g_it_action==5)grips++;',
  ' if(g_it_action==17){CHECK(g_stage3_phase==4 && g_vehicle_lift_current_deg==0 && g_dm_can2_logical_position_deg==36.5f);precloses++;}\n if(g_it_action==5){CHECK(precloses==grips+1);grips++;}')
 text=text.replace('commands=motions=grips=releases=slots=dm_moves=x_moves=z_moves=0;',
  'commands=motions=grips=releases=slots=dm_moves=x_moves=z_moves=precloses=0;')
 text=text.replace('g_stage3_colors[c]=perm[b][c];','g_stage3_slot_colors[c]=perm[b][c];g_stage3_colors[c]=perm[b][c];')
 start=text.index(' /* Capture waits for new diagnostic cycles,');end=text.index(' g_stage3_state=6;g_stage3_request=1;',start)
 text=text[:start]+''' /* Pick/center do not request DM or generic diagnostics. */
 g_stage3_state=7;g_it_state=2;g_it_request=0;g_lift_ruler_controller.top_scale=100;
 g_lift_ruler_controller.top_position_raw=342;g_vehicle_lift_current_deg=903.8099975585938f;
 g_vehicle_lift_feedback_valid=1;g_stage3_teach_request=3;service();
 CHECK(g_stage3_teach_result==1 && g_stage3_target_yaw[1]==180 && g_stage3_target_x[1]==0);
 CHECK(fabsf(g_stage3_target_depth[1]-8003.4668f)<.002f && !g_lift_diag_request && !g_recovery_diag_request);
 CHECK(g_stage3_teach_sequence==g_stage3_teach_saved_sequence);
 g_stage3_teach_request=2;service();CHECK(g_stage3_teach_result==3 && g_stage3_teach_missing==4);
 g_stage3_teach_measured_x=25;g_dm_posvel_holding=1;g_dm_can2_feedback_tick=10;
 g_stage3_teach_request=2;service();CHECK(g_stage3_teach_result==2 && g_stage3_teach_missing==8 && !g_dm_can2_read_request);
 uint32_t seq=g_stage3_teach_sequence;g_stage3_teach_request=4;service();CHECK(g_stage3_teach_sequence==seq && s3_teach_pending==2);
 g_dm_can2_feedback_tick++;g_dm_can2_logical_position_deg=151;service();
 CHECK(g_stage3_teach_result==1 && g_stage3_target_yaw[0]==151 && g_stage3_target_x[0]==25 && g_stage3_teach_measured_x==-1);
 CHECK(g_stage3_teach_dm_source==1 && g_stage3_teach_saved_sequence==seq);
 g_stage3_teach_measured_x=35;g_dm_posvel_holding=0;g_stage3_teach_request=4;service();
 CHECK(g_stage3_teach_result==2 && g_dm_can2_read_request==2);
 g_dm_can2_read_request=0;g_dm_can2_read_cycle++;g_dm_can2_state=4;g_dm_can2_feedback_tick++;g_dm_can2_logical_position_deg=209;service();
 CHECK(g_stage3_teach_result==1 && g_stage3_target_yaw[2]==209 && g_stage3_target_x[2]==35 && g_stage3_teach_dm_source==2);
 g_dm_posvel_holding=1;g_stage3_teach_request=5;service();CHECK(g_stage3_teach_result==2);
 g_stage3_request=9;g_dm_can2_feedback_tick++;service();CHECK(g_stage3_state==9 && g_stage3_teach_result==3 && !s3_teach_pending);
 g_stage3_state=7;g_stage3_fault=0;g_it_state=2;g_it_cause=0;g_vehicle_lift_feedback_valid=0;g_stage3_teach_request=1;service();
 CHECK(g_stage3_teach_result==3 && g_stage3_teach_missing==2);g_vehicle_lift_feedback_valid=1;
 g_stage3_teach_request=1;service();CHECK(g_stage3_teach_result==1 && fabsf(g_stage3_pick_depth-8003.4668f)<.002f);
 for(unsigned kind=6;kind<=7;kind++)for(unsigned known=0;known<2;known++){
  unsigned idx=kind==6?0:2;float olddepth=known?62000:-1;
  g_stage3_target_depth[idx]=olddepth;g_vehicle_lift_feedback_valid=0;g_dm_posvel_holding=1;
  g_stage3_teach_measured_x=26;g_stage3_teach_request=kind;service();CHECK(g_stage3_teach_result==2);
  g_dm_can2_feedback_tick++;g_dm_can2_logical_position_deg=kind==6?151:209;service();
  CHECK(g_stage3_teach_result==1 && g_stage3_target_depth[idx]==olddepth && g_stage3_target_x[idx]==26 && !g_stage3_teach_z_source);
 }
 g_vehicle_lift_feedback_valid=1;
''' +text[end:]
 # Additional 216 task x destination x inventory permutations, actual phase0.
 pos=text.index(' teach_tests();')
 text=text[:pos]+''' const unsigned tc[6][3]={{2,1,5},{2,5,1},{1,2,5},{1,5,2},{5,2,1},{5,1,2}};
 const unsigned rp[6][3]={{1,2,3},{1,3,2},{2,1,3},{2,3,1},{3,1,2},{3,2,1}};
 for(unsigned ta=0;ta<6;ta++)for(unsigned tr=0;tr<6;tr++)for(unsigned inv=0;inv<6;inv++){
  reset();Stage3_Task t={77,1,{0},{0}};for(unsigned k=0;k<3;k++){t.colors[k]=tc[ta][k];t.positions[k]=rp[tr][k];}
  CHECK(Stage3_BindMission(&t,tc[inv]));CHECK(g_stage3_task_run==77 && g_stage3_task_batch==1);
  g_stage3_pick_depth=8003;for(unsigned k=0;k<3;k++){g_stage3_target_yaw[k]=k==1?180:150+30*k;g_stage3_target_x[k]=k==1?0:20;g_stage3_target_depth[k]=62000;}
  CHECK(S3_Config());S3_LockConfig();for(unsigned k=0;k<3;k++)g_stage3_slot_colors[k]=6;
  for(unsigned k=0;k<3;k++){
   g_stage3_item=k;g_stage3_phase=0;s3_pending=0;g_it_request=0;g_it_state=2;g_region_dispatch_area=3;service();
   unsigned si=0;while(tc[inv][si]!=tc[ta][k])si++;
   CHECK(g_it_request==10 && g_it_carrier_deg==si*120 && s3_positions[k]==rp[tr][k]);
  }
 }
''' +text[pos:]
 return text.replace('36 task mappings, STOP/exclusion','36 legacy mappings plus216 independent task/ring/inventory mappings, teach/STOP/exclusion')

def main():
 proof=json.loads((E/'matlab-result.json').read_text());assert proof['passed']
 common.P=P;common.E=E;results={}
 path=E/'actual-offline-owner.c';path.write_text(owner_fixture())
 results['owner_teach_task']=common.run('actual-offline-owner',path,flags=['-DSPF_STAGE3_MAX_LIFT_PULSES=82000U'])
 preclose.P=P;path=E/'actual-preclose-it.c';path.write_text(preclose.dispatch_fixture())
 results['preclose']=common.run('actual-preclose-it',path)
 results['deep']=common.run('actual-deep-z','tests/stage3_deep_z_20261116.c',[P/'CHASSIS/USER/source_parallel_flow.c',P/'CHASSIS/USER/lift_ruler_calibration.c'])
 lrc=(R/'tests/stage3_deep_z_owner_lrc_20261116.c').read_text().replace('{0,1,6,11}','{0,1,7,12}')
 lrc=lrc.replace('volatile uint8_t g_lift_diag_raw[6][8];','volatile uint32_t g_lift_diag_raw[6][8];').replace('static uint32_t g_dm_can2_read_cycle,g_dm_can2_read_request;', 'static uint32_t g_dm_can2_read_cycle,g_dm_can2_read_request,g_dm_can2_feedback_tick,g_dm_posvel_holding,g_dm_can2_state;')
 path=E/'actual-owner-lrc.c';path.write_text(lrc)
 results['owner_lrc']=common.run('actual-owner-lrc',path,[P/'CHASSIS/USER/lift_ruler_calibration.c'],flags=['-Wno-unused-variable'])
 fixture=(R/'tests/lift_home_exit_20261114.c').read_text().replace('(int64_t)32*360*scale/3200','(int64_t)80*360*scale/3200').replace('bounded32p','bounded80p')
 path=E/'home-exit-current.c';path.write_text(fixture)
 results['home']=common.run('actual-home-exit',path,[P/'CHASSIS/USER/lift_ruler_calibration.c'])
 results['source_group']=common.run('actual-source-group','tests/simplified_parallel_group_20261114.c',[P/'CHASSIS/USER/source_parallel_flow.c'])
 runtime=(P/'CHASSIS/USER/integration10_complete.inc').read_text(encoding='utf-8')
 start=runtime.index('uint32_t Vision_ReadLoadedBatch(');end=runtime.index('static uint8_t vis_buf[10];',start)
 fixture='''#include <stdint.h>
#include <string.h>
#include <stdio.h>
#include <stdlib.h>
static uint32_t g_vis_task_valid=1,g_vis_task_run_id=77,g_vis_task_batch=1,g_vis_batch_complete=1,g_vis_loaded_mask=7,g_vis_task_item=3,g_vis_pending_seq;
static uint32_t g_vis_task_colors[3]={2,1,5};static char vis_task_code[16]="215-312-512-231";
#define CHECK(x) do{if(!(x))exit(1);}while(0)
'''+runtime[start:end]+'''int main(void){uint32_t c[3],p[3],s[3];
 CHECK(Vision_ReadLoadedTask(77,1,c,p,s));CHECK(p[0]==3 && p[1]==1 && p[2]==2 && c[0]==2 && s[2]==5);
 CHECK(!Vision_ReadLoadedTask(78,1,c,p,s));CHECK(!Vision_ReadLoadedTask(77,2,c,p,s));
 for(unsigned mask=0;mask<7;mask++){g_vis_loaded_mask=mask;CHECK(!Vision_ReadLoadedTask(77,1,c,p,s));}g_vis_loaded_mask=7;
 g_vis_pending_seq=1;CHECK(!Vision_ReadLoadedTask(77,1,c,p,s));g_vis_pending_seq=0;
 g_vis_task_batch=2;CHECK(Vision_ReadLoadedTask(77,2,c,p,s));CHECK(p[0]==2 && p[1]==3 && p[2]==1);
 CHECK(!Vision_ReadLoadedTask(77,2,0,p,s));puts("PASS actual readonly loaded task getter:14 checks; strict physical commit gate unchanged");}
'''
 path=E/'actual-loaded-task.c';path.write_text(fixture);results['loaded_task']=common.run('actual-loaded-task',path)
 changes={};unchanged={}
 for path in P.rglob('*'):
  if not path.is_file() or any(x in ('Objects','Listings') for x in path.relative_to(P).parts):continue
  rel=path.relative_to(P).as_posix()
  if not (BASE/rel).exists() or (BASE/rel).read_bytes()!=path.read_bytes():changes[rel]=hashlib.sha256(path.read_bytes()).hexdigest()
 for name in ['USER/vehicle_parameters.h','CHASSIS/USER/source_mcu_cycle.inc','CHASSIS/USER/vehicle_source_parallel.inc','CHASSIS/USER/source_parallel_flow.c','CHASSIS/USER/source_parallel_flow.h','CHASSIS/USER/lift_ruler_adapter.c','CHASSIS/USER/motion_control.h','HARDWARE/DM_CAN2/dm_can2_motion.c','HARDWARE/DM_CAN2/dm_can2.c']:
  assert (P/name).read_bytes()==(BASE/name).read_bytes(),name
  unchanged[name]=hashlib.sha256((P/name).read_bytes()).hexdigest()
 images={}
 for target in ['Vehicle_LiftRuler','Vehicle_LiftRuler_Offline']:
  for folder in ['Objects','Listings']:(P/folder/target).mkdir(parents=True,exist_ok=True)
  log=E/(target+'-build.log')
  r=subprocess.run(['C:/Keil_v5/UV4/UV4.exe','-r',str(P/'gongxun-f407.uvprojx'),'-t',target,'-j0','-o',str(log)],timeout=180)
  data=log.read_bytes();print(data.decode('gb18030',errors='replace')[-600:],flush=True)
  assert r.returncode==0 and b'0 Error(s), 0 Warning(s)' in data
  axf=P/'Objects'/target/(target+'.axf');images[target]={'axf':axf.relative_to(R).as_posix(),'sha256':hashlib.sha256(axf.read_bytes()).hexdigest()}
 c=dict(base=BASE.relative_to(R).as_posix(),project=P.relative_to(R).as_posix(),changes=changes,unchanged_sha256=unchanged,images=images,build_passed=True,flashed=False,hardware_access=False,matlab_prior_to_implementation=True,matlab_when=proof['when'],physical_calibration_open=True,stage3_lift_max=82000,source_lift_max_unchanged=29700,stage3_extension_rpm=106,teach_entry_request=30,preclose_request=17,preclose_angle=160,software_settle_ms=600,servo_angle_feedback=False,teach_feedback_mode='measured-axis-v1',teach_measured_x=True,teach_geometry_only_kinds=[6,7],dm_manual_teach_existing_requests=True,independent_task_inventory=True)
 c['matlab_prior_to_implementation']=False
 c['matlab_control_scenarios_prior_to_corresponding_implementation']=True
 c['matlab_timeline']={
  'teach_dependencies_and_CAN_ownership_before_teach_code':'2026-11-16 15:51:24',
  'task_inventory_216_before_task_code':'2026-11-16 15:54:37',
  'sequence_publication_post_firmware_code_before_host_code':'2026-11-16 15:59:44',
  'geometry_only_6_7_before_geometry_code':'2026-11-16 16:02:19',
  'note':'The pure software sequence/saved-sequence model followed firmware publication code; no blanket claim that all final eight models preceded all implementation.'}
 (E/'candidate.json').write_text(json.dumps(c,indent=2)+'\n')
 (E/'checks.json').write_text(json.dumps(dict(passed=True,actual_c=results,dual_build=True,matlab=proof,hardware_access=False,flashed=False),indent=2)+'\n')
if __name__=='__main__':main()
