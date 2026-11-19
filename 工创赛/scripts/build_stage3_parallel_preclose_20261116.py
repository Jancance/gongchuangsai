"""Real owner plus extracted real IT3 runtime, LRC native and dual Keil. No hardware."""
from pathlib import Path
import json,hashlib,subprocess,re
import build_stage3_teach_mode_20261116 as common
import build_stage3_offline_ready_20261116 as prior
import build_stage3_preclose160_20261116 as manual
from prepare_stage3_parallel_preclose_20261116 import R,BASE,P,E
def feature_metadata():
 baseline=json.loads((R/'evidence/integration/2026-11-16/stage3-offline-ready/candidate.json').read_text())
 keys=['stage3_lift_max','source_lift_max_unchanged','stage3_extension_rpm','teach_entry_request','preclose_request','preclose_angle','software_settle_ms','servo_angle_feedback','teach_feedback_mode','teach_measured_x','teach_geometry_only_kinds','dm_manual_teach_existing_requests','independent_task_inventory','physical_calibration_open']
 features={key:baseline[key] for key in keys}
 header=(P/'USER/vehicle_parameters.h').read_text(encoding='latin1')
 features['manual_teach_limits']={key:float(re.search(r'^#define\s+'+macro+r'\s+([\d.]+)f?\b',header,re.M).group(1)) for key,macro in [('top_deg','SOURCE_TOP_TOLERANCE_DEG'),('x_zero_deg','SOURCE_EXTENSION_ZERO_DONE_DEG')]}
 features.update(stage3_parallel_preclose=True,preclose_trigger='IT3 real started_mask SPF_DM; PWM-success current RTOS tick',stage3_return_top_margin_pulses=600,source_return_top_margin_pulses=1800,mechanical_zero_unchanged=True)
 return features
def owner():
 t=prior.owner_fixture()
 t=t.replace('#include "stage3_cycle.inc"','static uint32_t tick;\nstatic unsigned pwm_calls,pwm_ok=1,pwm_delay;\nstatic uint32_t xTaskGetTickCount(void){return tick;}\nstatic uint32_t jiazhua(float a){CHECK(a==160);pwm_calls++;tick+=pwm_delay;return pwm_ok;}\n#include "stage3_cycle.inc"')
 t=t.replace('static void service(void){RegionDispatch_Service(tick);}', 'static void service(void){RegionDispatch_Service(tick);}')
 runtime=(P/'CHASSIS/USER/integration10_complete.inc').read_text(encoding='utf-8')
 start=runtime.index(' case 3U:\n  if(it_phase==1U)');end=runtime.index(' case 15U:',start)
 case=runtime[start:end]
 support='''
#define APP_MOTOR_OFFLINE_ONLY 1
#include "source_parallel_flow.h"
static uint32_t it_phase,it_dm_cycle,g_source_parallel_state,g_source_parallel_started_mask,begin_ok=1;
static SPF_Command it_command;
static uint32_t VehicleArm_BeginSourceParallel(const SPF_Command*c,uint32_t a,uint32_t b){(void)c;(void)a;(void)b;return begin_ok;}
static void it_done(void){g_it_state=2;}
static void it_fail(uint32_t why){g_it_state=9;g_it_cause=why;}
static void actual_it3(uint32_t now){(void)now;switch(g_it_action){
'''+case+'''default:break;}}
'''
 t=t.replace('static void complete(void){',support+'\nstatic void complete(void){')
 t=t.replace('if(g_it_axes==SPF_DM){g_dm_can2_logical_position_deg=g_it_dm_deg;dm_moves++;}', '''if(g_it_axes==SPF_DM){
   unsigned before=pwm_calls;g_it_state=1;it_phase=2;it_command.axes_mask=SPF_DM;
   g_source_parallel_state=2;g_source_parallel_started_mask=0;actual_it3(tick);CHECK(pwm_calls==before);
   g_source_parallel_started_mask=SPF_DM;actual_it3(tick);actual_it3(tick+1);
   CHECK(pwm_calls==before+(g_stage3_phase==3));
   g_source_parallel_state=3;actual_it3(tick+100);CHECK(g_it_state==2);
   g_dm_can2_logical_position_deg=g_it_dm_deg;dm_moves++;
  }''')
 t=t.replace(' if(g_it_action==17){CHECK(g_stage3_phase==4 && g_vehicle_lift_current_deg==0 && g_dm_can2_logical_position_deg==36.5f);precloses++;}',
 ''' if(g_it_action==3 && g_it_axes==SPF_DM && g_stage3_phase==3){CHECK(s3_preclose_started);precloses++;}''')
 t=t.replace(' service();\n}\nstatic void frame', ' service();if(g_stage3_state==5 && g_stage3_phase==4){tick+=600;service();service();}\n}\nstatic void frame')
 # Separate deterministic delay/failure matrix uses the actual owner and IT3.
 pos=t.index('int main(void){')
 matrix='''static void parallel_matrix(void){
 const uint32_t durations[]={0,100,599,600,601,2000};
 for(unsigned w=0;w<2;w++)for(unsigned k=0;k<6;k++){
  reset();g_region_dispatch_area=3;g_stage3_state=5;g_stage3_phase=3;g_it_state=2;s3_pending=0;
  tick=w?0xffffff00U:1000;service();CHECK(g_it_request==3 && !s3_preclose_started);
  g_it_request=0;g_it_action=3;g_it_state=1;it_phase=2;it_command.axes_mask=SPF_DM;
  g_source_parallel_state=2;g_source_parallel_started_mask=0;unsigned before=pwm_calls;actual_it3(tick);
  CHECK(pwm_calls==before && !s3_preclose_started);
  g_source_parallel_started_mask=1;actual_it3(tick);actual_it3(tick+1);CHECK(pwm_calls==before+1);
  uint32_t start=tick;tick+=durations[k];g_source_parallel_state=3;actual_it3(tick);service();
  CHECK(g_stage3_phase==(durations[k]<600?4U:5U) && !g_it_request);
  if(durations[k]<600){tick=start+599;service();CHECK(g_stage3_phase==4);tick=start+600;service();}
  CHECK(g_stage3_phase==5 && !g_it_request);service();CHECK(g_it_request==3 && g_it_axes==4);
 }
 for(unsigned delay=1;delay<=250;delay+=83){
  reset();g_region_dispatch_area=3;g_stage3_state=5;g_stage3_phase=3;g_it_state=2;tick=1000;service();
  g_it_request=0;g_it_action=3;g_it_state=1;it_command.axes_mask=1;it_phase=3;
  g_source_parallel_state=3;g_source_parallel_started_mask=1;pwm_delay=delay;actual_it3(963);
  CHECK(s3_preclose_tick==1000+delay);pwm_delay=0;service();CHECK(g_stage3_phase==4);
  tick=1599+delay;service();CHECK(g_stage3_phase==4 && !g_it_request);
  tick=1600+delay;service();CHECK(g_stage3_phase==5 && !g_it_request);
 }
 for(unsigned scope=0;scope<2;scope++)for(unsigned phase=0;phase<9;phase++){
  reset();g_region_dispatch_area=scope?3:1;g_stage3_state=5;g_stage3_phase=phase;s3_pending=3;
  g_it_action=3;g_it_state=1;it_command.axes_mask=1;it_phase=3;
  g_source_parallel_state=2;g_source_parallel_started_mask=1;s3_preclose_started=0;
  unsigned before=pwm_calls;actual_it3(tick);CHECK(pwm_calls==before+(scope && phase==3));
 }
 for(unsigned f=0;f<4;f++){
  reset();g_region_dispatch_area=3;g_stage3_state=5;g_stage3_phase=3;g_it_state=2;service();
  g_it_request=0;g_it_action=3;g_it_state=1;it_command.axes_mask=1;it_phase=f==0?2:3;
  begin_ok=f!=0;pwm_ok=f!=1;g_source_parallel_state=f==2?4:2;g_source_parallel_started_mask=1;unsigned before=pwm_calls;
  if(f==3){g_stage3_request=9;service();CHECK(g_stage3_state==9);}
  else {actual_it3(tick);service();CHECK(g_stage3_state==9 && g_it_cause==(f==2?8:7));}
  CHECK(g_it_axes!=4);CHECK(pwm_calls==before+(f==1));begin_ok=pwm_ok=1;
 }
 puts("PASS actual IT3 + Stage3 matrix: max(DMdone,600), wrap, launch/PWM/DM/STOP failures");
}
'''
 t=t[:pos]+matrix+t[pos:]
 t=t.replace(' teach_tests();',' parallel_matrix();\n teach_tests();')
 return t
def main():
 proof=json.loads((E/'matlab-result.json').read_text());assert proof['passed']
 common.P=P;common.E=E;results={}
 path=E/'actual-parallel-owner-it3.c';path.write_text(owner());results['owner_it3']=common.run('actual-owner-it3',path,flags=['-DSPF_STAGE3_MAX_LIFT_PULSES=82000U'])
 manual.P=P;path=E/'actual-manual-it17.c';path.write_text(manual.dispatch_fixture());results['manual_it17']=common.run('actual-manual-it17',path)
 # Old deep scope checks now expect the deliberately selected 600p only for stage3.
 t=(R/'tests/stage3_deep_z_20261116.c').read_text().replace('(int64_t)1800*360*scale/3200','(int64_t)600*360*scale/3200')
 path=E/'actual-deep-scope.c';path.write_text(t);results['deep']=common.run('actual-deep-scope',path,[P/'CHASSIS/USER/source_parallel_flow.c',P/'CHASSIS/USER/lift_ruler_calibration.c'])
 results['return_chain']=common.run('actual-return-chain','tests/stage3_return600_20261116.c',[P/'CHASSIS/USER/lift_ruler_calibration.c'])
 t=(R/'tests/lift_home_exit_20261114.c').read_text().replace('(int64_t)32*360*scale/3200','(int64_t)80*360*scale/3200').replace('bounded32p','bounded80p')
 path=E/'actual-home-exit.c';path.write_text(t);results['home_exit']=common.run('actual-home-exit',path,[P/'CHASSIS/USER/lift_ruler_calibration.c'])
 results['source_group']=common.run('actual-source-group','tests/simplified_parallel_group_20261114.c',[P/'CHASSIS/USER/source_parallel_flow.c'])
 changes={};unchanged={}
 for path in P.rglob('*'):
  if not path.is_file() or any(x in ('Objects','Listings') for x in path.relative_to(P).parts):continue
  rel=path.relative_to(P).as_posix()
  if (BASE/rel).read_bytes()!=path.read_bytes():changes[rel]=hashlib.sha256(path.read_bytes()).hexdigest()
 assert set(changes)=={'CHASSIS/USER/stage3_cycle.inc','CHASSIS/USER/integration10_complete.inc','CHASSIS/USER/lift_ruler_calibration.c'}
 for name in ['USER/vehicle_parameters.h','CHASSIS/USER/source_mcu_cycle.inc','CHASSIS/USER/vehicle_source_parallel.inc','CHASSIS/USER/source_parallel_flow.c','CHASSIS/USER/source_parallel_flow.h','CHASSIS/USER/lift_ruler_adapter.c','CHASSIS/USER/motion_control.h','HARDWARE/DM_CAN2/dm_can2_motion.c','HARDWARE/DM_CAN2/dm_can2.c']:
  assert (P/name).read_bytes()==(BASE/name).read_bytes(),name
  unchanged[name]=hashlib.sha256((P/name).read_bytes()).hexdigest()
 adapter=(P/'CHASSIS/USER/lift_ruler_adapter.c').read_bytes()
 assert b'LiftRuler_StartReturnWithLimit(&g_lift_ruler_controller' in adapter and b'Stage3_Owns()' in adapter
 images={}
 for target in ['Vehicle_LiftRuler','Vehicle_LiftRuler_Offline']:
  for folder in ['Objects','Listings']:(P/folder/target).mkdir(parents=True,exist_ok=True)
  log=E/(target+'-build.log')
  r=subprocess.run(['C:/Keil_v5/UV4/UV4.exe','-r',str(P/'gongxun-f407.uvprojx'),'-t',target,'-j0','-o',str(log)],timeout=240)
  data=log.read_bytes();print(data.decode('gb18030',errors='replace')[-650:],flush=True)
  assert r.returncode==0 and b'0 Error(s), 0 Warning(s)' in data
  axf=P/'Objects'/target/(target+'.axf');images[target]={'axf':str(axf),'sha256':hashlib.sha256(axf.read_bytes()).hexdigest()}
 (E/'candidate.json').write_text(json.dumps(dict(base=str(BASE),project=str(P),changes=changes,unchanged_sha256=unchanged,images=images,matlab_latest=proof,matlab_control_models_before_corresponding_implementation=True,matlab_timeline={'initial_837_before_three_source_deltas':'2026-11-16 21:37:12','supplement_961_before_real_write_tick_delta':'2026-11-16 21:42:04','initial_result_evidence':'session tool stdout captured before prepare script ran; latest JSON is supplemental run'},build_passed=True,flashed=False,hardware_access=False,requirements=['SRC-01','SRC-02','RUN-01','RUN-02','PWR-01','MECH-01','MECH-02'],rule_class='E; R/Q unchanged'),indent=2)+'\n')
 candidate_path=E/'candidate.json';candidate=json.loads(candidate_path.read_text());candidate.update(feature_metadata());candidate_path.write_text(json.dumps(candidate,indent=2)+'\n')
 (E/'checks.json').write_text(json.dumps(dict(passed=True,actual_c=results,dual_build=True,hardware_access=False,flashed=False),indent=2)+'\n')
if __name__=='__main__':main()
