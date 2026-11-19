"""Actual complete IT service, VSP, DM source and native SPF; simulated hardware."""
from pathlib import Path
import json,shutil,subprocess,re
from prepare_three_frame_initial5_20261118 import R,P,E,BASE
from check_stage2_extension_mirror_20261117 import function
import check_stage2_sync_opt_20261117 as inherited
import check_three_frame_warm_c_20261118 as warm

def run(name,s,extra=()):
 if name=='actual-joint-vsp-dm':s=s.replace('v->current=a==6?-2000:100;','v->current=a==6?2000:100;')
 p=E/(name+'.c');p.write_text(s,encoding='utf-8');exe=p.with_suffix('.exe')
 args=['gcc','-std=c99','-O1','-Wall','-Wextra','-Wno-unused-function','-Wno-unused-variable','-Wno-unused-parameter','-Wno-misleading-indentation','-I'+str(E),'-I'+str(P/'CHASSIS/USER'),'-I'+str(P/'USER'),str(p),*[str(x) for x in extra],'-lm','-o',str(exe)]
 b=subprocess.run(args,capture_output=True,text=True);(E/(name+'-build.log')).write_text(b.stdout+b.stderr);assert not b.returncode,b.stderr
 z=subprocess.run([str(exe)],capture_output=True,text=True);(E/(name+'.log')).write_text(z.stdout+z.stderr);assert not z.returncode,z.stdout+z.stderr
 print(z.stdout.strip(),flush=True);return z.stdout.strip()

def main():
 assert json.loads((E/'matlab-result.json').read_text())['passed']
 results={}
 # Inherited three-piece timings, heights, beep and all-regions regressions.
 warm.P=P;warm.E=E
 def warmrun(n,s,extra=()):
  if n=='actual-queued-initial-DM-X':s=s.replace('g_dm_stage2_sync;','g_dm_stage2_sync,g_dm_initial_observe;')
  return run(n,s,extra)
 warm.run=warmrun;warm.main()
 inherited.P=P;inherited.E=E;inherited.run=run;inherited.dm()
 s=(E/'actual-joint-vsp-dm.c').read_text(encoding='utf-8')
 s=s[:s.rfind('int main(void)')]
 # Replace only fake-hardware adapters; all DM/VSP/IT production code stays real.
 a=function(s,'static void DM_CAN2_Service(')
 s=s.replace(a,r'''static void DM_CAN2_Service(unsigned a,unsigned b){(void)a;(void)b;
 if(!g_dm_can2_read_request)return;g_dm_can2_read_request=0;g_dm_can2_read_cycle++;
 g_dm_can2_state=4;g_dm_can2_result=1;g_dm_can2_valid_mask=g_dm_can2_range_mask=7;
 g_dm_can2_feedback_valid=1;g_dm_can2_feedback_tick=tick;g_dm_can2_motor_state=motor;
 g_dm_can2_esc_id=1;g_dm_can2_master_id=0;g_dm_can2_ctrl_mode=2;
 g_dm_can2_position_rad=feedback;g_dm_can2_position_deg=feedback*57.2957795f;
 }''')
 s=s.replace('if(t->Data[0]==253){stops++;motor=0;}','if(t->Data[0]==252)motor=1;if(t->Data[0]==253){stops++;motor=0;}')
 s=s.replace('CHECK(cached==6 && !g_dm_parallel_group_go);','CHECK(cached==(vsp_group.command.axes_mask&6) && !g_dm_parallel_group_go);')
 s=s.replace('CHECK(q[13]==1 && !g_dm_parallel_group_go);','CHECK(q[13]==1 && !g_dm_parallel_group_go);')
 s=s.replace('if(c==0x3a)s_rx.Data[1]=1;','if(c==0x3a)s_rx.Data[1]=3;')
 s=s.replace('v->current=a==6?2000:100;','v->current=broadcasts?v->goal:0;')
 # Full lifecycle and polling, real Begin and bus-owner policy.
 backend=(P/'CHASSIS/USER/vehicle_source_parallel.inc').read_text(encoding='utf-8')
 it=(P/'CHASSIS/USER/integration10_complete.inc').read_text(encoding='utf-8')
 cycle=(P/'CHASSIS/USER/source_mcu_cycle.inc').read_text(encoding='utf-8')
 support=r'''
#define APP_MOTOR_OFFLINE_ONLY 0
#define LRC_DONE 4
#define LRC_MOVING 2
static uint32_t g_source_parallel_state,g_source_parallel_cause,g_source_parallel_started_mask;
static uint32_t g_source_parallel_done_mask,g_source_parallel_stop_mask,g_source_parallel_start_skew_ms;
static uint32_t g_vehicle_arm_axis_owned,g_vehicle_arm_dm_owned,g_vehicle_arm_state,g_vehicle_arm_cause;
static uint32_t g_extension_home_reference_valid=1,g_vehicle_lift_reference_valid=1;
static uint32_t g_startup_axes_state=4,g_startup_axes_cause,g_extension_hold_confirmed=1;
static uint32_t g_vehicle_lift_feedback_valid=1,g_lift_ruler_cause,g_lift_ruler_state=4;
static uint32_t g_lift_ruler_request,g_lift_ruler_stop_pending,g_lift_ruler_samples,g_lift_ruler_rejected;
static uint32_t g_safety_ready=1,g_can_test_result=1,g_safety_fault,g_rtos_fault,g_can_ctrl_fault_latched,g_chassis_auto_active,g_can_guard_ready=1;
static uint32_t g_it_auto_state,g_it_auto_fault,g_src_now_ms,g_source_body_step_request;
static float g_startup_extension_current_deg,g_startup_extension_target_deg,g_vehicle_lift_current_deg;
static uint32_t g_source_cycle_state=1,g_source_cycle_prep=4;
typedef struct{unsigned active,cause;}ChassisAutoStatus;
static unsigned ChassisAuto_ReadStatus(ChassisAutoStatus*x){memset(x,0,sizeof(*x));return 1;}
static unsigned SingleTeach_ReferencesReady(void){return g_extension_home_reference_valid && g_vehicle_lift_reference_valid;}
static unsigned VehicleArm_Busy(void){return vsp_owned;}
static unsigned Low_ObserveReturnSettling(void){return 0;}
static unsigned Low_Busy(void){return 0;}
static unsigned Low_XYAllowed(void){return 0;}
static unsigned Region_ObserveYawAllowed(void){return 0;}
static unsigned Stage3_Owns(void){return 0;}
static unsigned S3_PrecloseOnDmStarted(void){return 1;}
static void StartupAxes_SetExtensionHoldTarget(float p){g_startup_extension_target_deg=p;}
static unsigned StartupAxes_Begin(void){return 1;}
static unsigned SourceBodyStep_WheelWindow(void){return 0;}
static unsigned StartupAxes_Holding(void){return g_startup_axes_state==4;}
static unsigned Stage2Fast_IsActive(void){return 0;}
static unsigned SourceStage2_IsActive(void){return 0;}
static void Vision_Poll(void){}
static unsigned jiazhua(float a){(void)a;return 1;}
static unsigned CarrierPwm_Write(unsigned a){(void)a;return 1;}
static unsigned Vision_TakeTaskTarget(unsigned t){(void)t;return 0;}
static unsigned Vision_TakeTarget(unsigned t){(void)t;return 0;}
static unsigned SrcTxn_Take(unsigned t){(void)t;return 0;}
static unsigned it_auto_start(unsigned t){(void)t;return 0;}
static unsigned it_empty_overlap_begin(void){return 0;}
static unsigned it_loaded_overlap_begin(void){return 0;}
static void Low_Service(unsigned t){(void)t;}
static void Low_Fail(unsigned t){(void)t;}
static void RegionDispatch_Service(unsigned t){(void)t;}
static void Region_Service(unsigned t){(void)t;}
static void PZQNear_Service(unsigned t){(void)t;}
static void SourceStage2_Service(unsigned t){(void)t;}
static void Stage2Fast_Service(unsigned t){(void)t;}
static void Stage2Fast_Cancel(unsigned t){(void)t;}
static void SourceStage2_Cancel(unsigned t){(void)t;}
static void StartupAxes_Cancel(void){}
static void StartupAxes_Service(unsigned a,unsigned b){(void)a;(void)b;}
static void it_loaded_overlap_gates(unsigned t){(void)t;}
static void it_lead5_grip_service(unsigned t){(void)t;}
static void it_auto_service(unsigned t){(void)t;}
static void it_service_lift_command(unsigned t){(void)t;}
static void it_empty_overlap_publish(void){}
static void it_loaded_overlap_publish(void){}
'''
 # Enlarge the fake lift struct to fields consumed by the actual full IT service.
 s=s.replace('static struct{unsigned top_scale;}g_lift_ruler_controller;',
  'static struct{unsigned top_scale,top_valid,backoff_valid,scale,operation,samples,state,moves_sent;int64_t safe_position_raw,position_after_raw;}g_lift_ruler_controller;')
 # Prefix comes before VSP structs; insert support immediately after real VSP code.
 s+=support
 s+=it[it.index('volatile uint32_t g_it_request'):it.index('/* ===== ')]
 for sig in ('static uint32_t it_platform(','static uint32_t it_hold_platform(','static uint32_t it_at_top(','static uint32_t it_dm_fresh('):s+=function(it,sig)+'\n'
 s+=function(cycle,'static uint32_t SC_InitialObserveAllowed(')+'\n'
 for sig in ('static uint32_t VSP_PositionWindow(','static uint32_t VSP_Poll(','static uint32_t VSP_Clock(','static uint32_t VehicleSourceParallel_Owns(','static void VehicleSourceParallel_Cancel(','static void VSP_ReleasePickDescent(','static void VehicleSourceParallel_Service(','uint32_t VehicleArm_BeginSourceParallel(','uint32_t VehicleArm_SourceParallelCanAllowed('):s+=function(backend,sig)+'\n'
 s+='static void VehicleArm_Service(void){VehicleSourceParallel_Service();}\n'
 for sig in ('static void it_fail(','static void it_done(','void Integration10_Service('):s+=function(it,sig)+'\n'
 tests=r'''
static void initial(float work,unsigned fast){
 unsigned fallback=fast==2;fast=fast?1:0;
 setup(1);if(fallback)g_dm_can2_ctrl_mode=1;g_dm_initial_observe=0;vsp_owned=vsp_pick_sync=vsp_pick_queued=0;cached=broadcasts=failstage=0;
 memset(vsp_axis,0,sizeof(vsp_axis));memset(&vsp_group,0,sizeof(vsp_group));
 g_dm_can2_position_rad=feedback=sent=(DM_CAN2_LEFT_ZERO_DEG+work)*JOG_RAD_PER_DEG;
 g_dm_can2_position_deg=feedback*JOG_DEG_PER_RAD;g_dm_can2_logical_position_deg=work;
 g_dm_can2_velocity_rad_s=0;g_dm_can2_read_cycle=10;g_dm_can2_read_request=0;
 g_dm_posvel_holding=fast;g_dm_jog_state=fast?7:0;g_dm_jog_result=fast?1:0;
 g_dm_jog_request=0;s_enable_possible=fast;s_start=s_target=s_reference=feedback;s_feedback.position_rad=feedback;s_feedback.motor_state=fast;
 motor=fast;g_dm_can2_motor_state=motor;s_point_mode=0;s_queued_request=0;
 g_it_state=g_it_cause=g_it_rejected=0;g_it_request=3;g_it_axes=3;g_it_dm_deg=180;g_it_extension_mm=65;g_it_lift_pulses=29700;
 g_source_cycle_state=1;g_source_cycle_prep=4;g_startup_extension_current_deg=g_extension_home_reference_deg=0;
 g_startup_axes_state=4;g_vehicle_lift_current_deg=0;g_lift_ruler_state=4;
 g_lift_ruler_controller.top_valid=g_lift_ruler_controller.backoff_valid=1;g_lift_ruler_controller.top_scale=g_lift_ruler_controller.scale=100;
 g_lift_ruler_controller.safe_position_raw=0;g_lift_ruler_samples=it_seen_samples=0;
 g_source_parallel_state=g_source_parallel_started_mask=0;
 g_dm_stage2_sync=g_dm_parallel_group_active=g_dm_parallel_group_go=0;
}
static void service(void){
 tick+=12;DM_CAN2_ModuleService(1,0);Integration10_Service();
}
int main(void){unsigned i,fast,scenario;const float positions[]={-4.999f,-4,-3.1146f,-3.103683f,-2,0};
 for(fast=0;fast<=2;fast++)for(unsigned k=0;k<sizeof(positions)/sizeof(positions[0]);k++){
  initial(positions[k],fast);
  for(i=0;i<100 && !broadcasts && g_it_state!=9;i++){
   service();if(!broadcasts){CHECK(!g_dm_parallel_group_go);CHECK(fabsf(sent-feedback)<.001f);}
  }
  if(!(i<100 && broadcasts==1 && cached==2 && g_it_state==1 && !g_it_cause))printf("debug first fast%u work%f i%u it%u cause%u reject%u phase%u vsp%u fault%u dm%u result%u first%u token%u read%u cycle%u\n",fast,positions[k],i,g_it_state,g_it_cause,g_it_rejected,it_phase,vsp_group.state,vsp_group.fault,g_dm_jog_state,g_dm_jog_result,g_dm_jog_first_fault,g_dm_initial_observe,g_dm_can2_read_request,g_dm_can2_read_cycle);
  CHECK(i<100 && broadcasts==1 && cached==2 && g_it_state==1 && !g_it_cause);
  CHECK(vsp_group.prepared==3 && vsp_group.started==3 && g_dm_parallel_group_go && !g_dm_initial_observe);
  CHECK(fabsf(sent-feedback)<.001f);service();CHECK(g_dm_jog_state==4 && sent>feedback);
  feedback=sent;for(i=0;i<100 && g_it_state==1;i++)service();
  CHECK(i<100 && g_it_state==2 && !g_it_cause && g_source_parallel_state==3 && g_source_parallel_done_mask==3 && !vsp_owned);
 }
 for(fast=0;fast<=1;fast++){
  initial(-5.01f,fast);for(i=0;i<100 && g_it_state!=9;i++)service();
  if(!(g_dm_jog_first_fault==14 && !broadcasts && !g_source_parallel_started_mask))printf("debug below fast%u dm%u result%u first%u it%u cause%u broadcasts%u start%u\n",fast,g_dm_jog_state,g_dm_jog_result,g_dm_jog_first_fault,g_it_state,g_it_cause,broadcasts,g_source_parallel_started_mask);
  CHECK(g_dm_jog_first_fault==14 && !broadcasts && !g_source_parallel_started_mask);
 }
 for(scenario=0;scenario<9;scenario++){
  initial(-3.1146f,1);
  if(scenario==0)g_source_cycle_state=2;
  if(scenario==1)g_source_cycle_prep=3;
  if(scenario==2)g_it_axes=1;
  if(scenario==3)g_it_dm_deg=195;
  if(scenario==4)g_it_extension_mm=0;
  if(scenario==5)g_startup_extension_current_deg=2.01f;
  if(scenario==6)g_vehicle_lift_current_deg=SOURCE_TOP_TOLERANCE_DEG+1;
  if(scenario==7)g_extension_home_reference_valid=0;
  if(scenario==8)g_it_extension_mm=164;
  for(i=0;i<40 && g_it_state!=9 && !g_it_rejected;i++)service();
  CHECK(!g_dm_initial_observe && !broadcasts);g_extension_home_reference_valid=1;
 }
 for(scenario=0;scenario<4;scenario++){
  initial(-3.1146f,1);for(i=0;i<40 && g_dm_jog_state!=3;i++)service();CHECK(g_dm_jog_state==3 && !broadcasts);
  if(scenario==0){g_it_request=9;service();}
  if(scenario==1){VehicleSourceParallel_Cancel();service();}
  if(scenario==2){failstage=6;for(i=0;i<40 && g_it_state!=9;i++)service();}
  if(scenario==3){failstage=7;for(i=0;i<40 && g_it_state!=9;i++)service();}
  CHECK(!g_dm_initial_observe && !broadcasts && !g_dm_parallel_group_go);
  for(i=0;i<40 && vsp_owned;i++)service();CHECK(!vsp_owned && !vsp_group.stop_mask);
 }
 initial(-3.1146f,1);g_dm_initial_observe=g_dm_stage2_sync=g_dm_parallel_group_active=1;
 CHECK(JogPointStartAllowed(-5,180));CHECK(!JogPointStartAllowed(-5.001f,180));
 CHECK(!JogPointStartAllowed(NAN,180));CHECK(!JogPointStartAllowed(-INFINITY,180));CHECK(!JogPointStartAllowed(INFINITY,180));
 g_dm_initial_observe=0;CHECK(!JogPointStartAllowed(-2.001f,180));CHECK(JogPointStartAllowed(-2,180));
 printf("PASS full actual Integration10_Service + VSP + DM + SPF: %u checks, cold/fast first180/65 -5..0 completion, below-5 refusal, unrelated scope -2, failures STOP\n",checks);return 0;}
'''
 # Both hardware adapter declarations need the later mock globals: lift/support
 # is intentionally moved before full IT code, VSP extracted code uses originals.
 results['full_joint']=run('actual-full-it3-vsp-dm',s+tests,[P/'CHASSIS/USER/source_parallel_flow.c'])
 old_motion='\n'.join(x for x in (BASE/'HARDWARE/DM_CAN2/dm_can2_motion.c').read_text(encoding='utf-8').splitlines() if not x.startswith('#include'))
 new_motion='\n'.join(x for x in (P/'HARDWARE/DM_CAN2/dm_can2_motion.c').read_text(encoding='utf-8').splitlines() if not x.startswith('#include'))
 assert s.count(new_motion)==1
 repro=s.replace(new_motion,'volatile uint32_t g_dm_initial_observe;\n'+old_motion,1)
 repro+=tests[:tests.index('int main(void)')]+r'''
int main(void){for(unsigned fast=0;fast<=1;fast++){
 initial(-3.1146f,fast);for(unsigned i=0;i<100 && g_it_state!=9;i++)service();
 CHECK(g_it_state==9 && g_dm_jog_first_fault==14 && !broadcasts && !g_source_parallel_started_mask);
}printf("PASS original cb DM reproduced fault14 before broadcast: %u checks, cold/fast actual complete IT/VSP chain\n",checks);return 0;}
'''
 results['original_dm_reproduction']=run('actual-original-dm-fault14-repro',repro,[P/'CHASSIS/USER/source_parallel_flow.c'])
 (E/'actual-c-result.json').write_text(json.dumps(dict(passed=True,results=results,hardware_access=False,full_it_service=True),indent=2))

if __name__=='__main__':main()
