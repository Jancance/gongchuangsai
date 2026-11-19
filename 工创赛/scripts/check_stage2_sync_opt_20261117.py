"""Execute actual Stage2 owner, queued VSP/group and complete DM jog source."""
from pathlib import Path
import re,json,shutil,subprocess
from prepare_stage2_sync_opt_20261117 import R,BASE,P,E
import build_integration23_sync_20261115 as runner
def run(name,source,extra=()):
 runner.P=P;runner.E=E;runner.run(name,source,extra)
def owner():
 prior=R/'evidence/integration/2026-11-16/source-real-qr'
 shutil.copyfile(prior/'can.h',E/'can.h')
 (E/'tested-commit-owner.inc').write_bytes((P/'CHASSIS/USER/pzq_commit_overlap.inc').read_bytes().replace(b'#include "low_readonly_diag.inc"',b'/* separate diagnostics */'))
 s=(prior/'all-regions-owner.c').read_text(encoding='utf-8')
 s=s.replace('vsp_pick_lead_ms,vsp_pick_sync;', 'vsp_pick_lead_ms,vsp_pick_sync,g_dm_stage2_sync;')
 s=s.replace('vsp_pick_sync=0;owned=1;', 'vsp_pick_sync=g_dm_stage2_sync=0;owned=1;')
 s=s.replace('(region==2?(SPF_EXT|SPF_LIFT):SPF_DM)', '(region==2?6U:7U)')
 s=s.replace('tick+=299;', 'tick+=99;').replace('g_low_release_wait_ms==300','g_low_release_wait_ms==100')
 s=s.replace('vsp_group.command.axes_mask==6 && vsp_group.command.goal.lift_pulses==29700', 'vsp_group.command.axes_mask==(region==2?6U:7U) && vsp_group.command.goal.lift_pulses==29700')
 s=s.replace('CHECK(region<2?(vsp_pick_sync && !vsp_group.delayed_mask):(!vsp_pick_sync && vsp_group.delayed_mask==SPF_LIFT));', 'CHECK(vsp_pick_sync && !vsp_group.delayed_mask && !vsp_pick_lead_ms);CHECK(g_dm_stage2_sync==(region<2));')
 s=s.replace('g_low_pick_overlap_axes==6', 'g_low_pick_overlap_axes==(region==2?6U:7U)').replace('pulse+300','pulse+100').replace('recorded==(region==2?3U:4U)','recorded==3U')
 s=s.replace('static uint32_t Gripper_PulseApplied(void){return tick>=pulse_apply_at;}',
   'static unsigned pulse_read_delay;static uint32_t Gripper_PulseApplied(void){if(pulse_read_delay){tick+=pulse_read_delay;return 1;}return tick>=pulse_apply_at;}')
 matrix=r'''
static void timing_matrix(void){unsigned w,slot,before;
 for(w=0;w<2;w++){
  reset();tick=w?0xffffffd0U:1000U;low_begin=low_phase_tick=tick;low_slot=1;
  g_low_state=1;g_low_phase=6;g_it_state=1;low_release_applied=0;pulse_read_delay=7;
  Low_Service(tick);CHECK(low_pulse_tick==tick && !g_low_release_wait_ms && !recorded);
  pulse_read_delay=0;tick=low_pulse_tick+99U;Low_Service(tick);CHECK(!recorded && g_low_phase==6);
  tick++;Low_Service(tick);CHECK(recorded==1 && g_low_phase==7 && g_low_release_wait_ms==100);
 }
 for(slot=1;slot<=3;slot++){
  start_region(0,slot);finish_group();CHECK(g_low_phase==5 && owned);
  tick=low_tray_tick+700U;before=releases;finish_group();
  CHECK(g_low_phase==6 && releases==before+1); /* open in this completion call */
  start_region(0,slot);finish_group();tick=low_tray_tick+100U;before=releases;finish_group();
  CHECK(g_low_phase==12 && releases==before);tick=low_tray_tick+699U;Low_Service(tick);CHECK(releases==before);
  tick++;Low_Service(tick);CHECK(g_low_phase==6 && releases==before+1);
 }
 puts("PASS actual owner timing matrix: PWM-applied call latency, wrap100, full-pose immediate open, real700 tray window");
}
'''
 s=s.replace('int main(void){',matrix+'int main(void){timing_matrix();',1)
 (E/'all-regions-owner.c').write_text(s)
 run('actual-owner',s)
def queued():
 # Reuse test transport scaffolding; execute current real VSP_Start and policy.
 old=(R/'scripts/build_integration23_sync_20261115.py').read_text(encoding='utf-8');prefix=old.split("    prefix=r'''",1)[1].split("'''\n    suffix",1)[0]
 axis=(P/'CHASSIS/USER/vehicle_source_parallel.inc').read_text(encoding='utf-8');axisdecl=axis[axis.index('typedef struct {uint32_t phase'):axis.index('static VSP_Axis')]
 prefix=prefix.replace("'''+axis+r'''",axisdecl)
 prefix=prefix.replace('g_dm_parallel_group_go;', 'g_dm_parallel_group_go,g_dm_stage2_sync;')
 prefix=prefix.replace('vsp_group.prepared==6 && vsp_group.started==6','(vsp_group.prepared&6)==6 && (vsp_group.started&6)==6')
 prefix=prefix.replace('if(fail==3)return 4;trigger_count++;moving=queued;return 1;', 'CHECK(!g_dm_stage2_sync || !g_dm_parallel_group_go);if(fail==3)return 4;trigger_count++;moving=queued;return 1;')
 start=axis[axis.index('static uint32_t VSP_StartQueuedPick('):axis.index('static uint32_t VSP_Poll(')]
 policy=axis[axis.index('uint32_t VehicleArm_SourceParallelCanAllowed('):]
 suffix=r'''
static unsigned prep(void*p,unsigned b,const SPF_Command*c){(void)p;CHECK(c->extension_rpm==150);return fail==b+3?2:1;}
static unsigned poll(void*p,unsigned b){(void)p;(void)b;return 1;}
static unsigned stop(void*p,unsigned b){(void)p;stops|=b;return 1;}
static unsigned clk(void*p){(void)p;return tick;}
static void setup(unsigned axes,unsigned delayed){
 SPF_Command c={0};SPF_GroupIO io={0,prep,VSP_Start,poll,stop,clk};
 tick=queued=moving=stops=calls=trigger_count=vsp_pick_queued=g_dm_parallel_group_go=0;
 memset(&vsp_group,0,sizeof(vsp_group));memset(vsp_axis,0,sizeof(vsp_axis));
 vsp_axis[0].goal=5000;vsp_axis[1].goal=3000;vsp_axis[0].move[14]=vsp_axis[1].move[14]=0x6b;
 c.axes_mask=axes;c.kind=SPF_CMD_PICK;c.goal.extension_mm=162;c.goal.lift_pulses=29700;
 c.dm_deg_s=300;c.extension_rpm=150;c.lift_rpm=1846.4f;c.stepper_accel_rpm_s=600;c.keep_vm_on=1;
 CHECK(SPF_GroupBegin(&vsp_group,&c,&io,1,90,12000));g_dm_stage2_sync=(axes==7 && !delayed);
 if(delayed){vsp_group.delayed_mask=1;vsp_group.release_mask=6;}
}
int main(void){unsigned axes,d,i;unsigned char q[3]={255,102,107};
 for(axes=6;axes<=7;axes++)for(d=0;d<=1;d++){
  if(d && axes==6)continue;
  for(fail=0;fail<=7;fail++){
   setup(axes,d);for(i=0;i<20 && vsp_group.state<3;i++){
    SPF_GroupService(&vsp_group);
    if(moving){CHECK(vsp_pick_queued==6 && trigger_count==1);CHECK(g_dm_stage2_sync?g_dm_parallel_group_go==1:1);}
    else CHECK(!g_dm_parallel_group_go);
    if(d && moving && vsp_group.state==2){CHECK(!(vsp_group.started&1));vsp_group.release_mask|=1;}
   }
   if(fail<=3 && fail){SPF_GroupService(&vsp_group);CHECK(vsp_group.state==4 && stops==axes && !moving);}
   else if((fail==4 && axes==7)||fail==5||fail==7){SPF_GroupService(&vsp_group);CHECK(vsp_group.state==4 && stops==axes);CHECK((d && fail==4)?moving==6:!moving);}
   else CHECK(vsp_group.state==3 && moving==6);
  }
  fail=0;setup(axes,d);SPF_GroupService(&vsp_group);SPF_GroupCancel(&vsp_group,2);SPF_GroupService(&vsp_group);CHECK(stops==axes && !moving);
 }
 fail=0;setup(7,0);vsp_group.state=1;vsp_group.started=7;vsp_pick_queued=6;
 CHECK(VehicleArm_SourceParallelCanAllowed(0,q,3));vsp_group.started=6;CHECK(!VehicleArm_SourceParallelCanAllowed(0,q,3));
 vsp_group.state=2;vsp_group.delayed_mask=1;CHECK(VehicleArm_SourceParallelCanAllowed(0,q,3));vsp_group.delayed_mask=0;CHECK(!VehicleArm_SourceParallelCanAllowed(0,q,3));
 printf("PASS actual queued VSP/group: %u checks; LR7/near6/rise6/return7, failures STOP, scoped broadcast\n",checks);return 0;
}
'''
 run('actual-queued-vsp-group',prefix+start+policy+suffix,[P/'CHASSIS/USER/source_parallel_flow.c'])
def dm():
 code=(P/'HARDWARE/DM_CAN2/dm_can2_motion.c').read_text(encoding='utf-8')
 code='\n'.join(x for x in code.splitlines() if not x.startswith('#include'))
 hdr=(P/'HARDWARE/DM_CAN2/dm_can2.h').read_text(encoding='utf-8')
 globals='\n'.join(x.replace('extern ','',1) for x in hdr.splitlines() if x.startswith('extern ') and ('g_dm_can2_' in x))
 prefix=r'''
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <math.h>
#include <string.h>
#include "vehicle_parameters.h"
#include "source_prepare_math.h"
#include "dm_can2_startup_plan.h"
typedef uint32_t TickType_t;
typedef struct{unsigned ExtId,IDE,RTR,DLC;unsigned char Data[8];}CanTxMsg;
typedef CanTxMsg CanRxMsg;
typedef struct{uint32_t position_raw,velocity_raw,torque_raw,motor_state;float position_rad,velocity_rad_s,torque_nm;}DM_CAN2_Feedback;
static struct{unsigned ESR;}bus;
#define CAN2 (&bus)
#define CAN_ESR_BOFF 4
#define CAN_FIFO0 0
#define CAN_FLAG_FOV0 1
#define CAN_TxStatus_NoMailBox 4
#define CAN_TxStatus_Ok 1
#define CAN_TxStatus_Failed 2
#define RESET 0
#define DM_CAN2_MODE 2U
#define DM_CAN2_POSVEL_SPEED_DEG_S VEHICLE_DM_SPEED_DEG_S
#define DM_CAN2_JOG_DEG 45.0F
#define DM_CAN2_LEFT_ZERO_DEG VEHICLE_DM_ZERO_RAW_DEG
#define pdMS_TO_TICKS(x) (x)
#define portTICK_PERIOD_MS 1
static unsigned tick,pending,stops,txs,failtx,checks,stopdelay,reply_at;static float sent,feedback;static unsigned motor=1;
#define CHECK(x) do{checks++;if(!(x)){printf("FAIL %u %s\n",__LINE__,#x);exit(1);}}while(0)
'''+globals+r'''
static unsigned xTaskGetTickCount(void){return tick;}
static void DM_CAN2_CancelRead(void){}
static void DM_CAN2_Service(unsigned a,unsigned b){(void)a;(void)b;}
static unsigned DM_CAN2_BuildMode2(CanTxMsg*t){t->Data[0]=9;return 1;}
static unsigned DM_CAN2_BuildSwitch(unsigned c,CanTxMsg*t){t->Data[0]=c;return 1;}
static unsigned DM_CAN2_BuildPosVel(float p,float v,CanTxMsg*t){(void)v;t->Data[0]=1;sent=p;return 1;}
static unsigned CAN_MessagePending(void*b,unsigned f){(void)b;(void)f;return pending && tick>=reply_at;}
static void CAN_Receive(void*b,unsigned f,CanRxMsg*r){(void)b;(void)f;(void)r;pending=0;}
static void CAN_ClearFlag(void*b,unsigned f){(void)b;(void)f;}
static unsigned CAN_GetFlagStatus(void*b,unsigned f){(void)b;(void)f;return 0;}
static unsigned CAN_Transmit(void*b,CanTxMsg*t){(void)b;txs++;if(failtx)return 4;if(t->Data[0]==253){stops++;motor=0;}reply_at=tick+(t->Data[0]==253?stopdelay:0);pending=1;return 0;}
static unsigned CAN_TransmitStatus(void*b,unsigned m){(void)b;(void)m;return 1;}
static void CAN_CancelTransmit(void*b,unsigned m){(void)b;(void)m;pending=0;}
static unsigned DM_CAN2_ParseStatus(const CanRxMsg*r,float p,float v,float t,DM_CAN2_Feedback*f){(void)r;(void)p;(void)v;(void)t;*f=(DM_CAN2_Feedback){0};f->position_rad=feedback;f->motor_state=motor;return 1;}
'''
 suffix=r'''
static void setup(unsigned scope){
 tick=100;s_last_service=tick;g_dm_jog_first_fault=0;stopdelay=reply_at=0;pending=stops=txs=failtx=0;motor=1;
 g_dm_point_enable=1;g_dm_can2_feedback_valid=1;g_dm_can2_feedback_tick=tick;
 g_dm_can2_state=4;g_dm_can2_result=1;g_dm_can2_ctrl_mode=2;g_dm_can2_motor_state=1;
 g_dm_can2_velocity_rad_s=0;g_dm_can2_pmax=12.5f;g_dm_can2_vmax=30;g_dm_can2_tmax=10;
 g_dm_can2_position_rad=feedback=sent=(DM_CAN2_LEFT_ZERO_DEG+180)*JOG_RAD_PER_DEG;
 g_dm_pick_target_deg=195;g_dm_parallel_group_active=1;g_dm_parallel_group_go=0;g_dm_stage2_sync=scope;
 g_dm_posvel_holding=1;g_dm_jog_state=7;g_dm_jog_request=4;g_dm_integrated_owner=1;
 s_pending=s_enable_possible=s_mode_pending=0;s_start=s_target=s_reference=feedback;s_feedback=(DM_CAN2_Feedback){0};s_feedback.position_rad=feedback;s_feedback.motor_state=1;
}
int main(void){unsigned f;
 setup(1);DM_CAN2_ModuleService(1,0);CHECK(g_dm_jog_state==3 && s_pending && s_enable_possible && txs==1);CHECK(fabsf(sent-feedback)<.001f);
 tick+=12;DM_CAN2_ModuleService(1,0);CHECK(g_dm_jog_state==3 && txs==2 && fabsf(sent-feedback)<.001f);
 g_dm_parallel_group_go=1;tick+=12;DM_CAN2_ModuleService(1,0);CHECK(g_dm_jog_state==4 && txs==3 && fabsf(sent-s_target)<.001f && sent>feedback);
 for(f=0;f<3;f++){setup(1);DM_CAN2_ModuleService(1,0);g_dm_jog_request=3;tick+=12;DM_CAN2_ModuleService(1,0);CHECK(g_dm_jog_state==6 && stops==1);tick+=12;DM_CAN2_ModuleService(1,0);CHECK(g_dm_jog_state==8 && g_dm_jog_stop_result==1);}
 setup(1);failtx=1;DM_CAN2_ModuleService(1,0);CHECK(g_dm_jog_state==8 && g_dm_jog_first_fault==5);
 setup(0);DM_CAN2_ModuleService(1,0);CHECK(g_dm_jog_state==4 && sent>feedback); /* S3/original fast path */
 printf("PASS complete actual DM source: %u checks; real fastprepare hold/pending/go target and STOP ack, non-S2 fast unchanged\n",checks);return 0;}
'''
 # Startup plan header is simple host-safe; copied to keep real constants.
 shutil.copyfile(P/'HARDWARE/DM_CAN2/dm_can2_startup_plan.h',E/'dm_can2_startup_plan.h')
 run('actual-complete-dm-jog',prefix+code+suffix)
 backend=(P/'CHASSIS/USER/vehicle_source_parallel.inc').read_text(encoding='utf-8')
 axis=backend[backend.index('typedef struct {uint32_t phase'):backend.index('static uint32_t VSP_Query(')]
 axis=axis.replace('volatile uint32_t g_source_parallel_sync_tick;', 'static uint32_t g_source_parallel_sync_tick;')
 prepare=backend[backend.index('static uint32_t VSP_Prepare('):backend.index('static uint32_t VSP_PositionWindow(')]
 start=backend[backend.index('static uint32_t VSP_StartQueuedPick('):backend.index('static uint32_t VSP_Poll(')]
 stop=backend[backend.index('static uint32_t VSP_Stop('):backend.index('static uint32_t VSP_Clock(')]
 support=r'''
#include "source_parallel_flow.h"
'''+axis+r'''
static struct{unsigned char Data[8];}s_rx;
static struct{unsigned top_scale;}g_lift_ruler_controller;
static float g_extension_home_reference_deg;
static unsigned cached,broadcasts,failstage;
static float SingleTeach_TopDeg(void){return 0;}
static unsigned VSP_Query(uint8_t a,uint8_t c,uint8_t n){(void)a;(void)n;memset(&s_rx,0,sizeof(s_rx));if(c==0x3a)s_rx.Data[1]=1;return 1;}
static unsigned VSP_Position(uint8_t a,VSP_Axis*v){v->current=a==6?-2000:100;return 1;}
static unsigned VehicleArm_Exchange(uint8_t a,const uint8_t*q,unsigned n,unsigned count,void*out){
 (void)count;(void)out;
 if(n==15){CHECK(q[13]==1 && !g_dm_parallel_group_go);if(failstage==a)return 4;cached|=a==6?2:4;}
 s_rx.Data[1]=2;return 1;
}
static unsigned CAN1_StartQueuedSourceAxes(void){CHECK(cached==6 && !g_dm_parallel_group_go);if(failstage==7)return 0;broadcasts++;return 1;}
'''+prepare+start+stop+r'''
static unsigned poll_axes(void*p,unsigned b){(void)p;(void)b;return 0;}
static unsigned clock_axes(void*p){(void)p;return tick;}
'''
 joint=r'''
int main(void){unsigned i,f,delay;SPF_Command c={0};SPF_GroupIO io={0,VSP_Prepare,VSP_Start,poll_axes,VSP_Stop,clock_axes};
 for(delay=0;delay<=36;delay+=12)for(f=0;f<5;f++){
  setup(1);stopdelay=delay;vsp_owned=1;vsp_pick_lead_ms=0;CHECK(!vsp_pick_lead_ms && vsp_owned);g_dm_jog_request=0;g_dm_can2_logical_position_deg=180;
  vsp_dm_posted=vsp_dm_hold_tick=cached=broadcasts=vsp_pick_queued=0;vsp_pick_sync=1;
  failstage=f==1?6:(f==2?5:(f==3?7:0));
  memset(vsp_axis,0,sizeof(vsp_axis));memset(&vsp_group,0,sizeof(vsp_group));
  c.axes_mask=7;c.kind=SPF_CMD_PICK;c.dm_deg_s=300;c.extension_rpm=150;c.lift_rpm=1846.4f;c.stepper_accel_rpm_s=600;c.keep_vm_on=1;
  c.goal.dm_deg=195;c.goal.extension_mm=162;c.goal.lift_pulses=29700;
  CHECK(SPF_GroupBegin(&vsp_group,&c,&io,1,90,12000));
  for(i=0;i<100 && vsp_group.state<2;i++){
   tick+=12;DM_CAN2_ModuleService(1,0);
   if(f==4 && g_dm_jog_state==3U){SPF_GroupCancel(&vsp_group,2);break;}
   SPF_GroupService(&vsp_group);
   if(!broadcasts)CHECK(!g_dm_parallel_group_go && fabsf(sent-feedback)<.001f);
  }
  CHECK(i<100 && (f==4?!(vsp_group.prepared&1):(vsp_group.prepared&1)));
  if(!f){CHECK(broadcasts==1 && g_dm_parallel_group_go && vsp_group.state==2);CHECK(fabsf(sent-feedback)<.001f);
   tick+=12;DM_CAN2_ModuleService(1,0);CHECK(g_dm_jog_state==4 && sent>feedback);SPF_GroupCancel(&vsp_group,2);
  }else CHECK(vsp_group.state==4 && !broadcasts && !g_dm_parallel_group_go);
  for(i=0;i<20 && vsp_group.stop_mask;i++){tick+=12;DM_CAN2_ModuleService(1,0);SPF_GroupService(&vsp_group);tick++;DM_CAN2_ModuleService(1,0);}
  if(vsp_group.stop_mask || stops!=1 || g_dm_jog_stop_result!=1)printf("debug f%u mask%u stops%u jog%u result%u pending%u motor%u\n",f,vsp_group.stop_mask,stops,g_dm_jog_state,g_dm_jog_stop_result,s_pending,motor);CHECK(vsp_group.stop_mask==0 && stops==1 && g_dm_jog_stop_result==1);
 }
 printf("PASS joint real VSP_Prepare + full DM + queued group: %u checks; fresh hold ready, broadcast release, next-owner target, cache/broadcast failures and STOP\n",checks);return 0;}
'''
 setup=suffix[:suffix.index('int main(void)')]
 run('actual-joint-vsp-dm',prefix+code+support+setup+joint,[P/'CHASSIS/USER/source_parallel_flow.c'])
def main():
 assert json.loads((E/'matlab-result.json').read_text(encoding='utf-8'))['passed']
 owner();queued();dm()
 (E/'actual-c-result.json').write_text(json.dumps({'passed':True,'tests':['actual-owner','actual-queued-vsp-group','actual-complete-dm-jog','actual-joint-vsp-dm'],'hardware_access':False},indent=2))
if __name__=='__main__':main()
