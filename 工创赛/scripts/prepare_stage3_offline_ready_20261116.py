"""Minimal offline teach candidate; MATLAB proof required, no hardware imports."""
from pathlib import Path
import json,shutil
R=Path(__file__).resolve().parents[1]
BASE=R/'tmp/stage3_preclose160_20261116/project'
P=R/'tmp/stage3_offline_ready_20261116/project'
E=R/'evidence/integration/2026-11-16/stage3-offline-ready'
S=R/'src/stage3_offline_ready_20261116'

def save(name,b):
 (P/name).write_bytes(b);(S/name).parent.mkdir(parents=True,exist_ok=True);(S/name).write_bytes(b)

def main():
 proof=json.loads((E/'matlab-result.json').read_text());assert proof['passed']
 assert not P.exists(),'Preserve existing candidate'
 shutil.copytree(BASE,P,ignore=shutil.ignore_patterns('Objects','Listings'))
 name='CHASSIS/USER/stage3_cycle.inc';b=(P/name).read_bytes()
 old=b'volatile uint32_t g_stage3_teach_request,g_stage3_teach_result;'
 new=old+b'\r\n/* result:0 idle,1 saved,2 pending real DM feedback,3 not-ready/cancelled. */\r\nvolatile uint32_t g_stage3_teach_missing,g_stage3_teach_dm_source,g_stage3_teach_z_source;\r\nvolatile float g_stage3_teach_measured_x=-1.0f;'
 assert b.count(old)==1;b=b.replace(old,new)
 old=b'extern volatile uint8_t g_lift_diag_raw[6][8];';assert b.count(old)==1
 b=b.replace(old,b'extern volatile uint32_t g_lift_diag_raw[6][8];')
 old=b'static uint32_t s3_teach_pending,s3_teach_z_cycle,s3_teach_x_cycle,s3_teach_dm_cycle;'
 new=b'static uint32_t s3_teach_pending,s3_teach_dm_cycle,s3_teach_dm_tick,s3_teach_holding;\r\nstatic float s3_teach_z,s3_teach_x;'
 assert b.count(old)==1;b=b.replace(old,new)
 start=b.index(b'static void S3_Teach(void)');end=b.index(b'static void S3_LockConfig(void)',start)
 code='''static uint32_t S3_TeachReady(void)
{
 return (g_stage3_state==0U || g_stage3_state==7U || g_stage3_state==11U) &&
  SingleTeach_ReferencesReady() && g_it_state==2U && !g_it_request &&
  !g_it_cause && !g_it_rejected && !g_stage3_request;
}
static void S3_Teach(void)
{
 uint32_t n=g_stage3_teach_request,needs_dm;g_stage3_teach_request=0U;
 if(s3_teach_pending && !S3_TeachReady()){
  s3_teach_pending=0U;g_stage3_teach_result=3U;g_stage3_teach_missing=1U;
 }
 if(n && !s3_teach_pending){
  g_stage3_teach_result=3U;g_stage3_teach_missing=0U;
  g_stage3_teach_z_source=g_stage3_teach_dm_source=0U;
  if(n>5U || !S3_TeachReady()){g_stage3_teach_missing=1U;return;}
  if(n!=5U && (!g_vehicle_lift_feedback_valid || !g_lift_ruler_controller.top_scale)){
   g_stage3_teach_missing=2U;return;
  }
  if((n==2U || n==4U) && !(g_stage3_teach_measured_x>=0.0f && g_stage3_teach_measured_x<=170.0f)){
   g_stage3_teach_missing=4U;return;
  }
  /* Existing IT completion forwards actual VSP/LRC samples. Never use target Z. */
  s3_teach_z=g_stage3_actual_depth;s3_teach_x=g_stage3_teach_measured_x;
  s3_teach_pending=n;g_stage3_teach_result=2U;
  if(n==2U || n==4U || n==5U){
   s3_teach_dm_cycle=g_dm_can2_read_cycle;s3_teach_dm_tick=g_dm_can2_feedback_tick;
   s3_teach_holding=g_dm_posvel_holding?1U:0U;
   /* Holding already owns and polls CAN2. Never run generic read beside it. */
   if(!s3_teach_holding)g_dm_can2_read_request=2U;
  }
 }
 if(!s3_teach_pending)return;
 n=s3_teach_pending;needs_dm=(n==2U || n==4U || n==5U);
 if(needs_dm){
  g_stage3_teach_missing=8U;
  if(s3_teach_holding){
   if(!g_dm_posvel_holding){s3_teach_pending=0U;g_stage3_teach_result=3U;return;}
   if(g_dm_can2_feedback_tick==s3_teach_dm_tick || !it_dm_fresh())return;
   g_stage3_teach_dm_source=1U;
  }else{
   if(g_dm_can2_read_cycle==s3_teach_dm_cycle)return;
   if(g_dm_can2_state==5U){s3_teach_pending=0U;g_stage3_teach_result=3U;return;}
   if(g_dm_can2_state!=4U || !it_dm_fresh())return;
   g_stage3_teach_dm_source=2U;
  }
 }
 if(n==1U)g_stage3_pick_depth=s3_teach_z;
 else if(n>=2U && n<=4U){
  if(n!=3U){g_stage3_target_yaw[n-2U]=g_stage3_actual_yaw;g_stage3_target_x[n-2U]=s3_teach_x;}
  g_stage3_target_depth[n-2U]=s3_teach_z;
 }else g_stage3_carrier_yaw=g_stage3_actual_yaw;
 if(n!=5U)g_stage3_teach_z_source=1U;
 s3_teach_pending=0U;g_stage3_teach_missing=0U;g_stage3_teach_result=1U;
}
'''.replace('\n','\r\n').encode('ascii')
 b=b[:start]+code+b[end:];save(name,b)
 name='CHASSIS/USER/stage3_cycle.h';b=(P/name).read_bytes();old=b'extern volatile uint32_t g_stage3_teach_request,g_stage3_teach_result;'
 assert b.count(old)==1;b=b.replace(old,old+b'\r\nextern volatile uint32_t g_stage3_teach_missing,g_stage3_teach_dm_source,g_stage3_teach_z_source;\r\nextern volatile float g_stage3_teach_measured_x;');save(name,b)
 print('Prepared offline teach candidate after MATLAB '+proof['when'])
if __name__=='__main__':main()
