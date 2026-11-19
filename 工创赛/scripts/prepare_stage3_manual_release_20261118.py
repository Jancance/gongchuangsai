"""Prepare and build the isolated bench manual-teach candidate."""
import hashlib
import json
from pathlib import Path
import shutil
import subprocess

ROOT=Path(__file__).resolve().parents[1]
BASE=ROOT/'tmp/stage3_three_ring_points_20261118/project'
PROJECT=ROOT/'tmp/stage3_manual_release_20261118/project'
E=ROOT/'evidence/integration/2026-11-18/stage3-resume'
USER='CHASSIS/USER/'


def edit(name, old, new):
    path=PROJECT/name
    raw=path.read_bytes()
    newline=b'\r\n' if b'\r\n' in raw else b'\n'
    def encode(s):
        return newline.join(s.encode('ascii').split(b'\n'))
    old,new=encode(old),encode(new)
    assert raw.count(old)==1,(name,old)
    path.write_bytes(raw.replace(old,new,1))


MANUAL=r'''/* Bench-only ownership, with independent DM and extension readback. */
volatile uint32_t g_stage3_manual_request,g_stage3_manual_state;
#if VEHICLE_EXTENSION_HOME_ONLY
extern volatile uint32_t g_stage3_manual_x;
#endif
static uint32_t S3_ManualTeach(void)
{
#if VEHICLE_EXTENSION_HOME_ONLY && !APP_MOTOR_OFFLINE_ONLY
 uint32_t request=g_stage3_manual_request;
 g_stage3_manual_request=0U;
 if(!g_stage3_manual_state){
  if(request!=1U || !g_region_dispatch_test_enable || g_stage3_state!=7U ||
     !S3_TeachReady() || !it_platform() || !it_at_top() || !g_low_wheel_hold_ready ||
     !g_extension_hold_confirmed || !g_dm_posvel_holding || g_dm_can2_motor_state!=1U ||
     g_dm_jog_request || g_stage3_teach_request || s3_teach_pending)return 0U;
  g_stage3_manual_state=1U;
  g_dm_jog_request=3U;
  return 1U;
 }
 if(g_stage3_request==9U || g_it_request==9U || !it_platform() || g_stage3_fault ||
    g_startup_axes_cause || g_dm_jog_first_fault || g_stage3_manual_x==4U){
  if(g_stage3_manual_x==3U)g_stage3_manual_x=2U;
  if(g_stage3_manual_state==4U && g_startup_axes_state>=1U && g_startup_axes_state<=3U)
   StartupAxes_Cancel();
  g_stage3_manual_state=5U;
  /* Pass STOP to the existing owner. Other requests remain inhibited. */
  return (g_stage3_request!=9U && g_it_request!=9U);
 }
 if(g_stage3_manual_state==1U){
  if(g_dm_jog_stop_result==2U)g_stage3_manual_state=5U;
  else if(g_dm_jog_state==8U && g_dm_jog_result==2U && g_dm_jog_stop_result==1U &&
          !g_dm_posvel_holding && g_dm_can2_motor_state==0U){
   g_stage3_manual_x=1U;
   g_stage3_manual_state=2U;
  }
 }else if(g_stage3_manual_state==2U && g_stage3_manual_x==2U){
  g_stage3_manual_state=3U;
 }else if(g_stage3_manual_state==3U && request==2U){
  g_stage3_manual_state=4U;
  g_stage3_manual_x=3U;
 }else if(g_stage3_manual_state==4U && g_stage3_manual_x==0U &&
          StartupAxes_Holding() && g_extension_home_reference_valid && g_extension_hold_confirmed){
  g_stage3_manual_state=0U;
 }
 return 1U;
#else
 g_stage3_manual_request=0U;
 return 0U;
#endif
}
'''


def main():
    model=json.loads((E/'manual-xyz-teach-matlab.json').read_text(encoding='utf-8'))
    assert model['passed'] and model['model_cases']==89
    shutil.copytree(BASE,PROJECT,ignore=shutil.ignore_patterns('Objects','Listings'))
    edit(USER+'stage3_cycle.inc','static void S3_Teach(void)',MANUAL+'\nstatic void S3_Teach(void)')
    # The declaration lives in region_dispatch.inc below stage3_cycle.inc.
    edit(USER+'stage3_cycle.inc','/* Bench-only ownership, with independent DM and extension readback. */',
         '/* Bench-only ownership, with independent DM and extension readback. */\nextern volatile uint32_t g_region_dispatch_test_enable;')
    edit(USER+'integration10_complete.inc',' RegionDispatch_Service((uint32_t)xTaskGetTickCount());',
         ''' if(g_region_dispatch_area==3U && S3_ManualTeach()){
  g_it_request=0U;g_stage3_request=0U;g_stage3_teach_request=0U;
  g_src_now_ms=now;Vision_Poll();StartupAxes_Service(it_hold_platform(),g_can_guard_ready);return;
 }
 RegionDispatch_Service((uint32_t)xTaskGetTickCount());''')
    edit(USER+'extension_home_only.inc','volatile uint32_t g_extension_hold_enable_count=0U,g_extension_hold_samples=0U;',
         '''/* 0 normal, 1 release request, 2 readback disabled, 3 rehome request, 4 failed. */
volatile uint32_t g_stage3_manual_x;
volatile uint32_t g_extension_hold_enable_count=0U,g_extension_hold_samples=0U;''')
    edit(USER+'extension_home_only.inc','    if(g_startup_axes_state==4U){',
         '''    if(g_startup_axes_state==4U && g_stage3_manual_x){
        if(g_stage3_manual_x==1U){
            const uint8_t disable[5]={0xf3,0xab,0,0,0x6b};
            if(!eh_control(disable,5U) || !eh_query(0x3a,3U) || (eh_reply.Data[1]&0x09U)){
                g_stage3_manual_x=4U;eh_fail(5U);return;
            }
            g_extension_hold_confirmed=0U;
            g_extension_home_reference_valid=0U;
            g_startup_extension_feedback_valid=0U;
            g_stage3_manual_x=2U;
        }else if(g_stage3_manual_x==3U){
            g_startup_axes_state=0U;
            if(StartupAxes_Begin())g_stage3_manual_x=0U;
            else g_stage3_manual_x=4U;
        }
        return;
    }
    if(g_startup_axes_state==4U){''')
    # A feedback report that says enabled is also evidence that disable is needed.
    dm='HARDWARE/DM_CAN2/dm_can2_motion.c'
    edit(dm,'    if (request == 3U && (DM_CAN2_JogIsBusy() || s_enable_possible))',
         '''    if (request == 3U && (DM_CAN2_JogIsBusy() || s_enable_possible || g_dm_posvel_holding || g_dm_can2_motor_state==1U))''')
    edit(dm,'    if (!s_enable_possible)\n    { g_dm_jog_state = 8U; return; }',
         '''    if (!s_enable_possible && g_dm_can2_motor_state==0U)
    { g_dm_jog_state = 8U; return; }''')
    images={}
    for target in ('Vehicle_LiftRuler','Vehicle_LiftRuler_Offline'):
        for folder in ('Objects','Listings'):
            (PROJECT/folder/target).mkdir(parents=True,exist_ok=True)
        log=E/('manual-release-'+target+'.log')
        run=subprocess.run(['C:/Keil_v5/UV4/UV4.exe','-r',str(PROJECT/'gongxun-f407.uvprojx'),
                            '-t',target,'-j0','-o',str(log)],timeout=180)
        assert run.returncode==0 and b'0 Error(s), 0 Warning(s)' in log.read_bytes(),log
        axf=PROJECT/'Objects'/target/(target+'.axf')
        images[target]={'axf':str(axf),'sha256':hashlib.sha256(axf.read_bytes()).hexdigest()}
        print(target,'0 errors, 0 warnings',flush=True)
    report={'project':str(PROJECT),'images':images,'flashed':False,'physical_teach_verified':False}
    (E/'manual-release-candidate.json').write_text(json.dumps(report,indent=2),encoding='utf-8')


if __name__=='__main__':
    main()
