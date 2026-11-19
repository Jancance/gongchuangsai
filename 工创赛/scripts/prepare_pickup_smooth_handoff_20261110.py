"""Build a separate timing/terminal-approach candidate, with no hardware I/O."""
from pathlib import Path
import json,hashlib,shutil,sys
R=Path(__file__).resolve().parents[1]
B=R/'tmp/pickup_trial1_settle_20261110/project'
P=R/'tmp/pickup_smooth_handoff_20261110/project'
E=R/'evidence/integration/2026-11-10/smooth-handoff'
PREV=R/'evidence/integration/2026-11-10/speed-trials-settle'

def main():
    assert json.loads((E/'matlab.json').read_text(encoding='utf-8'))['passed']
    assert not P.exists() or '--resume-partial' in sys.argv
    old=json.loads((PREV/'candidate.json').read_text(encoding='utf-8'))
    for rel,digest in old['changed'].items():assert hashlib.sha256((B/rel).read_bytes()).hexdigest()==digest,rel
    if not P.exists():shutil.copytree(B,P,ignore=shutil.ignore_patterns('Objects','Listings'))
    def edit(rel,old,new):
        p=P/rel;b=p.read_bytes();a=old.encode('utf-8');z=new.encode('utf-8')
        if b.count(z)==1:return
        assert b.count(a)==1,(rel,old[:70]);p.write_bytes(b.replace(a,z,1))
    v='CHASSIS/USER/vehicle_source_parallel.inc'
    edit(v,'static uint32_t vsp_fast_lift;', '''static uint32_t vsp_fast_lift;
volatile uint32_t g_pick_extension_segment;
volatile float g_pick_extension_waypoint_deg,g_pick_extension_final_goal_deg;
#define VSP_EXT_TAIL_DEG (8.0f*VEHICLE_EXTENSION_PULSES_PER_MM*360.0f/3200.0f)''')
    edit(v,'  units=(uint32_t)(fabsf(delta)*', '''  if(address==6U){
   g_pick_extension_segment=0U;g_pick_extension_final_goal_deg=a->goal;
   if(g_pick_trial_profile>=2U && g_pick_trial_profile<=3U && fabsf(delta)>2.0f*VSP_EXT_TAIL_DEG &&
    (c->goal.extension_mm==0.0f || c->goal.extension_mm==156.0f || c->goal.extension_mm==160.0f)){
    g_pick_extension_segment=1U;
    g_pick_extension_waypoint_deg=a->goal-(delta<0.0f?-VSP_EXT_TAIL_DEG:VSP_EXT_TAIL_DEG);
    delta=g_pick_extension_waypoint_deg-a->current;
   }
  }
  units=(uint32_t)(fabsf(delta)*''')
    edit(v,' zero_return=(address==6U && vsp_group.command.goal.extension_mm==0.0f);', ''' zero_return=(address==6U && vsp_group.command.goal.extension_mm==0.0f);
 if(address==6U && a->phase==3U && g_pick_extension_segment==1U){
  uint32_t units,result;float delta;
  if(!VSP_Position(address,a))return 0U;
  if(!VSP_PositionWindow(a,VSP_SettleAllowed(address,zero_return)))return 2U;
  delta=a->goal-a->current;
  if(fabsf(delta)>VSP_EXT_TAIL_DEG+45.1f)return 2U;
  units=(uint32_t)(fabsf(delta)*10.0f+0.5f);
  memset(a->move,0,15);a->move[0]=0xfd;a->move[1]=delta<0;
  a->move[2]=a->move[4]=2U;a->move[3]=a->move[5]=88U; /* 600 RPM/s */
  a->move[6]=4U;a->move[7]=36U; /* 106 RPM */
  a->move[8]=(uint8_t)(units>>24);a->move[9]=(uint8_t)(units>>16);
  a->move[10]=(uint8_t)(units>>8);a->move[11]=(uint8_t)units;a->move[12]=2U;a->move[14]=0x6b;
  if(units){result=VehicleArm_Exchange(address,a->move,15U,3U,&s_rx);if(result!=1U && result!=5U)return 2U;}
  g_pick_extension_segment=2U;a->samples=0U;a->phase=0U;return 0U;
 }''')
    edit(v,'fabsf(a->current-a->goal)<=(zero_return?2.0f:45.0f)',
         'fabsf(a->current-((address==6U && g_pick_extension_segment==1U)?g_pick_extension_waypoint_deg:a->goal))<=((zero_return && g_pick_extension_segment!=1U)?2.0f:45.0f)')
    edit(v,' a->phase=0;return a->samples>=3;', ''' if(address==6U && g_pick_extension_segment==1U && a->samples>=3U){a->phase=3U;return 0U;}
 a->phase=0;return a->samples>=3;''')
    edit(v,' vsp_scurve_active=vsp_scurve_issued=0U;', ' vsp_scurve_active=vsp_scurve_issued=0U;g_pick_extension_segment=0U;')
    edit(v,' if(address==6U && n==11U && q[0]==0xfdU)', ''' if(address==6U && n==15U && q[0]==0xfdU && vsp_group.state==2U &&
  g_pick_extension_segment==1U && a->phase==3U && (vsp_group.started&SPF_EXT) &&
  !memcmp(q,a->move,15U))return 1U;
 if(address==6U && n==11U && q[0]==0xfdU)''')
    it='CHASSIS/USER/integration10_complete.inc'
    edit(it,' vsp_fast_lift=1U;g_it_carrier_auto_done=0U;g_it_lead5_grip_done=0U;', ''' vsp_fast_lift=1U;g_it_carrier_auto_done=0U;g_it_lead5_grip_done=0U;
 if(g_pick_trial_profile>=2U && g_it_extension_mm>0.0f){
  vsp_group.delayed_mask=SPF_LIFT;vsp_group.release_mask=SPF_EXT;
 }''')
    edit(it,'static void it_empty_overlap_publish(void)', '''static void it_empty_overlap_gates(void)
{
#if !APP_MOTOR_OFFLINE_ONLY
 if(g_it_state==1U && g_it_action==14U && VehicleSourceParallel_Owns() &&
  vsp_group.delayed_mask==SPF_LIFT && g_pick_extension_segment==2U &&
  (vsp_group.started&SPF_EXT))vsp_group.release_mask|=SPF_LIFT;
#endif
}
static void it_empty_overlap_publish(void)''')
    edit(it,'static void it_lead5_grip_service(uint32_t now)', '''volatile uint32_t g_it_lead5_grip_tick;
volatile float g_it_lead5_grip_depth,g_it_lead5_grip_x_error;
static uint32_t it_grip_extension_ready(uint32_t now)
{
#if APP_MOTOR_OFFLINE_ONLY
 (void)now;return 0U;
#else
 VSP_Axis *x=&vsp_axis[0];
 if(!(it_command.axes_mask&SPF_EXT))return 1U;
 if(vsp_group.done&SPF_EXT)return 1U;
 return g_pick_extension_segment!=1U && x->samples>=1U && x->position_cycle &&
  (uint32_t)(now-x->position_tick)<=100U && (x->flags&0x0fU)==3U &&
  fabsf(x->current-x->goal)<=45.0f;
#endif
}
static void it_lead5_grip_service(uint32_t now)''')
    edit(it,'  ((it_command.axes_mask&SPF_EXT) && !(vsp_group.done&SPF_EXT)))return;', '  !it_grip_extension_ready(now))return;')
    edit(it,' g_it_lead5_grip_done=1U;', ''' g_it_lead5_grip_done=1U;g_it_lead5_grip_tick=(uint32_t)xTaskGetTickCount();
 g_it_lead5_grip_depth=depth;g_it_lead5_grip_x_error=vsp_axis[0].current-vsp_axis[0].goal;''')
    edit(it,' it_loaded_overlap_gates((uint32_t)xTaskGetTickCount());',
         ' it_empty_overlap_gates();\r\n it_loaded_overlap_gates((uint32_t)xTaskGetTickCount());')
    chain='CHASSIS/USER/integration13_immediate.inc'
    edit(chain,'static uint32_t it_auto_begin,it_auto_phase_begin,it_auto_sequence;',
         'static uint32_t it_auto_begin,it_auto_phase_begin,it_auto_sequence;\r\nvolatile uint32_t g_it_auto_next_request_ms[3];')
    edit(chain,' g_it_auto_release_ms[0]=g_it_auto_release_ms[1]=g_it_auto_release_ms[2]=0U;',
         ' g_it_auto_release_ms[0]=g_it_auto_release_ms[1]=g_it_auto_release_ms[2]=0U;\r\n g_it_auto_next_request_ms[0]=g_it_auto_next_request_ms[1]=g_it_auto_next_request_ms[2]=0U;')
    edit(chain,'before200ms.', 'before50ms.')
    edit(chain,'  if(g_it_auto_phase_ms<200U)return;',
         '  if(g_it_auto_phase_ms<50U)return;\r\n  g_it_auto_next_request_ms[g_it_auto_item]=now-it_auto_begin;')
    edit(chain,'  g_it_auto_release_ms[g_it_auto_item]=now-it_auto_begin;',
         '  it_auto_phase_begin=(uint32_t)xTaskGetTickCount();\n  g_it_auto_release_ms[g_it_auto_item]=it_auto_phase_begin-it_auto_begin;')
    edit(chain,'  g_it_auto_phase=6U;break;', '  g_it_auto_phase=6U;return;')
    dm='HARDWARE/DM_CAN2/dm_can2_motion.c'
    edit(dm,'JogAbs(s_target - s_feedback.position_rad) <= 3.0F * JOG_RAD_PER_DEG',
         'JogAbs(s_target - s_feedback.position_rad) <= ((g_dm_parallel_group_active && g_dm_integrated_owner)?1.0F:3.0F) * JOG_RAD_PER_DEG')
    edit(dm,'s_settled >= 15U &&', 's_settled >= ((g_dm_parallel_group_active && g_dm_integrated_owner)?3U:15U) &&')
    m={k:v for k,v in old.items() if k not in ('field_runs','field_review','latest_run_complete','mcu_three_complete','field_verified_profiles')}
    m.update(base='tmp/pickup_trial1_settle_20261110/project',parent_deployed_sha256=old['images']['Vehicle_LiftRuler']['sha256'],
             images={},revision='staged_extension_and_grip50ms_handoff',hardware_access=False,deployed=False,actual_tests_run=0,field_verified=False,
             release_to_next_request_ms=50,terminal_extension_mm=8,terminal_extension_rpm=106,
             terminal_extension_accel_decel=600,dm_parallel_settled_samples=3,dm_parallel_window_deg=1,
             motion_change='High profiles stage extension in both directions; descent starts with tail; first fresh zero extension sample permits pregrip; 3 fresh tight DM samples; 50ms after PWM release.')
    m['changed']={}
    for p in P.rglob('*'):
        if p.is_file():
            rel=p.relative_to(P);z=R/'handoff/pzq/project'/rel
            if not z.exists() or z.read_bytes()!=p.read_bytes():m['changed'][rel.as_posix()]=hashlib.sha256(p.read_bytes()).hexdigest()
    (E/'candidate.json').write_text(json.dumps(m,ensure_ascii=False,indent=2)+'\n',encoding='utf-8')
    for name in ('timing-model.json','integration13-three-direct-matlab-20261109.json'):
        shutil.copy2(PREV/name,E/name)
    for kind in ('build','check','live'):
        name=f'{kind}_pickup_trial1_settle_20261110.py' if kind!='live' else 'pickup_trial1_settle_live_20261110.py'
        text=(R/'scripts'/name).read_text(encoding='utf-8').replace('pickup_trial1_settle_20261110','pickup_smooth_handoff_20261110').replace('speed-trials-settle','smooth-handoff')
        text=text.replace('.read_text()',".read_text(encoding='utf-8')")
        dest=name.replace('pickup_trial1_settle','pickup_smooth_handoff');(R/'scripts'/dest).write_text(text,encoding='utf-8')
    print('Prepared separate smooth-handoff candidate; original pzq and field-tested source remain intact.')

if __name__=='__main__':main()
