"""Whole chain uses actual source preparation/arm; DM/PWM and extension plant modeled.
Actual DM trajectory controller is separately checked in check_source_parallel.
"""
import re
def adapt(u):
    u=u.replace('scenario>=20 && stopped','scenario>=20 && scenario<30 && stopped')
    u=u.replace('g_radar_zone_ready,g_radar_zone_selected,g_vehicle_arm_state,g_vehicle_arm_cause;', 'g_radar_zone_ready,g_radar_zone_selected;')
    u=u.replace('static uint32_t g_vehicle_lift_feedback_valid;','')
    u=u.replace('static float g_vehicle_lift_current_deg,s_perimeter_targets[9][3];','static float s_perimeter_targets[9][3];')
    for name in ['VehicleArm_Busy','VehicleArm_Cancel','VehicleArm_Begin']:
        u=re.sub(r'static (?:uint32_t|void) '+name+r'\([^\n]*\}\n','',u)
    u=u.replace('float g_dm_posvel_speed_deg_s;',r'''
float g_dm_posvel_speed_deg_s,g_dm_pick_target_deg,g_dm_place_target_deg,g_dm_can2_logical_position_deg;
uint32_t g_dm_can2_read_cycle,g_dm_can2_feedback_tick;
uint32_t g_dm_jog_stop_result;
volatile uint32_t g_dm_integrated_owner,g_dm_source_smooth,g_dm_source_go;
volatile float g_dm_source_accel_deg_s2;
static uint32_t model_source_dm,model_source_dm_started,model_source_fault,model_source_queued;
static float model_source_position=-25.6f,model_source_target=-25.6f;
''')
    mark=' if(g_dm_jog_request==6){'
    u=u.replace(mark,r'''
 if(g_dm_jog_request==4 || g_dm_jog_request==5){g_dm_jog_request=0;model_source_queued=1;g_dm_jog_state=8;g_dm_jog_result=2;g_dm_jog_stop_result=1;g_dm_posvel_holding=0;return;}
 if(model_source_queued){model_source_queued=0;model_source_dm=1;model_source_dm_started=0;g_dm_jog_state=3;g_dm_can2_motor_state=1;g_dm_posvel_holding=0;g_dm_can2_read_cycle++;}
 if(model_source_dm){
  g_dm_can2_read_cycle++;g_dm_can2_feedback_tick=test_tick;
  if(model_source_fault==1){g_dm_jog_first_fault=7;return;}
  if(!g_dm_source_go)return;
  if(!model_source_dm_started)model_source_dm_started=test_tick;
  unsigned done=test_tick-model_source_dm_started>=4000;
  g_dm_jog_state=done?7:4;g_dm_jog_result=done;g_dm_posvel_holding=done;
  g_dm_can2_logical_position_deg=done?180:180.0f*(test_tick-model_source_dm_started)/4000;return;
 }
'''+mark)
    mark='#include "startup_group.inc"'
    model=r'''
#define DM_CAN2_H
uint32_t DM_CAN2_SourceTransitionPending(void){return model_source_queued && g_dm_jog_state==8 && g_dm_jog_result==2 && g_dm_jog_stop_result==1 && g_dm_source_smooth && !g_dm_source_go;}
static float model_source_grip=145;
uint8_t jiazhua(float deg){if(model_source_fault==3)return 0;model_source_grip=deg;return deg>=145 && deg<=180;}
uint32_t CarrierPwm_Write(uint32_t pulse){(void)pulse;return 1;}
#include "vehicle_arm.c"
uint32_t VehicleArm_Exchange(uint8_t address,const uint8_t *q,uint8_t length,uint8_t rn,void *out){
 CanRxMsg *p=out;uint32_t raw;memset(p,0,sizeof(*p));p->Data[0]=q[0];p->Data[1]=2;
 if(address!=6 || model_source_fault==2)return 5;
 if(q[0]==0x1a)p->Data[2]=0x80;
 if(q[0]==0x36){raw=(uint32_t)(fabsf(model_source_position)*10+.5f);p->Data[1]=model_source_position<0;p->Data[2]=raw>>24;p->Data[3]=raw>>16;p->Data[4]=raw>>8;p->Data[5]=raw;}
 if(q[0]==0x35)p->Data[3]=fabsf(model_source_position-model_source_target)>.1f;
 if(q[0]==0x3a)p->Data[1]=model_source_fault==4?9:(fabsf(model_source_position-model_source_target)<.1f?3:1);
 if(q[0]==0xfd){
  raw=((uint32_t)q[8]<<24)|((uint32_t)q[9]<<16)|((uint32_t)q[10]<<8)|q[11];
  ck(length==15 && raw<=18000 && q[12]==2);
  model_source_target=model_source_position+(q[1]?-1:1)*(float)raw/100;
 }
 if(q[0]==0xfe)model_source_target=model_source_position;
 (void)rn;return 1;
}
'''
    u=u.replace(mark,model+'\n'+mark)
    mark=' model_settle_case=model_pre_reads=model_speed_interrupt=0;'
    assert mark in u
    u=u.replace(mark,mark+r'''
 model_source_dm=model_source_dm_started=model_source_fault=model_source_queued=g_dm_jog_stop_result=0;model_source_position=model_source_target=-25.6f;model_source_grip=145;
 g_dm_integrated_owner=g_dm_source_smooth=g_dm_source_go=g_dm_can2_read_cycle=g_dm_can2_feedback_tick=0;
 g_dm_can2_logical_position_deg=0;
 g_source_prepare_state=g_source_prepare_cause=g_source_prepare_done_mask=g_source_prepare_release_ms=g_source_prepare_elapsed_ms=g_source_prepare_segment=0;
 g_vehicle_arm_state=g_vehicle_arm_cause=g_vehicle_arm_axis_owned=g_vehicle_arm_dm_owned=g_vehicle_arm_stop_pending=0;
''')
    mark='   StartupAxes_Service(g_safety_ready,g_can_guard_ready);dispatch();'
    assert mark in u
    u=u.replace(mark,r'''
   StartupAxes_Service(g_safety_ready,g_can_guard_ready);
   if(model_source_fault!=5){
    float delta=model_source_target-model_source_position;
    float step=fminf(fabsf(delta),.15f);model_source_position+=delta<0?-step:step;
   }
   if(StartupAxes_Holding())DM_CAN2_ModuleService(1,0);
   if(VehicleArm_Busy() || g_vehicle_arm_stop_pending)VehicleArm_Service();
   if(StartupAxes_Holding())DM_CAN2_ModuleService(1,0);
   if(!VehicleArm_Busy())dispatch();
''')
    # New run denied while the preparatory extension remains out.
    u=u.replace('if(scenario==19 && g_vehicle_state==4 && !rerun){','if(scenario==19 && g_vehicle_state==4 && !rerun){')
    u=u.replace('sent=0;rerun=1;g_vehicle_key=1;VehicleMission_Service();','rerun=1;g_vehicle_key=1;VehicleMission_Service();')
    u=u.replace('memset(plant_pos,0,sizeof(plant_pos));memset(plant_velocity,0,sizeof(plant_velocity));memset(plant_target,0,sizeof(plant_target));\n    rerun=1;', 'rerun=1;')
    u=u.replace('ck(g_vehicle_stage==2 && g_vehicle_state==4 && !g_vehicle_cause &&', 'ck(g_vehicle_stage==2 && g_vehicle_state==4 && (g_vehicle_cause==(scenario==19?19U:0U)) &&')
    u=u.replace('display_calls==(scenario==19?2:1)','display_calls==1')
    u=u.replace('if(scenario==19)ck(g_vehicle_run_id==2);','if(scenario==19)ck(g_vehicle_run_id==1 && g_vehicle_cause==19);')
    u=u.replace('for(unsigned scenario=0;scenario<30;scenario++)','for(unsigned scenario=0;scenario<37;scenario++)')
    mark='   if(scenario==13 && g_vehicle_stage==2 && moving)g_vehicle_key=1;'
    u=u.replace(mark,mark+r'''
   if(scenario>=30 && g_source_prepare_state==2){
    if(scenario<=34)model_source_fault=scenario-29;
    if(scenario==35)g_vehicle_key=1;
    if(scenario==36)g_safety_fault=1;
   }
   if(g_source_prepare_state==1 || g_source_prepare_state==2){
    ck(g_vehicle_stage==2 && g_vehicle_state==3 && !g_vehicle_alignment_state && !g_chassis_auto_active && !g_vm_test_request);
   }
''')
    mark='  VehicleAlignmentRequest req;ck(!arm_calls && !test_depth && s_vehicle_zone==1);'
    u=u.replace(mark,mark+r'''
  if(g_vehicle_state==4)ck(g_source_prepare_state==3 && g_source_prepare_done_mask==7 && fabsf(g_source_prepare_extension_mm-70)<.2f && fabsf(model_source_grip-180)<.1f);
  if(scenario>=30)ck(g_source_prepare_release_ms && g_source_prepare_state==4 && g_vehicle_alignment_state==3 && !fixture_vm);
''')
    return u
