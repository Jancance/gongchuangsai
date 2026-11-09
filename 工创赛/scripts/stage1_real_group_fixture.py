"""Use actual startup_group/extension controllers in the whole-flow fixture.
DM, PWM, home-driver replies, scheduler, CAN and plant remain models.
"""
import re


def adapt(unit):
    unit=unit.replace('volatile uint32_t g_startup_axes_state;','')
    unit=unit.replace('uint32_t StartupAxes_Holding(void){return g_startup_axes_state==4 && fixture_vm;}','')
    unit=unit.replace('volatile uint32_t g_startup_axes_cause,g_startup_extension_feedback_valid;','')
    unit=unit.replace('volatile float g_startup_extension_current_deg;','')
    unit=unit.replace('static uint32_t startup_begin_tick;','')
    unit=re.sub(r'uint32_t StartupAxes_Begin\(void\)\{[^\n]*\}\n','',unit)
    unit=re.sub(r'void StartupAxes_Cancel\(void\)\{[^\n]*\}\n','',unit)
    unit=unit.replace('static uint32_t App_SerialDiagnosticWindow(uint32_t now,uint32_t ms){(void)now;(void)ms;return 0;}',
                      'static uint32_t App_SerialDiagnosticWindow(uint32_t now,uint32_t ms){(void)now;(void)ms;return 1;}')
    unit=unit.replace('static uint16_t App_SerialCrc(const char *s,uint32_t n){(void)s;(void)n;return 0;}',
                     'static uint16_t App_SerialCrc(const char *s,uint32_t n){uint16_t c=0xffff;for(uint32_t i=0;i<n;i++){c^=(uint16_t)(uint8_t)s[i]<<8;for(uint32_t j=0;j<8;j++)c=(uint16_t)((c&0x8000)?(c<<1)^0x1021:c<<1);}return c;}')
    unit=unit.replace('static uint32_t App_DebugSendString(const char *s){(void)s;return 1;}',
                     'static uint32_t model_last_tx_tick;\nstatic uint32_t App_DebugSendString(const char *s){if(model_last_tx_tick==test_tick)return 0;model_last_tx_tick=test_tick;printf("%s",s);return 1;}')
    group=r'''
#define DM_CAN2_POSVEL_SPEED_DEG_S VEHICLE_DM_SPEED_DEG_S
volatile uint32_t g_dm_startup_go,g_dm_jog_first_fault;
uint32_t g_dm_point_enable,g_dm_jog_request,g_dm_jog_state,g_dm_can2_motor_state,g_dm_jog_result,g_dm_posvel_holding;
float g_dm_posvel_speed_deg_s;
static uint32_t model_home_tick,model_extension_timeout,model_ext_enabled=1,model_home_drop_once;
static uint32_t model_home_disable,model_keep_disabled,model_drop_late,model_hold_protect,model_enable_fail,model_position_jump;
static uint32_t model_enable_attempts,model_service_injected;
static uint32_t model_settle_case,model_pre_reads,model_speed_interrupt;
uint32_t StartupPose_GroupValid(void){return 1;}
uint32_t StartupPose_GroupCommand(void){return 1;}
uint32_t DM_CAN2_JogIsBusy(void){return g_dm_jog_state>0 && g_dm_jog_state<7;}
void DM_CAN2_ModuleService(uint32_t ready,uint32_t busy){
 (void)busy;
 if(g_dm_jog_request==3 || !ready){g_dm_jog_request=0;g_dm_jog_state=8;g_dm_posvel_holding=0;return;}
 if(g_dm_jog_request==6){g_dm_jog_request=0;g_dm_jog_state=3;g_dm_can2_motor_state=1;}
 if(g_dm_startup_go){
  extern volatile uint32_t g_startup_group_release_ms;
  uint32_t done=test_tick-g_startup_group_release_ms>=300;
  g_dm_jog_state=done?7:4;g_dm_jog_result=done;g_dm_posvel_holding=done;
 }
}
uint32_t StartupAxis_Exchange(const uint8_t *q,uint8_t length,uint8_t reply_n,void *out){
 (void)length;(void)reply_n;CanRxMsg *rx=out;memset(rx,0,sizeof(*rx));rx->Data[0]=q[0];rx->Data[1]=2;
 if(q[0]==0x1a)rx->Data[2]=0;
 if(q[0]==0x3a){
  if(model_extension_timeout && g_chassis_auto_active){test_tick+=30;return 5;}
  if(model_drop_late)model_ext_enabled=0;
  rx->Data[1]=2U|(model_ext_enabled?1U:0U);
  if(model_hold_protect && model_home_tick && test_tick-model_home_tick>=100)rx->Data[1]|=8U;
 }
 if(q[0]==0x3b){
  extern volatile uint32_t g_extension_home_done_notice;
  rx->Data[1]=model_home_tick && test_tick-model_home_tick<100?7:3;
  if(model_home_tick && test_tick-model_home_tick>=100){
   g_extension_home_done_notice=1;
   if(model_home_disable && !model_home_drop_once){model_ext_enabled=0;model_home_drop_once=1;}
  }
 }
 if(q[0]==0x35){
  extern volatile uint32_t g_extension_home_state,g_extension_hold_samples;
  uint32_t rpm=0;
  if(g_extension_home_state==9 && !model_enable_attempts){
   model_pre_reads++;
   if(model_settle_case==18 || ((model_settle_case==17 || model_settle_case==22) && model_pre_reads<=2))rpm=model_settle_case==22?256:123;
  }
  if(g_extension_home_state==9 && model_enable_attempts){
   if(model_settle_case==20)rpm=300;
   if(model_settle_case==19 && g_extension_hold_samples==1 && !model_speed_interrupt){rpm=1;model_speed_interrupt=1;}
   if(model_settle_case==21)return 5;
  }
  rx->Data[1]=1;rx->Data[2]=rpm>>8;rx->Data[3]=rpm&255;
 }
 if(q[0]==0x36){uint32_t raw=256U+(model_position_jump && model_enable_attempts?100U:0U);rx->Data[1]=1;rx->Data[2]=raw>>24;rx->Data[3]=raw>>16;rx->Data[4]=raw>>8;rx->Data[5]=raw;}
 if(q[0]==0x9a)model_home_tick=test_tick;
 if(q[0]==0xf3 && q[2]){model_enable_attempts++;if(model_enable_fail)return 5;if(!model_keep_disabled)model_ext_enabled=1;}
 return 1;
}
uint32_t StartupAxis_ReadHomeParameters(uint8_t p[17]){
 memset(p,0,17);p[0]=0x22;p[16]=0x6b;p[2]=VEHICLE_EXTENSION_HOME_DIRECTION;
 p[3]=VEHICLE_EXTENSION_HOME_RPM>>8;p[4]=VEHICLE_EXTENSION_HOME_RPM;
 p[5]=VEHICLE_EXTENSION_HOME_DRIVER_TIMEOUT_MS>>24;p[6]=VEHICLE_EXTENSION_HOME_DRIVER_TIMEOUT_MS>>16;
 p[7]=VEHICLE_EXTENSION_HOME_DRIVER_TIMEOUT_MS>>8;p[8]=VEHICLE_EXTENSION_HOME_DRIVER_TIMEOUT_MS&255;
 p[9]=VEHICLE_EXTENSION_HOME_DETECT_RPM>>8;p[10]=VEHICLE_EXTENSION_HOME_DETECT_RPM&255;
 p[11]=VEHICLE_EXTENSION_HOME_DETECT_CURRENT_MA>>8;p[12]=VEHICLE_EXTENSION_HOME_DETECT_CURRENT_MA&255;
 p[13]=VEHICLE_EXTENSION_HOME_DETECT_TIME_MS>>8;p[14]=VEHICLE_EXTENSION_HOME_DETECT_TIME_MS;
 return 1;
}
#include "startup_group.inc"
static void model_group_reset(void){
 g_startup_axes_vm_request=g_group_extension_state=g_group_extension_cause=g_group_extension_vm_request=0;
 g_extension_home_state=g_extension_home_result=g_extension_home_reported=g_extension_home_flags=0;
 g_extension_home_capture=g_extension_home_done_notice=g_extension_home_elapsed_ms=0;
 eh_begin=eh_motion=eh_last=eh_poll=eh_started=eh_seen=eh_stop_pending=eh_phase=0;
 sg_begin=sg_last=sg_dm_started=0;g_startup_group_go=g_startup_group_release_ms=g_startup_group_pwm_result=0;
 g_dm_startup_go=g_dm_jog_first_fault=g_dm_point_enable=g_dm_jog_request=g_dm_jog_state=0;
 g_dm_can2_motor_state=g_dm_jog_result=g_dm_posvel_holding=0;
 g_eh_first_reason=g_eh_first_command=g_eh_first_exchange=g_eh_first_flags=g_eh_first_tick=g_eh_first_service_gap=g_eh_first_phase=0;
 eh_last_command=eh_last_exchange=eh_last_flags=model_home_tick=model_extension_timeout=0;
 model_ext_enabled=1;model_home_drop_once=model_home_disable=model_keep_disabled=model_drop_late=model_hold_protect=0;
 model_enable_fail=model_position_jump=model_enable_attempts=model_service_injected=0;
 model_settle_case=model_pre_reads=model_speed_interrupt=0;
#ifdef EXTENSION_HOLD_HANDSHAKE_V1
 g_extension_hold_enable_count=g_extension_hold_samples=g_extension_hold_confirmed=g_extension_hold_ready_tick=g_extension_hold_delta_mdeg=0;
 eh_hold_begin=0;eh_hold_position=0;
#endif
}
'''
    unit=unit.replace('#include "vehicle_mission.inc"',group+'\n#include "vehicle_mission.inc"')
    unit=unit.replace('g_vehicle_key=1;VehicleMission_Service();','model_group_reset();g_vehicle_key=1;VehicleMission_Service();')
    unit=unit.replace('if(g_startup_axes_state==1 && test_tick-startup_begin_tick>=500)g_startup_axes_state=4;',
                      'fixture_vm=(g_startup_axes_vm_request || g_vm_test_request) && !g_safety_fault && !g_rtos_fault;\n   StartupAxes_Safety(!g_safety_fault && !g_rtos_fault && !g_can_ctrl_fault_latched);\n   StartupAxes_Service(g_safety_ready,g_can_guard_ready);')
    unit=unit.replace('scenario<7','scenario<23')
    unit=unit.replace('unsigned sent=0,seen_search1=0,seen_search2=0;reset_case(zone);',
                      'unsigned sent=0,seen_search1=0,seen_search2=0,rerun=0;reset_case(zone);')
    unit=unit.replace('(scenario==0 && nominal)', '((scenario==0 || scenario==8 || scenario==9 || scenario==16 || scenario==17 || scenario==19 || scenario==22) && nominal)')
    mark='   test_tick+=10;'
    unit=unit.replace(mark,mark+'''
   model_home_disable=(scenario==9 || scenario==10);model_keep_disabled=scenario==10;
   model_settle_case=scenario;
   model_drop_late=scenario==11 && g_chassis_auto_active;model_hold_protect=scenario==12;
   model_enable_fail=scenario==13;model_position_jump=scenario==14;
   if(scenario==16 && !model_home_tick && !model_enable_attempts)model_ext_enabled=0;
   if(scenario==15 && StartupAxes_Holding() && g_chassis_auto_active && !model_service_injected){test_tick+=151;model_service_injected=1;}
''',1)
    unit=unit.replace('if(scenario==6 && g_vehicle_scan_search_state==1 && searching)fixture_can_failure=1;',
                      'if(scenario==6 && g_vehicle_scan_search_state==1 && searching)fixture_can_failure=1;\n   if(scenario==7 && g_chassis_auto_active)model_extension_timeout=1;')
    unit=unit.replace('   VehicleMission_Service();\n',
                      '   VehicleMission_Service();fixture_vm=(g_startup_axes_vm_request || g_vm_test_request) && !g_safety_fault && !g_rtos_fault;\n')
    # Insert diagnostics/assertion once at final summary, not inside each step.
    mark='  if(scenario==0)ck(fabsf(wx-1200)<=5 && fabsf(wy-300)<=5);'
    unit=unit.replace(mark,'  if(scenario==7)ck(g_vehicle_state==6 && g_vehicle_cause==4 && g_startup_axes_cause==11 && g_eh_first_reason==5 && g_eh_first_exchange==5 && g_eh_first_command==0x3a && !fixture_vm);\n  VehicleMission_Telemetry();test_tick+=500;VehicleMission_Telemetry();\n'+mark)
    # New run in the same boot: model manually returning chassis to start.
    mark='  }\n  ck(!arm_calls && !test_depth'
    restart='''   if(scenario==8 && g_vehicle_state==4 && !rerun){
    memset(plant_pos,0,sizeof(plant_pos));memset(plant_velocity,0,sizeof(plant_velocity));
    memset(plant_target,0,sizeof(plant_target));fixture_heading_yaw=1.0f;sent=0;rerun=1;
    g_vehicle_key=1;VehicleMission_Service();
   }
  }
  ck(!arm_calls && !test_depth'''
    assert mark in unit
    unit=unit.replace(mark,restart)
    mark='  if(scenario==0)ck(starts==1 && !seen_search1 && !seen_search2);'
    unit=unit.replace(mark,mark+'\n  if(scenario==8)ck(g_vehicle_state==4 && !g_vehicle_cause && display_calls==2 && starts==2 && g_vehicle_run_id==2 && s_route_heading_reference.valid && fabsf(s_route_heading_reference.origin_deg-1.0f)<.001f);')
    mark='  if(scenario==0)ck(starts==1 && !seen_search1 && !seen_search2);'
    assertions=r'''
#ifdef EXTENSION_HOLD_HANDSHAKE_V1
  if(scenario==9)ck(g_vehicle_state==4 && !g_vehicle_cause && display_calls==1 && starts==1 && g_extension_hold_confirmed && g_extension_hold_samples==3 && model_enable_attempts==1);
  if(scenario==10)ck(g_vehicle_state==6 && g_vehicle_cause==4 && g_eh_first_reason==3 && !g_extension_hold_confirmed && !starts && model_enable_attempts==1 && !fixture_vm);
  if(scenario==11)ck(g_vehicle_state==6 && g_vehicle_cause==4 && g_eh_first_reason==5 && g_eh_first_flags==2 && g_extension_hold_confirmed && model_enable_attempts==1 && !fixture_vm);
  if(scenario==12)ck(g_vehicle_state==6 && g_eh_first_reason==5 && !g_extension_hold_confirmed && !starts && !fixture_vm);
  if(scenario==13)ck(g_vehicle_state==6 && g_eh_first_reason==4 && g_eh_first_command==0xf3 && model_enable_attempts==1 && !starts && !fixture_vm);
  if(scenario==14)ck(g_vehicle_state==6 && g_eh_first_reason==5 && g_extension_hold_delta_mdeg>2000 && !g_extension_hold_confirmed && !starts && !fixture_vm);
  if(scenario==15)ck(g_vehicle_state==6 && g_eh_first_reason==8 && !fixture_vm);
  if(scenario==16)ck(g_vehicle_state==4 && !g_vehicle_cause && g_extension_hold_confirmed && g_extension_hold_enable_count==1 && model_enable_attempts==2 && display_calls==1);
  if(scenario==17 || scenario==19 || scenario==22)ck(g_vehicle_state==4 && !g_vehicle_cause && g_extension_hold_confirmed && g_extension_hold_samples==3 && model_enable_attempts==1 && display_calls==1 && starts==1);
  if(scenario==18)ck(g_vehicle_state==6 && g_eh_first_reason==3 && !g_extension_hold_confirmed && !starts && !model_enable_attempts && !fixture_vm);
  if(scenario==20)ck(g_vehicle_state==6 && g_eh_first_reason==3 && !g_extension_hold_confirmed && !starts && model_enable_attempts==1 && !fixture_vm);
  if(scenario==21)ck(g_vehicle_state==6 && g_eh_first_reason==4 && g_eh_first_command==0x35 && !g_extension_hold_confirmed && !starts && !fixture_vm);
  if(scenario==19)ck(model_speed_interrupt==1);
  ck(g_extension_hold_enable_count<=1);
  if(g_chassis_auto_active)ck(g_extension_hold_confirmed);
#else
  if(scenario==9)ck(g_vehicle_state==4 && !g_vehicle_cause && display_calls==1);
#endif
'''
    unit=unit.replace(mark,mark+assertions)
    unit=unit.replace('  ck(StartupAxes_Holding());ck(ChassisAuto_BeginSequence());',
                      '  ck(StartupAxes_Holding());\n#ifdef EXTENSION_HOLD_HANDSHAKE_V1\n  ck(g_extension_hold_confirmed && g_extension_hold_samples>=3);\n#endif\n  ck(ChassisAuto_BeginSequence());')
    return unit
