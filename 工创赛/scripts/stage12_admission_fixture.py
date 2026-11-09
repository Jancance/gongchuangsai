"""Extract actual main.c admission functions; device and sensor values remain models."""
def function(source,name):
    lo=source.index('static uint32_t '+name+'(');i=source.index('{',lo)+1;depth=1
    while depth:
        depth+=(source[i]=='{')-(source[i]=='}');i+=1
    return source[lo:i]+'\n'

def inject(prefix,root):
    main=(root/'CHASSIS/USER/main.c').read_bytes()
    try:main=main.decode('utf-8')
    except UnicodeDecodeError:main=main.decode('gbk')
    model=r'''
#define HS_SOURCE_MS 100U
typedef struct {uint32_t healthy,sample_ms,valid_mask,angle_ms,rate_ms;float rate_dps;} AppHeadingSource;
static AppHeadingSource s_heading_source;
static uint32_t s_heading_source_ready=1,g_hwt_cal_active,g_hwt_cal_request,g_hwt_cal_end_tx_result,g_hwt_version_state,g_hwt_version_request,g_hwt_a7_request;
static uint32_t g_can_jog_request,s_can_jog_preparing,g_can_feedback_enable,g_can_poll_enable=1,g_can_test_result=1;
static uint32_t g_motion_diag_hold_report,g_motion_diag_hold_stop_service,g_motion_diag_fail_stop_once;
static uint32_t g_translation_hold_selftest=1,s_serial_proto_ok=1;
static uint32_t model_gate_started,model_gate_until,model_gate_injected;
uint32_t CAN1_EmmSettingsConfirmed(void){return 1;}
static uint32_t App_CanJogOtherRequest(void){return 0;}
#ifndef MOTION_STOP_IDLE
#define MOTION_STOP_IDLE 0U
#endif
static uint32_t App_HeadingFrameAge(const AppHeadingSource *s,uint32_t now,uint32_t bit)
{return (s->valid_mask&bit)?now-((bit==1)?s->angle_ms:s->rate_ms):0xffffffffU;}
'''
    model+=function(main,'App_HeadingRuntimeExclusive')+function(main,'App_HeadingRuntimeSource')
    if 'static uint32_t App_SerialIdleBlocked(' in main:model+=function(main,'App_SerialIdleBlocked')
    model+=function(main,'App_SerialIdleReady')
    old='static uint32_t App_SerialIdleReady(uint32_t now){(void)now;return !g_combined_test.state && !g_chassis_auto_active && !g_chassis_auto_result && g_safety_ready && !g_rtos_fault && !g_safety_fault && !g_can_ctrl_fault_latched;}'
    assert old in prefix
    return prefix.replace(old,model)

def extend(unit):
    unit=unit.replace('model_group_reset();g_vehicle_key=1;VehicleMission_Service();',
        'model_group_reset();g_can_feedback_enable=0;s_heading_source_ready=1;\n s_heading_source=(AppHeadingSource){1,test_tick,3,test_tick,test_tick,0};g_vehicle_key=1;VehicleMission_Service();')
    unit=unit.replace('scenario<20','scenario<30')
    unit=unit.replace('stop_sent=0;\n  for(','''stop_sent=0;model_gate_started=model_gate_until=model_gate_injected=0;
  g_can_feedback_enable=0;s_heading_source_ready=1;
  s_heading_source=(AppHeadingSource){1,test_tick,3,test_tick,test_tick,0};
  for(''')
    unit=unit.replace('test_tick+=1;', '''test_tick+=1;
   s_heading_source.healthy=1;s_heading_source.valid_mask=3;s_heading_source.rate_dps=0;
   s_heading_source.sample_ms=s_heading_source.angle_ms=s_heading_source.rate_ms=test_tick;
''')
    unit=unit.replace('scenario>=7)', 'scenario>=7 && scenario!=28)')
    unit=unit.replace('(scenario==1 && g_vehicle_scan_search_state==1', '((scenario==1 || scenario==28) && g_vehicle_scan_search_state==1')
    marker='   VehicleMission_Service();fixture_vm='
    assert marker in unit
    block=r'''
   if(scenario>=20 && stopped && !model_gate_injected && (scenario!=28 || (sent && g_vehicle_scan_search_state==1))){
    model_gate_started=test_tick;model_gate_injected=1;
    model_gate_until=test_tick+((scenario==20 || scenario==28)?10:scenario==21?700:scenario==23?100:scenario==27?999:2000);
   }
   if(model_gate_injected && test_tick<model_gate_until){
    if(scenario==23)s_heading_source.rate_dps=4;
    else if(scenario==24)g_can_feedback_enable=1;
    else{
     uint32_t age=scenario==29?99:100;
     s_heading_source.angle_ms=test_tick-(age-1);s_heading_source.rate_ms=test_tick-age;
     s_heading_source.sample_ms=test_tick-94;
    }
   }
#ifdef VEHICLE_QUEUE_ADMISSION_TIMEOUT_MS
   if(g_vehicle_queue_state==1 && scenario==25)g_safety_fault=1;
   if(g_vehicle_queue_state==1 && scenario==26)g_vehicle_key=1;
#endif
'''
    unit=unit.replace(marker,block+marker)
    # Cancellation/global errors must not be hidden by a pending queue.
    marker='   if(g_combined_test.request)stop_sent=0;'
    unit=unit.replace(marker,marker+r'''
#ifdef VEHICLE_QUEUE_ADMISSION_TIMEOUT_MS
   if(g_vehicle_queue_state==1)ck(!g_combined_test.request && !g_vm_test_request && !g_chassis_auto_active && fixture_vm);
#endif
''')
    unit=unit.replace('scenario==19){\n   float wx=', 'scenario==19 || scenario==20 || scenario==21 || scenario==23 || scenario==27 || scenario==28 || scenario==29){\n   float wx=')
    marker='  printf("CHAIN input%u'
    assert marker in unit
    assertions=r'''
#ifdef VEHICLE_QUEUE_ADMISSION_TIMEOUT_MS
  if(scenario==20 || scenario==21 || scenario==27 || scenario==28)ck(g_vehicle_state==4 && g_vehicle_queue_wait_ms>0 && g_vehicle_queue_wait_ms<1000);
  if(scenario==22)ck(g_vehicle_state==6 && g_vehicle_cause==7 && g_vehicle_queue_state==3 && g_vehicle_queue_first_mask==32 && g_vehicle_queue_wait_ms>=1000 && !second_stage_seen);
  if(scenario==23)ck(g_vehicle_state==4 && g_vehicle_queue_first_mask==256 && g_vehicle_queue_wait_ms>=100);
  if(scenario==24)ck(g_vehicle_state==6 && g_vehicle_cause==7 && g_vehicle_queue_first_mask==64 && !second_stage_seen);
  if(scenario==25)ck(g_vehicle_state==6 && g_vehicle_cause==4 && !second_stage_seen);
  if(scenario==26)ck(g_vehicle_state==7 && g_vehicle_cause==2 && !second_stage_seen);
  if(scenario==29)ck(g_vehicle_state==4 && !g_vehicle_queue_first_mask && !g_vehicle_queue_wait_ms);
#endif
'''
    unit=unit.replace(marker,assertions+marker)
    return unit
