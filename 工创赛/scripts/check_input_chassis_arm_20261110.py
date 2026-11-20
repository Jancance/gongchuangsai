"""Main-F paired actual C owner calls, with explicit external-service stubs."""
import json,re,struct,subprocess,sys

def replay(c,old,texts,pp,opts,label,fixture):
 inp=texts['app_input_runtime.c'];svc=texts['app_chassis_service.c'];blocks=json.loads((c.T/'extracted_blocks.json').read_text('utf8'));f=c.functions(old)
 oldbody='volatile uint32_t g_start_key_count = 0U;\n'+blocks['input_declarations']+f['App_StartGateUpdate']+'\n'+blocks['key_declarations']+f['App_ComboIdlePoll']+'\n'
 for name,key,arg in [('AppInput_ScanDisplay','scan_body','TickType_t screen_start_tick'),('AppInput_KeyStep','key_body','void'),('AppChassis_CanInit','can_init','void'),('AppChassis_CanStep','can_step','void'),('AppChassis_SafetyCheck','safety','void')]:oldbody+='void '+name+'('+arg+'){\n'+blocks[key]+'}\n'
 owned=sorted(set(re.findall(r'\bg_start_\w+|\bs_start_\w+|\bg_scan_\w+|\bg_chassis_key_\w+|\bs_chassis_key_\w+|\bg_hwt_turn_\w+|\bs_hwt_turn_\w+',inp)))
 active=[n for n in owned if (not n.startswith(('g_chassis_key_','s_chassis_key_')) or fixture in ['hwt_turn','chassis_key']) and (not n.startswith(('g_hwt_turn_','s_hwt_turn_')) or fixture=='hwt_turn') and (n!='s_start_gate_revision' or fixture in ['ordinary','competition','uart_static'])]
 names=owned+list(c.functions(inp))+list(c.functions(svc))
 def rename(s):return re.sub(r'\b('+'|'.join(names)+r')\b',lambda m:'old_'+m[0],s)
 includes=''.join(dict.fromkeys(re.findall(r'^#include .*\n',inp+svc,re.M)))+'#include <string.h>\n'
 used=set(re.findall(r'\bg_\w+|\bSAOMA_Data(?:Ready)?',inp+svc));defs={}
 for m in re.finditer(r'^extern\s+([^;]+);','\n'.join(pp),re.M):
  decl=m[1];ns=re.findall(r'\bg_\w+|\bSAOMA_Data(?:Ready)?',decl)
  if ns and any(n in used for n in ns) and not any(n in owned for n in ns):assert '(' not in decl;defs[ns[0]]=decl+';'
 shared=sorted(set(n for d in defs.values() for n in re.findall(r'\bg_\w+|\bSAOMA_Data(?:Ready)?',d)))
 prefix=includes+'\n'+'\n'.join(defs.values())+'\n'
 kinds={'actual':0,'ordinary':1,'competition':2,'heading':3,'stage1':4,'hwt_turn':5,'chassis_key':6,'uart_static':7};kind=kinds[fixture]
 if fixture!='actual':
  macros={'VEHICLE_INTEGRATED_ENABLE':0,'VEHICLE_STAGE1_ENABLE':int(fixture=='stage1'),'APP_IS_COMPETITION':int(fixture=='competition'),'APP_HEADING_SESSION_TEST':int(fixture in ['heading','stage1']),'APP_CHASSIS_KEY_TEST_ENABLE':int(fixture in ['hwt_turn','chassis_key']),'APP_HWT_TURN_TEST':int(fixture=='hwt_turn'),'APP_HWT_STATIC_TEST':int(fixture=='uart_static'),'APP_UART_LINK_TEST':int(fixture=='uart_static')}
  for n,v in macros.items():prefix+='\n#undef '+n+'\n#define '+n+' '+str(v)+'U\n'
 prefix+='#define FIXTURE_KIND '+str(kind)+'U\n'
 stubs=r'''
uint32_t clock_ms=1000U,seed,depth,key_event,key_reads,hold_reads,busy_reads,screen_ok=1U,jog_preparing,jog_other,jog_ready=1U,loopback=1U;
uint32_t *fixture_clock_pointer=&clock_ms;
uint32_t trace[2048],trace_count,events[512],event_count,sent_used,screen_calls,stop_calls,cancel_calls;
char sent[1024];
volatile uint32_t assertions,cases,gate_cases,scan_cases,key_cases,can_cases,safety_cases,legacy_cases;
void event(uint32_t id,uint32_t a,uint32_t b,uint32_t d){if(trace_count+4U<=2048U){trace[trace_count++]=id;trace[trace_count++]=a;trace[trace_count++]=b;trace[trace_count++]=d;}if(event_count<512U)events[event_count++]=id;}
void enter(void){event(100U,depth++,0,0);}void leave(void){event(101U,--depth,0,0);}
#undef taskENTER_CRITICAL
#undef taskEXIT_CRITICAL
#define taskENTER_CRITICAL() enter()
#define taskEXIT_CRITICAL() leave()
TickType_t xTaskGetTickCount(void){event(1U,clock_ms,0,0);return clock_ms;}
uint8_t Key_GetEvent(void){uint32_t result=key_event;event(2U,result,depth,0);key_event=0U;key_reads++;return (uint8_t)result;}
uint8_t App_DebugSendString(const char *s){uint32_t n=strlen(s);event(3U,n,depth,0);if(sent_used+n+1U<sizeof(sent)){memcpy(sent+sent_used,s,n+1U);sent_used+=n+1U;}return (uint8_t)(seed%3U!=0U);}
uint32_t Screen_ShowTaskCode(const char *s){uint32_t i;event(4U,screen_ok,depth,0);for(i=0;s[i];i++)event(40U,(uint8_t)s[i],0,0);screen_calls++;return screen_ok;}
void AppCompetitionEntry_Submit(const char *s,uint32_t ready){uint32_t i;event(5U,ready,depth,0);for(i=0;s[i];i++)event(50U,(uint8_t)s[i],0,0);g_competition_entry.state=COMP_ENTRY_BLOCKED;}
void RadarZoneSwitch_Service(uint32_t now){event(6U,now,depth,0);if(seed&64U)g_radar_zone_ready=1U;}
void RadarLift_Service(uint32_t now,uint32_t ready){event(7U,now,ready,depth);if(seed&128U)g_vm_test_request=1U;}
void RadarServo_DebugService(uint32_t ready){event(8U,ready,depth,0);}
uint32_t RadarServo_KeyTest(uint32_t zone,uint32_t ready){event(9U,zone,ready,depth);return ready;}
void StartupAxes_Service(uint32_t ready,uint32_t guard){event(10U,ready,guard,depth);if(seed&64U)g_startup_axes_state=(seed>>4)&7U;}
uint32_t StartupAxes_Holding(void){uint32_t v=(seed>>(hold_reads++%8U))&1U;event(11U,v,hold_reads,depth);return v;}
void DM_CAN2_ModuleService(uint32_t ready,uint32_t busy){event(12U,ready,busy,depth);}
uint32_t VehicleArm_Busy(void){uint32_t v=(seed>>(busy_reads++%8U))&1U;event(13U,v,busy_reads,depth);return v;}
void VehicleArm_Service(void){event(14U,depth,0,0);if(seed&128U)g_startup_axes_state=4U;}
void App_CombinedTestStep(void){event(15U,depth,0,0);if(seed&32U)g_combined_test.state=(seed>>1)&3U;}
void ChassisAuto_OdometryPoll(void){event(16U,depth,0,0);}
void App_CanFeedbackStep(void){event(17U,depth,0,0);}
void AppCanDiagnostics_Dispatch(void){event(18U,depth,0,0);}
void App_CanPollRefresh(void){uint32_t i;event(19U,depth,0,0);g_can_poll_valid_mask=seed&15U;for(i=0;i<4U;i++)g_can_poll_tick[i]=clock_ms-20U-i;}
void App_CanFeedbackRefresh(void){uint32_t i;event(20U,depth,0,0);g_can_feedback_speed_valid_mask=seed&15U;g_can_feedback_position_valid_mask=(seed>>2)&15U;for(i=0;i<4U;i++){g_can_feedback_speed_tick[i]=clock_ms-50U-i;g_can_feedback_position_tick[i]=clock_ms-100U-i;}if(seed&128U)g_vm_test_request=0U;}
void Safety_CanReport(uint32_t en,uint32_t valid,uint32_t protect,const volatile TickType_t *ticks){uint32_t i;event(21U,en,valid,protect);for(i=0;i<4U;i++)event(210U,ticks[i],0,0);}
void Safety_MotionFeedbackReport(uint32_t en,uint32_t speed,uint32_t pos,const volatile TickType_t *s,const volatile TickType_t *p){uint32_t i;event(22U,en,speed,pos);event(220U,s!=0,p!=0,depth);if(s&&p)for(i=0;i<4U;i++)event(221U,s[i],p[i],0);}
void AppMotion_InitOdometry(void){event(23U,depth,0,0);}
void AppMotion_StepOdometry(void){event(24U,depth,0,0);}
uint32_t CAN1_LoopbackTest(void){event(25U,loopback,depth,0);return loopback;}
void Power_Disable(void){event(26U,depth,0,0);}
void App_SerialMotionSafety(void){event(27U,depth,0,0);if(seed&16U)g_vm_test_request=0U;}
void StartupAxes_Safety(uint32_t healthy){event(28U,healthy,depth,0);if(seed&64U){g_startup_axes_state=5U;g_combined_test.state=1U;}}
void MotionStop(void){event(29U,depth,0,0);stop_calls++;g_vm_test_request=0U;}
void App_CombinedTestSafety(void){event(30U,depth,0,0);if(seed&32U)g_vm_test_request=0U;}
void Safety_Update(uint32_t vm,uint32_t can){event(31U,vm,can,depth);}
void StartupAxes_Cancel(void){event(32U,depth,0,0);cancel_calls++;}
void ChassisAuto_Abort(void){event(33U,depth,0,0);g_chassis_auto_active=0U;}
const uint32_t *AppCanDiagnostics_JogPreparing(void){event(34U,jog_preparing,depth,0);return &jog_preparing;}
uint32_t App_CanJogOtherRequest(void){event(35U,jog_other,depth,0);return jog_other;}
uint32_t App_CanJogReady(void){event(36U,jog_ready,depth,0);return jog_ready;}
void AppDebug_HeadingObserve(void){event(37U,depth,0,0);}
#define CHECK(x) do{assertions++;if(!(x))return __LINE__;}while(0)
'''
 newbody=re.sub(r'^#include .*\n','',inp+svc,flags=re.M)
 harness=prefix+stubs+newbody+'\n'+rename(oldbody)+'\n'
 harness+='uint32_t equal(void){\n'
 for n in active:harness+=f'CHECK(memcmp((void*)&{n},(void*)&old_{n},sizeof({n}))==0);\n'
 harness+='return 0U;}\nvoid sync(void){\n'
 for n in active:harness+=f'memcpy((void*)&old_{n},(void*)&{n},sizeof({n}));\n'
 harness+='}\nvoid clear_owned(void){\n'
 for n in active:harness+=f'memset((void*)&{n},0,sizeof({n}));\n'
 harness+='}\nvoid clear_external(void){\n'
 for n in shared:harness+=f'memset((void*)&{n},0,sizeof({n}));\n'
 harness+='}\n'
 env=shared+['clock_ms','seed','key_event','key_reads','hold_reads','busy_reads','screen_ok','jog_preparing','jog_other','jog_ready','loopback','screen_calls','stop_calls','cancel_calls']
 harness+='uint8_t env_before[8192],env_after[8192],env_current[8192];\nvoid save_env(uint8_t *b){uint32_t i=0U;memset(b,0,8192U);\n'
 for n in env:harness+=f'memcpy(b+i,(void*)&{n},sizeof({n}));i+=sizeof({n});\n'
 harness+='}\nvoid load_env(const uint8_t *b){uint32_t i=0U;\n'
 for n in env:harness+=f'memcpy((void*)&{n},b+i,sizeof({n}));i+=sizeof({n});\n'
 harness+='}\n'
 harness+=r'''
uint32_t saved_trace[2048],saved_count,saved_sent_used;char saved_sent[1024];
void clear_trace(void){trace_count=event_count=depth=sent_used=0U;memset(sent,0,sizeof(sent));}
uint32_t has(uint32_t id){uint32_t i;for(i=0;i<event_count;i++)if(events[i]==id)return 1U;return 0U;}
uint32_t index_of(uint32_t id){uint32_t i;for(i=0;i<event_count;i++)if(events[i]==id)return i;return 0xFFFFFFFFU;}
void invoke(uint32_t old,uint32_t op){
 if(op==0U){if(old)old_App_StartGateUpdate();else App_StartGateUpdate();}
 if(op==1U){if(old)old_AppInput_ScanDisplay(1000U);else AppInput_ScanDisplay(1000U);}
 if(op==2U){if(old)old_AppInput_KeyStep();else AppInput_KeyStep();}
 if(op==3U){if(old)old_AppChassis_CanInit();else AppChassis_CanInit();}
 if(op==4U){if(old)old_AppChassis_CanStep();else AppChassis_CanStep();}
 if(op==5U){if(old)old_AppChassis_SafetyCheck();else AppChassis_SafetyCheck();Safety_Update(g_vm_test_request,g_can_test_result);}
#if APP_CHASSIS_KEY_TEST_ENABLE && !APP_HWT_STATIC_TEST
 if(op==6U){if(old)old_App_ChassisKeyStep(key_event);else App_ChassisKeyStep(key_event);}
#if APP_HWT_TURN_TEST
 if(op==7U){if(old)old_App_HwtTurnStep(g_hwt_turn_request,key_event);else App_HwtTurnStep(g_hwt_turn_request,key_event);}
#endif
#endif
}
uint32_t pair(uint32_t op){sync();save_env(env_before);clear_trace();invoke(0U,op);CHECK(depth==0U);saved_count=trace_count;memcpy(saved_trace,trace,trace_count*4U);saved_sent_used=sent_used;memcpy(saved_sent,sent,sent_used);save_env(env_after);
 load_env(env_before);clear_trace();invoke(1U,op);CHECK(depth==0U);CHECK(saved_count==trace_count);CHECK(!memcmp(saved_trace,trace,trace_count*4U));CHECK(saved_sent_used==sent_used);CHECK(!memcmp(saved_sent,sent,sent_used));save_env(env_current);CHECK(!memcmp(env_after,env_current,8192U));CHECK(equal()==0U);cases++;return 0U;}
void environment(uint32_t n){uint32_t i;clear_external();clear_owned();seed=n;clock_ms=3000U;key_event=1U;key_reads=hold_reads=busy_reads=screen_calls=stop_calls=cancel_calls=0U;screen_ok=jog_ready=loopback=1U;jog_preparing=jog_other=0U;
 g_safety_ready=1U;g_can_test_result=1U;g_can_guard_ready=1U;g_can_poll_enable=1U;g_can_feedback_enable=1U;g_vm_test_request=1U;
 g_hwt_version_result=1U;g_hwt_valid_mask=3U;g_hwt_update_tick=clock_ms;
 g_saoma_last_result=1U;g_saoma_accept_count=7U;g_scan_display_result=SCREEN_OK;strcpy(SAOMA_Data,"123+321");SAOMA_DataReady=1U;
 for(i=0;i<4U;i++){g_can_poll_tick[i]=10U+i;g_can_feedback_speed_tick[i]=20U+i;g_can_feedback_position_tick[i]=30U+i;}
}
__attribute__((section("i.test_entry"))) uint32_t test_entry(void){uint32_t n;
 CHECK(clock_ms==1000U&&fixture_clock_pointer==&clock_ms&&screen_ok==1U&&jog_ready==1U&&loopback==1U);CHECK(cases==0U&&trace_count==0U&&key_reads==0U);CHECK(equal()==0U);
 /* Gate accepts only the same scan revision and consumes pending requests. */
 for(n=0U;n<64U;n++){
  environment(n);g_rtos_fault=n&1U;g_safety_fault=(n>>1)&1U;g_safety_ready=(n&4U)?0U:1U;g_saoma_last_result=(n&8U)?0U:1U;g_scan_display_result=(n&16U)?SCREEN_TIMEOUT:SCREEN_OK;
  s_start_pending=1U;s_start_pending_revision=(n&32U)?6U:7U;CHECK(pair(0U)==0U);CHECK(s_start_pending==0U);
  if(n==0U){CHECK(g_start_accept_count==1U&&g_start_request==1U&&!strcmp(g_start_task_code,"123+321"));CHECK(!strcmp(sent,"START:123+321\r\n"));}
  else CHECK(g_start_accept_count==0U&&g_start_deny_count==1U);gate_cases++;
 }
 environment(0U);g_start_request=1U;CHECK(pair(0U)==0U);CHECK(g_start_gate_ready==0U&&g_start_request==1U);gate_cases++;
 environment(0U);g_start_request=1U;g_rtos_fault=1U;CHECK(pair(0U)==0U);CHECK(g_start_request==0U);gate_cases++;
 for(n=0U;n<16U;n++){
  environment(n);clock_ms=(n&1U)?2000U:1999U;g_start_request=(n>>1)&1U;SAOMA_DataReady=(n>>2)&1U;screen_ok=(n&8U)?SCREEN_TIMEOUT:SCREEN_OK;CHECK(pair(1U)==0U);
  CHECK(screen_calls==(((n&7U)==5U)?1U:0U));CHECK(g_scan_display_count==(((n&15U)==5U)?1U:0U));scan_cases++;
 }
 /* Dynamic stub effects exercise repeat reads, short-circuit order and actual
    service data passed after refresh, rather than caching the first snapshot. */
 for(n=0U;n<256U;n++){
  environment(n);g_can_test_result=(n&1U)?0U:1U;g_startup_axes_state=(n>>1)&7U;g_vehicle_arm_stop_pending=(n>>4)&1U;g_chassis_auto_active=(n>>5)&1U;g_combined_test.state=(n>>6)&3U;
  g_motion_diag_hold_report=(n>>3)&1U;g_motion_diag_hold_stop_service=1U;g_motion_diag_fail_stop_once=1U;
  if(n&8U)g_can_feedback_enable=0U;if(n&16U)g_can_poll_enable=0U;if(n&32U)g_rtos_fault=1U;
  CHECK(pair(4U)==0U);CHECK(depth==0U);
  if(APP_UART_LINK_TEST)CHECK(g_vm_test_request==0U&&has(26U)&&has(21U)&&has(22U)&&!has(10U));
  else {if(g_can_test_result==0U)CHECK(!has(19U)&&!has(20U)&&has(21U)&&has(22U));if(has(10U)&&has(14U))CHECK(index_of(10U)<index_of(14U));if(has(19U))CHECK(index_of(19U)<index_of(20U)&&index_of(20U)<index_of(21U));}
  if(APP_POSITION_DIAGNOSTIC&&APP_HEADING_SESSION_TEST)CHECK(events[event_count-1U]==24U);can_cases++;
 }
 /* Exact healthy mechanical order: holding checks are deliberately repeated. */
 environment(255U);g_vehicle_arm_stop_pending=1U;g_startup_axes_state=4U;g_chassis_auto_active=0U;CHECK(pair(4U)==0U);
 if(APP_HEADING_SESSION_TEST&&!APP_UART_LINK_TEST){CHECK(index_of(10U)<index_of(12U)&&index_of(12U)<index_of(14U));CHECK(hold_reads>=3U&&busy_reads==2U);}can_cases++;
 for(n=0U;n<2U;n++){environment(n);loopback=n;CHECK(pair(3U)==0U);CHECK(g_can_test_result==n&&has(25U));if(APP_POSITION_DIAGNOSTIC&&APP_HEADING_SESSION_TEST)CHECK(events[0]==23U&&events[1]==25U);can_cases++;}
 for(n=0U;n<128U;n++){
  environment(n);g_rtos_fault=n&1U;g_safety_fault=(n>>1)&1U;g_can_ctrl_fault_latched=(n>>2)&1U;g_startup_axes_state=(n&8U)?5U:0U;g_combined_test.state=(n>>4)&3U;
  CHECK(pair(5U)==0U);CHECK(events[event_count-1U]==31U);if(has(29U))CHECK(index_of(28U)<index_of(29U)&&index_of(29U)<index_of(30U));safety_cases++;
 }
 /* Each key branch consumes at most one event, including service mutations. */
 for(n=0U;n<128U;n++){
  environment(n);key_event=n&1U;g_vm_test_request=0U;g_can_feedback_enable=0U;g_stage1_state=(n>>1)&7U;g_radar_servo_key_test_enable=(n>>4)&1U;g_radar_zone_selected=(n>>5)&1U;g_start_gate_ready=(n>>2)&1U;
  CHECK(pair(2U)==0U);CHECK(key_reads==1U&&key_event==0U);CHECK(events[0]==1U&&events[1]==6U&&index_of(6U)<index_of(7U)&&index_of(7U)<index_of(8U)&&index_of(8U)<index_of(2U));
  if(VEHICLE_INTEGRATED_ENABLE)CHECK(g_vehicle_key==(n&1U));
  else if(VEHICLE_STAGE1_ENABLE&&(n&1U)){CHECK(g_stage1_key_request==((g_stage1_state==1U||g_stage1_state==2U||g_stage1_state==6U||g_stage1_state==7U)?2U:1U));CHECK(cancel_calls==(g_stage1_state==7U?1U:0U));}
  key_cases++;
 }
#if FIXTURE_KIND==1U || FIXTURE_KIND==2U
 environment(0U);g_vm_test_request=0U;g_can_feedback_enable=0U;CHECK(pair(0U)==0U);CHECK(s_start_gate_revision==7U);CHECK(pair(2U)==0U);CHECK(s_start_pending==1U&&s_start_pending_revision==7U);g_saoma_accept_count=8U;CHECK(pair(0U)==0U);CHECK(!g_start_accept_count&&g_start_deny_count==1U&&!s_start_pending);legacy_cases++;
 environment(0U);g_vm_test_request=0U;g_can_feedback_enable=0U;CHECK(pair(0U)==0U);CHECK(pair(2U)==0U);CHECK(pair(0U)==0U);CHECK(g_start_accept_count==1U&&g_start_request==1U);legacy_cases++;
#endif
#if APP_CHASSIS_KEY_TEST_ENABLE && !APP_HWT_STATIC_TEST
 /* Direct legacy key state checks; never enable these profiles in firmware. */
 for(n=0U;n<16U;n++){
  environment(0U);g_vm_test_request=0U;g_can_feedback_enable=0U;key_event=1U;
  switch(n){case 1:g_vm_test_request=1U;break;case 2:g_rtos_fault=1U;break;case 3:g_safety_fault=1U;break;case 4:g_chassis_auto_active=1U;break;case 5:g_chassis_auto_request=1U;break;case 6:g_chassis_auto_cut_vm=1U;break;case 7:g_wheel_test_active=1U;break;case 8:g_wheel_test_cut_vm=1U;break;case 9:jog_preparing=1U;break;case 10:g_can_jog_request=1U;break;case 11:jog_other=1U;break;case 12:g_can_feedback_enable=1U;break;case 13:g_motion_diag_fault_latched=1U;break;case 14:g_motion_auto_stop_state=1U;break;case 15:g_motion_auto_stop_cut_vm=1U;break;}
  CHECK(pair(6U)==0U);CHECK(g_chassis_key_state==(n?5U:2U));CHECK(g_chassis_key_cause==(n?2U:0U));legacy_cases++;
 }
 environment(0U);g_vm_test_request=0U;g_can_feedback_enable=0U;key_event=1U;CHECK(pair(6U)==0U);key_event=0U;CHECK(pair(6U)==0U);CHECK(g_chassis_key_state==3U&&g_chassis_auto_request==1U&&g_chassis_auto_dry_run==0U);
 g_chassis_auto_request=0U;g_chassis_auto_result=1U;CHECK(pair(6U)==0U);CHECK(g_chassis_key_state==4U&&g_vm_test_request==0U);legacy_cases++;
 environment(0U);g_chassis_key_state=2U;g_can_feedback_enable=0U;s_chassis_key_tick=clock_ms-APP_CHASSIS_KEY_READY_MS;key_event=0U;CHECK(pair(6U)==0U);CHECK(g_chassis_key_cause==3U);legacy_cases++;
 environment(0U);g_chassis_key_state=3U;g_can_feedback_enable=0U;s_chassis_key_tick=clock_ms-50000U;key_event=0U;CHECK(pair(6U)==0U);CHECK(g_chassis_key_cause==4U);legacy_cases++;
 environment(0U);g_chassis_key_state=3U;g_can_feedback_enable=0U;key_event=1U;CHECK(pair(6U)==0U);CHECK(g_chassis_key_cause==1U);legacy_cases++;
#if APP_HWT_TURN_TEST
 environment(0U);g_vm_test_request=0U;g_can_feedback_enable=0U;key_event=0U;g_hwt_turn_request=1U;g_hwt_yaw_deg=179.0f;CHECK(pair(7U)==0U);CHECK(g_chassis_key_state==2U);
 g_hwt_turn_request=0U;g_hwt_angle_count=1U;g_hwt_yaw_deg=-179.0f;CHECK(pair(7U)==0U);CHECK(g_hwt_turn_yaw_delta==2.0f&&g_chassis_key_state==3U);g_hwt_valid_mask=0U;CHECK(pair(5U)==0U);CHECK(g_chassis_key_cause==5U&&g_vm_test_request==0U);legacy_cases++;
#endif
#endif
 CHECK(gate_cases==66U&&scan_cases==16U&&key_cases==128U&&can_cases==259U&&safety_cases==128U);CHECK(legacy_cases<=32U);return 0U;
}
'''
 out=c.T/('unit_'+label);out.mkdir(exist_ok=True);(out/'unit.c').write_bytes(harness.encode('gbk'))
 (out/'unit.sct').write_text('LR_ROM 0x08000000 0x100000 { ER_ROM 0x08000000 0x100000 { unit.o (i.test_entry, +First) *(+RO) } RW_RAM 0x20000000 0x20000 { *(+RW,+ZI) } }\n')
 simple=[x for x in opts if x not in ['--cpu','Cortex-M4.fp.sp']];commands=[[str(c.B/'armcc.exe'),'--cpu','Cortex-M3','--gnu']+simple+['--split_sections','-O1','-c','unit.c','-o','unit.o'],[str(c.B/'armlink.exe'),'--entry','test_entry','--scatter','unit.sct','unit.o','--library_type=microlib','-o','unit.axf']]
 for cmd in commands:
  run=subprocess.run(cmd,cwd=out,capture_output=True,timeout=60);assert run.returncode==0,(run.stdout+run.stderr).decode(errors='replace')
 (out/'commands.json').write_text(json.dumps(commands,indent=2),'utf8')
 sys.path.insert(0,str(c.R/'工创赛/tmp/position_alignment_deps'));import unicorn as uc
 from unicorn import arm_const as reg
 blob=(out/'unit.axf').read_bytes();elf=struct.unpack_from('<16sHHIIIIIHHHHHH',blob);m=uc.Uc(uc.UC_ARCH_ARM,uc.UC_MODE_THUMB|uc.UC_MODE_MCLASS);m.mem_map(0x08000000,0x100000);m.mem_map(0x20000000,0x20000);m.mem_write(0x20000000,b'\xa5'*0x20000)
 initialized=zeroed=nonzero=0
 for i in range(elf[12]):
  _,kind,flags,addr,off,size,_,_,_,_=struct.unpack_from('<IIIIIIIIII',blob,elf[6]+i*elf[11])
  if flags&2 and size:
   if kind==1:
    data=blob[off:off+size];m.mem_write(addr,data)
    if flags&1:initialized+=size;nonzero+=sum(bool(b) for b in data)
   elif kind==8:m.mem_write(addr,b'\x00'*size);zeroed+=size
 symbols=subprocess.check_output([str(c.B/'fromelf.exe'),'--text','-s','unit.axf'],cwd=out).decode(errors='replace');(out/'symbols.txt').write_text(symbols,'utf8')
 def address(n):return int(re.search(r'\b'+n+r'\s+(0x[0-9a-fA-F]+)',symbols)[1],16)
 def value(n):return struct.unpack('<I',m.mem_read(address(n),4))[0]
 initial={n:value(n) for n in ['clock_ms','screen_ok','jog_ready','loopback','fixture_clock_pointer','cases','trace_count','key_reads','g_start_request','old_g_start_request']};assert initial['clock_ms']==1000 and initial['screen_ok']==initial['jog_ready']==initial['loopback']==1;assert initial['fixture_clock_pointer']==address('clock_ms');assert all(initial[n]==0 for n in ['cases','trace_count','key_reads','g_start_request','old_g_start_request'])
 owner_initial_bytes=0
 for name in active+[('old_'+n) for n in active]:
  match=re.search(r'\b'+name+r'\s+(0x[0-9a-fA-F]+)\s+\w+\s+\d+\s+Data\s+\w+\s+(0x[0-9a-fA-F]+)',symbols);assert match,name
  size=int(match[2],16);assert bytes(m.mem_read(int(match[1],16),size))==b'\x00'*size,('original initializer',name);owner_initial_bytes+=size
 stop=0x080FF000;m.reg_write(reg.UC_ARM_REG_SP,0x2001FFF0);m.reg_write(reg.UC_ARM_REG_LR,stop|1);m.emu_start(elf[4]|1,stop,timeout=60000000,count=300000000);assert m.reg_read(reg.UC_ARM_REG_PC)==stop
 result=m.reg_read(reg.UC_ARM_REG_R0)
 if result:print('Failure',label,'line',result,harness.splitlines()[result-1],{n:value(n) for n in ['cases','assertions','gate_cases','scan_cases','can_cases','safety_cases','key_cases']},flush=True)
 assert result==0,f'C check line {result}'
 counts={n:value(n) for n in ['cases','assertions','gate_cases','scan_cases','can_cases','safety_cases','key_cases','legacy_cases']}
 report=dict(**counts,profile=fixture,initialized_rw_bytes=initialized,nonzero_initialized_bytes=nonzero,zeroed_nobits_bytes=zeroed,original_new_owner_initial_symbols=2*len(active),original_new_owner_initial_bytes=owner_initial_bytes,initial_values=initial,loader='SHF_ALLOC PROGBITS at execution addresses; NOBITS explicitly zeroed after poisoning RAM; test sentinel/pointer and every active original/new owner zero initializer checked',artifact_sha256={str(p.relative_to(c.R)).replace('\\','/'):c.sha(p) for p in sorted(out.iterdir()) if p.is_file() and p.name!='result.json'})
 (out/'result.json').write_text(json.dumps(report,ensure_ascii=False,indent=2),'utf8');return report
