"""C2c narrow actual-C differential replay; hardware/RTOS/plan helpers are stubs."""
from pathlib import Path
import re, struct, subprocess, sys

def replay(c, old, texts, opts):
    oldfn=c.functions(old); newfn={k:v for s in texts for k,v in c.functions(s).items()}
    names=['App_MotionPlanValid','App_DebugSelectPlan','App_RuntimeRouteInstall',
           'App_CombinedTestStop','App_CombinedTestSafety','App_CombinedTestStep','App_RuntimeRouteEditable']
    # Exact old declarations, omitting only the type now supplied by the public header.
    lines=old.splitlines(True)
    decl=''.join(lines[181:192]+lines[199:208]+lines[228:233]+lines[234:252]+lines[259:263]+lines[267:274])
    decl=re.sub(r'^#(?:if|endif).*\n','',decl,flags=re.M)
    serial='static uint32_t s_serial_token_seq,s_serial_token_tick,s_serial_arm_ms,s_serial_keepalive_ms;\n'
    owned=sorted(set(re.findall(r'\b(?:s_\w+|g_combined_test|g_motion_plan_valid|g_debug_plan_id|g_route_heading_targets|g_route_heading_start_yaw)\b',decl)))
    oldnames=owned+names+['AppMotion_LoadDefaultPlan']+re.findall(r's_serial_\w+',serial)+re.findall(r'^#define (\w+)',decl,re.M)
    def rename(s): return re.sub(r'\b('+'|'.join(oldnames)+r')\b',lambda m:'old_'+m[0],s)
    # New owner declarations use the same initializers; ownership is checked separately.
    prefix=''.join(x for x in re.findall(r'^#include .*\n',texts[1],re.M) if '.inc' not in x)+'\n#include <string.h>\n'
    prefix+=decl+serial+'\n'+rename(decl+serial)
    external=['g_vm_test_request','g_can_ctrl_fault_latched','g_safety_ready','g_can_guard_ready',
              'g_heading_exec_request','g_heading_exec_phase','g_heading_exec_state','g_heading_exec_trip',
              'g_translation_hold_selftest','g_stage1_stop_requested','g_stage1_motion_skipped',
              'g_chassis_auto_request','g_can_jog_request','g_radar_servo_departure_done',
              'g_serial_motion_state','g_serial_motion_result','g_serial_motion_link_lost']
    prefix+='\nvolatile uint32_t '+','.join(external)+';\nvolatile float g_heading_exec_error_deg;\nvolatile ForwardHoldDebug g_forward_hold;\n'
    # Capture real const live wiring and real narrow bridges, not test substitutes.
    obs=texts[1][texts[1].index('static const AppMotionObservation s_motion_observation'):texts[1].index('\n#endif',texts[1].index('static const AppMotionObservation s_motion_observation'))]
    route=texts[2][texts[2].index('const RuntimeRoute *AppRuntimeRoute_Live'):texts[2].rindex('#endif')]
    maintext=texts[0]
    bridges='\n'.join(newfn[n] for n in ['AppSerial_ResetToken','AppSerial_SetKeepalive'])
    prefix+=obs+'\n'+route+'\n'+bridges+'\n'+newfn['AppMotion_SetStartPermit']+'\n'
    stubs=r'''
uint32_t side,clock_ms,trace[512],trace_count,depth,io_ok,editable,load_ok,health_ok,exclusive_ok,backend_rearm;
uint32_t load_calls,install_calls,rearm_calls,permit_accepts,permit_rejects,stop_calls,live_checks,select_rejects,route_rejects;
ChassisAutoStatus backend;PowerCommandStatus power;AppHeadingSource heading;
void event(uint32_t x){if(trace_count<512U)trace[trace_count++]=x;}
void enter(void){event(100U+depth++);}void leave(void){event(200U+--depth);}
#undef taskENTER_CRITICAL
#undef taskEXIT_CRITICAL
#define taskENTER_CRITICAL() enter()
#define taskEXIT_CRITICAL() leave()
TickType_t xTaskGetTickCount(void){event(1U);return clock_ms;}
uint32_t AppDebugPlan_Load(uint32_t id,MotionAction *actions,uint32_t *count){uint32_t i;event(2U);event(id);load_calls++;if(!load_ok)return 0U;*count=2U;for(i=0;i<2U;i++)actions[i]=(MotionAction){MOTION_FORWARD,5U,1000U,0.0f,0.0f,0.0f};return 1U;}
uint32_t ChassisAuto_ReadStatus(ChassisAutoStatus *s){event(3U);*s=backend;return 1U;}
void Power_ReadCommandStatus(PowerCommandStatus *s){event(4U);*s=power;}
void Power_Disable(void){event(5U);stop_calls++;}
uint32_t ChassisAuto_RearmCompleted(uint32_t s){event(6U);event(s);rearm_calls++;return backend_rearm;}
void AppHeadingRuntime_ResetSession(void){event(7U);}
void AppOdometry_ResetRecords(void){event(8U);}
void RuntimeRoute_Init(RuntimeRoute *r){memset(r,0,sizeof(*r));}
void RuntimeRoute_Install(RuntimeRoute *r,const RuntimeRoutePrepared *p){event(9U);event(p->id);install_calls++;r->committed=*p;r->ready=1U;r->consumed=0U;}
void App_HeadingRuntimeTrip(uint32_t c){event(10U);event(c);g_heading_exec_trip=c;}
void App_ForwardHoldTrip(uint32_t c){event(11U);event(c);g_forward_hold.trip=c;}
void ChassisAuto_Abort(void){event(12U);}
uint32_t App_HeadingRuntimeSource(uint32_t n){event(13U);event(n);return health_ok;}
uint32_t App_HeadingRuntimeExclusive(void){event(14U);return exclusive_ok;}
uint32_t App_SerialIdleReady(uint32_t n){event(15U);event(n);return editable;}
uint32_t App_SerialCompletedStopped(void){event(16U);return g_combined_test.state==3U;}
uint32_t App_HeadingSourceRead(AppHeadingSource *h){event(17U);*h=heading;return 1U;}
uint32_t AppCompetitionEntry_MotionAllowed(void){event(18U);return 1U;}
uint32_t StartupAxes_Holding(void){event(19U);return io_ok;}
uint32_t MotionGetState(void){event(21U);return MOTION_IDLE;}
uint32_t MotionFinish(void){event(22U);return 1U;}
uint32_t App_CombinedLoadAction(uint32_t i){event(23U);event(i);permit_accepts++;return io_ok;}
void App_HeadingRuntimeStep(const AppHeadingRuntimeInputs *i){event(24U);event(*i->keep_vm);}
void App_ForwardHoldStep(const AppTranslationRuntimeInputs *i){event(25U);event(*i->mode);}
void App_HeadingRuntimeSafety(const AppHeadingRuntimeInputs *i){event(26U);event(*i->keep_vm);}
void App_ForwardHoldSafety(const AppTranslationRuntimeInputs *i){event(27U);event(*i->mode);}
uint32_t App_HeadingRuntimeOwns(const ChassisAutoStatus *s){event(28U);return io_ok;}
uint32_t App_ForwardHoldOwns(const ChassisAutoStatus *s,const AppTranslationRuntimeInputs *i){event(29U);return io_ok;}
uint32_t RadarServo_OnDeparture(uint32_t z){event(30U);event(z);return io_ok;}
uint32_t RadarZoneSwitch_Lock(void){event(31U);return 1U;}
void App_TimingRecordCompleted(uint32_t i,uint32_t s,uint32_t n){event(32U);event(i);event(s);event(n);}
uint32_t Chassis_DistanceSelfTest(void){return 1U;}
uint32_t Chassis_CalibrateDistanceMm(uint32_t mode,float mm,float *out){(void)mode;*out=mm;return 1U;}
const ChassisGeometry g_chassis_geometry={77.0f,212.0f,206.0f};
'''
    # Match the real RouteHeadingReference member name from its header.
    stubs=stubs.replace('r->initial_yaw_deg=yaw;','(void)yaw;')
    # CompletedStopped is a boundary stub, but must read the appropriate old/new state.
    stubs=stubs.replace('return g_combined_test.state==3U;','return (side?old_g_combined_test.state:g_combined_test.state)==3U;')
    # Functions used mutually need declarations before their exact source bodies.
    prototypes='\n'.join((re.sub(r'^static\s+','',oldfn[n][:oldfn[n].index('{')]).strip()+';') for n in names)
    bodies='\n'.join(newfn[n] for n in names)+ '\n'+newfn['AppMotion_LoadDefaultPlan']
    olddefault=oldfn['App_TestTaskA'];at=olddefault.index('(void)AppDebugPlan_Load');start=olddefault.rindex('    if (APP_IS_DEBUG)',0,at);end=olddefault.index('}',at)+1
    olddefault='void AppMotion_LoadDefaultPlan(void){\n'+olddefault[start:end]+'\n}'
    oldbodies='\n'.join(oldfn[n] for n in names)+'\n'+olddefault
    harness=prefix+stubs+'\n'+prototypes+'\n'+rename(prototypes)+'\n'+bodies+'\n'+rename(oldbodies)
    # App_MotionPlanValid has internal linkage in both real units.
    harness=harness.replace('uint32_t App_MotionPlanValid(void);','static uint32_t App_MotionPlanValid(void);').replace('uint32_t old_App_MotionPlanValid(void);','static uint32_t old_App_MotionPlanValid(void);')
    values=[n for n in owned if n not in ['s_heading_runtime_inputs','s_translation_runtime_inputs']]+re.findall(r's_serial_\w+',serial)
    harness+='\nvolatile uint32_t assertions,cases;\n#define CHECK(x) do{assertions++;if(!(x))return __LINE__;}while(0)\nuint32_t equal(void){\n'
    for n in values:harness+=f'CHECK(memcmp((void*)&{n},(void*)&old_{n},sizeof({n}))==0);\n'
    harness+='return 0U;}\nvoid sync(void){\n'
    for n in values:harness+=f'memcpy((void*)&old_{n},(void*)&{n},sizeof({n}));\n'
    harness+='}\nuint32_t saved_external[256];\nvoid save_external(void){uint32_t i=0U;\n'
    for n in external+['g_forward_hold']:harness+=f'memcpy(saved_external+i,(void*)&{n},sizeof({n}));i+=sizeof({n})/4U;\n'
    harness+='}\nuint32_t equal_external(void){uint32_t i=0U;\n'
    for n in external+['g_forward_hold']:harness+=f'CHECK(memcmp(saved_external+i,(void*)&{n},sizeof({n}))==0);i+=sizeof({n})/4U;\n'
    harness+='return 0U;}\nvoid environment(uint32_t n){clock_ms=10000U;trace_count=depth=0U;io_ok=editable=load_ok=health_ok=exclusive_ok=backend_rearm=1U;\n'
    for n in external+['g_forward_hold']:harness+=f'memset((void*)&{n},0,sizeof({n}));\n'
    harness+=r'''
 memset(&backend,0,sizeof(backend));memset(&power,0,sizeof(power));memset(&heading,0,sizeof(heading));
 g_translation_hold_selftest=g_safety_ready=g_can_guard_ready=1U;backend.session=71U;heading.yaw_deg=12.0f;
 switch(n%16U){case 1:backend.active=1U;break;case 2:backend.result=1U;break;case 3:backend.cut_vm=1U;break;
 case 4:power.active=1U;break;case 5:g_vm_test_request=1U;break;case 6:g_forward_hold.request=1U;break;
 case 7:g_heading_exec_request=1U;break;case 8:g_forward_hold.phase=HR_RUN;break;case 9:g_heading_exec_phase=HR_RUN;break;
 case 10:editable=0U;break;case 11:load_ok=0U;break;case 12:backend_rearm=0U;break;
 case 13:health_ok=0U;break;case 14:exclusive_ok=0U;break;case 15:g_can_ctrl_fault_latched=1U;break;}
}
RuntimeRoutePrepared prepared;
uint32_t invoke(uint32_t op,uint32_t n){
 if(op==0U){if(side)old_AppMotion_LoadDefaultPlan();else AppMotion_LoadDefaultPlan();return 1U;}
 if(op==1U)return side?old_App_DebugSelectPlan(n%8U):App_DebugSelectPlan(n%8U);
 if(op==2U)return side?old_App_RuntimeRouteInstall(&prepared,clock_ms):App_RuntimeRouteInstall(&prepared,clock_ms);
 if(op==3U){if(side)old_App_CombinedTestStop(n%4U);else App_CombinedTestStop(n%4U);return 1U;}
 if(op==4U){if(side)old_App_CombinedTestSafety();else App_CombinedTestSafety();return 1U;}
 if(op==5U){if(side)old_App_CombinedTestStep();else App_CombinedTestStep();return 1U;}
 return side?old_App_RuntimeRouteEditable(clock_ms):App_RuntimeRouteEditable(clock_ms);
}
__attribute__((section("i.test_entry"))) uint32_t test_entry(void){
 uint32_t n,op,i,r,a,b,num,save[512];const AppMotionObservation *live=AppMotion_Live();
 CHECK(sizeof(s_motion_actions)==192U);CHECK(live->actions==s_motion_actions);CHECK(live->action_count==&s_motion_action_count);
 CHECK(live->perimeter_targets==(const float (*)[3])s_perimeter_targets);CHECK(AppRuntimeRoute_Live()==&s_runtime_route);
 CHECK(live->session_open==&s_motion_session_open);CHECK(live->has_action==&s_motion_has_action);CHECK(live->turn==&s_combined_turn);
 CHECK(live->mode==&s_combined_mode);CHECK(live->base_rpm==&s_combined_base_rpm);CHECK(live->distance_mm==&s_combined_distance_mm);
 CHECK(live->reference_mm_s==&s_combined_reference_mm_s);CHECK(live->begin_ms==&s_combined_begin_ms);
 CHECK(live->end_ms==&s_combined_end_ms);CHECK(live->stage_ms==&s_combined_stage_ms);
 prepared.count=3U;prepared.id=77U;prepared.digest=0x1234U;
 for(i=0;i<3U;i++)prepared.actions[i]=(MotionAction){MOTION_FORWARD,6U,1000U,0.0f,0.0f,0.0f};
 for(n=0;n<96U;n++)for(op=0;op<7U;op++){
  memset((void*)&g_combined_test,0,sizeof(g_combined_test));memset(s_motion_actions,0,sizeof(s_motion_actions));
  memset(&s_runtime_route,0,sizeof(s_runtime_route));s_motion_action_count=2U;g_motion_plan_valid=1U;
  s_motion_session_open=s_motion_has_action=s_combined_keep_vm=s_combined_powered_segment=0U;
  s_combined_turn=n%2U;s_combined_begin_ms=s_combined_stage_ms=9900U;s_combined_distance_mm=0.0f;
  g_combined_test.state=(op==2U)?(n/16U==1U?3U:0U):((op>=3U&&op<=4U)?n/16U:0U);
  if(op==1U&&n/16U==1U)g_combined_test.request=1U;
  if(op==1U&&n/16U==2U)s_motion_session_open=1U;
  if(op==1U&&n/16U==3U)s_motion_has_action=1U;
  if(op==5U)g_combined_test.request=1U;
  if(op==4U&&n/16U==2U){s_runtime_route.committed.id=1U;s_runtime_route.consumed=1U;s_runtime_route.committed.count=2U;s_runtime_route.committed.rows[0].timeout_ms=1U;g_combined_test.state=1U;}
  CHECK(AppMotion_SetStartPermit(n%2U)==n%2U);
  s_serial_token_seq=7U;s_serial_token_tick=8U;s_serial_arm_ms=9U;s_serial_keepalive_ms=10U;
  sync();side=0U;environment(n);a=invoke(op,n);num=trace_count;memcpy(save,trace,num*4U);save_external();CHECK(depth==0U);
  if(op==1U&&!a)select_rejects++;if(op==2U&&!a)route_rejects++;
  if(op==5U&&n%2U==0U){CHECK(g_combined_test.state==4U&&g_combined_test.cause==3U);permit_rejects++;}
  if(op==1U&&n==0U)CHECK(a==1U&&s_motion_action_count==2U);
  if(op==2U&&n==0U)CHECK(a==1U&&s_motion_action_count==prepared.count&&s_runtime_route.committed.id==prepared.id);
  CHECK(live->actions==s_motion_actions&&*live->action_count==s_motion_action_count);live_checks++;
  side=1U;environment(n);b=invoke(op,n);CHECK(a==b);CHECK(num==trace_count);CHECK(memcmp(save,trace,num*4U)==0);CHECK(depth==0U);
  r=equal();CHECK(r==0U);r=equal_external();CHECK(r==0U);cases++;
 }
 CHECK(load_calls>0U&&install_calls>0U&&rearm_calls>0U&&permit_accepts>0U&&permit_rejects>0U&&stop_calls>0U&&select_rejects>0U&&route_rejects>0U);return 0U;
}
'''
    out=c.T/'unit';out.mkdir(exist_ok=True);(out/'unit.c').write_bytes(harness.encode('gbk'))
    (out/'unit.sct').write_text('LR_ROM 0x08000000 0x100000 { ER_ROM 0x08000000 0x100000 { unit.o (i.test_entry, +First) *(+RO) } RW_RAM 0x20000000 0x20000 { *(+RW,+ZI) } }\n')
    simple=[x for x in opts if x not in ['--cpu','Cortex-M4.fp.sp']]
    commands=[[str(c.B/'armcc.exe'),'--cpu','Cortex-M3','--gnu']+simple+['--split_sections','-O1','-c','unit.c','-o','unit.o'],[str(c.B/'armcc.exe'),'--cpu','Cortex-M3','--gnu']+simple+['--split_sections','-O1','-c',c.short(c.U/'heading_controller.c'),'-o','heading_controller.o'],[str(c.B/'armlink.exe'),'--entry','test_entry','--scatter','unit.sct','unit.o','heading_controller.o','--library_type=microlib','-o','unit.axf']]
    for cmd in commands:
        p=subprocess.run(cmd,cwd=out,capture_output=True,timeout=60);assert p.returncode==0,(p.stdout+p.stderr).decode(errors='replace')
    sys.path.insert(0,str(c.R/'工创赛/tmp/position_alignment_deps'));import unicorn as uc
    from unicorn import arm_const as reg
    blob=(out/'unit.axf').read_bytes();elf=struct.unpack_from('<16sHHIIIIIHHHHHH',blob)
    m=uc.Uc(uc.UC_ARCH_ARM,uc.UC_MODE_THUMB|uc.UC_MODE_MCLASS);m.mem_map(0x08000000,0x100000);m.mem_map(0x20000000,0x20000)
    for i in range(elf[10]):
        kind,off,addr,_,size,_,_,_=struct.unpack_from('<IIIIIIII',blob,elf[5]+i*elf[9])
        if kind==1 and size:m.mem_write(addr,blob[off:off+size])
    stop=0x080FF000;m.reg_write(reg.UC_ARM_REG_SP,0x2001FFF0);m.reg_write(reg.UC_ARM_REG_LR,stop|1)
    m.emu_start(elf[4]|1,stop,timeout=60000000,count=300000000)
    assert m.reg_read(reg.UC_ARM_REG_PC)==stop;assert m.reg_read(reg.UC_ARM_REG_R0)==0,f'C check line {m.reg_read(reg.UC_ARM_REG_R0)}'
    symbols=subprocess.check_output([str(c.B/'fromelf.exe'),'--text','-s','unit.axf'],cwd=out).decode(errors='replace')
    def value(n):a=int(re.search(r'\b'+n+r'\s+(0x[0-9a-fA-F]+)',symbols)[1],16);return struct.unpack('<I',m.mem_read(a,4))[0]
    return {n:value(n) for n in ['cases','assertions','load_calls','install_calls','rearm_calls','permit_accepts','permit_rejects','select_rejects','route_rejects','stop_calls','live_checks']}
