"""C2a: extraction equivalence with actual target preprocessing and ARM C replay."""
from pathlib import Path
import ctypes,hashlib,importlib.util,json,re,struct,subprocess,sys,xml.etree.ElementTree as ET
R=Path(__file__).resolve().parents[2];U=R/'3.stm32f407工程/CHASSIS/USER';T=R/'工创赛/tmp/heading_runtime_main_c2a';E=R/'工创赛/evidence/navigation-planning/2026-11-10/main-c2a'
B=Path(r'D:\Keil_v5\ARM\arm5_compiler\Bin');BASE='660223ec734e9fdcdfb86fb47f2314665b245730'
REPLACE={'(*inputs->keep_vm)':'App_StageDoneVm()','(*inputs->powered_segment)':'App_StageStartVm()','(*inputs->turn_max_rpm)':'s_motion_turn_max_rpm','(*inputs->route_turn_active)':'s_route_heading_turn_active','(*inputs->route_target)':'s_route_heading_target'}
def short(p):
 a=ctypes.create_unicode_buffer(32768);assert ctypes.windll.kernel32.GetShortPathNameW(str(p),a,32768);return a.value
def main():
 T.mkdir(exist_ok=True,parents=True);E.mkdir(exist_ok=True,parents=True)
 spec=importlib.util.spec_from_file_location('h',R/'工创赛/scripts/check_heading_source_extraction_20261110.py');h=importlib.util.module_from_spec(spec);spec.loader.exec_module(h)
 old=subprocess.check_output(['git','show',f'{BASE}:3.stm32f407工程/CHASSIS/USER/main.c'],cwd=R).decode('gbk');cur=(U/'main.c').read_bytes().decode('gbk');mod=(U/'app_heading_runtime.c').read_bytes().decode('gbk')
 (T/'main.c').write_bytes(old.encode('gbk'))
 before,after,moved=map(h.functions,(old,cur,mod));norm=lambda x:re.sub(r'\s+','',x).removeprefix('static')
 def restore(a,name,pp=False):
  for new,original in REPLACE.items():
   if pp and original=='App_StageDoneVm()':original='(s_combined_keep_vm)'
   elif pp and original=='App_StageStartVm()':original='(s_combined_powered_segment)'
   a=a.replace(new,original)
  a=a.replace('(const AppHeadingRuntimeInputs *inputs)','(void)')
  a=a.replace('AppMotion_InternalFaultCapture','App_MotionFaultCapture')
  a=a.replace('AppHeadingRuntime_ResetSession();','HeadingSession_Init(&s_heading_exec);')
  for f in ['App_HeadingRuntimeStep','App_HeadingRuntimeSafety']:a=a.replace(f+'(&s_heading_runtime_inputs)',f+'()')
  return a
 for name,b in before.items():
  if name in moved:a=moved[name]
  else:a=after[name] if name!='App_MotionFaultCapture' else after['AppMotion_InternalFaultCapture']
  assert norm(restore(a,name))==norm(b),name
 assert moved['AppHeadingRuntime_ResetSession']=='void AppHeadingRuntime_ResetSession(void)\r\n{\r\n    HeadingSession_Init(&s_heading_exec);\r\n}'
 for name in ['stage1_runtime.inc','vehicle_mission.inc']:
  b=subprocess.check_output(['git','show',f'{BASE}:3.stm32f407工程/CHASSIS/USER/{name}'],cwd=R)
  assert (U/name).read_bytes().replace(b'AppHeadingRuntime_ResetSession();',b'HeadingSession_Init(&s_heading_exec);')==b
 targets=ET.parse(U.parents[1]/'gongxun-f407.uvprojx').getroot().findall('./Targets/Target');pp=[];last=None
 for target in targets:
  ctrl=target.find('./TargetOption/TargetArmAds/Cads/VariousControls');opts=['--cpu','Cortex-M4.fp.sp','--c99']
  opts+=['-D'+d for d in ctrl.findtext('Define').split(',')]
  for inc in ctrl.findtext('IncludePath').split(';'):opts+=['-I',short((U.parents[1]/inc).resolve())]
  units=[]
  for name,p in [('old',T/'main.c'),('main',U/'main.c'),('module',U/'app_heading_runtime.c')]:
   run=subprocess.run([str(B/'armcc.exe')]+opts+['-E',short(p)],cwd=T,capture_output=True,timeout=60);assert run.returncode==0,run.stderr.decode(errors='replace')
   s=re.sub(r'^#.*$','',run.stdout.decode('gbk'),flags=re.M);(T/(target.findtext('TargetName')+'_'+name+'.i')).write_text(s,'utf8');units.append(s)
  pb,pa,pm=map(h.functions,units)
  for name,b in pb.items():
   if name in pm:a=pm[name]
   elif name=='App_MotionFaultCapture':a=pa['AppMotion_InternalFaultCapture']
   else:a=pa[name]
   a=restore(a,name,True)
   if name=='main':
    aa=re.findall(r'App_RtosAssertFailed\((\d+)\)',a);bb=re.findall(r'App_RtosAssertFailed\((\d+)\)',b);assert len(aa)==len(bb)==8;assert all(int(x)-int(y)==-477 for x,y in zip(aa,bb))
    a=re.sub(r'App_RtosAssertFailed\((\d+)\)',lambda m:'App_RtosAssertFailed('+str(int(m[1])+477)+')',a)
   assert norm(a)==norm(b),(target.findtext('TargetName'),name)
  pp.append(dict(target=target.findtext('TargetName'),functions=len(pb)));last=(units,opts)
 actual_c_replay(h,old,mod,last[0],last[1],pp)

def actual_c_replay(h,old,mod,units,opts,pp):
 oldfn=h.functions(old);newfn=h.functions(mod)
 # Same actual headers/types and actual application functions; only external I/O is stubbed.
 pre=''.join(re.findall(r'^#include .*\r?\n',mod,re.M))+'#include <string.h>\n'
 decl=mod[mod.index('#if APP_HEADING_SESSION_TEST')+len('#if APP_HEADING_SESSION_TEST'):mod.index('void App_HeadingRuntimeTrip')]
 owned=[]
 for m in re.finditer(r'^(?:static )?(?:volatile )?(?:uint32_t|float|HeadingStopTrace|HeadingSession|ChassisAutoPreparedHeading) ([^;=]+)',decl,re.M):owned+=m[1].split(',')
 owned=[n.strip() for n in owned]
 external=sorted(set(re.findall(r'\bg_\w+',mod))-set(owned));arrays={'g_can_poll_count','g_can_poll_flags','g_can_poll_result','g_can_poll_tick'}
 globals='\n'.join('volatile uint32_t '+n+('[4]' if n in arrays else '')+';' for n in external)
 current=re.sub(r'^#include .*\r?\n','',mod,flags=re.M)
 names=list(newfn)+owned
 rename=''.join('#define '+n+' old_'+n+'\n' for n in names);undo=''.join('#undef '+n+'\n' for n in names)
 rawlines=old.splitlines(keepends=True);seg=lambda a,b:''.join(rawlines[a-1:b-1])
 # Declaration text retains original initializers; remove only its duplicate extern.
 olddecl=seg(203,219)+seg(239,242)+seg(246,248)+seg(249,265)
 olddecl=olddecl.replace('extern volatile uint32_t g_motion_fault_diag_valid;','')
 oldbody='\n'.join(oldfn[n] for n in newfn if n in oldfn)
 oldbody=oldbody.replace('App_MotionFaultCapture(cause);','AppMotion_InternalFaultCapture(cause);')
 harness=pre+globals+r'''
uint32_t clock_ms,cycles,trace[1024],trace_count,lock_depth,hook,entered,io_ok,prepare_calls,begin_calls,mutations;
uint32_t s_combined_keep_vm,s_combined_powered_segment,s_motion_turn_max_rpm,s_route_heading_turn_active;
float s_route_heading_target;
#define App_StageDoneVm() (s_combined_keep_vm)
#define App_StageStartVm() (s_combined_powered_segment)
static const AppHeadingRuntimeInputs inputs={&s_combined_keep_vm,&s_combined_powered_segment,&s_motion_turn_max_rpm,&s_route_heading_turn_active,&s_route_heading_target};
ChassisAutoStatus backend;PowerCommandStatus power_input;AppHeadingSource heading_input;
void event(uint32_t x){if(trace_count<1024U)trace[trace_count++]=x;}
void trace_enter(void){event(100U+lock_depth++);entered++;if(hook==1U && entered==2U){mutations++;s_combined_keep_vm^=1U;s_combined_powered_segment^=1U;s_motion_turn_max_rpm=3U;s_route_heading_turn_active^=1U;s_route_heading_target=90.0f;}}
void trace_leave(void){event(200U+--lock_depth);}
#undef taskENTER_CRITICAL
#undef taskEXIT_CRITICAL
#define taskENTER_CRITICAL() trace_enter()
#define taskEXIT_CRITICAL() trace_leave()
TickType_t xTaskGetTickCount(void){event(1U);return clock_ms;}
uint32_t HWT101_TimingBegin(void){event(2U);return cycles++;}
void HWT101_TimingEnd(uint32_t s,uint32_t b){event(3U);event(s);event(b);}
void HWT101_DiagnosticArm(void){event(4U);}
void App_HeadingSourceObserve(void){event(5U);}
uint32_t App_HeadingSourceRead(AppHeadingSource *s){event(6U);*s=heading_input;return io_ok;}
void Power_ReadCommandStatus(PowerCommandStatus *s){event(7U);*s=power_input;}
void AppMotion_InternalFaultCapture(uint32_t cause){event(8U);event(cause);event(lock_depth);}
void ChassisAuto_Abort(void){event(9U);}
uint32_t ChassisAuto_ReadStatus(ChassisAutoStatus *s){event(10U);*s=backend;return 1U;}
uint32_t ChassisAuto_PrepareHeading(uint32_t dry,float rpm,int32_t sign,ChassisAutoPreparedHeading *p){prepare_calls++;event(11U);event(dry);event((uint32_t)(rpm*10));event((uint32_t)sign);memset(p,0,sizeof(*p));p->valid=1U;p->command_rpm=rpm;return io_ok;}
uint32_t ChassisAuto_BeginPreparedHeading(const ChassisAutoPreparedHeading *p,uint32_t expected){begin_calls++;event(12U);event(expected);event((uint32_t)(p->command_rpm*10));return io_ok;}
uint32_t ChassisAuto_UpdateHeading(float rpm){event(13U);event((uint32_t)(rpm*10));return io_ok;}
uint32_t ChassisAuto_RequestStop(void){event(14U);return io_ok;}
void ChassisAuto_CanStep(void){event(15U);}
uint32_t jog;
const uint32_t *AppCanDiagnostics_JogPreparing(void){event(16U);return &jog;}
uint32_t App_CanJogOtherRequest(void){event(17U);return 0U;}
uint32_t App_CanJogReady(void){event(18U);return io_ok;}
'''+current+'\n'+rename+olddecl+'\n'+oldbody+'\n'+undo
 harness+='\nvolatile uint32_t cases,assertions;\n#define CHECK(x) do{assertions++;if(!(x))return __LINE__;}while(0)\nuint32_t equal(void){\n'
 for n in owned:harness+=f'CHECK(memcmp((void*)&{n},(void*)&old_{n},sizeof({n}))==0);\n'
 harness+='return 0U;}\nvoid sync(void){\n'
 for n in owned:harness+=f'memcpy((void*)&old_{n},(void*)&{n},sizeof({n}));\n'
 harness+='}\nuint32_t saved_external[64];\nvoid save_external(void){uint32_t i=0U;\n'
 for n in external:harness+=f'memcpy(saved_external+i,(void*)&{n},sizeof({n}));i+=sizeof({n})/4U;\n'
 harness+='}\nuint32_t equal_external(void){uint32_t i=0U;\n'
 for n in external:harness+=f'CHECK(memcmp(saved_external+i,(void*)&{n},sizeof({n}))==0);i+=sizeof({n})/4U;\n'
 harness+='return 0U;}\nvoid environment(uint32_t n){clock_ms=10000U+n;cycles=trace_count=lock_depth=entered=0U;io_ok=1U;hook=n%5U==0U;\n'
 for n in external:harness+=f'memset((void*)&{n},0,sizeof({n}));\n'
 harness+=r'''
 g_safety_ready=g_can_guard_ready=g_can_poll_enable=g_can_test_result=1U;
 g_vm_test_request=1U;memset(&heading_input,0,sizeof(heading_input));heading_input.healthy=1U;heading_input.valid_mask=3U;heading_input.sample_ms=heading_input.angle_ms=heading_input.rate_ms=clock_ms;heading_input.yaw_deg=15.0f;heading_input.sequence=n;
 memset(&backend,0,sizeof(backend));backend.session=8U;backend.segment=2U;backend.heading_mode=1U;backend.rotation_stop_mask=15U;
 power_input.active=power_input.enable_seen=1U;power_input.enable_tick=clock_ms-1500U;
 {uint32_t i;for(i=0;i<4U;i++){g_can_poll_result[i]=g_can_poll_count[i]=1U;g_can_poll_tick[i]=clock_ms;}}
 jog=0U;s_combined_keep_vm=n%2U;s_combined_powered_segment=1U;s_motion_turn_max_rpm=5U;s_route_heading_turn_active=n%7U==0U;s_route_heading_target=45.0f;
 if(n%17U==0U)heading_input.sample_ms-=100U;
 if(n%19U==0U)heading_input.angle_ms-=100U;
 if(n%23U==0U)heading_input.rate_ms-=100U;
 if(n%29U==0U)g_safety_fault=1U;
 if(n%31U==0U)g_can_poll_enable=0U;
 if(n%37U==0U)power_input.enable_tick=clock_ms;
 if(n%41U==0U)io_ok=0U;
 if(n%43U==0U)g_vm_test_request=0U;
 if(n%47U==0U)g_hwt_cal_request=1U;
}
__attribute__((section("i.test_entry"))) uint32_t test_entry(void){
 uint32_t n,k,r,save[1024],count,cyc;
 for(n=0;n<1024U;n++){
  memset((void*)&g_heading_stop_trace,0,sizeof(g_heading_stop_trace));
  AppHeadingRuntime_ResetSession();HeadingSession_Init(&old_s_heading_exec);
  g_heading_exec_phase=n%6U;g_heading_exec_trip=n%11U==0U?2U:0U;g_heading_exec_request=n%3U;
  g_heading_exec_relative_deg=n%2U?30.0f:-30.0f;s_heading_exec_relative=g_heading_exec_relative_deg;g_heading_exec_backend_session=8U;g_heading_exec_state=n%3U==0U?HS_STOPPING:HS_RUNNING;
  s_heading_exec_wait_ms=10000U+n-(n%7U==0U?5000U:10U);
  sync();
  for(k=0;k<2U;k++){
   environment(n);if(k==0U)App_HeadingRuntimeStep(&inputs);else App_HeadingRuntimeSafety(&inputs);
   save_external();count=trace_count;memcpy(save,trace,count*4U);cyc=cycles;CHECK(lock_depth==0U);
   environment(n);if(k==0U)old_App_HeadingRuntimeStep();else old_App_HeadingRuntimeSafety();
   CHECK(count==trace_count);CHECK(memcmp(save,trace,count*4U)==0);CHECK(cyc==cycles);CHECK(lock_depth==0U);r=equal();CHECK(r==0U);r=equal_external();CHECK(r==0U);cases++;
  }
 }
 return 0U;
}
'''
 out=T/'unit';out.mkdir(exist_ok=True);(out/'unit.c').write_bytes(harness.encode('gbk'))
 (out/'unit.sct').write_text('LR_ROM 0x08000000 0x100000 { ER_ROM 0x08000000 0x100000 { unit.o (i.test_entry, +First) *(+RO) } RW_RAM 0x20000000 0x20000 { *(+RW,+ZI) } }\n')
 # Pure session/controller math are compiled from real source; hardware effects above are stubs.
 simple=[o for o in opts if o not in ['--cpu','Cortex-M4.fp.sp']]
 commands=[[str(B/'armcc.exe'),'--cpu','Cortex-M3','--gnu']+simple+['--split_sections','-O1','-c','unit.c','-o','unit.o']]
 for name in ['heading_session','heading_controller']:
  commands.append([str(B/'armcc.exe'),'--cpu','Cortex-M3','--gnu']+simple+['--split_sections','-O1','-c',short(U/(name+'.c')),'-o',name+'.o'])
 commands.append([str(B/'armlink.exe'),'--entry','test_entry','--scatter','unit.sct','unit.o','heading_session.o','heading_controller.o','--library_type=microlib','-o','unit.axf'])
 for cmd in commands:
  p=subprocess.run(cmd,cwd=out,capture_output=True,timeout=60);assert p.returncode==0,(p.stdout+p.stderr).decode(errors='replace')
 sys.path.insert(0,str(R/'工创赛/tmp/position_alignment_deps'));import unicorn as uc
 from unicorn import arm_const as reg
 blob=(out/'unit.axf').read_bytes();elf=struct.unpack_from('<16sHHIIIIIHHHHHH',blob);m=uc.Uc(uc.UC_ARCH_ARM,uc.UC_MODE_THUMB|uc.UC_MODE_MCLASS);m.mem_map(0x08000000,0x100000);m.mem_map(0x20000000,0x20000)
 for i in range(elf[10]):
  kind,offset,address,_,size,_,_,_=struct.unpack_from('<IIIIIIII',blob,elf[5]+i*elf[9])
  if kind==1 and size:m.mem_write(address,blob[offset:offset+size])
 stop=0x080FF000;m.reg_write(reg.UC_ARM_REG_SP,0x2001FFF0);m.reg_write(reg.UC_ARM_REG_LR,stop|1);m.emu_start(elf[4]|1,stop,timeout=60000000,count=300000000)
 assert m.reg_read(reg.UC_ARM_REG_PC)==stop,'did not finish';assert m.reg_read(reg.UC_ARM_REG_R0)==0,f'C assertion line {m.reg_read(reg.UC_ARM_REG_R0)}'
 symbols=subprocess.check_output([str(B/'fromelf.exe'),'--text','-s','unit.axf'],cwd=out).decode(errors='replace')
 def value(n):a=int(re.search(r'\b'+n+r'\s+(0x[0-9a-fA-F]+)',symbols)[1],16);return struct.unpack('<I',m.mem_read(a,4))[0]
 assert value('prepare_calls')>0 and value('begin_calls')>0 and value('mutations')>0,(value('prepare_calls'),value('begin_calls'),value('mutations'))
 report=dict(ok=True,baseline=BASE,preparation_calls=value('prepare_calls'),begin_calls=value('begin_calls'),live_mutations=value('mutations'),preprocessed_targets=pp,moved_functions=9,remaining_main_functions=69,owned_state_symbols=len(owned),actual_arm_cases=value('cases'),assertions=value('assertions'),hardware_accessed=False,limits='Actual application, heading_session and heading_controller C. Backend, heading-source publication, clock, power and RTOS effects stubbed; no true task scheduling or hardware validation.')
 (E/'heading_runtime_check.json').write_text(json.dumps(report,indent=2),'utf8');print(json.dumps(report))
if __name__=='__main__':main()
