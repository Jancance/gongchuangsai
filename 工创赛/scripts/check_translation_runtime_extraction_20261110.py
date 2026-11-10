"""C2b: real target preprocessing and old/new translation application ARM replay."""
from pathlib import Path
import ctypes,hashlib,importlib.util,json,re,struct,subprocess,sys,xml.etree.ElementTree as ET
R=Path(__file__).resolve().parents[2];U=R/'3.stm32f407工程/CHASSIS/USER';T=R/'工创赛/tmp/translation_runtime_main_c2b';E=R/'工创赛/evidence/navigation-planning/2026-11-10/main-c2b';B=Path(r'D:\Keil_v5\ARM\arm5_compiler\Bin');BASE='0ba2524bcac3eea5bc5be69f1f3a096e921cede2'
REP={'(*inputs->mode)':'App_TestTranslationMode()','(*inputs->keep_vm)':'App_StageDoneVm()','(*inputs->powered_segment)':'App_StageStartVm()','(*inputs->base_rpm)':'App_StageBaseRpm()','(*inputs->run_ms)':'App_StageRunMs()','(*inputs->distance_mm)':'App_StageDistanceMm()','(*inputs->reference_mm_s)':'s_combined_reference_mm_s','inputs->route_heading_reference->valid':'s_route_heading_reference.valid','(*inputs->route_target)':'s_route_heading_target'}
MACROS={'App_TestTranslationMode()':'(s_combined_mode)','App_StageDoneVm()':'(s_combined_keep_vm)','App_StageStartVm()':'(s_combined_powered_segment)','App_StageBaseRpm()':'(s_combined_base_rpm)','App_StageRunMs()':'(s_combined_run_ms)','App_StageDistanceMm()':'(s_combined_distance_mm)'}
def short(p):
 a=ctypes.create_unicode_buffer(32768);assert ctypes.windll.kernel32.GetShortPathNameW(str(p),a,32768);return a.value
def main():
 T.mkdir(parents=True,exist_ok=True);E.mkdir(parents=True,exist_ok=True);spec=importlib.util.spec_from_file_location('h',R/'工创赛/scripts/check_heading_source_extraction_20261110.py');h=importlib.util.module_from_spec(spec);spec.loader.exec_module(h)
 old=subprocess.check_output(['git','show',f'{BASE}:3.stm32f407工程/CHASSIS/USER/main.c'],cwd=R).decode('gbk');cur=(U/'main.c').read_bytes().decode('gbk');mod=(U/'app_translation_runtime.c').read_bytes().decode('gbk');(T/'main.c').write_bytes(old.encode('gbk'))
 before,after,moved=map(h.functions,(old,cur,mod));norm=lambda s:re.sub(r'\s+','',s).removeprefix('static')
 def restore(a,pp=False):
  for new,old in REP.items():a=a.replace(new,MACROS.get(old,old) if pp else old)
  a=a.replace('(const AppTranslationRuntimeInputs *inputs)','(void)').replace(', const AppTranslationRuntimeInputs *inputs','')
  for arg in ['&status','&backend']:
   a=a.replace('App_ForwardHoldOwns('+arg+',inputs)','App_ForwardHoldOwns('+arg+')').replace('App_ForwardHoldOwns('+arg+','+'&s_translation_runtime_inputs)','App_ForwardHoldOwns('+arg+')')
  a=a.replace('App_ForwardHoldSafety(inputs);','App_ForwardHoldSafety();')
  for name in ['App_ForwardHoldStep','App_ForwardHoldSafety']:a=a.replace(name+'(&s_translation_runtime_inputs)',name+'()')
  return a
 for name,b in before.items():assert norm(restore(moved[name] if name in moved else after[name]))==norm(b),name
 pp=[];opts=None
 for target in ET.parse(U.parents[1]/'gongxun-f407.uvprojx').getroot().findall('./Targets/Target'):
  ctrl=target.find('./TargetOption/TargetArmAds/Cads/VariousControls');opts=['--cpu','Cortex-M4.fp.sp','--c99']+['-D'+d for d in ctrl.findtext('Define').split(',')]
  for inc in ctrl.findtext('IncludePath').split(';'):opts+=['-I',short((U.parents[1]/inc).resolve())]
  units=[]
  for tag,p in [('old',T/'main.c'),('main',U/'main.c'),('module',U/'app_translation_runtime.c')]:
   run=subprocess.run([str(B/'armcc.exe')]+opts+['-E',short(p)],cwd=T,capture_output=True,timeout=60);assert run.returncode==0,run.stderr.decode(errors='replace');text=re.sub(r'^#.*$','',run.stdout.decode('gbk'),flags=re.M);(T/(target.findtext('TargetName')+'_'+tag+'.i')).write_text(text,'utf8');units.append(h.functions(text))
  pb,pa,pm=units
  for name,b in pb.items():
   a=restore(pm[name] if name in pm else pa[name],True)
   if name=='main':
    aa=re.findall(r'App_RtosAssertFailed\((\d+)\)',a);bb=re.findall(r'App_RtosAssertFailed\((\d+)\)',b);assert len(aa)==len(bb)==8 and all(int(x)-int(y)==-322 for x,y in zip(aa,bb));a=re.sub(r'App_RtosAssertFailed\((\d+)\)',lambda m:'App_RtosAssertFailed('+str(int(m[1])+322)+')',a)
   assert norm(a)==norm(b),(target.findtext('TargetName'),name)
  pp.append(dict(target=target.findtext('TargetName'),functions=len(pb)))
 replay(old,mod,h,opts,pp,len(before),len(moved))

def replay(old,mod,h,opts,pp,total,count):
 lines=old.splitlines(keepends=True);seg=lambda a,b:''.join(lines[a-1:b-1]);decl=seg(234,236)+seg(282,286)+seg(307,310)
 owned=['g_forward_hold','g_translation_hold_selftest','g_forward_hold_trace','g_forward_hold_trace_count','s_hold_wait_ms','s_hold_begin_ms','s_hold_run_ms','s_hold_stop_ms','s_hold_sequence','s_hold_new_ms','s_hold_target','s_hold_controller','s_hold_trace_ms']
 external=sorted(set(re.findall(r'\bg_\w+',mod))-set(owned));newbody=re.sub(r'^#include .*\r?\n','',mod,flags=re.M);oldfn=h.functions(old);moved=h.functions(mod)
 rename=''.join('#define '+n+' old_'+n+'\n' for n in owned+list(moved));undo=''.join('#undef '+n+'\n' for n in owned+list(moved))
 prefix=''.join(re.findall(r'^#include .*\r?\n',mod,re.M))+'#include <string.h>\n'+'\n'.join('volatile uint32_t '+n+('[3]' if n=='g_chassis_auto_start_mask' else '')+';' for n in external)+'\n'
 prefix+=r'''
uint32_t clock_ms,cycles,trace[1024],trace_count,depth,io_ok,mutate,prepare_calls,begin_calls,mutations;
uint32_t s_combined_mode,s_combined_keep_vm,s_combined_powered_segment,s_combined_base_rpm,s_combined_run_ms;
float s_combined_distance_mm,s_combined_reference_mm_s,s_route_heading_target;
RouteHeadingReference s_route_heading_reference;
#define App_TestTranslationMode() (s_combined_mode)
#define App_StageDoneVm() (s_combined_keep_vm)
#define App_StageStartVm() (s_combined_powered_segment)
#define App_StageBaseRpm() (s_combined_base_rpm)
#define App_StageRunMs() (s_combined_run_ms)
#define App_StageDistanceMm() (s_combined_distance_mm)
static const AppTranslationRuntimeInputs inputs={&s_combined_mode,&s_combined_keep_vm,&s_combined_powered_segment,&s_combined_base_rpm,&s_combined_run_ms,&s_combined_distance_mm,&s_combined_reference_mm_s,&s_route_heading_target,&s_route_heading_reference};
AppHeadingSource source;ChassisAutoStatus backend;
void event(uint32_t v){if(trace_count<1024U)trace[trace_count++]=v;}
void enter(void){event(100U+depth++);}void leave(void){event(200U+--depth);}
#undef taskENTER_CRITICAL
#undef taskEXIT_CRITICAL
#define taskENTER_CRITICAL() enter()
#define taskEXIT_CRITICAL() leave()
TickType_t xTaskGetTickCount(void){event(1U);return clock_ms;}
uint32_t HWT101_TimingBegin(void){event(2U);return cycles++;}void HWT101_TimingEnd(uint32_t s,uint32_t b){event(3U);event(s);event(b);}
void HWT101_DiagnosticArm(void){event(4U);}
void App_HeadingSourceObserve(void){event(5U);}uint32_t App_HeadingSourceRead(AppHeadingSource *p){event(6U);*p=source;return io_ok;}
uint32_t App_HeadingRuntimeSource(uint32_t now){event(7U);event(now);return io_ok;}
uint32_t App_HeadingRuntimeExclusive(void){event(8U);return io_ok;}
uint32_t App_HeadingPowerReady(uint32_t now){event(9U);event(now);return io_ok;}
uint32_t App_CanJogReady(void){event(10U);return io_ok;}
void AppMotion_InternalFaultCapture(uint32_t cause){event(11U);event(cause);event(depth);}
void ChassisAuto_Abort(void){event(12U);}
uint32_t ChassisAuto_ReadStatus(ChassisAutoStatus *s){event(13U);*s=backend;return 1U;}
uint32_t ChassisAuto_PrepareTranslation(uint32_t mode,uint32_t rpm,uint32_t ms,float mm,ChassisAutoPreparedTranslation *p){event(14U);event(mode);event(rpm);event(ms);event((uint32_t)mm);prepare_calls++;memset(p,0,sizeof(*p));p->valid=1U;if(mutate){s_combined_base_rpm++;mutations++;}return io_ok;}
uint32_t ChassisAuto_PrepareRouteTranslation(uint32_t mode,uint32_t rpm,uint32_t ms,float mm,float route,ChassisAutoPreparedTranslation *p){event((uint32_t)route);return ChassisAuto_PrepareTranslation(mode,rpm,ms,mm,p);}
uint32_t ChassisAuto_BeginPreparedTranslation(const ChassisAutoPreparedTranslation *p,uint32_t session){event(15U);event(session);event(p->valid);begin_calls++;backend.active=1U;backend.state=CHASSIS_AUTO_RUN;return io_ok;}
uint32_t ChassisAuto_UpdateForwardHold(float rpm){event(16U);event((uint32_t)(rpm*10));return io_ok;}
uint32_t ChassisAuto_RequestStop(void){event(17U);return io_ok;}
void ChassisAuto_CanStep(void){event(18U);}
'''
 harness=prefix+newbody+'\n'+rename+decl+'\n'+'\n'.join(oldfn[n] for n in moved)+'\n'+undo
 harness+='\nvolatile uint32_t assertions,cases;\n#define CHECK(x) do{assertions++;if(!(x))return __LINE__;}while(0)\nuint32_t equal(void){\n'
 for n in owned:harness+=f'CHECK(memcmp((void*)&{n},(void*)&old_{n},sizeof({n}))==0);\n'
 harness+='return 0U;}\nvoid sync(void){\n'
 for n in owned:harness+=f'memcpy((void*)&old_{n},(void*)&{n},sizeof({n}));\n'
 harness+='}\nuint32_t saved_external[64];\nvoid save_external(void){uint32_t i=0U;\n'
 for n in external:harness+=f'memcpy(saved_external+i,(void*)&{n},sizeof({n}));i+=sizeof({n})/4U;\n'
 harness+='}\nuint32_t equal_external(void){uint32_t i=0U;\n'
 for n in external:harness+=f'CHECK(memcmp(saved_external+i,(void*)&{n},sizeof({n}))==0);i+=sizeof({n})/4U;\n'
 harness+='return 0U;}\nvoid environment(uint32_t n){clock_ms=10000U+n;cycles=trace_count=depth=0U;io_ok=n%17U!=0U;mutate=n%5U==0U;\n'
 for n in external:harness+=f'memset((void*)&{n},0,sizeof({n}));\n'
 harness+=r'''
 s_combined_mode=n%4U;s_combined_keep_vm=n%2U;s_combined_powered_segment=1U;s_combined_base_rpm=10U;s_combined_run_ms=8000U;
 s_combined_distance_mm=n%2U?100.0f:0.0f;s_combined_reference_mm_s=n%7U==0U?20.0f:0.0f;s_route_heading_reference.valid=n%19U!=0U;s_route_heading_target=10.0f;
 memset(&source,0,sizeof(source));source.sequence=n+1U;source.healthy=1U;source.yaw_deg=10.0f;source.rate_dps=0.1f;
 memset(&backend,0,sizeof(backend));backend.session=8U;backend.heading_mode=backend.forward_hold_mode=1U;backend.translation_mode=s_combined_mode;backend.segment=2U;
 g_vm_test_request=g_safety_ready=g_can_guard_ready=1U;
 if(n%23U==0U)g_vm_test_request=0U;
 if(n%29U==0U)g_stage1_stop_requested=1U;
}
__attribute__((section("i.test_entry"))) uint32_t test_entry(void){
 uint32_t n,k,r,save[1024],num,cyc;
 for(n=0;n<512U;n++){
  memset((void*)&g_forward_hold,0,sizeof(g_forward_hold));g_forward_hold.phase=n%4U;g_forward_hold.request=n%3U;g_forward_hold.backend_session=8U;
  g_translation_hold_selftest=1U;s_hold_wait_ms=s_hold_begin_ms=s_hold_new_ms=10000U+n-10U;s_hold_target=10.0f;s_hold_sequence=n;s_hold_controller=(HeadingHoldController){0};
  sync();
  for(k=0;k<2U;k++){
   environment(n);if(k==0U)App_ForwardHoldStep(&inputs);else App_ForwardHoldSafety(&inputs);num=trace_count;memcpy(save,trace,num*4U);cyc=cycles;save_external();CHECK(depth==0U);
   environment(n);if(k==0U)old_App_ForwardHoldStep();else old_App_ForwardHoldSafety();CHECK(num==trace_count);CHECK(memcmp(save,trace,num*4U)==0);CHECK(cyc==cycles);CHECK(depth==0U);r=equal();CHECK(r==0U);r=equal_external();CHECK(r==0U);cases++;
  }
 }
 return 0U;
}
'''
 out=T/'unit';out.mkdir(exist_ok=True);(out/'unit.c').write_bytes(harness.encode('gbk'));(out/'unit.sct').write_text('LR_ROM 0x08000000 0x100000 { ER_ROM 0x08000000 0x100000 { unit.o (i.test_entry, +First) *(+RO) } RW_RAM 0x20000000 0x20000 { *(+RW,+ZI) } }\n')
 simple=[o for o in opts if o not in ['--cpu','Cortex-M4.fp.sp']]
 commands=[[str(B/'armcc.exe'),'--cpu','Cortex-M3','--gnu']+simple+['--split_sections','-O1','-c','unit.c','-o','unit.o'],[str(B/'armcc.exe'),'--cpu','Cortex-M3','--gnu']+simple+['--split_sections','-O1','-c',short(U/'heading_controller.c'),'-o','heading_controller.o'],[str(B/'armlink.exe'),'--entry','test_entry','--scatter','unit.sct','unit.o','heading_controller.o','--library_type=microlib','-o','unit.axf']]
 for cmd in commands:
  p=subprocess.run(cmd,cwd=out,capture_output=True,timeout=60);assert p.returncode==0,(p.stdout+p.stderr).decode(errors='replace')
 sys.path.insert(0,str(R/'工创赛/tmp/position_alignment_deps'));import unicorn as uc
 from unicorn import arm_const as reg
 blob=(out/'unit.axf').read_bytes();elf=struct.unpack_from('<16sHHIIIIIHHHHHH',blob);m=uc.Uc(uc.UC_ARCH_ARM,uc.UC_MODE_THUMB|uc.UC_MODE_MCLASS);m.mem_map(0x08000000,0x100000);m.mem_map(0x20000000,0x20000)
 for i in range(elf[10]):
  kind,off,addr,_,size,_,_,_=struct.unpack_from('<IIIIIIII',blob,elf[5]+i*elf[9])
  if kind==1 and size:m.mem_write(addr,blob[off:off+size])
 stop=0x080FF000;m.reg_write(reg.UC_ARM_REG_SP,0x2001FFF0);m.reg_write(reg.UC_ARM_REG_LR,stop|1);m.emu_start(elf[4]|1,stop,timeout=60000000,count=300000000);assert m.reg_read(reg.UC_ARM_REG_PC)==stop;assert m.reg_read(reg.UC_ARM_REG_R0)==0,f'C check line {m.reg_read(reg.UC_ARM_REG_R0)}'
 symbols=subprocess.check_output([str(B/'fromelf.exe'),'--text','-s','unit.axf'],cwd=out).decode(errors='replace')
 def value(n):a=int(re.search(r'\b'+n+r'\s+(0x[0-9a-fA-F]+)',symbols)[1],16);return struct.unpack('<I',m.mem_read(a,4))[0]
 assert value('prepare_calls') and value('begin_calls') and value('mutations')
 report=dict(ok=True,baseline=BASE,moved_functions=count,remaining_main_functions=total-count,preprocessed_targets=pp,owned_symbols=len(owned),cases=value('cases'),assertions=value('assertions'),prepare_calls=value('prepare_calls'),begin_calls=value('begin_calls'),live_mutations=value('mutations'),hardware_accessed=False,limits='Actual translation application and heading_controller C. Backend, stage motion and RTOS/clock are stubbed; no real task timing or hardware validation.')
 (E/'translation_runtime_check.json').write_text(json.dumps(report,indent=2),'utf8');print(json.dumps(report))
if __name__=='__main__':main()
