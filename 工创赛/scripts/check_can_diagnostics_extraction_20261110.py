"""main-C1: unchanged CAN owner functions and actual old/new C with stub I/O."""
from pathlib import Path
import ctypes, hashlib, importlib.util, json, re, struct, subprocess, sys
import xml.etree.ElementTree as ET

R=Path(__file__).resolve().parents[2]
U=R/'3.stm32f407工程/CHASSIS/USER'
T=R/'工创赛/tmp/can_main_c1'
E=R/'工创赛/evidence/navigation-planning/2026-11-10/main-c1'
BIN=Path(r'D:\Keil_v5\ARM\arm5_compiler\Bin')
BASE='16f47b76753f115b2326602cabc0c760fa52eafd'

def main():
 spec=importlib.util.spec_from_file_location('helpers',R/'工创赛/scripts/check_heading_source_extraction_20261110.py');h=importlib.util.module_from_spec(spec);spec.loader.exec_module(h)
 old=subprocess.check_output(['git','show',f'{BASE}:3.stm32f407工程/CHASSIS/USER/main.c'],cwd=R).decode('gbk')
 T.mkdir(parents=True,exist_ok=True)
 (T/'main_before.c').write_bytes(old.encode('gbk'))
 current=(U/'main.c').read_bytes().decode('gbk');mod=(U/'app_can_diagnostics.c').read_bytes().decode('gbk')
 before,after,moved=map(h.functions,(old,current,mod))
 moved_names=[n for n in moved if n in before]
 for n in moved_names:assert moved[n].replace('static ','',1)==before[n].replace('static ','',1),n
 dispatch=moved['AppCanDiagnostics_Dispatch'];body=dispatch[dispatch.index('{')+3:dispatch.rindex('}')]
 assert body in before['App_CanTask']
 checked=0
 for n,b in before.items():
  if n in moved_names:continue
  a=after[n].replace('        AppCanDiagnostics_Dispatch();\r\n',body)
  a=a.replace('(*AppCanDiagnostics_JogPreparing())','s_can_jog_preparing')
  a=a.replace('    s_odometry_inputs.jog_preparing = AppCanDiagnostics_JogPreparing();\r\n','')
  assert a==b,n;checked+=1
 assert re.search(r'const uint32_t \*AppCanDiagnostics_JogPreparing\(void\)\r\n\{[^}]+\}',mod)[0]=='const uint32_t *AppCanDiagnostics_JogPreparing(void)\r\n{\r\n    return &s_can_jog_preparing;\r\n}'
 # Separate actual old declarations/functions, with no edited algorithmic body.
 lines=old.splitlines(keepends=True);seg=lambda a,b:''.join(lines[a-1:b-1])
 declarations=seg(68,182)+seg(202,240)
 old_unit=declarations+seg(257,265)+seg(266,481)+seg(536,748)+'\r\nstatic void OldDispatch(void)\r\n{\r\n'+body+'}\r\n'
 owned=re.findall(r'^(?:static )?(?:volatile )?(?:uint32_t|TickType_t|float|CanRxMsg) (\w+)',declarations,re.M)
 externals=sorted(set(re.findall(r'\bg_\w+',mod))-set(owned)-{'g_chassis_forward_direction'})
 # I/O-only stubs: trace order, addresses, values and transmitted bytes.
 preamble=r'''
#include <stdint.h>
#include <stddef.h>
#include <string.h>
#include <stdio.h>
typedef uint32_t TickType_t;
typedef struct {uint32_t StdId,ExtId;uint8_t IDE,RTR,DLC,Data[8],FMI;} CanRxMsg;
struct {uint32_t ESR;} controller;
#define CAN1 (&controller)
#define pdMS_TO_TICKS(x) (x)
#define CAN_POLL_VALID_MS 1000U
#define MOTION_STOP_IDLE 0U
#define MOTION_STOP_PENDING 1U
#define WHEEL_TEST_PREPARING 7U
#define WHEEL_TEST_STARTING 1U
#define WHEEL_TEST_STOPPING 3U
uint32_t clock_ms,io_result,io_flags,io_allowed,trace[512],trace_count,lock_depth;
const uint8_t g_chassis_forward_direction[4]={0,1,0,1};
void event(uint32_t x){if(trace_count<512U)trace[trace_count++]=x;}
void enter(void){event(10U+lock_depth++);}void leave(void){event(20U+--lock_depth);}
#define taskENTER_CRITICAL() enter()
#define taskEXIT_CRITICAL() leave()
uint32_t xTaskGetTickCount(void){event(30U);return clock_ms;}
uint32_t transfer(uint32_t op,uint32_t a,CanRxMsg *p){event(op);event(a);memset(p,0,sizeof(*p));p->Data[1]=(uint8_t)io_flags;p->ExtId=a*256U;p->DLC=2U;clock_ms++;return io_result;}
void CAN1_ResetSession(void){event(100U);}
uint32_t CAN1_EmmSettingsConfirmed(void){event(101U);return io_allowed;}
uint32_t CAN1_ReadStatus(uint8_t a,CanRxMsg *p){return transfer(102U,a,p);}
uint32_t CAN1_ReadVersion(uint8_t a,CanRxMsg *p){return transfer(103U,a,p);}
uint32_t CAN1_ReadSpeed(uint8_t a,CanRxMsg *p,float *rpm){*rpm=(float)(a*13U);return transfer(104U,a,p);}
uint32_t CAN1_ReadPosition(uint8_t a,CanRxMsg *p,uint8_t *sign,uint32_t *raw){*sign=a%2U;*raw=a*1000U;return transfer(105U,a,p);}
uint32_t CAN1_SetMotorEnable(uint8_t a,uint8_t en,CanRxMsg *p){event(en);return transfer(106U,a,p);}
uint32_t CAN1_StopMotor(uint8_t a,CanRxMsg *p){return transfer(107U,a,p);}
uint32_t CAN1_SetEmmSpeedRaw(uint8_t a,uint8_t d,uint16_t speed,uint16_t accel,CanRxMsg *p){event(d);event(speed);event(accel);return transfer(108U,a,p);}
void ChassisAuto_ReportIdleSpeed(uint32_t i,float rpm,uint32_t tick){event(110U);event(i);event((uint32_t)rpm);event(tick);}
void ChassisAuto_Abort(void){event(111U);}
void ChassisAuto_CanStep(void){event(112U);}
void ChassisAuto_SafetyUpdate(uint32_t vm){event(113U);event(vm);}
uint32_t ChassisAuto_Begin(uint32_t dry){event(114U);event(dry);return io_allowed;}
uint32_t Safety_WheelTestStartAllowed(uint32_t s){event(120U);event(s);return io_allowed;}
void Safety_WheelTestStartReport(uint32_t s,uint32_t r){event(121U);event(s);event(r);}
void Safety_WheelTestStopReport(uint32_t s,uint32_t r){event(122U);event(s);event(r);}
void Safety_WheelTestAbort(void){event(123U);}
void Safety_WheelTestPrepareFailed(void){event(124U);}
uint32_t Safety_WheelTestPrepare(void){event(125U);return io_allowed;}
uint32_t Safety_WheelTestArm(uint32_t *s){event(126U);*s=81U;return io_allowed;}
uint32_t Safety_WheelTestTakeStop(uint32_t *s){event(127U);*s=82U;return io_allowed;}
uint32_t Safety_MotionStopTake(uint8_t *a,uint32_t *s){event(128U);*s=3U;*a=2U;return io_allowed;}
void Safety_MotionStopReport(uint8_t a,uint32_t s,uint32_t r){event(129U);event(a);event(s);event(r);}
uint8_t USART1_SendString(const char *s){uint32_t v=0U;event(130U);while(*s)v=v*33U+(uint8_t)*s++;event(v);return (uint8_t)io_allowed;}
int32_t ZdtEmm_RpmDeci(float rpm){return (int32_t)(rpm*10.0f);}
'''
 preamble+='\n'.join('volatile uint32_t '+n+';' for n in externals)+'\n'
 stripped=re.sub(r'^#include .*\r?\n','',mod,flags=re.M)
 rename=''.join('#define '+n+' old_'+n+'\n' for n in owned+list(moved)+["App_DebugSendString"])
 undo=''.join('#undef '+n+'\n' for n in owned+list(moved)+["App_DebugSendString"])
 # Original main only static declarations; renamed to allow independent state.
 harness=preamble+stripped+'\n'+rename+old_unit+'\n'+undo
 harness+='\nvolatile uint32_t assertions,cases;\n#define CHECK(x) do{assertions++;if(!(x))return __LINE__;}while(0)\n'
 harness+='uint32_t compare(void){\n'
 for n in owned:harness+='CHECK(memcmp((const void*)&'+n+', (const void*)&old_'+n+',sizeof('+n+'))==0);\n'
 harness+='return 0U;}\n'
 harness+='void sync(void){\n'
 for n in owned:harness+='memcpy((void*)&old_'+n+', (const void*)&'+n+',sizeof('+n+'));\n'
 harness+='}\n'
 harness+='void environment(uint32_t n){clock_ms=0xFFFFFF00U+n;controller.ESR=n;trace_count=lock_depth=0U;io_result=n%5U==0U?4U:1U;io_allowed=n%13U!=0U;io_flags=n%4U;\n'
 for i,n in enumerate(externals):harness+=f'{n}=((n+{i}U)%29U==0U)?1U:0U;\n'
 harness+='g_vm_test_request=n%37U!=0U;g_safety_ready=n%41U!=0U;g_can_guard_ready=n%43U!=0U;g_rtos_fault=n%47U==0U;g_safety_fault=n%53U==0U;g_motion_diag_fault_latched=g_wheel_test_cut_vm=g_chassis_auto_cut_vm=g_motion_auto_stop_cut_vm=0U;g_wheel_test_state=n%5U==0U?7U:n%5U==1U?1U:n%5U==2U?3U:2U;g_wheel_test_active=n%7U!=0U;}\n'
 harness+=r'''
__attribute__((section("i.test_entry"))) uint32_t test_entry(void){
 uint32_t n,k,result,saved[512],count,time,oldtime;
 CHECK(AppCanDiagnostics_JogPreparing()==&s_can_jog_preparing);
 {const uint32_t *live=AppCanDiagnostics_JogPreparing();s_can_jog_preparing=1U;CHECK(*live==1U);s_can_jog_preparing=0U;CHECK(*live==0U);}
 for(n=0;n<2048U;n++){
  /* Mixed requests exercise the unchanged else-if priority, invalid addresses and failures. */
  environment(n);g_can_stop_address=g_can_disable_address=g_can_enable_address=g_can_zero_address=g_can_position_address=g_can_speed_address=g_can_version_address=n%6U;
  g_can_stop_request=(n&1U)!=0U;g_can_disable_request=(n&2U)!=0U;g_can_enable_request=(n&4U)!=0U;
  g_can_zero_request=(n&8U)!=0U;g_can_position_request=(n&16U)!=0U;g_can_speed_request=(n&32U)!=0U;
  g_can_version_request=(n&64U)!=0U;g_can_status_request=(n&128U)!=0U;g_can_jog_request=(n&256U)!=0U;
  g_can_jog_dry_run=n%3U;g_can_feedback_enable=n%2U;g_can_poll_enable=n%17U!=0U;
  g_motion_auto_stop_state=n%19U==0U?1U:0U;g_motion_diag_fail_stop_once=n%23U==0U;
  s_can_jog_preparing=n%31U==0U;g_can_jog_prepare_stage=1U+n%4U;
  s_can_jog_check_index=s_can_jog_start_index=s_can_jog_stop_index=n%4U;
  s_can_poll_index=n%4U;s_feedback_index=n%4U;s_feedback_phase=n%3U;
  sync();
  for(k=0;k<6U;k++){
   environment(n);switch(k){case 0:AppCanDiagnostics_Dispatch();break;case 1:App_CanFeedbackStep();break;case 2:App_CanPollRefresh();break;case 3:App_CanFeedbackRefresh();break;case 4:App_CanJogService();break;default:App_CanJogStartStep();}
   count=trace_count;memcpy(saved,trace,count*4U);time=clock_ms;CHECK(lock_depth==0U);
   environment(n);switch(k){case 0:OldDispatch();break;case 1:old_App_CanFeedbackStep();break;case 2:old_App_CanPollRefresh();break;case 3:old_App_CanFeedbackRefresh();break;case 4:old_App_CanJogService();break;default:old_App_CanJogStartStep();}
   oldtime=clock_ms;CHECK(time==oldtime);CHECK(lock_depth==0U);CHECK(count==trace_count);CHECK(memcmp(saved,trace,count*4U)==0);
   result=compare();CHECK(result==0U);cases++;
  }
 }
 return 0U;
}
'''
 out=T/'unit';out.mkdir(exist_ok=True)
 (out/'unit.c').write_bytes(harness.encode('gbk'))
 (out/'unit.sct').write_text('LR_ROM 0x08000000 0x100000 { ER_ROM 0x08000000 0x100000 { unit.o (i.test_entry, +First) *(+RO) } RW_RAM 0x20000000 0x20000 { *(+RW,+ZI) } }\n')
 for cmd in [[str(BIN/'armcc.exe'),'--cpu','Cortex-M3','--c99','--split_sections','-O1','-I','.','-c','unit.c','-o','unit.o'],[str(BIN/'armlink.exe'),'--entry','test_entry','--scatter','unit.sct','unit.o','--library_type=microlib','-o','unit.axf']]:
  p=subprocess.run(cmd,cwd=out,capture_output=True,timeout=60);assert p.returncode==0,(p.stdout+p.stderr).decode(errors='replace')
 sys.path.insert(0,str(R/'工创赛/tmp/position_alignment_deps'));import unicorn as uc
 from unicorn import arm_const as reg
 blob=(out/'unit.axf').read_bytes();elf=struct.unpack_from('<16sHHIIIIIHHHHHH',blob);machine=uc.Uc(uc.UC_ARCH_ARM,uc.UC_MODE_THUMB|uc.UC_MODE_MCLASS)
 machine.mem_map(0x08000000,0x100000);machine.mem_map(0x20000000,0x20000)
 for i in range(elf[10]):
  kind,offset,address,_,size,_,_,_=struct.unpack_from('<IIIIIIII',blob,elf[5]+i*elf[9])
  if kind==1 and size:machine.mem_write(address,blob[offset:offset+size])
 stop=0x080FF000;machine.reg_write(reg.UC_ARM_REG_SP,0x2001FFF0);machine.reg_write(reg.UC_ARM_REG_LR,stop|1)
 machine.emu_start(elf[4]|1,stop,timeout=60000000,count=400000000)
 assert machine.reg_read(reg.UC_ARM_REG_PC)==stop,f'did not finish PC={machine.reg_read(reg.UC_ARM_REG_PC):x}'
 assert machine.reg_read(reg.UC_ARM_REG_R0)==0,f'C check line {machine.reg_read(reg.UC_ARM_REG_R0)}'
 symbols=subprocess.check_output([str(BIN/'fromelf.exe'),'--text','-s','unit.axf'],cwd=out).decode(errors='replace')
 def value(n):
  a=int(re.search(r'\b'+n+r'\s+(0x[0-9a-fA-F]+)',symbols)[1],16);return struct.unpack('<I',machine.mem_read(a,4))[0]
 # Use the actual two targets' defines/include paths; compare preprocessed bodies too.
 def short(p):
  b=ctypes.create_unicode_buffer(32768);assert ctypes.windll.kernel32.GetShortPathNameW(str(p),b,32768);return b.value
 project=U.parents[1]/'gongxun-f407.uvprojx'
 targets=ET.parse(project).getroot().findall('./Targets/Target')
 pp=[]
 for target in targets:
  controls=target.find('./TargetOption/TargetArmAds/Cads/VariousControls')
  opts=['--cpu','Cortex-M4.fp.sp','--c99','-E']
  for d in controls.findtext('Define').split(','):opts+=['-D'+d]
  for inc in controls.findtext('IncludePath').split(';'):opts+=['-I',short((project.parent/inc).resolve())]
  units=[]
  for name,p in [('old',T/'main_before.c'),('main',U/'main.c'),('module',U/'app_can_diagnostics.c')]:
   run=subprocess.run([str(BIN/'armcc.exe')]+opts+[short(p)],cwd=T,capture_output=True,timeout=60)
   assert run.returncode==0,run.stderr.decode(errors='replace')
   text=re.sub(r'^#.*$','',run.stdout.decode('gbk'),flags=re.M)
   (T/(target.findtext('TargetName')+'_'+name+'.i')).write_text(text,encoding='utf8')
   units.append(h.functions(text))
  pb,pa,pm=units;count=0
  norm=lambda x:re.sub(r'\s+','',x).removeprefix('static')
  dispatch_body=pm['AppCanDiagnostics_Dispatch'].split('{',1)[1].rsplit('}',1)[0]
  for name,b in pb.items():
   if name in pm:
    assert norm(b)==norm(pm[name]),('preprocessed moved',name)
   elif name in pa:
    a=pa[name].replace('AppCanDiagnostics_Dispatch();',dispatch_body)
    a=re.sub(r's_odometry_inputs.jog_preparing\s*=\s*AppCanDiagnostics_JogPreparing\(\);','',a)
    a=a.replace('(*AppCanDiagnostics_JogPreparing())','s_can_jog_preparing')
    if name=='main':
     old_lines=re.findall(r'App_RtosAssertFailed\((\d+)\)',b);new_lines=re.findall(r'App_RtosAssertFailed\((\d+)\)',a)
     assert len(old_lines)==len(new_lines)==8
     assert all(int(x)-int(y)==924 for x,y in zip(old_lines,new_lines))
     a=re.sub(r'App_RtosAssertFailed\((\d+)\)',lambda m:'App_RtosAssertFailed('+str(int(m[1])+924)+')',a)
    assert norm(a)==norm(b),('preprocessed main',name)
   else:raise AssertionError(('missing preprocessed function',name))
   count+=1
  pp.append(dict(target=target.findtext('TargetName'),function_bodies=count))
 report=dict(ok=True,baseline=BASE,moved_functions=len(moved_names),unchanged_main_functions=checked,owned_symbols=len(owned),cases=value('cases'),assertions=value('assertions'),preprocessed_targets=pp,assert_line_metadata_shift=-924,hardware_accessed=False,limits='Actual CAN application C; CAN/RTOS/clock/Safety/backend external effects stubbed. No real scheduling, bus or hardware validation.')
 (E/'can_diagnostics_check.json').write_text(json.dumps(report,indent=2),encoding='utf8');print(json.dumps(report))

if __name__=='__main__':main()
