from pathlib import Path
import ctypes,hashlib,importlib.util,json,re,struct,subprocess,sys,xml.etree.ElementTree as ET
R=Path.cwd();E=Path(__file__).resolve().parent;U=R/'3.stm32f407工程/CHASSIS/USER';B=Path('D:/Keil_v5/ARM/arm5_compiler/Bin')
def extract(s,signature):
 start=s.index(signature);i=s.index('{',start);depth=1;j=i+1
 while depth:
  if s[j]=='{':depth+=1
  elif s[j]=='}':depth-=1
  j+=1
 return s[start:j]
serial=((E/'app_serial_runtime.c.before') if '--before' in sys.argv else (U/'app_serial_runtime.c')).read_bytes().decode('gbk').replace('\r\n','\n');motion=(U/'app_motion_runtime.c').read_bytes().decode('gbk').replace('\r\n','\n')
prefix=r"""
#include <stdint.h>
#include <stdio.h>
#include <string.h>
#include "app_config.h"
#include "app_entry.h"
#include "motion_control.h"
#define taskENTER_CRITICAL() ((void)0)
#define taskEXIT_CRITICAL() ((void)0)
#define RQ_LINE_MAX 240U
#define HR_WAIT 1U
#define HR_RUN 2U
#define HR_IDLE 0U
#define HR_DONE 3U
static uint32_t clock_ms,permit,abort_count,power_disable_count,heading_trip_count,forward_trip_count;
static uint32_t xTaskGetTickCount(void){return clock_ms;}
static struct {uint32_t state,request,index,cause,completed;} g_combined_test;
static struct {uint32_t phase,request,trip;} g_forward_hold;
static uint32_t s_motion_session_open,s_combined_keep_vm,s_combined_end_ms,s_combined_turn;
static uint32_t g_vm_test_request,g_heading_exec_request;
static uint32_t s_serial_token_seq,s_serial_token_tick,s_serial_arm_ms,s_serial_keepalive_ms,s_serial_tx_seq;
static uint32_t g_serial_motion_state,g_serial_motion_result,g_serial_motion_rejected,g_serial_motion_accepted,g_serial_motion_link_lost;
volatile uint32_t g_debug_plan_id;
static uint32_t action_count=1;
static struct {const uint32_t *action_count;} live={&action_count};
static struct {uint32_t staging,ready,consumed;struct {uint32_t id,count;struct {uint32_t timeout_ms;} rows[8];} committed;} route;
#define AppMotion_Live() (&live)
#define AppRuntimeRoute_Live() (&route)
static uint32_t s_combined_begin_ms,s_combined_stage_ms;
static uint32_t g_can_ctrl_fault_latched,g_safety_ready,g_can_guard_ready,source_healthy=1;
static const uint32_t s_heading_runtime_inputs=0,s_translation_runtime_inputs=0;
static uint32_t App_HeadingRuntimeSource(uint32_t now){return source_healthy;}
static uint32_t App_HeadingRuntimeExclusive(void){return 1;}
static void App_HeadingRuntimeSafety(const void *p){}
static void App_ForwardHoldSafety(const void *p){}
uint32_t MotionGetState(void){return MOTION_BUSY;}
static uint32_t FieldMap_LocalMode(void){return 0;}
static void App_RuntimeRouteCommand(char *p){(void)p;}
static uint32_t App_SerialIdleReady(uint32_t now){(void)now;return 1;}
static uint32_t App_SerialCompletedStopped(void){return 0;}
static uint32_t AppMotion_SetStartPermit(uint32_t p){permit=p;return 1;}
static uint32_t App_DebugSelectPlan(uint32_t id){MotionAction a[8];uint32_t n;if(!AppDebugPlan_Load(id,a,&n))return 0;g_debug_plan_id=id;return 1;}
static void App_HeadingRuntimeTrip(uint32_t reason){heading_trip_count++;}
static void App_ForwardHoldTrip(uint32_t reason){forward_trip_count++;}
static void ChassisAuto_Abort(void){abort_count++;}
static void Power_Disable(void){g_vm_test_request=0;power_disable_count++;}
"""
source='\n'.join([extract(serial,'uint16_t App_SerialCrc('),extract(serial,'static uint32_t App_SerialUInt('),extract(serial,'static void App_SerialReject('),extract(motion,'void App_CombinedTestStop('),extract(motion,'void App_CombinedTestSafety('),extract(serial,'static void App_SerialMotionCommand('),extract(serial,'void App_SerialMotionByte('),extract(serial,'void App_SerialMotionSafety(void)\n{')])
tests=r"""
volatile uint32_t review_assertions;
#define CK(x) do{review_assertions++;if(!(x))return __LINE__;}while(0)
static void send_frame(const char *verb,uint32_t seq,uint32_t tick,uint32_t corrupt,uint32_t incomplete){
 char line[128];uint32_t n,i;uint16_t crc;
 n=(uint32_t)sprintf(line,"RC1,%lu,%lu,%s,",(unsigned long)seq,(unsigned long)tick,verb);crc=App_SerialCrc(line,n);
 n+=(uint32_t)sprintf(line+n,"%04X\n",corrupt?(crc^1U):crc);
 for(i=0;i<n-(incomplete?1U:0U);i++)App_SerialMotionByte((uint8_t)line[i]);
}
static void reset(uint32_t plan,uint32_t running){
 App_SerialMotionByte('\n');
 memset(&g_combined_test,0,sizeof(g_combined_test));memset(&g_forward_hold,0,sizeof(g_forward_hold));memset(&route,0,sizeof(route));
 clock_ms=10000;s_combined_begin_ms=s_combined_stage_ms=10000;source_healthy=1;g_can_ctrl_fault_latched=0;g_safety_ready=g_can_guard_ready=1;g_debug_plan_id=plan;g_serial_motion_state=running?2:0;g_serial_motion_result=running?2:0;
 s_serial_token_seq=7;s_serial_token_tick=9000;s_serial_arm_ms=s_serial_keepalive_ms=10000;s_serial_tx_seq=8;
 g_serial_motion_rejected=g_serial_motion_accepted=g_serial_motion_link_lost=0;
 permit=running;abort_count=power_disable_count=heading_trip_count=forward_trip_count=0;
 g_combined_test.state=running;g_vm_test_request=running;s_motion_session_open=s_combined_keep_vm=running;s_combined_turn=1;
}
__attribute__((section("i.test_entry"))) uint32_t test_entry(void){
 uint32_t id,boundary,origin,last,i;MotionAction a[8];uint32_t count;
 for(id=8;id<=21;id++){
  boundary=AppDebugTurn90Plan(id)?3000:1000;
  reset(id,1);clock_ms+=boundary-1;App_SerialMotionSafety();CK(g_serial_motion_state==2 && permit==1 && !abort_count);
  clock_ms++;App_SerialMotionSafety();CK(g_serial_motion_state==3 && !permit && g_serial_motion_result==8 && g_serial_motion_link_lost==1);
  CK(abort_count==1 && power_disable_count==1 && heading_trip_count==1 && !g_vm_test_request && g_combined_test.state==5);
  send_frame("KEEP",7,9000,0,0);send_frame("START",7,9000,0,0);CK(g_serial_motion_state==3 && !permit && abort_count==1 && !g_vm_test_request);
 }
 for(id=20;id<=21;id++){
  boundary=AppDebugTurn90Plan(id)?3000:1000;
  reset(id,1);clock_ms+=boundary-1;send_frame("KEEP",7,9000,0,0);CK(s_serial_keepalive_ms==clock_ms && g_serial_motion_state==2);
  clock_ms+=boundary-1;App_SerialMotionSafety();CK(g_serial_motion_state==2);clock_ms++;App_SerialMotionSafety();CK(g_serial_motion_result==8 && !permit);
  for(i=0;i<4;i++){
   reset(id,1);last=s_serial_keepalive_ms;clock_ms+=boundary-1;
   if(i==0)send_frame("KEEP",6,9000,0,0);
   if(i==1)send_frame("KEEP",7,9000,1,0);
   if(i==2)send_frame("KEEP",7,9000,0,1);
   if(i==3)send_frame("START",7,9000,0,0);
   CK(s_serial_keepalive_ms==last);clock_ms++;App_SerialMotionSafety();CK(g_serial_motion_result==8 && !permit && abort_count==1);
  }
  reset(id,1);clock_ms+=1;send_frame("STOP",123,0,0,0);CK(g_serial_motion_state==3 && g_serial_motion_result==3 && !permit && abort_count==1 && !g_vm_test_request);
  reset(id,0);clock_ms=10000;send_frame(id==20?"PLAN20":"PLAN21",7,9999,0,0);CK((g_serial_motion_accepted==1)==AppDebugTurn90Plan(id));
  reset(id,0);send_frame("ARM",7,9999,0,0);CK((g_serial_motion_state==1)==AppDebugTurn90Plan(id));
  reset(id,1);clock_ms++;source_healthy=0;App_CombinedTestSafety();CK(abort_count==1 && !g_vm_test_request && g_combined_test.state==4);
  reset(id,1);clock_ms++;g_can_ctrl_fault_latched=1;App_CombinedTestSafety();CK(abort_count==1 && !g_vm_test_request && g_combined_test.state==4);
 }
 /* ARM lifetime stays exactly 2000 ms, independently of keepalive profile. */
 for(i=0;i<2;i++){
  reset(8,0);send_frame("ARM",7,9999,0,0);CK(g_serial_motion_state==1 && s_serial_token_seq==7 && s_serial_token_tick==9999);
  clock_ms+=i?2000:1999;send_frame("START",7,9999,0,0);
  CK(i?(g_serial_motion_state==1 && !permit && g_serial_motion_result==6):(g_serial_motion_state==2 && permit));
 }
 reset(20,0);g_serial_motion_state=1;permit=1;clock_ms+=1999;App_SerialMotionSafety();CK(g_serial_motion_state==1 && permit);
 clock_ms++;App_SerialMotionSafety();CK(g_serial_motion_state==0 && !permit && g_serial_motion_result==7);
 /* ARM token freshness is strict 500 ms. */
 reset(8,0);send_frame("ARM",7,9501,0,0);CK(g_serial_motion_state==1);
 reset(8,0);send_frame("ARM",7,9500,0,0);CK(g_serial_motion_state==0 && g_serial_motion_result==5);
 /* Unsigned keepalive age still handles wrap. */
 for(id=20;id<=21;id++){
  reset(id,1);boundary=AppDebugTurn90Plan(id)?3000:1000;s_serial_keepalive_ms=0xFFFFFF00U;clock_ms=s_serial_keepalive_ms+boundary-1;App_SerialMotionSafety();CK(g_serial_motion_state==2);
  clock_ms++;App_SerialMotionSafety();CK(g_serial_motion_state==3 && !permit);
 }
#if !APP_TURN90_90_ACTIVE
 CK(!AppDebugPlan_Load(20,a,&count) && !AppDebugPlan_Load(21,a,&count));
#endif
 return 0;
}
"""
unit=prefix+source+tests
control=ET.parse(U.parents[1]/'gongxun-f407.uvprojx').getroot().find('./Targets/Target/TargetOption/TargetArmAds/Cads/VariousControls')
def short(p):
 b=ctypes.create_unicode_buffer(32768);assert ctypes.windll.kernel32.GetShortPathNameW(str(p.resolve()),b,len(b));return b.value
opts=['--cpu','Cortex-M3','--c99','--gnu','--split_sections','-O1','--diag_suppress=177,550','-DAPP_VM_CALIBRATION_ONLY_ENABLE=0','-DAPP_VM_ONLY_ENABLE=0','-DAPP_WHEEL_ENABLE_HOLD_ENABLE=0','-DAPP_BASELINE40_ENABLE=1','-DVEHICLE_INTEGRATED_ENABLE=0','-DVEHICLE_STAGE1_ENABLE=0']
opts+=['-D'+d for d in control.findtext('Define').split(',')]
for inc in control.findtext('IncludePath').split(';'):opts+=['-I',short(U.parents[1]/inc)]
sys.path.insert(0,str(R/'工创赛/tmp/position_alignment_deps'));import unicorn as uc
from unicorn import arm_const as reg
report={'hardware_accessed':False,'actual_c':{},'source_sha256':hashlib.sha256((U/'app_serial_runtime.c').read_bytes()).hexdigest()}
for enabled in [1,0]:
 T=E/(('red-' if '--before' in sys.argv else '')+('actual-c-on' if enabled else 'actual-c-off'));T.mkdir(exist_ok=True);(T/'unit.c').write_bytes(unit.encode('gbk'))
 def run(cmd,log):
  cp=subprocess.run(cmd,cwd=T,capture_output=True,timeout=90);(T/log).write_bytes(cp.stdout+cp.stderr);assert cp.returncode==0,(cp.stdout+cp.stderr).decode('gbk',errors='replace')
 objects=[]
 for p in [T/'unit.c',U/'app_entry.c',U/'chassis_kinematics.c',U/'motion_profile.c']:
  obj=p.stem+'.o';objects.append(obj);run([str(B/'armcc.exe'),*opts,'-DAPP_TURN_BRAKE_TRIAL_DPS='+str(90 if enabled else 30),'-DAPP_TURN90_90_ENABLE='+str(enabled),'-c',short(p),'-o',obj],obj+'.log')
 (T/'unit.sct').write_text('LR_ROM 0x08000000 0x100000 { ER_ROM 0x08000000 0x100000 { unit.o (i.test_entry, +First) *(+RO) } RW_RAM 0x20000000 0x50000 { *(+RW,+ZI) } }\n')
 run([str(B/'armlink.exe'),'--entry','test_entry','--scatter','unit.sct',*objects,'--library_type=microlib','-o','unit.axf'],'link.log')
 blob=(T/'unit.axf').read_bytes();elf=struct.unpack_from('<16sHHIIIIIHHHHHH',blob);machine=uc.Uc(uc.UC_ARCH_ARM,uc.UC_MODE_THUMB|uc.UC_MODE_MCLASS)
 for addr,size in [(0x08000000,0x100000),(0x20000000,0x50000),(0xE0000000,0x100000)]:machine.mem_map(addr,size)
 for n in range(elf[12]):
  _,kind,flags,addr,offset,size,*_=struct.unpack_from('<IIIIIIIIII',blob,elf[6]+n*elf[11])
  if flags&2 and size:machine.mem_write(addr,b'\0'*size if kind==8 else blob[offset:offset+size])
 stop=0x080FF000;machine.reg_write(reg.UC_ARM_REG_SP,0x2004FFF0);machine.reg_write(reg.UC_ARM_REG_LR,stop|1);machine.emu_start(elf[4]|1,stop,timeout=60000000,count=200000000);assert machine.reg_read(reg.UC_ARM_REG_PC)==stop
 result=machine.reg_read(reg.UC_ARM_REG_R0);symbols=subprocess.check_output([str(B/'fromelf.exe'),'--text','-s','unit.axf'],cwd=T).decode(errors='replace');address=int(re.search(r'\breview_assertions\s+(0x[0-9a-fA-F]+)',symbols)[1],16)
 assertions=struct.unpack('<I',machine.mem_read(address,4))[0];assert result==0,(enabled,result,unit.splitlines()[result-1])
 report['actual_c'][str(enabled)]={'status':'PASS','assertions':assertions}
# Real isolated client's timing behavior; no COM instantiation or main execution.
client=E/'p1_serial_motion_turn90_keepalive3s.py';spec=importlib.util.spec_from_file_location('client',client);m=importlib.util.module_from_spec(spec);spec.loader.exec_module(m);host_checks=0

def ck(v):
 global host_checks;host_checks+=1;assert v,host_checks

def controller(plan,stage,start=True):
 c=m.Controller('run-once',3,plan,expected_complete_powered=True);c.stage=stage;c.stage_tick=0;c.last_rx=0;c.last_keep=0;c.start_sent=start;return c
for plan in [20,21]:
 c=controller(plan,'start_wait');ck(c.tick(.199)==[]);ck(any(b',KEEP,' in f for f in c.tick(.2)))
 ck(c.tick(2.01)==[m.command('KEEP',c.token)] and c.status is None)
 ck(c.tick(3.99)==[m.command('KEEP',c.token)] and c.status is None)
 ck(c.tick(4.0)==[m.command('STOP',c.token)] and c.status=='ack_timeout');ck(c.tick(4.1)==[])
 for stage in ['plan_wait','arm_wait','stop_wait']:
  c=controller(plan,stage,False);ck(c.tick(2.0)==[m.command('STOP',c.token)] and c.status=='ack_timeout')
 c=controller(plan,'running');ck(c.tick(4.999)==[m.command('KEEP',c.token)] and c.status is None);ck(c.tick(5.0)==[m.command('STOP',c.token)] and c.status=='telemetry_timeout')
for plan in [8,19]:
 c=controller(plan,'start_wait');ck(c.tick(2.0)==[m.command('STOP',c.token)] and c.status=='ack_timeout')
report.update(status='PASS',host_checks=host_checks,limitations=['RTOS scheduling, RF transport and physical stop are not simulated.','App_SerialIdleReady and low-level CAN/VM effects use bounded stubs; real parser, byte accumulator, safety and CombinedTestStop dispatch are executed.','One-way no-RX while uplink KEEP arrives can take about 5+3=8 seconds to command cancellation if STOP is lost; not a universal 3-second stop guarantee.'])
(E/'checks.json').write_text(json.dumps(report,indent=2),encoding='utf8');print(json.dumps(report))
