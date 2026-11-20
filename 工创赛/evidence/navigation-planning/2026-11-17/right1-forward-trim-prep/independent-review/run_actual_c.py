"""Compile actual pure C and an extracted actual runtime gate; execute ARM offline."""
from pathlib import Path
import hashlib
import json
import re
import struct
import subprocess
import sys

OUT = Path(__file__).resolve().parent
ROOT = next(p for p in OUT.parents if (p / '3.stm32f407工程/gongxun-f407.uvprojx').is_file())
SRC = ROOT / '3.stm32f407工程/CHASSIS/USER'
ARM = Path('D:/Keil_v5/ARM/arm5_compiler/bin')
names = ['chassis_kinematics.c', 'chassis_kinematics.h', 'app_config.h', 'motion_trial40.h',
         'chassis_auto_test.c', 'app_entry.c', 'app_entry.h', 'chassis_odometry.c', 'app_icm_observer.c']
sha = lambda p: hashlib.sha256(p.read_bytes()).hexdigest()
hashes = {str((SRC / n).relative_to(ROOT)): sha(SRC / n) for n in names}
runtime = (SRC / 'chassis_auto_test.c').read_bytes().decode('cp936')
gate = re.search(r'#if APP_BASELINE40_ACTIVE && APP_RIGHT40_FORWARD_TRIM_ENABLE\s+(.*?)\?\s+Chassis_Right40ForwardCommands', runtime, re.S).group(1).strip()
entry = (SRC / 'app_entry.c').read_bytes().decode('cp936')
entry_function = re.search(r'uint32_t AppDebugPlan40Allowed\([^\n]*\)\s*\{.*?\n\}', entry, re.S).group(0)
if (OUT / 'build.json').exists() and not (OUT / 'build_attempt1_full_entry.json').exists():
    (OUT / 'build_attempt1_full_entry.json').write_bytes((OUT / 'build.json').read_bytes())
(OUT / 'entry_scope.c').write_text('#include "app_entry.h"\n#include "motion_trial20.h"\n' + entry_function + '\n', 'ascii')
prefix = '''#include <stdint.h>
#include <math.h>
#include "chassis_kinematics.h"
#include "app_entry.h"
#include "motion_trial20.h"
static uint32_t gate_actual(uint32_t g_debug_plan_id,uint32_t s_translation_mode,
 uint32_t s_distance_max_rpm,float s_distance_target_mm,uint32_t s_route_trial) {
#if APP_BASELINE40_ACTIVE && APP_RIGHT40_FORWARD_TRIM_ENABLE
 return (''' + gate + ''') ? 1U : 0U;
#else
 return 0U;
#endif
}
'''
tests = r'''
volatile float maxima[5];
volatile uint32_t old_checksum=2166136261U,command_cases,gate_cases,gate_enabled;
static uint32_t checks;
#define CHECK(x) do {checks++;if(!(x))return 0x80000000U|__LINE__;}while(0)
static float ab(float v){return v<0?-v:v;}
static void maxf(uint32_t n,float v){if(v>maxima[n])maxima[n]=v;}
static float decode(ChassisMotorCommand *c,uint32_t i){
 float q=(float)c[i].speed_raw*0.1f;
 return c[i].direction==g_chassis_forward_direction[i]?q:-q;
}
static void dirty(ChassisMotorCommand *c){uint32_t i;for(i=0;i<4;i++)c[i].speed_raw=1234U;}
static uint32_t zero(ChassisMotorCommand *c){uint32_t i;for(i=0;i<4;i++)if(c[i].speed_raw)return 0U;return 1U;}
uint32_t test_entry(void) {
 uint32_t b,i,j,k,mode,id,rpm,route,n; int32_t hi;
 float h,t,q[4],ideal[4],F,L,H,requested,mm,expected,A,J;
 float lengths[7]={300,1500,2100,299,1501,2099,0};
 float special[8]={-.051f,-.05f,-.049f,.049f,.05f,.051f,-1.999f,1.999f};
 union {uint32_t u;float f;} nan,inf;
 ChassisMotorCommand c[4]; nan.u=0x7fc00000U;inf.u=0x7f800000U;lengths[6]=nan.f;
 for(b=0;b<=102;b++)for(hi=-200;hi<=208;hi++) {
  requested=hi<=200?(float)hi*.01f:special[hi-201];
  h=requested;if(h>(float)b)h=(float)b;if(h<-(float)b)h=-(float)b;
  CHECK(Chassis_Right40ForwardCommands(b,requested,c)==1U);command_cases++;
  t=.01f*(float)b;
  ideal[0]=-(float)b+t-h;ideal[1]=(float)b+t+h;
  ideal[2]=(float)b+t-h;ideal[3]=-(float)b+t+h;
  for(i=0;i<4;i++) {
   q[i]=decode(c,i);CHECK(c[i].address==i+1U);CHECK(c[i].speed_raw<=1050U);
   maxf(0,ab(q[i]-ideal[i]));CHECK(ab(q[i]-ideal[i])<=.0501f);maxf(4,ab(q[i]));
  }
  F=(q[0]+q[1]+q[2]+q[3])*.25f;L=(q[0]-q[1]-q[2]+q[3])*.25f;
  H=(-q[0]+q[1]-q[2]+q[3])*.25f;
  maxf(1,ab(F-t));maxf(2,ab(L+(float)b));maxf(3,ab(H-h));
  CHECK(ab(F-t)<=.0501f&&ab(L+(float)b)<=.0501f&&ab(H-h)<=.0501f);
  CHECK(F>=-.00001f&&q[0]<=0&&q[1]>=0&&q[2]>=0&&q[3]<=0);
  if(!b)CHECK(zero(c));
  /* Existing left integer trim remains exactly as before. */
  CHECK(Chassis_Left40DriftCommands(b,requested,c)==1U);
  t=(float)(uint32_t)(.02f*(float)b+.5f);
  ideal[0]=(float)b-t-h;ideal[1]=-(float)b-t+h;
  ideal[2]=-(float)b-t-h;ideal[3]=(float)b-t+h;
  for(i=0;i<4;i++)CHECK(ab(decode(c,i)-ideal[i])<=.0501f);
  /* Record all generic mode outputs for enable0/1 comparison. */
  for(mode=0;mode<4;mode++) {
   CHECK(Chassis_TranslationProfileCommands(mode,b,requested,c)==1U);
   for(i=0;i<4;i++) {old_checksum^=c[i].speed_raw|((uint32_t)c[i].direction<<16);old_checksum*=16777619U;}
  }
 }
 for(i=0;i<7;i++) {
  float bad[7]={-2.001f,2.001f,nan.f,inf.f,-inf.f,0,0};
  dirty(c);CHECK(Chassis_Right40ForwardCommands(i==5?103U:(i==6?0xffffffffU:100U),bad[i],c)==0U);CHECK(zero(c));
 }
 CHECK(Chassis_Right40ForwardCommands(10,0,0)==0U);
 for(id=0;id<=22;id++)for(mode=0;mode<=4;mode++)for(rpm=100;rpm<=103;rpm++)
 for(j=0;j<7;j++)for(route=0;route<=1;route++) {
  mm=lengths[j];n=gate_actual(id,mode,rpm,mm,route);gate_cases++;gate_enabled+=n;
  k=APP_RIGHT40_FORWARD_TRIM_ENABLE&&!route&&mode==2U&&rpm==102U&&
    ((id==10U&&mm==2100)||(id==12U&&mm==1500)||(id==13U&&mm==300));
  CHECK(n==k);
 }
 CHECK(gate_enabled==(APP_RIGHT40_FORWARD_TRIM_ENABLE?3U:0U));
 for(mode=0;mode<=4;mode++)for(rpm=100;rpm<=103;rpm++)for(j=0;j<7;j++) {
  mm=lengths[j];k=(mm==300||mm==1500||mm==2100)&&
    (((mode==0||mode==3)&&rpm==100)||((mode==1||mode==2)&&rpm==102));
  expected=k?150.0f:APP_DISTANCE_SCURVE_ACCEL_MM_S2;
  if(k&&mode==1)expected=150.0f/1.02f;
  if(k&&mode==2&&APP_RIGHT40_FORWARD_TRIM_ENABLE)expected=150.0f/1.01f;
  A=MotionDistanceAcceleration(mode,rpm,mm);CHECK(ab(A-expected)<.0001f);
  expected=k?300.0f:APP_DISTANCE_SCURVE_JERK_MM_S3;
  if(k&&mode==1)expected=300.0f/1.02f;
  if(k&&mode==2&&APP_RIGHT40_FORWARD_TRIM_ENABLE)expected=300.0f/1.01f;
  J=MotionDistanceJerk(mode,rpm,mm);CHECK(ab(J-expected)<.0001f);
  if(k)CHECK(MotionDistanceFeedbackLimit(mode,rpm,mm)==130U);
 }
 return checks;
}
'''
(OUT / 'unit.c').write_text(prefix + tests, 'ascii')
(OUT / 'guard.c').write_text('#include "app_config.h"\nint x;\n', 'ascii')
(OUT / 'unit.sct').write_text('LR_ROM 0x08000000 0x00100000 { ER_ROM 0x08000000 0x00100000 { unit.o (i.test_entry, +First) *(+RO) } RW_RAM 0x20000000 0x00020000 { *(+RW,+ZI) } }\n', 'ascii')
common = ['--cpu', 'Cortex-M4', '--c99', '--split_sections', '-O1', '-I', str(SRC)]
defs = ['-DVEHICLE_INTEGRATED_ENABLE=0', '-DVEHICLE_STAGE1_ENABLE=0', '-DAPP_BASELINE40_ENABLE=1', '-DAPP_LEFT40_DRIFT_TRIM_ENABLE=1']
logs = []; results = []
def run(cmd, label, expected_success=True):
    p = subprocess.run(cmd, cwd=OUT, capture_output=True, text=True, errors='replace')
    logs.append(dict(label=label, command=cmd, returncode=p.returncode, output=p.stdout+p.stderr))
    (OUT / 'build.json').write_text(json.dumps(logs, ensure_ascii=False, indent=2), 'utf8')
    if (p.returncode == 0) != expected_success: raise RuntimeError(label+'\n'+p.stdout+p.stderr)
    return p.stdout+p.stderr
sys.path.insert(0, str(ROOT / '工创赛/tmp/position_alignment_deps'))
import unicorn as uc
from unicorn import arm_const as reg
for enabled in (0, 1):
    flag = [f'-DAPP_RIGHT40_FORWARD_TRIM_ENABLE={enabled}']
    for source, target in [(OUT/'unit.c','unit.o'),(SRC/'chassis_kinematics.c','kinematics.o'),(OUT/'entry_scope.c','entry.o')]:
        run([str(ARM/'armcc.exe')]+common+defs+flag+['-c',str(source),'-o',target],f'enable{enabled}_{target}')
    run([str(ARM/'armlink.exe'),'--entry','test_entry','--scatter','unit.sct','unit.o','kinematics.o','entry.o','--library_type=microlib','-o',f'unit{enabled}.axf'],f'enable{enabled}_link')
    blob=(OUT/f'unit{enabled}.axf').read_bytes();hdr=struct.unpack_from('<16sHHIIIIIHHHHHH',blob)
    m=uc.Uc(uc.UC_ARCH_ARM,uc.UC_MODE_THUMB|uc.UC_MODE_MCLASS);m.mem_map(0x08000000,0x100000);m.mem_map(0x20000000,0x20000)
    # Load allocated ELF sections at their execution address, including initialized RW data.
    for i in range(hdr[12]):
        _,kind,flags,addr,off,size,*_=struct.unpack_from('<IIIIIIIIII',blob,hdr[6]+i*hdr[11])
        if flags&2 and kind!=8 and size:m.mem_write(addr,blob[off:off+size])
    stop=0x080ff000;m.reg_write(reg.UC_ARM_REG_SP,0x2001fff0);m.reg_write(reg.UC_ARM_REG_LR,stop|1)
    m.emu_start(hdr[4]|1,stop,count=1200000000)
    value=m.reg_read(reg.UC_ARM_REG_R0)
    symbols=subprocess.check_output([str(ARM/'fromelf.exe'),'--text','-s',f'unit{enabled}.axf'],cwd=OUT,text=True)
    def address(name):return int(re.search(r'\b'+name+r'\s+(0x[0-9a-fA-F]+)',symbols)[1],16)
    def readu(name):return struct.unpack('<I',m.mem_read(address(name),4))[0]
    stats=struct.unpack('<5f',m.mem_read(address('maxima'),20))
    result=dict(enabled=enabled,passed=not(value&0x80000000) and m.reg_read(reg.UC_ARM_REG_PC)==stop,
        assertions=value if not value&0x80000000 else None,failure_line=value&0x7fffffff if value&0x80000000 else None,
        command_cases=readu('command_cases'),gate_cases=readu('gate_cases'),gate_enabled=readu('gate_enabled'),
        old_generic_checksum=readu('old_checksum'),max_wheel_error_rpm=stats[0],max_component_error_F_L_H_rpm=list(stats[1:4]),max_wheel_rpm=stats[4])
    results.append(result);(OUT/'actual_c_report.json').write_text(json.dumps(dict(status='RUNNING',results=results),indent=2),'utf8')
    if not result['passed']:raise RuntimeError(str(result))
assert results[0]['old_generic_checksum']==results[1]['old_generic_checksum']
guards=[('flag2',defs+['-DAPP_RIGHT40_FORWARD_TRIM_ENABLE=2']),
        ('nonbaseline',[d for d in defs if not d.startswith('-DAPP_BASELINE40_ENABLE=')]+['-DAPP_BASELINE40_ENABLE=0','-DAPP_RIGHT40_FORWARD_TRIM_ENABLE=1'])]
for label,options in guards:
    output=run([str(ARM/'armcc.exe')]+common+options+['-c','guard.c','-o','guard.o'],label,False)
    assert 'Right40' in output
assert all(sha(ROOT/name)==digest for name,digest in hashes.items())
report=dict(status='PASS_ACTUAL_C_ARITHMETIC_AND_EXTRACTED_GATE',results=results,source_sha256=hashes,
    runtime_gate_text=gate,runtime_gate_sha256=hashlib.sha256(gate.encode('ascii')).hexdigest(),
    entry_function_text=entry_function,entry_function_sha256=hashlib.sha256(entry_function.encode('ascii')).hexdigest(),
    negative_compile_guards=[g[0] for g in guards],hardware_accessed=False,product_modified=False,
    limits='Actual full kinematics and motion_trial40 headers run offline. AppDebugPlan40Allowed and runtime gate are extracted verbatim, not model replacements. Full app_entry compile failed on existing empty route initializer; first failure saved. No full RTOS/CAN/STOP execution, dynamics or physical acceptance.')
(OUT/'actual_c_report.json').write_text(json.dumps(report,ensure_ascii=False,indent=2),'utf8')
print(json.dumps({k:v for k,v in report.items() if k not in ('source_sha256','runtime_gate_text')},ensure_ascii=False,indent=2))
