"""Actual ARM C test of output selection and preserved heading-input rejection."""
from pathlib import Path
import json, re, struct, subprocess, sys, hashlib
OUT=Path(__file__).resolve().parent;ROOT=OUT.parents[4]
USER=OUT/'firmware/USER';ARM=Path('D:/Keil_v5/ARM/arm5_compiler/bin')
UNIT=r'''
#include "no_heading_trial.h"
#include "heading_controller.h"
volatile unsigned checks, failures;
static void ck(unsigned good){checks++;if(!good)failures++;}
void test_entry(void){
 unsigned mode,d,i;float value,returned,error;HeadingHoldController hold={0};
 const float input[5]={-2.0f,-0.1f,0.0f,0.1f,2.0f};
 for(mode=0;mode<4;mode++)for(d=0;d<2;d++)for(i=0;i<5;i++){
  float expected=(APP_NO_HEADING_FORWARD_TRIAL && mode==0 && d==1)?0.0f:input[i];
  value=input[i];returned=NoHeadingTrialCorrection(mode,d?2100.0f:0.0f,&value);
  ck(returned==expected);ck(value==expected);
 }
 ck(HeadingController_HoldStep(&hold,0.0f,3601.0f,0.0f,100,&value,&error)==0);
 ck(HeadingController_HoldStep(&hold,0.0f,0.0f,3601.0f,100,&value,&error)==0);
}
'''
reports=[]
for enabled in (0,1):
 out=OUT/('unit_'+str(enabled));out.mkdir(exist_ok=True)
 (out/'unit.c').write_text(UNIT)
 (out/'unit.sct').write_text('LR_ROM 0x08000000 0x00100000 { ER_ROM 0x08000000 0x00100000 { unit.o (i.test_entry, +First) *(+RO) } RW_RAM 0x20000000 0x00020000 { *(+RW,+ZI) } }')
 commands=[]
 for src,obj in [('unit.c','unit.o'),(str(USER/'heading_controller.c'),'heading.o')]:
  commands.append([str(ARM/'armcc.exe'),'--cpu','Cortex-M3','--c99','--split_sections','-O1','-DAPP_X_OFFLINE_ONLY=1','-DAPP_NO_HEADING_FORWARD_TRIAL='+str(enabled),'-I',str(USER),'-c',src,'-o',obj])
 commands.append([str(ARM/'armlink.exe'),'--entry','test_entry','--scatter','unit.sct','unit.o','heading.o','--library_type=microlib','-o','unit.axf'])
 logs=[]
 for command in commands:
  p=subprocess.run(command,cwd=out,capture_output=True,text=True,errors='replace',timeout=60)
  logs.append({'command':command,'returncode':p.returncode,'output':p.stdout+p.stderr})
  (out/'build.json').write_text(json.dumps(logs,indent=2))
  assert p.returncode==0,p.stdout+p.stderr
 sys.path.insert(0,str(ROOT/'工创赛/tmp/position_alignment_deps'))
 import unicorn as uc
 from unicorn import arm_const as reg
 m=uc.Uc(uc.UC_ARCH_ARM,uc.UC_MODE_THUMB|uc.UC_MODE_MCLASS)
 m.mem_map(0x08000000,0x100000);m.mem_map(0x20000000,0x20000)
 b=(out/'unit.axf').read_bytes();h=struct.unpack_from('<16sHHIIIIIHHHHHH',b)
 for i in range(h[10]):
  kind,off,addr,_,size,_,_,_=struct.unpack_from('<IIIIIIII',b,h[5]+i*h[9])
  if kind==1 and size:m.mem_write(addr,b[off:off+size])
 symbols=subprocess.check_output([str(ARM/'fromelf.exe'),'--text','-s','unit.axf'],cwd=out,text=True,errors='replace')
 def address(name):return int(re.search(r'\b'+name+r'\s+(0x[0-9a-fA-F]+)',symbols)[1],16)
 stop=0x080ff000;m.reg_write(reg.UC_ARM_REG_SP,0x2001fff0);m.reg_write(reg.UC_ARM_REG_LR,stop|1)
 m.emu_start(address('test_entry')|1,stop,count=100000)
 checks=struct.unpack('<I',m.mem_read(address('checks'),4))[0]
 failures=struct.unpack('<I',m.mem_read(address('failures'),4))[0]
 assert checks==82 and failures==0,(checks,failures)
 reports.append({'enabled':enabled,'checks':checks,'failures':failures})
(OUT/'unit_report.json').write_text(json.dumps(reports,indent=2))
print(json.dumps(reports))
