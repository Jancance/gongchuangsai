"""实际机械臂C：回零后的非零驱动参考必须传入所有伸缩目标。"""
from pathlib import Path
import json,re,struct,subprocess,sys
ROOT=Path(__file__).resolve().parents[2]
FW=ROOT/'3.stm32f407工程'
OLD=ROOT/'工创赛/evidence/integration/2026-11-03/early-scan-startup'
OUT=ROOT/'工创赛/evidence/integration/2026-11-03/startup_extension_home/arm_reference_checks'
ARM=Path('D:/Keil_v5/ARM/arm5_compiler/bin')

unit=(OLD/'arm-unit.c').read_text(encoding='utf8')
unit=unit.replace('volatile uint32_t g_dm_integrated_owner;','volatile uint32_t g_dm_integrated_owner;\nvolatile float g_startup_extension_zero_deg=0.0f;')
unit=unit.replace('static uint32_t checks,failures;', 'volatile uint32_t checks,failures,first_failure;')
unit=re.sub(r'#define C\(x\).*?\n','#define C(x) do{checks++;if(!(x)){failures++;if(!first_failure)first_failure=__LINE__;}}while(0)\n',unit)
unit=unit.replace(' position5=position6=0;', ' g_startup_extension_zero_deg=0;position5=position6=0;')
unit=unit.replace('int main(void)', 'void test_entry(void)')
unit=re.sub(r' printf\("vehicle arm checks=.*?return failures\?1:0;', '',unit)
start=unit.index('#else',unit.index('void test_entry('))+len('#else')
unit=unit[:start]+r'''
 reset();g_startup_extension_zero_deg=-30.4f;position6=-30.4f;
 r=(VehicleArmRow){3,10,106,0};C(VehicleArm_Begin(&r,0));steps(20);
 C(g_vehicle_arm_state==2 && fabsf(position6+66.4f)<0.02f && fabsf(held_extension+66.4f)<0.02f);
 r=(VehicleArmRow){3,0,106,0};C(VehicleArm_Begin(&r,0));steps(20);
 C(g_vehicle_arm_state==2 && fabsf(position6+30.4f)<0.02f && moves==2);
 C(VehicleArm_Begin(&r,0));steps(20);C(g_vehicle_arm_state==2 && moves==2);
'''+unit[start:]

def main():
 sys.path.insert(0,str(ROOT/'工创赛/tmp/position_alignment_deps'))
 import unicorn as uc
 from unicorn import arm_const as reg
 reports=[]
 for offline in [0,1]:
  out=OUT/str(offline);out.mkdir(parents=True,exist_ok=True);(out/'unit.c').write_text(unit,encoding='utf8')
  (out/'unit.sct').write_text('LR_ROM 0x08000000 0x00100000 { ER_ROM 0x08000000 0x00100000 { unit.o (i.test_entry, +First) *(+RO) } RW_RAM 0x20000000 0x00020000 { *(+RW,+ZI) } }')
  inc=[OLD/'arm-stubs',FW/'CHASSIS/USER',FW/'HARDWARE/GRIPPER',FW/'HARDWARE/CARRIER'];logs=[]
  commands=[[str(ARM/'armcc.exe'),'--cpu','Cortex-M3','--c99','--split_sections','-O1',f'-DAPP_MOTOR_OFFLINE_ONLY={offline}',*[a for p in inc for a in ('-I',str(p))],'-c','unit.c','-o','unit.o'],[str(ARM/'armlink.exe'),'--entry','test_entry','--scatter','unit.sct','unit.o','--library_type=microlib','-o','unit.axf']]
  for cmd in commands:
   p=subprocess.run(cmd,cwd=out,capture_output=True,text=True,errors='replace');logs.append({'command':cmd,'returncode':p.returncode,'output':p.stdout+p.stderr});(out/'build.json').write_text(json.dumps(logs,indent=2),encoding='utf8');assert p.returncode==0,p.stdout+p.stderr
  b=(out/'unit.axf').read_bytes();h=struct.unpack_from('<16sHHIIIIIHHHHHH',b);m=uc.Uc(uc.UC_ARCH_ARM,uc.UC_MODE_THUMB|uc.UC_MODE_MCLASS);m.mem_map(0x08000000,0x100000);m.mem_map(0x20000000,0x20000)
  for i in range(h[10]):
   kind,offset,address,_,size,_,_,_=struct.unpack_from('<IIIIIIII',b,h[5]+i*h[9])
   if kind==1 and size:m.mem_write(address,b[offset:offset+size])
  symbols=subprocess.check_output([str(ARM/'fromelf.exe'),'--text','-s','unit.axf'],cwd=out,text=True,errors='replace')
  def addr(n):return int(re.search(r'\b'+n+r'\s+(0x[0-9a-fA-F]+)',symbols)[1],16)
  stop=0x080ff000;m.reg_write(reg.UC_ARM_REG_SP,0x2001fff0);m.reg_write(reg.UC_ARM_REG_LR,stop|1);m.emu_start(addr('test_entry')|1,stop,count=10000000)
  def val(n):return struct.unpack('<I',m.mem_read(addr(n),4))[0]
  row={'offline':offline,'checks':val('checks'),'failures':val('failures'),'first_failure_line':val('first_failure'),'completed':m.reg_read(reg.UC_ARM_REG_PC)==stop,'hardware_accessed':False};reports.append(row);(OUT/'report.json').write_text(json.dumps(reports,indent=2),encoding='utf8');assert row['completed'] and not row['failures'],row
 print(json.dumps(reports))

if __name__=='__main__':main()
