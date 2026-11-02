from pathlib import Path
import sys,json,shutil,subprocess,struct,re,argparse
R=Path(__file__).resolve().parents[6]
# Locate workspace without depending on the invocation directory.
R=next(p for p in Path(__file__).resolve().parents if (p/'3.stm32f407工程').exists())
sys.path.insert(0,str(R/'工创赛/tmp/position_alignment_deps'))
import unicorn as uc
from unicorn import arm_const as reg
p=argparse.ArgumentParser();p.add_argument('--ki',type=float,required=True);p.add_argument('--out',type=Path,required=True);p.add_argument('--candidate',type=Path,required=True);a=p.parse_args()
out=a.out.resolve();out.mkdir(parents=True,exist_ok=False);(out/'src').mkdir()
source=a.candidate.resolve()/'USER'
for n in ['heading_controller.c','heading_controller.h','heading_integral_trial.h']:shutil.copy2(source/n,out/'src'/n)
shutil.copy2(Path(__file__).with_name('fixture.c'),out/'unit.c')
(out/'unit.sct').write_text('LR_ROM 0x08000000 0x00100000 { ER_ROM 0x08000000 0x00100000 { unit.o (i.test_entry,+First) *(+RO) } RW_RAM 0x20000000 0x00020000 { *(+RW,+ZI) } }')
arm=Path('D:/Keil_v5/ARM/arm5_compiler/bin')
commands=[[str(arm/'armcc.exe'),'--cpu','Cortex-M3','--c99','--split_sections','-O1','-DHEADING_INTEGRAL_TRIAL_ENABLE=1','-DHEADING_INTEGRAL_TRIAL_KI='+str(a.ki)+'f','-c','unit.c','-o','unit.o'],[str(arm/'armlink.exe'),'--entry','test_entry','--scatter','unit.sct','unit.o','--library_type=microlib','-o','unit.axf']]
logs=[]
for cmd in commands:
 r=subprocess.run(cmd,cwd=out,capture_output=True,text=True,errors='replace');logs.append({'command':cmd,'returncode':r.returncode,'output':r.stdout+r.stderr});(out/'build.json').write_text(json.dumps(logs,indent=2));assert r.returncode==0,r.stdout+r.stderr
m=uc.Uc(uc.UC_ARCH_ARM,uc.UC_MODE_THUMB|uc.UC_MODE_MCLASS);m.mem_map(0x08000000,0x100000);m.mem_map(0x20000000,0x20000)
b=(out/'unit.axf').read_bytes();h=struct.unpack_from('<16sHHIIIIIHHHHHH',b)
for i in range(h[10]):
 kind,off,addr,_,size,_,_,_=struct.unpack_from('<IIIIIIII',b,h[5]+i*h[9])
 if kind==1 and size:m.mem_write(addr,b[off:off+size])
symbols=subprocess.check_output([str(arm/'fromelf.exe'),'--text','-s','unit.axf'],cwd=out,text=True)
def address(name):return int(re.search(r'\b'+re.escape(name)+r'\s+(0x[0-9a-fA-F]+)',symbols)[1],16)
stop=0x080ff000;m.reg_write(reg.UC_ARM_REG_SP,0x2001fff0);m.reg_write(reg.UC_ARM_REG_LR,stop|1);m.emu_start(h[4]|1,stop,count=1000000000,timeout=120000000)
value=m.reg_read(reg.UC_ARM_REG_R0);complete=m.reg_read(reg.UC_ARM_REG_PC)==stop
fields=['tail_mean_abs_error_deg','tail_rms_error_deg','peak_abs_error_deg','tail_peak_abs_error_deg','final_yaw_deg','final_integral_rpm','peak_integral_abs_rpm','peak_command_abs_rpm'];values=struct.unpack('<96f',m.mem_read(address('metrics'),96*4));report={'ki':a.ki,'passed':complete and value<0x80000000,'assertions':value if value<0x80000000 else None,'failure_line':value&0x7fffffff if value>=0x80000000 else None,'metrics':[dict(case=i,**dict(zip(fields,values[i*8:(i+1)*8]))) for i in range(12)],'limits':'Actual controller C; synthetic angular plant, not hardware, includes4tick latency, disturbances and load removal.'};(out/'report.json').write_text(json.dumps(report,indent=2));print(json.dumps(report));assert report['passed']
