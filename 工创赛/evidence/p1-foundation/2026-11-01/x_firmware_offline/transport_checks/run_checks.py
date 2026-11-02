from pathlib import Path
import sys,json,shutil,subprocess,struct,argparse
R=next(p for p in Path(__file__).resolve().parents if (p/'3.stm32f407工程').exists())
sys.path.insert(0,str(R/'工创赛/tmp/position_alignment_deps'))
import unicorn as uc
from unicorn import arm_const as reg
p=argparse.ArgumentParser();p.add_argument('--offline-only',type=int,choices=[0,1],required=True);p.add_argument('--out',type=Path,required=True);a=p.parse_args();out=a.out.resolve();out.mkdir(parents=True,exist_ok=False);(out/'src').mkdir()
for n in ['zdt_x_config.h','zdt_x_protocol.h']:shutil.copy2(R/'工创赛/tmp/x_firmware_offline_20261101/USER'/n,out/'src'/n)
shutil.copy2(Path(__file__).with_name('fixture.c'),out/'unit.c');(out/'unit.sct').write_text('LR_ROM 0x08000000 0x00100000 { ER_ROM 0x08000000 0x00100000 { unit.o (i.test_entry,+First) *(+RO) } RW_RAM 0x20000000 0x00020000 { *(+RW,+ZI) } }')
arm=Path('D:/Keil_v5/ARM/arm5_compiler/bin');logs=[]
for cmd in [[str(arm/'armcc.exe'),'--cpu','Cortex-M3','--c99','--split_sections','-O1','-DAPP_X_OFFLINE_ONLY='+str(a.offline_only),'-c','unit.c','-o','unit.o'],[str(arm/'armlink.exe'),'--entry','test_entry','--scatter','unit.sct','unit.o','--library_type=microlib','-o','unit.axf']]:
 r=subprocess.run(cmd,cwd=out,capture_output=True,text=True,errors='replace');logs.append({'command':cmd,'returncode':r.returncode,'output':r.stdout+r.stderr});(out/'build.json').write_text(json.dumps(logs,indent=2));assert not r.returncode,r.stdout+r.stderr
m=uc.Uc(uc.UC_ARCH_ARM,uc.UC_MODE_THUMB|uc.UC_MODE_MCLASS);m.mem_map(0x08000000,0x100000);m.mem_map(0x20000000,0x20000);b=(out/'unit.axf').read_bytes();h=struct.unpack_from('<16sHHIIIIIHHHHHH',b)
for i in range(h[10]):
 kind,off,addr,_,size,_,_,_=struct.unpack_from('<IIIIIIII',b,h[5]+i*h[9])
 if kind==1 and size:m.mem_write(addr,b[off:off+size])
stop=0x080ff000;m.reg_write(reg.UC_ARM_REG_SP,0x2001fff0);m.reg_write(reg.UC_ARM_REG_LR,stop|1);m.emu_start(h[4]|1,stop,count=100000000,timeout=30000000);v=m.reg_read(reg.UC_ARM_REG_R0);ok=m.reg_read(reg.UC_ARM_REG_PC)==stop and v<0x80000000;r={'hardware_accessed':False,'offline_only':a.offline_only,'passed':ok,'assertions':v if v<0x80000000 else None,'failure_line':v&0x7fffffff if v>=0x80000000 else None,'limits':'Actual extractedCAN functions/codec/zero-speed check; registers, time, and bus replies are stubs.'};(out/'report.json').write_text(json.dumps(r,indent=2));print(json.dumps(r));assert ok
