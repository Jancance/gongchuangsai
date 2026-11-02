import json, pathlib, re, shutil, struct, subprocess, sys
ROOT=pathlib.Path(__file__).resolve().parents[5]
BASE=pathlib.Path(__file__).resolve().parent
OUT=BASE/'reference_checks_final'
OUT.mkdir(exist_ok=False)
shutil.copytree(ROOT/'工创赛/tmp/fine_rpm_vehicle_20261031/USER',OUT/'src')
shutil.copy(BASE/'reference_checks.c',OUT/'unit.c')
(OUT/'unit.sct').write_text('LR_ROM 0x08000000 0x00100000 { ER_ROM 0x08000000 0x00100000 { unit.o (i.test_entry, +First) *(+RO) } RW_RAM 0x20000000 0x00020000 { *(+RW,+ZI) } }\n')
arm=pathlib.Path('D:/Keil_v5/ARM/arm5_compiler/bin')
logs=[]
for src,obj in [('unit.c','unit.o'),('src/heading_controller.c','controller.o'),('src/heading_session.c','session.o'),('src/chassis_kinematics.c','kinematics.o')]:
    p=subprocess.run([str(arm/'armcc.exe'),'--cpu','Cortex-M3','--c99','--split_sections','-O1','-I','src','-c',src,'-o',obj],cwd=OUT,capture_output=True,text=True)
    logs.append(dict(source=src,returncode=p.returncode,output=p.stdout+p.stderr))
    assert not p.returncode,logs[-1]
p=subprocess.run([str(arm/'armlink.exe'),'--entry','test_entry','--scatter','unit.sct','unit.o','controller.o','session.o','kinematics.o','--library_type=microlib','-o','unit.axf'],cwd=OUT,capture_output=True,text=True)
logs.append(dict(returncode=p.returncode,output=p.stdout+p.stderr));(OUT/'build.json').write_text(json.dumps(logs,indent=2));assert not p.returncode,logs[-1]
sys.path.insert(0,str(ROOT/'工创赛/tmp/position_alignment_deps'))
import unicorn as uc
from unicorn import arm_const as reg
m=uc.Uc(uc.UC_ARCH_ARM,uc.UC_MODE_THUMB|uc.UC_MODE_MCLASS);m.mem_map(0x08000000,0x100000);m.mem_map(0x20000000,0x20000)
b=(OUT/'unit.axf').read_bytes();h=struct.unpack_from('<16sHHIIIIIHHHHHH',b)
for i in range(h[10]):
    kind,off,addr,_,size,_,_,_=struct.unpack_from('<IIIIIIII',b,h[5]+i*h[9])
    if kind==1 and size:m.mem_write(addr,b[off:off+size])
stop=0x080ff000;m.reg_write(reg.UC_ARM_REG_SP,0x2001fff0);m.reg_write(reg.UC_ARM_REG_LR,stop|1);m.emu_start(h[4]|1,stop,count=100000000)
value=m.reg_read(reg.UC_ARM_REG_R0)
r=dict(passed=value<0x80000000 and m.reg_read(reg.UC_ARM_REG_PC)==stop,assertions=value if value<0x80000000 else None,failure_line=value&0x7fffffff if value&0x80000000 else None,hardware_accessed=False)
(OUT/'report.json').write_text(json.dumps(r,indent=2));print(json.dumps(r));assert r['passed'],r
