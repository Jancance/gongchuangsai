from pathlib import Path
import re,subprocess,sys,struct,json,hashlib
def run(R,T,B,source,commands):
 logs=[]
 for cmd in commands:
  p=subprocess.run(cmd,cwd=T,capture_output=True);logs.append((p.stdout+p.stderr).decode(errors='replace'));assert p.returncode==0,logs[-1]
 (T/'compile.log').write_text('\n'.join(logs),'utf8')
 (T/'commands.json').write_text(json.dumps(commands,indent=2),'utf8')
 sys.path.insert(0,str(R/'工创赛/tmp/position_alignment_deps'));import unicorn as uc
 from unicorn import arm_const as reg
 blob=(T/'unit.axf').read_bytes();elf=struct.unpack_from('<16sHHIIIIIHHHHHH',blob);m=uc.Uc(uc.UC_ARCH_ARM,uc.UC_MODE_THUMB|uc.UC_MODE_MCLASS);m.mem_map(0x08000000,0x100000);m.mem_map(0x20000000,0x20000);m.mem_write(0x20000000,b'\xa5'*0x20000)
 for i in range(elf[12]):
  _,kind,flags,addr,off,size,_,_,_,_=struct.unpack_from('<IIIIIIIIII',blob,elf[6]+i*elf[11])
  if flags&2 and size:
   if kind==1:m.mem_write(addr,blob[off:off+size])
   elif kind==8:m.mem_write(addr,b'\x00'*size)
 symbols=subprocess.check_output([str(B/'fromelf.exe'),'--text','-s','unit.axf'],cwd=T).decode(errors='replace')
 def val(n):return struct.unpack('<I',m.mem_read(int(re.search(r'\b'+n+r'\s+(0x[0-9a-fA-F]+)',symbols)[1],16),4))[0]
 stop=0x080ff000;m.reg_write(reg.UC_ARM_REG_SP,0x2001fff0);m.reg_write(reg.UC_ARM_REG_LR,stop|1);m.emu_start(elf[4]|1,stop,timeout=60000000,count=300000000);result=m.reg_read(reg.UC_ARM_REG_R0);print('result',result,'cases',val('cases'),'checks',val('assertions'))
 if result:print(source.splitlines()[result-1])
 assert result==0
 inputs=list((R/'3.stm32f407工程/CHASSIS/HARDWARE/RADAR_V6').glob('*.[ch]'))+[R/n for n in ['3.stm32f407工程/CHASSIS/HARDWARE/RADAR_SERVO/radar_lift.c','3.stm32f407工程/CHASSIS/HARDWARE/RADAR_SERVO/radar_lift.h','3.stm32f407工程/CHASSIS/USER/main.c','3.stm32f407工程/gongxun-f407.uvprojx']]
 return {'source_sha256':{str(p.relative_to(R)).replace('\\','/'):hashlib.sha256(p.read_bytes()).hexdigest() for p in inputs},'harness_sha256':hashlib.sha256((T/'unit.c').read_bytes()).hexdigest(),'axf_sha256':hashlib.sha256((T/'unit.axf').read_bytes()).hexdigest(),'passed':True,'cases':val('cases'),'checks':val('assertions'),'loader':'SHF_ALLOC PROGBITS and NOBITS'}
