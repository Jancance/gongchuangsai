from pathlib import Path
import subprocess,sys,struct,json,argparse,re
R=Path.cwd();sys.path.insert(0,str(next(R.glob('*/tmp/position_alignment_deps'))))
import unicorn as uc
from unicorn import arm_const as reg
p=argparse.ArgumentParser();p.add_argument('kind',choices=['logger','protocol','export','integral_export']);a=p.parse_args()
D=Path(__file__).parent/a.kind;arm=Path('D:/Keil_v5/ARM/arm5_compiler/bin')
(D/'unit.sct').write_text('LR_ROM 0x08000000 0x00100000 { ER_ROM 0x08000000 0x00100000 { unit.o (i.test_entry,+First) *(+RO) } RW_RAM 0x20000000 0x00020000 { *(+RW,+ZI) } }')
logs=[]
for cmd in [[str(arm/'armcc.exe'),'--cpu','Cortex-M3','--c99','--split_sections','-O1','-c','unit.c','-o','unit.o'],[str(arm/'armlink.exe'),'--entry','test_entry','--scatter','unit.sct','unit.o','--library_type=microlib','-o','unit.axf']]:
 x=subprocess.run(cmd,cwd=D,capture_output=True,text=True);logs.append({'command':cmd,'returncode':x.returncode,'output':x.stdout+x.stderr});(D/'build.json').write_text(json.dumps(logs,indent=2));assert x.returncode==0,x.stdout+x.stderr
m=uc.Uc(uc.UC_ARCH_ARM,uc.UC_MODE_THUMB|uc.UC_MODE_MCLASS);m.mem_map(0x08000000,0x100000);m.mem_map(0x20000000,0x20000);b=(D/'unit.axf').read_bytes();h=struct.unpack_from('<16sHHIIIIIHHHHHH',b)
for i in range(h[10]):
 kind,off,addr,_,size,_,_,_=struct.unpack_from('<IIIIIIII',b,h[5]+i*h[9])
 if kind==1 and size:m.mem_write(addr,b[off:off+size])
stop=0x080ff000;m.reg_write(reg.UC_ARM_REG_SP,0x2001fff0);m.reg_write(reg.UC_ARM_REG_LR,stop|1);m.emu_start(h[4]|1,stop,count=10000000,timeout=10000000)
v=m.reg_read(reg.UC_ARM_REG_R0);ok=m.reg_read(reg.UC_ARM_REG_PC)==stop and v==100
sy_debug=subprocess.check_output([str(arm/'fromelf.exe'),'--text','-s','unit.axf'],cwd=D,text=True)
(D/'symbols.txt').write_text(sy_debug)
debug={}
for name in ['tick','window','send_ok','attempts','s_count','s_session','cursor','session','last','g_vm_test_request','g_chassis_auto_active']:
 match=re.search(r'\b'+name+r'\s+(0x[0-9a-fA-F]+)',sy_debug)
 if match:debug[name]=struct.unpack('<I',m.mem_read(int(match[1],16),4))[0]
out={'debug':debug,'kind':a.kind,'passed':ok,'failure_line':v&0x7fffffff if v&0x80000000 else None,'hardware_accessed':False}
if a.kind=='export' and ok:
 sy=subprocess.check_output([str(arm/'fromelf.exe'),'--text','-s','unit.axf'],cwd=D,text=True)
 address=lambda name:int(re.search(r'\b'+name+r'\s+(0x[0-9a-fA-F]+)',sy)[1],16)
 frames=bytes(m.mem_read(address('exported_frames'),3*512));lens=struct.unpack('<3I',m.mem_read(address('exported_lengths'),12))
 sys.path.insert(0,str(next(R.glob('*/scripts'))));from p1_x_load_telemetry import parse_load_telemetry,LoadAccumulator
 acc=LoadAccumulator();packets=[]
 for i,n in enumerate(lens):
  frame=frames[i*512:i*512+n];packets.append(parse_load_telemetry(frame));acc.add(packets[-1])
 assert packets[0]['first_index']==packets[1]['first_index']==0 and packets[2]['first_index']==8 and acc.complete
 out['actual_C_export_decodes_in_host']=True;out['records']=len(acc.records)
if a.kind=='integral_export' and ok:
 sy=sy_debug
 address=lambda name:int(re.search(r'\b'+name+r'\s+(0x[0-9a-fA-F]+)',sy)[1],16)
 frame=bytes(m.mem_read(address('exported_frames'),512)).split(b'\0',1)[0]
 sys.path.insert(0,str(next(R.glob('*/scripts'))));from p1_x_load_telemetry import parse_integral_telemetry
 meta=parse_integral_telemetry(frame);assert meta['enabled'] and meta['ki']==.03 and meta['peak_integral_rpm']==.123 and meta['last_integral_rpm']== -.055
 out['integral_metadata']=meta
(D/'report.json').write_text(json.dumps(out,indent=2));print(json.dumps(out));assert ok
