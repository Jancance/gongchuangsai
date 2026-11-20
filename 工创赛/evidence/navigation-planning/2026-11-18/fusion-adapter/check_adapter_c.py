from pathlib import Path
import ctypes, csv, datetime, hashlib, json, re, struct, subprocess, sys
import xml.etree.ElementTree as ET

R=Path.cwd(); E=Path(__file__).resolve().parent; U=R/'3.stm32f407工程/CHASSIS/USER'
B=Path('D:/Keil_v5/ARM/arm5_compiler/Bin')
attempt=1
while (E/f'actual-c-run{attempt:02d}').exists(): attempt+=1
T=E/f'actual-c-run{attempt:02d}'; T.mkdir()
started=datetime.datetime.now().astimezone().isoformat()
def short(p):
    b=ctypes.create_unicode_buffer(32768)
    assert ctypes.windll.kernel32.GetShortPathNameW(str(p),b,len(b))
    return b.value
def literal(v):
    s=format(v,'.10g'); return s+'f' if '.' in s or 'e' in s else s+'.0f'
rows=list(csv.reader((E/'adapter_vectors_v10.csv').open()))
generated=[]
for row in rows:
    dt,direction,speed,corr,previous,current,vx,vy,omega,x,y=map(float,row)
    generated.append('CK(setup());in.map.pose.heading_deg='+literal(previous)+';CK(observe()==0);tick('+str(int(dt))+');')
    generated.append('in.map.raw_x_mm='+literal(x-corr)+';in.map.raw_y_mm='+literal(y+corr)+';in.map.pose.x_mm='+literal(x)+';in.map.pose.y_mm='+literal(y)+';in.map.correction_x_mm='+literal(corr)+';in.map.correction_y_mm='+literal(-corr)+';in.map.pose.heading_deg='+literal(current)+';CK(observe()==1);')
    for field,value in [('vx_mm_s',vx),('vy_mm_s',vy),('omega_deg_s',omega),('position.x_mm',x),('position.y_mm',y),('heading_deg',current)]:
        generated.append('CLOSE(obs.'+field+','+literal(value)+',.03f);')
    generated.append('CK(obs.feedback_source==FUSION_ESTIMATED_FEEDBACK&&!obs.stopped&&!obs.reasons&&!obs.continuity_lost);')
(T/'adapter_vectors.inc').write_text('\n'.join(generated)+'\n',encoding='ascii')
(T/'unit.c').write_bytes((E/'adapter_unit.c').read_bytes())
sources=[U/'fusion_adapter.c',U/'fusion_reference.c',U/'chassis_kinematics.c']
for source in sources+[U/'fusion_adapter.h']: (T/source.name).write_bytes(source.read_bytes())
hashes={p.name:hashlib.sha256(p.read_bytes()).hexdigest() for p in sources+[U/'fusion_adapter.h',U/'fusion_reference.h',U/'app_odometry.h',U/'chassis_odometry.h']}
control=ET.parse(U.parents[1]/'gongxun-f407.uvprojx').getroot().find('./Targets/Target/TargetOption/TargetArmAds/Cads/VariousControls')
opts=['--cpu','Cortex-M3','--c99','--gnu','--split_sections','-O1','--diag_suppress=177,550','-DVEHICLE_INTEGRATED_ENABLE=0','-DVEHICLE_STAGE1_ENABLE=0','-DAPP_VM_CALIBRATION_ONLY_ENABLE=0','-DAPP_VM_ONLY_ENABLE=0','-DAPP_WHEEL_ENABLE_HOLD_ENABLE=0','-DAPP_BASELINE40_ENABLE=0']
opts+=['-D'+d for d in control.findtext('Define').split(',')]
for inc in control.findtext('IncludePath').split(';'): opts+=['-I',short((U.parents[1]/inc).resolve())]
commands=[]
def run(cmd,log):
    commands.append(cmd);cp=subprocess.run(cmd,cwd=T,capture_output=True,timeout=90)
    (T/log).write_bytes(cp.stdout+cp.stderr)
    if cp.returncode: raise RuntimeError((cp.stdout+cp.stderr).decode('gbk',errors='replace'))
report={'started_at':started,'hardware_accessed':False,'source_sha256':hashes,'vector_rows':len(rows),'attempt':attempt,'execution':'Actual ARMCC Cortex-M3 soft-float product sources in Unicorn; no RTOS/getter/actuator providers'}
try:
    objects=[]
    for p in [T/'unit.c',*sources]:
        obj=p.stem+'.o';objects.append(obj)
        run([str(B/'armcc.exe'),*opts,'-c',short(p),'-o',obj],p.stem+'_compile.log')
    (T/'unit.sct').write_text('LR_ROM 0x08000000 0x100000 { ER_ROM 0x08000000 0x100000 { unit.o (i.test_entry, +First) *(+RO) } RW_RAM 0x20000000 0x50000 { *(+RW,+ZI) } }\n',encoding='ascii')
    run([str(B/'armlink.exe'),'--entry','test_entry','--scatter','unit.sct',*objects,'--library_type=microlib','-o','unit.axf'],'link.log')
    sys.path.insert(0,str(R/'工创赛/tmp/position_alignment_deps'))
    import unicorn as uc
    from unicorn import arm_const as reg
    data=(T/'unit.axf').read_bytes();elf=struct.unpack_from('<16sHHIIIIIHHHHHH',data)
    machine=uc.Uc(uc.UC_ARCH_ARM,uc.UC_MODE_THUMB|uc.UC_MODE_MCLASS)
    for addr,size in [(0x08000000,0x100000),(0x20000000,0x50000),(0xE0000000,0x100000)]:machine.mem_map(addr,size)
    for n in range(elf[12]):
        _,kind,flags,addr,offset,size,*_=struct.unpack_from('<IIIIIIIIII',data,elf[6]+n*elf[11])
        if flags&2 and size:machine.mem_write(addr,b'\0'*size if kind==8 else data[offset:offset+size])
    stop=0x080ff000;machine.reg_write(reg.UC_ARM_REG_SP,0x2004fff0);machine.reg_write(reg.UC_ARM_REG_LR,stop|1)
    machine.emu_start(elf[4]|1,stop,timeout=60000000,count=200000000)
    assert machine.reg_read(reg.UC_ARM_REG_PC)==stop
    result=machine.reg_read(reg.UC_ARM_REG_R0)
    symbols=subprocess.check_output([str(B/'fromelf.exe'),'--text','-s','unit.axf'],cwd=T).decode(errors='replace')
    (T/'symbols.txt').write_text(symbols,encoding='utf-8')
    def symbol_value(name):
        address=int(re.search(r'\b'+name+r'\s+(0x[0-9a-fA-F]+)',symbols)[1],16)
        return struct.unpack('<I',machine.mem_read(address,4))[0]
    report.update(status='PASS' if not result else 'FAIL',failed_line=result,assertions=symbol_value('review_assertions'),sizes={n:symbol_value('size_adapter_'+n) for n in ['state','input','report']})
    if result: report['failure_line_text']=(T/'unit.c').read_text().splitlines()[result-1]
except Exception as exc:
    report.update(status='ERROR',error=str(exc))
report.update(finished_at=datetime.datetime.now().astimezone().isoformat(),commands=commands,source_sha256_after={p.name:hashlib.sha256(p.read_bytes()).hexdigest() for p in sources})
(T/'receipt.json').write_text(json.dumps(report,indent=2),encoding='utf-8')
if report['status']=='PASS':(E/'actual_c_checks.json').write_text(json.dumps(report,indent=2),encoding='utf-8')
print(json.dumps({k:v for k,v in report.items() if k!='commands'},ensure_ascii=False))
raise SystemExit(0 if report['status']=='PASS' else 1)
