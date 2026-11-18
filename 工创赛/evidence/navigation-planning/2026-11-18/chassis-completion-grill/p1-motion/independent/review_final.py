from pathlib import Path
import ctypes as ct
import hashlib
import json
import math
import re
import struct
import subprocess
import sys
import tempfile
import xml.etree.ElementTree as ET

ROOT=Path.cwd()
HERE=Path(__file__).resolve().parent
T=Path(tempfile.mkdtemp(prefix='p1-motion-arm-review-'))
P=ROOT/'3.stm32f407工程'
U=P/'CHASSIS/USER'
E=ROOT/'工创赛/evidence/navigation-planning/2026-11-18/chassis-completion-grill/p1-motion'
B=Path('D:/Keil_v5/ARM/arm5_compiler/bin')
SOURCES=[U/f for f in ('fusion_motion.c','fusion_motion.h','chassis_kinematics.c','chassis_kinematics.h')]
f=ct.c_float
u=ct.c_uint32

def short(path):
    result=ct.create_unicode_buffer(32768)
    if not ct.windll.kernel32.GetShortPathNameW(str(path),result,len(result)):
        raise OSError(path)
    return result.value

def run(args,name):
    completed=subprocess.run(args,cwd=T,capture_output=True,timeout=60)
    (T/name).write_bytes(completed.stdout+completed.stderr)
    if completed.returncode:
        raise RuntimeError((completed.stdout+completed.stderr).decode('gbk',errors='replace'))

def digest(p):
    return hashlib.sha256(p.read_bytes()).hexdigest()

hashes={p.name:digest(p) for p in SOURCES}
cpu=sys.argv[1] if len(sys.argv)>1 else 'Cortex-M4.fp.sp'
controls=ET.parse(P/'gongxun-f407.uvprojx').getroot().find('./Targets/Target/TargetOption/TargetArmAds/Cads/VariousControls')
options=['--cpu',cpu,'--c99','--gnu','--split_sections','-O1']
options.extend('-D'+d for d in controls.findtext('Define').split(','))
options.extend('-I'+short((P/i).resolve()) for i in controls.findtext('IncludePath').split(';'))
for source in [HERE/'unit.c', SOURCES[0], SOURCES[2]]:
    run([str(B/'armcc.exe'),*options,'-c',short(source),'-o',source.stem+'.o'],source.stem+'.log')
(T/'unit.sct').write_text('LR_ROM 0x08000000 0x100000 { ER_ROM 0x08000000 0x100000 { unit.o (i.test_entry, +First) *(+RO) } RW_RAM 0x20000000 0x50000 { *(+RW,+ZI) } }\n')
run([str(B/'armlink.exe'),'--entry','test_entry','--scatter','unit.sct','unit.o','fusion_motion.o','chassis_kinematics.o','--library_type=microlib','-o','unit.axf'],'link.log')
symbols=subprocess.check_output([str(B/'fromelf.exe'),'--text','-s','unit.axf'],cwd=T).decode(errors='replace')
axf=(T/'unit.axf').read_bytes()
elf=struct.unpack_from('<16sHHIIIIIHHHHHH',axf)
sys.path.insert(0,str(ROOT/'工创赛/tmp/position_alignment_deps'))
import unicorn as uc
from unicorn import arm_const as reg
machine=uc.Uc(uc.UC_ARCH_ARM,uc.UC_MODE_THUMB|uc.UC_MODE_MCLASS)
if cpu.startswith('Cortex-M4'):
    machine.ctl_set_cpu_model(reg.UC_CPU_ARM_CORTEX_M4)
for address,size in [(0x08000000,0x100000),(0x20000000,0x50000),(0xE0000000,0x100000)]:
    machine.mem_map(address,size)
for n in range(elf[12]):
    _,kind,flags,address,offset,size,*_=struct.unpack_from('<IIIIIIIIII',axf,elf[6]+n*elf[11])
    if flags&2 and size:
        machine.mem_write(address,b'\0'*size if kind==8 else axf[offset:offset+size])
stop=0x080ff000
if cpu.startswith('Cortex-M4'):
    machine.mem_write(0xE000ED88,struct.pack('<I',0x00f00000))
    machine.reg_write(reg.UC_ARM_REG_FPEXC,1<<30)

class V(ct.Structure):
    _fields_=[(n,f) for n in ('x','y','yaw')]
    def values(self):
        return [self.x,self.y,self.yaw]

class L(ct.Structure):
    _fields_=[(n,f) for n in ('component','xy_a','xy_j','yaw_v','yaw_a','yaw_j','wheel','wheel_a','wheel_j')]

class R(ct.Structure):
    _fields_=[('now',u),('source',u*5)] + [(n,u) for n in ('max_age','compute','queue','batch','dt')] + [(n,f) for n in ('tx','ty','remaining','back','lat','angle','xy_err','yaw_err','mix_mm','mix_rad')]+[('sign',ct.c_int32),('lost',u)]

class Curve(ct.Structure):
    _fields_=[('p',V*4),('initial_a',V),('mm',f),('rad',f),('duration',u)]

class Sample(ct.Structure):
    _fields_=[('q',V),('a',V),('j',V)]
    def values(self):
        return self.q.values()+self.a.values()+self.j.values()

class Reserve(ct.Structure):
    _fields_=[('curve',Curve),('mm',f),('back',f),('lat',f),('rad',f),('held',u),('reason',u)]
    
class Proposal(ct.Structure):
    _fields_=[('before',Reserve),('after',Reserve),('candidate',Curve),('prefix',Sample),('prefix_mm',f),('reason',u)]

class Cmd(ct.Structure):
    _fields_=[('address',ct.c_uint8),('direction',ct.c_uint8),('raw',ct.c_uint16)]

class Input(ct.Structure):
    _fields_=[('q',V),('a',V),('target',V),('limits',L),('req',R),('mode',u),('dt',u),('sample_at',u)]

class Output(ct.Structure):
    _fields_=[(n,u) for n in ('decision','sampled','inverse','encoded')]+[('curve',Curve),('sample',Sample),('wheels',f*4),('scale',f),('commands',Cmd*4),('proposal',Proposal),('reservation',Reserve),('extrema',f*4),('overspeed_ms',u)]

def addr(name):
    return int(re.search(r'\b'+name+r'\s+(0x[0-9a-fA-F]+)',symbols)[1],16)

INPUT,OUTPUT=map(addr,['review_input','review_output'])
L0=L(400,150,300,math.pi/2,math.pi,math.pi*8,300,250,1400)

def exec_case(q,a,target,dt=20,sample=0,mode=0,req=None,limits=L0):
    raw=Input(V(*q),V(*a),V(*target),limits,req or R(),mode,dt,sample)
    machine.mem_write(INPUT,bytes(raw))
    machine.reg_write(reg.UC_ARM_REG_SP,0x2004fff0)
    machine.reg_write(reg.UC_ARM_REG_LR,stop|1)
    machine.emu_start(elf[4]|1,stop,timeout=10000000,count=30000000)
    if machine.reg_read(reg.UC_ARM_REG_PC)!=stop:
        raise RuntimeError(hex(machine.reg_read(reg.UC_ARM_REG_PC)))
    return Output.from_buffer_copy(machine.mem_read(OUTPUT,ct.sizeof(Output)))

vectors_path=E/'attempt7/p1_kernel_vectors.json'
vectors=json.loads(vectors_path.read_bytes())
checks=0
mismatches=[]
def chk(check,msg):
    global checks
    checks+=1
    if not check:
        mismatches.append(msg)
for v in vectors:
    name=v['name']
    o=exec_case(v['q0'],v['a0'],v['q1'])
    chk(o.decision==v['accepted'],[name,'accepted',o.decision,v['accepted']])
    if not o.decision or not v['accepted']:
        continue
    chk(o.curve.duration==v['duration_ms'],[name,'duration',o.curve.duration,v['duration_ms']])
    chk(abs(o.curve.mm-v['distance_mm'])<0.002,[name,'mm',o.curve.mm,v['distance_mm']])
    chk(abs(o.curve.rad-v['angular_rad'])<0.00001,[name,'rad',o.curve.rad,v['angular_rad']])
    for i,t in enumerate(range(0,o.curve.duration+1,o.curve.duration//4)):
        o=exec_case(v['q0'],v['a0'],v['q1'],sample=t)
        chk(o.sampled and o.inverse and o.encoded and o.scale==1,[name,'sample',t,o.sampled,o.inverse,o.encoded,o.scale])
        chk([x.raw for x in o.commands]==v['raw_at_samples'][i],[name,'raw',t,[x.raw for x in o.commands],v['raw_at_samples'][i]])
        chk([x.direction for x in o.commands]==v['direction_at_samples'][i],[name,'dir',t,[x.direction for x in o.commands],v['direction_at_samples'][i]])
        for actual,expected in zip(o.sample.values(),v['qva_samples'][i]):
            chk(abs(actual-expected)<0.002,[name,'qva',t,actual,expected])
 
r=R(0,(u*5)(0,0,0,0,0),0,0,0,0,200,1,0,3,100,100,1,10,0,0,0,0,0)
def show_prefix(o):
    p=o.proposal
    return dict(decision=o.decision,reason=p.reason,duration=p.candidate.duration,prefix_q=p.prefix.q.values(),
                prefix_a=p.prefix.a.values(),prefix_mm=p.prefix_mm,before_mm=p.before.mm,after_mm=p.after.mm,
                prefix_feedback_margin_mm=r.xy_err*r.dt/1000,brake_ms=p.after.curve.duration,
                available_mm=r.remaining)
prefix_cases={}
for target in [0,0.1,1,2,3,5]:
    o=exec_case([0,0,0],[0,0,0],[target,0,0],mode=1,req=r)
    prefix_cases[str(target)]=show_prefix(o)

boundary=[]
for i in [0,0.001,0.01,0.05,0.1,0.5,1]:
    for j in [0,400]:
        o=exec_case([400,0,0],[i,0,0],[j,0,0],mode=3)
        boundary.append(dict(q=400,a=i,target=j,accepted=o.decision,duration=o.curve.duration,
                             initial_a=o.sample.a.x,peak_q=o.extrema[0],peak_a=o.extrema[1],
                             peak_j=o.extrema[2],first_overspeed_ms=o.overspeed_ms))


BRAKE,ADVANCE=1,2
Z=[0,0,0]
for t,bounds in [([1,0,0],{'remaining':3}),([0,1,0],{'lat':3}),([-.5,0,0],{'back':3}),
                 ([0,0,.005],{'angle':.0035})]:
    r=R(0,(u*5)(0,0,0,0,0),0,0,0,0,200,1,0,100,100,100,1,10,0,0,0,0,0)
    if t[2]:
        r.xy_err=0
        r.yaw_err=.01
    for item,val in bounds.items(): setattr(r,item,val)
    o=exec_case(Z,Z,t,mode=1,req=r)
    chk(o.decision==BRAKE,['prefix error',t,bounds,o.decision,o.proposal.reason])
for q,a in [([400,0,0],[.1,0,0]),([400,0,0],[.001,0,0]),([0,-400,0],[0,-.001,0]),
            ([0,0,L0.yaw_v],[0,0,1e-6]),([0,0,-L0.yaw_v],[0,0,-1e-6])]:
    for target in [q,Z]:
        o=exec_case(q,a,target)
        chk(o.decision==0,['exact cap outward',q,a,target,o.decision])
    o=exec_case(q,[-x for x in a],Z)
    chk(o.decision==1,['exact cap inward',q,a,o.decision])
o=exec_case([398,0,0],[.001,0,0],[398,0,0],mode=3)
chk(o.decision==1 and o.curve.duration==20 and o.sample.a.x==f(.001).value,
    ['short curve a(0)',o.curve.duration,o.sample.a.x,o.decision])
o=exec_case([398,0,0],[.001,0,0],[398,0,0],sample=20)
chk(o.sampled and o.sample.a.x==0 and o.sample.q.x==398,
    ['short curve a(T) and q(T)',o.sampled,o.sample.a.x,o.sample.q.x])
o=exec_case([398,0,0],[-3.5980224609375,0,0],[397.9721374511719,0,0],mode=3)
jerk_replay=dict(duration_ms=o.curve.duration, initial_j=o.sample.j.x, max_abs_j=o.extrema[2])
chk(o.decision==1 and o.curve.duration>20 and o.extrema[2]<=300.0006,
    ['jerk roundoff',o.decision,o.curve.duration,o.extrema[2]])
o=exec_case([400,0,0],Z,[400,400,0],mode=3)
chk(o.decision and o.extrema[0]==400 and not o.overspeed_ms,
    ['constant cap',o.decision,o.curve.duration,o.extrema[0],o.overspeed_ms])
r=R(0,(u*5)(0,0,0,0,0),0,0,0,0,17,1,0,5000,5000,5000,3,0,0,0,0,0,0)
o=exec_case([400,0,0],Z,[400,400,0],mode=1,req=r)
chk(o.decision==ADVANCE and o.proposal.prefix.q.x==400,
    ['constant cap proposal',o.decision,o.proposal.reason,o.proposal.prefix.q.x])
q=[373.505615234375,350.18084716796875,1.499687671661377]
l=L.from_buffer_copy(bytes(L0)); l.wheel=257.240966796875
o=exec_case(q,Z,q,limits=l)
chk(o.decision==0,['real inverse endpoint capacity',o.decision,o.scale])
q=[240,0,0]; l.wheel=59.52808380126953
o=exec_case(q,Z,q)
l.wheel=max(map(abs,o.wheels))
for ax,expected in [(.001,0),(-.001,1)]:
    o=exec_case(q,[ax,0,0],Z,limits=l)
    chk(o.decision==expected,['custom wheel endpoint outward/inward',ax,o.decision,expected,l.wheel])
r=R(35,(u*5)(2**32-15,3,14,12,18),100,5,10,28,20,1,0,1000,60,25,math.pi,2,.001,.2,.001,0,0)
o=exec_case([350,0,0],[90,0,0],Z,mode=2,req=r)
chk(o.decision==0 and o.reservation.reason==0 and o.reservation.held==93
    and o.reservation.curve.duration==4520 and abs(o.reservation.mm-986.204)<.001,
    ['wrapped reserve',o.reservation.reason,o.reservation.held,o.reservation.mm])
r.lost=1
o=exec_case([350,0,0],[90,0,0],Z,mode=2,req=r)
chk(o.reservation.reason==3,['partial delivery',o.reservation.reason])
r.lost=0; r.source[0]=36
o=exec_case([350,0,0],[90,0,0],Z,mode=2,req=r)
chk(o.reservation.reason==2,['future observation',o.reservation.reason])

report=dict(cpu=cpu,vector_sha256=digest(vectors_path),sha256=hashes,unchanged=hashes=={p.name:digest(p) for p in SOURCES},vectors=len(vectors),
            checks=checks,mismatches=mismatches,jerk_replay=jerk_replay,axf_sha256=digest(T/'unit.axf'),
            fixture_sha256=digest(HERE/'unit.c'), script_sha256=digest(Path(__file__)),prefix_budget=prefix_cases,numeric_boundaries=boundary)
(T/'review.json').write_text(json.dumps(report,indent=2))
print(T/'review.json')
print(json.dumps({k: report[k] for k in ('cpu','sha256','checks','mismatches','unchanged')}))
if mismatches or not report['unchanged']:
    raise SystemExit(1)

