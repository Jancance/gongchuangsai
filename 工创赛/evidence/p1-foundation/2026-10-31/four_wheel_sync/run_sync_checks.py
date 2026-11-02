from pathlib import Path
import argparse,json,re,shutil,struct,subprocess,sys
ROOT=Path(__file__).resolve().parents[5];EV=Path(__file__).resolve().parent
sys.path.insert(0,str(ROOT/'工创赛/scripts'))
from p1_turn_fault_audit import function
p=argparse.ArgumentParser();p.add_argument('--kind',choices=['gate','transport','backend'],required=True);p.add_argument('--out',type=Path,required=True);a=p.parse_args()
out=a.out.resolve();out.mkdir(parents=True,exist_ok=False)
candidate=ROOT/'工创赛/tmp/fine_rpm_sync_candidate_20261031'
shutil.copytree(candidate/'USER',out/'src')
shutil.copytree(EV/'models/stubs',out/'stubs')
if a.kind=='gate':source=(EV/'sync_gate_checks.c').read_text(encoding='utf8');objects=[('unit.c','unit.o')]
elif a.kind=='transport':
    can=(candidate/'SYSTEM/CAN/can.c').read_bytes().decode('gbk').replace('\r\n','\n')
    start=can.index('volatile uint32_t g_can_ctrl_fault_latched');end=can.index('void CAN1_ResetSession(void)')
    globals=can[start:end]+'\nstatic uint32_t s_sync_trigger_session,s_sync_trigger_generation;\n'
    funcs='\n'.join(function(can,n) for n in ['CAN1_ResetSession','CAN1_ExchangeDirected','CAN1_ExchangeSingle','CAN1_ControlSingle','CAN1_StageEmmSpeedRaw','CAN1_TriggerSync','CAN1_StopMotor','CAN1_SetEmmSpeedRaw'])
    source=(EV/'transport_checks.c').read_text(encoding='utf8').replace('/* CAN_GLOBALS */',globals).replace('/* CAN_FUNCTIONS */',funcs)
    objects=[('unit.c','unit.o')]
else:
    source=(EV/'models/unit.c').read_bytes().decode('gbk').replace('\r\n','\n')
    source=source.replace('uint32_t test_entry(void){','static uint32_t original_model_entry(void){',1)
    old=function(source,'App_SyncTriggerSourceHealthy')
    source=source.replace(old,'''static uint32_t sync_test_failure;
uint32_t App_SyncTriggerSourceHealthy(void){return power_active && sync_test_failure!=4U;}''')
    old=function(source,'CAN1_StageEmmSpeedRaw');new=old.replace('return 1U;','if(sync_test_failure==10U+a)return 5U;return 1U;')
    source=source.replace(old,new)
    old=function(source,'CAN1_TriggerSync')
    new='''uint32_t CAN1_TriggerSync(uint32_t session,uint32_t generation,CanRxMsg *r) {
 uint32_t i;(void)r;transaction_count++;CHECK(!test_depth);
 if(sync_test_failure==2U)ChassisAuto_Abort();
 if(sync_test_failure==3U)power_active=0;
 if(!ChassisAuto_SyncTriggerClaim(session,generation))return 9U;
 CHECK(sync_cached_mask==15U);
 for(i=0;i<4;i++)demand[i]=sync_cached[i];
 sync_cached_mask=0U;sync_trigger_count++;
 if(sync_test_failure==5U)test_tick+=50;
 if(sync_test_failure==6U)ChassisAuto_Abort();
 return sync_test_failure==1U?5U:(sync_test_failure==7U?8U:1U);
}'''
    source=source.replace(old,new)+'\n'+(EV/'backend_guard_checks.c').read_text(encoding='utf8')
    objects=[('unit.c','unit.o'),('src/chassis_kinematics.c','kinematics.o'),('src/motion_profile.c','profile.o'),('src/heading_controller.c','heading.o'),('src/heading_session.c','session.o')]
(out/'unit.c').write_bytes(source.encode('gbk'))
(out/'unit.sct').write_text('LR_ROM 0x08000000 0x00100000 { ER_ROM 0x08000000 0x00100000 { unit.o (i.test_entry, +First) *(+RO) } RW_RAM 0x20000000 0x00020000 { *(+RW,+ZI) } }\n')
arm=Path('D:/Keil_v5/ARM/arm5_compiler/bin');logs=[]
commands=[[str(arm/'armcc.exe'),'--cpu','Cortex-M3','--c99','--split_sections','-O1','-I','src','-I','stubs','-c',s,'-o',o] for s,o in objects]
commands.append([str(arm/'armlink.exe'),'--entry','test_entry','--scatter','unit.sct']+[o for s,o in objects]+['--library_type=microlib','-o','unit.axf'])
for cmd in commands:
    r=subprocess.run(cmd,cwd=out,capture_output=True,text=True,errors='replace');logs.append(dict(command=cmd,returncode=r.returncode,output=r.stdout+r.stderr));(out/'build.json').write_text(json.dumps(logs,indent=2))
    if r.returncode:raise RuntimeError(r.stdout+r.stderr)
sys.path.insert(0,str(ROOT/'工创赛/tmp/position_alignment_deps'))
import unicorn as uc
from unicorn import arm_const as reg
m=uc.Uc(uc.UC_ARCH_ARM,uc.UC_MODE_THUMB|uc.UC_MODE_MCLASS);m.mem_map(0x08000000,0x100000);m.mem_map(0x20000000,0x20000)
b=(out/'unit.axf').read_bytes();h=struct.unpack_from('<16sHHIIIIIHHHHHH',b)
for i in range(h[10]):
    kind,off,addr,_,size,_,_,_=struct.unpack_from('<IIIIIIII',b,h[5]+i*h[9])
    if kind==1 and size:m.mem_write(addr,b[off:off+size])
stop=0x080ff000;m.reg_write(reg.UC_ARM_REG_SP,0x2001fff0);m.reg_write(reg.UC_ARM_REG_LR,stop|1);m.emu_start(h[4]|1,stop,count=100000000)
value=m.reg_read(reg.UC_ARM_REG_R0);report=dict(kind=a.kind,passed=value<0x80000000 and m.reg_read(reg.UC_ARM_REG_PC)==stop,assertions=value if value<0x80000000 else None,failure_line=value&0x7fffffff if value&0x80000000 else None,hardware_accessed=False)
(out/'report.json').write_text(json.dumps(report,indent=2));print(json.dumps(report));assert report['passed'],report
