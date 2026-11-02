"""Compile actual runtime-route C and extracted frontend with offline platform stubs."""
import argparse
import hashlib
import json
from pathlib import Path
import re
import shutil
import struct
import subprocess
import sys

ROOT=Path(__file__).resolve().parents[2]

def function(text,name):
    m=re.search(r'^(?:static )?(?:uint32_t|uint16_t|int32_t|void) '+re.escape(name)+r'\([^;]*?\)\s*\{',text,re.M)
    if not m:raise ValueError(name)
    i=text.index('{',m.start());j=i+1;depth=1
    while depth:depth+=(text[j]=='{')-(text[j]=='}');j+=1
    return text[m.start():j]

def run(out):
    out.mkdir(parents=True,exist_ok=False)
    user=ROOT/'3.stm32f407工程/USER';shutil.copytree(user,out/'src')
    shutil.copytree(Path(__file__).parent/'p1_offline_tests/stubs',out/'stubs')
    base=(ROOT/'工创赛/evidence/p1-foundation/2026-11-01/x_perimeter_comp008/scheduler_final/unit.c').read_bytes().decode('gbk')
    base=base[:base.index('static uint32_t App_MotionPlanValid(')]
    base=base.replace('#include <stdint.h>', '#include <stdint.h>\nstatic void CaptureFailure(uint32_t line);')
    base=re.sub(r'#define CHECK\(x\)[^\n]+', '#define CHECK(x) do{checks++;if(!(x)&&!first_failure)CaptureFailure(__LINE__);}while(0)', base)
    base=base.replace('static struct {float yaw_deg;}','static struct {float yaw_deg,rate_dps;}')
    extra='''
#include <stdio.h>
#include "src/runtime_route.h"
static RuntimeRoute s_runtime_route;
static RuntimeRoutePrepared s_runtime_prepared;
static uint32_t s_route_ack_pending,s_route_ack_id,s_route_ack_op,s_route_ack_result,s_route_ack_generation;
static uint32_t s_perimeter_odom_started,s_perimeter_completed_seen;
static float s_perimeter_targets[9][3];
static uint32_t g_perimeter_records[9][8];
static uint32_t g_serial_motion_state,g_serial_motion_result,g_serial_motion_rejected,g_serial_motion_accepted,g_serial_motion_link_lost;
static uint32_t s_serial_token_seq,s_serial_token_tick,s_serial_arm_ms,s_serial_keepalive_ms;
static uint32_t s_serial_tx_seq,s_serial_tx_valid,s_serial_proto_ok;
static uint32_t FrontendRearm(uint32_t session){
 uint32_t ok=ChassisAuto_RearmCompleted(session);
 if(ok){mock_status.state=CHASSIS_AUTO_IDLE;mock_status.result=0;mock_status.cut_vm=0;}
 return ok;}
volatile uint32_t route_context[18];
static void CaptureFailure(uint32_t line){first_failure=line;
 route_context[0]=s_route_ack_result;route_context[1]=g_serial_motion_state;route_context[2]=g_serial_motion_result;
 route_context[3]=s_runtime_route.stage_mask;route_context[4]=s_runtime_route.committed.id;route_context[5]=s_runtime_route.ready;
 route_context[6]=g_combined_test.state;route_context[7]=g_combined_test.completed;route_context[8]=s_runtime_route.committed.count;
 route_context[9]=test_tick;route_context[10]=g_chassis_auto_state;route_context[11]=g_motion_plan_valid;route_context[12]=g_combined_test.cause;route_context[13]=g_forward_hold.phase;route_context[14]=g_heading_exec_phase;route_context[15]=mock_status.result;route_context[16]=s_motion_session_open;route_context[17]=s_serial_start_permit;}

'''
    main=(user/'main.c').read_bytes().decode('gbk').replace('\r\n','\n')
    names=['App_MotionPlanValid','App_DebugSelectPlan','App_MoveSubmit','App_PerimeterTurnStart','App_CombinedLoadAction',
           'App_CombinedTestStop','App_CombinedTestSafety','App_CombinedTestStep','App_SerialCrc','App_SerialUInt',
           'App_SerialIdleReady','App_SerialCompletedStopped','App_SerialReject','App_RuntimeRouteEditable',
           'App_RuntimeRouteInstall','App_RuntimeRouteCommand','App_SerialMotionCommand','App_SerialMotionByte','App_SerialMotionSafety']
    functions='\n'.join(function(main,n) for n in names).replace('!ChassisAuto_RearmCompleted(', '!FrontendRearm(')
    # Preserve the reviewed scheduler reset setup, including fixed-heading source.
    old=(ROOT/'工创赛/evidence/p1-foundation/2026-11-01/x_perimeter_comp008/scheduler_final/unit.c').read_bytes().decode('gbk')
    reset=function(old,'reset_scheduler')
    tests=(Path(__file__).parent/'p1_offline_tests/runtime_route_checks.c').read_text(encoding='utf8')
    # Cross-language fixture: host-produced bytes pass the actual RX byte parser.
    import p1_runtime_route as host
    rows=[(4,18144,200,35000),(6,18000,200,35000),(3,-90000,30000,35000),
          (3,-90000,30000,35000),(4,18144,200,35000),(3,90000,30000,35000),
          (3,90000,30000,35000),(5,18000,200,35000)]
    digest=host.digest(7,rows)
    frames=[host.command('BEGIN',(99,1000),7,8)]
    frames += [host.command('ROW',(99,1000),7,i,*r) for i,r in enumerate(rows)]
    frames += [host.command('COMMIT',(99,1000),7,digest)]
    interop='setup_route();\n'
    for frame in frames:
        interop+='feed_bytes('+json.dumps(frame.decode('ascii'))+');CHECK(s_route_ack_result==RQ_OK);\n'
    interop+='CHECK(s_runtime_route.ready);CHECK(s_runtime_route.committed.digest=='+str(digest)+'U);CHECK(MOTION_ACTION_COUNT==8);\n'
    tests=tests.replace(' for(n=1;n<=8;n++) {',interop+' for(n=1;n<=8;n++) {')
    (out/'unit.c').write_bytes((base+extra+functions+'\n'+reset+'\n'+tests).encode('gbk'))
    (out/'unit.sct').write_text('LR_ROM 0x08000000 0x00100000 { ER_ROM 0x08000000 0x00100000 { unit.o (i.test_entry,+First) *(+RO) } RW_RAM 0x20000000 0x00020000 { *(+RW,+ZI) } }')
    arm=Path('D:/Keil_v5/ARM/arm5_compiler/bin');logs=[]
    files=[('unit.c','unit.o'),('src/runtime_route.c','route.o'),('src/chassis_kinematics.c','kinematics.o'),('src/motion_profile.c','profile.o'),('src/heading_controller.c','heading.o')]
    commands=[[str(arm/'armcc.exe'),'--cpu','Cortex-M3','--c99','--split_sections','-O1','-I','src','-I','stubs','-DAPP_X_OFFLINE_ONLY=0','-c',s,'-o',o] for s,o in files]
    commands.append([str(arm/'armlink.exe'),'--entry','test_entry','--scatter','unit.sct',*[o for _,o in files],'--library_type=microlib','-o','unit.axf'])
    for cmd in commands:
        p=subprocess.run(cmd,cwd=out,capture_output=True,text=True,errors='replace')
        logs.append(dict(command=cmd,returncode=p.returncode,output=p.stdout+p.stderr));(out/'build.json').write_text(json.dumps(logs,indent=2))
        if p.returncode:raise RuntimeError(p.stdout+p.stderr)
    sys.path.insert(0,str(ROOT/'工创赛/tmp/position_alignment_deps'))
    import unicorn as uc
    from unicorn import arm_const as reg
    m=uc.Uc(uc.UC_ARCH_ARM,uc.UC_MODE_THUMB|uc.UC_MODE_MCLASS);m.mem_map(0x08000000,0x100000);m.mem_map(0x20000000,0x20000)
    b=(out/'unit.axf').read_bytes();h=struct.unpack_from('<16sHHIIIIIHHHHHH',b)
    for i in range(h[10]):
        kind,off,addr,_,size,_,_,_=struct.unpack_from('<IIIIIIII',b,h[5]+i*h[9])
        if kind==1 and size:m.mem_write(addr,b[off:off+size])
    stop=0x080ff000;m.reg_write(reg.UC_ARM_REG_SP,0x2001fff0);m.reg_write(reg.UC_ARM_REG_LR,stop|1)
    m.emu_start(h[4]|1,stop,count=50000000)
    v=m.reg_read(reg.UC_ARM_REG_R0)
    sy=subprocess.check_output([str(arm/'fromelf.exe'),'--text','-s','unit.axf'],cwd=out,text=True)
    addr=int(re.search(r'\broute_context\s+(0x[0-9a-fA-F]+)',sy)[1],16)
    result=dict(passed=m.reg_read(reg.UC_ARM_REG_PC)==stop and v<0x80000000,
        assertions=v if v<0x80000000 else None,failure_line=v&0x7fffffff if v&0x80000000 else None,
        context=list(struct.unpack('<18I',m.mem_read(addr,72))),hardware_accessed=False,
        source_sha256={p.name:hashlib.sha256(p.read_bytes()).hexdigest() for p in (out/'src').glob('*') if p.is_file()},
        limits='Actual route parser, validation, command admission, scheduler and backend rearm; segment completion, heading source, CAN/VM/RTOS are stubs.')
    (out/'report.json').write_text(json.dumps(result,indent=2));print(json.dumps({k:result[k] for k in ['passed','assertions','failure_line','context']}))
    if not result['passed']:raise RuntimeError('Runtime route C checks failed')

if __name__=='__main__':
    p=argparse.ArgumentParser();p.add_argument('--out',type=Path,required=True);a=p.parse_args();run(a.out)
