from pathlib import Path
import json, re, struct, subprocess, sys

ROOT=Path.cwd()
BASE=Path(__file__).parent
sys.path.insert(0,str(ROOT/'工创赛/tmp/position_alignment_deps'))
sys.path.insert(0,str(ROOT/'工创赛/scripts'))
import unicorn as uc
from unicorn import arm_const as reg
from p1_turn_fault_audit import function

def run(directory):
    arm=Path('D:/Keil_v5/ARM/arm5_compiler/bin')
    (directory/'unit.sct').write_text('LR_ROM 0x08000000 0x00100000 { ER_ROM 0x08000000 0x00100000 { unit.o (i.test_entry,+First) *(+RO) } RW_RAM 0x20000000 0x00020000 { *(+RW,+ZI) } }')
    logs=[]
    for cmd in [[str(arm/'armcc.exe'),'--cpu','Cortex-M3','--c99','--split_sections','-O1','-c','unit.c','-o','unit.o'],[str(arm/'armlink.exe'),'--entry','test_entry','--scatter','unit.sct','unit.o','--library_type=microlib','-o','unit.axf']]:
        p=subprocess.run(cmd,cwd=directory,capture_output=True,text=True)
        logs.append(dict(command=cmd,returncode=p.returncode,output=p.stdout+p.stderr))
        (directory/'build.json').write_text(json.dumps(logs,indent=2))
        assert p.returncode==0,p.stdout+p.stderr
    sy=subprocess.check_output([str(arm/'fromelf.exe'),'--text','-s','unit.axf'],cwd=directory,text=True)
    m=uc.Uc(uc.UC_ARCH_ARM,uc.UC_MODE_THUMB|uc.UC_MODE_MCLASS)
    m.mem_map(0x08000000,0x100000);m.mem_map(0x20000000,0x20000)
    b=(directory/'unit.axf').read_bytes();h=struct.unpack_from('<16sHHIIIIIHHHHHH',b)
    for i in range(h[10]):
        kind,off,addr,_,size,_,_,_=struct.unpack_from('<IIIIIIII',b,h[5]+i*h[9])
        if kind==1 and size:m.mem_write(addr,b[off:off+size])
    stop=0x080ff000
    m.reg_write(reg.UC_ARM_REG_SP,0x2001fff0);m.reg_write(reg.UC_ARM_REG_LR,stop|1)
    m.emu_start(h[4]|1,stop,count=1000000)
    value=m.reg_read(reg.UC_ARM_REG_R0)
    report=dict(passed=m.reg_read(reg.UC_ARM_REG_PC)==stop and value<0x80000000,assertions=value if value<0x80000000 else None,failure_line=value&0x7fffffff if value&0x80000000 else None,hardware_accessed=False)
    if directory.name=='hf3_unit' and report['passed']:
        address=int(re.search(r'\bexported_frame\s+(0x[0-9a-fA-F]+)',sy)[1],16)
        frame=bytes(m.mem_read(address,512)).split(b'\0',1)[0]
        from p1_x_main_transport import parse_fault_telemetry
        result=parse_fault_telemetry(frame)
        assert result['backend_target_decirpm']==-10 and result['backend_measured_decirpm']==3
        assert result['backend_address']==1 and result['backend_fault_reason']==11 and result['backend_reverse_samples']==2
        report['actual_C_HF3_roundtrip']=result
    (directory/'report.json').write_text(json.dumps(report,indent=2))
    print(directory.name,json.dumps(report))
    assert report['passed']

main=(ROOT/'3.stm32f407工程/USER/main.c').read_bytes().decode('gbk')
formatter=function(main,'App_MotionFaultTelemetry')
d=BASE/'hf3_unit';d.mkdir(exist_ok=True)
source='''#include <stdint.h>
#include <stdio.h>
#include <string.h>
#define MOTION_FAULT_DIAG_FIELDS 43U
#define taskENTER_CRITICAL() ((void)0)
#define taskEXIT_CRITICAL() ((void)0)
static uint32_t g_motion_fault_diag_valid,g_motion_fault_diag[43];
volatile char exported_frame[512];
static uint32_t xTaskGetTickCount(void){return 1000U;}
static uint32_t App_SerialDiagnosticWindow(uint32_t n,uint32_t w){(void)n;(void)w;return 1U;}
static uint32_t App_DebugSendString(const char *s){strcpy((char*)exported_frame,s);return 1U;}
'''+main[main.index('static uint16_t App_SerialCrc('):main.index('static int32_t App_SerialMilli(')]+'\n'+formatter+'''
uint32_t test_entry(void){
 g_motion_fault_diag_valid=1;g_motion_fault_diag[6]=1;g_motion_fault_diag[29]=11;g_motion_fault_diag[30]=6;
 g_motion_fault_diag[32]=1;g_motion_fault_diag[34]=g_motion_fault_diag[35]=15;
 g_motion_fault_diag[38]=(uint32_t)-10;g_motion_fault_diag[39]=3;
 g_motion_fault_diag[40]=900;g_motion_fault_diag[41]=950;g_motion_fault_diag[42]=2;
 App_MotionFaultTelemetry();return exported_frame[0]=='H'?1U:0x80000001U;
}
'''
(d/'unit.c').write_text(source,encoding='ascii')
run(BASE/'guard_unit')
run(d)
