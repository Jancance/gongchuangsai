"""Offline source and saved-fault audit. No serial, probe, reset or vehicle API."""
from pathlib import Path
import hashlib
import json
import re
import shutil
import struct
import subprocess
import sys

ROOT=Path(__file__).resolve().parents[2]
# Historical reproducer uses the exact failed candidate, not subsequently repaired firmware.
USER=ROOT/'工创赛/tmp/turn30_vehicle_20261029/USER'
OUT=ROOT/'工创赛/evidence/p1-foundation/2026-10-29/turn30_fault_audit'


def function(text,name):
    match=re.search(r'^(?:static )?(?:uint32_t|void) '+re.escape(name)+r'\([^;]*?\)\s*\{',text,re.M)
    if not match:raise ValueError(name)
    start=match.start();pos=text.index('{',match.start());depth=1;end=pos+1
    while depth:
        depth+=(text[end]=='{')-(text[end]=='}');end+=1
    return text[start:end]


def main():
    OUT.mkdir(exist_ok=False)
    source=(USER/'main.c').read_bytes().decode('gbk')
    publish=function(source,'App_HeadingSourcePublish')
    prelude='''#include <stdint.h>
#include "hwt101_parser.h"
#define HEADING_SOURCE_FRESH_MS 100U
#define taskENTER_CRITICAL() ((void)0)
#define taskEXIT_CRITICAL() ((void)0)
typedef struct {uint32_t sequence,sample_ms,angle_ms,rate_ms,healthy;float yaw_deg,rate_dps;} AppHeadingSource;
static AppHeadingSource s_heading_source;
static uint32_t s_heading_source_ready,clock_ms;
static uint32_t g_hwt_update_tick,g_hwt_angle_age_ms,g_hwt_rate_age_ms,g_hwt_version_result,g_hwt_valid_mask;
static uint32_t g_hwt_cal_active,g_hwt_cal_request,g_hwt_cal_end_tx_result,g_hwt_version_state,g_hwt_version_request,g_hwt_a7_request;
static float g_hwt_yaw_deg,g_hwt_wz_dps;
static uint32_t xTaskGetTickCount(void){return clock_ms;}
volatile uint32_t results[12];
#define CHECK(x) do{if(!(x))return 0x80000000U|__LINE__;}while(0)
'''
    body='''
uint32_t test_entry(void){
 Hwt101Parser parser;uint32_t i;uint8_t angle[11]={0x55,0x53},rate[11]={0x55,0x52};
 g_hwt_version_result=1;g_hwt_yaw_deg=-93.043f;g_hwt_valid_mask=3;
 clock_ms=g_hwt_update_tick=22930;g_hwt_angle_age_ms=g_hwt_rate_age_ms=7;
 App_HeadingSourcePublish();CHECK(s_heading_source.healthy==1);
 results[0]=s_heading_source.angle_ms;CHECK(results[0]==22923);
 Hwt101Parser_Init(&parser);
 for(i=0;i<10;i++){angle[10]+=angle[i];rate[10]+=rate[i];}
 for(i=0;i<11;i++)Hwt101Parser_Feed(&parser,angle[i],22923);
 for(i=0;i<11;i++)Hwt101Parser_Feed(&parser,rate[i],22923);
 CHECK(Hwt101Parser_Valid(&parser,22930)==3);
 Hwt101Parser_Invalidate(&parser);CHECK(Hwt101Parser_Valid(&parser,22930)==0);
 g_hwt_valid_mask=0;g_hwt_angle_age_ms=g_hwt_rate_age_ms=0xffffffffU;
 App_HeadingSourcePublish();CHECK(s_heading_source.healthy==0);
 results[1]=s_heading_source.sample_ms;results[2]=s_heading_source.angle_ms;
 results[3]=22938U-s_heading_source.angle_ms;results[4]=s_heading_source.healthy;
 CHECK(results[1]==22930 && results[2]==22931 && results[3]==7 && results[4]==0);
 g_hwt_valid_mask=2;g_hwt_rate_age_ms=7;App_HeadingSourcePublish();
 CHECK(!s_heading_source.healthy);results[5]=22938U-s_heading_source.angle_ms;
 results[6]=22938U-s_heading_source.rate_ms;CHECK(results[5]==7 && results[6]==15);
 /* Genuine UINT32 clock wrap must remain supported, separately from invalid age. */
 clock_ms=g_hwt_update_tick=3;g_hwt_valid_mask=3;
 g_hwt_angle_age_ms=g_hwt_rate_age_ms=7;App_HeadingSourcePublish();
 CHECK(s_heading_source.healthy==1);CHECK((uint32_t)(3U-s_heading_source.angle_ms)==7);
 results[7]=1;return 0;
}
'''
    (OUT/'unit.c').write_bytes((prelude+publish+body).encode('gbk'))
    for name in ['hwt101_parser.c','hwt101_parser.h']:shutil.copyfile(USER/name,OUT/name)
    (OUT/'unit.sct').write_text('LR_ROM 0x08000000 0x00100000 { ER_ROM 0x08000000 0x00100000 { unit.o (i.test_entry, +First) *(+RO) } RW_RAM 0x20000000 0x00020000 { *(+RW,+ZI) } }\n')
    arm=Path('D:/Keil_v5/ARM/arm5_compiler/bin');logs=[]
    commands=[[str(arm/'armcc.exe'),'--cpu','Cortex-M3','--c99','--split_sections','-O1','-c',src,'-o',obj] for src,obj in [('unit.c','unit.o'),('hwt101_parser.c','parser.o')]]
    commands.append([str(arm/'armlink.exe'),'--entry','test_entry','--scatter','unit.sct','unit.o','parser.o','--library_type=microlib','-o','unit.axf'])
    for command in commands:
        p=subprocess.run(command,cwd=OUT,capture_output=True,text=True,errors='replace',timeout=60)
        logs.append(dict(command=command,returncode=p.returncode,stdout=p.stdout,stderr=p.stderr))
        (OUT/'build.json').write_text(json.dumps(logs,indent=2))
        if p.returncode:raise RuntimeError(p.stdout+p.stderr)
    sys.path.insert(0,str(ROOT/'工创赛/tmp/position_alignment_deps'))
    import unicorn as uc
    from unicorn import arm_const as reg
    machine=uc.Uc(uc.UC_ARCH_ARM,uc.UC_MODE_THUMB|uc.UC_MODE_MCLASS)
    machine.mem_map(0x08000000,0x100000);machine.mem_map(0x20000000,0x20000)
    blob=(OUT/'unit.axf').read_bytes();h=struct.unpack_from('<16sHHIIIIIHHHHHH',blob)
    for i in range(h[10]):
        kind,offset,address,_,size,_,_,_=struct.unpack_from('<IIIIIIII',blob,h[5]+i*h[9])
        if kind==1 and size:machine.mem_write(address,blob[offset:offset+size])
    stop=0x080ff000;machine.reg_write(reg.UC_ARM_REG_SP,0x2001fff0);machine.reg_write(reg.UC_ARM_REG_LR,stop|1)
    machine.emu_start(h[4]|1,stop,count=1000000)
    result=machine.reg_read(reg.UC_ARM_REG_R0)
    if machine.reg_read(reg.UC_ARM_REG_PC)!=stop or result:raise RuntimeError(hex(result))
    symbols=subprocess.check_output([str(arm/'fromelf.exe'),'--text','-s','unit.axf'],cwd=OUT,text=True)
    address=int(re.search(r'\bresults\s+(0x[0-9a-fA-F]+)',symbols)[1],16)
    values=struct.unpack('<12I',machine.mem_read(address,48))
    report=dict(hardware_accessed=False,firmware_modified=False,actual_publisher_c_tested=True,
        actual_parser_c_tested=True,invalid_age_wrap_reproduced=True,
        values=dict(zip(['valid_angle_ms','invalid_sample_ms','invalid_angle_ms','misleading_age_at_fault_ms',
                        'healthy_with_invalid_age','partial_angle_age_ms','partial_rate_age_ms','genuine_clock_wrap_passed'],values[:8])),
        main_sha256=hashlib.sha256((USER/'main.c').read_bytes()).hexdigest(),
        conclusions=['Invalid age UINT32_MAX becomes a future reconstructed timestamp; health remains false.',
        '7ms in the saved source-age diagnostic does not establish a valid angle frame.',
        'Startup basic-command floating-point preparation is nested in heading runtime and backend critical sections.',
        'No heading-runtime critical-duration measurement or armed HWT event was retained for this failure.',
        'Interrupt masking is a candidate cause of ORE, not a measured physical root cause.'])
    (OUT/'report.json').write_text(json.dumps(report,indent=2))
    print(json.dumps(report,indent=2))


if __name__=='__main__':main()
