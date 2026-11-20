from pathlib import Path
import json,hashlib
R=Path.cwd();T=R/'工创赛/tmp/icm_raw_20261112/raw_test';T.mkdir(parents=True,exist_ok=True); E=R/'工创赛/evidence/navigation-planning/2026-11-12/icm-raw';B=Path('D:/Keil_v5/ARM/arm5_compiler/bin');D=R/'3.stm32f407工程/CHASSIS/HARDWARE/ICM45686'
source=r'''
#include "icm45686_raw.h"
#include <string.h>
unsigned cases,assertions;
#define CHECK(x) do { assertions++; if(!(x)) return __LINE__; }while(0)
static unsigned ms,cyc,seq,edge,pending,race,fault,reset_done,bigend,badconfig;
static unsigned short address;
static unsigned char regs[256],internal[65536];
int IcmPort_Init(void){return 0;}
unsigned IcmPort_Cycles(void){return cyc;}
unsigned IcmPort_CyclesPerUs(void){return 168;}
unsigned IcmPort_Millis(void){return ms;}
unsigned IcmPort_Lock(void){return 0;}
void IcmPort_Unlock(unsigned p){(void)p;}
void IcmPort_DelayMs(unsigned n){ms+=n;cyc+=168000*n;}
int IcmPort_DelayUs(unsigned n){cyc+=168*n;return 0;}
void IcmPort_Edge(unsigned *s,unsigned *c){*s=seq;*c=edge;}
unsigned IcmPort_Pending(void){return pending;}
void IcmPort_EnableEdge(void){seq=0;}
int IcmPort_Read(unsigned char reg,unsigned char *p,unsigned n){unsigned i;
 if(fault)return -1;
 if(reg==0 && race){seq++;edge=cyc;}
 for(i=0;i<n;i++)p[i]=reg==0x7e?internal[address++]:regs[reg+i];
 if(reg==0x19)p[0]=reset_done?128:0;
 if(reg==0x1b && badconfig)p[0]^=1;
 return 0;
}
int IcmPort_Write(unsigned char reg,const unsigned char *p,unsigned n){unsigned i;
 if(fault)return -1;
 if(reg==0x7c){address=((unsigned short)p[0]<<8)|p[1];if(n==3)internal[address]=p[2];}
 else for(i=0;i<n;i++)regs[reg+i]=p[i];return 0;
}
static void reset(void){memset(regs,0,sizeof(regs));memset(internal,0,sizeof(internal));ms=cyc=seq=edge=pending=race=fault=badconfig=0;reset_done=1;regs[0x72]=0xe9;internal[0xa267]=bigend?2:0;}
static void tick(void){ms++;cyc+=168000;seq++;edge=cyc;}
unsigned test_entry(void){IcmRawStatus s;unsigned old,i;
 reset();regs[0x72]=0;CHECK(IcmRaw_Init()!=0);IcmRaw_GetStatus(&s);CHECK(s.last_error==ICM_RAW_ID&&s.state==ICM_RAW_FAILED);cases++;
 reset();fault=1;CHECK(IcmRaw_Init()!=0);IcmRaw_GetStatus(&s);CHECK(s.last_error==ICM_RAW_TRANSFER);cases++;
 reset();reset_done=0;CHECK(IcmRaw_Init()!=0);IcmRaw_GetStatus(&s);CHECK(s.last_error==ICM_RAW_RESET);cases++;
 reset();badconfig=1;CHECK(IcmRaw_Init()!=0);IcmRaw_GetStatus(&s);CHECK(s.last_error==ICM_RAW_CONFIG);cases++;
 for(i=0;i<2;i++) {bigend=i;reset();CHECK(IcmRaw_Init()==0);regs[0]=0x80;regs[1]=0;regs[2]=0xff;regs[3]=0xff;tick();IcmRaw_Service();IcmRaw_GetStatus(&s);CHECK(s.sample_sequence==1&&s.fresh);CHECK(s.raw_accel[0]==(i?-32768:128)&&s.raw_accel[1]==-1);old=s.sample_sequence;IcmRaw_Service();IcmRaw_GetStatus(&s);CHECK(s.sample_sequence==old);cases++;}
 fault=1;tick();IcmRaw_Service();IcmRaw_GetStatus(&s);CHECK(s.sample_sequence==old&&!s.fresh&&s.raw_accel[0]==-32768);cases++;
 fault=0;race=1;tick();IcmRaw_Service();IcmRaw_GetStatus(&s);CHECK(s.sample_sequence==old&&!s.fresh&&s.read_races==1);cases++;
 race=0;tick();seq+=3;IcmRaw_Service();IcmRaw_GetStatus(&s);CHECK(s.sample_sequence==old+1&&s.skipped_edges>=5);cases++;
 old=s.sample_sequence;ms+=26000;cyc+=731032704U;seq++;edge=cyc;IcmRaw_Service();IcmRaw_GetStatus(&s);CHECK(s.sample_sequence==old&&!s.fresh);cases++;
 tick();IcmRaw_Service();IcmRaw_GetStatus(&s);CHECK(s.fresh);ms+=21;IcmRaw_GetStatus(&s);CHECK(!s.fresh);cases++;
 IcmRaw_Service();cyc=0xfffffff0;tick();IcmRaw_Service();IcmRaw_GetStatus(&s);CHECK(s.fresh);cases++;
 tick();pending=1;IcmRaw_Service();IcmRaw_GetStatus(&s);CHECK(!s.fresh);cases++;
 return 0;
}
'''
(T/'unit.c').write_text(source,'utf8');(T/'unit.sct').write_text('LR_ROM 0x08000000 0x100000 { ER_ROM 0x08000000 0x100000 { unit.o (i.test_entry, +First) *(+RO) } RW_RAM 0x20000000 0x20000 { *(+RW,+ZI) } }')
commands=[[str(B/'armcc.exe'),'--cpu','Cortex-M3','--c99','--gnu','--split_sections','-O1','-I'+str(D),'-c',str(D/'icm45686_raw.c'),'-o','raw.o'],[str(B/'armcc.exe'),'--cpu','Cortex-M3','--c99','--gnu','--split_sections','-O1','-I'+str(D),'-c','unit.c','-o','unit.o'],[str(B/'armlink.exe'),'--entry','test_entry','--scatter','unit.sct','unit.o','raw.o','--library_type=microlib','-o','unit.axf']]
from radar_zone2_arm_runner_20261111 import run
result=run(R,T,B,source,commands)
result['source_sha256']={str(p.relative_to(R)).replace('\\','/'):hashlib.sha256(p.read_bytes()).hexdigest() for p in list(D.glob('*.[ch]'))+[R/'3.stm32f407工程/CHASSIS/USER/main.c',R/'3.stm32f407工程/gongxun-f407.uvprojx']}
(E/'raw_arm_result.json').write_text(json.dumps({**result,'actual_raw_source':True,'port_stubs':True,'hardware':False},indent=2),'utf8')
