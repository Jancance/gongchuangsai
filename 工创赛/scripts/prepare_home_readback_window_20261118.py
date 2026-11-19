"""Use the existing parameter-read window for extension, after MATLAB."""
from pathlib import Path
import hashlib,json,shutil,subprocess
from check_stage2_extension_mirror_20261117 import function
R=Path(__file__).resolve().parents[1]
BASE=R/'tmp/home_direction_diag_20261117/project'
P=R/'tmp/home_readback250_20261118/project'
E=R/'evidence/integration/2026-11-18/home-readback-window'


def main():
    assert json.loads((E/'matlab-result.json').read_text())['passed']
    assert not P.exists(),'Do not overwrite an existing candidate'
    path=Path('CHASSIS/SYSTEM/CAN/can.c');b=(BASE/path).read_bytes()
    sig='uint32_t StartupAxis_ReadHomeParameters('
    old=function(b.decode('gb18030'),sig).encode('gb18030')
    needle=b'pdMS_TO_TICKS(CAN1_SINGLE_REPLY_WAIT_MS)'
    assert old.count(needle)==1
    new=old.replace(needle,b'pdMS_TO_TICKS(CAN1_PARAM_REPLY_WAIT_MS)')
    assert b'#define CAN1_PARAM_REPLY_WAIT_MS 250U' in b
    shutil.copytree(BASE,P,ignore=shutil.ignore_patterns('Objects','Listings'))
    (P/path).write_bytes(b.replace(old,new,1))
    # Run the actual reader, including native frame assembly and guard exits.
    stubs=r'''
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#define CAN1 0
#define CAN_FIFO0 0
#define CAN_Mode_Normal 0
#define CAN_Id_Extended 4
#define CAN_RTR_Data 0
#define CAN_TxStatus_NoMailBox 4
#define CAN_TxStatus_Ok 1
#define CAN_TxStatus_Failed 2
#define CAN1_SINGLE_TX_WAIT_MS 20
#define CAN1_SINGLE_REPLY_WAIT_MS 60
#define CAN1_PARAM_REPLY_WAIT_MS 250
#define pdMS_TO_TICKS(x) (x)
typedef unsigned TickType_t;
typedef struct{unsigned ExtId,IDE,RTR,DLC;uint8_t Data[8];}CanTxMsg;
typedef CanTxMsg CanRxMsg;
static unsigned g_startup_axes_vm_request=1,g_startup_axes_state=2,g_safety_fault,g_rtos_fault,g_can_ctrl_fault_latched,s_can1_normal_ready=1;
static unsigned tick,sent,packet,delay,bad,checks,txcount;
static uint8_t params[17]={0x22,0,1,0,30,0,0,0x27,0x10,1,0x2c,3,0x20,0,60,0,0x6b};
#define CHECK(x) do{checks++;if(!(x)){printf("FAIL %u %s\n",__LINE__,#x);exit(1);}}while(0)
static unsigned CAN1_InitMode(unsigned m){(void)m;return 1;}
static unsigned xTaskGetTickCount(void){return tick++;}
static void vTaskDelay(unsigned ms){tick+=ms;}
static unsigned CAN_MessagePending(unsigned c,unsigned f){(void)c;(void)f;return sent && packet<3 && tick>=delay+5*packet;}
static unsigned CAN_Transmit(unsigned c,CanTxMsg*t){(void)c;CHECK(t->ExtId==0x600 && t->DLC==2 && t->Data[0]==0x22 && t->Data[1]==0x6b);sent=1;txcount++;return 0;}
static unsigned CAN_TransmitStatus(unsigned c,unsigned m){(void)c;(void)m;return 1;}
static void CAN_CancelTransmit(unsigned c,unsigned m){(void)c;(void)m;}
static void StartupHome_Receive(CanRxMsg*r){unsigned offset=1+7*packet,n=17-offset;if(n>7)n=7;
 memset(r,0,sizeof(*r));r->IDE=CAN_Id_Extended;r->RTR=CAN_RTR_Data;r->ExtId=0x600|packet;r->DLC=n+1;r->Data[0]=0x22;memcpy(r->Data+1,params+offset,n);if(bad==packet+1)r->DLC--;packet++;}
'''
    suffix=r'''
int main(void){uint8_t out[17];unsigned i,r;
 for(i=0;i<5;i++){tick=sent=packet=bad=txcount=0;delay=(unsigned[]){0,15,30,40,45}[i];r=StartupAxis_ReadHomeParameters(out);CHECK(r==1 && !memcmp(out,params,17) && txcount==1);}
 for(i=1;i<=3;i++){tick=sent=packet=txcount=0;delay=0;bad=i;CHECK(StartupAxis_ReadHomeParameters(out)==6);}
 tick=sent=packet=bad=txcount=0;delay=300;CHECK(StartupAxis_ReadHomeParameters(out)==5 && txcount==1);
 g_safety_fault=1;txcount=0;CHECK(StartupAxis_ReadHomeParameters(out)==9 && !txcount);g_safety_fault=0;
 for(i=0;i<3;i++){tick=sent=packet=bad=txcount=0;delay=(unsigned[]){65,100,200}[i];r=StartupAxis_ReadHomeParameters(out);
 #ifdef LONG_WINDOW
 CHECK(r==1 && !memcmp(out,params,17) && txcount==1);
 #else
 CHECK(r==5 && txcount==1);
 #endif
 }
 printf("PASS actual reader %u checks\n",checks);return 0;}
'''
    results={}
    for name,body in (('original',old),('window250',new)):
        src=E/('actual-reader-'+name+'.c');src.write_text(stubs+body.decode('ascii')+suffix)
        exe=src.with_suffix('.exe')
        args=['gcc','-std=c99','-Wall','-Wextra']+(['-DLONG_WINDOW'] if name=='window250' else [])+[str(src),'-o',str(exe)]
        build=subprocess.run(args,capture_output=True,text=True);(E/(src.stem+'-build.log')).write_text(build.stdout+build.stderr);assert build.returncode==0,build.stderr
        run=subprocess.run([str(exe)],capture_output=True,text=True);(E/(src.stem+'.log')).write_text(run.stdout+run.stderr);assert run.returncode==0,run.stdout
        results[name]=run.stdout.strip()
    (E/'checks.json').write_text(json.dumps(dict(passed=True,actual_reader_tests=results,only_parameter_window_changed=True,baseline_sha256=hashlib.sha256(b).hexdigest(),hardware_access=False),indent=2)+'\n')
    print(json.dumps(results),flush=True)


if __name__=='__main__':main()
