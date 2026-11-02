"""运行真实Emm协议/运动学/CAN事务C，外设与RTOS为桩；不连接硬件。"""
from pathlib import Path
import argparse, hashlib, json, re, struct, subprocess, sys

ROOT=Path(__file__).resolve().parents[2]
FW=ROOT/'3.stm32f407工程'
OUT=ROOT/'工创赛/evidence/integration/2026-11-03/emm_migration'
ARM=Path('D:/Keil_v5/ARM/arm5_compiler/bin')

UNIT=r'''
#define CAN_Transmit Test_CAN_Transmit
#define CAN_TransmitStatus Test_CAN_TransmitStatus
#define CAN_MessagePending Test_CAN_MessagePending
#define CAN_Receive Test_CAN_Receive
#define CAN_CancelTransmit Test_CAN_CancelTransmit
#include "can.c"
#include "chassis_kinematics.h"
#include <math.h>
uint32_t test_tick,test_depth;
uint32_t SystemCoreClock=168000000U;
volatile uint32_t g_can_guard_ready;
volatile uint32_t g_can_poll_result[4],g_can_poll_flags[4],g_can_poll_count[4];
volatile TickType_t g_can_poll_tick[4];
volatile uint32_t checks,failures,first_failure;
static uint32_t tx_count,nonzero_count,enable_count,rx_head,rx_count,kind,bad_sequence,ack_code,no_reply;
static CanRxMsg rx_queue[8];static CanTxMsg last_tx;
static ChassisSystemSnapshot last_sample;static uint32_t sample_result;
static void ck(uint32_t x,uint32_t line){checks++;if(!x){failures++;if(!first_failure)first_failure=line;}}
#define CK(x) ck((x),__LINE__)
void Power_ReadCommandStatus(PowerCommandStatus *p){memset(p,0,sizeof(*p));}
void ChassisLoad_RecordCommand(uint8_t a,int32_t t,uint8_t k,uint32_t r,uint32_t ms){(void)a;(void)t;(void)k;(void)r;(void)ms;}
void ChassisLoad_RecordSnapshot(uint8_t a,uint32_t r,const ChassisSystemSnapshot *s,uint32_t ms){(void)a;(void)ms;sample_result=r;last_sample=*s;}
static void make_system(uint8_t *p){
 memset(p,0,36);p[0]=0x43;p[1]=31;p[2]=9;p[3]=0x5c;p[4]=0x67;p[5]=0x01;p[6]=0x2c;
 p[14]=1;p[15]=0;p[16]=50;p[17]=1;p[18]=0;p[19]=1;p[20]=0;p[21]=0;p[27]=3;p[28]=3;p[29]=0x6b;
}
static void queue_system(uint8_t address){
 uint8_t p[36],n=30,at=0,packet=0,i;make_system(p);
 if(kind==1){p[1]=37;p[2]=12;p[35]=0x6b;n=36;}
 if(kind==2)p[14]=2;
 while(at<n){CanRxMsg *m=&rx_queue[rx_count++];memset(m,0,sizeof(*m));
  m->IDE=CAN_Id_Extended;m->RTR=CAN_RTR_Data;m->ExtId=((uint32_t)address<<8)|packet;
  if(bad_sequence && packet==1)m->ExtId++;
  i=0;if(packet)m->Data[i++]=0x43;
  while(i<8 && at<n)m->Data[i++]=p[at++];m->DLC=i;packet++;
 }
}
uint8_t Test_CAN_Transmit(CAN_TypeDef *c,CanTxMsg *m){
 uint8_t a=(uint8_t)(m->ExtId>>8),f=m->Data[0];(void)c;last_tx=*m;tx_count++;
 if(f==0xf6 && m->DLC==7 && (m->Data[2]||m->Data[3]))nonzero_count++;
 if(f==0xf3 && m->Data[2])enable_count++;
 rx_head=rx_count=0;if(no_reply)return 0;
 if(f==0x43)queue_system(a);
 else {CanRxMsg *q=&rx_queue[rx_count++];memset(q,0,sizeof(*q));q->IDE=CAN_Id_Extended;q->RTR=CAN_RTR_Data;q->ExtId=(uint32_t)a<<8;
  q->Data[0]=f;
  if(f==0x35){q->DLC=5;q->Data[1]=1;q->Data[2]=0;q->Data[3]=50;q->Data[4]=0x6b;}
  else if(f==0x36){q->DLC=7;q->Data[1]=1;q->Data[2]=0;q->Data[3]=1;q->Data[4]=0;q->Data[5]=0;q->Data[6]=0x6b;}
  else {q->DLC=3;q->Data[1]=(uint8_t)ack_code;q->Data[2]=0x6b;}
 }return 0;
}
uint8_t Test_CAN_TransmitStatus(CAN_TypeDef *c,uint8_t m){(void)c;(void)m;return CAN_TxStatus_Ok;}
uint8_t Test_CAN_MessagePending(CAN_TypeDef *c,uint8_t f){(void)c;(void)f;return rx_head<rx_count;}
void Test_CAN_Receive(CAN_TypeDef *c,uint8_t f,CanRxMsg *q){(void)c;(void)f;*q=rx_queue[rx_head++];}
void Test_CAN_CancelTransmit(CAN_TypeDef *c,uint8_t m){(void)c;(void)m;}
static void reset_fixture(void){s_can1_normal_ready=1;g_can_ctrl_fault_latched=0;tx_count=nonzero_count=enable_count=rx_head=rx_count=kind=bad_sequence=no_reply=0;ack_code=2;test_tick=0;}
void test_entry(void){
 uint8_t out[8],p[36],sign;uint32_t raw,r,i,d,a,sync;float rpm;CanRxMsg reply;
 ZdtEmmSystemData parsed;ChassisWheelSpeeds wheels;ChassisMotorCommand commands[4];
 CK(CAN1_EmmSettingsConfirmed()==(ZDT_EMM_SETTINGS_CONFIRMED && !APP_MOTOR_OFFLINE_ONLY));
 /* 精确帧向量、方向、缩放目标和加速度边界。 */
 for(d=0;d<2;d++)for(a=0;a<256;a++)for(sync=0;sync<2;sync++){
  memset(out,0xaa,sizeof(out));CK(ZdtEmm_EncodeSpeed(d,123,a,sync,out));
  CK(out[0]==0xf6 && out[1]==d && out[2]==0 && out[3]==123 && out[4]==a && out[5]==sync && out[6]==0x6b && out[7]==0xaa);
 }
 CK(!ZdtEmm_EncodeSpeed(2,0,0,0,out));CK(!ZdtEmm_EncodeSpeed(0,3001,0,0,out));
 CK(!ZdtEmm_EncodeSpeed(0,0,256,0,out));CK(!ZdtEmm_EncodeSpeed(0,0,1000,0,out));
 CK(!ZdtEmm_EncodeSpeed(0,0,0,2,out));CK(!ZdtEmm_EncodeSpeed(0,0,0,0,0));
 for(i=0;i<2;i++){
  uint8_t speed[5]={0x35,0,0,50,0x6b};speed[1]=i;
  CK(ZdtEmm_DecodeSpeed(speed,5,&rpm) && rpm==(i?-50.0f:50.0f));
  CK(ZdtEmm_RpmDeci(rpm)==(i?-500:500));speed[4]=0;CK(!ZdtEmm_DecodeSpeed(speed,5,&rpm));
 }
 p[0]=0x36;p[1]=1;p[2]=0;p[3]=1;p[4]=0;p[5]=0;p[6]=0x6b;
 CK(ZdtEmm_DecodePosition(p,7,&sign,&raw) && sign==1 && raw==65536U);
 p[2]=p[3]=p[4]=p[5]=255;CK(ZdtEmm_DecodePosition(p,7,&sign,&raw) && raw==0xffffffffU);
 p[1]=2;CK(!ZdtEmm_DecodePosition(p,7,&sign,&raw));
 make_system(p);CK(ZdtEmm_DecodeSystem(p,30,&parsed));
 CK(parsed.speed_decirpm==-500 && parsed.position_counts==65536U && parsed.position_deci_deg==3600 && parsed.position_sign==1);
 CK(parsed.phase_ma==300 && parsed.bus_mv==23655 && parsed.home_flags==3 && parsed.motor_flags==3);
 for(i=0;i<30;i++)CK(!ZdtEmm_DecodeSystem(p,i,&parsed));
 for(i=0;i<4;i++){uint32_t k=i==0?9:i==1?14:i==2?17:22;make_system(p);p[k]=2;CK(!ZdtEmm_DecodeSystem(p,30,&parsed));}
 make_system(p);p[1]=37;p[2]=12;CK(!ZdtEmm_DecodeSystem(p,30,&parsed));CK(!ZdtEmm_DecodeSystem(p,36,&parsed));
 p[0]=0xf6;p[1]=2;p[2]=0x6b;CK(ZdtEmm_DecodeAck(0xf6,p,3));p[1]=0x12;CK(!ZdtEmm_DecodeAck(0xf6,p,3));
 for(i=0;i<4;i++)wheels.rpm[i]=(i&1)?-1.2f:1.2f;
 CK(Chassis_EncodeEmm(&wheels,5,commands));for(i=0;i<4;i++)CK(commands[i].speed_raw==12);
 CK(Chassis_TranslationHoldSelfTest());CK(Chassis_DistanceSelfTest());
 /* 真实CAN事务与分包解析；只替换外设/RTOS，不替换业务函数。 */
 reset_fixture();CK(CAN1_ReadSpeed(1,&reply,&rpm)==1 && rpm==-50);
 reset_fixture();CK(CAN1_ReadPosition(1,&reply,&sign,&raw)==1 && sign==1 && raw==65536U);
 reset_fixture();CK(CAN1_ReadEmmSystemSnapshot(2,&reply,&rpm)==1 && rpm==-50);
 CK(sample_result==1 && last_sample.speed_decirpm==-500 && last_sample.position_deci_deg==3600);
 reset_fixture();kind=1;CK(CAN1_ReadEmmSystemSnapshot(1,&reply,&rpm)==6);
 reset_fixture();bad_sequence=1;CK(CAN1_ReadEmmSystemSnapshot(1,&reply,&rpm)==6);
 reset_fixture();kind=2;CK(CAN1_ReadEmmSystemSnapshot(1,&reply,&rpm)==6);
 reset_fixture();no_reply=1;CK(CAN1_ReadEmmSystemSnapshot(1,&reply,&rpm)==5);
 reset_fixture();CK(CAN1_StopMotor(1,&reply)==1);CK(last_tx.DLC==4 && last_tx.Data[0]==0xfe);
 reset_fixture();CK(CAN1_SetMotorEnable(1,0,&reply)==1 && enable_count==0);
 reset_fixture();CK(CAN1_SetEmmSpeedRaw(1,0,0,256,&reply)==7 && tx_count==0);
 reset_fixture();r=CAN1_SetEmmSpeedRaw(1,0,123,236,&reply);
#if !ZDT_EMM_SETTINGS_CONFIRMED
 CK(r==9 && tx_count==0);
#elif APP_MOTOR_OFFLINE_ONLY
 CK(r==7 && tx_count==0);
#else
 CK(r==1 && last_tx.DLC==7 && last_tx.Data[2]==0 && last_tx.Data[3]==123 && last_tx.Data[4]==236);
#endif
 reset_fixture();r=CAN1_SetMotorEnable(1,1,&reply);
#if !ZDT_EMM_SETTINGS_CONFIRMED
 CK(r==9 && tx_count==0);
#elif APP_MOTOR_OFFLINE_ONLY
 CK(r==7 && enable_count==0);
#else
 CK(r==1 && tx_count==2 && enable_count==1);
 reset_fixture();kind=1;CK(CAN1_SetMotorEnable(1,1,&reply)==6 && enable_count==0);
 reset_fixture();ack_code=0x12;CK(CAN1_SetEmmSpeedRaw(1,0,123,236,&reply)==8 && g_can_ctrl_fault_latched);
 reset_fixture();CK(CAN1_StageEmmSpeedRaw(1,0,123,236,&reply)==1 && last_tx.Data[5]==1);
#endif
 reset_fixture();CK(CAN1_SetEmmSpeedRaw(1,0,0,10,&reply)==1 && nonzero_count==0);
}
'''

def main():
    p=argparse.ArgumentParser();p.add_argument('--unicorn-path',type=Path,default=ROOT/'工创赛/tmp/position_alignment_deps');args=p.parse_args()
    sys.path.insert(0,str(args.unicorn_path))
    import unicorn as uc
    from unicorn import arm_const as reg
    OUT.mkdir(parents=True,exist_ok=True);reports=[]
    for offline,confirmed in [(0,0),(0,1),(1,0),(1,1)]:
        out=OUT/f'actual_can_offline{offline}_confirmed{confirmed}';out.mkdir(exist_ok=True)
        stub=out/'stubs';stub.mkdir(exist_ok=True)
        (stub/'FreeRTOS.h').write_text('#ifndef TEST_FREERTOS_H\n#define TEST_FREERTOS_H\n#include "task.h"\n#endif\n')
        (stub/'task.h').write_text('''#ifndef TEST_TASK_H
#define TEST_TASK_H
#include <stdint.h>
typedef uint32_t TickType_t;
extern uint32_t test_tick,test_depth;
#define portTICK_PERIOD_MS 1U
#define pdMS_TO_TICKS(x) (x)
#define taskENTER_CRITICAL() (++test_depth)
#define taskEXIT_CRITICAL() (--test_depth)
static __inline uint32_t xTaskGetTickCount(void){return test_tick++;}
static __inline void vTaskDelay(TickType_t ticks){test_tick+=ticks;}
#endif
''')
        (out/'unit.c').write_text(UNIT,encoding='utf8')
        (out/'unit.sct').write_text('LR_ROM 0x08000000 0x00100000 { ER_ROM 0x08000000 0x00100000 { unit.o (i.test_entry, +First) *(+RO) } RW_RAM 0x20000000 0x00020000 { *(+RW,+ZI) } }')
        inc=[stub,FW/'CORE',FW/'USER',FW/'StdPeriph_Driver/inc',FW/'SYSTEM/CAN',FW/'SYSTEM/SAFETY',FW/'HARDWARE/POWER']
        sources=[(out/'unit.c','unit.o'),(FW/'USER/chassis_kinematics.c','kinematics.o')]
        sources += [(FW/'StdPeriph_Driver/src'/f'stm32f4xx_{n}.c',n+'.o') for n in ['can','rcc','gpio']]
        logs=[]
        for src,obj in sources:
            cmd=[str(ARM/'armcc.exe'),'--cpu','Cortex-M3','--c99','--split_sections','-O1','-DSTM32F40_41xxx','-DUSE_STDPERIPH_DRIVER',f'-DAPP_MOTOR_OFFLINE_ONLY={offline}',f'-DZDT_EMM_SETTINGS_CONFIRMED={confirmed}',*[x for d in inc for x in ('-I',str(d))],'-c',str(src),'-o',obj]
            result=subprocess.run(cmd,cwd=out,capture_output=True,text=True,errors='replace');logs.append({'command':cmd,'returncode':result.returncode,'output':result.stdout+result.stderr})
            (out/'build.json').write_text(json.dumps(logs,indent=2),encoding='utf8');assert result.returncode==0,result.stdout+result.stderr
        cmd=[str(ARM/'armlink.exe'),'--entry','test_entry','--scatter','unit.sct',*[obj for _,obj in sources],'--library_type=microlib','-o','unit.axf']
        result=subprocess.run(cmd,cwd=out,capture_output=True,text=True,errors='replace');logs.append({'command':cmd,'returncode':result.returncode,'output':result.stdout+result.stderr})
        (out/'build.json').write_text(json.dumps(logs,indent=2),encoding='utf8');assert result.returncode==0,result.stdout+result.stderr
        blob=(out/'unit.axf').read_bytes();h=struct.unpack_from('<16sHHIIIIIHHHHHH',blob)
        m=uc.Uc(uc.UC_ARCH_ARM,uc.UC_MODE_THUMB|uc.UC_MODE_MCLASS);m.mem_map(0x08000000,0x100000);m.mem_map(0x20000000,0x20000);m.mem_map(0x40000000,0x30000)
        for i in range(h[10]):
            kind,offset,address,_,size,_,_,_=struct.unpack_from('<IIIIIIII',blob,h[5]+i*h[9])
            if kind==1 and size:m.mem_write(address,blob[offset:offset+size])
        symbols=subprocess.check_output([str(ARM/'fromelf.exe'),'--text','-s','unit.axf'],cwd=out,text=True,errors='replace')
        def addr(n):return int(re.search(r'\b'+n+r'\s+(0x[0-9a-fA-F]+)',symbols)[1],16)
        stop=0x080ff000;m.reg_write(reg.UC_ARM_REG_SP,0x2001fff0);m.reg_write(reg.UC_ARM_REG_LR,stop|1);m.emu_start(addr('test_entry')|1,stop,count=20000000)
        def value(n):return struct.unpack('<I',m.mem_read(addr(n),4))[0]
        report={'offline':offline,'settings_confirmed':confirmed,'checks':value('checks'),'failures':value('failures'),'first_failure_line':value('first_failure'),'completed':m.reg_read(reg.UC_ARM_REG_PC)==stop,'hardware_accessed':False}
        reports.append(report);(OUT/'actual_can_report.json').write_text(json.dumps(reports,indent=2),encoding='utf8')
        assert report['completed'] and report['checks']>2000 and not report['failures'],report
    print(json.dumps(reports))

if __name__=='__main__':main()
