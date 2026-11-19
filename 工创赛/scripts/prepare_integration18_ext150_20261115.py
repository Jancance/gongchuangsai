"""Prepare an isolated150RPM/3000ms candidate; no hardware access."""
from pathlib import Path
import json
import shutil

ROOT=Path(__file__).resolve().parents[1]
BASE=ROOT/'tmp/source_top_simplified_20261114/project'
PROJECT=ROOT/'tmp/source_top_extension150_20261115/project'
EVIDENCE=ROOT/'evidence/integration/2026-11-15/integration18/ext150-clog3000'


def change(relative,old,new,count=1):
    p=PROJECT/relative;data=p.read_bytes();old=old.encode('ascii');new=new.encode('ascii')
    assert data.count(old)==count,(relative,data.count(old),count)
    p.write_bytes(data.replace(old,new))


def main():
    model=json.loads((EVIDENCE/'matlab-result.json').read_text(encoding='utf-8'))
    assert model['passed'] and model['local_matlab']
    assert not PROJECT.exists(),'Preserve prior candidate'
    shutil.copytree(BASE,PROJECT,ignore=shutil.ignore_patterns('Objects','Listings'))
    u='CHASSIS/USER/'
    change(u+'source_parallel_flow.h','#define SPF_EXT_RPM 200.0f','#define SPF_EXT_RPM 150.0f')
    change('USER/vehicle_parameters.h','#define VEHICLE_EXTENSION_RPM 200.0f','#define VEHICLE_EXTENSION_RPM 150.0f')
    change('USER/vehicle_parameters.h','#define STARTUP_EXTENSION_RPM 200.0f','#define STARTUP_EXTENSION_RPM 150.0f')
    change('USER/vehicle_parameters.h','#define SOURCE_RELEASE_DWELL_MS 50U',
           '#define SOURCE_EXTENSION_STALL_CONFIRM_MS 3000U /* User:2026-11-15; existing stall window. */\r\n#define SOURCE_RELEASE_DWELL_MS 50U')
    change(u+'vehicle_source_parallel.inc','xTaskGetTickCount()-a->stall_ms>=2000',
           'xTaskGetTickCount()-a->stall_ms>=SOURCE_EXTENSION_STALL_CONFIRM_MS',2)
    change(u+'vehicle_arm.c','now-s_stall_ms>=2000U','now-s_stall_ms>=SOURCE_EXTENSION_STALL_CONFIRM_MS')
    header=(ROOT/'software_interfaces/extension_clog_settings_20261115.h').read_bytes()
    (PROJECT/u/'extension_clog_settings.h').write_bytes(header.replace(b'\r\n',b'\n').replace(b'\n',b'\r\n'))
    change('CHASSIS/SYSTEM/CAN/can.h','uint32_t CAN1_ReadExtensionConfiguration(uint8_t out[36]);',
           'uint32_t CAN1_ReadExtensionConfiguration(uint8_t out[36]);\r\nuint32_t CAN1_ApplyExtensionClog3000(uint8_t before[36],uint8_t after[36],uint32_t *ack);')
    writer=r'''
/* Explicit node6 parameter operation, sole CAN task only. No motion/clear/reset. */
#include "../../USER/extension_clog_settings.h"
uint32_t CAN1_ApplyExtensionClog3000(uint8_t before[36],uint8_t after[36],uint32_t *ack)
{
#if APP_MOTOR_OFFLINE_ONLY
 (void)before;(void)after;(void)ack;return 9U;
#else
 uint8_t frame[36],packet=0U,offset=1U,chunk,mailbox,status;
 uint32_t result;TickType_t start;CanTxMsg tx;CanRxMsg rx;
 if(!before || !after || !ack)return 7U;
 *ack=0U;memset(before,0,36U);memset(after,0,36U);
 result=CAN1_ReadExtensionConfiguration(before);if(result!=1U)return result;
 if(!ExtensionClog3000_Frame(before,frame))return 7U;
 if(before[31]==0x0bU && before[32]==0xb8U){memcpy(after,before,36U);return 1U;}
 while(offset<36U){
  memset(&tx,0,sizeof(tx));chunk=(uint8_t)(36U-offset);if(chunk>7U)chunk=7U;
  tx.ExtId=(6U<<8)|packet;tx.IDE=CAN_Id_Extended;tx.RTR=CAN_RTR_Data;
  tx.DLC=chunk+1U;tx.Data[0]=0x48U;memcpy(&tx.Data[1],&frame[offset],chunk);
  mailbox=CAN_Transmit(CAN1,&tx);if(mailbox>2U)return 4U;
  start=xTaskGetTickCount();
  for(;;){status=CAN_TransmitStatus(CAN1,mailbox);if(status==CAN_TxStatus_Ok)break;
   if(status==CAN_TxStatus_Failed || (TickType_t)(xTaskGetTickCount()-start)>=pdMS_TO_TICKS(20U)){
    CAN_CancelTransmit(CAN1,mailbox);return 4U;}
   vTaskDelay(pdMS_TO_TICKS(1U));}
  offset+=chunk;packet++;
 }
 start=xTaskGetTickCount();
 while((TickType_t)(xTaskGetTickCount()-start)<pdMS_TO_TICKS(80U)){
  if(CAN_MessagePending(CAN1,CAN_FIFO0)){
   StartupHome_Receive(&rx);
   if(rx.IDE==CAN_Id_Extended && rx.RTR==CAN_RTR_Data && rx.ExtId==(6U<<8) &&
      rx.DLC==3U && rx.Data[0]==0x48U && rx.Data[2]==0x6bU){*ack=rx.Data[1];break;}
  }
  vTaskDelay(pdMS_TO_TICKS(1U));
 }
 /* Read after even when the ACK was lost. Never invent ACK or repeat a write. */
 result=CAN1_ReadExtensionConfiguration(after);if(result!=1U)return result;
 if(!ExtensionClog3000_Readback(before,after))return 6U;
 return *ack==2U?1U:5U;
#endif
}

'''.replace('\n','\r\n')
    change('CHASSIS/SYSTEM/CAN/can.c','uint32_t VehicleArm_Exchange(uint8_t address',writer+'uint32_t VehicleArm_Exchange(uint8_t address')
    p=PROJECT/u/'low_readonly_diag.inc';data=p.read_bytes()
    key=b'volatile uint32_t g_recovery_diag_request,g_recovery_diag_cycle,g_recovery_diag_done;'
    assert data.count(key)==1
    data=data.replace(key,key+b'\r\n/* request1 remains read-only; request2 is explicit user-approved Clog_Ms=3000. */\r\nvolatile uint32_t g_recovery_config_ack,g_recovery_config_result;\r\nvolatile uint8_t g_recovery_config_before[36],g_recovery_config_after[36];')
    data=data.replace(b'PowerCommandStatus power;CanRxMsg reply;uint32_t i,j;',b'PowerCommandStatus power;CanRxMsg reply;uint32_t i,j,request,ack,result;uint8_t before[36],after[36];')
    old=b' if(g_recovery_diag_request!=1U)return;\r\n g_recovery_diag_request=0U;'
    new=b' request=g_recovery_diag_request;if(request!=1U && request!=2U)return;\r\n g_recovery_diag_request=0U;'
    assert old in data;data=data.replace(old,new)
    marker=b' memset(configuration,0,sizeof(configuration));'
    block=r'''
 if(request==2U){
  /* Reuse normal status/zero-speed checks. This request never runs in motion. */
  result=9U;ack=0U;memset(before,0,sizeof(before));memset(after,0,sizeof(after));
  if(g_recovery_diag_result[0]==1U && g_recovery_diag_result[1]==1U &&
     g_recovery_diag_result[2]==1U && g_recovery_diag_result[3]==1U &&
     !(g_recovery_diag_raw[1][1]&0x0cU) &&
     g_recovery_diag_raw[2][1]<=1U && !g_recovery_diag_raw[2][2] && !g_recovery_diag_raw[2][3])
   result=CAN1_ApplyExtensionClog3000(before,after,&ack);
  g_recovery_config_result=result;g_recovery_config_ack=ack;g_recovery_diag_result[4]=result;
  for(i=0U;i<36U;i++){
   g_recovery_config_before[i]=before[i];g_recovery_config_after[i]=after[i];
   g_recovery_diag_config[i]=after[i];
  }
  g_recovery_diag_done=1U;g_recovery_diag_cycle++;return;
 }
'''.replace('\n','\r\n').encode('ascii')
    assert data.count(marker)==1;p.write_bytes(data.replace(marker,block+marker))
    # Retain provenance for unchanged geometry and wheel-hold models.
    for name in ['body-registration-matlab.json','wheel-hold-rearm-matlab.json']:
        (EVIDENCE/name).write_bytes((ROOT/'evidence/integration/2026-11-14/overlap200-simplified'/name).read_bytes())
    print('Prepared local candidate',PROJECT)


if __name__=='__main__':main()
