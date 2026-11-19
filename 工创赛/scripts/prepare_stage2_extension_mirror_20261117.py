"""Create a MATLAB-gated isolated extension mirror candidate; no hardware."""
from pathlib import Path
import json,shutil
R=Path(__file__).resolve().parents[1]
BASE=R/'tmp/stage2_settle_ring_20261117/project'
P=R/'tmp/stage2_extension_mirror_20261117/project'
E=R/'evidence/integration/2026-11-17/stage2-extension-mirror'

def edit(path,pairs):
 b=path.read_bytes()
 for old,new in pairs:
  newline='\r\n' if old.replace('\n','\r\n').encode('ascii') in b else '\n'
  old=old.replace('\n',newline).encode('ascii');new=new.replace('\n',newline).encode('ascii')
  assert b.count(old)==1,(path,old,b.count(old));b=b.replace(old,new,1)
 path.write_bytes(b)

def main():
 assert json.loads((E/'matlab-result.json').read_text())['passed']
 assert not P.exists(),'Preserve previous candidate'
 shutil.copytree(BASE,P,ignore=shutil.ignore_patterns('Objects','Listings'))
 edit(P/'USER/vehicle_parameters.h',[
  ('#define VEHICLE_EXTENSION_HOME_DIRECTION 0U','#define VEHICLE_EXTENSION_HOME_DIRECTION 1U'),
  ('#define STARTUP_EXTENSION_EXTEND_SIGN (-1)','#define STARTUP_EXTENSION_EXTEND_SIGN (1)')])
 edit(P/'CHASSIS/USER/extension_home_only.inc',[
  ('static CanRxMsg eh_reply;','static CanRxMsg eh_reply;\nstatic uint8_t eh_home_parameters[17];\nuint32_t StartupAxis_SetHomeDirection(const uint8_t before[17],uint8_t after[17]);'),
  ('if(VEHICLE_EXTENSION_HOME_DIRECTION!=0U || VEHICLE_EXTENSION_HOME_RPM!=30U ||',
   'if(VEHICLE_EXTENSION_HOME_DIRECTION!=(STARTUP_EXTENSION_EXTEND_SIGN>0?1U:0U) || VEHICLE_EXTENSION_HOME_RPM!=30U ||'),
  ('if(p[0]!=0x22 || p[16]!=0x6b || p[2]!=VEHICLE_EXTENSION_HOME_DIRECTION ||',
   'if(p[0]!=0x22 || p[16]!=0x6b || p[2]>1U ||'),
  ('eh_u16(p+13)!=VEHICLE_EXTENSION_HOME_DETECT_TIME_MS || p[15]!=0U){eh_fail(7U);return;}break;',
   'eh_u16(p+13)!=VEHICLE_EXTENSION_HOME_DETECT_TIME_MS || p[15]!=0U){eh_fail(7U);return;}\n            memcpy(eh_home_parameters,p,17U);break;'),
  ('if(eh_reply.Data[2] || eh_reply.Data[3]){eh_fail(5U);return;}break;\n        default:',
   '''if(eh_reply.Data[2] || eh_reply.Data[3]){eh_fail(5U);return;}
            /* Direction-only, RAM-only change after the existing exact-zero check.
             * The node6 helper always performs fresh complete parameter readback. */
            raw=StartupAxis_SetHomeDirection(eh_home_parameters,p);
            if(g_startup_axes_state!=2U || !g_startup_axes_vm_request)return;
            if(raw!=1U){eh_fail(raw==7U?7U:4U);return;}break;
        default:'''),
  ('eh_started=1U;eh_motion=now;g_extension_home_capture=g_extension_home_done_notice=0U;',
   'if(g_startup_axes_state!=2U || !g_startup_axes_vm_request)return;\n            eh_started=1U;eh_motion=now;g_extension_home_capture=g_extension_home_done_notice=0U;'),
  ('if(!eh_control(home,4U)){eh_fail(4U);return;}\n            g_startup_axes_state=3U;',
   'if(!eh_control(home,4U)){eh_fail(4U);return;}\n            if(g_startup_axes_state!=2U || !g_startup_axes_vm_request){eh_stop_pending=1U;return;}\n            g_startup_axes_state=3U;')])
 # Keep existing tolerances, but derive the signed physical target from the one axis convention.
 for name,old,new in [
  ('integration10_complete.inc','g_startup_extension_current_deg-g_extension_home_reference_deg+\n   g_it_extension_mm*','g_startup_extension_current_deg-g_extension_home_reference_deg-\n   STARTUP_EXTENSION_EXTEND_SIGN*g_it_extension_mm*'),
  ('integration10_complete.inc','g_extension_home_reference_deg-48.0f*3.6f','g_extension_home_reference_deg+STARTUP_EXTENSION_EXTEND_SIGN*48.0f*3.6f'),
  ('pzq_commit_overlap.inc','sample.current-g_extension_home_reference_deg+\n  mm*','sample.current-g_extension_home_reference_deg-\n  STARTUP_EXTENSION_EXTEND_SIGN*mm*'),
  ('pzq_region_batch.inc','g_startup_extension_current_deg-g_extension_home_reference_deg+\n  65.0f*','g_startup_extension_current_deg-g_extension_home_reference_deg-\n  STARTUP_EXTENSION_EXTEND_SIGN*65.0f*'),
  ('stage3_cycle.inc','(g_extension_home_reference_deg-g_startup_extension_current_deg)*','STARTUP_EXTENSION_EXTEND_SIGN*(g_startup_extension_current_deg-g_extension_home_reference_deg)*')]:
  edit(P/'CHASSIS/USER'/name,[(old,new)])
 helper=r'''
/* Sole CAN-task node6 operation. Fixed save=0; never writes wheel parameters.
 * Only the direction byte changes; all other bytes must match fresh readback. */
uint32_t StartupAxis_SetHomeDirection(const uint8_t before[17],uint8_t after[17])
{
    uint8_t q[19],expected[17],fresh[17],offset=1U,packet=0U,chunk,mailbox,status,flush;
    uint32_t result;TickType_t start;CanTxMsg tx;CanRxMsg rx;
    if(!before || !after || before[0]!=0x22U || before[16]!=0x6bU || before[2]>1U ||
       before[15]!=0U || VEHICLE_EXTENSION_HOME_DIRECTION>1U)return 7U;
    if(!g_startup_axes_vm_request || g_startup_axes_state!=2U || g_safety_fault || g_rtos_fault || g_can_ctrl_fault_latched)return 9U;
    result=StartupAxis_ReadHomeParameters(fresh);if(result!=1U)return result;
    if(memcmp(fresh,before,17U))return 7U;
    memcpy(expected,before,17U);expected[2]=VEHICLE_EXTENSION_HOME_DIRECTION;
    if(before[2]!=VEHICLE_EXTENSION_HOME_DIRECTION){
        q[0]=0x4cU;q[1]=0xaeU;q[2]=0U;q[3]=before[1];q[4]=VEHICLE_EXTENSION_HOME_DIRECTION;
        memcpy(q+5,before+3,13U);q[18]=0x6bU;
        for(flush=0U;flush<3U && CAN_MessagePending(CAN1,CAN_FIFO0);flush++)StartupHome_Receive(&rx);
        while(offset<19U){
            if(!g_startup_axes_vm_request || g_startup_axes_state!=2U || g_safety_fault || g_rtos_fault || g_can_ctrl_fault_latched)return 9U;
            memset(&tx,0,sizeof(tx));chunk=(uint8_t)(19U-offset);if(chunk>7U)chunk=7U;
            tx.ExtId=0x0600U|packet;tx.IDE=CAN_Id_Extended;tx.RTR=CAN_RTR_Data;
            tx.DLC=chunk+1U;tx.Data[0]=0x4cU;memcpy(tx.Data+1,q+offset,chunk);
            mailbox=CAN_Transmit(CAN1,&tx);if(mailbox>2U)return 4U;
            start=xTaskGetTickCount();
            for(;;){status=CAN_TransmitStatus(CAN1,mailbox);if(status==CAN_TxStatus_Ok)break;
                if(status==CAN_TxStatus_Failed || (TickType_t)(xTaskGetTickCount()-start)>=pdMS_TO_TICKS(CAN1_SINGLE_TX_WAIT_MS)){
                    CAN_CancelTransmit(CAN1,mailbox);return 4U;}
                vTaskDelay(pdMS_TO_TICKS(1U));}
            offset+=chunk;packet++;
        }
        result=5U;start=xTaskGetTickCount();
        while((TickType_t)(xTaskGetTickCount()-start)<pdMS_TO_TICKS(CAN1_SINGLE_REPLY_WAIT_MS)){
            if(!g_startup_axes_vm_request || g_startup_axes_state!=2U || g_safety_fault || g_rtos_fault || g_can_ctrl_fault_latched)return 9U;
            if(CAN_MessagePending(CAN1,CAN_FIFO0)){
                StartupHome_Receive(&rx);
                if(rx.IDE==CAN_Id_Extended && rx.RTR==CAN_RTR_Data && rx.ExtId==0x0600U && rx.Data[0]==0x4cU){
                    result=(rx.DLC==3U && rx.Data[1]==2U && rx.Data[2]==0x6bU)?1U:6U;break;}
            }
            vTaskDelay(pdMS_TO_TICKS(1U));
        }
        if(result!=1U)return result; /* Never retry an uncertain parameter write. */
    }
    if(!g_startup_axes_vm_request || g_startup_axes_state!=2U || g_safety_fault || g_rtos_fault || g_can_ctrl_fault_latched)return 9U;
    result=StartupAxis_ReadHomeParameters(after);if(result!=1U)return result;
    if(!g_startup_axes_vm_request || g_startup_axes_state!=2U || g_safety_fault || g_rtos_fault || g_can_ctrl_fault_latched)return 9U;
    return memcmp(after,expected,17U)?7U:1U;
}
'''
 # Place the helper inside the same online+startup-profile guard as ReadHomeParameters.
 path=P/'CHASSIS/SYSTEM/CAN/can.c';b=path.read_bytes();anchor=b'    return 5U;\r\n}\r\n\r\n#endif\r\n'
 assert b.count(anchor)==1
 path.write_bytes(b.replace(anchor,b'    return 5U;\r\n}\r\n'+helper.replace('\n','\r\n').encode('ascii')+b'\r\n#endif\r\n',1))
 print('Prepared isolated extension-mirror implementation after MATLAB; no hardware.')

if __name__=='__main__':main()
