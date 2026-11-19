"""Prepare isolated original-speed left/right synchronized pickup after MATLAB."""
from pathlib import Path
import json
import shutil

R=Path(__file__).resolve().parents[1]
BASE=R/'tmp/source_release100_20261115/project'
P=R/'tmp/source_integration23_sync_20261115/project'
E=R/'evidence/integration/2026-11-15/integration23-sync'

def replace(path, old, new):
    data=path.read_bytes()
    old=old.encode('ascii');new=new.encode('ascii')
    assert data.count(old)==1,(path,old)
    path.write_bytes(data.replace(old,new,1))

def main():
    proof=json.loads((E/'matlab-result.json').read_text(encoding='utf-8'))
    assert proof['passed'] and proof['local_matlab']
    assert not P.exists(),'Preserve existing candidate'
    shutil.copytree(BASE,P,ignore=shutil.ignore_patterns('Objects','Listings'))
    owner=P/'CHASSIS/USER/pzq_commit_overlap.inc'
    replace(owner,'  vsp_pick_lead_ms=SourcePick_ExtensionLeadMs(g_low_region);\r\n'
        '  vsp_group.delayed_mask=SPF_LIFT;vsp_group.release_mask=SPF_EXT;',
        '  if(g_low_region<2U){vsp_pick_sync=1U;}else{\r\n'
        '   vsp_pick_lead_ms=SourcePick_ExtensionLeadMs(g_low_region);\r\n'
        '   vsp_group.delayed_mask=SPF_LIFT;vsp_group.release_mask=SPF_EXT;\r\n  }')
    backend=P/'CHASSIS/USER/vehicle_source_parallel.inc'
    replace(backend,'static uint32_t vsp_pick_lead_ms;',
        'static uint32_t vsp_pick_lead_ms;\r\nstatic uint32_t vsp_pick_sync,vsp_pick_queued;\r\n'
        'volatile uint32_t g_source_parallel_sync_tick;')
    helper='''/* Both different goals are cached first. One CAN broadcast releases X/Z.
 * Success here means transmitted, never measured physical simultaneity. */
static uint32_t VSP_StartQueuedPick(uint32_t bit,VSP_Axis *a,uint8_t address)
{
 uint32_t r;
 a->move[13]=1U;
 r=VehicleArm_Exchange(address,a->move,15U,3U,&s_rx);
 if(address==6U){if(r!=1U && r!=5U)return 0U;}
 else if(r!=1U || (s_rx.Data[1]!=2U && s_rx.Data[1]!=0x9fU))return 0U;
 vsp_pick_queued|=bit;
 if(vsp_pick_queued==(SPF_EXT|SPF_LIFT)){
  if(CAN1_StartQueuedSourceAxes()!=1U)return 0U;
  g_source_parallel_sync_tick=(uint32_t)xTaskGetTickCount();
 }
 return 1U;
}
'''
    replace(backend,'static uint32_t VSP_Start(void *context,uint32_t bit,const SPF_Command *c)',
        helper.replace('\n','\r\n')+'static uint32_t VSP_Start(void *context,uint32_t bit,const SPF_Command *c)')
    replace(backend,' if(fabsf(a->goal-a->before)<=.2f)return 1;',
        ' if(vsp_pick_sync)return VSP_StartQueuedPick(bit,a,address);\r\n'
        ' if(fabsf(a->goal-a->before)<=.2f)return 1;')
    replace(backend,' vsp_pick_lead_ms=0U;memset(vsp_axis,0,sizeof(vsp_axis));',
        ' vsp_pick_sync=vsp_pick_queued=g_source_parallel_sync_tick=0U;\r\n'
        ' vsp_pick_lead_ms=0U;memset(vsp_axis,0,sizeof(vsp_axis));')
    replace(backend,' if(!vsp_owned || !q || (address!=5U && address!=6U))return 0U;',
        ' if(!vsp_owned || !q)return 0U;\r\n'
        ' if(address==0U)return vsp_pick_sync && vsp_pick_queued==6U &&\r\n'
        '  vsp_group.state==1U && vsp_group.started==6U && n==3U &&\r\n'
        '  q[0]==0xffU && q[1]==0x66U && q[2]==0x6bU;\r\n'
        ' if(address!=5U && address!=6U)return 0U;')
    can=P/'CHASSIS/SYSTEM/CAN/can.c'
    broadcast='''uint32_t CAN1_StartQueuedSourceAxes(void)
{
#if APP_MOTOR_OFFLINE_ONLY
 return 9U;
#else
 extern uint32_t VehicleArm_SourceParallelCanAllowed(uint8_t,const uint8_t*,uint32_t);
 uint8_t q[3]={0xffU,0x66U,0x6bU},mailbox,status;CanTxMsg tx;TickType_t start;
 if(g_safety_fault || g_rtos_fault || g_can_ctrl_fault_latched ||
    !g_startup_axes_vm_request || !s_can1_normal_ready ||
    !VehicleArm_SourceParallelCanAllowed(0U,q,3U))return 9U;
 memset(&tx,0,sizeof(tx));tx.IDE=CAN_Id_Extended;tx.RTR=CAN_RTR_Data;
 tx.ExtId=0U;tx.DLC=3U;memcpy(tx.Data,q,3U);
 mailbox=CAN_Transmit(CAN1,&tx);if(mailbox>2U)return 4U;
 start=xTaskGetTickCount();
 for(;;){
  status=CAN_TransmitStatus(CAN1,mailbox);
  if(status==CAN_TxStatus_Ok)return 1U; /* bus TX, not a drive ACK */
  if(status==CAN_TxStatus_Failed ||
     (TickType_t)(xTaskGetTickCount()-start)>=pdMS_TO_TICKS(CAN1_SINGLE_TX_WAIT_MS)){
   CAN_CancelTransmit(CAN1,mailbox);return 4U;
  }
  vTaskDelay(pdMS_TO_TICKS(1U));
 }
#endif
}

'''
    # Place after Exchange so existing parameter-writer extraction stays exact.
    replace(can,'uint32_t StartupAxis_Exchange(const uint8_t *q,uint8_t n,uint8_t reply_n,void *out)',
        broadcast.replace('\n','\r\n')+'uint32_t StartupAxis_Exchange(const uint8_t *q,uint8_t n,uint8_t reply_n,void *out)')
    replace(can,'q[12]!=2 || q[13]!=0 || q[14]!=0x6b',
        'q[12]!=2 || (q[13]!=0U && !(q[13]==1U && VehicleArm_SourceParallelCanAllowed(address,q,length))) || q[14]!=0x6b')
    replace(can,'uint32_t VehicleArm_Exchange(uint8_t address,',
        'extern uint32_t VehicleArm_SourceParallelCanAllowed(uint8_t,const uint8_t*,uint32_t);\r\n'
        'uint32_t VehicleArm_Exchange(uint8_t address,')
    header=P/'CHASSIS/SYSTEM/CAN/can.h'
    replace(header,'uint32_t CAN1_HoldWheelCurrentPosition(uint8_t address,CanRxMsg *reply);',
        'uint32_t CAN1_HoldWheelCurrentPosition(uint8_t address,CanRxMsg *reply);\r\n'
        'uint32_t CAN1_StartQueuedSourceAxes(void);')
    print('Prepared isolated synchronized left/right pickup + carrier release100; no hardware access')

if __name__=='__main__':main()
