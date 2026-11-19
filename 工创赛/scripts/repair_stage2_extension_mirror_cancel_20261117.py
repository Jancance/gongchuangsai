"""MATLAB-gated narrow cancellation commits; only bounded register/state operations."""
import json
from prepare_stage2_extension_mirror_20261117 import P,E,edit

def main():
 assert json.loads((E/'matlab-cancel-result.json').read_text())['passed']
 edit(P/'CHASSIS/USER/extension_home_only.inc',[
  ('if(g_startup_axes_state!=2U || !g_startup_axes_vm_request){eh_stop_pending=1U;return;}\n            g_startup_axes_state=3U;g_extension_home_state=6U;eh_phase=0U;eh_poll=now;return;',
   '''taskENTER_CRITICAL();
            if(g_startup_axes_state!=2U || !g_startup_axes_vm_request){
                eh_stop_pending=1U;taskEXIT_CRITICAL();return;}
            g_startup_axes_state=3U;g_extension_home_state=6U;eh_phase=0U;eh_poll=now;
            taskEXIT_CRITICAL();return;''')])
 edit(P/'CHASSIS/SYSTEM/CAN/can.c',[
  ('    if (s_can_ctrl_trace_active) g_can_ctrl_trace_phase = 2U;\n    mailbox = CAN_Transmit(CAN1, &tx);\n    if (mailbox == CAN_TxStatus_NoMailBox)',
   '''    if (s_can_ctrl_trace_active) g_can_ctrl_trace_phase = 2U;
#if (VEHICLE_EXTENSION_HOME_ONLY || VEHICLE_STARTUP_GROUP_ENABLE)
    if(address==6U && request_length==4U && request[0]==0x9aU){
        extern volatile uint32_t g_startup_axes_vm_request,g_startup_axes_state;
        taskENTER_CRITICAL();
        if(!g_startup_axes_vm_request || g_startup_axes_state!=2U || g_safety_fault || g_rtos_fault || g_can_ctrl_fault_latched){
            taskEXIT_CRITICAL();return 9U;}
        mailbox=CAN_Transmit(CAN1,&tx);
        taskEXIT_CRITICAL();
    }else
#endif
    mailbox = CAN_Transmit(CAN1, &tx);
    if (mailbox == CAN_TxStatus_NoMailBox)'''),
  ('            mailbox=CAN_Transmit(CAN1,&tx);if(mailbox>2U)return 4U;\n            start=xTaskGetTickCount();',
   '''            taskENTER_CRITICAL();
            if(!g_startup_axes_vm_request || g_startup_axes_state!=2U || g_safety_fault || g_rtos_fault || g_can_ctrl_fault_latched){
                taskEXIT_CRITICAL();return 9U;}
            mailbox=CAN_Transmit(CAN1,&tx);
            taskEXIT_CRITICAL();
            if(mailbox>2U)return 4U;
            start=xTaskGetTickCount();''')])
 print('Applied prevalidated narrow owner enqueue/state commits; no hardware.')
if __name__=='__main__':main()
