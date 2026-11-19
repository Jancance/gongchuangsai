"""Preserve fb2bc; add passive first-failure home-direction diagnostics only."""
from pathlib import Path
import json,shutil
R=Path(__file__).resolve().parents[1]
BASE=R/'tmp/boot_ready_beep_20261117/project'
P=R/'tmp/home_direction_diag_20261117/project'
E=R/'evidence/integration/2026-11-17/home-direction-diag'
PREFIX=r'''
/* Passive sole-CAN-task evidence. Frozen diagnostics never gate real work. */
typedef struct {
 uint32_t stage,result,frozen,read_result,fresh_valid,after_valid;
 uint32_t ack_seen,ack_extid,ack_dlc,write_tx_submit_count,write_tx_ok_count;
 uint32_t packet,offset,last_mailbox,last_status;
 uint8_t before[17],fresh[17],expected[17],after[17],ack_data[8];
} HomeDirectionDiag;
volatile HomeDirectionDiag g_home_direction_diag;
#define HD_SET(field,value) do{if(!g_home_direction_diag.frozen)g_home_direction_diag.field=(value);}while(0)
#define HD_COPY(field,src,n) do{if(!g_home_direction_diag.frozen)memcpy((void*)g_home_direction_diag.field,(src),(n));}while(0)
static uint32_t HomeDirectionDiag_Return(uint32_t result)
{
 if(!g_home_direction_diag.frozen){g_home_direction_diag.result=result;if(result!=1U)g_home_direction_diag.frozen=1U;}
 return result;
}
'''
def main():
 assert json.loads((E/'matlab-result.json').read_text())['passed']
 if P.exists():
  # A previous interrupted copy may be resumed only while byte-identical.
  existing={f.relative_to(P) for f in P.rglob('*') if f.is_file()}
  source={f.relative_to(BASE) for f in BASE.rglob('*') if f.is_file() and not any(x in ('Objects','Listings') for x in f.relative_to(BASE).parts)}
  assert existing==source,'Partial or modified candidate requires inspection'
  assert all((P/n).read_bytes()==(BASE/n).read_bytes() for n in existing),'Do not overwrite modified candidate'
 else:
  shutil.copytree(BASE,P,ignore=shutil.ignore_patterns('Objects','Listings'))
 p=P/'CHASSIS/SYSTEM/CAN/can.c';b=p.read_bytes()
 start=b.index(b'uint32_t StartupAxis_SetHomeDirection(');end=b.index(b'\r\n}\r\n',start)+3
 original=b[start:end].decode('ascii');s=original
 def rep(old,new):
  nonlocal s
  assert s.count(old)==1,(old,s.count(old));s=s.replace(old,new,1)
 rep('    if(!before', '    if(!g_home_direction_diag.frozen)memset((void*)&g_home_direction_diag,0,sizeof(g_home_direction_diag));\r\n    HD_SET(stage,1U);if(before)HD_COPY(before,before,17U);\r\n    memset(fresh,0,sizeof(fresh));\r\n    if(!before')
 # Record each unchanged exit expression, including the final compare.
 import re
 s=re.sub(r'return ([^;]+);',r'return HomeDirectionDiag_Return(\1);',s)
 guard='if(!g_startup_axes_vm_request || g_startup_axes_state!=2U || g_safety_fault || g_rtos_fault || g_can_ctrl_fault_latched)'
 rep('    '+guard+'return HomeDirectionDiag_Return(9U);\r\n    result=StartupAxis_ReadHomeParameters(fresh);', '    HD_SET(stage,2U);'+guard+'return HomeDirectionDiag_Return(9U);\r\n    HD_SET(stage,3U);result=StartupAxis_ReadHomeParameters(fresh);')
 rep('result=StartupAxis_ReadHomeParameters(fresh);if(result!=1U)', 'result=StartupAxis_ReadHomeParameters(fresh);HD_SET(read_result,result);HD_COPY(fresh,fresh,17U);HD_SET(fresh_valid,result==1U);if(result!=1U)')
 rep('    if(memcmp(fresh,before,17U))','    HD_SET(stage,4U);if(memcmp(fresh,before,17U))')
 rep('expected[2]=VEHICLE_EXTENSION_HOME_DIRECTION;','expected[2]=VEHICLE_EXTENSION_HOME_DIRECTION;HD_COPY(expected,expected,17U);')
 rep('        while(offset<19U){','        while(offset<19U){\r\n            HD_SET(stage,5U);HD_SET(packet,packet);HD_SET(offset,offset);')
 rep('            taskENTER_CRITICAL();','            HD_SET(stage,6U);taskENTER_CRITICAL();')
 rep('            taskEXIT_CRITICAL();\r\n            if(mailbox>2U)', '            taskEXIT_CRITICAL();\r\n            HD_SET(last_mailbox,mailbox);if(mailbox<=2U)HD_SET(write_tx_submit_count,g_home_direction_diag.write_tx_submit_count+1U);\r\n            if(mailbox>2U)')
 rep('status=CAN_TransmitStatus(CAN1,mailbox);if(status==CAN_TxStatus_Ok)break;', 'status=CAN_TransmitStatus(CAN1,mailbox);HD_SET(last_status,status);if(status==CAN_TxStatus_Ok){HD_SET(write_tx_ok_count,g_home_direction_diag.write_tx_ok_count+1U);break;}')
 rep('        result=5U;start=', '        HD_SET(stage,7U);result=5U;start=')
 rep('                    result=(rx.DLC', '                    HD_SET(ack_seen,1U);HD_SET(ack_extid,rx.ExtId);HD_SET(ack_dlc,rx.DLC);HD_COPY(ack_data,rx.Data,8U);\r\n                    result=(rx.DLC')
 rep('    '+guard+'return HomeDirectionDiag_Return(9U);\r\n    result=StartupAxis_ReadHomeParameters(after);', '    HD_SET(stage,8U);'+guard+'return HomeDirectionDiag_Return(9U);\r\n    HD_SET(stage,9U);result=StartupAxis_ReadHomeParameters(after);HD_SET(read_result,result);HD_COPY(after,after,17U);HD_SET(after_valid,result==1U);')
 rep('    '+guard+'return HomeDirectionDiag_Return(9U);\r\n    return HomeDirectionDiag_Return(memcmp(after,expected,17U)?7U:1U);', '    HD_SET(stage,10U);'+guard+'return HomeDirectionDiag_Return(9U);\r\n    result=memcmp(after,expected,17U)?7U:1U;if(result==1U)HD_SET(stage,11U);return HomeDirectionDiag_Return(result);')
 prefix=PREFIX.replace('\n','\r\n').encode('ascii');new=b[:start]+prefix+s.encode('ascii')+b[end:]
 p.write_bytes(new)
 (E/'helper-original.c').write_bytes(original.encode('ascii'))
 (E/'helper-instrumented.c').write_bytes(prefix+s.encode('ascii'))
 print('Prepared one-file passive diagnostic candidate; no hardware.')
if __name__=='__main__':main()
