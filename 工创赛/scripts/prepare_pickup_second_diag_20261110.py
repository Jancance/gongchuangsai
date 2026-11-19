"""Prepare read-only extension diagnostics and strict ACK classification. No hardware I/O."""
from pathlib import Path
import json,hashlib,shutil
R=Path(__file__).resolve().parents[1];B=R/'tmp/pickup_single_speed_20261110/project'
P=R/'tmp/pickup_second_diag_20261110/project';E=R/'evidence/integration/2026-11-10/second-diag'
PREV=R/'evidence/integration/2026-11-10/single-speed';E.mkdir(exist_ok=True)
assert not P.exists()
m=json.loads((PREV/'candidate.json').read_text(encoding='utf-8'))
for rel,h in m['changed'].items():assert hashlib.sha256((B/rel).read_bytes()).hexdigest()==h,rel
shutil.copytree(B,P,ignore=shutil.ignore_patterns('Objects','Listings'))
U=P/'CHASSIS/USER'
core=r'''/* Node6 diagnosis only: no enables, reference edits or motion/menu writes.
 * All calls are made by the existing sole CAN owner. Raw wire values retained. */
#ifndef EXTENSION_READONLY_DIAG_INCLUDED
#define EXTENSION_READONLY_DIAG_INCLUDED
#include <stdint.h>
#include <string.h>
#define EXT_DIAG_ITEMS 7U
#define EXT_TRACE_CAPACITY 128U
#define EXT_TRACE_WORDS 12U
volatile uint32_t g_ext_diag_request,g_ext_diag_state,g_ext_diag_cycle,g_ext_diag_index,g_ext_diag_rejected;
volatile uint32_t g_ext_diag_context; /* 0 idle/DONE snapshot,1 fault-after-STOP */
volatile uint32_t g_ext_diag_cmd[7]={0x1aU,0x33U,0x34U,0x35U,0x36U,0x37U,0x3aU};
volatile uint32_t g_ext_diag_result[7],g_ext_diag_tick[7],g_ext_diag_raw[7][8];
volatile uint32_t g_ext_move_ack_sequence,g_ext_move_ack_result,g_ext_move_ack_code,g_ext_move_ack_tick;
volatile uint32_t g_ext_move_frame[15];
volatile uint32_t g_ext_move_target_sequence,g_ext_move_target_result,g_ext_move_target_tick,g_ext_move_target_raw[8];
volatile uint32_t g_ext_trace_count,g_ext_trace_rows[128][12],g_ext_trace_speed_miss_count;
#if !APP_MOTOR_OFFLINE_ONLY
static uint32_t ext_target_pending,ext_target_pending_sequence;
static uint32_t ext_position_raw,ext_position_sign,ext_position_tick;
static uint32_t ExtDiag_RecordMove(uint32_t sequence,const uint8_t *frame,uint32_t result,uint32_t code)
{
 uint32_t i,accepted=(result==5U || (result==1U && (code==2U || code==0x9fU)));
 g_ext_move_ack_sequence=sequence;g_ext_move_ack_result=result;
 g_ext_move_ack_code=result==1U?code:0xffffffffU;g_ext_move_ack_tick=(uint32_t)xTaskGetTickCount();
 for(i=0;i<15U;i++)g_ext_move_frame[i]=frame[i];
 ext_target_pending=accepted;ext_target_pending_sequence=sequence;
 if(accepted){g_ext_move_target_sequence=sequence;g_ext_move_target_result=0U;g_ext_move_target_tick=0U;
  for(i=0;i<8U;i++)g_ext_move_target_raw[i]=0U;}
 return accepted;
}
static void ExtDiag_RecordPosition(uint32_t raw,uint32_t sign,uint32_t tick)
{ext_position_raw=raw;ext_position_sign=sign;ext_position_tick=tick;}
static void ExtDiag_RecordSpeed(uint32_t sequence,uint32_t flags,float goal,const uint8_t *frame,const uint8_t *reply)
{
 uint32_t row=g_ext_trace_count%EXT_TRACE_CAPACITY;union {float f;uint32_t u;} bits;
 bits.f=goal;
 g_ext_trace_rows[row][0]=(uint32_t)xTaskGetTickCount();g_ext_trace_rows[row][1]=sequence;
 g_ext_trace_rows[row][2]=ext_position_tick;g_ext_trace_rows[row][3]=ext_position_sign;
 g_ext_trace_rows[row][4]=ext_position_raw;g_ext_trace_rows[row][5]=reply[1];
 g_ext_trace_rows[row][6]=((uint32_t)reply[2]<<8)|reply[3];g_ext_trace_rows[row][7]=flags;
 g_ext_trace_rows[row][8]=bits.u;
 g_ext_trace_rows[row][9]=((uint32_t)frame[8]<<24)|((uint32_t)frame[9]<<16)|((uint32_t)frame[10]<<8)|frame[11];
 g_ext_trace_rows[row][10]=g_ext_move_ack_result;g_ext_trace_rows[row][11]=g_ext_move_ack_code;
 g_ext_trace_count++; /* publish after the entire bounded row */
}
static uint32_t ExtDiag_MoveTargetOnce(uint32_t sequence)
{
 const uint8_t q[2]={0x33U,0x6bU};CanRxMsg reply;uint32_t i,result;
 if(!ext_target_pending)return 0U;
 ext_target_pending=0U;
 if(ext_target_pending_sequence!=sequence)return 0U;
 memset(&reply,0,sizeof(reply));result=VehicleArm_Exchange(6U,q,2U,7U,&reply);
 g_ext_move_target_sequence=sequence;g_ext_move_target_result=result;
 g_ext_move_target_tick=(uint32_t)xTaskGetTickCount();
 for(i=0;i<8U;i++)g_ext_move_target_raw[i]=reply.Data[i];
 return 1U; /* one extra read after FD, before ordinary polling/STOP */
}
#endif
static void ExtDiag_Service(uint32_t safe,uint32_t context)
{
 uint32_t i,j,request=g_ext_diag_request;
 if(request){
  g_ext_diag_request=0U;g_ext_diag_cycle++;g_ext_diag_rejected=0U;
  if(request!=1U || g_ext_diag_state==1U){g_ext_diag_state=3U;g_ext_diag_rejected=2U;return;}
  g_ext_diag_context=context;g_ext_diag_index=0U;
  for(i=0;i<EXT_DIAG_ITEMS;i++){g_ext_diag_result[i]=0U;g_ext_diag_tick[i]=0U;for(j=0;j<8U;j++)g_ext_diag_raw[i][j]=0U;}
  g_ext_diag_state=1U;
 }
 if(g_ext_diag_state!=1U)return;
#if APP_MOTOR_OFFLINE_ONLY
 (void)safe;(void)context;g_ext_diag_state=3U;g_ext_diag_rejected=9U;
#else
 if(!safe){g_ext_diag_state=3U;g_ext_diag_rejected=1U;return;}
 i=g_ext_diag_index;
 if(i<EXT_DIAG_ITEMS){
  static const uint8_t commands[7]={0x1aU,0x33U,0x34U,0x35U,0x36U,0x37U,0x3aU};
  static const uint8_t lengths[7]={4U,7U,7U,5U,7U,7U,3U};
  uint8_t q[2]={commands[i],0x6bU};CanRxMsg reply;
  memset(&reply,0,sizeof(reply));
  g_ext_diag_result[i]=VehicleArm_Exchange(6U,q,2U,lengths[i],&reply);
  g_ext_diag_tick[i]=(uint32_t)xTaskGetTickCount();
  for(j=0;j<8U;j++)g_ext_diag_raw[i][j]=reply.Data[j];
  g_ext_diag_index=i+1U;
 }
 if(g_ext_diag_index==EXT_DIAG_ITEMS)g_ext_diag_state=2U;
#endif
}
#endif
'''
(U/'extension_readonly_diag.inc').write_bytes(core.replace('\n','\r\n').encode('utf-8'))
allow=r'''#ifndef EXTENSION_DIAG_READ_H
#define EXTENSION_DIAG_READ_H
#include <stdint.h>
static uint32_t ExtensionDiag_ReadAllowed(uint8_t address,const uint8_t *q,uint8_t length,uint8_t reply_length)
{return q && address==6U && length==2U && q[1]==0x6bU && reply_length==7U && (q[0]==0x34U || q[0]==0x37U);}
#endif
'''
(U/'extension_diag_read.h').write_bytes(allow.replace('\n','\r\n').encode('utf-8'))
def edit(rel,old,new):
 p=P/rel;b=p.read_bytes();old=old.encode('utf-8');assert b.count(old)==1,(rel,old[:90]);p.write_bytes(b.replace(old,new.encode('utf-8'),1))
v='CHASSIS/USER/vehicle_source_parallel.inc'
edit(v,'#include "chassis_auto_test.h"','#include "chassis_auto_test.h"\r\n#include "extension_readonly_diag.inc"')
edit(v,' a->position_tick=(uint32_t)xTaskGetTickCount();a->position_cycle++;',
 ' a->position_tick=(uint32_t)xTaskGetTickCount();a->position_cycle++;\r\n if(address==6U)ExtDiag_RecordPosition(raw,s_rx.Data[1],a->position_tick);')
edit(v,'  if(address==6U)return (r==1U || r==5U)?1U:0U;',
 '  if(address==6U)return ExtDiag_RecordMove(vsp_group.command.sequence,a->move,r,s_rx.Data[1]);')
edit(v,' zero_return=(address==6U && vsp_group.command.goal.extension_mm==0.0f);',
 ' zero_return=(address==6U && vsp_group.command.goal.extension_mm==0.0f);\r\n if(address==6U && ExtDiag_MoveTargetOnce(vsp_group.command.sequence))return 0U;')
edit(v,' if(!VSP_Query(address,0x35,5) || (zero_return && s_rx.Data[1]>1U)){',
 ' {uint32_t speed_ok=VSP_Query(address,0x35,5);\r\n if(address==6U){if(speed_ok)ExtDiag_RecordSpeed(vsp_group.command.sequence,a->flags,a->goal,a->move,s_rx.Data);else g_ext_trace_speed_miss_count++;}\r\n if(!speed_ok || (zero_return && s_rx.Data[1]>1U)){')
edit(v,' a->phase=0;return a->samples>=3;',' a->phase=0;return a->samples>=3;}')
can='CHASSIS/SYSTEM/CAN/can.c'
edit(can,'#include "lift_ruler_adapter.h"','#include "lift_ruler_adapter.h"\r\n#include "extension_diag_read.h"')
edit(can,'    if(length==2U && q[1]==0x6b &&',
 '    if(ExtensionDiag_ReadAllowed(address,q,length,reply_length))\r\n        return CAN1_ExchangeSingle(address,q,length,reply_length,reply);\r\n    if(length==2U && q[1]==0x6b &&')
it='CHASSIS/USER/integration10_complete.inc'
edit(it,'#include "usart.h"','#include "usart.h"\r\n#include "power.h"')
edit(it,'void Integration10_Service(void)',r'''static void it_extension_diag_service(void)
{
#if APP_MOTOR_OFFLINE_ONLY
 ExtDiag_Service(0U,0U);
#else
 PowerCommandStatus power;uint32_t safe;
 Power_ReadCommandStatus(&power);
 safe=power.active && it_platform() && (g_it_state==0U || g_it_state==2U || g_it_state==9U) &&
  g_it_auto_state!=1U && !g_it_request && !g_lift_ruler_request && !g_lift_ruler_stop_pending &&
  !g_vehicle_arm_stop_pending && !g_source_parallel_stop_mask && !VehicleArm_Busy() &&
  !VehicleSourceParallel_Owns() && !SourceStage2_IsActive() && !Stage2Fast_IsActive() &&
  (g_startup_axes_state==0U || g_startup_axes_state==4U) &&
  !g_dm_jog_request && !DM_CAN2_JogIsBusy();
 ExtDiag_Service(safe,g_it_state==9U?1U:0U);
#endif
}
void Integration10_Service(void)''')
edit(it,' if(g_it_state!=1U){it_auto_service((uint32_t)xTaskGetTickCount());return;}',
 ' if(g_it_state!=1U){it_auto_service((uint32_t)xTaskGetTickCount());it_extension_diag_service();return;}')
edit(it,' it_auto_service((uint32_t)xTaskGetTickCount());\r\n}',
 ' it_auto_service((uint32_t)xTaskGetTickCount());\r\n it_extension_diag_service();\r\n}')
prior_images=m['images'];m={k:v for k,v in m.items() if k not in ('field_runs','field_review','field_verified_profiles','latest_run_complete','mcu_three_complete')}
m.update(base='tmp/pickup_single_speed_20261110/project',images={},revision='second_gear_readonly_diagnostics',hardware_access=False,deployed=False,
 actual_tests_run=0,field_verified=False,parent_deployed_sha256=prior_images['Vehicle_LiftRuler']['sha256'],
 motion_change='No trajectories/speeds/thresholds or50ms changes. Strict node6 positive ACK; one0x33 afterFD; existing35/36 raw ring; gated idle/fault-stop readonly batch.',
 trace_row_words=12,trace_rows=128,diag_requests=[0x1a,0x33,0x34,0x35,0x36,0x37,0x3a])
m['changed']={}
for p in P.rglob('*'):
 if p.is_file():
  rel=p.relative_to(P);base=R/'handoff/pzq/project'/rel
  if not base.exists() or p.read_bytes()!=base.read_bytes():m['changed'][rel.as_posix()]=hashlib.sha256(p.read_bytes()).hexdigest()
(E/'candidate.json').write_text(json.dumps(m,ensure_ascii=False,indent=2)+'\n',encoding='utf-8')
for name in ('timing-model.json','integration13-three-direct-matlab-20261109.json'):shutil.copy2(PREV/name,E/name)
for name in ('build_pickup_single_speed_20261110.py','check_pickup_single_speed_20261110.py'):
 s=(R/'scripts'/name).read_text(encoding='utf-8').replace('pickup_single_speed_20261110','pickup_second_diag_20261110').replace('2026-11-10/single-speed','2026-11-10/second-diag')
 (R/'scripts'/name.replace('pickup_single_speed','pickup_second_diag')).write_text(s,encoding='utf-8')
print('Prepared isolated diagnostics candidate. No probe/serial/hardware access; original single-speed/pzq unchanged.')
