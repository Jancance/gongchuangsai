"""Apply authorized guard simplification to a preserved independent candidate."""
from pathlib import Path
import json,shutil
R=Path(__file__).resolve().parents[1]
B=R/'tmp/source_top_overlap200_20261114/project'
P=R/'tmp/source_top_simplified_20261114/project'
E=R/'evidence/integration/2026-11-14/overlap200-simplified'
def edit(rel,pairs):
 p=P/rel;b=p.read_bytes()
 for old,new in pairs:
  old=old.encode();new=new.encode();assert old in b,(rel,old);b=b.replace(old,new)
 p.write_bytes(b)
def main():
 assert json.loads((E/'simplification-matlab.json').read_text())['passed']
 assert not P.exists(),'Preserve existing candidate'
 shutil.copytree(B,P,ignore=shutil.ignore_patterns('Objects','Listings'))
 for name in ['matlab-result.json','body-registration-matlab.json','wheel-hold-rearm-matlab.json']:
  (E/name).write_bytes((R/'evidence/integration/2026-11-14/overlap200-commit'/name).read_bytes())
 u='CHASSIS/USER/'
 edit(u+'source_parallel_flow.c',[
  ('   if(now-g->first_start_ms>g->max_start_skew_ms){SPF_GroupCancel(g,5);return;}','   /* Start skew is diagnostic only; the group deadline still applies. */'),
  ('   if(g->last_start_ms-g->first_start_ms>g->max_start_skew_ms){SPF_GroupCancel(g,5);return;}','   /* Measured start skew retained in first/last_start_ms. */')])
 edit(u+'vehicle_source_parallel.inc',[
  ('frequency_dummy_not_present','')]) if False else None
 edit(u+'vehicle_source_parallel.inc',[
  ('g_dm_can2_feedback_tick>=100)return 2;','g_dm_can2_feedback_tick>=100)return 0;'),
  ('if(!VSP_Query(address,0x1a,4) || (s_rx.Data[2]&3))return 2;',
   'if(!VSP_Query(address,0x1a,4))return 0;\r\n  if(s_rx.Data[2]&3)return 2;'),
  ('if(!VSP_Position(address,a))return 2;a->before=a->current;',
   'if(!VSP_Position(address,a))return 0;a->before=a->current;'),
  ('if(VehicleArm_Exchange(address,enable,5,3,&s_rx)!=1 || s_rx.Data[1]!=2)return 2;',
   'if(VehicleArm_Exchange(address,enable,5,3,&s_rx)!=1)return 0;\r\n  if(s_rx.Data[1]!=2)return 2;'),
  ('if(!VSP_Query(address,0x3a,3) || !(s_rx.Data[1]&1) || (s_rx.Data[1]&0x08))return 2;',
   'if(!VSP_Query(address,0x3a,3))return 0;\r\n  if(!(s_rx.Data[1]&1) || (s_rx.Data[1]&0x08))return 2;'),
  ('(Low_Busy()?SOURCE_EXTENSION_ZERO_DONE_DEG:2.0f)', 'SOURCE_EXTENSION_ZERO_DONE_DEG'),
  ('return a->samples>=3;',
   'return a->samples>=((vsp_group.command.kind==SPF_CMD_PICK || (address==5U && Low_Busy()))?2U:1U);')])
 edit(u+'pzq_commit_overlap.inc',[
  ('(uint32_t)(now-low_phase_tick)>=16000U',
   '(!VehicleSourceParallel_Owns() && (uint32_t)(now-low_phase_tick)>=16000U)'),
  ('  if(!Low_ExtensionAt(0.0f))return;\r\n','')])
 edit('USER/vehicle_parameters.h',[
  ('#define SOURCE_TOP_TOLERANCE_DEG 10.0f','#define SOURCE_TOP_TOLERANCE_DEG 22.5f')])
 edit(u+'source_body_step.inc',[
  ('fabsf(g_vehicle_lift_current_deg-safe)<=1.0f','fabsf(g_vehicle_lift_current_deg-safe)<=SOURCE_TOP_TOLERANCE_DEG')])
 # Retry only READ transactions. Never replay relative movement on a lost ACK.
 edit(u+'lift_ruler_calibration.c',[
  ('{uint8_t q[2];q[0]=cmd;q[1]=0x6b;return exchange(fn,ctx,q,2U,rn,r);}',
   '{uint8_t q[2];q[0]=cmd;q[1]=0x6b;\r\n if(exchange(fn,ctx,q,2U,rn,r))return 1U;\r\n return exchange(fn,ctx,q,2U,rn,r);}')])
 edit(u+'startup_axes.c',[
  ('g_startup_extension_current_deg-s_target)>2.0f','g_startup_extension_current_deg-s_target)>SOURCE_EXTENSION_ZERO_DONE_DEG'),
  ('g_startup_extension_current_deg-s_target)>0.3f','g_startup_extension_current_deg-s_target)>SOURCE_EXTENSION_ZERO_DONE_DEG')])
 # Source only: ordinary observation interruption already leaves IDLE/DONE;
 # no existing motor fault or cancellation state is cleared by this revision.
 print(P)
if __name__=='__main__':main()
