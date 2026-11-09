"""Refine staged source transition; actual install remains separate and hash checked."""
from pathlib import Path
import json,hashlib
R=Path(__file__).resolve().parents[1];E=R/'evidence/integration/2026-11-04/source-parallel'
m=json.loads((E/'manifest.json').read_text(encoding='utf-8'));S=Path(m['stage']);A=Path(m['actual'])
sha=lambda p:hashlib.sha256(p.read_bytes()).hexdigest()
assert m['installed'] and not m['flashed']
m['installed_sha256']={n:sha(A/n) for n in m['changed']}
rel='CHASSIS/USER/startup_group.inc';b=(A/rel).read_bytes();m['original'][rel]=sha(A/rel);m['installed_sha256'][rel]=sha(A/rel)
q=E/'before'/rel;q.parent.mkdir(parents=True,exist_ok=True);q.write_bytes(b)
def edit(rel,fn):
 p=S/rel;b=p.read_bytes()
 try:u=b.decode('utf-8');enc='utf-8'
 except UnicodeDecodeError:u=b.decode('gbk');enc='gbk'
 p.write_bytes(fn(u.replace('\r\n','\n')).replace('\n','\r\n').encode(enc))
edit('HARDWARE/DM_CAN2/dm_can2.h',lambda u:u.replace('extern volatile uint32_t g_dm_source_smooth,',
    'uint32_t DM_CAN2_SourceTransitionPending(void);\nextern volatile uint32_t g_dm_source_smooth,'))
def motion(u):
 marker='static uint32_t s_point_mode = 0U;';assert u.count(marker)==1
 return u.replace(marker,marker+'''
/* 只接受源区准备的新目标排队、且已独立确认失能的正常过渡。 */
uint32_t DM_CAN2_SourceTransitionPending(void)
{
 return g_dm_source_smooth && !g_dm_source_go && g_dm_integrated_owner &&
    (s_queued_request==4U || s_queued_request==5U) &&
    g_dm_jog_state==8U && g_dm_jog_result==2U && g_dm_jog_stop_result==1U && !s_enable_possible;
}
''')
edit('HARDWARE/DM_CAN2/dm_can2_motion.c',motion)
def group(u):
 old='if(g_dm_jog_first_fault || g_dm_jog_state==8U)';assert u.count(old)==2
 return u.replace(old,'if(g_dm_jog_first_fault || (g_dm_jog_state==8U && !DM_CAN2_SourceTransitionPending()))')
edit(rel,group)
m['changed']={n:sha(S/n) for n in m['original']}
(E/'manifest.json').write_text(json.dumps(m,ensure_ascii=False,indent=2),encoding='utf-8');print('Staged precise queued-transition allowance.')
