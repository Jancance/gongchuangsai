"""Two authorized local deltas, preserving flashed baseline and byte encodings."""
from pathlib import Path
import json, shutil
R=Path(__file__).resolve().parents[1]
BASE=R/'tmp/stage3_offline_ready_20261116/project'
P=R/'tmp/stage3_parallel_preclose_20261116/project'
E=R/'evidence/integration/2026-11-16/stage3-parallel-preclose'
def edit(name,pairs):
 b=(P/name).read_bytes()
 for old,new in pairs:
  old=old.replace('\n','\r\n').encode('ascii');new=new.replace('\n','\r\n').encode('ascii')
  assert b.count(old)==1,(name,old,b.count(old));b=b.replace(old,new)
 (P/name).write_bytes(b)
def main():
 proof=json.loads((E/'matlab-result.json').read_text());assert proof['passed'] and proof['checks']==961
 assert not P.exists(),'Preserve existing candidate'
 shutil.copytree(BASE,P,ignore=shutil.ignore_patterns('Objects','Listings'))
 edit('CHASSIS/USER/stage3_cycle.inc',[
 ('static uint32_t s3_pending,s3_wait_tick', 'static uint32_t s3_preclose_started,s3_preclose_tick;\nstatic uint32_t s3_pending,s3_wait_tick'),
 ('static void S3_Pose(void)', '''/* Called only after IT3 reports its real DM start; 600ms is the existing settle. */
static uint32_t S3_PrecloseOnDmStarted(void)
{
 if(!Stage3_Owns() || g_stage3_state!=5U || g_stage3_phase!=3U ||
    s3_pending!=3U || s3_preclose_started)return 1U;
 if(jiazhua(160.0f)!=1U)return 0U;
 s3_preclose_started=1U;s3_preclose_tick=(uint32_t)xTaskGetTickCount();return 1U;
}
static void S3_Pose(void)'''),
 ('case 3U:S3_Axis(SPF_DM,s3_carrier);break;','case 3U:s3_preclose_started=0U;s3_preclose_tick=0U;S3_Axis(SPF_DM,s3_carrier);break;'),
 ('case 4U:S3_Command(17U);break; /* Stage3 preclose before pickup descent. */','case 4U:if(s3_preclose_started && (uint32_t)(now-s3_preclose_tick)>=600U)g_stage3_phase++;break; /* DM done plus original settle. */')])
 edit('CHASSIS/USER/integration10_complete.inc',[
 ('  if(g_source_parallel_state==4U){it_fail(8U);return;}\n  if(g_source_parallel_state==3U){',
  '  if(g_source_parallel_state==4U){it_fail(8U);return;}\n  if((it_command.axes_mask&SPF_DM) && (g_source_parallel_started_mask&SPF_DM) &&\n     !S3_PrecloseOnDmStarted()){it_fail(7U);return;}\n  if(g_source_parallel_state==3U){')])
 edit('CHASSIS/USER/lift_ruler_calibration.c',[
 ('units=c->operation==1U?c->down_target_pulses:RETURN_TOP_MARGIN_PULSES;',
  'units=c->operation==1U?c->down_target_pulses:\n    (c->return_limit_pulses==SPF_STAGE3_MAX_LIFT_PULSES?SOURCE_HOME_BACKOFF_PULSES:RETURN_TOP_MARGIN_PULSES);'),
 ('every 5th return.','every 10th return.')])
 print('Prepared independent candidate; MATLAB precedes three source changes; no hardware.')
if __name__=='__main__':main()
