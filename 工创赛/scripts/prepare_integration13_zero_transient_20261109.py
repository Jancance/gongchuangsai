"""Bound one demonstrated small zero-return transient, retaining hard stops."""
from pathlib import Path
import hashlib,json
from prepare_stage2_fast_candidate_20261109 import replace
R=Path(__file__).resolve().parents[1];E=R/'evidence/integration/2026-11-07/stage2-mainline'
P=R/'tmp/stage2_sub20_20261109/project/CHASSIS/USER'
assert json.loads((E/'integration13-zero-transient-matlab-20261109.json').read_text())['passed']
p=P/'vehicle_source_parallel.inc';d=p.read_bytes()
d=replace(d,'position_tick,position_cycle;float before,goal,current;',
 'position_tick,position_cycle,overshoot_active,overshoot_ms;float before,goal,current;')
helper='''static uint32_t VSP_PositionWindow(VSP_Axis *a,uint32_t zero_return)
{
 float lo=fminf(a->before,a->goal),hi=fmaxf(a->before,a->goal);
 uint32_t now=(uint32_t)xTaskGetTickCount();
 if(a->current>=lo-60.0f && a->current<=hi+60.0f){a->overshoot_active=0U;return 1U;}
 /* Only a demonstrated zero-return stop transient gets a bounded settling
  * interval. Severe excursion and every other axis/goal keep the old exit. */
 if(!zero_return || a->current<lo-90.0f || a->current>hi+90.0f)return 0U;
 if(!a->overshoot_active){a->overshoot_active=1U;a->overshoot_ms=now;return 1U;}
 return (uint32_t)(now-a->overshoot_ms)<150U;
}
'''
d=replace(d,'static uint32_t VSP_Start(void *context,uint32_t bit,const SPF_Command *c)',
 helper.replace('\n','\r\n')+'static uint32_t VSP_Start(void *context,uint32_t bit,const SPF_Command *c)')
d=replace(d,'  if(a->current<fminf(a->before,a->goal)-60.0f || a->current>fmaxf(a->before,a->goal)+60.0f)return 2;',
 '  if(!VSP_PositionWindow(a,zero_return))return 2;')
p.write_bytes(d)
f=E/'stage2-fast-candidate-20261109.json';m=json.loads(f.read_text(encoding='utf-8'))
m.setdefault('previous_deployed_images',[]).append({'images':m['images'],'flash_log':m.get('flash_log'),
 'failed_run':'evidence/integration/2026-11-07/stage2-mainline/integration13-three-immediate-once-20261109-212842.json'})
m['changed'][p.name]=hashlib.sha256(d).hexdigest();m['images']={};m['deployed']=False;m['hardware_motion']=False
m['zero_return_transient']={'soft_motor_deg':60,'hard_motor_deg':90,'max_ms':150,
 'completion_window_motor_deg':2,'field_verified':False}
f.write_text(json.dumps(m,indent=2)+'\n',encoding='utf-8')
print('Prepared bounded150ms zero-return transient handling; other motion boundaries retained.')
