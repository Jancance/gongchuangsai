"""Bridge a real completed approach into the existing lift reference owner."""
import hashlib,json
from pathlib import Path
from prepare_stage2_fast_candidate_20261109 import replace
R=Path(__file__).resolve().parents[1];E=R/'evidence/integration/2026-11-07/stage2-mainline'
P=R/'tmp/stage2_sub20_20261109/project/CHASSIS/USER'
assert json.loads((E/'stage2-fast-gates-matlab-20261109.json').read_text())['gate_cases']==199
p=P/'integration10_complete.inc';d=p.read_bytes()
helper='''static void it_empty_overlap_publish(void)
{
#if !APP_MOTOR_OFFLINE_ONLY
 /* Actual settled position from this completed EXT|LIFT group. Preserve top
  * reference, invalidate at-top backoff, and let existing return own reuse. */
 g_lift_ruler_controller.position_after_raw=(int64_t)(vsp_axis[1].current*
  (float)g_lift_ruler_controller.top_scale+0.5f);
 g_lift_ruler_controller.backoff_valid=0U;g_lift_ruler_controller.operation=2U;
 g_lift_ruler_controller.samples++;
#endif
}
'''
d=replace(d,'/* Left single-piece loaded commissioning:',helper.replace('\n','\r\n')+'/* Left single-piece loaded commissioning:')
d=replace(d,'   (g_source_parallel_done_mask&6U)==6U)it_done();break;\r\n',
          '   (g_source_parallel_done_mask&6U)==6U){it_empty_overlap_publish();it_done();}break;\r\n')
p.write_bytes(d)
f=E/'stage2-fast-candidate-20261109.json';m=json.loads(f.read_text())
m.setdefault('previous_deployed_images',[]).append({'images':m['images'],'flash_log':m.get('flash_log')})
m['changed'][p.name]=hashlib.sha256(d).hexdigest();m['images']={};m['deployed']=False;m['hardware_motion']=False
m['pick_bridge']='actual settled EXT|LIFT group publishes pose; no duplicate motion'
f.write_text(json.dumps(m,indent=2,ensure_ascii=False)+'\n')
print('Prepared actual-position bridge; not flashed.')
