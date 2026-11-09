"""Permit bounded zero-speed settling before a staged axis starts."""
from pathlib import Path
import hashlib,json
from prepare_stage2_fast_candidate_20261109 import replace
R=Path(__file__).resolve().parents[1];E=R/'evidence/integration/2026-11-07/stage2-mainline'
P=R/'tmp/stage2_sub20_20261109/project/CHASSIS/USER'
assert json.loads((E/'integration13-prepare-wait-matlab-20261109.json').read_text())['passed']
p=P/'vehicle_source_parallel.inc';d=p.read_bytes()
d=replace(d,'static uint32_t vsp_fast_lift;\n',
 'static uint32_t vsp_fast_lift;\n'
 'volatile uint32_t g_vsp_prepare_miss,g_vsp_prepare_nonzero,g_vsp_prepare_speed_raw,g_vsp_prepare_wait_address;\n')
d=replace(d,'  if(!VSP_Query(address,0x35,5) || s_rx.Data[2] || s_rx.Data[3])return 2;\n'
 '  a->phase=3;return 0;\n',
 '  if(!VSP_Query(address,0x35,5)){\n'
 '   g_vsp_prepare_miss++;g_vsp_prepare_wait_address=address;a->cycle=1U;return 0;\n'
 '  }\n'
 '  if(s_rx.Data[1]>1U)return 2;\n'
 '  g_vsp_prepare_speed_raw=((uint32_t)s_rx.Data[2]<<8)|s_rx.Data[3];\n'
 '  if(g_vsp_prepare_speed_raw){\n'
 '   g_vsp_prepare_nonzero++;g_vsp_prepare_wait_address=address;a->cycle=1U;return 0;\n'
 '  }\n'
 '  /* A wait may have changed the relative-move origin. Re-read it once,\n'
 '   * rebuild the same physical goal, then require zero speed again. */\n'
 '  if(a->cycle){a->cycle=0U;a->phase=1U;return 0;}\n'
 '  a->phase=3;return 0;\n')
p.write_bytes(d)
f=E/'stage2-fast-candidate-20261109.json';m=json.loads(f.read_text())
m.setdefault('previous_deployed_images',[]).append({'images':m['images'],'flash_log':m.get('flash_log')})
m['changed'][p.name]=hashlib.sha256(d).hexdigest();m['images']={};m['deployed']=False;m['hardware_motion']=False
m['prepare_wait']='single missed/nonzero speed sample waits within original timeout, position refreshed'
f.write_text(json.dumps(m,indent=2,ensure_ascii=False)+'\n')
print('Prepared bounded fresh-zero-speed settling; no fixed wait and no hardware access.')
