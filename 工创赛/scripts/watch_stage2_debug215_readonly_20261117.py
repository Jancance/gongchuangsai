"""Read-only wrapper of the existing PC13 watcher; --watch alone attaches."""
from pathlib import Path
import hashlib,json,sys
R=Path(__file__).resolve().parents[1]
E=R/'evidence/integration/2026-11-17/stage2-debug215'
FIELDS=('g_region_dispatch_area','g_source_parallel_state','g_source_parallel_cause',
 'g_source_parallel_started_mask','g_source_parallel_done_mask','g_source_parallel_stop_mask',
 'g_source_parallel_sync_tick','g_dm_stage2_sync','g_dm_parallel_group_active','g_dm_parallel_group_go',
 'g_low_group_count','g_low_fault','g_dm_jog_state','g_dm_jog_stop_result')
def manifest():
 c=json.loads((E/'candidate.json').read_text());assert c['build_passed'] and c['review_passed']
 assert json.loads((E/'checks.json').read_text())['passed']
 for n,h in json.loads((E/'source-manifest.json').read_text()).items():
  assert hashlib.sha256((Path(c['project'])/n).read_bytes()).hexdigest()==h,n
 image=c['images']['Vehicle_LiftRuler'];assert hashlib.sha256(Path(image['axf']).read_bytes()).hexdigest()==image['sha256']
 return image
def main():
 source=(R/'scripts/source_mcu_cycle_watch_20261115.py').read_text(encoding='utf-8')
 source=source.replace("default=180)","default=45)")
 source=source.replace('args=p.parse_args();image=manifest()',"args=p.parse_args();assert 0<args.duration<=45,'Read-only watcher duration must be in (0,45] seconds';image=manifest()")
 old="state=board.snapshot();state.update({k:session.target.read32(symbols[k]) for k in keys})"
 assert source.count(old)==1
 source=source.replace(old,old+"\n                state.update({k:session.target.read32(symbols[k]) for k in EXTRA_FIELDS if k in symbols})\n                state['low_group_records']=board.motion_evidence()")
 scope={'__name__':'stage2_debug215_readonly_wrapper','__file__':str(R/'scripts/source_mcu_cycle_watch_20261115.py')}
 exec(compile(source,scope['__file__'],'exec'),scope)
 scope.update(E=E,manifest=manifest,EXTRA_FIELDS=FIELDS)
 scope['main']()
if __name__=='__main__':main()
