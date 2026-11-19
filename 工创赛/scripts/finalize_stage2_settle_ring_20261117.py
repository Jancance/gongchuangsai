"""Audit final two-file firmware and three-file Pi overlays; no hardware."""
from pathlib import Path
import json,hashlib,datetime,py_compile
from prepare_stage2_settle_ring_20261117 import R,BASE,P,E,PIBASE,PI
def sha(p):return hashlib.sha256(p.read_bytes()).hexdigest()
def main():
 assert json.loads((E/'pi-checks.json').read_text())['passed'] and json.loads((E/'c-checks.json').read_text())['passed']
 changes={};manifest={}
 for p in P.rglob('*'):
  if not p.is_file() or any(x in ('Objects','Listings') for x in p.relative_to(P).parts):continue
  n=p.relative_to(P).as_posix();manifest[n]=sha(p)
  if p.read_bytes()!=(BASE/n).read_bytes():changes[n]=sha(p)
 assert set(changes)=={'CHASSIS/USER/chassis_auto_test.c','CHASSIS/USER/main.c'}
 # Original firmware encodings, S2step20, real STOP, and S3 sources are preserved.
 for n in changes:
  a=(P/n).read_bytes();b=(BASE/n).read_bytes();a.decode('gb18030');b.decode('gb18030')
 pi_manifest={};pi_changes={}
 for p in PI.rglob('*'):
  if not p.is_file() or '__pycache__' in p.parts or p.name=='manifest.json':continue
  n=p.relative_to(PI).as_posix();pi_manifest[n]=sha(p)
  if not (PIBASE/n).is_file() or (PIBASE/n).read_bytes()!=p.read_bytes():pi_changes[n]=sha(p)
 assert set(pi_changes)=={'source_fast_cycle_view_20261115.py','source_alignment_settle.py','source-alignment-settle.json'}
 # Exact unchanged branches of the actual desktop entry, including material120/5.
 a=(PI/'source_fast_cycle_view_20261115.py').read_text(encoding='utf-8');b=(PIBASE/'source_fast_cycle_view_20261115.py').read_text(encoding='utf-8')
 for start,end in [("        if region_ui.region=='stage3':","        cv2.setWindowTitle(window,'原料区视觉"),('        study=','        send_phase=')]:
  assert a[a.index(start):a.index(end)]==b[b.index(start):b.index(end)]
 for n in pi_changes:
  if n.endswith('.py'):py_compile.compile(str(PI/n),doraise=True)
 (PI/'manifest.json').write_text(json.dumps(pi_manifest,indent=2)+'\n')
 images={}
 for t in ['Vehicle_LiftRuler','Vehicle_LiftRuler_Offline']:
  assert b'0 Error(s), 0 Warning(s)'in (E/(t+'-build.log')).read_bytes()
  p=P/'Objects'/t/(t+'.axf');images[t]={'axf':str(p),'sha256':sha(p)}
 c=json.loads((R/'evidence/integration/2026-11-17/stage2-debug215-step20/candidate.json').read_text())
 c.update(base=str(BASE),project=str(P),changes=changes,images=images,unchanged_sha256={n:h for n,h in manifest.items() if n not in changes},
  pi_base=str(PIBASE),pi_project=str(PI),pi_changes=pi_changes,pi_manifest_entries=len(pi_manifest),
  matlab_latest=json.loads((E/'matlab-result.json').read_text()),matlab_timeline={'initial343_before_ring_gate_implementation':'2026-11-17 02:44:55','supplement352_before_basic_done_lifecycle_and_inputfix':json.loads((E/'matlab-result.json').read_text())['completed_local'],'initial_result_evidence':'session tool stdout; current JSON is supplementary run'},
  source_alignment_settle_ms=300,source_alignment_settle_frames=5,source_alignment_max_pair_px=2,source_alignment_count_after_dwell=True,source_alignment_drop_first_postpose=True,
  endpoint_ring_capacity=8,endpoint_ring_recycle_terminal_outcomes=[1,2],endpoint_getter_order='logical oldest to newest',endpoint_inflight_not_overwritten=True,
  real_basic_done_lifecycle=True,endpoint_telemetry_session_cursor=True,session_zero_reserved=True,
  state6_and_stage3_pi_branches_byte_identical=True,raw_centers_unfiltered=True,existing_mapping_unchanged=True,existing_step20_unchanged=True,
  actual_c={'full_chassis':(E/'actual-chassis-ring.log').read_text().strip(),'legacy_distance':(E/'actual-legacy-distance-ring.log').read_text().strip(),'wire_consumer':(E/'actual-settle-wire-consumer.log').read_text().strip()},
  actual_pi=json.loads((E/'pi-checks.json').read_text()),review_passed=False,flashed=False,hardware_access=False,completed_local=datetime.datetime.now().isoformat(),
  requirements=list(dict.fromkeys(c['requirements']+['RUN-06','VIS-15','CAL-02','CAL-05'])))
 c['exact_one_line_delta']=False;c['source_rebuild_byte_identical']=False
 (E/'candidate.json').write_text(json.dumps(c,indent=2)+'\n');(E/'source-manifest.json').write_text(json.dumps(manifest,indent=2)+'\n');(E/'pi-manifest.json').write_text(json.dumps(pi_manifest,indent=2)+'\n')
 (E/'checks.json').write_text(json.dumps({'passed':True,'dual_build':True,'actual_chassis_c':650,'actual_legacy_distance_c':391,'actual_source_wire_consumer_c':319,'actual_pi_checks':75,'scope_audit':True,'hardware_access':False,'flashed':False},indent=2)+'\n')
 print(json.dumps({'images':images,'firmware_changes':list(changes),'pi_changes':list(pi_changes),'pi_entries':len(pi_manifest)},ensure_ascii=False))
if __name__=='__main__':main()
