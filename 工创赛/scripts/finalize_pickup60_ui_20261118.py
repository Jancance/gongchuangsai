"""Freeze four-code overlay and paired unchanged MCU; no remote/hardware work."""
from pathlib import Path
import hashlib,json,py_compile,datetime
from prepare_pickup60_ui_20261118 import R,P,DEPS,BASE,E
def sha(p):return hashlib.sha256(p.read_bytes()).hexdigest()
def main():
 tests=json.loads((E/'python-checks.json').read_text());assert tests['passed'] and tests['tests']==11
 warm=json.loads((R/'evidence/integration/2026-11-18/three-frame-warm/candidate.json').read_text())
 files={p.name:sha(p) for p in P.glob('*.py')};assert set(files)==set(warm['pi_changes'])|{'initial_rotation.py'}
 for p in P.glob('*.py'):py_compile.compile(str(p),doraise=True)
 bases=dict(warm['pi_changes']);bases['initial_rotation.py']=warm['pi_unchanged_dependencies']['initial_rotation.py']
 deps={k:v for k,v in warm['pi_unchanged_dependencies'].items() if k!='initial_rotation.py'}
 deps['boot_ready_handshake.py']='7f2346069b87bbc01035a90de6285108af3db77b000867df3bdcdf3a478c4b20'
 mcuE=R/'evidence/integration/2026-11-18/three-frame-grip-lead20';mc=json.loads((mcuE/'candidate.json').read_text());manifest=json.loads((mcuE/'source-manifest.json').read_text())
 assert all(sha(Path(mc['project'])/n)==v for n,v in manifest.items())
 assert sha(Path(mc['images']['Vehicle_LiftRuler']['axf']))==mc['images']['Vehicle_LiftRuler']['sha256']
 firmware=dict(mc['images']['Vehicle_LiftRuler']);firmware.update(manifest=str(mcuE/'source-manifest.json'),candidate=str(mcuE/'candidate.json'),source_hashes_frozen=True)
 c=dict(project=str(P),pi_changes=files,pi_base_hashes=bases,pi_unchanged_dependencies=deps,first_pick_rotation_deg=60,legacy_initial_rotation_default_deg=120,later_picks_require_rotation=False,
  required_stopped_frames=3,near_stop_speed_px_s=25,angle_45_fallback_authorized=True,angle_45_fallback_condition='Only after main observes first-item60 gate misses a stopped window; no automatic downgrade; do not misattribute ROI/frame/UART failures',active_release_angle_deg=60,
  warm_during_stationary_alignment=True,blind_gap_angle_inference=False,first_alignment_beep_release_unchanged=True,actual_ui_policy='actual_window_trigger_v1',legacy_detector_status_rendered=False,HSV_ROI_rules_unchanged=True,all_existing_json_and_launchers_preserved=True,
  paired_firmware=firmware,matlab=json.loads((E/'matlab-result.json').read_text()),python_tests=tests,renderer_qa=dict(actual_renderer=True,dimensions=[960,400],screenshots=[str(p) for p in E.glob('ui-*.png')],inspected=True,no_clipping=True),
  build_passed=True,self_review_passed=True,review_passed=False,hardware_access=False,deployed=False,physical_acceptance=False,cold_boot_tested=False,
  requirements_R=['SRC-02','RUN-01','RUN-02','PWR-01','MECH-01','MECH-02'],requirements_E=['RUN-06','VIS-15','CAL-02','CAL-05'],requirements_Q='Official questions unchanged; SRC02 physical120-degree layout unchanged;60 is user engineering gate',completed_local=datetime.datetime.now().isoformat())
 (E/'candidate.json').write_text(json.dumps(c,indent=2)+'\n',encoding='utf-8');(P/'manifest-delta.json').write_text(json.dumps(dict(files=files,base_hashes=bases,unchanged_dependencies=deps,preserve_all_runtime_configs_and_launchers=True),indent=2)+'\n',encoding='utf-8')
 print(json.dumps(dict(files=files,paired_firmware=firmware['sha256'],review_passed=False),ensure_ascii=True))
if __name__=='__main__':main()
