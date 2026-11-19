"""Freeze only owner + lead20 parameter; inherit verified a649 unchanged paths."""
from pathlib import Path
import hashlib,json,sys,datetime
from prepare_three_frame_grip_lead20_20261118 import R,BASE,P,E
def sha(p):return hashlib.sha256(p.read_bytes()).hexdigest()
def main():
 old=R/'evidence/integration/2026-11-18/three-frame-initial5'
 base=json.loads((old/'candidate.json').read_text());prior=json.loads((old/'source-manifest.json').read_text())
 assert base['deployment']['physical_acceptance']
 assert all(sha(BASE/n)==v for n,v in prior.items())
 assert all(sha(Path(m['axf']))==m['sha256'] for m in base['images'].values())
 manifest={};changes={};encoding={}
 for p in P.rglob('*'):
  if not p.is_file() or any(s in ('Objects','Listings') for s in p.relative_to(P).parts):continue
  n=p.relative_to(P).as_posix();a=p.read_bytes();b=(BASE/n).read_bytes();manifest[n]=sha(p)
  if a==b:continue
  changes[n]=manifest[n]
  if n=='USER/vehicle_parameters.h':
   line=('#define SOURCE_GRIP_LEAD_MM 20.0f /* 用户2026-11-18：剩20mm发140度夹紧，继续到原抓取高度即升，无新增停顿。 */\r\n').encode('utf-8')
   assert a.replace(line,b'',1)==b
  else:assert bytes(x for x in a if x>=128)==bytes(x for x in b if x>=128)
  assert a.count(b'\n')-a.count(b'\r\n')==b.count(b'\n')-b.count(b'\r\n')
  encoding[n]=dict(existing_nonascii_preserved=True,parameter_single_line_addition=n.endswith('vehicle_parameters.h'),bare_lf_preserved=True,old_crlf=b.count(b'\r\n'),new_crlf=a.count(b'\r\n'))
 assert set(changes)=={'CHASSIS/USER/pzq_commit_overlap.inc','USER/vehicle_parameters.h'}
 sys.path[:0]=[str(R/'tmp/gripper_debug_deps'),str(R/'src')]
 from elftools.elf.elffile import ELFFile
 from smart_carrier.vision_manipulation.mirror_committed_pick_board import flash_segments_digest
 images={}
 for t in ('Vehicle_LiftRuler','Vehicle_LiftRuler_Offline'):
  assert b'0 Error(s), 0 Warning(s)' in (E/(t+'-build.log')).read_bytes()
  p=P/'Objects'/t/(t+'.axf')
  with p.open('rb') as f:
   elf=ELFFile(f);ss=[(int(s['p_paddr']),s.data()) for s in elf.iter_segments() if s['p_type']=='PT_LOAD' and s['p_filesz'] and 0x08000000<=s['p_paddr']<0x08100000]
  images[t]=dict(axf=str(p),sha256=sha(p),flash_segments_sha256=flash_segments_digest(ss),flash_bytes=sum(len(d) for _,d in ss))
 c=dict(project=str(P),base=str(BASE),base_online_sha256=base['images']['Vehicle_LiftRuler']['sha256'],base_single_three_piece_user_accepted=True,
  user_choice_original='继续最后20mm，到位立即升（推荐：保持原抓取高度，无额外停顿）',images=images,changes=changes,source_files=len(manifest),
  extend_sign=1,home_direction=1,parameter_save=0,home_diag_version=1,home_after_write_probe_version=1,
  grip_lead_mm=20,grip_angle_deg=140,grip_feedback_freshness_ms=150,side_early_threshold_pulses=17700,bottom_early_threshold_pulses=21700,
  side_final_pick_pulses=25700,bottom_final_pick_pulses=29700,grip_dwell_added_ms=0,grip_pwm_applied_wait_added=False,
  rise_trigger='original complete grab-pose group DONE immediately submits original rise/retract; no owner bypass',
  sparse_feedback_fallback='if no early close, issue140 then submit rise in same DONE call; no physical closure claim',
  original_a649_sources_and_images_unchanged=True,unrelated_source_count=len(manifest)-2,
  pi_deployment_needed=False,pi_overlay=base['pi_overlay'],pi_changes=base['pi_changes'],required_stopped_frames=3,first_pick_requires_observed120=True,later_picks_require_rotation=False,initial_dm_start_min_deg=-5,later_dm_start_min_deg=-2,
  matlab_checks=7756,actual_complete_owner_checks=1227,build_passed=True,self_review_passed=True,review_passed=False,
  hardware_access=False,flashed=False,physical_acceptance=False,actual_project_replaced=False,
  current_hardware='a649 complete hold, three tray slots full; no reset/flash/motion by coding agent; new clear-tray/support confirmation required before any future main deployment',
  requirements_R=base['requirements_R'],requirements_E=base['requirements_E'],requirements_Q='Official pending technical questions unchanged; lead20 real pickup/swept clearance/retention and actual closure timing unverified',completed_local=datetime.datetime.now().isoformat())
 checks=dict(passed=True,matlab=json.loads((E/'matlab-result.json').read_text()),actual_c=json.loads((E/'actual-c-result.json').read_text()),dual_build_clean=True,exact_two_file_delta=True,original_a649_unchanged=True,hardware_access=False,flashed=False)
 for n,d in [('candidate.json',c),('checks.json',checks),('source-manifest.json',manifest),('encoding-line-endings.json',encoding)]:
  (E/n).write_text(json.dumps(d,indent=2)+'\n',encoding='utf-8')
 print(json.dumps(dict(images=images,source_files=len(manifest),changes=list(changes)),ensure_ascii=True))
if __name__=='__main__':main()
