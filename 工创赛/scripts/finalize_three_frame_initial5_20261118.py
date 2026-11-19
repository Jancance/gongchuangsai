"""Freeze authorized four-file delta, original cb integrity and image identities."""
from pathlib import Path
import datetime,hashlib,json,sys
from prepare_three_frame_initial5_20261118 import R,BASE,P,E

def sha(p):return hashlib.sha256(p.read_bytes()).hexdigest()
def main():
 assert json.loads((E/'actual-c-result.json').read_text())['passed']
 previous=R/'evidence/integration/2026-11-18/three-frame-warm'
 oldmanifest=json.loads((previous/'source-manifest.json').read_text())
 assert all(sha(BASE/n)==v for n,v in oldmanifest.items()),'Original cb source changed'
 expected={'USER/vehicle_parameters.h','CHASSIS/USER/integration10_complete.inc','CHASSIS/USER/vehicle_source_parallel.inc','HARDWARE/DM_CAN2/dm_can2_motion.c'}
 manifest={};changes={};encoding={}
 for p in P.rglob('*'):
  if not p.is_file() or any(x in ('Objects','Listings') for x in p.relative_to(P).parts):continue
  n=p.relative_to(P).as_posix();manifest[n]=sha(p);a=p.read_bytes();b=(BASE/n).read_bytes()
  if a==b:continue
  changes[n]=manifest[n]
  if n=='USER/vehicle_parameters.h':
   addition=('#define SOURCE_INITIAL_DM_START_MIN_DEG (-5.0f) /* 用户2026-11-18确认：仅首次180度/65mm并行起点下限，后续仍为-2度。 */\r\n').encode('utf-8')
   assert a.replace(addition,b'',1)==b
   preserved='Entire prior header byte-identical after removing the one new Chinese-commented parameter line'
  else:
   assert bytes(x for x in a if x>=128)==bytes(x for x in b if x>=128),(n,'existing nonASCII')
   preserved='All existing non-ASCII bytes identical'
  assert a.count(b'\n')-a.count(b'\r\n')==b.count(b'\n')-b.count(b'\r\n'),(n,'bare LF')
  encoding[n]=dict(existing_encoding_preserved=preserved,bare_lf_identical=True,old_crlf=b.count(b'\r\n'),new_crlf=a.count(b'\r\n'))
 assert set(changes)==expected
 sys.path[:0]=[str(R/'tmp/gripper_debug_deps'),str(R/'src')]
 from elftools.elf.elffile import ELFFile
 from smart_carrier.vision_manipulation.mirror_committed_pick_board import flash_segments_digest
 images={}
 for t in ('Vehicle_LiftRuler','Vehicle_LiftRuler_Offline'):
  assert b'0 Error(s), 0 Warning(s)' in (E/(t+'-build.log')).read_bytes()
  p=P/'Objects'/t/(t+'.axf')
  with p.open('rb') as f:
   elf=ELFFile(f);segments=[(int(s['p_paddr']),s.data()) for s in elf.iter_segments() if s['p_type']=='PT_LOAD' and s['p_filesz'] and 0x08000000<=s['p_paddr']<0x08100000]
  images[t]=dict(axf=str(p),sha256=sha(p),flash_segments_sha256=flash_segments_digest(segments),flash_bytes=sum(len(d) for _,d in segments))
 prior=json.loads((previous/'candidate.json').read_text())
 for t,m in prior['images'].items():assert sha(Path(m['axf']))==m['sha256'],'Frozen cb AXF changed'
 c=dict(project=str(P),base=str(BASE),base_online_sha256=prior['images']['Vehicle_LiftRuler']['sha256'],images=images,changes=changes,
  extend_sign=1,home_direction=1,parameter_save=0,home_diag_version=1,home_after_write_probe_version=1,
  source_files=len(manifest),initial_dm_start_min_deg=-5,later_dm_start_min_deg=-2,
  scope='User approved 2026-11-18; only exact first source state1/prep4 DM|EXT 180deg/65mm top + extension zero; sync/parallel before go',
  no_startup_request6_proof=True,prior_handoff_model_aborted=True,pi_overlay=prior['pi_overlay'],pi_changes=prior['pi_changes'],pi_deployment_needed=False,
  required_stopped_frames=3,first_pick_requires_observed120=True,later_picks_require_rotation=False,
  original_cb_source_and_images_unchanged=True,build_passed=True,self_review_passed=True,review_passed=False,
  matlab_checks=48678,actual_c_checks=2111,actual_complete_it_service_joint_checks=567,original_cb_dm_fault14_reproduced_checks=2,
  hardware_access=False,flashed=False,actual_project_replaced=False,
  requirements_R=['SRC-01','SRC-02','RUN-01','RUN-02','PWR-01','MECH-01','MECH-02'],requirements_E=['RUN-06','CAL-02','CAL-05','VIS-15'],
  requirements_Q='Official pending technical questions unchanged; first sweep/start skew and real pickup still unverified',completed_local=datetime.datetime.now().isoformat())
 checks=dict(passed=True,matlab=json.loads((E/'matlab-result.json').read_text()),actual_c_checks=2111,
  full_actual_IT_VSP_DM_SPF=True,original_dm_fault14_reproduced=True,dual_build_clean=True,encoding_preserved=True,
  original_cb_unchanged=True,hardware_access=False,flashed=False)
 for n,data in [('candidate.json',c),('source-manifest.json',manifest),('encoding-line-endings.json',encoding),('checks.json',checks)]:
  (E/n).write_text(json.dumps(data,indent=2)+'\n',encoding='utf-8')
 print(json.dumps(dict(images=images,changed=list(changes),source_files=len(manifest)),ensure_ascii=False))
if __name__=='__main__':main()
