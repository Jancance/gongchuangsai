"""Final scope audit, inherited Stage3 regression, reproducibility, dual Keil."""
from pathlib import Path
import hashlib,json,subprocess,datetime
from prepare_stage2_sync_opt_20261117 import R,BASE,P,E
import prepare_stage2_sync_opt_20261117 as prep
import build_stage3_parallel_preclose_20261116 as stage3
import build_stage3_teach_mode_20261116 as common
def sha(p):return hashlib.sha256(p.read_bytes()).hexdigest()
def main():
 assert json.loads((E/'actual-c-result.json').read_text())['passed']
 stage3.P=P;common.P=P;common.E=E
 p=E/'actual-inherited-stage3-owner-it3.c';p.write_text(stage3.owner(),encoding='utf-8')
 inherited=common.run('actual-inherited-stage3-owner-it3',p,flags=['-DSPF_STAGE3_MAX_LIFT_PULSES=82000U'])
 changes={};manifest={};unchanged={}
 for p in P.rglob('*'):
  if not p.is_file() or any(x in ('Objects','Listings') for x in p.relative_to(P).parts):continue
  n=p.relative_to(P).as_posix();manifest[n]=sha(p)
  if p.read_bytes()!=(BASE/n).read_bytes():changes[n]=sha(p)
 assert set(changes)=={'CHASSIS/USER/pzq_commit_overlap.inc','CHASSIS/USER/vehicle_source_parallel.inc','HARDWARE/DM_CAN2/dm_can2_motion.c','USER/vehicle_parameters.h'}
 for n in ['CHASSIS/USER/stage3_cycle.inc','CHASSIS/USER/integration10_complete.inc','CHASSIS/USER/lift_ruler_calibration.c','CHASSIS/USER/lift_ruler_adapter.c','CHASSIS/USER/region_dispatch.inc','CHASSIS/USER/source_mcu_cycle.inc','CHASSIS/USER/source_parallel_flow.c','CHASSIS/USER/source_parallel_flow.h','CHASSIS/USER/main.c','CHASSIS/SYSTEM/CAN/can.c']:
  assert (P/n).read_bytes()==(BASE/n).read_bytes();unchanged[n]=sha(P/n)
 # Stepper packets/rates/acceleration/physical goals and polling remain byte exact.
 a=(P/'CHASSIS/USER/vehicle_source_parallel.inc').read_text(encoding='utf-8');b=(BASE/'CHASSIS/USER/vehicle_source_parallel.inc').read_text(encoding='utf-8')
 for begin,end in [(' address=bit==SPF_EXT?6:5;a=&vsp_axis[bit==SPF_EXT?0:1];\n switch','static uint32_t VSP_PositionWindow('),('static uint32_t VSP_Poll(', 'static uint32_t VSP_Stop(')]:
  assert a[a.index(begin):a.index(end)]==b[b.index(begin):b.index(end)]
 # Re-run preparation into a separate retained directory, compare exact source bytes.
 rebuild=R/'tmp/stage2_sync_opt_rebuild_20261117/project'
 if not rebuild.exists():
  prep.P=rebuild;prep.main()
 assert {p.relative_to(rebuild).as_posix():sha(p) for p in rebuild.rglob('*') if p.is_file()}==manifest
 images={}
 for target in ['Vehicle_LiftRuler','Vehicle_LiftRuler_Offline']:
  for folder in ['Objects','Listings']:(P/folder/target).mkdir(parents=True,exist_ok=True)
  log=E/(target+'-build.log');r=subprocess.run(['C:/Keil_v5/UV4/UV4.exe','-r',str(P/'gongxun-f407.uvprojx'),'-t',target,'-j0','-o',str(log)],timeout=240)
  data=log.read_bytes();print(data.decode('gb18030',errors='replace')[-500:],flush=True)
  assert r.returncode==0 and b'0 Error(s), 0 Warning(s)' in data
  axf=P/'Objects'/target/(target+'.axf');images[target]={'axf':str(axf),'sha256':sha(axf)}
 c=json.loads((R/'evidence/integration/2026-11-16/stage3-parallel-preclose/candidate.json').read_text())
 c.update(base=str(BASE),project=str(P),changes=changes,unchanged_sha256=unchanged,images=images,
  matlab_latest=json.loads((E/'matlab-result.json').read_text()),matlab_timeline={'initial_model_before_4_source_deltas':'2026-11-17 00:56:51','supplement_before_scope_stop_fix':json.loads((E/'matlab-result.json').read_text())['completed_local']},
  inherited_stage3_c=inherited,completed_local=datetime.datetime.now().isoformat(),build_passed=True,flashed=False,hardware_access=False,
  source_release_ms=100,carrier_release_ms=100,stage2_sync_regions=[0,1,2],stage2_pick_mask_by_region=[7,7,6],
  stage2_rise_xz_cached=True,stage2_return12_sync=True,stage2_third_return_unchanged=True,
  stage2_stop_inflight_not_reposted=True,stage3_semantics_unchanged=True,scope='E; R/Q unchanged',
  synchronization='X/Z cache then CAN1 FF666B and DMgo same owner call; actual DM target on existing DM service; physical skew unmeasured',
  source_rebuild_byte_identical=True,root_git_synced=False,isolated_bare_fetch_completed=True)
 (E/'candidate.json').write_text(json.dumps(c,indent=2)+'\n')
 (E/'source-manifest.json').write_text(json.dumps(manifest,indent=2)+'\n')
 (E/'checks.json').write_text(json.dumps({'passed':True,'actual_c':json.loads((E/'actual-c-result.json').read_text()),'dual_build':True,'source_rebuild_byte_identical':True,'inherited_stage3_c':inherited,'hardware_access':False,'flashed':False},indent=2)+'\n')
 print('PASS exact four-file delta; source rebuild identical; Stage3 owner+IT3 unchanged; dual target0err0warn')
if __name__=='__main__':main()
