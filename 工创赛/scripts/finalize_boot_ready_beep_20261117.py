"""Boot cue scope/hash/encoding manifest; no hardware or deployment."""
import sys,json,hashlib,datetime,py_compile
from pathlib import Path
from prepare_boot_ready_beep_20261117 import R,BASE,P,PI,PIBASE,E

def sha(p):return hashlib.sha256(p.read_bytes()).hexdigest()
def main():
 sys.path[:0]=[str(R/'tmp/gripper_debug_deps'),str(R/'src')]
 from elftools.elf.elffile import ELFFile
 from smart_carrier.vision_manipulation.mirror_committed_pick_board import flash_segments_digest
 checks=json.loads((E/'checks.json').read_text());assert checks['passed']
 changes={};manifest={};encoding={}
 for p in P.rglob('*'):
  if not p.is_file()or any(n in('Objects','Listings')for n in p.relative_to(P).parts):continue
  n=p.relative_to(P).as_posix();manifest[n]=sha(p);old=BASE/n
  if not old.exists():changes[n]=manifest[n];continue
  a=p.read_bytes();b=old.read_bytes()
  if a==b:continue
  changes[n]=manifest[n];assert bytes(v for v in a if v>=128)==bytes(v for v in b if v>=128)
  assert a.count(b'\n')-a.count(b'\r\n')==b.count(b'\n')-b.count(b'\r\n')
  encoding[n]=dict(non_ascii_bytes_identical=True,original_crlf=b.count(b'\r\n'),new_crlf=a.count(b'\r\n'),bare_lf_unchanged=a.count(b'\n')-a.count(b'\r\n'))
 assert set(changes)=={'CHASSIS/USER/main.c','CHASSIS/USER/integration10_complete.inc','CHASSIS/USER/boot_ready_link.h','CHASSIS/USER/boot_ready_link.inc'}
 images={}
 for t in('Vehicle_LiftRuler','Vehicle_LiftRuler_Offline'):
  assert b'0 Error(s), 0 Warning(s)'in(E/(t+'-build.log')).read_bytes()
  p=P/'Objects'/t/(t+'.axf')
  with p.open('rb')as f:
   elf=ELFFile(f);segments=[(int(s['p_paddr']),s.data())for s in elf.iter_segments()if s['p_type']=='PT_LOAD'and s['p_filesz']and 0x08000000<=s['p_paddr']<0x08100000]
  images[t]=dict(axf=str(p),sha256=sha(p),flash_segments_sha256=flash_segments_digest(segments),flash_bytes=sum(len(data)for _,data in segments))
 pi_changes={p.name:sha(p)for p in PI.glob('*.py')}
 assert set(pi_changes)=={'source_fast_cycle_view_20261115.py','boot_ready_handshake.py'}
 for n in pi_changes:py_compile.compile(str(PI/n),doraise=True)
 base_main=sha(PIBASE/'source_fast_cycle_view_20261115.py');assert base_main=='20814ac5f8d38c16efefd7291ee1be910b8dc73ae100dc7543cd14fbc4f40fd4'
 delta=dict(base_main_sha256=base_main,files=pi_changes,overlay_only=True,preserve_all_runtime_profiles=True,overwrite_full_manifest=False)
 (PI/'manifest-delta.json').write_text(json.dumps(delta,indent=2)+'\n')
 candidate=dict(base=str(BASE),base_online_sha256='30994d4a7fb445ea45e101425beeb73ca82b02ef74f9d893eaea133f7fe75a61',project=str(P),changes=changes,images=images,extend_sign=1,home_direction=1,parameter_save=0,pi_overlay=str(PI),pi_changes=pi_changes,pi_base_main_sha256=base_main,pi_runtime_profile_policy='preserve actual 3aee1fdd J and all HSV/reference/threshold/config files; update only two code keys in existing runtime manifest',matlab=json.loads((E/'matlab-result.json').read_text()),checks=checks,build_passed=True,review_passed=False,hardware_access=False,flashed=False,requirements_R=['RUN-01','PWR-01','MECH-01','MECH-02'],requirements_E=['RUN-06'],requirements_Q='Official unresolved technical clarifications unchanged; physical UART/audible boot validation pending',completed_local=datetime.datetime.now().isoformat())
 (E/'candidate.json').write_text(json.dumps(candidate,indent=2)+'\n');(E/'source-manifest.json').write_text(json.dumps(manifest,indent=2)+'\n');(E/'encoding-line-endings.json').write_text(json.dumps(encoding,indent=2)+'\n')
 print(json.dumps(dict(images=images,changes=list(changes),pi_changes=pi_changes,source_files=len(manifest)),ensure_ascii=False),flush=True)
if __name__=='__main__':main()
