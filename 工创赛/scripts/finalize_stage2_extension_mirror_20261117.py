"""Scope/encoding/Flash identity manifest only; never touches hardware."""
from pathlib import Path
import hashlib,json,sys,datetime
from prepare_stage2_extension_mirror_20261117 import R,BASE,P,E

def sha(p):return hashlib.sha256(p.read_bytes()).hexdigest()

def main():
 sys.path.insert(0,str(R/'tmp/gripper_debug_deps'))
 sys.path.insert(0,str(R/'src'))
 from elftools.elf.elffile import ELFFile
 from smart_carrier.vision_manipulation.mirror_committed_pick_board import flash_segments_digest
 assert json.loads((E/'c-checks.json').read_text())['passed']
 expected={'USER/vehicle_parameters.h','CHASSIS/USER/extension_home_only.inc','CHASSIS/SYSTEM/CAN/can.c','CHASSIS/USER/integration10_complete.inc','CHASSIS/USER/pzq_commit_overlap.inc','CHASSIS/USER/pzq_region_batch.inc','CHASSIS/USER/stage3_cycle.inc'}
 changes={};manifest={};encoding={}
 for p in P.rglob('*'):
  if not p.is_file() or any(n in ('Objects','Listings') for n in p.relative_to(P).parts):continue
  n=p.relative_to(P).as_posix();manifest[n]=sha(p);a=p.read_bytes();b=(BASE/n).read_bytes()
  if a==b:continue
  changes[n]=manifest[n]
  assert bytes(x for x in a if x>=128)==bytes(x for x in b if x>=128),(n,'nonASCII changed')
  assert a.count(b'\n')-a.count(b'\r\n')==b.count(b'\n')-b.count(b'\r\n'),(n,'bare LF changed')
  try:b.decode('utf-8');a.decode('utf-8');codec='utf-8'
  except UnicodeDecodeError:b.decode('gb18030');a.decode('gb18030');codec='gb18030'
  encoding[n]={'encoding':codec,'non_ascii_bytes_identical':True,'base_crlf':b.count(b'\r\n'),'new_crlf':a.count(b'\r\n'),'bare_lf_unchanged':a.count(b'\n')-a.count(b'\r\n')}
 assert set(changes)==expected,set(changes)
 images={}
 for t in ('Vehicle_LiftRuler','Vehicle_LiftRuler_Offline'):
  assert b'0 Error(s), 0 Warning(s)'in(E/(t+'-build.log')).read_bytes()
  p=P/'Objects'/t/(t+'.axf')
  with p.open('rb') as f:
   elf=ELFFile(f);segments=[(int(s['p_paddr']),s.data())for s in elf.iter_segments()if s['p_type']=='PT_LOAD'and s['p_filesz']and 0x08000000<=s['p_paddr']<0x08100000]
  images[t]={'axf':str(p),'sha256':sha(p),'flash_segments_sha256':flash_segments_digest(segments),'flash_bytes':sum(len(d)for _,d in segments)}
 # All parameters beyond the two explicit sign choices are byte-identical.
 a=(P/'USER/vehicle_parameters.h').read_bytes();b=(BASE/'USER/vehicle_parameters.h').read_bytes()
 assert b.replace(b'#define VEHICLE_EXTENSION_HOME_DIRECTION 0U',b'#define VEHICLE_EXTENSION_HOME_DIRECTION 1U').replace(b'#define STARTUP_EXTENSION_EXTEND_SIGN (-1)',b'#define STARTUP_EXTENSION_EXTEND_SIGN (1)')==a
 c={'base':str(BASE),'base_online_sha256':'18cc11eec2bf28b2642dbbb31c7c25807d651317a20464ca29997584262f293d','project':str(P),'changes':changes,'images':images,'extend_sign':1,'home_direction':1,'parameter_save':0,'other_parameters_unchanged':True,'speed_parameters_unchanged':True,'hardware_access':False,'flashed':False,'review_passed':False,'build_passed':True,'matlab_initial':json.loads((E/'matlab-result.json').read_text()),'matlab_cancel_supplement':json.loads((E/'matlab-cancel-result.json').read_text()),'actual_tests':json.loads((E/'c-checks.json').read_text()),'requirements_R':['RUN-01','RUN-02','PWR-01','MECH-01','MECH-02'],'requirements_E':['RUN-06','CAL-05','VIS-15'],'requirements_Q':'Official unresolved technical clarifications unchanged; mechanical mirror and new origin await hardware validation','completed_local':datetime.datetime.now().isoformat()}
 (E/'candidate.json').write_text(json.dumps(c,indent=2)+'\n')
 (E/'source-manifest.json').write_text(json.dumps(manifest,indent=2)+'\n')
 (E/'encoding-line-endings.json').write_text(json.dumps(encoding,indent=2)+'\n')
 print(json.dumps({'images':images,'changed_files':list(changes),'source_files':len(manifest),'encoding_line_endings_preserved':True},ensure_ascii=False),flush=True)
if __name__=='__main__':main()
