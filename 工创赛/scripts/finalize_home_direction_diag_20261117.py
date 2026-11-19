"""Freeze one-file passive diagnostic candidate and actual build identities."""
import datetime
import difflib
import hashlib
import json
import sys
from pathlib import Path
from prepare_home_direction_diag_20261117 import R, BASE, P, E


def main():
    sys.path[:0] = [str(R/'tmp/gripper_debug_deps'), str(R/'src')]
    from elftools.elf.elffile import ELFFile
    from smart_carrier.vision_manipulation.mirror_committed_pick_board import flash_segments_digest
    def sha(p): return hashlib.sha256(p.read_bytes()).hexdigest()
    checks=json.loads((E/'checks.json').read_text()); assert checks['passed']
    baseline=json.loads((E.parent/'boot-ready-beep/candidate.json').read_text())
    assert sha(Path(baseline['images']['Vehicle_LiftRuler']['axf']))==baseline['images']['Vehicle_LiftRuler']['sha256']
    manifest={};changes={}
    for p in P.rglob('*'):
        if not p.is_file() or any(x in ('Objects','Listings') for x in p.relative_to(P).parts): continue
        n=p.relative_to(P).as_posix();manifest[n]=sha(p)
        if p.read_bytes()!=(BASE/n).read_bytes(): changes[n]=manifest[n]
    assert set(changes)=={'CHASSIS/SYSTEM/CAN/can.c'}
    a=(P/'CHASSIS/SYSTEM/CAN/can.c').read_bytes();b=(BASE/'CHASSIS/SYSTEM/CAN/can.c').read_bytes()
    assert bytes(x for x in a if x>=128)==bytes(x for x in b if x>=128)
    assert a.count(b'\n')-a.count(b'\r\n')==b.count(b'\n')-b.count(b'\r\n')
    # Everything outside the recorded helper insertion/replacement stays exact.
    old=(E/'helper-original.c').read_bytes();new=(E/'helper-instrumented.c').read_bytes()
    assert b.count(old)==1 and b.replace(old,new,1)==a
    (E/'source-diff.patch').write_text(''.join(difflib.unified_diff(b.decode('gb18030').splitlines(True),a.decode('gb18030').splitlines(True),fromfile='boot-ready-beep/can.c',tofile='home-direction-diag/can.c')),encoding='utf-8')
    images={}
    for target in ('Vehicle_LiftRuler','Vehicle_LiftRuler_Offline'):
        assert b'0 Error(s), 0 Warning(s)' in (E/(target+'-build.log')).read_bytes()
        p=P/'Objects'/target/(target+'.axf')
        with p.open('rb') as f:
            elf=ELFFile(f);segments=[(int(s['p_paddr']),s.data()) for s in elf.iter_segments() if s['p_type']=='PT_LOAD' and s['p_filesz'] and 0x08000000<=s['p_paddr']<0x08100000]
            objects={s.name:int(s['st_size']) for s in elf.get_section_by_name('.symtab').iter_symbols() if s['st_info']['type']=='STT_OBJECT'}
            if target=='Vehicle_LiftRuler': assert objects['g_home_direction_diag']==136
        images[target]=dict(axf=str(p),sha256=sha(p),flash_segments_sha256=flash_segments_digest(segments),flash_bytes=sum(len(d) for _,d in segments))
    candidate=dict(project=str(P),base=str(BASE),base_online_sha256=baseline['images']['Vehicle_LiftRuler']['sha256'],images=images,changes=changes,extend_sign=1,home_direction=1,parameter_save=0,home_diag_version=1,diagnostic_bytes=136,checks=checks,build_passed=True,self_review_passed=True,independent_model_review=False,hardware_access=False,flashed=False,requirements_R=['RUN-01','RUN-02','PWR-01','MECH-01','MECH-02'],requirements_E=['RUN-06','CAL-05','VIS-15'],requirements_Q='Unchanged; pending official technical clarification',completed_local=datetime.datetime.now().isoformat())
    (E/'candidate.json').write_text(json.dumps(candidate,indent=2)+'\n')
    (E/'source-manifest.json').write_text(json.dumps(manifest,indent=2)+'\n')
    (E/'encoding-line-endings.json').write_text(json.dumps(dict(non_ascii_identical=True,bare_lf_identical=True,base_crlf=b.count(b'\r\n'),new_crlf=a.count(b'\r\n'),outside_helper_byte_identical=True),indent=2)+'\n')
    print(json.dumps(dict(images=images,changes=changes,source_files=len(manifest))))


if __name__=='__main__':main()
