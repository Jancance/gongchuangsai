"""Verify the exact one-token firmware change and freeze built images."""
import datetime,hashlib,json,sys
from prepare_home_readback_window_20261118 import R,BASE,P,E


def main():
    sys.path[:0]=[str(R/'tmp/gripper_debug_deps'),str(R/'src')]
    from elftools.elf.elffile import ELFFile
    from smart_carrier.vision_manipulation.mirror_committed_pick_board import flash_segments_digest
    assert json.loads((E/'checks.json').read_text())['passed']
    def sha(p):return hashlib.sha256(p.read_bytes()).hexdigest()
    manifest={};changes={}
    for p in P.rglob('*'):
        if not p.is_file() or any(x in ('Objects','Listings') for x in p.relative_to(P).parts):continue
        n=p.relative_to(P).as_posix();manifest[n]=sha(p)
        if p.read_bytes()!=(BASE/n).read_bytes():changes[n]=manifest[n]
    assert set(changes)=={'CHASSIS/SYSTEM/CAN/can.c'}
    p=P/'CHASSIS/SYSTEM/CAN/can.c';a=p.read_bytes();b=(BASE/'CHASSIS/SYSTEM/CAN/can.c').read_bytes()
    sig=b'uint32_t StartupAxis_ReadHomeParameters(';i=b.index(sig);j=b.index(b'\r\n}\r\n',i)+3
    old=b[i:j];new=old.replace(b'pdMS_TO_TICKS(CAN1_SINGLE_REPLY_WAIT_MS)',b'pdMS_TO_TICKS(CAN1_PARAM_REPLY_WAIT_MS)')
    assert b[:i]+new+b[j:]==a
    images={}
    for t in ('Vehicle_LiftRuler','Vehicle_LiftRuler_Offline'):
        assert b'0 Error(s), 0 Warning(s)' in (E/(t+'-build.log')).read_bytes()
        p=P/'Objects'/t/(t+'.axf')
        with p.open('rb') as f:
            elf=ELFFile(f);segments=[(int(s['p_paddr']),s.data()) for s in elf.iter_segments() if s['p_type']=='PT_LOAD' and s['p_filesz'] and 0x08000000<=s['p_paddr']<0x08100000]
        images[t]=dict(axf=str(p),sha256=sha(p),flash_segments_sha256=flash_segments_digest(segments),flash_bytes=sum(len(d) for _,d in segments))
    c=dict(project=str(P),base=str(BASE),base_online_sha256='871ea5372affd24c7d9c1849bb535782a26564629e30cdf27fafc65c88348192',images=images,changes=changes,extend_sign=1,home_direction=1,parameter_save=0,home_diag_version=1,parameter_reply_window_ms=250,only_change='Extension 0x22 multi-frame read uses existing 250ms parameter window instead of 60ms single-frame window',matlab=json.loads((E/'matlab-result.json').read_text()),checks=json.loads((E/'checks.json').read_text()),build_passed=True,self_review_passed=True,independent_model_review=False,hardware_access=False,flashed=False,requirements_R=['RUN-01','RUN-02','PWR-01','MECH-01','MECH-02'],requirements_E=['RUN-06','CAL-05','VIS-15'],requirements_Q='Unchanged; pending official technical clarification',completed_local=datetime.datetime.now().isoformat())
    (E/'candidate.json').write_text(json.dumps(c,indent=2)+'\n');(E/'source-manifest.json').write_text(json.dumps(manifest,indent=2)+'\n')
    print(json.dumps(images))


if __name__=='__main__':main()
