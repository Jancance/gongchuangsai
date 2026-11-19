"""Freeze bounded changes, source/Flash identities and three-code overlay only."""
import datetime,hashlib,json,sys,py_compile
from prepare_three_frame_warm_20261118 import R,BASE,P,PI,PIBASE,E


def sha(p):return hashlib.sha256(p.read_bytes()).hexdigest()


def main():
    assert json.loads((E/'python-checks.json').read_text())['passed']
    assert json.loads((E/'c-checks.json').read_text())['passed']
    assert json.loads((E/'initial-guard-checks.json').read_text())['passed']
    sys.path[:0]=[str(R/'tmp/gripper_debug_deps'),str(R/'src')]
    from elftools.elf.elffile import ELFFile
    from smart_carrier.vision_manipulation.mirror_committed_pick_board import flash_segments_digest
    expected={'CHASSIS/USER/source_mcu_cycle.inc','CHASSIS/USER/integration10_complete.inc','CHASSIS/USER/vehicle_source_parallel.inc','CHASSIS/USER/pzq_commit_overlap.inc'}
    manifest={};changes={};encoding={}
    for p in P.rglob('*'):
        if not p.is_file() or any(n in ('Objects','Listings') for n in p.relative_to(P).parts):continue
        n=p.relative_to(P).as_posix();manifest[n]=sha(p);a=p.read_bytes();b=(BASE/n).read_bytes()
        if a==b:continue
        changes[n]=manifest[n]
        assert bytes(x for x in a if x>=128)==bytes(x for x in b if x>=128),(n,'nonASCII')
        assert a.count(b'\n')-a.count(b'\r\n')==b.count(b'\n')-b.count(b'\r\n'),(n,'bare LF')
        encoding[n]=dict(non_ascii_identical=True,bare_lf_identical=True,old_crlf=b.count(b'\r\n'),new_crlf=a.count(b'\r\n'))
    assert set(changes)==expected,changes
    # In particular, mirror/home write-gap and all actual speeds remain frozen.
    for n in ('USER/vehicle_parameters.h','CHASSIS/SYSTEM/CAN/can.c','CHASSIS/USER/extension_home_only.inc','CHASSIS/USER/boot_ready_link.inc','HARDWARE/GRIPPER/gripper.c'):
        assert (P/n).read_bytes()==(BASE/n).read_bytes(),n
    images={}
    for t in ('Vehicle_LiftRuler','Vehicle_LiftRuler_Offline'):
        assert b'0 Error(s), 0 Warning(s)' in (E/(t+'-build.log')).read_bytes()
        p=P/'Objects'/t/(t+'.axf')
        with p.open('rb') as f:
            elf=ELFFile(f);segments=[(int(s['p_paddr']),s.data()) for s in elf.iter_segments() if s['p_type']=='PT_LOAD' and s['p_filesz'] and 0x08000000<=s['p_paddr']<0x08100000]
        images[t]=dict(axf=str(p),sha256=sha(p),flash_segments_sha256=flash_segments_digest(segments),flash_bytes=sum(len(d) for _,d in segments))
    pi_files={p.name:sha(p) for p in PI.glob('*.py')}
    assert set(pi_files)=={'source_fast_cycle_view_20261115.py','source_pickup_window.py','source_cycle_dashboard_20261115.py'}
    for p in PI.glob('*.py'):py_compile.compile(str(p),doraise=True)
    bases={'source_fast_cycle_view_20261115.py':sha(R/'output/stage23_boot_ready_overlay_20261117/source_fast_cycle_view_20261115.py'),'source_cycle_dashboard_20261115.py':sha(PIBASE/'source_cycle_dashboard_20261115.py')}
    dependencies={n:sha(PIBASE/n) for n in ('stopped_frame_trigger.py','direct_frame_trigger_20261115.py','initial_rotation.py','observation_frame_epoch.py','source_alignment_settle.py','mcu_cycle_protocol.py')}
    c=dict(project=str(P),base=str(BASE),base_online_sha256='b06559122b766e622614cc33eed821f65de0cae7ea07f949349364007ff61efe',images=images,changes=changes,extend_sign=1,home_direction=1,parameter_save=0,home_diag_version=1,home_after_write_probe_version=1,pi_overlay=str(PI),pi_changes=pi_files,pi_base_hashes=bases,pi_unchanged_dependencies=dependencies,overlay_only=True,preserve_runtime_profiles=True,required_stopped_frames=3,first_pick_requires_observed120=True,first120_can_warm_during_alignment=True,later_picks_require_rotation=False,pick_release='first_alignment_beep_start',per_committed_pick_beep_count=1,side_pick_pulses=25700,bottom_pick_pulses=29700,lift_pulses_per_mm=400,grip_then_return_same_control_call=True,initial_parallel_axes=3,initial_extension_mm=65,build_passed=True,self_review_passed=True,independent_model_review=False,hardware_access=False,flashed=False,actual_project_replaced=False,requirements_R=['SRC-01','SRC-02','RUN-01','RUN-02','PWR-01','MECH-01','MECH-02'],requirements_E=['RUN-06','CAL-02','CAL-05','VIS-15'],requirements_Q='Official outstanding questions unchanged; physical pickup height/retention and initial swept-volume/start skew remain to be verified',completed_local=datetime.datetime.now().isoformat())
    checks=dict(passed=True,matlab=[json.loads((E/n).read_text()) for n in ('matlab-result.json','matlab-details.json','matlab-initial-gate.json')],python=json.loads((E/'python-checks.json').read_text()),actual_c=json.loads((E/'c-checks.json').read_text()),complete_initial_guard=json.loads((E/'initial-guard-checks.json').read_text()),dual_build_clean=True,encoding_preserved=True,hardware_access=False)
    for n,data in [('candidate.json',c),('source-manifest.json',manifest),('encoding-line-endings.json',encoding),('checks.json',checks)]:
        (E/n).write_text(json.dumps(data,indent=2)+'\n')
    (PI/'manifest-delta.json').write_text(json.dumps(dict(files=pi_files,base_hashes=bases,unchanged_dependencies=dependencies,overlay_only=True,preserve_all_profiles=True),indent=2)+'\n')
    print(json.dumps(dict(images=images,firmware_changed=list(changes),pi_changed=pi_files,source_files=len(manifest)),ensure_ascii=False))


if __name__=='__main__':main()
