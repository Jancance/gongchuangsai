"""Actual deep-Z C regression and dual Keil builds; no hardware access."""
from pathlib import Path
import hashlib,json,subprocess
from prepare_stage3_teach_mode_20261116 import R,BASE,P,E

def run(name,source,extra=(),flags=()):
    exe=E/(name+'.exe')
    command=['gcc','-std=c99','-Wall','-Wextra','-Werror','-Wno-misleading-indentation','-Wno-unused-function',
             '-I'+str(P/'CHASSIS/USER'),*flags,str(R/source),*map(str,extra),'-lm','-o',str(exe)]
    subprocess.run(command,check=True)
    result=subprocess.run([str(exe)],capture_output=True,text=True)
    (E/(name+'.log')).write_text(result.stdout+result.stderr)
    print(result.stdout+result.stderr,flush=True);assert result.returncode==0
    return result.stdout.strip()

def main():
    results={}
    results['deep']=run('actual-deep-z','tests/stage3_deep_z_20261116.c',
       [P/'CHASSIS/USER/source_parallel_flow.c',P/'CHASSIS/USER/lift_ruler_calibration.c'])
    results['owner']=run('actual-stage3-teach','tests/stage3_teach_mode_20261116.c',flags=['-DSPF_STAGE3_MAX_LIFT_PULSES=82000U'])
    results['owner_lrc']=run('actual-owner-lrc','tests/stage3_deep_z_owner_lrc_20261116.c',[P/'CHASSIS/USER/lift_ruler_calibration.c'],flags=['-Wno-unused-variable'])
    # Historical fixture used32p; current baseline explicitly uses80p. Preserve
    # that historical source and adapt only the generated fixture to its header.
    fixture=(R/'tests/lift_home_exit_20261114.c').read_text()
    assert fixture.count('(int64_t)32*360*scale/3200')==1
    fixture=fixture.replace('(int64_t)32*360*scale/3200','(int64_t)80*360*scale/3200').replace('bounded32p','current80p')
    fixture_path=E/'home-exit-current-fixture.c';fixture_path.write_text(fixture)
    results['home_exit']=run('actual-home-exit',fixture_path,[P/'CHASSIS/USER/lift_ruler_calibration.c'])
    results['source_group']=run('actual-source-group','tests/simplified_parallel_group_20261114.c',[P/'CHASSIS/USER/source_parallel_flow.c'])
    changes={};unchanged={}
    for path in P.rglob('*'):
        if not path.is_file() or any(x in ('Objects','Listings') for x in path.relative_to(P).parts):continue
        rel=path.relative_to(P).as_posix();before=BASE/rel
        if not before.exists() or before.read_bytes()!=path.read_bytes():changes[rel]=hashlib.sha256(path.read_bytes()).hexdigest()
    for n in ['USER/vehicle_parameters.h','CHASSIS/USER/source_mcu_cycle.inc','CHASSIS/USER/pzq_commit_overlap.inc','CHASSIS/USER/motion_control.h']:
        assert (P/n).read_bytes()==(BASE/n).read_bytes(),n
        unchanged[n]=hashlib.sha256((P/n).read_bytes()).hexdigest()
    images={}
    for target in ['Vehicle_LiftRuler','Vehicle_LiftRuler_Offline']:
        for folder in ['Objects','Listings']:(P/folder/target).mkdir(parents=True,exist_ok=True)
        log=E/(target+'-build.log')
        r=subprocess.run(['C:/Keil_v5/UV4/UV4.exe','-r',str(P/'gongxun-f407.uvprojx'),'-t',target,'-j0','-o',str(log)],timeout=180)
        data=log.read_bytes();print(data.decode('gb18030',errors='replace')[-600:],flush=True)
        assert r.returncode==0 and b'0 Error(s), 0 Warning(s)' in data
        axf=P/'Objects'/target/(target+'.axf')
        images[target]={'axf':axf.relative_to(R).as_posix(),'sha256':hashlib.sha256(axf.read_bytes()).hexdigest()}
    candidate=dict(base=BASE.relative_to(R).as_posix(),project=P.relative_to(R).as_posix(),changes=changes,
        unchanged_sha256=unchanged,images=images,build_passed=True,flashed=False,hardware_access=False,
        matlab_prior_to_implementation=True,physical_calibration_open=True,
        stage3_lift_max=82000,source_lift_max_unchanged=29700,stage3_extension_rpm=106,teach_entry_request=30,manual_teach_never_sets_alignment=True)
    (E/'candidate.json').write_text(json.dumps(candidate,indent=2)+'\n')
    (E/'checks.json').write_text(json.dumps(dict(passed=True,actual_c=results,dual_build=True,hardware_access=False,flashed=False),indent=2)+'\n')
if __name__=='__main__':main()
