"""Freeze the reviewed module firmware against the direct handoff baseline."""
import json
import re

from finalize_stage3_fast_cycle import ELFFile, WORK, evidence, load, sha

D = WORK/'stage3_modules_20261120'
PROJECT = D/'project'
PARENT = WORK/'stage3_directhandoff_20261120/stage3-directhandoff-compatible.json'
CHANGED = {'CHASSIS/USER/' + name for name in ('main.c','region_dispatch.h','region_dispatch.inc',
    'source_mcu_cycle.inc','stage3_cycle.inc')} | {'USER/vehicle_parameters.h'}


def main():
    m = load(PARENT)
    before = m['unchanged_sha256'] | m['changes']
    after = {name:sha(PROJECT/name) for name in before}
    changed = {name for name in before if before[name] != after[name]}
    assert len(after)==2053 and changed==CHANGED, changed
    changed_at = max((PROJECT/name).stat().st_mtime_ns for name in changed)
    model = WORK/'stage3_modules_validation.json'
    result = load(model)
    assert result['passed'] and result['model_only'] and not result['physical_acceptance']
    assert (result['total_cases'],result['stack_offset_pulses'])==(72,24000)
    fixture = D/'modules-fixture.log'
    assert 'PASS module lifecycle' in fixture.read_text(encoding='utf-8-sig')
    assert fixture.stat().st_mtime_ns>changed_at
    assert (m['stage3_strict_align']['accept_norm_um'],m['stage3_strict_align']['accept_abs_yaw_mdeg'])==(2000,1000)
    for spec in (*m['stage3_trial_profile']['modes'].values(),m['stage3_strict_align']['pi_module'],m['offline_vision_config']):
        assert sha(D/spec['path'])==spec['sha256']
    m['compatibility_evidence']['current_source_hashes'] = {
        k:after[k] for k in m['compatibility_evidence']['current_source_hashes']}
    cumulative = m['changes'].keys() | changed
    m.update(project=str(PROJECT),parent_manifest={**evidence(PARENT),'image_sha256':m['images']['Vehicle_LiftRuler']['sha256']},
             changes={k:after[k] for k in sorted(cumulative)},
             unchanged_sha256={k:v for k,v in after.items() if k not in cumulative},
             offline_candidate_summary=dict(source_inventory_count=len(after),changed_relative_to_parent=sorted(changed)),
             flashed=False,pi_deployed=False,physical_acceptance=False,run_count=0)
    for target, prior in m['images'].copy().items():
        axf = PROJECT/'Objects'/target/f'{target}.axf'
        log = D/f'{target}.log'
        assert min(axf.stat().st_mtime_ns,log.stat().st_mtime_ns)>changed_at and sha(axf)!=prior['sha256']
        assert re.search(r'\.axf" - 0 Error\(s\), 0 Warning\(s\)\.',log.read_text(encoding='ascii'))
        with axf.open('rb') as stream:
            elf = ELFFile(stream)
            symbols = {s.name:int(s['st_size']) for s in elf.get_section_by_name('.symtab').iter_symbols()
                       if s['st_info']['type']=='STT_OBJECT'}
            size = sum(int(s['p_filesz']) for s in elf.iter_segments()
                       if s['p_type']=='PT_LOAD' and 0x08000000<=int(s['p_paddr'])<0x08100000)
        for key in ('cycle_symbol_sizes','timing_symbol_sizes','finalzero_observed_symbol_sizes'):
            assert {n:symbols[n] for n in prior[key]}==prior[key]
        m['images'][target]=dict(prior,axf=str(axf),sha256=sha(axf),flash_bytes=size,
                                 build=dict(log=evidence(log),errors=0,warnings=0))
    m['capabilities']['stage3_region_modules'] = True
    m['capabilities']['stage3_standalone_pc13'] = False
    m['stage3_standalone_pc13']['enabled'] = False
    m['stage3_region_modules'] = dict(
        api=['RegionDispatch_Begin','RegionDispatch_Poll','RegionDispatch_Stop'],
        module=['source','rough','stack'],call_context='RTOS task',one_can_owner=True,
        fresh_increasing_run_id=True,stack_terminal_phase=36,stack_offset_pulses=24000,
        bench_pc13_enabled=False,full_route_entry_enabled=False,physical_acceptance=False,
        model={k:evidence(model.with_suffix(s)) for k,s in (('report','.json'),('script','.m'),('log','.log'))},
        fixture=dict(log=evidence(fixture),source=evidence(WORK/'verify_stage3_modules.py')))
    m['build_passed'] = True
    output = D/'stage3-modules-compatible.json'
    output.write_text(json.dumps(m,indent=2)+'\n',encoding='ascii')
    print(output)
    for target, image in m['images'].items():
        print(target,image['sha256'],image['flash_bytes'])


if __name__=='__main__':
    main()
