"""Stage and install a separate chassis target in the user's existing arm project.

No flashing, driver writes, or hardware commands. Existing arm source is preserved.
"""
from pathlib import Path
import copy
import hashlib
import json
import re
import shutil
import subprocess
import sys
import xml.etree.ElementTree as ET

ROOT = Path(__file__).resolve().parents[1]
ACTUAL = Path('D:/新建文件夹4/工创赛/代码/stm32/3.stm32f407工程')
SOURCE = ROOT / '3.stm32f407工程'
EVIDENCE = ROOT / 'evidence/integration/2026-11-03/actual-project-chassis'
STAGE = ROOT / 'tmp/actual_project_chassis_20261103_v3'
PROJECT = 'gongxun-f407.uvprojx'
SUBDIR = 'CHASSIS'
VEHICLE_TARGET = 'Chassis_Stage1_Emm'
OFFLINE_TARGET = 'Chassis_Stage1_Offline'
SUFFIXES = {'.c', '.h', '.s', '.inc', '.lib', '.a', '.scvd'}


def sha(path):
    return hashlib.sha256(path.read_bytes()).hexdigest()


def snapshot(folder):
    return {p.relative_to(folder).as_posix(): sha(p)
            for p in folder.rglob('*') if p.is_file()
            and (p.suffix.lower() in SUFFIXES or p.name == PROJECT)
            and SUBDIR not in p.relative_to(folder).parts
            and not {'Objects', 'Listings'}.intersection(p.relative_to(folder).parts)}


def save(data):
    (EVIDENCE / 'manifest.json').write_text(
        json.dumps(data, ensure_ascii=False, indent=2) + '\n', encoding='utf-8')


def build(folder, target, log):
    r = subprocess.run(['C:/Keil_v5/UV4/UV4.exe', '-r', str(folder / PROJECT),
                        '-t', target, '-j0', '-o', str(log)],
                       capture_output=True, timeout=180)
    s = log.read_text(encoding='utf-8', errors='replace')
    print(target, r.returncode, s[-850:], flush=True)
    assert r.returncode == 0 and re.search(r'0 Error\(s\), 0 Warning\(s\)', s), s[-5000:]
    return {'target': target, 'returncode': r.returncode,
            'log': str(log), 'log_sha256': sha(log)}


def target_element(template, compiler, name, offline):
    target = copy.deepcopy(template)
    target.find('TargetName').text = name
    target.find('pCCUsed').text = compiler
    for n in target.findall('.//GroupName'):
        n.text = 'Chassis_' + n.text
    for n in target.findall('.//FilePath'):
        rel = (n.text or '').replace('\\', '/').removeprefix('./')
        assert not rel.startswith('../') and ':' not in rel, rel
        n.text = '.\\' + SUBDIR + '\\' + rel.replace('/', '\\')
    for n in target.findall('.//IncludePath'):
        if n.text:
            paths = []
            for path in n.text.split(';'):
                rel = path.replace('\\', '/').removeprefix('./')
                assert not rel.startswith('../') and ':' not in rel, rel
                paths.append('.\\' + SUBDIR + '\\' + rel.replace('/', '\\'))
            n.text = ';'.join(paths)
    common = target.find('TargetOption/TargetCommonOption')
    common.find('OutputDirectory').text = '.\\Objects\\' + name + '\\'
    common.find('ListingPath').text = '.\\Listings\\' + name + '\\'
    common.find('OutputName').text = name
    common.find('SelectedForBatchBuild').text = '0'
    define = target.find('.//Cads/VariousControls/Define')
    assert 'APP_MOTOR_OFFLINE_ONLY=0' in define.text
    define.text = define.text.replace('APP_MOTOR_OFFLINE_ONLY=0',
                                    'APP_MOTOR_OFFLINE_ONLY=' + str(offline))
    return target


def exclude_groups(groups):
    result = copy.deepcopy(groups)
    for f in result.findall('.//File'):
        option = f.find('FileOption')
        if option is None:
            option = ET.SubElement(f, 'FileOption')
        common = option.find('CommonProperty')
        if common is None:
            common = ET.SubElement(option, 'CommonProperty')
        include = common.find('IncludeInBuild')
        if include is None:
            include = ET.SubElement(common, 'IncludeInBuild')
        include.text = '0'
    return result


def stage():
    assert not STAGE.exists() and not (ACTUAL / SUBDIR).exists()
    EVIDENCE.mkdir(parents=True, exist_ok=True)
    initial = snapshot(ACTUAL)
    assert 'USER/arm_sequence.c' in initial
    STAGE.mkdir(parents=True)
    for rel in initial:
        dst = STAGE / rel
        dst.parent.mkdir(parents=True, exist_ok=True)
        shutil.copyfile(ACTUAL / rel, dst)
    # Only sources and required headers, never old build products or debug settings.
    for p in SOURCE.rglob('*'):
        rel = p.relative_to(SOURCE)
        if (p.is_file() and p.suffix.lower() in SUFFIXES
                and not {'Objects', 'Listings'}.intersection(rel.parts)):
            dst = STAGE / SUBDIR / rel
            dst.parent.mkdir(parents=True, exist_ok=True)
            shutil.copyfile(p, dst)
    config = STAGE / SUBDIR / 'USER/zdt_emm_config.h'
    data = config.read_bytes()
    assert data.count(b'#define ZDT_EMM_SETTINGS_CONFIRMED 0U') == 1
    config.write_bytes(data.replace(b'#define ZDT_EMM_SETTINGS_CONFIRMED 0U',
                                   b'#define ZDT_EMM_SETTINGS_CONFIRMED 1U'))
    original = (ACTUAL / PROJECT).read_bytes()
    arm = ET.fromstring(original).find('Targets/Target')
    compiler = arm.find('pCCUsed').text
    template = ET.parse(SOURCE / PROJECT).find('Targets/Target')
    targets = [target_element(template, compiler, VEHICLE_TARGET, 0),
               target_element(template, compiler, OFFLINE_TARGET, 1)]
    # uVision shares the group/file inventory across targets. Include every file
    # explicitly and exclude the other firmware to prevent implicit file merging.
    project = ET.fromstring(original)
    arm_target = project.find('Targets/Target')
    arm_groups = copy.deepcopy(arm_target.find('Groups'))
    chassis_groups = copy.deepcopy(targets[0].find('Groups'))
    for g in exclude_groups(chassis_groups):
        arm_target.find('Groups').append(g)
    for t in targets:
        own = t.find('Groups')
        t.remove(own)
        combined = ET.SubElement(t, 'Groups')
        combined.extend(exclude_groups(arm_groups))
        combined.extend(own)
        project.find('Targets').append(t)
    ET.ElementTree(project).write(STAGE / PROJECT, encoding='utf-8', xml_declaration=True)
    for n in ET.parse(STAGE / PROJECT).findall('.//FilePath'):
        assert (STAGE / n.text.replace('\\', '/')).is_file(), n.text
    # Original options and original arm file groups are unchanged; appended
    # chassis groups in the arm target have explicit IncludeInBuild=0.
    assert ET.tostring(arm_target.find('TargetOption')) == ET.tostring(arm.find('TargetOption'))
    assert all(sha(STAGE / rel) == h for rel, h in initial.items() if rel != PROJECT)
    manifest = dict(actual=str(ACTUAL), stage=str(STAGE), source=str(SOURCE),
                    original_sha256=initial, project_original_sha256=sha(ACTUAL / PROJECT),
                    user_report='User confirmed all four chassis driver menus configured in this chat.',
                    settings_confirmed=1, hardware_readback=False, flashed=False,
                    hardware_commands_sent=False, integrated_arm_chassis_runtime=False,
                    requirements_R=['RUN-01', 'RUN-02', 'FIELD-02', 'FIELD-06', 'PWR-01',
                                    'MECH-01', 'MECH-02', 'QR-01', 'DSP-01'],
                    requirements_E=['LOC-03', 'RUN-06'],
                    official_Q='待技术答疑确认', builds=[])
    save(manifest)
    for name in [arm.find('TargetName').text, VEHICLE_TARGET, OFFLINE_TARGET]:
        manifest['builds'].append(build(STAGE, name, EVIDENCE / ('stage-' + name.replace(' ', '_') + '.log')))
        save(manifest)
    manifest['installation_sha256'] = {
        p.relative_to(STAGE).as_posix(): sha(p) for p in (STAGE / SUBDIR).rglob('*')
        if p.is_file()}
    manifest['installation_sha256'][PROJECT] = sha(STAGE / PROJECT)
    manifest['staged_passed'] = True
    save(manifest)


def install():
    m = json.loads((EVIDENCE / 'manifest.json').read_text(encoding='utf-8'))
    assert m['staged_passed'] and len(m['builds']) == 3
    assert snapshot(ACTUAL) == m['original_sha256'], 'Actual source changed since staging'
    assert not (ACTUAL / SUBDIR).exists()
    backup = EVIDENCE / 'before-install'
    for rel in m['original_sha256']:
        dst = backup / rel
        dst.parent.mkdir(parents=True, exist_ok=True)
        shutil.copyfile(ACTUAL / rel, dst)
    for rel, h in m['installation_sha256'].items():
        assert sha(STAGE / rel) == h
    shutil.copytree(STAGE / SUBDIR, ACTUAL / SUBDIR)
    shutil.copyfile(STAGE / PROJECT, ACTUAL / PROJECT)
    assert all(sha(ACTUAL / rel) == h for rel, h in m['installation_sha256'].items())
    assert all(sha(ACTUAL / rel) == h for rel, h in m['original_sha256'].items() if rel != PROJECT)
    m['installed'] = True
    m['actual_builds'] = []
    save(m)
    for name in [VEHICLE_TARGET, OFFLINE_TARGET]:
        m['actual_builds'].append(build(ACTUAL, name, EVIDENCE / ('actual-' + name + '.log')))
        save(m)
    m['original_arm_sources_preserved'] = all(
        sha(ACTUAL / rel) == h for rel, h in m['original_sha256'].items() if rel != PROJECT)
    assert m['original_arm_sources_preserved']
    save(m)


if __name__ == '__main__':
    install() if '--install' in sys.argv else stage()
