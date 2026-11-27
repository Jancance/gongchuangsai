from pathlib import Path
import hashlib, json, re, shutil, subprocess, xml.etree.ElementTree as ET

repo = Path(__file__).resolve().parents[2]
out = Path(__file__).resolve().parent
fw = repo / '3.stm32f407工程'
tree = out / 'build_tree'
if not tree.exists():
    shutil.copytree(fw, tree, ignore=shutil.ignore_patterns('Objects', 'Listings', '*.uvoptx', '*.uvguix.*'))
project = tree / 'gongxun-f407.uvprojx'
compiler = Path('D:/Keil_v5/ARM/arm5_compiler/Bin/armcc.exe')
uv4 = Path('D:/Keil_v5/UV4/UV4.exe')
root = ET.parse(project).getroot()
targets = {t.findtext('TargetName'): t for t in root.findall('./Targets/Target')}
parameter = fw / 'USER/vehicle_parameters.h'
original = parameter.read_bytes()
before = (out / 'before/USER/vehicle_parameters.h').read_bytes()
head = subprocess.check_output(['git', 'show', 'HEAD:3.stm32f407工程/USER/vehicle_parameters.h'], cwd=repo)
staged = original.replace(b'#define VEHICLE_FIELD_MODE VEHICLE_FIELD_MODE_STRAIGHT_DEBUG', b'#define VEHICLE_FIELD_MODE VEHICLE_FIELD_MODE_NO_RADAR', 1).replace(b'#define VEHICLE_SPEED_TRIAL160_DIRECTION 1U', b'#define VEHICLE_SPEED_TRIAL160_DIRECTION 3U', 1)
(out / 'vehicle_parameters.index.h').write_bytes(staged)
assert head.replace(b'\r\n', b'\n').replace(b'#define VEHICLE_FIELD_MODE VEHICLE_FIELD_MODE_NO_RADAR', b'#define VEHICLE_FIELD_MODE VEHICLE_FIELD_MODE_STRAIGHT_DEBUG', 1).replace(b'#define VEHICLE_SPEED_TRIAL160_DIRECTION 3U', b'#define VEHICLE_SPEED_TRIAL160_DIRECTION 1U', 1) == before.replace(b'\r\n', b'\n')
report = {'hardware_accessed': False, 'downloaded': False, 'preprocess': [], 'builds': [], 'index_header_sha256': hashlib.sha256(staged).hexdigest()}

def preprocess(mode, direction, offline, old=False):
    header = before if old else original
    header = header.replace(b'#define VEHICLE_FIELD_MODE VEHICLE_FIELD_MODE_STRAIGHT_DEBUG', ('#define VEHICLE_FIELD_MODE %dU' % mode).encode(), 1)
    if not old:
        header = header.replace(b'#define VEHICLE_TURN_DIRECTION 0U', ('#define VEHICLE_TURN_DIRECTION %dU' % direction).encode(), 1)
    (tree / 'USER/vehicle_parameters.h').write_bytes(header)
    target = targets['Vehicle_Offline' if offline else 'Vehicle_Integrated']
    controls = target.find('./TargetOption/TargetArmAds/Cads/VariousControls')
    defines = controls.findtext('Define').split(',')
    includes = controls.findtext('IncludePath').split(';')
    args = [str(compiler), '--cpu', 'Cortex-M4', '--c99', '-E', *['-D' + d for d in defines]]
    for inc in includes:
        args += ['-I', str(tree / inc.replace('\\', '/'))]
    text = '#include "app_config.h"\nCONFIG_VALUES VEHICLE_SPEED_TRIAL160_MODE APP_VM_CALIBRATION_ONLY_ENABLE APP_ZONE2_NAV_ENABLE APP_ZONE2_MISSION_ENABLE VEHICLE_INTEGRATED_ENABLE\n'
    if mode == 3:
        text += '#include "app_entry.h"\nTURN_VALUES APP_TURN90_90_ACTIVE APP_TURN_BRAKE_TRIAL_DPS\n'
    source = out / 'config_probe.c'
    source.write_text(text, encoding='ascii')
    proc = subprocess.run(args + [str(source)], capture_output=True)
    assert proc.returncode == 0, proc.stderr.decode(errors='replace')
    result = proc.stdout.decode(errors='replace')
    values = re.search(r'^CONFIG_VALUES (.+)$', result, re.M)[1]
    if mode == 3:
        for relative in ['CHASSIS/USER/app_motion_runtime.c', 'CHASSIS/USER/app_entry.c']:
            proc = subprocess.run(args + [str(tree / relative)], capture_output=True)
            assert proc.returncode == 0, proc.stderr.decode(errors='replace')
            dest = out / ('pp_%s_%s_%s.txt' % (direction, offline, Path(relative).stem))
            dest.write_bytes(proc.stdout)
            actual = re.sub(r'\s+', '', proc.stdout.decode(errors='replace'))
            if 'app_motion' in relative:
                assert 'g_debug_plan_id=%dU?21U:20U;' % direction in actual
            else:
                assert 'if(id!=(%dU?21U:20U))return0U;' % direction in actual
    return values

for mode in range(3):
    for offline in [0, 1]:
        expected = preprocess(mode, 0, offline, old=True)
        actual = preprocess(mode, 0, offline)
        assert actual == expected, (mode, offline, expected, actual)
        report['preprocess'].append({'mode': mode, 'offline': offline, 'mapping_unchanged': True, 'values': actual})
for direction in [0, 1]:
    for offline in [0, 1]:
        values = preprocess(3, direction, offline)
        report['preprocess'].append({'mode': 3, 'direction': direction, 'offline': offline, 'values': values, 'default_and_plan_filter_match': True})
        target = 'Vehicle_Offline' if offline else 'Vehicle_Integrated'
        log = out / ('build_direction%d_%s.log' % (direction, target))
        proc = subprocess.run([str(uv4), '-r', str(project), '-t', target, '-o', str(log)], capture_output=True, timeout=360, creationflags=subprocess.CREATE_NO_WINDOW)
        content = log.read_bytes().decode('gbk', errors='replace')
        summary = re.search(r'" - (\d+) Error\(s\), (\d+) Warning\(s\)\.', content)
        result = {'direction': direction, 'target': target, 'return_code': proc.returncode, 'log': str(log), 'errors': int(summary[1]) if summary else None, 'warnings': int(summary[2]) if summary else None}
        report['builds'].append(result)
        (out / 'verification.json').write_text(json.dumps(report, indent=2), encoding='utf-8')
        print(json.dumps(result), flush=True)
        assert summary and int(summary[1]) == 0 and 'Target not created.' not in content
assert parameter.read_bytes() == original
report['source_sha256'] = {str(f.relative_to(fw)): hashlib.sha256(f.read_bytes()).hexdigest() for f in [parameter, fw/'CHASSIS/USER/app_entry.c', fw/'CHASSIS/USER/app_motion_runtime.c', fw/'CHASSIS/USER/turn_debug_link_support.c', fw/'gongxun-f407.uvprojx']}
report['main_axf_sha256'] = hashlib.sha256((fw/'Objects/Vehicle_Integrated/Vehicle_Integrated.axf').read_bytes()).hexdigest()
assert report['main_axf_sha256'] == '55064a37d4392a2d8cd474caa6dbc1cbbd06114339f2732575e9f4b079768d5e'
(tree / 'USER/vehicle_parameters.h').write_bytes(original)
(out / 'verification.json').write_text(json.dumps(report, indent=2), encoding='utf-8')
