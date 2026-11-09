"""Prepare and check top-homing direction; never flash or send motor commands."""
from pathlib import Path
import hashlib, json, re, shutil, subprocess, sys

ROOT = Path(__file__).resolve().parents[1]
ACTUAL = Path('D:/新建文件夹4/工创赛/代码/stm32/3.stm32f407工程')
STAGE = ROOT/'tmp/lift_top_20261028/project'
EV = ROOT/'evidence/p2-arm/2026-10-28/lift-top'
FILES = ['HARDWARE/LIFT/lift_home.c', 'HARDWARE/LIFT/lift_home.h']

def sha(p):
    return hashlib.sha256(p.read_bytes()).hexdigest()

def build(folder, log):
    r = subprocess.run(['C:/Keil_v5/UV4/UV4.exe', '-b', str(folder/'gongxun-f407.uvprojx'), '-j0', '-o', str(log)], capture_output=True)
    text = log.read_text(encoding='utf-8', errors='replace')
    assert r.returncode == 0 and re.search(r'0 Error\(s\), 0 Warning\(s\)', text), text
    print(text[-450:])

if '--install' in sys.argv:
    m = json.loads((EV/'manifest.json').read_text(encoding='utf-8'))
    assert m['host_pass'] and m['build_pass']
    for f in m['files']:
        assert sha(ACTUAL/f['file']) == f['before']
        assert sha(STAGE/f['file']) == f['after']
    for f in m['files']:
        shutil.copy2(STAGE/f['file'], ACTUAL/f['file'])
        assert sha(ACTUAL/f['file']) == f['after']
    build(ACTUAL, EV/'actual-build.log')
    m['installed'] = True
    (EV/'installed.json').write_text(json.dumps(m, ensure_ascii=False, indent=2), encoding='utf-8')
else:
    assert not STAGE.exists()
    EV.mkdir(parents=True, exist_ok=True)
    shutil.copytree(ACTUAL, STAGE)
    m = {'files': [], 'estimated_top_gap_mm': 8, 'gap_source': 'user estimate', 'flash_performed': False, 'hardware_motion_executed': False}
    for name in FILES:
        src = ACTUAL/name
        backup = EV/'original'/name
        backup.parent.mkdir(parents=True, exist_ok=True)
        shutil.copy2(src, backup)
        data = src.read_bytes()
        if name.endswith('.c'):
            old = b'g_lift_home_params.direction != 0U'
            assert data.count(old) == 1
            data = data.replace(old, b'g_lift_home_params.direction != 1U')
        else:
            old = '手动向下碰撞回零'.encode()
            assert data.count(old) == 1
            data = data.replace(old, '手动向上碰撞回零（顶部，参数方向须为1）'.encode())
        (STAGE/name).write_bytes(data)
        m['files'].append({'file': name, 'before': sha(src), 'after': sha(STAGE/name)})
    host = ROOT/'tmp/lift_top_20261028/host-check'
    shutil.copytree(ROOT/'tmp/lift_sensorless_20261025/host-check', host)
    p = host/'test.c'
    t = p.read_text(encoding='utf-8').replace('raw[17]={0x22,0,0,0,30', 'raw[17]={0x22,0,1,0,30')
    t = t.replace('home_config_changed=0;', 'home_config_changed=0, home_wrong_direction=0;', 1)
    t = t.replace('if(home_config_changed)', 'if(home_wrong_direction) out[2]=0;\n    if(home_config_changed)', 1)
    marker = 'assert(home_starts==0); /* Existing tests must not start a home. */'
    assert marker in t
    t = t.replace(marker, marker + '''
    ready();g_lift_home_request=2;step(1);home_wrong_direction=1;
    { unsigned before=home_starts;g_lift_home_request=1;step(30);
      assert(home_starts==before && g_lift_home_state==9 && g_lift_home_result==7); }
    home_wrong_direction=0;
''')
    p.write_text(t, encoding='utf-8')
    module = STAGE/'HARDWARE/LIFT'
    args = ['C:/mingw/mingw64/bin/gcc.exe', '-std=c99', '-Wall', '-Wextra', '-Werror', '-I'+str(host), '-I'+str(module), str(p)]
    args += [str(module/f) for f in ['lift.c', 'lift_home.c', 'lift_protocol.c']]
    r = subprocess.run(args+['-o', str(host/'test.exe')], capture_output=True, text=True)
    assert r.returncode == 0, r.stdout+r.stderr
    r = subprocess.run([str(host/'test.exe')], capture_output=True, text=True)
    (EV/'host-check.log').write_text(r.stdout+r.stderr, encoding='utf-8')
    assert r.returncode == 0, r.stdout+r.stderr
    print(r.stdout)
    build(STAGE, EV/'stage-build.log')
    m.update(host_pass=True, build_pass=True)
    (EV/'manifest.json').write_text(json.dumps(m, ensure_ascii=False, indent=2), encoding='utf-8')
