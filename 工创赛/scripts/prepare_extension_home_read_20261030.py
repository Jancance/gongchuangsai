"""节点6回零参数只读入口：准备、编译、受控安装；不含回零指令。"""
from pathlib import Path
import hashlib
import json
import re
import shutil
import subprocess
import sys

ROOT = Path(__file__).resolve().parents[1]
ACTUAL = Path('D:/新建文件夹4/工创赛/代码/stm32/3.stm32f407工程')
STAGE = ROOT/'tmp/extension_home_read_20261030/project'
EV = ROOT/'evidence/p2-arm/2026-10-30/extension-home-read'
FILES = ['HARDWARE/EXTENSION/extension.c', 'HARDWARE/EXTENSION/extension.h',
         'SYSTEM/CAN/can.c', 'SYSTEM/CAN/can.h', 'USER/main.c']


def sha(p):
    return hashlib.sha256(p.read_bytes()).hexdigest()


def save(name, data):
    (EV/name).write_text(json.dumps(data, ensure_ascii=False, indent=2)+'\n', encoding='utf-8')


def function(data, name):
    a = data.index(('uint32_t '+name+'(').encode())
    z = data.index(b'{', a)+1
    depth = 1
    while depth:
        if data[z] == 123: depth += 1
        elif data[z] == 125: depth -= 1
        z += 1
    return data[a:z]


def verify(folder, m):
    for name, digest in m['protected'].items():
        assert sha(folder/name) == digest, name


def build(folder, log):
    p = subprocess.run(['C:/Keil_v5/UV4/UV4.exe', '-b', str(folder/'gongxun-f407.uvprojx'),
                        '-j0', '-o', str(log)], capture_output=True, timeout=120)
    output = log.read_text(encoding='utf-8', errors='replace')
    assert p.returncode == 0 and re.search(r'0 Error\(s\), 0 Warning\(s\)', output), output[-5000:]
    print(output[-550:], flush=True)


if '--build' in sys.argv or '--install' in sys.argv:
    m = json.loads((EV/'manifest.json').read_text(encoding='utf-8'))
    verify(STAGE, m)
    for e in m['files']:
        assert sha(STAGE/e['file']) == e['after'], e['file']
    if '--build' in sys.argv:
        r = json.loads((EV/'host-review.json').read_text(encoding='utf-8'))
        assert r['passed']
        h = json.loads((EV/'home-read-review.json').read_text(encoding='utf-8'))
        assert h['passed']
        for e in m['files']:
            assert r['source_sha256'][e['file']] == e['after'], e['file']
            assert h['source_sha256'][e['file']] == e['after'], e['file']
        m['host_pass'] = True
        build(STAGE, EV/'stage-build.log')
        verify(STAGE, m)
        m['stage_build_pass'] = True
        save('manifest.json', m)
    else:
        assert m['host_pass'] and m['stage_build_pass']
        verify(ACTUAL, m)
        for e in m['files']:
            assert sha(ACTUAL/e['file']) == e['before'], e['file']
        for e in m['files']:
            shutil.copy2(STAGE/e['file'], ACTUAL/e['file'])
        m.update(installed=True, actual_build_pass=False)
        save('installed.json', m)
        build(ACTUAL, EV/'actual-build.log')
        verify(ACTUAL, m)
        for e in m['files']:
            assert sha(ACTUAL/e['file']) == e['after'], e['file']
        m['actual_build_pass'] = True
        save('installed.json', m)
        print('Node6 home-parameter reader installed; no flash or home/motion commands.', flush=True)
    raise SystemExit(0)

assert not STAGE.exists()
EV.mkdir(parents=True, exist_ok=True)
shutil.copytree(ACTUAL, STAGE, ignore=shutil.ignore_patterns('Objects', 'Listings', '.git', '.vscode', '*.uvguix.*'))
for name in FILES:
    backup = EV/'original'/name
    backup.parent.mkdir(parents=True, exist_ok=True)
    shutil.copy2(ACTUAL/name, backup)
protected = {p.relative_to(ACTUAL).as_posix(): sha(p) for p in ACTUAL.rglob('*')
             if p.is_file() and p.suffix.lower() in ('.c', '.h', '.uvprojx')
             and p.relative_to(ACTUAL).as_posix() not in FILES}

p = STAGE/'HARDWARE/EXTENSION/extension.h'
t = p.read_text(encoding='utf-8')
assert t.count('#endif') == 1
decl = '''/* 只读6号回零参数，默认请求0，写1读取一次，不触发回零或写参数。 */
typedef struct
{
    uint32_t mode, direction, rpm, timeout_ms;
    uint32_t detect_rpm, detect_current_ma, detect_time_ms, auto_home;
} ExtensionHomeParameters;
extern volatile uint32_t g_extension_home_read_request;
/* 1读出并解释成功，6格式/数据范围错误，7未就绪/机构忙，其他同CAN结果。 */
extern volatile uint32_t g_extension_home_read_result;
extern ExtensionHomeParameters g_extension_home_params;
extern uint8_t g_extension_home_raw[17];
extern CanRxMsg g_extension_home_replies[3];
void Extension_HomeReadService(uint32_t ready, uint32_t other_busy);
'''
t = t.replace('#endif', decl+'#endif')
t = t.replace('3停止；伸/缩方向待实物确认。', '3停止；已确认1缩回、2伸出。')
p.write_text(t, encoding='utf-8')
p = STAGE/'HARDWARE/EXTENSION/extension.c'
original_extension = p.read_bytes()
p.write_bytes(original_extension+(ROOT/'output/extension_home_read_20261030/home_read.inc').read_bytes())
p = STAGE/'SYSTEM/CAN/can.c'
original_can = p.read_bytes()
reader = function(original_can, 'CAN1_ReadLiftHomeParameters')
reader = reader.replace(b'CAN1_ReadLiftHomeParameters', b'CAN1_ReadExtensionHomeParameters')
reader = reader.replace(b'0x0500U', b'0x0600U').replace(b'(rx.ExtId >> 8) == 5U', b'(rx.ExtId >> 8) == 6U')
assert b'0x0500' not in reader and b'== 5U' not in reader
p.write_bytes(original_can+'\n/* 只读6号回零参数22，严格拼接0x600/601/602，不提供回零或写参数。 */\n'.encode('utf-8')+reader+b'\n')
p = STAGE/'SYSTEM/CAN/can.h'
t = p.read_bytes()
assert t.count(b'#endif') == 1
t = t.replace(b'#endif', b'uint32_t CAN1_ReadExtensionHomeParameters(uint8_t out[17], CanRxMsg replies[3]);\n\n#endif')
p.write_bytes(t)
p = STAGE/'USER/main.c'
t = p.read_bytes()
newline = b'\r\n' if b'\r\n' in t else b'\n'
marker = b'            Extension_MotionService('
assert t.count(marker) == 1
call = b'''            Extension_HomeReadService(((g_can_test_result == 1U) &&
                                       (g_safety_ready == 1U) &&
                                       (g_vm_test_request == 1U) &&
                                       (g_arm_menu_power_state == 1U) &&
                                       (g_lift_state == 4U)) ? 1U : 0U,
                                      (DM_CAN2_JogIsBusy() || LiftHome_IsBusy() ||
                                       g_lift_motion_request != 0U || g_lift_home_request != 0U ||
                                       (g_lift_motion_state >= 1U && g_lift_motion_state <= 6U)) ? 1U : 0U);
'''.replace(b'\n', newline)
t = t.replace(marker, call+marker, 1)
p.write_bytes(t)
save('manifest.json', dict(files=[dict(file=n, before=sha(ACTUAL/n), after=sha(STAGE/n)) for n in FILES],
                         protected=protected, host_pass=False, stage_build_pass=False, installed=False,
                         user_text='没有限位开关，但是可以限位，可以作为原点',
                         mechanical_stop_origin_candidate=True, no_home_command_added=True,
                         flash_performed=False, hardware_commands_sent=False))
print('Prepared node6 read-only 22 parameter reader; existing motion and node5 unchanged.', flush=True)
