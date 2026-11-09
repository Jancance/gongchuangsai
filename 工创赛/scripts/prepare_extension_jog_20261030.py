"""构建并安装节点6独立低速小步候选；不烧录、不访问硬件。"""
from pathlib import Path
import hashlib
import json
import re
import shutil
import subprocess
import sys
import xml.etree.ElementTree as ET

ROOT = Path(__file__).resolve().parents[1]
ACTUAL = Path('D:/新建文件夹4/工创赛/代码/stm32/3.stm32f407工程')
STAGE = ROOT/'tmp/extension_jog_20261030/project'
EV = ROOT/'evidence/p2-arm/2026-10-30/extension-jog'
ASSETS = ROOT/'output/extension_jog_20261030'
FILES = ['HARDWARE/EXTENSION/extension.c', 'HARDWARE/EXTENSION/extension.h',
         'HARDWARE/EXTENSION/extension_config.h', 'SYSTEM/CAN/can.c',
         'SYSTEM/CAN/can.h', 'USER/main.c', 'gongxun-f407.uvprojx']


def sha(path):
    return hashlib.sha256(path.read_bytes()).hexdigest() if path.exists() else None


def save(name, data):
    (EV/name).write_text(json.dumps(data, ensure_ascii=False, indent=2)+'\n', encoding='utf-8')


def one(data, old, new):
    assert data.count(old) == 1, old[:100]
    return data.replace(old, new, 1)


def function(data, name):
    start = data.index(('uint32_t '+name+'(').encode())
    brace = data.index(b'{', start)
    depth = 1
    end = brace+1
    while depth:
        if data[end] == 123: depth += 1
        elif data[end] == 125: depth -= 1
        end += 1
    return data[start:end]


def build(folder, log):
    p = subprocess.run(['C:/Keil_v5/UV4/UV4.exe', '-b', str(folder/'gongxun-f407.uvprojx'),
                        '-j0', '-o', str(log)], capture_output=True, timeout=120)
    output = log.read_text(encoding='utf-8', errors='replace')
    assert p.returncode == 0 and re.search(r'0 Error\(s\), 0 Warning\(s\)', output), output[-5000:]
    print(output[-600:], flush=True)


if '--install' in sys.argv:
    m = json.loads((EV/'manifest.json').read_text(encoding='utf-8'))
    assert m['host_pass'] and m['stage_build_pass']
    for entry in m['files']:
        assert sha(ACTUAL/entry['file']) == entry['before'], entry['file']
        assert sha(STAGE/entry['file']) == entry['after'], entry['file']
    for name, expected in m['protected'].items():
        assert sha(ACTUAL/name) == expected, name
    for entry in m['files']:
        shutil.copy2(STAGE/entry['file'], ACTUAL/entry['file'])
    m.update(installed=True, actual_build_pass=False)
    save('installed.json', m)
    build(ACTUAL, EV/'actual-build.log')
    for entry in m['files']:
        assert sha(ACTUAL/entry['file']) == entry['after'], entry['file']
    for name, expected in m['protected'].items():
        assert sha(ACTUAL/name) == expected, name
    m['actual_build_pass'] = True
    save('installed.json', m)
    print('Node6 candidate installed; no flashing or hardware commands.', flush=True)
    raise SystemExit(0)

if '--build' in sys.argv:
    m = json.loads((EV/'manifest.json').read_text(encoding='utf-8'))
    assert (EV/'host-review.json').exists()
    review = json.loads((EV/'host-review.json').read_text(encoding='utf-8'))
    assert review['passed']
    for entry in m['files']:
        assert sha(STAGE/entry['file']) == entry['after'], entry['file']
        assert review['source_sha256'][entry['file']] == entry['after'], entry['file']
    m['host_pass'] = True
    build(STAGE, EV/'stage-build.log')
    m['stage_build_pass'] = True
    save('manifest.json', m)
    raise SystemExit(0)

assert not STAGE.exists()
EV.mkdir(parents=True, exist_ok=True)
shutil.copytree(ACTUAL, STAGE, ignore=shutil.ignore_patterns('Objects', 'Listings', '.git', '.vscode', '*.uvguix.*'))
protected = {}
for path in ACTUAL.rglob('*'):
    if path.is_file() and path.suffix.lower() in ('.c', '.h', '.uvprojx'):
        rel = path.relative_to(ACTUAL).as_posix()
        if rel not in FILES:
            protected[rel] = sha(path)
for name in FILES:
    if (ACTUAL/name).exists():
        backup = EV/'original'/name
        backup.parent.mkdir(parents=True, exist_ok=True)
        assert not backup.exists()
        shutil.copy2(ACTUAL/name, backup)

module = STAGE/'HARDWARE/EXTENSION'
shutil.copy2(ASSETS/'extension_config.h', module/'extension_config.h')
p = module/'extension.h'
t = p.read_text(encoding='utf-8')
t = one(t, '/* 28夹爪伸缩轴，地址6；本模块仅查询，不含任何运动或参数写入。 */',
        '/* 28伸缩轴，地址6；只读识别及人工单次低速小步，均不写驱动参数。 */')
decl = '''/* 请求默认0；1方向0小步，2方向1小步，3停止；伸/缩方向待实物确认。 */
extern volatile uint32_t g_extension_motion_request;
/* 0空闲，1至6准备/运行，7反馈完成，8人工停止，9失败锁存。 */
extern volatile uint32_t g_extension_motion_state;
/* 1完成，2人工停止，3超时，4通信/选项，5驱动状态，6互锁，7服务间隔超限。 */
extern volatile uint32_t g_extension_motion_result;
extern volatile uint32_t g_extension_enable_result;
extern volatile uint32_t g_extension_move_result;
extern volatile uint32_t g_extension_stop_result;
extern volatile uint32_t g_extension_motion_options_result;
extern volatile uint32_t g_extension_flags;
extern volatile int32_t g_extension_speed_rpm;
extern volatile int64_t g_extension_delta_raw;
extern volatile uint32_t g_extension_motion_elapsed_ms;
extern volatile uint32_t g_extension_motion_direction;
extern CanRxMsg g_extension_motion_reply;
extern CanRxMsg g_extension_control_reply;
uint32_t Extension_MotionIsBusy(void);
void Extension_MotionService(uint32_t ready, uint32_t other_busy);
'''
t = one(t, '#endif', decl+'#endif')
p.write_text(t, encoding='utf-8')
p = module/'extension.c'
t = p.read_text(encoding='utf-8')
t = one(t, '#include "can.h"', '#include "can.h"\n#include "extension_config.h"\n#include "FreeRTOS.h"\n#include "task.h"')
t = one(t, '    g_extension_read_request = 0U;\n', '''    g_extension_read_request = 0U;
    /* 运动请求或动作占用时不插入身份读取，不排队。 */
    if (Extension_MotionIsBusy() || g_extension_motion_request != 0U) return;
''')
t += (ASSETS/'motion.inc').read_text(encoding='utf-8')
p.write_text(t, encoding='utf-8')

p = STAGE/'SYSTEM/CAN/can.c'
t = p.read_bytes()
legacy = function(t, 'CAN1_LiftExchange')
transport = legacy.replace(b'CAN1_LiftExchange', b'CAN1_ExtensionExchange')
transport = transport.replace(b'CAN1_ExchangeSingle(5U,', b'CAN1_ExchangeSingle(6U,')
transport = transport.replace(b'0x0500U', b'0x0600U')
transport = one(transport, b'speed_rpm > 3000U', b'speed_rpm > EXTENSION_JOG_RPM')
transport = one(transport, b'request[4] == 0U', b'(request[4] == 0U || request[4] > EXTENSION_JOG_ACCELERATION)')
transport = one(transport, b'(request[1] == 1U ? 233333U : 5000U)', b'EXTENSION_JOG_PULSES')
assert b'0x0500' not in transport and b'ExchangeSingle(5U' not in transport
t = one(t, b'#include "can.h"', b'#include "can.h"\n#include "extension_config.h"')
t += '\n/* 节点6独立小步传输；不扩大原底盘/升降运动接口的地址或速度范围。 */\n'.encode('utf-8')+transport+b'\n'
assert function(t, 'CAN1_LiftExchange') == legacy
p.write_bytes(t)
p = STAGE/'SYSTEM/CAN/can.h'
t = p.read_bytes()
t = one(t, b'#endif', b'uint32_t CAN1_ExtensionExchange(const uint8_t *request, uint8_t length, CanRxMsg *reply);\n\n#endif')
p.write_bytes(t)

p = STAGE/'USER/main.c'
t = p.read_bytes()
a = t.index(b'        if (ArmPower_IsIsolatedMode() != 0U)')
z = t.index(b'            g_can_task_stack_words = uxTaskGetStackHighWaterMark(NULL);', a)
branch = t[a:z]
newline = b'\r\n' if b'\r\n' in branch else b'\n'
branch = branch.replace(b'\r\n', b'\n')
marker = b'            Lift_Service('
insert = b'''            if (Extension_MotionIsBusy())
            {
                if (g_lift_motion_request != 3U) g_lift_motion_request = 0U;
                if (g_lift_home_request != 2U) g_lift_home_request = 0U;
                g_lift_read_request = 0U;
                g_lift_home_read_request = 0U;
            }
'''
branch = one(branch, marker, insert+marker)
old = b'''                              (LiftHome_IsBusy() || g_lift_motion_request != 0U ||
                               g_lift_home_request != 0U ||
                               (g_lift_motion_state >= 1U && g_lift_motion_state <= 6U)) ? 1U : 0U);'''
new = b'''                              (DM_CAN2_JogIsBusy() || LiftHome_IsBusy() || g_lift_motion_request != 0U ||
                               g_lift_home_request != 0U ||
                               (g_lift_motion_state >= 1U && g_lift_motion_state <= 6U)) ? 1U : 0U);
            Extension_MotionService(((g_can_test_result == 1U) &&
                                     (g_safety_ready == 1U) &&
                                     (g_vm_test_request == 1U) &&
                                     (g_arm_menu_power_state == 1U) &&
                                     (g_lift_state == 4U)) ? 1U : 0U,
                                    (DM_CAN2_JogIsBusy() || LiftHome_IsBusy() ||
                                     g_lift_motion_request != 0U || g_lift_home_request != 0U ||
                                     (g_lift_motion_state >= 1U && g_lift_motion_state <= 6U) ||
                                     (g_extension_state >= 1U && g_extension_state <= 3U)) ? 1U : 0U);'''
branch = one(branch, old, new)
branch = one(branch, b'                            (LiftHome_IsBusy() || g_lift_motion_request != 0U ||',
             b'                            (Extension_MotionIsBusy() || g_extension_motion_request != 0U ||\n                             LiftHome_IsBusy() || g_lift_motion_request != 0U ||')
branch = branch.replace(b'\n', newline)
t = t[:a]+branch+t[z:]
p.write_bytes(t)

p = STAGE/'gongxun-f407.uvprojx'
t = p.read_text(encoding='utf-8')
pattern = r'(<Group>\s*<GroupName>EXTENSION</GroupName>.*?<Files>)(.*?)(</Files>)'
def header(m):
    return m[1]+m[2]+'<File><FileName>extension_config.h</FileName><FileType>5</FileType><FilePath>.\\HARDWARE\\EXTENSION\\extension_config.h</FilePath></File>\n'+m[3]
t, count = re.subn(pattern, header, t, flags=re.S)
assert count == 1
p.write_text(t, encoding='utf-8')
ET.parse(p)
m = dict(files=[dict(file=n, before=sha(ACTUAL/n), after=sha(STAGE/n)) for n in FILES],
         protected=protected, host_pass=False, stage_build_pass=False, installed=False,
         user_clearance_confirmation='在中间，两侧都有余量',
         candidate=dict(node=6, pulses=16, rpm=5, acceleration=20, timeout_ms=3000),
         flash_performed=False, hardware_commands_sent=False)
save('manifest.json', m)
print('Prepared independent node6 candidate at '+str(STAGE), flush=True)
