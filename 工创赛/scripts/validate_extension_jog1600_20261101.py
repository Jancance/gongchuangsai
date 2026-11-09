"""复用原合成夹具，检查真实伸缩C状态机与CAN边界；不访问硬件。"""
from pathlib import Path
import ast
import hashlib
import json
import subprocess

ROOT = Path(__file__).resolve().parents[1]
STAGE = ROOT / 'tmp/extension_jog1600_20261101/project'
HOST = ROOT / 'tmp/extension_jog1600_20261101/host-check'
EV = ROOT / 'evidence/p2-arm/2026-11-01/extension-jog1600'
GCC = 'C:/mingw/mingw64/bin/gcc.exe'
HOST.mkdir(parents=True, exist_ok=True)


def replace_once(text, old, new):
    assert text.count(old) == 1, old
    return text.replace(old, new, 1)


def function(data, name):
    start = data.index(('uint32_t ' + name + '(').encode())
    brace = data.index(b'{', start)
    depth, end = 1, brace + 1
    while depth:
        if data[end] == 123:
            depth += 1
        elif data[end] == 125:
            depth -= 1
        end += 1
    return data[start:end]


def run(cmd):
    result = subprocess.run(cmd, capture_output=True, text=True, timeout=30)
    if result.returncode != 0:
        raise RuntimeError(repr(cmd) + '\n' + result.stdout + '\n' + result.stderr)
    return result.stdout.strip()


legacy = ast.parse((ROOT / 'scripts/validate_extension_jog_20261030.py').read_text(encoding='utf-8'))
templates = {}
headers = {}
for node in legacy.body:
    if isinstance(node, ast.Assign) and isinstance(node.targets[0], ast.Name):
        if node.targets[0].id in ('MODEL', 'TRANSPORT'):
            templates[node.targets[0].id] = ast.literal_eval(node.value)
    if isinstance(node, ast.Expr) and isinstance(node.value, ast.Call):
        call = node.value
        if isinstance(call.func, ast.Attribute) and call.func.attr == 'write_text':
            target = call.func.value
            if isinstance(target, ast.BinOp) and isinstance(target.op, ast.Div):
                if isinstance(target.left, ast.Name) and target.left.id == 'HOST':
                    filename = ast.literal_eval(target.right)
                    if filename in ('stm32f4xx_can.h', 'FreeRTOS.h', 'task.h', 'can.h'):
                        headers[filename] = ast.literal_eval(call.args[0])
assert len(headers) == 4
for filename, content in headers.items():
    (HOST / filename).write_text(content, encoding='utf-8')

model = templates['MODEL'].replace('__BRANCH__', '')
model = replace_once(model, 'b[7]==0&&b[8]==16', 'b[7]==6&&b[8]==64')
model = replace_once(model, 'for(i=0;i<450;i++)', 'for(i=0;i<1250;i++)')
model = replace_once(model, 'if(is("fault")&&moved)r->Data[1]=7;',
                     'if(is("fault")&&moved)r->Data[1]=5;\n if(is("protect")&&moved)r->Data[1]=9;')
model = replace_once(model, 'if(is("no_arrival")&&moved)r->Data[1]=1;',
                     'if((is("no_arrival")||(is("late_success")&&now<9500))&&moved)r->Data[1]=1;')
model = replace_once(model, 'if(is("fault")||is("disable")||is("pre_moving"))expected=5;',
                     'if(is("fault")||is("protect")||is("disable")||is("pre_moving"))expected=5;')
model = replace_once(model, 'assert(g_extension_motion_result==expected);',
                     'assert(g_extension_motion_result==expected);\n'
                     ' if(expected==3)assert(g_extension_motion_elapsed_ms>=10000&&g_extension_motion_elapsed_ms<=10010);\n'
                     ' if(is("late_success"))assert(g_extension_motion_elapsed_ms>=9500&&g_extension_motion_elapsed_ms<10000);\n'
                     ' if(is("fault")||is("protect"))assert(moves==1&&stops==1);')
(HOST / 'motion-model.c').write_text(model, encoding='utf-8')
module = STAGE / 'HARDWARE/EXTENSION'
common = [GCC, '-std=c99', '-Wall', '-Wextra', '-Werror', '-I' + str(HOST), '-I' + str(module)]
exe = HOST / 'motion.exe'
run(common + [str(module / 'extension.c'), str(HOST / 'motion-model.c'), '-o', str(exe)])
records = []
motion_cases = ['positive', 'negative', 'enable', 'wrap', 'signed', 'repeat', 'edit_direction',
                'ack_only', 'no_stop', 'no_arrival', 'bad_options', 'pre_moving', 'fault', 'protect',
                'disable', 'read_fail', 'move_fail', 'not_ready', 'busy', 'power_loss', 'busy_during',
                'gap', 'abort', 'stop_fail', 'readonly', 'readonly_during', 'late_success']
for case in motion_cases:
    records.append({'group': 'motion', 'case': case, 'output': run([str(exe), case])})

can_bytes = (STAGE / 'SYSTEM/CAN/can.c').read_bytes()
transport = templates['TRANSPORT'].replace('__FUNCTION__', function(can_bytes, 'CAN1_ExtensionExchange').decode('utf-8'))
for old, new in [
    ('0,0,0,16,2,0,0x6B', '0,0,6,64,2,0,0x6B'),
    ('if(is("zero_pulses"))q[8]=0;', 'if(is("zero_pulses"))memset(&q[5],0,4);'),
    ('q[8]=17', 'q[8]=65'),
    ('first[8]={0xFD,0,0,5,20,0,0,0}', 'first[8]={0xFD,0,0,5,20,0,0,6}'),
    ('second[5]={0xFD,16,2,0,0x6B}', 'second[5]={0xFD,64,2,0,0x6B}'),
]:
    transport = replace_once(transport, old, new)
(HOST / 'transport.c').write_text(transport, encoding='utf-8')
exe = HOST / 'transport.exe'
run(common + [str(HOST / 'transport.c'), '-o', str(exe)])
transport_cases = ['valid', 'notice', 'enable', 'stop', 'bad_small_ack', 'unlock', 'null', 'short',
                   'bad_direction', 'too_fast', 'high_speed_byte', 'zero_speed', 'zero_pulses',
                   'too_many_pulses', 'high_pulse_byte', 'bad_mode', 'synchronous', 'bad_checksum',
                   'too_much_accel', 'zero_accel', 'wrong_node', 'wrong_ide', 'wrong_rtr', 'no_reply',
                   'bad_length', 'bad_tail', 'bad_ack', 'tx_fail', 'tx_timeout', 'no_mailbox', 'init_fail']
for case in transport_cases:
    records.append({'group': 'transport', 'case': case, 'output': run([str(exe), case])})
exe = HOST / 'transport-disabled.exe'
run(common + ['-DAPP_ARM_LIFT_TEST=0', str(HOST / 'transport.c'), '-o', str(exe)])
records.append({'group': 'transport', 'case': 'disabled', 'output': run([str(exe), 'disabled'])})

manifest = json.loads((EV / 'manifest.json').read_text(encoding='utf-8'))
for relative, digest in manifest['protected'].items():
    assert hashlib.sha256((STAGE / relative).read_bytes()).hexdigest() == digest, relative
config = module / 'extension_config.h'
report = {
    'passed': True, 'case_count': len(records), 'cases': records,
    'expected_pulses': 1600, 'expected_timeout_ms': 10000,
    'config_sha256': hashlib.sha256(config.read_bytes()).hexdigest(),
    'source_sha256': {name: hashlib.sha256((STAGE / name).read_bytes()).hexdigest()
                      for name in ['HARDWARE/EXTENSION/extension.c', 'SYSTEM/CAN/can.c', 'USER/main.c']},
    'protected_source_count': len(manifest['protected']),
    'scope': 'Actual ordinary-motion C state machine and CAN transport; unchanged task arbitration protected by hashes, not re-run.',
    'synthetic_only': True, 'hardware_acceptance': False,
}
(EV / 'host-review.json').write_text(json.dumps(report, ensure_ascii=False, indent=2) + '\n', encoding='utf-8')
(EV / 'host-check.log').write_text('\n'.join(r['output'] for r in records) + '\n', encoding='utf-8')
print(json.dumps({'passed': True, 'cases': len(records), 'motion': len(motion_cases),
                  'transport': len(transport_cases) + 1, 'protected_source_count': len(manifest['protected'])}))
