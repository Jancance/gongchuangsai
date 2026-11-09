"""只查新增毫米入口及它与停止/故障交互；不重跑未改CAN旧套件。"""
from pathlib import Path
import hashlib
import json
import shutil
import subprocess

ROOT = Path(__file__).resolve().parents[1]
STAGE = ROOT / 'tmp/extension_mm_entry_20261101/project'
HOST = ROOT / 'tmp/extension_mm_entry_20261101/host-check'
EV = ROOT / 'evidence/p2-arm/2026-11-01/extension-mm-entry'
OLD = ROOT / 'tmp/extension_stall_confirm_20261101/host-check'
FIXTURES = EV / 'fixtures'
HOST.mkdir(parents=True, exist_ok=True)
FIXTURES.mkdir(parents=True, exist_ok=True)
for name in ('stm32f4xx_can.h', 'FreeRTOS.h', 'task.h', 'can.h', 'motion-model.c'):
    if not (FIXTURES / name).exists():
        shutil.copy2(OLD / name, FIXTURES / name)
    if name.endswith('.h'):
        shutil.copy2(FIXTURES / name, HOST / name)


def one(text, old, new):
    assert text.count(old) == 1, old
    return text.replace(old, new, 1)


model = (FIXTURES / 'motion-model.c').read_text(encoding='utf-8')
model = one(model, '#include <string.h>', '#include <string.h>\n#include <math.h>')
model = one(model, 'static TickType_t now;', 'static TickType_t now;\nstatic uint32_t expected_step=1600;')
model = one(model, 'b[1]==(is("negative")?1:0)', 'b[1]==((is("negative")||is("mm_extend"))?1:0)')
model = one(model, 'assert(b[2]==0&&b[3]==5&&b[4]==20&&b[5]==0&&b[6]==0&&b[7]==6&&b[8]==64);',
            'assert(b[2]==0&&b[3]==5&&b[4]==20);\n assert((((uint32_t)b[5]<<24)|((uint32_t)b[6]<<16)|((uint32_t)b[7]<<8)|b[8])==expected_step);')
setup = '''
 if(is("mm_default"))assert(g_extension_use_mm==1&&g_extension_distance_mm==10.0f);
 if(strncmp(scenario,"mm_",3)==0){g_extension_use_mm=1;g_extension_distance_mm=10.0f;expected_step=320;}
 else g_extension_use_mm=0;
 if(is("mm_default")){assert(g_extension_use_mm==1&&g_extension_distance_mm==10.0f);}
 if(is("mm_25")){g_extension_distance_mm=25.0f;expected_step=800;}
 if(is("mm_max")){g_extension_distance_mm=50.0f;expected_step=1600;}
 if(is("mm_round")){g_extension_distance_mm=0.1f;expected_step=3;}
 if(is("mm_min_round")){g_extension_distance_mm=0.015625f;expected_step=1;}
 if(is("mm_zero"))g_extension_distance_mm=0.0f;
 if(is("mm_negative"))g_extension_distance_mm=-10.0f;
 if(is("mm_nan"))g_extension_distance_mm=NAN;
 if(is("mm_inf"))g_extension_distance_mm=INFINITY;
 if(is("mm_too_far"))g_extension_distance_mm=50.1f;
 if(is("mm_too_small"))g_extension_distance_mm=0.001f;
 if(is("mm_bad_units"))g_extension_use_mm=2;
 if(is("raw_fallback"))g_extension_distance_mm=NAN;
'''
assert model.count('assert(argc==2);scenario=argv[1];') == 2
model = model.replace('assert(argc==2);scenario=argv[1];', 'assert(argc==2);scenario=argv[1];\n' + setup, 1)
model = one(model, 'g_extension_motion_request=is("negative")?2:1;',
            'g_extension_motion_request=(is("negative")||is("mm_extend"))?2:1;')
model = one(model, 'r->Data[0]=0x3A;r->Data[1]=3;',
            'r->Data[0]=0x3A;r->Data[1]=3;\n'
            ' if(is("mm_fault")&&moved)r->Data[1]=5;\n'
            ' if(is("mm_protect")&&moved)r->Data[1]=9;\n'
            ' if(is("mm_timeout")&&moved)r->Data[1]=1;')
model = one(model, 'busy=is("busy")||(is("busy_during")&&moved);',
            'busy=is("busy")||(is("busy_during")&&moved)||is("mm_interlock");\n'
            '  if(is("mm_power_loss")&&moved)ready=0;\n'
            '  if(is("mm_busy_edit")&&Extension_MotionIsBusy()){g_extension_distance_mm=40.0f;g_extension_use_mm=0;g_extension_target_pulses=1599;}\n'
            '  if(is("mm_busy_invalid_edit")&&Extension_MotionIsBusy())g_extension_distance_mm=NAN;\n'
            '  if(is("mm_stop_override")&&moved){g_extension_use_mm=2;g_extension_distance_mm=NAN;g_extension_motion_request=3;}')
assertions = '''
 if(is("mm_zero")||is("mm_negative")||is("mm_nan")||is("mm_inf")||is("mm_too_far")||is("mm_too_small")||is("mm_bad_units"))expected=8;
 if(is("mm_fault")||is("mm_protect"))expected=5;
 if(is("mm_timeout"))expected=3;
 if(is("mm_interlock")||is("mm_power_loss"))expected=6;
 if(is("mm_stop_override"))expected=2;
 assert(g_extension_motion_result==expected);
 if(expected==8){assert(moves==0&&stops==0&&reads==0&&g_extension_target_pulses==0);}
 if(expected==1){assert(g_extension_target_pulses==expected_step||is("mm_busy_edit"));}
 if(is("mm_busy_edit")||is("mm_busy_invalid_edit")){assert(moves==1&&g_extension_target_mm==10.0f);}
 if(is("mm_fault")){assert(stops==1&&g_extension_stall_max_ms>=2000);}
 if(is("mm_protect")){assert(stops==1&&g_extension_stall_events==0);}
'''
model = one(model, 'assert(g_extension_motion_result==expected);', assertions)
(HOST / 'mm-model.c').write_text(model, encoding='utf-8')
module = STAGE / 'HARDWARE/EXTENSION'
exe = HOST / 'mm-model.exe'
compile_cmd = ['C:/mingw/mingw64/bin/gcc.exe', '-std=c99', '-Wall', '-Wextra', '-Werror',
               '-I' + str(HOST), '-I' + str(module), str(module / 'extension.c'), str(HOST / 'mm-model.c'), '-o', str(exe)]


def run(command):
    proc = subprocess.run(command, capture_output=True, text=True, timeout=30)
    if proc.returncode != 0:
        raise RuntimeError(repr(command) + '\n' + proc.stdout + '\n' + proc.stderr)
    return proc.stdout.strip()


run(compile_cmd)
cases = ['mm_default', 'mm_extend', 'mm_25', 'mm_max', 'mm_round', 'mm_min_round',
         'mm_zero', 'mm_negative', 'mm_nan', 'mm_inf', 'mm_too_far', 'mm_too_small',
         'mm_bad_units', 'raw_fallback', 'mm_busy_edit', 'mm_busy_invalid_edit',
         'mm_stop_override', 'mm_fault', 'mm_protect', 'mm_timeout', 'mm_interlock', 'mm_power_loss']
records = [{'case': case, 'output': run([str(exe), case])} for case in cases]
m = json.loads((EV / 'manifest.json').read_text(encoding='utf-8'))
for relative, digest in m['protected'].items():
    assert hashlib.sha256((STAGE / relative).read_bytes()).hexdigest() == digest, relative
report = {'passed': True, 'case_count': len(records), 'cases': records,
          'changed_source_sha256': {e['file']: hashlib.sha256((STAGE / e['file']).read_bytes()).hexdigest() for e in m['files']},
          'scope': 'New mm conversion, rounding/bounds, private target latching, raw fallback and safety interaction only. Unchanged CAN/arbitration/home/carrier suites not repeated; sources hash protected.',
          'synthetic_only': True, 'hardware_acceptance': False}
(EV / 'host-review.json').write_text(json.dumps(report, ensure_ascii=False, indent=2) + '\n', encoding='utf-8')
(EV / 'host-check.log').write_text('\n'.join(row['output'] for row in records) + '\n', encoding='utf-8')
print(json.dumps({'passed': True, 'new_cases': len(records), 'protected_files': len(m['protected'])}))
