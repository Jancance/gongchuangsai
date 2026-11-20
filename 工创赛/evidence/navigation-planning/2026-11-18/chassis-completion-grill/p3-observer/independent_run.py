import argparse
from hashlib import sha256
from pathlib import Path
import json
import re
import shutil
import struct
import subprocess
import sys

parser = argparse.ArgumentParser()
parser.add_argument('tag')
parser.add_argument('--arm-bin', type=Path, default=Path(r'D:\Keil_v5\ARM\arm5_compiler\bin'))
parser.add_argument('--radar-source', type=Path)
args = parser.parse_args()
here = Path(__file__).resolve().parent
root = here.parents[5]
chassis = root / '3.stm32f407工程/CHASSIS'
radar = args.radar_source or (chassis / 'HARDWARE/RADAR_V6')
out = root / '工创赛/tmp/zone2_observer_independent' / args.tag
out.mkdir(parents=True, exist_ok=True)
for name in ('radar_client.c', 'radar_client.h', 'radar_observer.c', 'radar_observer.h',
             'radar_uart_port.h'):
    source = radar / name
    if name == 'radar_uart_port.h' and not source.exists():
        source = chassis / 'HARDWARE/RADAR_V6' / name
    shutil.copyfile(source, out / name)
for name in ('radar_lift.h', 'radar_servo.h', 'radar_servo_config.h'):
    shutil.copyfile(chassis / 'HARDWARE/RADAR_SERVO' / name, out / name)
for name in ('link_protocol.c', 'link_protocol.h'):
    path, = chassis.rglob(name)
    shutil.copyfile(path, out / name)
shutil.copyfile(here / 'independent_fixture.c', out / 'unit.c')
shutil.copyfile(here / 'independent_stm32_stub.h', out / 'stm32f4xx.h')
(out / 'FreeRTOS.h').write_text('#define portTICK_PERIOD_MS 1U\n#define pdMS_TO_TICKS(x) (x)\n', 'ascii')
(out / 'task.h').write_text('#include <stdint.h>\nuint32_t xTaskGetTickCount(void);\n'
                            'void vTaskDelay(uint32_t);\n', 'ascii')
(out / 'unit.sct').write_text(
    'LR_ROM 0x08000000 0x100000 {\n'
    '  ER_ROM 0x08000000 0x100000 { unit.o (i.test_entry, +First) *(+RO) }\n'
    '  RW_RAM 0x20000000 0x20000 { *(+RW,+ZI) }\n}\n', 'ascii')
arm = args.arm_bin
commands = []
logs = []


def command(tool, *args):
    cmd = [str(arm / (tool + '.exe')), *args]
    commands.append(cmd)
    run = subprocess.run(cmd, cwd=out, capture_output=True)
    logs.append((run.stdout + run.stderr).decode(errors='replace'))
    if run.returncode:
        raise RuntimeError(logs[-1])
    return run.stdout


for stem in ['unit', 'radar_client', 'radar_observer', 'link_protocol']:
    command('armcc', '--cpu', 'Cortex-M4', '--c99', '--gnu',
            '--split_sections', '-O1', '-I.', '-c', stem+'.c', '-o', stem+'.o')
command('armlink', '--entry', 'test_entry', '--scatter', 'unit.sct',
        'unit.o', 'radar_client.o', 'radar_observer.o', 'link_protocol.o',
        '--library_type=microlib', '-o', 'unit.axf')
symbols = command('fromelf', '--text', '-s', 'unit.axf').decode(errors='replace')
(out / 'compile.log').write_text('\n'.join(logs[:-1]), encoding='utf-8')
(out / 'commands.json').write_text(json.dumps(commands, indent=2), encoding='utf-8')

try:
    import unicorn as uc
except ModuleNotFoundError:
    sys.path.insert(0, str(root / '工创赛/tmp/position_alignment_deps'))
    import unicorn as uc
from unicorn import arm_const as reg

blob = (out / 'unit.axf').read_bytes()
elf = struct.unpack_from('<16sHHIIIIIHHHHHH', blob)
m = uc.Uc(uc.UC_ARCH_ARM, uc.UC_MODE_THUMB | uc.UC_MODE_MCLASS)
m.mem_map(0x08000000, 0x100000)
m.mem_map(0x20000000, 0x20000)
m.mem_write(0x20000000, b'\xa5' * 0x20000)
for index in range(elf[12]):
    _, kind, flags, addr, offset, size, _, _, _, _ = struct.unpack_from(
        '<IIIIIIIIII', blob, elf[6] + index * elf[11])
    if flags & 2 and size:
        if kind == 1:
            m.mem_write(addr, blob[offset:offset + size])
        elif kind == 8:
            m.mem_write(addr, b'\x00' * size)
stop = 0x080ff000
m.reg_write(reg.UC_ARM_REG_SP, 0x2001fff0)
m.reg_write(reg.UC_ARM_REG_LR, stop | 1)
m.emu_start(elf[4] | 1, stop, timeout=60000000, count=300000000)


def read(name, length=1):
    addr = int(re.search(r'\b' + name + r'\s+(0x[0-9a-fA-F]+)', symbols)[1], 16)
    values = list(struct.unpack('<' + 'I'*length, m.mem_read(addr, 4*length)))
    return values[0] if length == 1 else values


results = {
    'cpu': 'Cortex-M4 from ARMCC, executed under Unicorn',
    'checks': read('audit_checks'),
    'failed_lines': [n for n in read('audit_failed_lines', 48) if n],
    'arm_return': m.reg_read(reg.UC_ARM_REG_R0),
    'sha256': {name: sha256((out / name).read_bytes()).hexdigest() for name in
               ['unit.c', 'radar_client.c', 'radar_client.h', 'radar_observer.c',
                'radar_observer.h', 'link_protocol.c', 'unit.axf']},
}
(out / 'result.json').write_text(json.dumps(results, indent=2), encoding='utf-8')
print(json.dumps(results, indent=2))
lines = (out / 'unit.c').read_text(encoding='ascii').splitlines()
for line in results['failed_lines']:
    print(line, lines[line-1])
raise SystemExit(bool(results['arm_return']))
