from pathlib import Path
import csv, ctypes, hashlib, json, re, struct, subprocess, sys, xml.etree.ElementTree as ET

R = Path.cwd()
E = Path(__file__).resolve().parent
U = R / '3.stm32f407工程/CHASSIS/USER'
B = Path('D:/Keil_v5/ARM/arm5_compiler/Bin')
T = E / 'actual-arm'
T.mkdir(exist_ok=True)
PRODUCT = ['fusion_adapter.c', 'fusion_adapter.h', 'fusion_reference.c', 'fusion_reference.h',
           'app_odometry.h', 'chassis_odometry.h', 'chassis_kinematics.c', 'chassis_kinematics.h']
before = {n: hashlib.sha256((U / n).read_bytes()).hexdigest() for n in PRODUCT}

def short(path):
    result = ctypes.create_unicode_buffer(32768)
    if not ctypes.windll.kernel32.GetShortPathNameW(str(path), result, len(result)):
        raise OSError(str(path))
    return result.value

def run(command, log):
    result = subprocess.run(command, cwd=T, capture_output=True, timeout=90)
    (T / log).write_bytes(result.stdout + result.stderr)
    if result.returncode:
        raise RuntimeError((result.stdout + result.stderr).decode('gbk', errors='replace'))

unit = (E / 'independent_unit.c').read_text(encoding='ascii')
rows = list(csv.reader((E.parent / 'adapter_vectors_v8.csv').open()))
def literal(value):
    text = format(float(value), '.10g')
    return text + ('f' if '.' in text or 'e' in text else '.0f')
for row in rows:
    dt, direction, speed, corr, h0, h1, vx, vy, omega, px, py = map(float, row)
    unit += '\nsetup(); CK(prepare()); input.map.pose.heading_deg=' + literal(h0) + '; CK(observe()==FUSION_ADAPTER_WAIT);'
    unit += 'advance(' + str(int(dt)) + 'U); input.map.raw_x_mm=' + literal(px-corr) + '; input.map.raw_y_mm=' + literal(py+corr) + ';'
    unit += 'input.map.pose.x_mm=' + literal(px) + '; input.map.pose.y_mm=' + literal(py) + '; input.map.correction_x_mm=' + literal(corr) + '; input.map.correction_y_mm=' + literal(-corr) + '; input.map.pose.heading_deg=' + literal(h1) + ';'
    unit += 'CK(observe()==FUSION_ADAPTER_READY); CLOSE(observation.vx_mm_s,' + literal(vx) + ',.004f); CLOSE(observation.vy_mm_s,' + literal(vy) + ',.004f); CLOSE(observation.omega_deg_s,' + literal(omega) + ',.004f);'
    unit += 'CLOSE(observation.position.x_mm,' + literal(px) + ',.001f); CLOSE(observation.position.y_mm,' + literal(py) + ',.001f); CK(observation.feedback_source==FUSION_ESTIMATED_FEEDBACK&&!observation.stopped);'
unit += '\nreturn 0;\n}\n'
(T / 'unit.c').write_bytes(unit.encode('ascii'))
control = ET.parse(U.parents[1] / 'gongxun-f407.uvprojx').getroot().find('./Targets/Target/TargetOption/TargetArmAds/Cads/VariousControls')
options = ['--cpu', 'Cortex-M3', '--c99', '--gnu', '--split_sections', '-O1', '--diag_suppress=177,550',
           '-DVEHICLE_INTEGRATED_ENABLE=0', '-DVEHICLE_STAGE1_ENABLE=0', '-DAPP_VM_CALIBRATION_ONLY_ENABLE=0',
           '-DAPP_VM_ONLY_ENABLE=0', '-DAPP_WHEEL_ENABLE_HOLD_ENABLE=0', '-DAPP_BASELINE40_ENABLE=0']
options += ['-D' + d for d in control.findtext('Define').split(',')]
for include in control.findtext('IncludePath').split(';'):
    options += ['-I', short((U.parents[1] / include).resolve())]
objects = []
for path in [T / 'unit.c', U / 'fusion_adapter.c', U / 'fusion_reference.c', U / 'chassis_kinematics.c']:
    obj = path.stem + '.o'
    objects.append(obj)
    run([str(B / 'armcc.exe'), *options, '-c', short(path), '-o', obj], path.stem + '_compile.log')
(T / 'unit.sct').write_text('LR_ROM 0x08000000 0x100000 { ER_ROM 0x08000000 0x100000 { unit.o (i.test_entry, +First) *(+RO) } RW_RAM 0x20000000 0x50000 { *(+RW,+ZI) } }\n')
run([str(B / 'armlink.exe'), '--entry', 'test_entry', '--scatter', 'unit.sct', *objects,
     '--library_type=microlib', '-o', 'unit.axf'], 'link.log')
sys.path.insert(0, str(R / '工创赛/tmp/position_alignment_deps'))
import unicorn as uc
from unicorn import arm_const as register
data = (T / 'unit.axf').read_bytes()
elf = struct.unpack_from('<16sHHIIIIIHHHHHH', data)
machine = uc.Uc(uc.UC_ARCH_ARM, uc.UC_MODE_THUMB | uc.UC_MODE_MCLASS)
for address, size in [(0x08000000, 0x100000), (0x20000000, 0x50000), (0xE0000000, 0x100000)]:
    machine.mem_map(address, size)
for index in range(elf[12]):
    _, kind, flags, address, offset, size, *_ = struct.unpack_from('<IIIIIIIIII', data, elf[6] + index * elf[11])
    if flags & 2 and size:
        machine.mem_write(address, b'\0' * size if kind == 8 else data[offset:offset + size])
stop = 0x080ff000
machine.reg_write(register.UC_ARM_REG_SP, 0x2004fff0)
machine.reg_write(register.UC_ARM_REG_LR, stop | 1)
machine.emu_start(elf[4] | 1, stop, timeout=60000000, count=200000000)
if machine.reg_read(register.UC_ARM_REG_PC) != stop:
    raise RuntimeError('ARM execution did not return to sentinel')
result = machine.reg_read(register.UC_ARM_REG_R0)
symbols = subprocess.check_output([str(B / 'fromelf.exe'), '--text', '-s', 'unit.axf'], cwd=T).decode(errors='replace')
def value(name):
    address = int(re.search(r'\b' + name + r'\s+(0x[0-9a-fA-F]+)', symbols)[1], 16)
    return struct.unpack('<I', machine.mem_read(address, 4))[0]
after = {n: hashlib.sha256((U / n).read_bytes()).hexdigest() for n in PRODUCT}
report = {'status': 'PASS' if not result and before == after else 'FAIL', 'failed_line': result,
          'assertions': value('review_assertions'), 'matlab_vectors': len(rows), 'source_sha256': before,
          'sources_unchanged_during_run': before == after, 'hardware_accessed': False,
          'execution': 'Actual ARMCC Cortex-M3 soft-float machine code in Unicorn; real AppMapPose header; no platform stubs',
          'sizes': {name: value('size_' + name) for name in ['adapter_state', 'adapter_input', 'adapter_report', 'map', 'observation']}}
(E / 'actual_arm_results.json').write_text(json.dumps(report, indent=2), encoding='utf-8')
print(json.dumps(report))
if result:
    print(unit.splitlines()[result - 1])
    raise SystemExit(result)
if before != after:
    raise RuntimeError('Product changed during independent run')
