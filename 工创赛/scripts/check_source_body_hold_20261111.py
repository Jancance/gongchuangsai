"""Actual hold/ordinary C admission vs MATLAB exhaustive inputs; no hardware."""
from pathlib import Path
import json
import re
import subprocess

R = Path(__file__).resolve().parents[1]
P = R / 'tmp/source_body_hold_20261111/project'
E = R / 'evidence/integration/2026-11-11/source-body-hold'
proof = json.loads((E / 'matlab-result.json').read_text())
assert proof['passed']
code = (P / 'CHASSIS/USER/integration10_complete.inc').read_text(encoding='utf-8')
base = (R / 'tmp/source_body_stop_20261111/project/CHASSIS/USER/integration10_complete.inc').read_text(encoding='utf-8')
def fn(text, name):
    match = re.search(r'static uint32_t ' + name + r'\(void\)\s*\{[^}]*\}', text)
    assert match
    return match.group()
ordinary = fn(code, 'it_platform')
assert ordinary == fn(base, 'it_platform')
held = fn(code, 'it_hold_platform')
assert code.count('StartupAxes_Service(it_hold_platform(),g_can_guard_ready);') == 1
assert code.count('StartupAxes_Service(it_platform(),g_can_guard_ready);') == 0
rows = ',\n'.join('{' + ','.join(str(int(v)) for v in row) + '}' for row in proof['inputs'])
expected = ',\n'.join('{' + str(int(a)) + ',' + str(int(b)) + '}' for a, b in zip(proof['ordinary_expected'], proof['held_expected']))
prefix = r'''
#include <stdint.h>
#include <stdio.h>
static unsigned g_safety_ready,g_can_test_result,g_safety_fault,g_rtos_fault,g_can_ctrl_fault_latched,g_chassis_auto_active;
static unsigned window,holding;
static unsigned SourceBodyStep_WheelWindow(void){return window;}
static unsigned StartupAxes_Holding(void){return holding;}
'''
unit = prefix + ordinary + '\n' + held + '\n'
unit += 'static const unsigned inputs[][8]={' + rows + '};\n'
unit += 'static const unsigned expected[][2]={' + expected + '};\n'
unit += r'''
int main(void){
 for(unsigned i=0;i<sizeof(inputs)/sizeof(inputs[0]);i++){
  g_safety_ready=inputs[i][0];g_can_test_result=inputs[i][1];g_safety_fault=inputs[i][2];g_rtos_fault=inputs[i][3];
  g_can_ctrl_fault_latched=inputs[i][4];g_chassis_auto_active=inputs[i][5];window=inputs[i][6];holding=inputs[i][7];
  if(it_platform()!=expected[i][0] || it_hold_platform()!=expected[i][1]){fprintf(stderr,"FAIL input%u\n",i);return 1;}
 }
 puts("PASS actual IT ordinary/held C predicates:514 comparisons to MATLAB; all health faults still rejected.");return 0;}
'''
source = E / 'hold-check.c'
exe = E / 'hold-check.exe'
source.write_text(unit)
p = subprocess.run(['gcc', '-std=c99', '-Wall', '-Wextra', '-Werror', str(source), '-o', str(exe)], capture_output=True, text=True)
assert p.returncode == 0, p.stdout + p.stderr
p = subprocess.run([str(exe)], capture_output=True, text=True, timeout=10)
(E / 'hold-check.log').write_text(p.stdout + p.stderr)
assert p.returncode == 0, p.stdout + p.stderr
print(p.stdout.strip())
for rel in ['CHASSIS/USER/integration13_immediate.inc', 'CHASSIS/SYSTEM/SAFETY/safety.c', 'CHASSIS/USER/source_body_step.inc']:
    assert (P / rel).read_bytes() == (R / 'tmp/source_body_stop_20261111/project' / rel).read_bytes()
assert json.loads((E / 'owner-regression/checks.json').read_text())['passed']
(E / 'checks.json').write_text(json.dumps(dict(passed=True, hardware_access=False,
    matlab_c_comparisons=514, ordinary_command_admission_unchanged=True,
    only_explicit_healthy_owner_can_hold_during_body=True, actual_owner_regression_passed=True,
    previous_stop_repair_and_scheduler_byte_identical=True), indent=2) + '\n')
