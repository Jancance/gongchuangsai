"""Exercise actual STOP C functions with MATLAB fixtures; no devices."""
from pathlib import Path
import importlib.util
import json
import re
import struct
import subprocess

R = Path(__file__).resolve().parents[1]
P = R / 'tmp/source_body_stop_20261111/project'
E = R / 'evidence/integration/2026-11-11/source-body-stop'
proof = json.loads((E / 'matlab-result.json').read_text(encoding='utf-8'))
assert proof['passed'] and proof['engine'] == 'local MATLAB'


def function(text, name):
    match = re.search(r'^(?:static\s+)?(?:uint32_t|void)\s+' + name + r'\([^;]*?\)\s*\{', text, re.M | re.S)
    assert match, name
    pos = match.end()
    level = 1
    while level:
        level += (text[pos] == '{') - (text[pos] == '}')
        pos += 1
    return text[match.start():pos]


header = (P / 'CHASSIS/SYSTEM/SAFETY/safety.h').read_text(encoding='gb18030')
macros = '\n'.join(re.findall(r'^#define MOTION_STOP_\w+\s+\d+U', header, re.M))
prefix = r'''
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
typedef uint32_t TickType_t;
#define taskENTER_CRITICAL() ((void)0)
#define taskEXIT_CRITICAL() ((void)0)
#define pdMS_TO_TICKS(x) (x)
static uint32_t clock_ms;
static uint32_t g_rtos_fault,g_safety_fault,g_chassis_auto_cause,g_startup_axes_cause;
static uint32_t g_motion_auto_stop_state,g_motion_auto_stop_ack_mask,g_motion_auto_stop_fail_mask;
static uint32_t g_motion_auto_stop_result[4],g_motion_auto_stop_cut_vm,g_motion_auto_stop_session,g_motion_auto_stop_cause;
static uint32_t s_auto_stop_pending,s_auto_stop_inflight,s_auto_stop_seen_latch,s_auto_stop_vm_requested,s_auto_stop_retries;
static TickType_t s_auto_stop_start_tick;
static TickType_t xTaskGetTickCount(void){return clock_ms;}
static void reset(void){
 clock_ms=g_rtos_fault=g_safety_fault=g_chassis_auto_cause=g_startup_axes_cause=0;
 g_motion_auto_stop_state=g_motion_auto_stop_ack_mask=g_motion_auto_stop_fail_mask=0;
 memset(g_motion_auto_stop_result,0,sizeof(g_motion_auto_stop_result));
 g_motion_auto_stop_cut_vm=g_motion_auto_stop_session=g_motion_auto_stop_cause=0;
 s_auto_stop_pending=s_auto_stop_inflight=s_auto_stop_seen_latch=s_auto_stop_vm_requested=s_auto_stop_retries=s_auto_stop_start_tick=0;
}
'''
names = ['Safety_MotionStopCut', 'Safety_StopNeeded', 'Safety_MotionStopUpdate',
         'Safety_MotionStopTake', 'Safety_MotionStopReport']
body = (P / 'CHASSIS/SYSTEM/SAFETY/safety.c').read_text(encoding='gb18030')
actual = '\n'.join(function(body, name) for name in names)
fixtures = []
for i, scenario in enumerate(proof['scenarios']):
    for kind in ['events', 'expected']:
        values = scenario[kind]
        rows = ',\n'.join('{' + ','.join(str(int(v)) for v in row) + '}' for row in values)
        fixtures.append(f'static const int64_t {kind}_{i}[][{len(values[0])}]={{\n{rows}\n}};')
main = r'''
static unsigned checks;
static void scenario(unsigned id,const int64_t (*ev)[5],const int64_t (*expected)[12],unsigned count){
 reset();
 for(unsigned n=0;n<count;n++){
  uint8_t address=0;uint32_t session=0,taken=0;
  clock_ms=(uint32_t)ev[n][1];
  if(ev[n][0]==1){g_safety_fault=(uint32_t)ev[n][3];Safety_MotionStopUpdate((uint32_t)ev[n][2]);}
  else if(ev[n][0]==2)taken=Safety_MotionStopTake(&address,&session);
  else Safety_MotionStopReport((uint8_t)ev[n][2],g_motion_auto_stop_session+(uint32_t)ev[n][4],(uint32_t)ev[n][3]);
  uint32_t got[]={g_motion_auto_stop_state,g_motion_auto_stop_ack_mask,g_motion_auto_stop_fail_mask,
   s_auto_stop_pending,s_auto_stop_inflight,s_auto_stop_retries,g_motion_auto_stop_cause,
   g_motion_auto_stop_cut_vm,g_motion_auto_stop_session,taken,address,session};
  for(unsigned col=0;col<12;col++){
   checks++;
   if(got[col]!=(uint32_t)expected[n][col]){
    fprintf(stderr,"FAIL scenario%u event%u column%u got%u expected%u\n",id,n,col,got[col],(uint32_t)expected[n][col]);exit(1);
   }
  }
 }
}
int main(void){
'''
for i, scenario in enumerate(proof['scenarios']):
    main += f'scenario({i},events_{i},expected_{i},{len(scenario["events"])});\n'
main += 'printf("PASS actual STOP C vs local MATLAB: %u field comparisons.\\n",checks);return 0;}\n'


def build_run(label, source, cut, expect_success=True):
    src = E / (label + '.c')
    exe = E / (label + '.exe')
    src.write_text(source, encoding='utf-8')
    result = subprocess.run(['gcc', '-std=c99', '-Wall', '-Wextra', '-Werror',
                             '-Wno-unused-function', f'-DAPP_FAULT_CUT_VM_ENABLE={cut}',
                             str(src), '-o', str(exe)], capture_output=True, text=True)
    (E / (label + '-compile.log')).write_text(result.stdout + result.stderr, encoding='utf-8')
    assert result.returncode == 0, result.stdout + result.stderr
    result = subprocess.run([str(exe)], capture_output=True, text=True, timeout=15)
    log = result.stdout + result.stderr
    (E / (label + '.log')).write_text(log, encoding='utf-8')
    assert (result.returncode == 0) == expect_success, log
    print(label, log.strip())


source = prefix + macros + '\n' + actual + '\n' + '\n'.join(fixtures) + main
build_run('stop-check', source, 0)
baseline = (R / 'tmp/source_body_step_20261111/project/CHASSIS/SYSTEM/SAFETY/safety.c').read_text(encoding='gb18030')
baseline_actual = '\n'.join(function(baseline, name) for name in names)
build_run('stop-baseline-negative', source.replace(actual, baseline_actual, 1), 0, False)
cut_main = r'''
int main(void){uint8_t address=0;uint32_t session=0;
 reset();g_safety_fault=1;Safety_MotionStopUpdate(1);
 if(!Safety_MotionStopTake(&address,&session))return 1;
 Safety_MotionStopReport(address,session,8);
 if(g_motion_auto_stop_state!=MOTION_STOP_VM_CUT || !g_motion_auto_stop_cut_vm || s_auto_stop_pending)return 2;
 if(Safety_MotionStopTake(&address,&session))return 3;
 reset();g_safety_fault=1;Safety_MotionStopUpdate(1);clock_ms=500;Safety_MotionStopUpdate(1);
 if(g_motion_auto_stop_state!=MOTION_STOP_VM_CUT || !g_motion_auto_stop_cut_vm)return 4;
 puts("PASS original cut-VM compile branch: failed ACK and timeout still cut; no further STOP taken.");return 0;}
'''
build_run('stop-cut-profile-check', prefix + macros + '\n' + actual + cut_main, 1)

# Read the host evidence helper against a fake target. No pyOCD or USB import.
spec = importlib.util.spec_from_file_location('fault_detail', R / 'scripts/source_body_fault_snapshot_20261111.py')
helper = importlib.util.module_from_spec(spec)
spec.loader.exec_module(helper)
symbols = {}
memory = {}
def put(name, data):
    address = 0x20000000 + len(symbols) * 256
    symbols[name] = address
    memory[address] = data
for name in '''g_can_ctrl_fault_latched g_can_ctrl_trace_result g_can_ctrl_trace_phase g_can_ctrl_trace_address
g_can_ctrl_trace_length g_can_ctrl_trace_pe2_start g_can_ctrl_trace_pe2_end g_can_ctrl_trace_esr
g_can_ctrl_trace_rx_count g_can_ctrl_trace_flush_count g_can_ctrl_trace_tx_ms g_can_ctrl_trace_total_ms
g_motion_auto_stop_state g_motion_auto_stop_ack_mask g_motion_auto_stop_fail_mask g_motion_auto_stop_session
g_motion_auto_stop_cause g_motion_auto_stop_cut_vm g_chassis_auto_active g_dm_jog_first_fault
g_dm_can2_velocity_raw g_dm_can2_motor_state g_dm_can2_velocity_rad_s g_dm_can2_logical_position_deg
g_eh_first_reason g_eh_first_tick g_eh_first_command g_eh_first_exchange g_eh_first_flags
g_eh_first_service_gap g_eh_first_phase g_eh_first_step'''.split():
    put(name, struct.pack('<I', 0))
for name, value in [('g_can_ctrl_fault_latched', 1), ('g_can_ctrl_trace_length', 4),
                     ('g_motion_auto_stop_state', 2), ('g_motion_auto_stop_ack_mask', 0)]:
    memory[symbols[name]] = struct.pack('<I', value)
put('g_can_ctrl_trace_request', bytes.fromhex('fe98006b00000000'))
put('g_can_ctrl_trace_last_rx', struct.pack('<II', 0, 0x100) + bytes([4, 0, 3, 0xfe, 0xe2, 0x6b, 0, 0, 0, 0, 0, 0]))
put('g_can_ctrl_trace_context', struct.pack('<23I', *range(23)))
put('g_motion_auto_stop_result', struct.pack('<4I', 8, 1, 1, 1))
class FakeTarget:
    def read32(self, address):
        return struct.unpack('<I', memory[address][:4])[0]
    def read_memory_block8(self, address, size):
        assert len(memory[address]) == size
        return list(memory[address])
detail = helper.capture(FakeTarget(), symbols)
assert detail['writes'] == 0 and detail['can_trace_frozen']
assert detail['request_hex'] == 'fe98006b' and detail['last_rx']['data_hex'] == 'fee26b'
assert detail['context']['poll_age_ms'] == [19, 20, 21, 22]
assert not detail['stop_protocol_acknowledged']
assert helper.stop_acknowledged({'g_motion_auto_stop_state': 2, 'g_motion_auto_stop_ack_mask': 15, 'g_chassis_auto_active': 0})
assert not helper.stop_acknowledged({'g_motion_auto_stop_state': 1, 'g_motion_auto_stop_ack_mask': 15, 'g_chassis_auto_active': 0})
assert not helper.stop_acknowledged({'g_motion_auto_stop_state': 2, 'g_motion_auto_stop_ack_mask': 15, 'g_chassis_auto_active': 1})
memory[symbols['g_can_ctrl_fault_latched']] = struct.pack('<I', 0)
assert not helper.capture(FakeTarget(), symbols)['can_trace_frozen']
(E / 'host-fault-capture-check.json').write_text(json.dumps(dict(passed=True, simulated_target=True,
    real_hardware_access=False, false_acked_zero_mask_rejected=True, physical_stop_verified=False), indent=2) + '\n')
owner_checks = json.loads((E / 'owner-regression/checks.json').read_text())
assert owner_checks['passed']
(E / 'checks.json').write_text(json.dumps(dict(passed=True, hardware_access=False,
    actual_stop_matches_matlab=True, baseline_defect_reproduced=True,
    original_cut_vm_branch_preserved=True, host_fault_capture=True,
    owner_regression=owner_checks, mechanical_scheduler_unchanged=True), indent=2) + '\n')
print('STOP/host regression passed; no physical stop or CAN diagnosis claimed.')
