"""Exercise current lift state machine and actual direction-dependent packet guards."""
from pathlib import Path
import shutil
import subprocess

ROOT = Path(__file__).resolve().parents[1]
STAGE = ROOT/'tmp/lift_up_step_20261028/project'
OUT = ROOT/'tmp/lift_up_step_20261028/host-check'
OLD = ROOT/'tmp/lift_sensorless_20261025/host-check'
EV = ROOT/'evidence/p2-arm/2026-10-28/lift-up-step'
GCC = 'C:/mingw/mingw64/bin/gcc.exe'

OUT.mkdir(parents=True, exist_ok=True)
for name in ['stm32f4xx.h', 'stm32f4xx_can.h', 'FreeRTOS.h', 'task.h', 'can.h']:
    shutil.copy2(OLD/name, OUT/name)
test = (OLD/'test.c').read_text(encoding='utf-8')
old = 'else {assert(q[0]==0xFD && n==12 && q[9]==2 && q[2]==11 && q[3]==184 && q[4]==250 && q[7]==19 && q[8]==136);position+=q[1]?-5000:5000;moves++;}'
new = '''else {
        uint32_t p=((uint32_t)q[5]<<24)|((uint32_t)q[6]<<16)|((uint32_t)q[7]<<8)|q[8];
        assert(q[0]==0xFD && n==12 && q[9]==2 && q[2]==11 && q[3]==184 && q[4]==250);
        assert(p>0 && p<=(q[1]?50000U:5000U));
        position+=q[1]?-(int32_t)p:(int32_t)p;moves++;
    }'''
assert test.count(old) == 1
test = test.replace(old, new)
extra = '''
    assert(LiftProtocol_Move(1,50000,q));
    assert(q[1]==1 && q[7]==0xC3 && q[8]==0x50 && q[9]==2);
    assert(!LiftProtocol_Move(1,50001,q));
    assert(!LiftProtocol_Move(0,50000,q));
    ready();g_lift_home_request=2;step(1);
    g_lift_up_pulses=0;
    { unsigned before=moves;g_lift_motion_request=2;step(1);
      assert(moves==before && g_lift_motion_state==9 && g_lift_motion_result==6); }
    ready();g_lift_up_pulses=50001;
    { unsigned before=moves;g_lift_motion_request=2;step(1);
      assert(moves==before && g_lift_motion_state==9); }
    ready();g_lift_up_pulses=50000;g_lift_motion_request=2;step(1);
    g_lift_up_pulses=1000; /* Mid-move writes must not alter the accepted target. */
    step(80);assert(g_lift_motion_state==7 && g_lift_delta_raw==-50000);
    { unsigned before=moves;step(300);assert(moves==before); } /* No auto-repeat. */
    g_lift_motion_request=2;step(80);
    assert(g_lift_motion_state==7 && g_lift_delta_raw==-1000);
    g_lift_up_pulses=50000;g_lift_motion_request=1;step(80);
    assert(g_lift_delta_raw==5000); /* Upward setting cannot enlarge downward jog. */
    moving=1;g_lift_motion_request=2;step(7);g_lift_motion_request=3;step(1);
    assert(g_lift_motion_state==8 && g_lift_motion_result==2);
    puts("PASS: upward bounds, per-request snapshot, no auto-repeat, downward unchanged, stop.");
'''
test = test.replace('    return 0;\n}', extra+'    return 0;\n}')
(OUT/'test.c').write_text(test, encoding='utf-8')
module = STAGE/'HARDWARE/LIFT'

def run(args):
    result = subprocess.run(args, capture_output=True, text=True, encoding='utf-8', errors='replace')
    if result.returncode:
        raise RuntimeError(result.stdout+result.stderr+str(result.returncode))
    return result.stdout+result.stderr

run([GCC, '-std=c99', '-Wall', '-Wextra', '-Werror', '-I'+str(OUT), '-I'+str(module),
     str(module/'lift.c'), str(module/'lift_protocol.c'), str(module/'lift_home.c'),
     str(OUT/'test.c'), '-o', str(OUT/'test.exe')])
log = run([str(OUT/'test.exe')])
# Compile the guard directly extracted from the current staged transport.
can = (STAGE/'SYSTEM/CAN/can.c').read_bytes().decode('utf-8', errors='replace')
can = can[can.index('uint32_t CAN1_LiftExchange('):]
guard = can[can.index('    speed_rpm ='):can.index('    memset(reply,')]
guard_test = '''#include <stdint.h>
#include <assert.h>
#include <stdio.h>
static uint32_t validate(const uint8_t *request,uint8_t length) {
uint32_t pulses,speed_rpm;
'''+guard+'''return 1; }
int main(void) {
 uint8_t q[12]={0xFD,1,11,184,250,0,0,0xC3,0x50,2,0,0x6B};
 assert(validate(q,12)==1);q[8]++;assert(validate(q,12)==7);q[8]--;
 q[1]=0;assert(validate(q,12)==7);q[7]=19;q[8]=136;assert(validate(q,12)==1);
 q[8]++;assert(validate(q,12)==7);q[7]=0;q[8]=0;assert(validate(q,12)==7);
 q[1]=1;q[7]=0xC3;q[8]=0x50;q[9]=0;assert(validate(q,12)==7);
 q[9]=2;q[2]=11;q[3]=185;assert(validate(q,12)==7);
 q[3]=184;q[1]=2;assert(validate(q,12)==7);
 q[1]=1;assert(validate(q,11)==7);
 puts("PASS: actual CAN guard accepts upward 50000, rejects overlimit/downward extension/invalid mode/speed/direction/length.");
 return 0;
}
'''
(OUT/'guard.c').write_text(guard_test, encoding='utf-8')
run([GCC,'-std=c99','-Wall','-Wextra','-Werror',str(OUT/'guard.c'),'-o',str(OUT/'guard.exe')])
log += run([str(OUT/'guard.exe')])
(EV/'host-check.log').write_text(log, encoding='utf-8')
print(log)
