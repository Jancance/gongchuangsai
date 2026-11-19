"""Test the actual MCU owner, UART framing, preserved motion and dual Keil build."""
from pathlib import Path
import json
import subprocess
import sys
import build_integration18_ext150_20261115 as build

R=Path(__file__).resolve().parents[1]
E=R/'evidence/integration/2026-11-15/source-mcu-cycle'
P=R/'tmp/source_mcu_cycle_20261115/project'

def main():
    assert json.loads((E/'matlab-result.json').read_text())['extension_arrival_mm']==5
    # Adapt the existing real poll fixture to the user's new arrival boundary.
    source=(R/'scripts/check_integration18_ext150_20261115.py').read_text(encoding='utf-8')
    source=source.replace('SOURCE_EXTENSION_DONE_DEG==71','fabsf(SOURCE_EXTENSION_DONE_DEG-177.46875f)<.001f')
    source=source.replace('a->current=29;','a->current=100.0f-177.46875f+.01f;')
    source=source.replace('a->current=28;','a->current=100.0f-177.46875f-.01f;')
    scope={'__name__':'current_poll_fixture','__file__':str(R/'scripts/check_integration18_ext150_20261115.py')}
    exec(compile(source,scope['__file__'],'exec'),scope)
    scope.update(P=P,E=E,EXPECTED_CORRIDOR_DEG=354.9375);scope['main']()
    (E/'Buzzer.h').write_text('void Buzzer_StartTimed(unsigned,unsigned,unsigned);\nunsigned Buzzer_IsBusy(void);\n',encoding='ascii')
    # The unchanged piece fixture has room for one piece; this integration runs3.
    fixture=(R/'tests/overlap_commit_owner_20261114.c').read_text(encoding='utf-8')
    fixture=fixture.replace('records[16]','records[32]').replace('recorded<16','recorded<32')
    (E/'base-commit-fixture.c').write_text(fixture,encoding='utf-8')
    main=(P/'CHASSIS/USER/main.c').read_bytes()
    start=main.index(b'        if(SOURCE_MCU_CYCLE_ENABLE)')
    end=main.index(b'        else if (g_radar_servo_key_test_enable',start)
    (E/'tested-key-branch.inc').write_bytes(main[start:end])
    owner=(P/'CHASSIS/USER/pzq_commit_overlap.inc').read_bytes()
    (E/'tested-commit-owner.inc').write_bytes(owner.replace(b'#include "low_readonly_diag.inc"',b'/* tested separately */'))
    (E/'can.h').write_text('#ifndef FIXTURE_CAN_H\n#define FIXTURE_CAN_H\n#include <stdint.h>\ntypedef struct{uint8_t Data[8];}CanRxMsg;\n#endif\n',encoding='ascii')
    sys.path.insert(0,str(R/'src'))
    from smart_carrier.vision_manipulation.mcu_cycle_protocol import observation
    (E/'python-observation.txt').write_bytes(observation(1,2,1,True,12,0,10,False,[0,0,0],0))
    exe=E/'mcu-cycle.exe'
    subprocess.run(['gcc','-std=c99','-Wall','-Wextra','-Werror','-Wno-unused-function','-Wno-misleading-indentation',
        '-I'+str(E),'-I'+str(P/'CHASSIS/USER'),'-I'+str(P/'USER'),
        str(R/'tests/source_mcu_cycle_20261115.c'),'-lm','-o',str(exe)],check=True)
    result=subprocess.run([str(exe)],cwd=E,capture_output=True,text=True)
    (E/'mcu-cycle.log').write_text(result.stdout+result.stderr,encoding='utf-8');print(result.stdout+result.stderr,flush=True)
    assert result.returncode==0
    build.PROJECT=P;build.BASE=R/'tmp/source_top_window10_yaw165_20261115/project';build.E=E;build.main()

if __name__=='__main__':main()
