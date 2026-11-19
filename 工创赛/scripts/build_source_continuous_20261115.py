"""Offline-only continuous scheduler checks and dual Keil builds."""
from pathlib import Path
import json
import subprocess
import sys
import build_integration18_ext150_20261115 as build

R=Path(__file__).resolve().parents[1]
P=R/'tmp/source_mcu_continuous_20261115/project'
E=R/'evidence/integration/2026-11-15/source-continuous'

def main(build_firmware=True):
    assert json.loads((E/'matlab-result.json').read_text())['passed']
    command=[sys.executable,'-X','utf8','-m','unittest','discover','-s','tests','-p','test_continuous_turntable_20261115.py','-v']
    env=__import__('os').environ.copy();env['PYTHONPATH']=str(R/'src')
    result=subprocess.run(command,cwd=R,env=env,capture_output=True,text=True)
    (E/'prediction-tests.log').write_text(result.stdout+result.stderr,encoding='utf-8');print(result.stdout+result.stderr,flush=True)
    assert result.returncode==0
    checker=(R/'scripts/check_integration18_ext150_20261115.py').read_text(encoding='utf-8')
    checker=checker.replace('SOURCE_EXTENSION_DONE_DEG==71','fabsf(SOURCE_EXTENSION_DONE_DEG-177.46875f)<.001f')
    checker=checker.replace('a->current=29;','a->current=100.0f-177.46875f+.01f;').replace('a->current=28;','a->current=100.0f-177.46875f-.01f;')
    namespace={'__name__':'continuous_poll_fixture','__file__':str(R/'scripts/check_integration18_ext150_20261115.py')}
    exec(compile(checker,namespace['__file__'],'exec'),namespace);namespace.update(P=P,E=E,EXPECTED_CORRIDOR_DEG=354.9375);namespace['main']()
    (E/'can.h').write_text('#ifndef FIXTURE_CAN_H\n#define FIXTURE_CAN_H\n#include <stdint.h>\ntypedef struct{uint8_t Data[8];}CanRxMsg;\n#endif\n')
    (E/'Buzzer.h').write_text('void Buzzer_StartTimed(unsigned,unsigned,unsigned);\nunsigned Buzzer_IsBusy(void);\n')
    fixture=(R/'tests/overlap_commit_owner_20261114.c').read_text().replace('records[16]','records[32]').replace('recorded<16','recorded<32')
    (E/'base-commit-fixture.c').write_text(fixture,encoding='utf-8')
    fixture=(R/'tests/source_mcu_cycle_20261115.c').read_text().replace('int main(void){','static int previous_cycle_test_main(void){',1)
    (E/'static-cycle-fixture.c').write_text(fixture,encoding='utf-8')
    main=(P/'CHASSIS/USER/main.c').read_bytes();start=main.index(b'        if(SOURCE_MCU_CYCLE_ENABLE)');end=main.index(b'        else if (g_radar_servo_key_test_enable',start)
    (E/'tested-key-branch.inc').write_bytes(main[start:end])
    owner=(P/'CHASSIS/USER/pzq_commit_overlap.inc').read_bytes()
    (E/'tested-commit-owner.inc').write_bytes(owner.replace(b'#include "low_readonly_diag.inc"',b'/* tested separately */'))
    sys.path.insert(0,str(R/'src'))
    from smart_carrier.vision_manipulation.mcu_cycle_protocol import continuous_observation
    for index,(pose,sequence,colors,target,delays) in enumerate(((4,3,[2,5,1],2,[300,100,400]),(6,1,[5,0,1],1,[400,500,100]),(8,1,[5,0,0],5,[100,300,400]))):
        (E/f'python-continuous-{index}.txt').write_bytes(continuous_observation(1,pose,sequence,True,colors,0,target,delays))
    (E/'python-continuous-3.txt').write_bytes(continuous_observation(1,99,2,True,[2,5,1],0,2,[300,100,400]))
    (E/'python-continuous-4.txt').write_bytes(continuous_observation(1,4,2,True,[2,5,1],201,2,[300,100,400]))
    bad=bytearray(continuous_observation(1,4,2,True,[2,5,1],0,2,[300,100,400]));bad[-4]=ord('Z')
    (E/'python-continuous-5.txt').write_bytes(bad)
    (E/'python-continuous-6.txt').write_bytes(continuous_observation(1,4,2,True,[2,5,1],0,1,[300,100,400]))
    exe=E/'continuous-owner.exe'
    subprocess.run(['gcc','-std=c99','-Wall','-Wextra','-Werror','-Wno-unused-function','-Wno-misleading-indentation',
        '-I'+str(E),'-I'+str(R/'tests'),'-I'+str(P/'CHASSIS/USER'),'-I'+str(P/'USER'),
        str(R/'tests/source_continuous_owner_20261115.c'),'-lm','-o',str(exe)],check=True)
    result=subprocess.run([str(exe)],cwd=E,capture_output=True,text=True)
    (E/'continuous-owner.log').write_text(result.stdout+result.stderr,encoding='utf-8');print(result.stdout+result.stderr,flush=True);assert result.returncode==0
    (E/'continuous-checks.json').write_text(json.dumps({'passed':True,'owner_result':result.stdout.strip(),'python_tests_passed':True,'hardware_access':False},indent=2)+'\n')
    if build_firmware:
        build.PROJECT=P;build.BASE=R/'tmp/source_mcu_cycle_20261115/project';build.E=E;build.main()

if __name__=='__main__':main('--checks-only' not in sys.argv)
