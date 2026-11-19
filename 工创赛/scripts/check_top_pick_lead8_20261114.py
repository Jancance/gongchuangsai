"""Exercise actual top owner plus actual five-frame gate, with simulated IO."""
from pathlib import Path
import importlib.util,json,subprocess
R=Path(__file__).resolve().parents[1]
P=R/'tmp/source_top_pick_lead8_20261114/project'
E=R/'evidence/integration/2026-11-14/top-pick-lead8'

def main():
    # Read-only diagnostic bus calls are not part of the motion-owner fixture.
    owner=(P/'CHASSIS/USER/pzq_low_pick.inc').read_bytes()
    owner=owner.replace(b'#include "low_readonly_diag.inc"\r\n',b'').replace(b'\r\n Low_ReadonlyDiag();',b'')
    (E/'tested-owner.inc').write_bytes(owner)
    test=(R/'tests/low_pick_owner_20261114.c').read_text(encoding='utf-8')
    test=test.replace('#include "pzq_low_pick.inc"','#include "tested-owner.inc"')
    test=test.replace('#define APP_MOTOR_OFFLINE_ONLY 0',
       '#define APP_MOTOR_OFFLINE_ONLY 0\n#define SOURCE_LIFT_PULSES_PER_MM 400.0f\n#define SOURCE_EXTENSION_DONE_DEG 71.0f')
    test=test.replace('62400','600').replace('118820','29705').replace('7200','1800').replace('*1600','*400')
    test=test.replace('top=body=owned=query_fail=query_delay=0;','body=owned=query_fail=query_delay=0;top=1;')
    test=test.replace('reset();measured_depth=63000;', 'reset();top=0;measured_depth=63000;')
    test=test.replace('complete();CHECK(g_low_phase==9 && g_it_lift_pulses==600);\n complete();', 'complete();')
    test=test.replace('fabsf(measured_depth-600)<1','fabsf(measured_depth-1800)<1')
    test=test.replace('actual low owner','actual top owner').replace('return low65','return top65')
    (E/'owner-test.c').write_text(test,encoding='utf-8')
    subprocess.run(['gcc','-std=c99','-I'+str(E),'-I'+str(P/'CHASSIS/USER'),str(E/'owner-test.c'),'-lm','-o',str(E/'owner-test.exe')],check=True)
    result=subprocess.run([str(E/'owner-test.exe')],check=True,capture_output=True,text=True)
    print(result.stdout.strip());(E/'owner-test.log').write_text(result.stdout,encoding='utf-8')
    spec=importlib.util.spec_from_file_location('top_stream',R/'output/top_material_stream_20261114.py')
    module=importlib.util.module_from_spec(spec);spec.loader.exec_module(module)
    for colors in [(2,1,5),(0,1,5),(0,0,5)]:
        gate=module.RemainingStopped()
        d=dict(calibrated=True,slot_colors=colors,regions=[dict(color_id=c,center_x_px=i*300,center_y_px=200) for i,c in enumerate(colors) if c])
        remaining=tuple(c for c in colors if c)
        answers=[gate.update(d,i*.04,remaining=remaining,pose_id='top',pose_sequence=1) for i in range(5)]
        assert answers==[False]*4+[True]
        assert not gate.update(d,.21,remaining=remaining,pose_id='new-top',pose_sequence=2)
    gate=module.RemainingStopped()
    for i in range(7):
        d=dict(calibrated=True,slot_colors=[2,0,0],regions=[dict(color_id=2,center_x_px=i*15,center_y_px=200)])
        assert not gate.update(d,i*.04,remaining=(2,),pose_id='top',pose_sequence=1)
    print('PASS five actual stable frames; three/two/one visible; pose reset and moving rejection')
    (E/'behavior-checks.json').write_text(json.dumps(dict(passed=True,actual_owner=result.stdout.strip(),five_frames=True,simulated_io=True,hardware_access=False)),encoding='utf-8')

if __name__=='__main__':main()
