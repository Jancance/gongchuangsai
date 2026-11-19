"""Test actual read steps/wrapper, regress accepted owner, build both targets."""
from pathlib import Path
import json
import shutil
import subprocess
import build_integration18_ext150_20261115 as build

R=Path(__file__).resolve().parents[1]
E=R/'evidence/integration/2026-11-15/compact-poll'
BASE=R/'tmp/source_extension_lead_pick_20261115/project'
P=R/'tmp/source_compact_poll_20261115/project'

def main():
    assert json.loads((E/'matlab-result.json').read_text())['passed']
    old=R/'evidence/integration/2026-11-15/extension-lead-pick'
    for f in old.iterdir():
        if f.suffix in ('.c','.h','.inc','.txt'):shutil.copyfile(f,E/f.name)
    script=(R/'scripts/check_integration18_ext150_20261115.py').read_text()
    script=script.replace('SOURCE_EXTENSION_DONE_DEG==71','fabsf(SOURCE_EXTENSION_DONE_DEG-177.46875f)<.001f')
    script=script.replace('a->current=29;','a->current=100.0f-177.46875f+.01f;').replace('a->current=28;','a->current=100.0f-177.46875f-.01f;')
    script=script.replace("source.index('static uint32_t VSP_Poll(')","source.index('static uint32_t VSP_PollStep(')")
    scope={'__name__':'compact_poll_fixture','__file__':str(R/'scripts/check_integration18_ext150_20261115.py')}
    exec(compile(script,scope['__file__'],'exec'),scope)
    scope.update(P=P,E=E,EXPECTED_CORRIDOR_DEG=354.9375)
    original_run=scope['run_fixture']
    def run_fixture(name,source,offline=0):
        if name=='actual-vsp-poll':source=source.replace('VSP_Poll(0,','VSP_PollStep(0,')
        return original_run(name,source,offline)
    scope['run_fixture']=run_fixture;scope['main']()
    source=(E/'actual-vsp-poll.c').read_text()
    source=source.replace('static unsigned checks;','static unsigned checks,queries,miss_command,miss_position,active=1;')
    source=source.replace('static uint32_t Low_Busy(void){return 1;}','static uint32_t Low_Busy(void){return active;}')
    source=source.replace('(void)a;(void)n;memset(&s_rx','(void)a;(void)n;queries++;if(miss_command==c)return 0;memset(&s_rx')
    source=source.replace('(void)a;v->position_tick=tick;','(void)a;queries++;if(miss_position)return 0;v->position_tick=tick;')
    source=source.replace('int main(void){VSP_Axis *a=&vsp_axis[0];','static int previous_poll_test(void){VSP_Axis *a=&vsp_axis[0];',1)
    source+=(R/'tests/compact_poll_wrapper_20261115.inc').read_text()
    file=E/'actual-compact-poll.c';file.write_text(source,encoding='utf-8');exe=E/'actual-compact-poll.exe'
    subprocess.run(['gcc','-std=c99','-Wall','-Wextra','-Werror','-Wno-unused-function','-Wno-misleading-indentation',
        '-I'+str(P/'CHASSIS/USER'),'-I'+str(P/'USER'),str(file),'-lm','-o',str(exe)],check=True)
    result=subprocess.run([str(exe)],capture_output=True,text=True)
    (E/'actual-compact-poll.log').write_text(result.stdout+result.stderr,encoding='utf-8');print(result.stdout+result.stderr);assert result.returncode==0
    text=(R/'scripts/build_integration18_ext150_20261115.py').read_text()
    text=text.replace("str(ROOT/'tests'/source)","str(OWNER_TEST if name=='owner' else ROOT/'tests'/source)")
    scope={'__name__':'compact_firmware','__file__':build.__file__};exec(compile(text,build.__file__,'exec'),scope)
    scope.update(PROJECT=P,BASE=BASE,E=E,OWNER_TEST=E/'all-regions-owner.c');scope['main']()
    checks=json.loads((E/'checks.json').read_text());checks.update(actual_compact_poll_passed=True,
        same_real_queries=True,confirmation_rounds=2,trajectory_and_speed_unchanged=True,
        actual_cycle_time_verified=False,simultaneous_descent_not_implemented=True)
    (E/'checks.json').write_text(json.dumps(checks,indent=2)+'\n')

if __name__=='__main__':main()
