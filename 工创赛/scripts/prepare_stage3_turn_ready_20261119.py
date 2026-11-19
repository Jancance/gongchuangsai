"""Wait on the existing turn-rate admission before dispatch; expose reject detail."""
import json
import shutil
import subprocess
from pathlib import Path
from prepare_stage3_ring2_touchdown_20261119 import edit
from prepare_stage3_speed_offline_20261119 import ROOT,EVIDENCE,TARGETS,digest

BASE=ROOT/'tmp/stage3_tray_overlap_close200_20261119/stage3-three-place-compatible.json'
OUT=ROOT/'tmp/stage3_turn_ready_20261119'


def main():
    proof=EVIDENCE/'stage3-turn-ready-matlab.log'
    assert 'PASS existing turn rate readiness and bounded wait: 240 cases' in proof.read_text()
    m=json.loads(BASE.read_text(encoding='utf-8'));base=Path(m['project']);project=OUT/'project'
    for n,h in (m['changes']|m['unchanged_sha256']).items():assert digest(base/n)==h,n
    shutil.copytree(base,project,ignore=shutil.ignore_patterns('Objects','Listings'))
    edit(project,'source_body_step.inc',(
        '  if(!g_can_guard_ready)return;',
        '''  if(!g_can_guard_ready)return;
  /* Same rate boundary as HeadingRuntimeStep; wait within the existing 12s. */
  if(s_align_kind==3U &&
     !(s_heading_source.rate_dps>=-3.0f && s_heading_source.rate_dps<=3.0f))return;'''))
    edit(project,'main.c',(
        'volatile uint32_t g_heading_fault_phase = 0U;',
        '''volatile uint32_t g_heading_start_reject_mask = 0U;
volatile uint32_t g_heading_fault_phase = 0U;'''))
    path=project/'CHASSIS/USER/main.c'
    text=path.read_bytes().decode('latin1').replace('\r\n','\n')
    begin=text.index('        if (((s_heading_exec_relative != 5.0f)')
    end=text.index('            App_HeadingRuntimeTrip(3U);',begin)
    old=text[begin:end]
    angle=old.split('if (',1)[1].split(' ||\n            !App_HeadingRuntimeSource(now)',1)[0]
    # Keep the first predicate's parentheses, dropping only the outer if's one.
    condition=old.strip()[4:-1]
    (OUT/'original-turn-admission.txt').write_text(condition,encoding='ascii')
    diagnostic='''        g_heading_start_reject_mask=0U;
        if(ANGLE)g_heading_start_reject_mask|=1U;
        if(!App_HeadingRuntimeSource(now))g_heading_start_reject_mask|=2U;
        if(!App_HeadingRuntimeExclusive())g_heading_start_reject_mask|=4U;
        if(g_vm_test_request!=App_StageStartVm())g_heading_start_reject_mask|=8U;
        if(g_heading_exec_trip)g_heading_start_reject_mask|=16U;
        if(status.active)g_heading_start_reject_mask|=32U;
        if(status.result)g_heading_start_reject_mask|=64U;
        if(status.cut_vm)g_heading_start_reject_mask|=128U;
        if(!(s_heading_source.rate_dps>=-3.0f && s_heading_source.rate_dps<=3.0f))g_heading_start_reject_mask|=256U;
        if(g_heading_start_reject_mask)
'''.replace('ANGLE',angle)
    edit(project,'main.c',(old.rstrip('\n'),diagnostic.rstrip('\n')))
    for n in ('main.c','source_body_step.inc'):
        key='CHASSIS/USER/'+n;m['unchanged_sha256'].pop(key,None);m['changes'][key]=digest(project/key)
    m.update(project=str(project),images={},flashed=False,build_passed=False,
             baseline_manifest=str(BASE),turn_ready_matlab_sha256=digest(proof))
    for target in TARGETS:
        for folder in ('Objects','Listings'):(project/folder/target).mkdir(parents=True,exist_ok=True)
        log=OUT/(target+'.log')
        run=subprocess.run(['C:/Keil_v5/UV4/UV4.exe','-r',str(project/'gongxun-f407.uvprojx'),'-t',target,'-j0','-o',str(log)],timeout=180)
        assert run.returncode==0 and b'0 Error(s), 0 Warning(s)' in log.read_bytes(),log
        axf=project/'Objects'/target/(target+'.axf');m['images'][target]=dict(axf=str(axf),sha256=digest(axf))
        print(target,'0 errors, 0 warnings',flush=True)
    m['build_passed']=True
    (OUT/'stage3-three-place-compatible.json').write_text(json.dumps(m,indent=2),encoding='utf-8')


if __name__=='__main__':main()
