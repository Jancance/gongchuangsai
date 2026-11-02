"""Run corrected endpoint fixture against preserved baseline/candidate C sources."""
import argparse
import hashlib
import json
from pathlib import Path
import sys

from p1_longitudinal_feedforward_sim import ROOT, adapter, hashes

SOURCE=ROOT/'工创赛/evidence/p1-foundation/2026-11-02/longitudinal_feedforward_v4'


def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--out',type=Path,required=True)
    parser.add_argument('--variant',choices=['baseline','zero','005','010','020'],required=True)
    parser.add_argument('--quick',action='store_true')
    a=parser.parse_args();out=a.out.resolve()
    if not out.is_relative_to(ROOT):raise ValueError('Output outside workspace')
    out.mkdir(parents=True,exist_ok=False)
    before=hashes(ROOT/'3.stm32f407工程')
    candidate=SOURCE/('baseline' if a.variant=='baseline' else 'candidate')
    candidate_before=hashes(candidate/'USER')
    ratio={'baseline':-1,'zero':0,'005':5,'010':10,'020':20}[a.variant]
    module=adapter(out,ratio,ratio,False,corrected_endpoint=True)
    if a.quick:
        p=out/'adapter_regression/fixture.c'
        fixture=p.read_text(encoding='utf8')
        fixture=fixture[:fixture.index('uint32_t test_entry(void){')]+'''uint32_t test_entry(void){
 endpoint_guard_contract_checks();feedforward_boundaries();
 single_turn_case=-1;micro_case=1;
 scenario_run(1,1800,200,.08f,0,0,0);CHECK(micro_injected>0);
 return first_failure?(0x80000000U|first_failure):checks;
}
'''
        p.write_text(fixture,encoding='utf8')
    error=None
    try:
        report=module.run(candidate,out/'models')
    except RuntimeError as exc:
        error=str(exc)
        p=out/'models/report.json'
        if not p.exists():raise
        report=json.loads(p.read_text(encoding='utf8'))
    same=before==hashes(ROOT/'3.stm32f407工程')
    immutable=candidate_before==hashes(candidate/'USER')
    evidence=dict(variant=a.variant,quick=a.quick,passed=report['passed'],
                  assertions=report['assertions'],failure_context=report['failure_context'],
                  models=len(report['models']),endpoint_injections=report.get('endpoint_injections'),
                  live_unchanged=same,candidate_user_unchanged=immutable,hardware_accessed=False,
                  error=error,script_sha256=hashlib.sha256(Path(__file__).read_bytes()).hexdigest())
    (out/'verification.json').write_text(json.dumps(evidence,ensure_ascii=False,indent=2),encoding='utf8')
    print(json.dumps(evidence,ensure_ascii=False,indent=2),flush=True)
    assert same and immutable
    return 0 if report['passed'] else 1


if __name__=='__main__':
    sys.exit(main())
