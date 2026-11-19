"""Reuse existing verified attach/action tools with the lead8 candidate."""
import argparse
from pathlib import Path
R=Path(__file__).resolve().parents[1]
E=R/'evidence/integration/2026-11-14/top-pick-lead8'
P=R/'tmp/source_top_pick_lead8_20261114/project'
def main():
    parser=argparse.ArgumentParser()
    parser.add_argument('mode',choices=['read-only','prepare-once','driver-config'])
    parser.add_argument('--pb9-candidate',action='store_true')
    parser.add_argument('--overlap-candidate',action='store_true')
    parser.add_argument('--simplified-candidate',action='store_true')
    args=parser.parse_args()
    if sum([args.pb9_candidate,args.overlap_candidate,args.simplified_candidate])>1:parser.error('select one candidate')
    evidence=R/'evidence/integration/2026-11-14/top-pick-pb9' if args.pb9_candidate else E
    project=R/'tmp/source_top_pick_pb9_20261114/project' if args.pb9_candidate else P
    if args.overlap_candidate:
        evidence=R/'evidence/integration/2026-11-14/overlap200-commit'
        project=R/'tmp/source_top_overlap200_20261114/project'
    if args.simplified_candidate:
        evidence=R/'evidence/integration/2026-11-14/overlap200-simplified'
        project=R/'tmp/source_top_simplified_20261114/project'
    if args.mode=='driver-config':
        import read_extension_config_live_20261114 as tool
        tool.E=evidence
        tool.run()
    else:
        import source_body_step_live_20261111 as tool
        tool.E=evidence;tool.P=project
        if args.pb9_candidate or args.overlap_candidate or args.simplified_candidate:tool.GRIPPER_PWM_REGISTER=0x40000840
        tool.run(args.mode)
if __name__=='__main__':main()
