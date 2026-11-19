"""Simplified candidate selector; default local check, no automatic deployment."""
import argparse
from pathlib import Path
import source_commit_trial_20261114 as trial

def main():
    parser=argparse.ArgumentParser(description=__doc__)
    action=parser.add_mutually_exclusive_group()
    action.add_argument('--align-only',action='store_true')
    action.add_argument('--once-batch',action='store_true')
    parser.add_argument('--debug-budget',type=float,default=0)
    parser.add_argument('--task-code',default='215+213+512+321')
    parser.add_argument('--port',type=int,default=18887)
    args=parser.parse_args()
    trial.E=Path(__file__).resolve().parents[1]/'evidence/integration/2026-11-14/overlap200-simplified'
    if args.align_only or args.once_batch:trial.run(args)
    else:trial.manifest();print('LOCAL CHECK: no hardware, deployment or reset. Total debug deadline disabled by default.')

if __name__=='__main__':main()
