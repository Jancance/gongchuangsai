"""One user-authorized batch at195/165deg, without any body positioning."""
import argparse
import hashlib
import json
from pathlib import Path
import source_latched_trial_20261115 as trial

R=Path(__file__).resolve().parents[1]
E=R/'evidence/integration/2026-11-15/integration20-yaw165-window10'

def manifest():
    m=json.loads((E/'candidate.json').read_text(encoding='utf-8'))
    assert m['build_passed'] and json.loads((E/'checks.json').read_text())['passed']
    assert json.loads((E/'matlab-result.json').read_text())['passed']
    for name,digest in m['changes'].items():
        assert hashlib.sha256((R/m['project']/name).read_bytes()).hexdigest()==digest,name
    image=m['images']['Vehicle_LiftRuler']
    assert hashlib.sha256((R/image['axf']).read_bytes()).hexdigest()==image['sha256']
    config=json.loads((E/'trial-config.json').read_text())
    assert config['skip_body_alignment'] and config['yaw_deg']==[195,165,180]
    assert config['extension_path_window_mm']==10
    poses=list(zip(config['yaw_deg'],config['extensions_mm']))
    assert all(0<=angle<=270 and 0<=extension<=170 for angle,extension in poses)
    return image,poses,None

def hold_without_alignment(board,vision,remaining,profile,deadline,report,save):
    state=trial.checked_state(board)
    if not board.pose_ready(state) or state['g_source_align_chain_active']:
        raise trial.ObservationUnavailable('Not at completed observation pose; no body correction requested')
    report['body_alignment_skipped_by_user']=True
    trial.ensure_wheel_hold(board,deadline,report)
    save()

def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--once-batch',action='store_true')
    parser.add_argument('--debug-budget',type=float,default=0)
    parser.add_argument('--task-code',default='215+213+512+321')
    parser.add_argument('--port',type=int,default=18887)
    args=parser.parse_args();args.align_only=False
    image,poses,_=manifest()
    if not args.once_batch:
        print(json.dumps(dict(local_check=True,hardware_access=False,image_sha256=image['sha256'],poses=poses,skip_body_alignment=True)))
        return
    trial.E=E;trial.manifest=manifest;trial.position_body=hold_without_alignment
    trial.run(args)

if __name__=='__main__':main()
