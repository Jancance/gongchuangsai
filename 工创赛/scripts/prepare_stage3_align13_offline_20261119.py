"""Build local matched Stage3 vision config; never connects to the vehicle."""
import argparse
import copy
import hashlib
import json
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
PROOF = ROOT / 'evidence/integration/2026-11-18/stage3-resume/pi-continuation-readonly-20261119-110639-583772.json'
MATLAB = ROOT / 'evidence/integration/2026-11-19/stage3-offline/align13-matlab.log'
BASE = ROOT / 'tmp/stage3_integrated_align15_yaw365_20261119/stage3-integrated-align15-compatible.json'

def sha(path):
    return hashlib.sha256(path.read_bytes()).hexdigest()

def prepare(right, manifest_path=BASE):
    if 'PASS align13: 250 ideal-response convergence cases' not in MATLAB.read_text(encoding='utf-8'):
        raise ValueError('Run matching MATLAB pre-validation first')
    manifest=json.loads(manifest_path.read_text(encoding='utf-8'))
    if not manifest['build_passed'] or not manifest['capabilities']['stage3_integrated_visual_compensation']:
        raise ValueError('Requires built integrated firmware')
    for item in manifest['images'].values():
        if sha(Path(item['axf'])) != item['sha256']:
            raise ValueError('Firmware image no longer matches its manifest')
    proof=json.loads(PROOF.read_text(encoding='utf-8'))
    cfg=copy.deepcopy(proof['stage3_config'])
    row=cfg['ring_row_alignment']; j=row['response_pixel_per_right_forward_ccw']
    target=[base+j[i][0]*right+j[i][1]*13 for i,base in enumerate((960,540))]
    row['target_center_px']=target
    row['integrated_compensation']={'forward_mm':13,'right_mm':right,
        'reference_center_px':[960,540],'post_alignment_replay':False}
    out=ROOT / f'tmp/stage3_align13_right{right}_offline_20261119'
    out.mkdir(parents=True,exist_ok=True)
    config_path=out/'stage3.json'
    config_path.write_text(json.dumps(cfg,indent=2)+'\n',encoding='utf-8')
    manifest.update(baseline_manifest=str(manifest_path),baseline_manifest_sha256=sha(manifest_path),
        flashed=False,pi_deployed=False,physical_acceptance=False,
        integrated_visual_compensation={'forward_mm':13,'right_mm':right,'target_pixel':target,
            'carrier_yaw_deg':36.5,'abi_symbol':'g_stage3_integrated_compensation','abi_value':1},
        offline_vision_config={'path':str(config_path),'sha256':sha(config_path)},
        compensation_matlab={'path':str(MATLAB),'sha256':sha(MATLAB)})
    result=out/'stage3-align13-compatible.json'
    result.write_text(json.dumps(manifest,indent=2)+'\n',encoding='utf-8')
    return {'manifest':str(result),'target_pixel':target,'forward_mm':13,'right_mm':right,
            'hardware_access':False}

if __name__=='__main__':
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--right-mm',type=int,choices=(0,5),required=True)
    parser.add_argument('--manifest',type=Path,default=BASE)
    args=parser.parse_args()
    print(json.dumps(prepare(args.right_mm,args.manifest),ensure_ascii=False))
