#!/usr/bin/env python3
"""Copy historical search ROI into a separate pixel-only profile, never confirm metric."""
import argparse
import hashlib
import json
from pathlib import Path

parser=argparse.ArgumentParser(description=__doc__)
parser.add_argument('--search-profile',type=Path,required=True)
parser.add_argument('--output',type=Path,required=True)
args=parser.parse_args()
raw=json.loads(args.search_profile.read_text(encoding='utf-8-sig'))
profile=json.loads((Path(__file__).resolve().parent/'stage3_candidate/config/stage3.json').read_text(encoding='utf-8'))
search=raw.get('search_calibration')
if not isinstance(search,dict):raise ValueError('No historical search_calibration record')
if raw.get('camera',{}).get('undistort') or search.get('undistort_enabled'):
    raise ValueError('This import is for the existing raw-image profile only; keep calibrated camera profile independent')
if args.output.exists():raise ValueError('Output already exists; preserve it and choose a new path')
profile['search_calibration']=search
profile['camera'].update(width=search['image_size'][0],height=search['image_size'][1])
profile['pixel_preview_provenance']=dict(path=str(args.search_profile.resolve()),
    sha256=hashlib.sha256(args.search_profile.read_bytes()).hexdigest(),historical=True,
    meaning='Search pixels only. Not current camera/TCP/axis or metric acceptance.')
args.output.parent.mkdir(parents=True,exist_ok=True)
args.output.write_text(json.dumps(profile,ensure_ascii=False,indent=2)+'\n',encoding='utf-8')
print(json.dumps(dict(output=str(args.output.resolve()),calibration_verified=False,metric_matrices_imported=False)))
