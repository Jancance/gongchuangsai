"""从冻结v1已存图片建立独立v2对照包；不改已有应用或旧包。"""
import hashlib
import json
from pathlib import Path
from zipfile import ZipFile,ZIP_DEFLATED

ROOT=Path(__file__).resolve().parents[1]
BASE=ROOT/'output/p2_partial_ring_offline_20261027'
DEST=ROOT/'output/p2_partial_ring_offline_v2_20261027'


def main():
    validation=json.loads((ROOT/'output/partial-ring-v2-validation-20261027/review.json').read_text(encoding='utf8'))
    assert validation['passed']==19
    old=json.loads((BASE/'manifest.json').read_text(encoding='utf8'))
    files={name:(BASE/name).read_bytes() for name in old['files']}
    for name,digest in old['files'].items():
        assert hashlib.sha256(files[name]).hexdigest()==digest
    files['p2_partial_ring_baseline_v1.py']=files['p2_partial_ring_candidate.py']
    files['p2_partial_ring_candidate.py']=(ROOT/'scripts/p2_partial_ring_candidate_v2.py').read_bytes()
    files['p2_partial_ring_offline_replay.py']=(ROOT/'scripts/p2_partial_ring_offline_replay_v2.py').read_bytes()
    files['p2_ring_raw_shared.py']=(ROOT/'scripts/p2_ring_raw_shared.py').read_bytes()
    manifest=dict(version=2,files={n:hashlib.sha256(d).hexdigest() for n,d in sorted(files.items())},cases=old['cases'],
                  source='v1 frozen inputs and baseline; v2 feature reuse and ROI-only feature calculation; all gates preserved.',
                  camera_opened=False,motion_allowed=False,metric_valid=False,hardware_acceptance=False)
    files['manifest.json']=json.dumps(manifest,ensure_ascii=False,indent=2).encode('utf8')
    DEST.mkdir(exist_ok=True)
    for name,data in files.items():
        path=DEST/name
        path.parent.mkdir(parents=True,exist_ok=True)
        path.write_bytes(data)
    archive=DEST.with_suffix('.zip')
    with ZipFile(archive,'w',ZIP_DEFLATED) as z:
        for name,data in sorted(files.items()):
            z.writestr(DEST.name+'/'+name,data)
    with ZipFile(archive) as z:
        assert z.testzip() is None
        for name,data in files.items():
            assert z.read(DEST.name+'/'+name)==data
    print(json.dumps(dict(archive=str(archive),bytes=archive.stat().st_size,
                         sha256=hashlib.sha256(archive.read_bytes()).hexdigest())))


if __name__=='__main__':
    main()
