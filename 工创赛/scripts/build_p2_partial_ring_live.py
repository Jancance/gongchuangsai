"""完整保留pixel1并添加独立有界实时取证入口，不改原应用文件。"""
import hashlib
import json
from pathlib import Path
from zipfile import ZipFile,ZIP_DEFLATED

ROOT=Path(__file__).resolve().parents[1]
BASE=ROOT/'output/p2_vision_generic135_pixel1_20261027/vision_app'
DEST=ROOT/'output/p2_partial_ring_live1_20261027'


def main():
    originals={p.relative_to(BASE).as_posix():p.read_bytes() for p in BASE.rglob('*') if p.is_file() and '__pycache__' not in p.parts and p.suffix!='.pyc'}
    files={'vision_app/'+name:data for name,data in originals.items()}
    for source,target in [('p2_partial_ring_candidate_v2.py','p2_partial_ring_candidate.py'),
                          ('p2_ring_raw_shared.py','p2_ring_raw_shared.py'),
                          ('p2_partial_ring_live_gate.py','p2_partial_ring_live_gate.py'),
                          ('p2_partial_ring_live_audit.py','p2_partial_ring_live_audit.py')]:
        data=(ROOT/'scripts'/source).read_bytes()
        compile(data,target,'exec')
        files[target]=data
    manifest=dict(version=1,base=str(BASE.relative_to(ROOT)),
                  files={name:hashlib.sha256(data).hexdigest() for name,data in sorted(files.items())},
                  original_files_preserved=len(originals),changed_original_files=[],
                  camera_profile='generic_135_1920x1080_20261026_v1',maximum_host_age_ms=150,
                  motion_allowed=False,metric_valid=False,hardware_acceptance=False,
                  scope='Independent bounded live audit. Original full application files retained byte for byte.')
    files['LIVE_MANIFEST.json']=json.dumps(manifest,ensure_ascii=False,indent=2).encode('utf8')
    DEST.mkdir(exist_ok=True)
    for name,data in files.items():
        path=DEST/name;path.parent.mkdir(parents=True,exist_ok=True);path.write_bytes(data)
    archive=DEST.with_suffix('.zip')
    with ZipFile(archive,'w',ZIP_DEFLATED) as z:
        for name,data in sorted(files.items()):z.writestr(DEST.name+'/'+name,data)
    with ZipFile(archive) as z:
        assert z.testzip() is None
        for name,data in files.items():assert z.read(DEST.name+'/'+name)==data
    for name,data in originals.items():assert (DEST/'vision_app'/name).read_bytes()==data==(BASE/name).read_bytes()
    print(json.dumps(dict(archive=str(archive),bytes=archive.stat().st_size,
                         sha256=hashlib.sha256(archive.read_bytes()).hexdigest(),original_files=len(originals))))


if __name__=='__main__':
    main()
