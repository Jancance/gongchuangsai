"""打包独立离线复核工具和已存原图；不覆盖pixel1或部署硬件程序。"""
import hashlib
import json
from pathlib import Path
from zipfile import ZipFile,ZIP_DEFLATED

ROOT=Path(__file__).resolve().parents[1]
VALIDATION=ROOT/'output/partial-ring-candidate-validation-20261027'
APP=ROOT/'output/p2_vision_generic135_pixel1_20261027/vision_app/code'
DEST=ROOT/'output/p2_partial_ring_offline_20261027'


def main():
    report=json.loads((VALIDATION/'review.json').read_text(encoding='utf8'))
    assert report['passed']==report['total']
    files={name:(ROOT/'scripts'/name).read_bytes() for name in ('p2_partial_ring_candidate.py','p2_partial_ring_offline_replay.py')}
    files['ring_station_detector.py']=(APP/'ring_station_detector.py').read_bytes()
    files['ring_station_detector.json']=(APP/'config/ring_station_detector.json').read_bytes()
    for case in report['cases']:
        name='inputs/'+case['name']+'.png'
        files[name]=(VALIDATION/name).read_bytes()
        assert hashlib.sha256(files[name]).hexdigest()==case['input_image_sha256']
    manifest=dict(version=1,files={n:hashlib.sha256(data).hexdigest() for n,data in sorted(files.items())},
                  cases=[dict(name=c['name'],expected_accepted=c['expected_accepted']) for c in report['cases']],
                  source='Preserved pixel1 detector/config plus independent offline candidate and archived/development images.',
                  hardware_acceptance=False,camera_opened=False,motion_allowed=False,metric_valid=False)
    files['manifest.json']=json.dumps(manifest,ensure_ascii=False,indent=2).encode('utf8')
    DEST.mkdir(parents=True,exist_ok=True)
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
                         sha256=hashlib.sha256(archive.read_bytes()).hexdigest(),cases=len(report['cases']))))


if __name__=='__main__':
    main()
