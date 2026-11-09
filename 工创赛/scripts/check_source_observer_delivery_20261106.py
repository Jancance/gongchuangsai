"""Validate reversible install, real A/M replay and no-hardware check entry."""
from pathlib import Path
import hashlib
import json
import shutil
import subprocess
import sys
import cv2
import numpy as np
from install_source_observer_20261106 import install

ROOT = Path(__file__).resolve().parents[1]
PAYLOAD = ROOT / 'output/source-observer-20261106/vision_app'
OUT = ROOT / 'evidence/integration/2026-11-06/source-observer'
WORK = ROOT / 'tmp/source_observer_delivery_validation_20261106'


def run(args, name):
    p = subprocess.run([sys.executable, '-X', 'utf8', str(PAYLOAD / 'run.py'), '--feature', 'source-observe', *args],
                       capture_output=True, text=True, encoding='utf-8', timeout=100)
    (OUT / (name + '.log')).write_text(p.stdout+p.stderr, encoding='utf-8')
    assert p.returncode == 0, p.stdout+p.stderr
    return p.stdout


def main():
    assert not WORK.exists(), 'Preserve prior delivery validation'
    WORK.mkdir(parents=True)
    old = WORK / 'old-app'
    (old / 'code/config').mkdir(parents=True)
    (old / 'run.py').write_text('# existing app unchanged\n', encoding='utf-8')
    for p in (ROOT / 'output/source-workflow-20261105/vision_app/code/config').iterdir():
        if p.is_file() and p.suffix in ('.json', '.yaml', '.yml'):
            shutil.copy2(p, old / 'code/config' / p.name)
    (old / 'code/config/inherited-only.json').write_text('{"fixture":"do-not-lose"}', encoding='utf-8')
    before = {p.relative_to(old).as_posix(): hashlib.sha256(p.read_bytes()).hexdigest()
              for p in old.rglob('*') if p.is_file()}
    installed = WORK / 'installed-app'
    result = install(PAYLOAD, old, installed)
    assert not result['app_launched'] and not result['source_app_modified']
    for n, h in before.items():
        assert hashlib.sha256((old/n).read_bytes()).hexdigest() == h
        if n.startswith('code/config/'):
            assert hashlib.sha256((installed/n).read_bytes()).hexdigest() == h
    checks = 1
    for destination in [installed, old, old / 'nested']:
        try:install(PAYLOAD, old, destination)
        except ValueError:checks += 1
        else:raise AssertionError('existing/source directory was overwritten')
    record = json.loads(run(['--check'], 'bundle-check').strip().splitlines()[-1])
    assert record['serial'] == 'DISABLED' and not record['camera_opened'] and not record['motion_allowed'];checks += 1
    # Synthetic image fits the unchanged placeholder pixel detector's range.
    frame = np.zeros((1080,1920,3), dtype=np.uint8)
    cv2.circle(frame, (960,540), 300, (220,220,220), -1)
    image = WORK / 'synthetic-rim.png'
    cv2.imencode('.png',frame)[1].tofile(image)
    for mode, count in [('A',5),('M',1)]:
        destination = OUT / ('replay-'+mode)
        run(['--mode',mode,'--replay',str(image),'--frames',str(count),
             '--session-dir',str(destination)], 'replay-'+mode)
        session = json.loads((destination/'session.json').read_text(encoding='utf-8'))
        assert session['frames']==count and session['error'] is None and not session['camera_opened'] and not session['motion_allowed']
        assert session['serial']=='DISABLED' and not session['sensor_exposure_age_verified']
        raw = cv2.imdecode(np.fromfile(destination/'first-raw.png', dtype=np.uint8),cv2.IMREAD_COLOR)
        assert np.array_equal(raw, frame)
        for line in (destination/'observations.jsonl').read_text(encoding='utf-8').splitlines():
            observation=json.loads(line);assert not observation['motion_allowed'] and observation['replay']
        checks += 3
    # Runtime and package originals unchanged in code; saved config inheritance
    # is explicit and does not silently replace the old directory.
    for n in ['code/main.py','code/turntable_detector.py','code/material_pickup_detector.py',
              'code/颜色标定.py','code/物料抓取区域标定.py']:
        assert (PAYLOAD/n).read_bytes()==(ROOT/'output/source-workflow-20261105/vision_app'/n).read_bytes();checks+=1
    for n in ['code/source_metric_preview.py','code/source_observe.py','code/source_metric_bridge.py','code/metric_alignment_protocol.py']:
        assert (PAYLOAD/n).read_bytes()==(ROOT/'vision_app'/n).read_bytes();checks+=1
    report={'passed':True,'delivery_checks':checks,'install_report':result,'replay_kind':'synthetic only',
            'camera_or_serial_opened':False,'hardware_motion_commands':0,'A_frames':5,'M_frames':1,
            'original_A_M_and_tools_byte_identical':True,'source_and_delivery_overlay_match':True}
    (OUT/'delivery-checks.json').write_text(json.dumps(report,ensure_ascii=False,indent=2),encoding='utf-8')
    print(json.dumps({'passed':True,'checks':checks,'A_frames':5,'M_frames':1,'config_inheritance_preserved':True}))


if __name__=='__main__':
    main()
