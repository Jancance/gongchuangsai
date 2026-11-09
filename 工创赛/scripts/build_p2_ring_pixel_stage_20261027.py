"""归档完整原应用、独立v2记录器和阶段证据摘要；不部署或修改应用。"""
import hashlib
import json
from pathlib import Path
from zipfile import ZipFile, ZIP_DEFLATED

ROOT = Path(__file__).resolve().parents[1]
SOURCE = ROOT / 'output/p2_vision_generic135_lightblue1_20261027/vision_app'
DEST = ROOT / 'output/p2_ring_pixel_stage_20261027'


def main():
    files = {}
    for p in sorted(SOURCE.rglob('*')):
        if p.is_file() and '__pycache__' not in p.parts and p.suffix != '.pyc':
            files['vision_app/' + p.relative_to(SOURCE).as_posix()] = p.read_bytes()
    files['tools/p2_ring_pixel_audit_v2_20261027.py'] = (
        ROOT / 'output/p2_ring_pixel_audit_v2_20261027.py').read_bytes()
    docs = ('P2_CAMERA12_PIXEL_HANDOFF_2026-10-27.md', 'P2_RING_OCCLUSION_REVIEW_2026-10-27.md',
            'P2_RING_ROTATION_REVIEW_2026-10-27.md', 'P2_RING_SHADOW_REVIEW_2026-10-27.md',
            'competition_requirements_2027.md')
    for name in docs:
        files['docs/' + name] = (ROOT / 'docs' / name).read_bytes()
    files['README_PIXEL_STAGE.md'] = (ROOT / 'docs/P2_CAMERA12_PIXEL_HANDOFF_2026-10-27.md').read_bytes()
    reports = ('ring-pixel-audit/review.json', 'ring-pixel-audit/review-position2.json',
               'ring-pixel-audit/physical_dimensions_user_report.json', 'ring-occlusion/review.json',
               'ring-rotation/review.json', 'ring-shadow/review.json', 'ring-empty/review.json')
    for name in reports:
        files['evidence_summaries/' + name] = (
            ROOT / 'evidence/p2-camera/2026-10-27' / name).read_bytes()
    profile = json.loads(files['vision_app/code/config/project_profile.json'])
    assert profile['serial_enabled'] is False
    manifest = {
        'artifact': '2026-10-27 ring pixel basic verification archive',
        'application_source': 'output/p2_vision_generic135_lightblue1_20261027/vision_app',
        'original_application_files_preserved': True, 'deployed_by_this_build': False,
        'runtime_source_hash_coverage': ['main.py', 'ring_station_detector.py', 'project_camera.py',
                                         'diagnostic_entry', 'three recorded configs'],
        'runtime_hash_coverage_note': 'Do not infer full live imported-module coverage from the three recorded modules.',
        'geometry_calibration_status': 'Deferred by user; not accepted',
        'motion_allowed': False, 'serial': 'DISABLED',
        'files': {name: {'bytes': len(data), 'sha256': hashlib.sha256(data).hexdigest()}
                  for name, data in sorted(files.items())},
    }
    files['FILE_MANIFEST.json'] = json.dumps(manifest, ensure_ascii=False, indent=2).encode('utf8')
    DEST.mkdir(exist_ok=True)
    for name, data in files.items():
        p = DEST / name
        p.parent.mkdir(parents=True, exist_ok=True)
        p.write_bytes(data)
    archive = DEST.with_suffix('.zip')
    with ZipFile(archive, 'w', ZIP_DEFLATED) as z:
        for name, data in sorted(files.items()):
            z.writestr(name, data)
    with ZipFile(archive) as z:
        assert z.testzip() is None and set(z.namelist()) == set(files)
        for name, data in files.items():
            assert z.read(name) == data == (DEST / name).read_bytes()
    print(json.dumps({'archive': str(archive), 'bytes': archive.stat().st_size,
                      'members': len(files), 'original_app_files': sum(n.startswith('vision_app/') for n in files),
                      'sha256': hashlib.sha256(archive.read_bytes()).hexdigest(),
                      'crc_and_every_member_bytes_verified': True}, ensure_ascii=False))


if __name__ == '__main__':
    main()
