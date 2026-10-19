"""Offline, read-only verification of the portable P1 handoff package."""
from pathlib import Path
import hashlib
import json
import re
import xml.etree.ElementTree as ET

root = Path(__file__).resolve().parents[1]
bundle = root / 'handoff/p1_2026-10-15'
manifest = json.loads((bundle / 'manifest.json').read_text(encoding='utf-8'))
for record in manifest['files']:
    path = (bundle / record['path']).resolve()
    assert path.is_relative_to(bundle.resolve()), record['path']
    assert path.is_file(), record['path']
    assert path.stat().st_size == record['bytes'], record['path']
    assert hashlib.sha256(path.read_bytes()).hexdigest() == record['sha256'], record['path']

firmware = bundle / 'firmware'
project = ET.parse(firmware / 'gongxun-f407.uvprojx')
for e in project.findall('.//FilePath'):
    p = (firmware / e.text.replace('\\', '/')).resolve()
    assert p.is_relative_to(firmware.resolve()) and p.is_file(), e.text
for e in project.findall('.//IncludePath'):
    for raw in (e.text or '').split(';'):
        if raw:
            p = (firmware / raw.replace('\\', '/')).resolve()
            assert p.is_relative_to(firmware.resolve()) and p.is_dir(), raw

doc = root / 'P1电控组交接_2026-10-15.md'
links = re.findall(r'\[[^\]]*\]\(([^)]+)\)', doc.read_text(encoding='utf-8'))
for link in links:
    assert '://' not in link and not re.match(r'^[A-Za-z]:', link), link
    assert (doc.parent / link.split('#',1)[0]).exists(), link
assert len(manifest['files']) == manifest['file_count']
print(json.dumps({'files_verified':len(manifest['files']), 'handoff_links_verified':len(links), 'project_file_paths_checked':len(project.findall('.//FilePath')), 'source_files_or_hardware_modified':False, 'status':'PASS'},ensure_ascii=False))
