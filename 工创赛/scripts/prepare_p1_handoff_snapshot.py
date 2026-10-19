"""Copy a byte-preserving, portable P1 handoff snapshot; never edit source."""
from pathlib import Path
from datetime import datetime, timezone
import hashlib
import json
import shutil
import xml.etree.ElementTree as ET

ROOT = Path(__file__).resolve().parents[1]
SOURCE = Path('D:/新建文件夹4/工创赛/代码/stm32/3.stm32f407工程')
DEST = ROOT / 'handoff/p1_2026-10-15'
if DEST.exists():
    raise SystemExit('Snapshot already exists. Verify it instead of overwriting it.')

def sha(p):
    return hashlib.sha256(p.read_bytes()).hexdigest()

allowed_dirs = {'CORE', 'FreeRTOS', 'HARDWARE', 'StdPeriph_Driver', 'SYSTEM', 'USER'}
allowed_ext = {'.c', '.h', '.s', '.inc', '.txt', '.md', '.sct', '.lib'}
selected = []
for p in sorted(SOURCE.rglob('*')):
    if not p.is_file():
        continue
    rel = p.relative_to(SOURCE)
    if any(x.lower() in {'objects','listings','.git','__pycache__'} for x in rel.parts):
        continue
    root_project = len(rel.parts) == 1 and p.suffix.lower() in {'.uvprojx','.uvoptx','.scvd'}
    module_source = rel.parts[0] in allowed_dirs and (p.suffix.lower() in allowed_ext or p.name.lower().startswith(('license','copying')))
    if root_project or module_source:
        selected.append((p, Path('firmware') / rel))

for name in ('gongxun-f407.axf', 'gongxun-f407.build_log.htm', 'gongxun-f407.htm'):
    p = SOURCE / 'Objects' / name
    if p.exists():
        selected.append((p, Path('build_reference') / name))
hmi = Path('D:/新建文件夹4/工创赛/代码/串口屏/工创赛.HMI')
if hmi.exists():
    selected.append((hmi, Path('screen/工创赛.HMI')))

records = []
for src, rel in selected:
    digest = sha(src)
    dst = DEST / rel
    dst.parent.mkdir(parents=True, exist_ok=True)
    shutil.copy2(src, dst)
    assert sha(dst) == digest and sha(src) == digest, str(src)
    records.append({'path': rel.as_posix(), 'source_path': str(src), 'bytes': dst.stat().st_size, 'sha256': digest})

firmware = DEST / 'firmware'
project = ET.parse(firmware / 'gongxun-f407.uvprojx')
refs = []
for e in project.findall('.//FilePath'):
    raw = e.text or ''
    path = firmware / raw.replace('\\','/')
    assert path.resolve().is_relative_to(firmware.resolve()), raw
    assert path.is_file(), raw
    refs.append(raw)
includes = []
for e in project.findall('.//IncludePath'):
    for raw in (e.text or '').split(';'):
        if not raw:
            continue
        path = firmware / raw.replace('\\','/')
        assert path.resolve().is_relative_to(firmware.resolve()), raw
        assert path.is_dir(), raw
        includes.append(raw)

main = (firmware / 'USER/main.c').read_bytes()
can_h = (firmware / 'SYSTEM/CAN/can.h').read_bytes()
assert b'g_vm_test_request = 0U' in main
assert b'g_can_feedback_enable = 0U' in main
assert b'App_CanTask' in main
assert b'CAN1_SetMotorEnable' not in can_h
assert b'CAN1_StopMotor' not in can_h
assert b'CAN1_SetEmmSpeedRaw' not in can_h
watchdog = (firmware / 'SYSTEM/WATCHDOG/watchdog.c').read_bytes()
assert b'#define WATCHDOG_DEBUG_FREEZE 1U' in watchdog

# Recheck every original file after the entire snapshot operation.
for item in records:
    assert sha(Path(item['source_path'])) == item['sha256']
manifest = {
    'snapshot_date': '2026-10-15',
    'created_at_utc': datetime.now(timezone.utc).isoformat(),
    'source_firmware_root': str(SOURCE),
    'firmware_entry': 'firmware/gongxun-f407.uvprojx',
    'copy_policy': 'Byte-preserving source snapshot; excluded Objects/Listings/user GUI state. Build reference stored separately, not an instruction to flash.',
    'files': records,
    'file_count': len(records),
    'total_bytes': sum(r['bytes'] for r in records),
    'project_file_references_checked': len(refs),
    'include_directories_checked': len(includes),
    'all_project_source_paths_internal': True,
    'all_copied_hashes_match': True,
    'original_files_unchanged_after_copy': True,
    'snapshot_state': {'vm_request_default': 0, 'feedback_default': 0, 'watchdog_debug_freeze': 1, 'can_control_interfaces_present': False},
    'validation_boundary': 'File/path/hash verification only; no build, flash, CAN transmission or hardware movement performed.'
}
(DEST / 'manifest.json').write_text(json.dumps(manifest, ensure_ascii=False, indent=2)+'\n', encoding='utf-8')
print(json.dumps({k: manifest[k] for k in ('file_count','total_bytes','project_file_references_checked','include_directories_checked','snapshot_state')},ensure_ascii=False))
