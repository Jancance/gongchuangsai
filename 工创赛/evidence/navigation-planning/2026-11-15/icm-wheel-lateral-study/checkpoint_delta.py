"""Freeze this task's exact delta; no Git staging, commit, apply or networking."""
from pathlib import Path
import base64, difflib, hashlib, json

R=Path.cwd(); E=Path(__file__).resolve().parent
paths=[
 '3.stm32f407工程/CHASSIS/USER/main.c',
 '3.stm32f407工程/CHASSIS/USER/app_serial_runtime.c',
 '3.stm32f407工程/gongxun-f407.uvprojx',
 '工创赛/scripts/vehicle_uart_capture.py',
]
new_paths=['3.stm32f407工程/CHASSIS/USER/app_icm_observer.c',
           '3.stm32f407工程/CHASSIS/USER/app_icm_observer.h']
def sha(b):return hashlib.sha256(b).hexdigest()
def main():
    after=E/'after';after.mkdir(exist_ok=True)
    patch=bytearray();manifest={}; sources={}
    for path in paths+new_paths:
        p=Path(path);before=E/'before'/p.name
        old=before.read_bytes() if path in paths else b''
        new=(R/p).read_bytes()
        (after/p.name).write_bytes(new)
        encoded=path.encode('utf8')
        patch.extend(b'diff --git a/'+encoded+b' b/'+encoded+b'\n')
        if path in new_paths:patch.extend(b'new file mode 100644\n')
        patch.extend(b''.join(difflib.diff_bytes(difflib.unified_diff,
            old.splitlines(keepends=True),new.splitlines(keepends=True),
            fromfile=b'a/'+encoded if path in paths else b'/dev/null',
            tofile=b'b/'+encoded,lineterm=b'\n')))
        manifest[path]={'before_sha256':sha(old) if path in paths else None,
            'after_sha256':sha(new),'before_snapshot':str(before.relative_to(E)) if path in paths else None,
            'after_snapshot':str((after/p.name).relative_to(E)),
            'bytes':len(new),'CRLF':new.count(b'\r\n'),'LF':new.count(b'\n')}
        sources[path]={'before_base64':base64.b64encode(old).decode('ascii') if path in paths else None,
            'after_base64':base64.b64encode(new).decode('ascii'),
            'before_sha256':sha(old) if path in paths else None,'after_sha256':sha(new)}
    (E/'integration.patch').write_bytes(patch)
    (E/'checkpoint_sources.json').write_text(json.dumps(sources,ensure_ascii=False,indent=2),'utf8')
    report={'status':'FROZEN_LOCAL_DELTA_NOT_FULL_CHECKOUT_CHECKPOINT',
        'paths':manifest,'patch_sha256':sha(patch),'script_sha256':sha(Path(__file__).read_bytes()),
        'note':'Four before snapshots include pre-existing uncommitted work; patch only adds observer integration. No staging/apply/commit/network performed.'}
    (E/'checkpoint_delta_manifest.json').write_text(json.dumps(report,ensure_ascii=False,indent=2),'utf8')
    print(json.dumps({'status':report['status'],'files':len(manifest),'patch_bytes':len(patch),'patch_sha256':sha(patch)}))
if __name__=='__main__':main()
