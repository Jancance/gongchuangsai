"""Copy/check the independent stage23 desktop package. Never stop/start robot processes."""
import argparse
import base64
import hashlib
import io
import json
from pathlib import Path
import shlex
import subprocess
import sys
import uuid
import zipfile

ROOT = Path(__file__).resolve().parents[1]
DEFAULT = ROOT / 'output/stage23_desktop_20261116'
DEST = '/home/orangepi/vision_tools/stage23_desktop_20261116'


def package(folder):
    entries = {}
    for path in sorted(folder.rglob('*'), key=lambda p:p.relative_to(folder).as_posix()):
        if not path.is_file() or '__pycache__' in path.parts:
            continue
        relative = path.relative_to(folder)
        if relative.parts[0] in ('records',) or relative.as_posix().startswith(('sample/result/', 'sample/capture-smoke/')):
            continue
        if relative.name == 'manifest.json':
            continue
        entries[relative.as_posix()] = path.read_bytes()
    manifest = {name:hashlib.sha256(data).hexdigest() for name,data in entries.items()}
    entries['manifest.json'] = (json.dumps(manifest,indent=2)+'\n').encode()
    (folder/'manifest.json').write_bytes(entries['manifest.json'])
    output=io.BytesIO()
    with zipfile.ZipFile(output,'w',zipfile.ZIP_DEFLATED) as archive:
        for name,data in entries.items():
            info=zipfile.ZipInfo(name,date_time=(2026,11,16,0,0,0))
            info.compress_type=zipfile.ZIP_DEFLATED
            info.create_system=3
            info.external_attr=(0o100644 << 16)
            archive.writestr(info,data,compress_type=zipfile.ZIP_DEFLATED,compresslevel=9)
    return output.getvalue(),manifest


def remote(helper, script):
    command='python3 -c '+shlex.quote(script)
    result=subprocess.run([sys.executable,str(helper),'command',command],cwd=ROOT,
                          capture_output=True,text=True,encoding='utf-8',errors='replace')
    if result.returncode:
        raise RuntimeError(f'Authorized SSH helper failed (exit {result.returncode}); no processes were changed')
    return result.stdout


def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--package',type=Path,default=DEFAULT)
    parser.add_argument('--ssh-helper',type=Path,default=ROOT/'tmp/integration22_ssh.py')
    parser.add_argument('--install-only',action='store_true',help='Explicit authorized SSH copy/check only; never start/stop')
    parser.add_argument('--prepare-only',action='store_true')
    args=parser.parse_args()
    data,manifest=package(args.package.resolve())
    digest=hashlib.sha256(data).hexdigest()
    zip_path=args.package.parent/'stage23_desktop_20261116.zip'
    zip_path.write_bytes(data)
    print(json.dumps({'archive':str(zip_path),'sha256':digest,'files':len(manifest),'install_only':True}))
    if args.prepare_only or not args.install_only:
        return
    upload='/home/orangepi/vision_tools/.stage23-upload-'+uuid.uuid4().hex+'.zip'
    remote(args.ssh_helper, 'from pathlib import Path; p=Path('+repr(upload)+');p.parent.mkdir(parents=True,exist_ok=True);p.write_bytes(b"")')
    for offset in range(0,len(data),3072):
        block=base64.b64encode(data[offset:offset+3072]).decode('ascii')
        remote(args.ssh_helper,'from pathlib import Path;import base64;p=Path('+repr(upload)+');p.open("ab").write(base64.b64decode('+repr(block)+'))')
    script='''from pathlib import Path
import hashlib,json,zipfile,py_compile
p=Path(UPLOAD); destination=Path(DESTINATION)
assert hashlib.sha256(p.read_bytes()).hexdigest()==EXPECTED
with zipfile.ZipFile(p) as z:
    names=z.namelist()
    assert all(not Path(n).is_absolute() and '..' not in Path(n).parts for n in names)
    manifest=json.loads(z.read('manifest.json'))
    old_manifest=json.loads((destination/'manifest.json').read_text()) if (destination/'manifest.json').is_file() else {}
    assert set(names)==set(manifest)|{'manifest.json'}
    for name,digest in manifest.items():
        assert hashlib.sha256(z.read(name)).hexdigest()==digest
        target=destination/name
        assert not target.exists() or hashlib.sha256(target.read_bytes()).hexdigest() in (digest,old_manifest.get(name)), 'Existing local edits preserved: '+name
    destination.mkdir(parents=True,exist_ok=True)
    z.extractall(destination)
for name,digest in manifest.items():
    assert hashlib.sha256((destination/name).read_bytes()).hexdigest()==digest
    if name.endswith('.py'):py_compile.compile(str(destination/name),doraise=True)
p.unlink()
print(json.dumps({'installed':str(destination),'files':len(manifest),'sha256':EXPECTED,'started':False,'stopped_source':False}))
'''
    script=script.replace('UPLOAD',repr(upload)).replace('DESTINATION',repr(DEST)).replace('EXPECTED',repr(digest))
    output=remote(args.ssh_helper,script)
    # Print only the tool's final structured result, never helper login diagnostics.
    final=[line for line in output.splitlines() if line.startswith('{') and '"installed"' in line]
    if not final:
        raise RuntimeError('Remote verification result missing; installation not claimed')
    print(final[-1])


if __name__=='__main__':main()
