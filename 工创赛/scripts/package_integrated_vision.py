"""Build a small portable source package; excludes diagnostics/cache."""
from pathlib import Path
from zipfile import ZipFile,ZipInfo,ZIP_DEFLATED
import hashlib,json,struct,zlib

ROOT=Path(__file__).resolve().parents[1]

def portable_member(name):
    """兼容板端unzip：除UTF-8标志外，提供Info-ZIP Unicode路径字段。"""
    member=ZipInfo(name)
    member.compress_type=ZIP_DEFLATED
    if not name.isascii():
        raw=name.encode('utf-8')
        payload=struct.pack('<BI',1,zlib.crc32(raw))+raw
        member.extra=struct.pack('<HH',0x7075,len(payload))+payload
    return member

def build_package(target,app=None):
    app=Path(app) if app is not None else ROOT/'vision_app'
    target=Path(target)
    target.parent.mkdir(parents=True,exist_ok=True)
    files=[]
    with ZipFile(target,'w',ZIP_DEFLATED) as z:
        for p in sorted(app.rglob('*')):
            if not p.is_file() or '__pycache__' in p.parts or 'diagnostics' in p.parts or p.suffix=='.pyc':continue
            data=p.read_bytes();name='vision_app/'+p.relative_to(app).as_posix()
            z.writestr(portable_member(name),data)
            files.append({'file':name,'sha256':hashlib.sha256(data).hexdigest(),'bytes':len(data)})
        z.writestr('DEPLOY_MANIFEST.json',json.dumps({'built_for':'DECXIN1280x960 desktop','date':'2026-10-04',
            'filename_encoding':'UTF-8 with Info-ZIP Unicode Path (0x7075)',
            'hardware_acceptance':False,'serial_default':'disabled','files':files},ensure_ascii=False,indent=2))
    with ZipFile(target) as z:
        assert z.testzip() is None
        for item in files:assert hashlib.sha256(z.read(item['file'])).hexdigest()==item['sha256']
    return files

if __name__=='__main__':
    target=ROOT/'output/vision_app_decxin_2026-10-04.zip'
    files=build_package(target)
    print('Package:',target,'files:',len(files),'bytes:',target.stat().st_size)
