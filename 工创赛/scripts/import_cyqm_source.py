"""Import the user supplied ZIP once, preserving provenance and file names."""
from pathlib import Path, PurePosixPath
from zipfile import ZipFile
import hashlib
import json

source=Path(r'D:\新建文件夹4\cyqm_vision_gongxun-main.zip')
dest=Path(__file__).resolve().parents[1]/'vision_app'
prefix='cyqm_vision_gongxun-main/'
dest.mkdir(exist_ok=True)
manifest=[]
with ZipFile(source) as archive:
    for info in archive.infolist():
        if info.is_dir():continue
        if not info.filename.startswith(prefix):raise ValueError('Unexpected archive root')
        relative=PurePosixPath(info.filename[len(prefix):])
        if relative.is_absolute() or '..' in relative.parts:raise ValueError('Unsafe ZIP member')
        relative=PurePosixPath('UPSTREAM_README.md') if str(relative)=='README.md' else relative
        if str(relative)=='.gitignore':relative=PurePosixPath('UPSTREAM.gitignore')
        target=dest.joinpath(*relative.parts)
        if not target.resolve().is_relative_to(dest.resolve()):raise ValueError('Path escape')
        if target.exists():raise FileExistsError('Import will not overwrite adapted files: '+str(target))
        data=archive.read(info)
        target.parent.mkdir(parents=True,exist_ok=True);target.write_bytes(data)
        manifest.append({'member':info.filename,'imported_path':str(relative),
                         'sha256':hashlib.sha256(data).hexdigest(),'bytes':len(data)})
        if str(relative).startswith('code/config/'):
            reference=dest/'reference_config'/relative.name
            reference.parent.mkdir(exist_ok=True);reference.write_bytes(data)
(dest/'upstream_manifest.json').write_text(json.dumps({
    'source_zip':str(source),'archive_sha256':hashlib.sha256(source.read_bytes()).hexdigest(),
    'imported_on':'2026-10-03','license_file_in_archive':False,'files':manifest},
    ensure_ascii=False,indent=2),encoding='utf-8')
print('Imported',len(manifest),'files into',dest)
