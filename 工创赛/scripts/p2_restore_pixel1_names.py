"""从现有pixel1 ZIP只补缺失中文模块，保留已有文件/配置/乱码文件。"""
import argparse
import hashlib
import json
from pathlib import Path, PurePosixPath
import subprocess
import sys
from zipfile import ZipFile


def restore_missing(archive, root):
    archive = Path(archive)
    root = Path(root).resolve()
    code = root/'vision_app/code'
    if not code.is_dir() or not (code/'main.py').is_file():
        raise RuntimeError('候选目录不存在或main.py缺失，停止而不写文件')
    sha = lambda data: hashlib.sha256(data).hexdigest()
    with ZipFile(archive) as z:
        if z.testzip() is not None:
            raise RuntimeError('ZIP CRC检查失败')
        manifest = json.loads(z.read('vision_app/PIXEL1_MANIFEST.json'))
        main_hash = manifest['candidate_files']['code/main.py']
        if sha((code/'main.py').read_bytes()) != main_hash:
            raise RuntimeError('候选main.py与ZIP版本不一致，保留现场，不进行恢复')
        pending = []
        existing = []
        for name in z.namelist():
            if not (name.startswith('vision_app/code/') and name.endswith('.py')
                    and any('\u4e00' <= c <= '\u9fff' for c in name)):
                continue
            parts = PurePosixPath(name).parts
            if len(parts) != 3 or any(p in ('.', '..') for p in parts) or '\\' in name:
                raise RuntimeError('ZIP含非预期模块路径')
            target = root.joinpath(*parts)
            if target.is_symlink() or not target.resolve().is_relative_to(root):
                raise RuntimeError('恢复目标越出候选目录或为符号链接')
            data = z.read(name)
            source_name = '/'.join(parts[1:])
            if sha(data) != manifest['source_files'][source_name]:
                raise RuntimeError('中文模块与原文件manifest哈希不一致')
            if target.exists():
                if not target.is_file():
                    raise RuntimeError('目标已存在且不是普通文件')
                existing.append({'name': parts[-1], 'matches_archive': target.read_bytes() == data})
            else:
                pending.append((target, data))
        if not pending and not existing:
            raise RuntimeError('ZIP未找到可核对的中文模块')
        # 全部来源/路径检查完成后再写；xb避免覆盖已有文件或竞态创建的文件。
        restored = []
        for target, data in pending:
            with target.open('xb') as out:
                out.write(data)
            assert target.read_bytes() == data
            restored.append(target.name)
    return {'restored': restored, 'existing_preserved': existing,
            'root': str(root), 'configs_modified': False,
            'scope': 'Only missing correctly named Chinese Python modules; existing/garbled files preserved.'}


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--archive', type=Path, default=Path('/home/orangepi/p2_vision_generic135_pixel1_20261027.zip'))
    parser.add_argument('--root', type=Path, default=Path('/home/orangepi/p2-generic135-pixel1-20261027'))
    parser.add_argument('--check', action='store_true', help='恢复后只加载候选配置，不开相机/GUI/串口')
    args = parser.parse_args()
    print(json.dumps(restore_missing(args.archive, args.root), ensure_ascii=False, indent=2), flush=True)
    if args.check:
        return subprocess.run([sys.executable, str(args.root/'vision_app/run.py'), '--check',
                               '--start-ring', '--ring-paper-labels', '3,2,1']).returncode
    return 0


if __name__ == '__main__':
    raise SystemExit(main())
