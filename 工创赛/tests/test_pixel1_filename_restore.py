"""模拟中文名被错误解码，恢复缺失模块但不覆盖用户配置或现有源文件。"""
import importlib.util
import json
from pathlib import Path
import tempfile
import unittest
from zipfile import ZipFile, ZIP_DEFLATED

ROOT=Path(__file__).resolve().parents[1]
ARCHIVE=ROOT/'output/p2_vision_generic135_pixel1_20261027.zip'
spec=importlib.util.spec_from_file_location('pixel1_restore', ROOT/'scripts/p2_restore_pixel1_names.py')
repair=importlib.util.module_from_spec(spec);spec.loader.exec_module(repair)


class RestoreTests(unittest.TestCase):
    def setUp(self):
        (ROOT/'tmp').mkdir(exist_ok=True)
        self.temp=tempfile.TemporaryDirectory(dir=ROOT/'tmp')
        self.root=Path(self.temp.name).resolve()
        assert self.root.is_relative_to((ROOT/'tmp').resolve())
        self.addCleanup(self.temp.cleanup)
        self.code=self.root/'vision_app/code';self.code.mkdir(parents=True)
        with ZipFile(ARCHIVE) as z:
            (self.code/'main.py').write_bytes(z.read('vision_app/code/main.py'))
            self.names=[n for n in z.namelist() if n.endswith('.py') and any('\u4e00'<=c<='\u9fff' for c in n)]
            self.garbled=[]
            for name in self.names:
                alias=self.code/Path(name).name.encode('utf8').decode('cp866')
                alias.write_bytes(z.read(name));self.garbled.append(alias)
        self.config=self.code/'config/user.json';self.config.parent.mkdir()
        self.config.write_text('{"user_setting": true}',encoding='utf8')

    def test_repairs_all_four_missing_names_preserving_configs_and_garbled_files(self):
        result=repair.restore_missing(ARCHIVE,self.root)
        self.assertEqual(len(result['restored']),4)
        with ZipFile(ARCHIVE) as z:
            for name in self.names:self.assertEqual((self.root/name).read_bytes(),z.read(name))
        self.assertTrue(all(p.exists() for p in self.garbled))
        self.assertEqual(self.config.read_text(encoding='utf8'),'{"user_setting": true}')

    def test_existing_user_module_is_not_overwritten_and_second_run_is_idempotent(self):
        target=self.code/'颜色标定.py';target.write_bytes(b'# user-edited module\n')
        result=repair.restore_missing(ARCHIVE,self.root)
        self.assertEqual(len(result['restored']),3)
        self.assertEqual(target.read_bytes(),b'# user-edited module\n')
        self.assertFalse(result['existing_preserved'][0]['matches_archive'])
        result=repair.restore_missing(ARCHIVE,self.root)
        self.assertEqual(result['restored'],[])

    def test_source_mismatch_aborts_before_any_restore(self):
        (self.code/'main.py').write_bytes(b'# another release\n')
        with self.assertRaisesRegex(RuntimeError,'版本不一致'):
            repair.restore_missing(ARCHIVE,self.root)
        self.assertFalse((self.code/'颜色标定.py').exists())

    def test_archive_path_escape_is_rejected_without_writing(self):
        bad=self.root/'bad.zip'
        with ZipFile(ARCHIVE) as src, ZipFile(bad,'w',ZIP_DEFLATED) as out:
            out.writestr('vision_app/PIXEL1_MANIFEST.json',src.read('vision_app/PIXEL1_MANIFEST.json'))
            out.writestr('vision_app/code/../../颜色标定.py',b'# bad\n')
        with self.assertRaisesRegex(RuntimeError,'非预期模块路径'):
            repair.restore_missing(bad,self.root)
        self.assertFalse((self.root/'颜色标定.py').exists())


if __name__=='__main__':unittest.main()
