"""Regression for the board unzip failure, without camera or serial access."""
import hashlib
import importlib.util
import json
from pathlib import Path
import struct
import tempfile
import unittest
from zipfile import ZipFile
import zlib

ROOT=Path(__file__).resolve().parents[1]
spec=importlib.util.spec_from_file_location('vision_package',ROOT/'scripts/package_integrated_vision.py')
package=importlib.util.module_from_spec(spec)
spec.loader.exec_module(package)


class VisionPackageTests(unittest.TestCase):
    def test_unicode_path_survives_legacy_cp866_name_conversion(self):
        name='vision_app/code/颜色标定.py'
        with tempfile.TemporaryDirectory() as temp:
            archive=Path(temp)/'test.zip'
            with ZipFile(archive,'w') as z:
                z.writestr(package.portable_member(name),b'example')
            with ZipFile(archive) as z:
                info=z.getinfo(name)
                self.assertTrue(info.flag_bits & 0x800)
                raw=info.filename.encode('utf-8')
                # The posted log exactly matches decoding these bytes as CP866.
                self.assertNotEqual(raw.decode('cp866'),name)
                field,length=struct.unpack_from('<HH',info.extra)
                version,crc=struct.unpack_from('<BI',info.extra,4)
                self.assertEqual((field,version),(0x7075,1))
                self.assertEqual(crc,zlib.crc32(raw))
                self.assertEqual(info.extra[9:4+length].decode('utf-8'),name)
            # The same Unicode field must be present in the local header.
            blob=archive.read_bytes()
            name_len,extra_len=struct.unpack_from('<HH',blob,26)
            self.assertEqual(blob[30+name_len:30+name_len+extra_len],info.extra)

    def test_package_preserves_source_and_manifest_hashes(self):
        with tempfile.TemporaryDirectory() as temp:
            archive=Path(temp)/'app.zip'
            items=package.build_package(archive)
            with ZipFile(archive) as z:
                manifest=json.loads(z.read('DEPLOY_MANIFEST.json'))
                self.assertFalse(manifest['hardware_acceptance'])
                self.assertEqual(manifest['files'],items)
                for item in items:
                    self.assertEqual(hashlib.sha256(z.read(item['file'])).hexdigest(),item['sha256'])
                    source=ROOT/item['file']
                    self.assertEqual(z.read(item['file']),source.read_bytes())
                self.assertIsNone(z.testzip())

    def test_python_missing_only_repair_restores_tools_preserves_config(self):
        with tempfile.TemporaryDirectory() as temp:
            archive=Path(temp)/'app.zip'
            dest=Path(temp)/'installed'
            package.build_package(archive)
            cfg=dest/'vision_app/code/config/material_color_thresholds.json'
            cfg.parent.mkdir(parents=True)
            cfg.write_bytes(b'USER MODIFIED CONFIG')
            wrong=dest/'vision_app/code/颜色标定.py'.encode('utf-8').decode('cp866')
            wrong.write_bytes(b'previous unzip output')
            with ZipFile(archive) as z:
                for info in z.infolist():
                    if not (dest/info.filename).exists():
                        z.extract(info,dest)
            self.assertEqual(cfg.read_bytes(),b'USER MODIFIED CONFIG')
            self.assertEqual(wrong.read_bytes(),b'previous unzip output')
            for name in ['颜色标定.py','转盘参数设置.py','物料抓取区域标定.py','叠放位直线标定.py']:
                restored=dest/'vision_app/code'/name
                self.assertEqual(restored.read_bytes(),(ROOT/'vision_app/code'/name).read_bytes())


if __name__=='__main__':
    unittest.main()
