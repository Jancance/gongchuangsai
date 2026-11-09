"""安装已验证的两处源码修改；不烧录、不复位、不触发路线。"""
import argparse
import json
import shutil
from prepare_grip210_persistent_20261105 import ACTUAL, STAGE, OUT, FILES, sha, sources


if __name__ == '__main__':
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--execute', action='store_true', required=True)
    parser.parse_args()
    manifest_path = OUT / 'manifest.json'
    manifest = json.loads(manifest_path.read_text(encoding='utf-8'))
    assert not manifest['installed']
    assert sources(ACTUAL) == manifest['baseline'], '实际源码已变化，拒绝覆盖'
    for name, digest in manifest['changed'].items():
        assert sha(STAGE / name) == digest
    for kind, count in [('arm', 33), ('whole', 94)]:
        report = json.loads((OUT / kind / 'result.json').read_text())
        assert report['passed'] and report['cases'] == count and report['source_grip_success_asserted_at_210']
        for name, digest in report['source_sha256'].items():
            assert sha(STAGE / name) == digest
    for target in ['Vehicle_Integrated', 'Vehicle_Offline']:
        assert '0 Error(s), 0 Warning(s)' in (OUT / ('candidate-' + target + '.log')).read_text(errors='replace')
    # 保留当前板端对应的旧AXF，当前不覆盖实际工程的构建产物。
    running_image = ACTUAL / 'Objects/Vehicle_Integrated/Vehicle_Integrated.axf'
    shutil.copy2(running_image, OUT / 'running-image-before.axf')
    shutil.copy2(STAGE / 'Objects/Vehicle_Integrated/Vehicle_Integrated.axf', OUT / 'source-grip210-candidate.axf')
    for name in FILES:
        (ACTUAL / name).write_bytes((STAGE / name).read_bytes())
    after = sources(ACTUAL)
    changed = {name for name in after if after[name] != manifest['baseline'][name]}
    assert changed == set(FILES)
    manifest.update(installed=True, installed_sha256={n: after[n] for n in FILES},
                    protected_source_files_verified=len(after)-2,
                    candidate_image_sha256=sha(OUT / 'source-grip210-candidate.axf'),
                    running_image_sha256=sha(OUT / 'running-image-before.axf'),
                    actual_build_outputs_unchanged=True, flashed=False)
    manifest_path.write_text(json.dumps(manifest, indent=2), encoding='utf-8')
    print(f'Installed 2 source files; preserved {len(after)-2} others; no flash/reset')
