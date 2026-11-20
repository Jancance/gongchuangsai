"""Read-only product checks; write this candidate's verification receipt only."""
from pathlib import Path
import hashlib
import json
import struct
import subprocess

R = Path.cwd()
E = R / '工创赛/evidence/navigation-planning/2026-11-15/s25-field-candidate'
S3 = R / '工创赛/evidence/navigation-planning/2026-11-15/bounded-lateral-enhancement'
P = '3.stm32f407工程/CHASSIS/USER/chassis_auto_test.c'


def sha(data):
    return hashlib.sha256(data).hexdigest()


def allocated(path):
    data = path.read_bytes()
    header = struct.unpack_from('<16sHHIIIIIHHHHHH', data)
    assert header[0][:5] == b'\x7fELF\x01'
    result = []
    for i in range(header[12]):
        _, kind, flags, address, offset, size, *_ = struct.unpack_from(
            '<IIIIIIIIII', data, header[6] + i * header[11])
        if flags & 2 and size:
            content = bytes(size) if kind == 8 else data[offset:offset + size]
            result.append(dict(kind=kind, flags=flags, address=address,
                               size=size, sha256=sha(content)))
    return result


def main():
    baseline = subprocess.check_output(['git', 'show', f'baa80295:{P}'])
    current = (R / P).read_bytes()
    assert sha(baseline) == '368fc6aa0cd2aada7168843c234dce7aa5c8f49512f83d9838ec1fe0226c22b9'
    old = b'lateral_target/3.0f'
    new = b'lateral_target/2.5f'
    assert baseline.count(old) == 2 and current == baseline.replace(old, new)
    assert current.decode('gbk').encode('gbk') == current
    assert current.count(b'\n') == current.count(b'\r\n') == 2047
    prepared = json.loads((S3 / 'preparation_input_manifest.json').read_text('utf-8'))
    final_s3 = json.loads((S3 / 'build_summary.json').read_text('utf-8'))
    s25 = json.loads((E / 'build_summary.json').read_text('utf-8'))
    overrides = {f'3.stm32f407工程/CHASSIS/USER/{name}': digest
                 for name, digest in final_s3['source_sha256'].items()}
    mismatches = []
    for name, record in prepared['source_files'].items():
        expected = overrides.get(name, record['sha256'])
        actual = sha((R / name).read_bytes())
        if actual != expected:
            mismatches.append(dict(path=name, expected=expected, actual=actual))
    assert len(mismatches) == 1 and mismatches[0]['path'] == P, mismatches
    comparisons = {}
    for name, build in s25['builds'].items():
        assert sha(Path(build['axf']).read_bytes()) == build['axf_sha256']
        before = allocated(Path(final_s3['builds'][name]['axf']))
        after = allocated(Path(build['axf']))
        comparisons[name] = dict(allocated_before=before, allocated_after=after,
                                 allocated_identical=before == after)
    assert comparisons['baseline_off_integrated']['allocated_identical']
    report = dict(source_sha256=sha(current), byte_exact_scale_only=True,
                  crlf_count=2047, gbk_preserved=True,
                  source_items_checked=len(prepared['source_files']),
                  mismatches_from_final_s3=mismatches, builds=comparisons,
                  hardware_accessed=False,
                  rollback_scope='baa80295 stores this controller file only; not a complete firmware checkout')
    (E / 'candidate_verification.json').write_text(json.dumps(report, indent=2), 'utf-8')
    print(json.dumps({key: report[key] for key in ['source_sha256', 'byte_exact_scale_only', 'source_items_checked']}))
    print('baseline-off allocated sections identical:', comparisons['baseline_off_integrated']['allocated_identical'])


if __name__ == '__main__':
    main()
