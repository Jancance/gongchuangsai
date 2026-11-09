"""复用最新33/94场景，运行当前候选的三机构并行和整车失败出口。"""
from pathlib import Path
import json
import sys
import types

from prepare_grip210_persistent_20261105 import ROOT, STAGE, OUT


def adjust_grip(unit):
    replacements = {'v<=180': 'v<=210', 'last_grip<=180.001f': 'last_grip<=210.001f',
                    'fabsf(last_grip-180)': 'fabsf(last_grip-210)',
                    'deg<=180;': 'deg<=210;', 'fabsf(model_source_grip-180)': 'fabsf(model_source_grip-210)'}
    for old, new in replacements.items():
        unit = unit.replace(old, new)
    return unit


if __name__ == '__main__':
    which = sys.argv[1]
    assert which in ('arm', 'whole')
    prepare = types.ModuleType('prepare')
    prepare.S, prepare.E = STAGE, OUT
    sys.modules['prepare'] = prepare
    sys.path.insert(0, str(ROOT / 'scripts'))
    path = ROOT / f'tmp/integration4_fix/test_{which}.py'
    source = path.read_text(encoding='utf-8')
    source = source.replace("R=Path(r'D:\\Codex\\工创赛')", 'R=Path(' + repr(str(ROOT)) + ')')
    source = source.replace("exec(compile(s,", "s=s.replace(\"p=E/'\", \"unit=adjust_grip(unit)\\np=E/'\")\nexec(compile(s,")
    exec(compile(source, str(path), 'exec'), dict(__name__='__main__', adjust_grip=adjust_grip))
    report = json.loads((OUT / which / 'result.json').read_text())
    assert report['passed']
    unit = (OUT / which / ('source-unit.c' if which == 'arm' else 'whole-unit.c')).read_text(encoding='utf-8')
    assert ('fabsf(last_grip-210)' if which == 'arm' else 'fabsf(model_source_grip-210)') in unit
    report['source_grip_target_deg'] = 210
    report['source_grip_success_asserted_at_210'] = True
    (OUT / which / 'result.json').write_text(json.dumps(report, indent=2), encoding='utf-8')
