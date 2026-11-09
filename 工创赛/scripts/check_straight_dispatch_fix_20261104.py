from pathlib import Path
import os
R=Path(__file__).resolve().parents[1]
os.environ.setdefault('STRAIGHT_SOURCE_ROOT',str(R/'tmp/straight_dispatch_fix_20261104'))
os.environ.setdefault('STRAIGHT_TEST_OUT',str(R/'evidence/integration/2026-11-04/straight-dispatch-fix/after'))
s=(R/'scripts/check_straight210_20261104.py').read_text(encoding='utf-8')
exec(compile(s,str(R/'scripts/check_straight210_20261104.py'),'exec'))
