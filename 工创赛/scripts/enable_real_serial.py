# -*- coding: utf-8 -*-
"""Enable the real TurntableSerialLink in the Generic135 vision app."""
import sys, py_compile
from pathlib import Path

p = Path('tmp/pi_sync/main_g135_display.py')
s = p.read_text(encoding='utf-8')

def rep(old, new, tag):
    global s
    n = s.count(old)
    if n != 1:
        print('FAIL %s occurrences=%d' % (tag, n)); sys.exit(1)
    s = s.replace(old, new); print('ok', tag)

rep('from turntable_serial import DEFAULT_SERIAL_RECONNECT_INTERVAL\n',
    'from turntable_serial import DEFAULT_SERIAL_RECONNECT_INTERVAL\n'
    'from turntable_serial import TurntableSerialLink\n', 'import')

rep('    serial_link = DisabledSerialLink(\n',
    '    serial_link = TurntableSerialLink(\n', 'instantiate')

p.write_text(s, encoding='utf-8')
py_compile.compile(str(p), doraise=True)
print('compiled OK')
