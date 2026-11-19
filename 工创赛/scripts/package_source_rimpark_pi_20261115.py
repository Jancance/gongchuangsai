"""Stage the validated Pi observer without changing live HSV settings."""
import hashlib
import json
from pathlib import Path
import shutil
import zipfile

R = Path(__file__).resolve().parents[1]
E = R / 'evidence/integration/2026-11-15/source-rim-reacquire'
P = R / 'output/source_rimpark_pi_20261115'


def main():
    assert json.loads((E / 'ready-status.json').read_text())['offline_checks_passed']
    P.mkdir(exist_ok=True)
    manifest = json.loads((R / 'evidence/integration/2026-11-15/source-continuous/pi-delivery-manifest.json').read_text())
    sources = {name: R / row['source'] for name, row in manifest.items()}
    sources.update({'adaptive_source_rim_20261115.py': R / 'output/adaptive_source_rim_20261115.py',
                    'rim-detector.json': E / 'actual-turntable-detector.json'})
    for name, source in sources.items():
        shutil.copyfile(source, P / name)
    # The existing installed camera application supplies its unchanged HSV,
    # pickup geometry and turntable_detector.py; do not overwrite those files.
    launcher = '''#!/bin/bash
set -eu
cd /home/orangepi/vision_tools/source_rimpark_20261115
export DISPLAY=${DISPLAY:-:0}
export XAUTHORITY=${XAUTHORITY:-/home/orangepi/.Xauthority}
export QT_QPA_PLATFORM=xcb
exec 9>/home/orangepi/.cache/source-rimpark.lock
flock -n 9 || exit 0
session="captures/run-$(date +%Y%m%d-%H%M%S)"
mkdir -p captures
exec /usr/bin/python3 -u source_mcu_cycle_view_20261115.py \\
 --body-profile body-profile.json \\
 --material-pickup-config /home/orangepi/vision_tools/material_pickup_top65_20261114.json \\
 --session-dir "$session" --continuous-pick --prediction-config prediction-config.json \\
 --rim-detector-config rim-detector.json "$@" >> "$session.log" 2>&1
'''
    (P / 'start.sh').write_text(launcher, encoding='utf-8', newline='\n')
    desktop = '''[Desktop Entry]
Type=Application
Name=原料区自主视觉
Exec=/bin/bash /home/orangepi/vision_tools/source_rimpark_20261115/start.sh
Terminal=false
X-GNOME-Autostart-enabled=true
'''
    (P / 'source-rimpark.desktop').write_text(desktop, encoding='utf-8', newline='\n')
    hashes = {f.name: hashlib.sha256(f.read_bytes()).hexdigest()
              for f in P.iterdir() if f.is_file() and f.name != 'manifest.json'}
    (P / 'manifest.json').write_text(json.dumps(hashes, indent=2) + '\n')
    (E / 'pi-delivery-manifest.json').write_text(json.dumps(hashes, indent=2) + '\n')
    archive = R / 'output/source_rimpark_pi_20261115.zip'
    with zipfile.ZipFile(archive, 'w', zipfile.ZIP_DEFLATED) as z:
        for f in P.iterdir():
            if f.is_file():
                z.write(f, f.name)
    print('Pi package ready:', archive)


if __name__ == '__main__':
    main()
