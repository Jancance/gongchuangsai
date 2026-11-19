"""Run on the Pi: install the isolated observer and desktop startup entry."""
import hashlib
import json
from pathlib import Path
import shutil
import subprocess
import zipfile

root = Path('/home/orangepi/vision_tools')
dest = root / 'source_rimpark_20261115'
assert not dest.exists(), 'Preserve any previous installation'
with zipfile.ZipFile(root / 'source_rimpark_20261115.zip') as archive:
    assert all('/' not in name and '\\' not in name for name in archive.namelist())
    archive.extractall(dest)
manifest = json.loads((dest / 'manifest.json').read_text())
for name, digest in manifest.items():
    assert hashlib.sha256((dest / name).read_bytes()).hexdigest() == digest, name
code = Path('/home/orangepi/p2-source-observer-20261106/vision_app/code')
pickup = root / 'material_pickup_top65_20261114.json'
assert hashlib.sha256(pickup.read_bytes()).hexdigest() == '15a93e31c5f268958c98ab3dd8e8a70e8570b2fc072b94252e78607d4d84c910'
proof = dict(files_verified=len(manifest),
    detector_sha256=hashlib.sha256((code / 'turntable_detector.py').read_bytes()).hexdigest(),
    material_config_sha256=hashlib.sha256(pickup.read_bytes()).hexdigest(),
    hsv_sha256=hashlib.sha256((code / 'config/material_color_thresholds.json').read_bytes()).hexdigest(),
    no_hsv_or_geometry_config_written=True, host_start_or_motion_commands=False)
subprocess.run(['/usr/bin/python3', '-m', 'compileall', '-q', str(dest)], check=True)
autostart = Path('/home/orangepi/.config/autostart/source-rimpark.desktop')
assert not autostart.exists(), 'Preserve existing startup entry'
shutil.copyfile(dest / 'source-rimpark.desktop', autostart)
(dest / 'start.sh').chmod(0o755)
with (dest / 'launcher.log').open('ab') as log:
    process = subprocess.Popen(['/bin/bash', str(dest / 'start.sh')],
        stdin=subprocess.DEVNULL, stdout=log, stderr=log, start_new_session=True)
proof.update(startup_entry=str(autostart), launcher_pid=process.pid,
             installed_path=str(dest), launch_requests_cycle=False)
(dest / 'deployment.json').write_text(json.dumps(proof, indent=2) + '\n')
print(json.dumps(proof))
