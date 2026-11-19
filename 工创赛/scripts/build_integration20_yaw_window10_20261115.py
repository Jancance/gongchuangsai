"""Build the user-requested10mm candidate after local MATLAB. No hardware IO."""
import json
from pathlib import Path
import shutil

import check_integration18_ext150_20261115 as check
import build_integration18_ext150_20261115 as build

R=Path(__file__).resolve().parents[1]
BASE=R/'tmp/source_top_window5_pause_20261115/project'
PROJECT=R/'tmp/source_top_window10_yaw165_20261115/project'
E=R/'evidence/integration/2026-11-15/integration20-yaw165-window10'

def main():
    model=json.loads((E/'matlab-result.json').read_text(encoding='utf-8'))
    assert model['passed'] and model['local_matlab'] and model['yaw_deg']==[195,165,180]
    assert not PROJECT.exists(),'Preserve an existing candidate'
    shutil.copytree(BASE,PROJECT,ignore=shutil.ignore_patterns('Objects','Listings'))
    p=PROJECT/'USER/vehicle_parameters.h'
    old=p.read_bytes();lines=old.splitlines(keepends=True);changed=0
    for i,line in enumerate(lines):
        if line.startswith(b'#define SOURCE_EXTENSION_CORRIDOR_DEG '):
            end=b'\r\n' if line.endswith(b'\r\n') else b'\n'
            lines[i]=b'#define SOURCE_EXTENSION_CORRIDOR_DEG (10.0f * VEHICLE_EXTENSION_PULSES_PER_MM * 360.0f / 3200.0f) /* User2026-11-15:10mm path window;arrival unchanged. */'+end
            changed+=1
    assert changed==1
    p.write_bytes(b''.join(lines))
    geometry=json.loads((R/'evidence/integration/2026-11-15/integration18/window5-pause/integration19-field/parking-current-matlab.json').read_text(encoding='utf-8'))
    config=dict(yaw_deg=[195.0,165.0,180.0],extensions_mm=[*geometry['current_extensions_mm'][:2],0.0],
        extension_path_window_mm=10,skip_body_alignment=True,one_batch_only=True,
        physical_grasp_verified=False,turntable_stopped_per_user=True)
    (E/'trial-config.json').write_text(json.dumps(config,indent=2)+'\n',encoding='utf-8')
    check.P=PROJECT;check.E=E;check.EXPECTED_CORRIDOR_DEG=10*315.5*360/3200;check.main()
    build.PROJECT=PROJECT;build.BASE=BASE;build.E=E;build.main()

if __name__=='__main__':main()
