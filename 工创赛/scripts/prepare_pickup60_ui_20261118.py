"""MATLAB-gated isolated four-code Pi overlay; no firmware or remote edits."""
from pathlib import Path
import json,shutil
R=Path(__file__).resolve().parents[1]
BASE=R/'output/three_frame_warm_overlay_20261118'
DEPS=R/'output/stage23_settle300_5x2_20261117'
P=R/'output/pickup60_ui_overlay_20261118'
E=R/'evidence/integration/2026-11-18/pickup60-ui'
def main():
 assert json.loads((E/'matlab-result.json').read_text())['passed'];assert not P.exists();P.mkdir()
 for n in ('source_fast_cycle_view_20261115.py','source_pickup_window.py','source_cycle_dashboard_20261115.py'):shutil.copy2(BASE/n,P/n)
 shutil.copy2(DEPS/'initial_rotation.py',P/'initial_rotation.py')
 p=P/'initial_rotation.py';s=p.read_text(encoding='utf-8').replace('def __init__(self, config):','def __init__(self, config, required_deg=120):',1)
 s=s.replace("        self.base_center =", "        self.required_deg = float(required_deg)\n        if not math.isfinite(self.required_deg) or not 0 < self.required_deg <= 360:\n            raise ValueError('Invalid observed net-angle requirement')\n        self.base_center =",1)
 s=s.replace('math.radians(120)','math.radians(self.required_deg)').replace('dict(required_deg=120,','dict(required_deg=self.required_deg,');p.write_text(s,encoding='utf-8')
 p=P/'source_pickup_window.py';s=p.read_text(encoding='utf-8').replace('class SourcePickupWindow:', '# 用户可调：仅本原料区首件视觉净转角门槛；其他InitialRotation调用仍默认120度。\nFIRST_PICK_ROTATION_DEG = 60.0\n\n\nclass SourcePickupWindow:',1)
 s=s.replace('def __init__(self, trigger, orbit_config):','def __init__(self, trigger, orbit_config, first_rotation_deg=FIRST_PICK_ROTATION_DEG):',1).replace('InitialRotation(orbit_config)','InitialRotation(orbit_config, required_deg=first_rotation_deg)',1)
 s=s.replace('required_deg=120 if first else 0','required_deg=self.rotation.required_deg if first else 0').replace("'observed_first120'","'observed_first_rotation'");p.write_text(s,encoding='utf-8')
 p=P/'source_fast_cycle_view_20261115.py';s=p.read_text(encoding='utf-8').replace('from source_cycle_dashboard_20261115 import compose','from source_cycle_dashboard_20261115 import compose,draw_pickup_geometry',1)
 s=s.replace('app.draw_material_pickup_detection(pickup,detected,app.material_pickup_detector.config)','draw_pickup_geometry(pickup,value,app.material_pickup_detector.config)',1)
 s=s.replace('initial_rotation=rotation_info)','initial_rotation=rotation_info,pickup_window=pickup_window.diagnostic(),trigger_info=direct.diagnostic(),candidate=prediction)',1)
 s=s.replace("'motor_commands_sent':False,", "'source_first_rotation_required_deg':pickup_window.rotation.required_deg,'pickup_required_frames':direct.required_frames,'pickup_ui_policy':'actual_window_trigger_v1','motor_commands_sent':False,",1)
 p.write_text(s,encoding='utf-8')
 print('PASS prepared isolated4-code overlay; no original file overwritten')
if __name__=='__main__':main()
