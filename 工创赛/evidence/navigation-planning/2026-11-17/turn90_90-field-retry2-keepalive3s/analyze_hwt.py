"""Evaluate saved HWT turn data. No COM/probe imports or device writes."""
from pathlib import Path
import json,math
import numpy as np
F=Path(__file__).resolve().parent
b=json.loads((F/'execution_binding.json').read_text('utf-8-sig'));s=json.loads((F/'result_summary.json').read_text('utf-8-sig'));d=json.loads((F/'capture/decoded.json').read_text('utf-8-sig'))
logs=list((F/'serial').glob('*.jsonl'));assert len(logs)==1
rows=[json.loads(v) for v in logs[0].read_text('utf-8-sig').splitlines() if v.strip()];tm=[r for r in rows if r.get('wire_tag')=='TMQ' and r.get('debug_plan_id')==b['plan_id'] and r.get('action_kind')==3]
active=[r for r in tm if r.get('backend_active') and r.get('target_mdeg') is not None];assert active
# First active heading target belongs to this single explicit relative turn.
target=active[0]['target_mdeg']/1000;relative=90 if b['plan_id']==20 else -90;direction=1 if relative>0 else -1
initial=(target-relative+180)%360-180
wrap=lambda x:(x+180)%360-180
h=d['header'];hw=[r for r in d['hwt'] if r['healthy'] and r['valid_mask']&1];assert hw
bytime={r['angle_ms']:r for r in hw};hw=[bytime[t] for t in sorted(bytime)];times=np.array([r['angle_ms'] for r in hw]);angles=np.array([initial+wrap(r['yaw_deg']-initial) for r in hw]);rates=np.array([r['rate_dps'] for r in hw]);errors=angles-(initial+relative)
observed=tm[-1];final_yaw=observed['yaw_mdeg']/1000;tail=[r for r in tm if r['tick_ms']>=h['first_terminal_receiver_ms']]
moving=np.flatnonzero(np.abs(rates)>=1);movement_time=int(times[moving[0]]) if len(moving) else None
stable_start=None;settled=None
for t,e,v in zip(times,errors,rates):
 if abs(e)<=.8 and abs(v)<=1:
  if stable_start is None:stable_start=int(t)
  if t-stable_start>=700 and settled is None:settled=int(t)
 else:stable_start=None
result=dict(status='OBSERVED_NOT_ABSOLUTE_GROUND_CALIBRATION',relative_angle_goal_deg=relative,reference_peak_deg_s=90,initial_from_target_deg=initial,target_deg=target,capture_peak_abs_HWT_rate_deg_s=float(max(abs(rates))),peak_rate_sampling='Retained HWT rate packets; finite-rate sample peak, not continuous-time maximum',capture_max_overshoot_deg=float(max(direction*errors)),last_serial_error_deg=wrap(final_yaw-target),last_serial_rate_deg_s=observed['rate_mdps']/1000,first_motion_source_ms=movement_time,first_stable_700ms_source_ms=settled,motion_to_stable_s=(settled-movement_time)/1000 if settled is not None and movement_time is not None else None,powered_tail_yaw_range_deg=max((wrap(r['yaw_mdeg']/1000-target) for r in tail),default=0)-min((wrap(r['yaw_mdeg']/1000-target) for r in tail),default=0),capture_quality=h['quality'],raw_source_drops={k:h[k] for k in ['wheel_dropped','hwt_dropped','od_dropped']},serial_bad_frames=s['invalid_frames'],serial_telemetry_dropped=observed.get('telemetry_dropped'),ground_90deg_verified=False,rotation_center_translation_known=False,automatic_reverse=False)
(F/'hwt_turn_metrics.json').write_text(json.dumps(result,ensure_ascii=False,indent=2),encoding='utf-8');print(json.dumps(result,ensure_ascii=False,indent=2))
