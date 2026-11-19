"""Offline actual model tests; does not connect camera, probe or vehicle."""
from pathlib import Path
import importlib.util,itertools,json,math
import numpy as np
R=Path(__file__).resolve().parents[1];E=R/'evidence/integration/2026-11-11/source-view-entry/triangle-feasibility'
assert json.loads((E/'matlab-result.json').read_text())['passed']
spec=importlib.util.spec_from_file_location('source_triangle_pose',R/'software_interfaces/source_station/source_triangle_pose.py')
import sys
module=importlib.util.module_from_spec(spec);sys.modules[spec.name]=module;spec.loader.exec_module(module)
data=json.loads((E.parent/'clean-disk-review/matlab-result.json').read_text())
template=np.array(data['target_source_frame_candidate_mm']);count=0
for deg in range(-180,180,5):
    c=math.cos(math.radians(deg));s=math.sin(math.radians(deg));rotation=np.array([[c,s],[-s,c]])
    expected=template@rotation+[13,-8]
    for slots in [(0,1,2),(0,1),(0,2),(1,2)]:
        pose=module.fit_observed_slots(template,{k:expected[k] for k in slots})
        assert np.allclose(pose.centers_mm,expected,atol=1e-9)
        assert set(pose.predicted_slots)==set(range(3))-set(slots)
        alignment=module.alignment_in_source_frame(pose)
        assert not alignment['motion_allowed'] and not alignment['grasp_allowed'];count+=1
for order in itertools.permutations(range(3)):
    remaining=set(range(3))
    for picked in order:
        if len(remaining)>=2:
            pose=module.fit_observed_slots(template,{k:template[k] for k in remaining})
            assert module.alignment_in_source_frame(pose)['near_slot']==2;count+=1
        remaining.remove(picked)
def rejected(observations):
    global count
    try:module.fit_observed_slots(template,observations)
    except ValueError:count+=1;return
    raise AssertionError('unsafe input accepted')
rejected({0:template[0]});rejected({0:[float('nan'),0],1:template[1]})
rejected({0:template[0],1:template[0]+1.5*(template[1]-template[0])});rejected({0:template[0],3:template[1]})
rejected({0:template[0],1:template[0]})
pose=module.fit_observed_slots(template,{k:template[k] for k in range(3)})
alignment=module.alignment_in_source_frame(pose)
matlab=json.loads((E/'matlab-result.json').read_text())
assert abs(alignment['yaw_error_source_deg']-matlab['median_axis_angle_source_deg'])<1e-8
assert np.allclose(alignment['pivot_target_shift_source_mm'],matlab['conditional_arm_pivot_shift_source_mm'])
count+=1
# Real contact arcs: reuse actual MATLAB-fitted estimates only in offline replay.
# Positions are first brought into the declared same plane, then rigid template fit.
H=np.array(data['source_frame_T_candidate']);real=[]
for image in sorted(set(x['image'] for x in matlab['real_contact_cases'])):
    cases=[x for x in matlab['real_contact_cases'] if x['image']==image]
    obs={}
    for k,item in enumerate(cases):
        p=np.r_[item['estimated_center_plane_mm'],1]@H.T;obs[k]=p[:2]/p[2]
    pose=module.fit_observed_slots(template,obs)
    real.append(dict(image=image,fit_residual_rms_mm=pose.residual_rms_mm,
                     alignment=module.alignment_in_source_frame(pose)))
    count+=1
report=dict(passed=True,checks=count,hardware_access=False,real_static_prior_seeded_replays=real,
            unknown_phase_acquisition_verified=False,side_top_center_is_not_base_center=True,
            predictions_not_presence=True,body_motion_enabled=False)
(E/'pose-checks.json').write_text(json.dumps(report,indent=2)+'\n')
print(json.dumps(report,indent=2))
