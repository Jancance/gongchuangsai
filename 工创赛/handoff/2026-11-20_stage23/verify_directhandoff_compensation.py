from pathlib import Path
import ast,json
import numpy as np
from types import SimpleNamespace
p=Path('work/stage3_serialhandoff_20261120/ring_row_alignment.py');tree=ast.parse(p.read_text())
method=next(n for c in tree.body if isinstance(c,ast.ClassDef) for n in c.body if isinstance(n,ast.FunctionDef) and n.name=='measured_motion')
ns={'np':np,'angle_error_in_plane':lambda *args:0.0};exec(compile(ast.Module(body=[method],type_ignores=[]),str(p),'exec'),ns)
c=json.loads(Path('work/stage3_serialhandoff_20261120/stage3-vision-13-5.json').read_text())['ring_row_alignment']
a=SimpleNamespace(target=np.array(c['target_center_px']),target_yaw=0,response=np.array(c['response_pixel_per_right_forward_ccw']),affine=np.array(c['nominal_image_to_station_affine']));a.planar=a.response[:2,:2]
ref=np.array(c['integrated_compensation']['reference_center_px']);want=ref+a.planar@np.array([5.,13.]);assert np.allclose(want,a.target)
r=SimpleNamespace(centers_px=[ref,ref,ref],station_yaw_deg=0)
m,e=ns['measured_motion'](a,r);assert np.allclose(m,[5,13,0]);r.centers_px=[a.target]*3;m0,e0=ns['measured_motion'](a,r);assert np.allclose(m0,[0,0,0])
out={'actual_method_extracted':True,'config_target_px':a.target.tolist(),'computed_target_px':want.tolist(),'motion_at_uncompensated_reference':list(m),'motion_at_compensated_target':list(m0),'duplicate_post_alignment_replay':c['integrated_compensation']['post_alignment_replay'],'passed':True,'physical_acceptance':False}
Path('work/directhandoff-actual-compensation-fixture.json').write_text(json.dumps(out,indent=2));print(json.dumps(out))
