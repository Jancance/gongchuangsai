"""Prepare an isolated candidate only after the local MATLAB simulation."""
from pathlib import Path
import json
import shutil

R = Path(__file__).resolve().parents[1]
E = R / 'evidence/integration/2026-11-15/continuous-arm-pick'
BASE = R / 'tmp/source_mcu_rimpark_20261115/project'
P = R / 'tmp/source_continuous_arm_pick_20261115/project'


def main():
    proof = json.loads((E / 'matlab-result.json').read_text())
    assert proof['passed'] and proof['local_matlab']
    assert not P.exists(), 'Preserve previous candidates'
    shutil.copytree(BASE, P, ignore=shutil.ignore_patterns('Objects', 'Listings'))
    owner = P / 'CHASSIS/USER/pzq_commit_overlap.inc'
    data = owner.read_bytes()

    def change(old, new):
        nonlocal data
        old, new = old.replace('\n', '\r\n').encode(), new.replace('\n', '\r\n').encode()
        assert data.count(old) == 1, old
        data = data.replace(old, new, 1)

    change('return Low_Group(2U,SPF_EXT|SPF_LIFT,low_yaw,low_extension,(float)SOURCE_OVERLAP_HOVER_PULSES);',
           'return Low_Group(4U,SPF_EXT|SPF_LIFT,low_yaw,low_extension,(float)SOURCE_PICK_PULSES);')
    change(' case 2U:\n  Low_PublishLift(0U);\n  if(!Low_Group(4U,SPF_LIFT,low_yaw,low_extension,(float)SOURCE_PICK_PULSES))Low_Fail(8U);\n  break;\n', '')
    change('fmaxf(0.0f,low_extension-SOURCE_LOADED_PARTIAL_RETRACT_MM),(float)SOURCE_RETURN_PULSES)',
           '0.0f,(float)SOURCE_RETURN_PULSES)')
    change('  if(low_extension>0.0f){if(!Low_Group(11U,SPF_EXT,low_yaw,0.0f,(float)SOURCE_RETURN_PULSES))Low_Fail(8U);}\n  else if(!Low_Group(12U,SPF_DM,37.0f,0.0f,(float)SOURCE_RETURN_PULSES))Low_Fail(8U);\n  break;\n case 11U:\n  if(!Low_Group(12U,SPF_DM,37.0f,0.0f,(float)SOURCE_RETURN_PULSES))Low_Fail(8U);',
           '  if(!Low_Group(12U,SPF_DM,37.0f,0.0f,(float)SOURCE_RETURN_PULSES))Low_Fail(8U);')
    owner.write_bytes(data)
    # Actual drive encoder converts motor degrees to turns. Do not reuse the
    # old offline extension conversion that understated travel by tenfold.
    backend = P / 'CHASSIS/USER/vehicle_source_parallel.inc'
    data = backend.read_bytes()
    old = b'#include "source_parallel_flow.h"'
    assert data.count(old) == 1
    data = data.replace(old, old + b'\r\n#include "source_continuous_descent_20261115.h"', 1)
    old = b'  speed=(uint32_t)((address==6?c->extension_rpm:c->lift_rpm)*10.0f+.5f);'
    new = old + b'''\n  if(Low_Busy() && vsp_fast_lift && address==5U && c->kind==SPF_CMD_PICK &&
     c->axes_mask==(SPF_EXT|SPF_LIFT) && c->goal.lift_pulses==(float)SOURCE_PICK_PULSES){
   float clearance=SingleTeach_TopDeg()+VEHICLE_LIFT_DOWN_SIGN*
     (float)SOURCE_OVERLAP_HOVER_PULSES*360.0f/3200.0f;
   float matched=SourcePick_ContinuousRPM(fabsf(vsp_axis[0].goal-vsp_axis[0].before)/360.0f,
     fabsf(a->goal-a->before)/360.0f,fabsf(clearance-a->before)/360.0f);
   speed=(uint32_t)(matched*10.0f+0.5f);
  }'''
    assert data.count(old) == 1
    # This original backend has mixed newline styles; preserve every old byte.
    backend.write_bytes(data.replace(old, new, 1))
    (P / 'CHASSIS/USER/source_continuous_descent_20261115.h').write_bytes(
        (R / 'output/source_continuous_descent_20261115.h').read_bytes().replace(b'\r\n', b'\n').replace(b'\n', b'\r\n'))
    print('Prepared continuous descent/full return candidate; no flash/device access')


if __name__ == '__main__':
    main()
