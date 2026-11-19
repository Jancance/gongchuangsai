"""Build only the requested pin/channel migration; preserve motion and radar sources."""
import json
from pathlib import Path
import build_top_pick_lead8_20261114 as build

def main():
    r=Path(__file__).resolve().parents[1]
    build.B=r/'tmp/source_top_pick_lead8_20261114/project'
    build.P=r/'tmp/source_top_pick_pb9_20261114/project'
    build.E=r/'evidence/integration/2026-11-14/top-pick-pb9'
    changed=[]
    for f in build.P.rglob('*'):
        rel=f.relative_to(build.P)
        if not f.is_file() or any(x in ('Objects','Listings') for x in rel.parts):continue
        if f.read_bytes()!=(build.B/rel).read_bytes():changed.append(rel.as_posix())
    assert changed==['HARDWARE/GRIPPER/gripper.c'],changed
    data=(build.P/changed[0]).read_bytes()
    assert b'TIM4' in data and b'GPIO_Pin_9' in data and b'TIM_SetCompare4' in data
    assert b'TIM_DeInit' not in data and b'TIM_Cmd(TIM4, DISABLE)' not in data
    assert b'TIM_CCxCmd(TIM4, TIM_Channel_4, TIM_CCx_Disable)' in data
    build.E.mkdir(parents=True,exist_ok=True)
    # Existing MATLAB geometry/speed evidence remains applicable; no new motion or timing.
    old=r/'evidence/integration/2026-11-14/top-pick-lead8/matlab-result.json'
    (build.E/'matlab-result.json').write_bytes(old.read_bytes())
    build.main()
    (build.E/'scope.json').write_text(json.dumps(dict(changed=changed,pin='PB9',timer='TIM4_CH4',
      user_requested=True,motion_unchanged=True,pwm_period_and_mapping_unchanged=True,
      radar_source_unchanged=True,new_matlab_run=False,reused_model=str(old))),encoding='utf-8')

if __name__=='__main__':main()
