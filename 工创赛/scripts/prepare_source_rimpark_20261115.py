"""Preserve e072; prepare the user-requested final clockwise0deg return."""
from pathlib import Path
import json
import shutil

R=Path(__file__).resolve().parents[1]
E=R/'evidence/integration/2026-11-15/source-rim-reacquire'
BASE=R/'tmp/source_mcu_continuous_20261115/project'
P=R/'tmp/source_mcu_rimpark_20261115/project'

def main():
    proof=json.loads((E/'matlab-result.json').read_text())
    assert proof['passed'] and proof['local_matlab'] and proof['final_request']==5
    assert not P.exists(),'Preserve existing candidates'
    shutil.copytree(BASE,P,ignore=shutil.ignore_patterns('Objects','Listings'))
    path=P/'CHASSIS/USER/pzq_commit_overlap.inc';data=path.read_bytes()
    old=b'if(!Low_Group(7U,SPF_DM|SPF_EXT|SPF_LIFT,180.0f,65.0f,(float)SOURCE_OBSERVE_PULSES))Low_Fail(8U);'
    new=b'if(!Low_Group(7U,SPF_DM|SPF_EXT|SPF_LIFT,low_slot==3U?0.0f:180.0f,low_slot==3U?0.0f:65.0f,(float)SOURCE_OBSERVE_PULSES))Low_Fail(8U);'
    assert data.count(old)==1;path.write_bytes(data.replace(old,new,1))
    # Preserve the actual final alignment observation in diagnostics; OC2 has
    # no body measurement and must not overwrite it with its unused zero field.
    path=P/'CHASSIS/USER/source_mcu_cycle.inc';data=path.read_bytes()
    old=b' g_source_cycle_right_milli_mm=(int32_t)dx;g_source_cycle_forward_milli_mm=(int32_t)dy;\r\n g_source_cycle_error_milli_mm=(uint32_t)norm;'
    new=b' if(g_source_cycle_state==2U && sc_valid){\r\n  g_source_cycle_right_milli_mm=(int32_t)dx;g_source_cycle_forward_milli_mm=(int32_t)dy;\r\n  g_source_cycle_error_milli_mm=(uint32_t)norm;\r\n }'
    assert data.count(old)==1;path.write_bytes(data.replace(old,new,1))
    print('Prepared separate final-park candidate: slots1/2 observer180/65; slot3 tray37->0 CW/X0/top600; no hardware access')

if __name__=='__main__':main()
