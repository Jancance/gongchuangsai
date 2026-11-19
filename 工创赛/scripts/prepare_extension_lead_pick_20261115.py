"""Original-speed candidate, prepared only after MATLAB timing validation."""
from pathlib import Path
import json
import shutil

R=Path(__file__).resolve().parents[1]
BASE=R/'tmp/source_mcu_rimpark_20261115/project'
P=R/'tmp/source_extension_lead_pick_20261115/project'
E=R/'evidence/integration/2026-11-15/extension-lead-pick'


def main():
    proof=json.loads((E/'matlab-result.json').read_text())
    assert proof['passed'] and proof['local_matlab']
    assert not P.exists(),'Preserve previous candidates'
    shutil.copytree(BASE,P,ignore=shutil.ignore_patterns('Objects','Listings'))
    # Reuse only the requested single-target/full-return owner change. The
    # superseded candidate's backend speed override/header are NOT copied.
    owner=R/'tmp/source_continuous_arm_pick_20261115/project/CHASSIS/USER/pzq_commit_overlap.inc'
    d=owner.read_bytes();old=b' vsp_fast_lift=(axes&SPF_LIFT)?1U:0U;'
    new=old+b'''\r\n if(phase==4U && axes==(SPF_EXT|SPF_LIFT)){
  vsp_pick_lead_ms=SourcePick_ExtensionLeadMs(g_low_region);
  vsp_group.delayed_mask=SPF_LIFT;vsp_group.release_mask=SPF_EXT;
 }'''.replace(b'\n',b'\r\n').replace(b'\r\r\n',b'\r\n')
    assert d.count(old)==1
    (P/'CHASSIS/USER/pzq_commit_overlap.inc').write_bytes(d.replace(old,new,1))
    leads=proof['config']['extension_lead_ms_by_region']
    header=('/* 用户确认：原速连续下降，伸缩先启动。时序经本机MATLAB验证。 */\n'
        '#ifndef SOURCE_PICK_EXTENSION_LEAD_20261115_H\n#define SOURCE_PICK_EXTENSION_LEAD_20261115_H\n'
        'static unsigned SourcePick_ExtensionLeadMs(unsigned region)\n{\n'
        ' static const unsigned ms[3]={%s};\n return ms[region];\n}\n#endif\n'%','.join(str(int(x))+'U' for x in leads))
    (P/'CHASSIS/USER/source_pick_extension_lead_20261115.h').write_bytes(header.replace('\n','\r\n').encode())
    path=P/'CHASSIS/USER/vehicle_source_parallel.inc';d=path.read_bytes()
    old=b'#include "source_parallel_flow.h"';assert d.count(old)==1
    d=d.replace(old,old+b'\r\n#include "source_pick_extension_lead_20261115.h"',1)
    old=b'static uint32_t vsp_fast_lift;';assert d.count(old)==1
    d=d.replace(old,old+b'\r\nstatic uint32_t vsp_pick_lead_ms;\r\n',1)
    old=b'static void VehicleSourceParallel_Service(void)\n{';assert d.count(old)==1
    helper=b'''static void VSP_ReleasePickDescent(uint32_t now)
{
 if(vsp_pick_lead_ms && vsp_group.state==2U &&
    (vsp_group.started&SPF_EXT) && !(vsp_group.started&SPF_LIFT) &&
    (uint32_t)(now-vsp_group.first_start_ms)>=vsp_pick_lead_ms)
  vsp_group.release_mask|=SPF_LIFT;
}
'''.replace(b'\n',b'\r\n')
    d=d.replace(old,helper+old,1)
    old=b' SPF_GroupService(&vsp_group);g_source_parallel_state=vsp_group.state;';assert d.count(old)==1
    d=d.replace(old,b' VSP_ReleasePickDescent((uint32_t)xTaskGetTickCount());\r\n'+old,1)
    old=b'memset(vsp_axis,0,sizeof(vsp_axis));vsp_dm_posted=0;vsp_fast_lift=0U;vsp_owned=1;';assert d.count(old)==1
    d=d.replace(old,b'vsp_pick_lead_ms=0U;'+old,1)
    path.write_bytes(d)
    print('Prepared original-speed extension-lead candidate; no hardware access')


if __name__=='__main__':main()
