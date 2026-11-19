"""Isolated deep-Z candidate after the local MATLAB gate; no hardware access."""
from pathlib import Path
import json,shutil
R=Path(__file__).resolve().parents[1]
BASE=R/'tmp/stage3_dispatch_20261116/project'
P=R/'tmp/stage3_deep_z_20261116/project'
E=R/'evidence/integration/2026-11-16/stage3-deep-z'
S=R/'src/stage3_deep_z_20261116'
changed=[]

def edit(name,pairs):
    path=P/name;data=path.read_bytes()
    for old,new in pairs:
        old=old.encode('ascii');new=new.encode('ascii')
        assert data.count(old)==1,(name,old,data.count(old))
        data=data.replace(old,new,1)
    path.write_bytes(data);changed.append(name)
    dst=S/name;dst.parent.mkdir(parents=True,exist_ok=True);dst.write_bytes(data)

def main():
    proof=json.loads((E/'matlab-result.json').read_text())
    assert proof['passed'] and len(proof['checks'])==9
    assert proof['checks'][8]['name']=='already_at_real_top_skips_redundant_return'
    assert 'genericIT3 Z82000p1846.4RPM/600' in proof['checks'][1]['details']
    assert not P.exists(),'Preserve existing candidate'
    shutil.copytree(BASE,P,ignore=shutil.ignore_patterns('Objects','Listings'))
    edit('CHASSIS/USER/source_parallel_flow.h',[
      ('#define SPF_MAX_LIFT_PULSES 29700U',
       '#define SPF_MAX_LIFT_PULSES 29700U\r\n#define SPF_STAGE3_MAX_LIFT_PULSES 82000U\r\n#define SPF_STAGE3_EXT_RPM 106.0f'),
      (' uint32_t pose_version,stepper_accel_rpm_s;',
       ' uint32_t pose_version,stepper_accel_rpm_s;\r\n /* Immutable per-command scope: zero keeps the source profile. */\r\n uint32_t lift_limit_pulses;')])
    edit('CHASSIS/USER/source_parallel_flow.c',[
      ('static uint32_t due(uint32_t now,uint32_t deadline)',
       'static uint32_t pose_valid_limit(const SPF_Pose *p,uint32_t limit)\n{return p && p->dm_deg>=0 && p->dm_deg<=270 && p->extension_mm>=0 &&\n p->extension_mm<=170 && p->lift_pulses>=0 && p->lift_pulses<=limit;}\nstatic uint32_t due(uint32_t now,uint32_t deadline)'),
      ('!(c->axes_mask&7) || (c->axes_mask&~7U) || !pose_valid(&c->goal) ||\n  c->dm_deg_s!=SPF_DM_DEG_S || c->extension_rpm!=SPF_EXT_RPM ||',
       '!(c->axes_mask&7) || (c->axes_mask&~7U) ||\n  (c->lift_limit_pulses && c->lift_limit_pulses!=SPF_STAGE3_MAX_LIFT_PULSES) ||\n  !pose_valid_limit(&c->goal,c->lift_limit_pulses?c->lift_limit_pulses:SPF_MAX_LIFT_PULSES) ||\n  c->dm_deg_s!=SPF_DM_DEG_S ||\n  c->extension_rpm!=(c->lift_limit_pulses?SPF_STAGE3_EXT_RPM:SPF_EXT_RPM) ||')])
    edit('CHASSIS/USER/vehicle_source_parallel.inc',[
      ('((float)SPF_MAX_LIFT_PULSES*360.0f/3200.0f+0.1f)',
       '((float)(c->lift_limit_pulses?c->lift_limit_pulses:SPF_MAX_LIFT_PULSES)*360.0f/3200.0f+0.1f)')])
    edit('CHASSIS/USER/integration10_complete.inc',[
      ('    uint32_t axes=g_it_axes;',
       '    uint32_t axes=g_it_axes;\r\n    uint32_t stage3_scope=Stage3_Owns();\r\n    uint32_t lift_limit=stage3_scope?SPF_STAGE3_MAX_LIFT_PULSES:SPF_MAX_LIFT_PULSES;'),
      ('!(g_it_lift_pulses>=0 && g_it_lift_pulses<=(float)SPF_MAX_LIFT_PULSES)',
       '!(g_it_lift_pulses>=0 && g_it_lift_pulses<=(float)lift_limit)'),
      ('((axes&4U) && g_it_place_depth_limit_pulses &&',
       '((axes&4U) && !stage3_scope && g_it_place_depth_limit_pulses &&'),
      ('     it_command.kind=SPF_CMD_TRANSFER;it_command.sequence=++g_it_sequence;',
       '     it_command.kind=SPF_CMD_TRANSFER;it_command.sequence=++g_it_sequence;\r\n     it_command.lift_limit_pulses=stage3_scope?SPF_STAGE3_MAX_LIFT_PULSES:0U;'),
      ('it_command.extension_rpm=g_it_ext_rpm_test?(float)g_it_ext_rpm_test:SPF_EXT_RPM;it_command.lift_rpm=1846.4f;',
       'it_command.extension_rpm=stage3_scope?SPF_STAGE3_EXT_RPM:(g_it_ext_rpm_test?(float)g_it_ext_rpm_test:SPF_EXT_RPM);it_command.lift_rpm=1846.4f;')])
    # Both pick and target teach ranges use this stage3-only configuration.
    path='CHASSIS/USER/stage3_cycle.inc';data=(P/path).read_bytes()
    assert data.count(b'(float)SPF_MAX_LIFT_PULSES')==2
    edit(path,[('g_stage3_pick_depth<=(float)SPF_MAX_LIFT_PULSES','g_stage3_pick_depth<=(float)SPF_STAGE3_MAX_LIFT_PULSES'),
               ('g_stage3_target_depth[j]<=(float)SPF_MAX_LIFT_PULSES','g_stage3_target_depth[j]<=(float)SPF_STAGE3_MAX_LIFT_PULSES'),
               ('  if(g_stage3_phase==6U){g_stage3_state=2U;', '  if(g_stage3_phase==0U && it_at_top()){g_stage3_phase++;return;}\r\n  if(g_stage3_phase==6U){g_stage3_state=2U;'),
               ('  switch(p){','  /* Real referenced feedback satisfies ensure-top; never synthesize LRC DONE. */\r\n  if((p==1U || p==6U || p==11U) && it_at_top()){g_stage3_phase++;return;}\r\n  switch(p){')])
    edit('CHASSIS/USER/lift_ruler_calibration.h',[
      (' uint32_t down_target_pulses;',
       ' uint32_t down_target_pulses;\r\n uint32_t return_limit_pulses; /* Latched only for the explicit return. */'),
      ('uint32_t LiftRuler_StartReturn(LiftRulerController*,uint32_t now,uint32_t ready);',
       'uint32_t LiftRuler_StartReturn(LiftRulerController*,uint32_t now,uint32_t ready);\r\nuint32_t LiftRuler_StartReturnWithLimit(LiftRulerController*,uint32_t now,uint32_t ready,uint32_t limit);')])
    edit('CHASSIS/USER/lift_ruler_calibration.c',[
      ('#include "lift_ruler_calibration.h"','#include "lift_ruler_calibration.h"\r\n#include "source_parallel_flow.h"'),
      ('uint32_t LiftRuler_StartReturn(LiftRulerController*c,uint32_t now,uint32_t ready)\r\n{\r\n uint32_t previous;',
       'uint32_t LiftRuler_StartReturn(LiftRulerController*c,uint32_t now,uint32_t ready)\r\n{return LiftRuler_StartReturnWithLimit(c,now,ready,DOWN_TARGET_PULSES);}\r\nuint32_t LiftRuler_StartReturnWithLimit(LiftRulerController*c,uint32_t now,uint32_t ready,uint32_t limit)\r\n{\r\n uint32_t previous;\r\n if(limit!=DOWN_TARGET_PULSES && limit!=SPF_STAGE3_MAX_LIFT_PULSES)return 0U;'),
      (' if(LiftRuler_Start(c,now,ready))return 1U;',
       ' if(LiftRuler_Start(c,now,ready)){c->return_limit_pulses=limit;return 1U;}'),
      ('absolute(delta)>(int64_t)DOWN_TARGET_PULSES*360*c->scale/3200+2*(int64_t)c->scale',
       'absolute(delta)>(int64_t)((c->operation==2U && !c->backoff_active && c->return_limit_pulses)?c->return_limit_pulses:DOWN_TARGET_PULSES)*360*c->scale/3200+2*(int64_t)c->scale')])
    edit('CHASSIS/USER/lift_ruler_adapter.c',[
      ('#include "lift_ruler_adapter.h"',
       '#include "lift_ruler_adapter.h"\r\n#include "stage3_cycle.h"\r\n#include "source_parallel_flow.h"'),
      (' if(request==3U && !LiftRuler_StartReturn(&g_lift_ruler_controller,now,ready))g_lift_ruler_rejected=1U;',
       ' if(request==3U && !(Stage3_Owns()?\r\n  LiftRuler_StartReturnWithLimit(&g_lift_ruler_controller,now,ready,SPF_STAGE3_MAX_LIFT_PULSES):\r\n  LiftRuler_StartReturn(&g_lift_ruler_controller,now,ready)))g_lift_ruler_rejected=1U;')])
    for n in ['USER/vehicle_parameters.h','CHASSIS/USER/source_mcu_cycle.inc','CHASSIS/USER/pzq_commit_overlap.inc']:
        assert (P/n).read_bytes()==(BASE/n).read_bytes(),n
    print(json.dumps({'project':str(P),'changed':changed,'matlab_passed':True}))

if __name__=='__main__':main()
