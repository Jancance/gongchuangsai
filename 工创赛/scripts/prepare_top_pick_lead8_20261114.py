"""Create isolated lead8/top-view revision after local MATLAB validation."""
from pathlib import Path
import json
import shutil

R=Path(__file__).resolve().parents[1]
B=R/'tmp/source_low_readonly_diag_20261114/project'
P=R/'tmp/source_top_pick_lead8_20261114/project'
E=R/'evidence/integration/2026-11-14/top-pick-lead8'

def edit(rel, replacements):
    path=P/rel
    data=path.read_bytes()
    for before,after in replacements:
        before=before.encode('ascii');after=after.encode('ascii')
        assert before in data,(rel,before)
        data=data.replace(before,after)
    path.write_bytes(data)

def main():
    assert json.loads((E/'matlab-result.json').read_text())['passed']
    assert not P.exists(),'Do not overwrite an existing candidate; preserve its flashed source.'
    shutil.copytree(B,P,ignore=shutil.ignore_patterns('Objects','Listings'))
    params=P/'USER/vehicle_parameters.h'
    data=params.read_bytes()
    # Header is UTF-8. Existing bytes/newlines remain unchanged.
    added='''
/* 2026-11-14 用户确认：丝杆2→8mm，仅导程改变；顶部识别抓取。 */
#define SOURCE_LIFT_PULSES_PER_MM 400.0f /* 原1600/4，虚拟脉冲/mm */
#define SOURCE_PICK_PULSES 29700U /* 原118800/4，物理下降74.25mm不变 */
#define SOURCE_RETURN_PULSES 1800U /* 原7200/4，距真实顶部4.5mm */
#define SOURCE_HOME_BACKOFF_PULSES 600U /* 原2400/4，回零退出1.5mm */
#define SOURCE_HOME_EXIT_TOLERANCE_PULSES 80U /* 原0.02mm放宽至0.2mm */
#define SOURCE_TOP_TOLERANCE_DEG 10.0f /* 顶部反馈容差约0.89mm */
#define SOURCE_EXTENSION_DONE_DEG 71.0f /* 到位约2mm，原约1.27mm */
#define SOURCE_EXTENSION_CORRIDOR_DEG 107.0f /* 路径偏差约3mm，原约1.69mm */
/* 本轮不新增保护类别；以后新增保护/故障/门限须用户先同意。 */
'''.replace('\n','\r\n').encode('utf-8')
    # Append before final include-guard close (not at an earlier conditional).
    index=data.rfind(b'#endif')
    assert index>0
    if b'#define SOURCE_LIFT_PULSES_PER_MM' not in data:
        params.write_bytes(data[:index]+added+data[index:])
    u='CHASSIS/USER/'
    for name in ['integration10_complete.inc','integration13_immediate.inc','pzq_region_batch.inc',
                 'pzq_low_pick.inc','lift_ruler_adapter.c','lift_ruler_calibration.c',
                 'source_parallel_flow.h','source_pick_single_action.h']:
        path=P/u/name
        data=path.read_bytes()
        data=data.replace(b'118800.0f',b'29700.0f').replace(b'118800U',b'29700U')
        data=data.replace(b'7200.0f',b'1800.0f').replace(b'7200U',b'1800U')
        path.write_bytes(data)
    # Include the single user-facing parameter table in the independent controller.
    edit(u+'lift_ruler_calibration.c',[
      ('#include "lift_ruler_calibration.h"','#include "lift_ruler_calibration.h"\r\n#include "../../USER/vehicle_parameters.h"'),
      ('#define PRELIFT_PULSES 90000U','#define PRELIFT_PULSES 22500U'),
      ('#define DOWN_TARGET_PULSES 29700U','#define DOWN_TARGET_PULSES SOURCE_PICK_PULSES'),
      ('#define RETURN_TOP_MARGIN_PULSES 1800U','#define RETURN_TOP_MARGIN_PULSES SOURCE_RETURN_PULSES'),
      ('pulses<100000U','pulses<25000U'),
      ('(int64_t)2400U*360','(int64_t)SOURCE_HOME_BACKOFF_PULSES*360'),
      ('(int64_t)3200U*360','(int64_t)800U*360'),
      ('(int64_t)32U*360','(int64_t)SOURCE_HOME_EXIT_TOLERANCE_PULSES*360')])
    edit(u+'integration10_complete.inc',[
      ('safe_position_raw/\r\n       (float)g_lift_ruler_controller.top_scale)<=1.0f',
       'safe_position_raw/\r\n       (float)g_lift_ruler_controller.top_scale)<=SOURCE_TOP_TOLERANCE_DEG'),
      ('depth<=110800.0f','depth<=27700.0f'),('depth<=15200.0f','depth<=3800.0f'),
      ('depth<110800.0f || depth>119200.0f','depth<27700.0f || depth>29800.0f')])
    # Top-view owner keeps the same public transport ABI, removes low descent/return.
    edit(u+'pzq_low_pick.inc',[
      ('{float depth;return Low_ReadDepth(&depth) && fabsf(depth-62400.0f)<=400.0f;}',
       '{float depth;return Low_ReadDepth(&depth) && it_at_top();}'),
      ('fabsf(low_measured_depth-62400.0f)<=400.0f','it_at_top()'),
      ('/1600.0f','/SOURCE_LIFT_PULSES_PER_MM'),
      ('case 8U:\r\n  g_it_axes=SPF_LIFT;g_it_lift_pulses=62400.0f;\r\n  Low_Command(3U,9U,now);break;\r\n case 9U:',
       'case 8U:')])
    # Existing nonzero-extension tolerances, relaxed in millimetres; no drive setting write.
    for name in ['integration10_complete.inc','pzq_low_pick.inc']:
        path=P/u/name;data=path.read_bytes().replace(b':45.0f',b':SOURCE_EXTENSION_DONE_DEG')
        if name=='integration10_complete.inc':
            data=data.replace(b'>45.0f ||',b'>SOURCE_EXTENSION_DONE_DEG ||')
        path.write_bytes(data)
    # Lift's motor-degree tolerance is scaled with lead, retaining bounded 2mm completion.
    edit(u+'vehicle_source_parallel.inc',[
      ('hi+60.0f','hi+SOURCE_EXTENSION_CORRIDOR_DEG'),
      ('lo-60.0f','lo-SOURCE_EXTENSION_CORRIDOR_DEG'),
      ('(zero_return?2.0f:45.0f)',
       '(zero_return?2.0f:(address==6U?SOURCE_EXTENSION_DONE_DEG:22.5f))')])
    path=P/u/'vehicle_source_parallel.inc'
    data=path.read_bytes();start=data.index(b'static uint32_t VSP_PositionWindow(')
    end=data.index(b'static uint32_t VSP_Start(',start)
    data=data[:start]+b'''static uint32_t VSP_PositionWindow(VSP_Axis *a,uint32_t settle_allowed)\r\n{\r\n float lo=fminf(a->before,a->goal),hi=fmaxf(a->before,a->goal);\r\n float corridor=a==&vsp_axis[0]?SOURCE_EXTENSION_CORRIDOR_DEG:45.0f;\r\n (void)settle_allowed;\r\n return a->current>=lo-corridor && a->current<=hi+corridor;\r\n}\r\n'''+data[end:]
    path.write_bytes(data)
    path=P/'USER/vehicle_parameters.h';data=path.read_bytes()
    for old,new in [(b'VEHICLE_LIFT_MAX_TARGET_PULSES 105000.0f',b'VEHICLE_LIFT_MAX_TARGET_PULSES 26250.0f'),
                    (b'VEHICLE_SOURCE_Z_PULSES 105000.0f',b'VEHICLE_SOURCE_Z_PULSES 26250.0f'),
                    (b'VEHICLE_CARRIER_Z_PULSES 28000.0f',b'VEHICLE_CARRIER_Z_PULSES 7000.0f')]:
        data=data.replace(old,new)
    path.write_bytes(data)
    print(P)

if __name__=='__main__':main()
