"""Add an explicitly requested CCW half-turn primitive; no boot-time motion."""
from pathlib import Path
import hashlib,json

ROOT=Path(__file__).resolve().parents[1]
STAGE=ROOT/'tmp/stage1_early_scan_startup_20261103'
EV=ROOT/'evidence/integration/2026-11-03/early-scan-startup'

PLAN='''#ifndef DM_CAN2_STARTUP_PLAN_H
#define DM_CAN2_STARTUP_PLAN_H
#include <stdint.h>
#include <math.h>
/* 新鲜原始角与既有显示基准换算；不设置电机零点，不恢复多圈机械参考。
 * 人工约180度出发，经逆时针正向半圈到同圈360度（显示0）。
 * 已在零位容差内不额外绕一整圈，其他起始姿态拒绝。 */
static uint32_t DM_CAN2_StartupCcwDelta(float raw_deg,float zero_deg,float *delta_deg)
{
    float work;
    if(!delta_deg || !(raw_deg>=-720.0f && raw_deg<=720.0f) ||
       !(zero_deg>=-720.0f && zero_deg<=720.0f))return 0U;
    work=fmodf(raw_deg-zero_deg,360.0f);if(work<0.0f)work+=360.0f;
    if(work<=3.0f || work>=357.0f){*delta_deg=0.0f;return 1U;}
    if(!(work>=170.0f && work<=190.0f))return 0U;
    *delta_deg=360.0f-work;return 1U;
}
#endif
'''

def edit(rel,fn):
    p=STAGE/rel;b=p.read_bytes()
    try:s=b.decode('utf-8');enc='utf-8'
    except UnicodeDecodeError:s=b.decode('gbk');enc='gbk'
    p.write_bytes(fn(s).encode(enc))

def motion(s):
    s=s.replace('#include "dm_can2.h"','#include "dm_can2.h"\n#include "dm_can2_startup_plan.h"',1)
    s=s.replace('request == 4U || request == 5U','request == 4U || request == 5U || request == 6U')
    s=s.replace('request != 4U && request != 5U','request != 4U && request != 5U && request != 6U')
    s=s.replace('s_point_goal = request == 4U ? g_dm_pick_target_deg : g_dm_place_target_deg;',
        's_point_goal = request == 6U ? 0.0F : (request == 4U ? g_dm_pick_target_deg : g_dm_place_target_deg);')
    mark='            current_work_deg = DM_CAN2_DisplayRelativeDeg(s_start * JOG_DEG_PER_RAD);'
    lo=s.index(mark);hi=s.index('            s_target = s_start + delta_deg * JOG_RAD_PER_DEG;',lo)
    old=s[lo:hi]
    s=s[:lo]+'''            if(s_point_mode==6U)
            {
                /* 请求6只由按键启动流程发起，逆时针到显示零，不走旧顺时针放料路径。 */
                if(!DM_CAN2_StartupCcwDelta(s_start*JOG_DEG_PER_RAD,DM_CAN2_LEFT_ZERO_DEG,&delta_deg))
                {JogStop(14U,now);return;}
            }
            else
            {
'''+old+'''            }
'''+s[hi:]
    return s

if __name__=='__main__':
    (STAGE/'HARDWARE/DM_CAN2/dm_can2_startup_plan.h').write_text(PLAN,encoding='utf-8')
    edit('HARDWARE/DM_CAN2/dm_can2_motion.c',motion)
    edit('HARDWARE/DM_CAN2/dm_can2.h',lambda s:s.replace('5去放料候选。','5去放料候选；6按键启动专用逆时针回显示0度。'))
    m=json.loads((EV/'manifest.json').read_text(encoding='utf-8'))
    for p in STAGE.rglob('*'):
        if p.is_file() and p.suffix.lower() in {'.c','.h','.s','.inc','.uvprojx'} and not {'Objects','Listings'}.intersection(p.relative_to(STAGE).parts):
            rel=p.relative_to(STAGE).as_posix();h=hashlib.sha256(p.read_bytes()).hexdigest()
            if m['original'].get(rel)!=h:m['changed'][rel]=h
    (EV/'manifest.json').write_text(json.dumps(m,ensure_ascii=False,indent=2),encoding='utf-8')
