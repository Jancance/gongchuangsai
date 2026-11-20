from pathlib import Path
import json
R=Path.cwd();E=Path(__file__).resolve().parent;U=R/'3.stm32f407工程/CHASSIS/USER';enc=json.loads((E/'encoding_before.json').read_text('utf-8'))
def get(n):return (E/'source_before'/n).read_bytes().decode(enc[n]).replace('\r\n','\n')
def put(n,s):
 (U/n).write_bytes(s.replace('\n','\r\n').encode(enc[n]))
def rep(s,a,b):
 assert a in s,a[:120];return s.replace(a,b,1)
s=get('app_config.h');s=rep(s,'/* Explicit single-turn debug candidate.', '''#ifndef APP_TURN90_90_ENABLE
#define APP_TURN90_90_ENABLE 0U
#endif
#if APP_TURN90_90_ENABLE > 1U
#error Turn90_90_enable_invalid
#endif
#define APP_TURN90_90_ACTIVE (APP_TURN90_90_ENABLE && APP_BASELINE40_ACTIVE)
#if APP_TURN90_90_ENABLE && !APP_BASELINE40_ACTIVE
#error Turn90_90_requires_baseline40_debug
#endif
/* Explicit single-turn debug candidate.''');s=rep(s,'APP_TURN_BRAKE_TRIAL_DPS != 30U','APP_TURN_BRAKE_TRIAL_DPS != 30U && !(APP_TURN90_90_ACTIVE && APP_TURN_BRAKE_TRIAL_DPS == 90U)');s=rep(s,'#define APP_TURN_BRAKE_MAX_RPM ((APP_TURN_BRAKE_TRIAL_DPS == 30U) ? 27U : 13U)','#define APP_TURN_BRAKE_MAX_RPM ((APP_TURN_BRAKE_TRIAL_DPS == 90U) ? 82U : ((APP_TURN_BRAKE_TRIAL_DPS == 30U) ? 27U : 13U))');s=rep(s,'#define APP_TURN_BRAKE_FEEDBACK_RPM ((APP_TURN_BRAKE_TRIAL_DPS == 30U) ? 40U : 30U)','#define APP_TURN_BRAKE_FEEDBACK_RPM ((APP_TURN_BRAKE_TRIAL_DPS == 90U) ? 130U : ((APP_TURN_BRAKE_TRIAL_DPS == 30U) ? 40U : 30U))\n#if APP_TURN90_90_ENABLE && APP_TURN_BRAKE_TRIAL_DPS != 90U\n#error Turn90_90_requires_explicit_90dps\n#endif');put('app_config.h',s)
s=get('app_entry.h');s=rep(s,'#define DEBUG_PLAN_MAX DEBUG_PLAN_LEFT30_40','''#define DEBUG_PLAN_TURN_CCW90_90 20U
#define DEBUG_PLAN_TURN_CW90_90 21U
#define DEBUG_PLAN_MAX DEBUG_PLAN_TURN_CW90_90
#define AppDebugTurn90Plan(id) (APP_TURN90_90_ACTIVE && ((id)==DEBUG_PLAN_TURN_CCW90_90 || (id)==DEBUG_PLAN_TURN_CW90_90))
uint32_t AppDebugTurn90Allowed(uint32_t id,uint32_t rpm,float angle);''');put('app_entry.h',s)
s=get('app_entry.c');s=rep(s,'uint32_t AppDebugPlan40Allowed', '''uint32_t AppDebugTurn90Allowed(uint32_t id,uint32_t rpm,float angle)
{
    return AppDebugTurn90Plan(id) && rpm==82U &&
        ((id==DEBUG_PLAN_TURN_CCW90_90 && angle==90.0f) ||
         (id==DEBUG_PLAN_TURN_CW90_90 && angle==-90.0f));
}
uint32_t AppDebugPlan40Allowed''');s=rep(s,'&& id!=DEBUG_PLAN_LEFT30_40) return 0U;','&& id!=DEBUG_PLAN_LEFT30_40 && !AppDebugTurn90Plan(id)) return 0U;');s=rep(s,'    if (id == DEBUG_PLAN_FORWARD210_40', '''    if (AppDebugTurn90Plan(id)) {
        MotionAction turn={MOTION_TURN,82U,0U,0.0f,0.0f};
        turn.relative_deg=(id==DEBUG_PLAN_TURN_CCW90_90)?90.0f:-90.0f;
        if(!AppDebugTurn90Allowed(id,turn.base_rpm,turn.relative_deg))return 0U;
        actions[0]=turn;*count=1U;return 1U;
    }
    if (id == DEBUG_PLAN_FORWARD210_40''');put('app_entry.c',s)
s=get('app_motion_runtime.c');s=rep(s,'    for (i = 0U; i < MOTION_ACTION_COUNT; i++)','''    if(AppDebugTurn90Plan(g_debug_plan_id) &&
        (MOTION_ACTION_COUNT!=1U || s_motion_actions[0].kind!=MOTION_TURN ||
         !AppDebugTurn90Allowed(g_debug_plan_id,s_motion_actions[0].base_rpm,s_motion_actions[0].relative_deg)))return 0U;
    for (i = 0U; i < MOTION_ACTION_COUNT; i++)''');s=rep(s,'&& g_debug_plan_id==DEBUG_PLAN_DEFAULT &&\n                action->base_rpm','&& (g_debug_plan_id==DEBUG_PLAN_DEFAULT || AppDebugTurn90Plan(g_debug_plan_id)) &&\n                action->base_rpm');s=rep(s,'MOTION_ACTION_COUNT==1U && g_debug_plan_id==DEBUG_PLAN_DEFAULT &&\n              angle_deg==90.0f','MOTION_ACTION_COUNT==1U && (g_debug_plan_id==DEBUG_PLAN_DEFAULT ||\n               AppDebugTurn90Allowed(g_debug_plan_id,max_rpm,(direction==TURN_CCW)?angle_deg:-angle_deg)) &&\n              angle_deg==90.0f');s=rep(s,'g_debug_plan_id==DEBUG_PLAN_LEFT30_40) && s_motion_session_open','g_debug_plan_id==DEBUG_PLAN_LEFT30_40 || AppDebugTurn90Plan(g_debug_plan_id)) && s_motion_session_open');put('app_motion_runtime.c',s)
s=get('app_serial_runtime.c');s=rep(s,'g_debug_plan_id==DEBUG_PLAN_LEFT30_40) && g_combined_test.state','g_debug_plan_id==DEBUG_PLAN_LEFT30_40 || AppDebugTurn90Plan(g_debug_plan_id)) && g_combined_test.state');s=rep(s,'strcmp(p,"PLAN18")==0 || strcmp(p,"PLAN19")==0)','strcmp(p,"PLAN18")==0 || strcmp(p,"PLAN19")==0 ||\n              strcmp(p,"PLAN20")==0 || strcmp(p,"PLAN21")==0)');s=rep(s,'&& g_debug_plan_id!=DEBUG_PLAN_LEFT30_40) ||','&& g_debug_plan_id!=DEBUG_PLAN_LEFT30_40 && !AppDebugTurn90Plan(g_debug_plan_id)) ||');needle='    values[57] = (*AppMotion_Live()->turn)';s=rep(s,needle,'''    if(AppDebugTurn90Plan(g_debug_plan_id)) {
        values[38]=82;values[39]=130;
    }
'''+needle);put('app_serial_runtime.c',s)
s=get('turn_brake.h');s=rep(s,'#include "app_config.h"','#include "app_config.h"\n#if APP_TURN90_90_ACTIVE\n#include "motion_profile.h"\n#endif');s=rep(s,'    float lookahead_s;','''    float lookahead_s;
#if APP_TURN90_90_ACTIVE
    MotionProfile profile;
    uint32_t profile_started,profile_tick;
#endif''');s=rep(s,'    s->speed_limit=speed;s->lookahead_s=0.7f;','''    s->speed_limit=speed;s->lookahead_s=0.7f;
#if APP_TURN90_90_ACTIVE
    {
        MotionProfileLimits limits={90.0f,180.0f,1440.0f};
        s->speed_limit=90.0f;
        if(s->angle==90.0f)(void)MotionProfile_Prepare(&s->profile,90.0f,&limits);
    }
#endif''');needle='    alpha=dt/(0.08f+dt);';s=rep(s,needle,'''#if APP_TURN90_90_ACTIVE
    {
        MotionProfileSample ref;
        float elapsed_s,velocity,goal,available,cap;
        if(!s->profile_started){s->profile_started=1U;s->profile_tick=now;}
        elapsed_s=(float)(now-s->profile_tick)/1000.0f;
        if(!MotionProfile_Sample(&s->profile,elapsed_s,&ref))return 2U;
        velocity=s->direction*rate;
        goal=ref.speed_mm_s+4.0f*(ref.position_mm-observed-0.08f*velocity)+
            0.3f*(ref.speed_mm_s-velocity);
        if(elapsed_s>=s->profile.duration_s)
            goal=fminf(15.0f,2.0f*(s->angle-observed)-0.3f*velocity);
        goal=fmaxf(0.0f,fminf(90.0f,goal));
        available=fmaxf(0.0f,s->angle-observed-0.28f*fmaxf(0.0f,velocity));
        cap=sqrtf(126.5625f+360.0f*available)-11.25f;
        goal=fminf(goal,cap);
        if(elapsed_s>=s->profile.duration_s && fabsf(s->angle-observed)<=0.8f && fabsf(rate)<=1.0f)
            s->stable_ms+=elapsed;
        else s->stable_ms=0U;
        if(s->stable_ms>=700U)return 1U;
        rpm=floorf(goal/(6.0f*77.0f/418.0f)*10.0f+0.000001f)*0.1f;
        if(rpm>81.4f)rpm=81.4f;
        if(rpm>0.0f)s->moved=1U;
        *command=s->direction*rpm;
        return 0U;
    }
#endif
'''+needle);put('turn_brake.h',s)
s=get('heading_session.c');needle='        s->controller.command_rpm = 0;';s=rep(s,needle,'''#if APP_TURN90_90_ACTIVE
        if(!s->brake.profile.valid){HS_Fail(s,HS_CAUSE_INPUT,out);return 0U;}
#endif
'''+needle);put('heading_session.c',s)
print('Wrote seven scoped files, each in original encoding/CRLF')
