from pathlib import Path
import hashlib,json
R=Path('D:/Desktop/工创赛');U=R/'3.stm32f407工程/CHASSIS/USER';E=R/'工创赛/evidence/navigation-planning/2026-11-17/left-nav-residual-prep/medium';B=E/'product_source_before';B.mkdir(exist_ok=True)
files=['app_config.h','app_odometry.c','app_odometry.h','app_icm_observer.c']
for name in files:
 assert not (B/name).exists();(B/name).write_bytes((U/name).read_bytes())
def ch(name,old,new):
 p=U/name;b=p.read_bytes();o=old.replace('\n','\r\n').encode('ascii');n=new.replace('\n','\r\n').encode('ascii');assert b.count(o)==1,(name,b.count(o));p.write_bytes(b.replace(o,n,1))
ch('app_config.h','#define APP_RIGHT40_FORWARD_RATIO 0.01f\n','''#define APP_RIGHT40_FORWARD_RATIO 0.01f
#ifndef APP_LEFT_NAV_RESIDUAL_ENABLE
#define APP_LEFT_NAV_RESIDUAL_ENABLE 0U
#endif
#if APP_LEFT_NAV_RESIDUAL_ENABLE > 1U
#error Left_nav_residual_enable_invalid
#endif
#if APP_LEFT_NAV_RESIDUAL_ENABLE && !APP_BASELINE40_ACTIVE
#error Left_nav_residual_requires_baseline40_debug
#endif
#define APP_LEFT_NAV_RESIDUAL_COEFFICIENT 0.0179055385f
#define APP_LEFT_NAV_RESIDUAL_WEIGHT 0.5f
''')
ch('app_odometry.c','#include <string.h>\n','''#include <string.h>
#if APP_LEFT_NAV_RESIDUAL_ENABLE
#include "app_motion_runtime.h"
#include "app_entry.h"
#include <math.h>
#endif
''')
ch('app_odometry.c','static AppOdometryBaseline40 s_baseline40;\n','''static AppOdometryBaseline40 s_baseline40;
#if APP_LEFT_NAV_RESIDUAL_ENABLE
static AppOdometryBaseline40 s_navigation40;
static uint32_t s_navigation_left_scope;
static uint32_t AppOdometry_LeftNavigationScope(const AppOdometryInputs *inputs)
{
    const AppMotionObservation *motion=AppMotion_Live();
    const MotionAction *action=motion->actions;
    uint32_t id=g_debug_plan_id;
    if ((*inputs->runtime_route_id) || (*motion->action_count)!=1U ||
        (id!=DEBUG_PLAN_LEFT210_40 && id!=DEBUG_PLAN_LEFT150_40) ||
        action->kind!=MOTION_LEFT_DISTANCE || action->base_rpm!=APP_BASELINE40_LEFT_RPM ||
        action->run_ms || action->reference_mm_s!=0.0f || action->relative_deg!=0.0f) return 0U;
    return AppDebugPlan40Allowed(id,1U,action->base_rpm,action->distance_mm);
}
#endif
''')
ch('app_odometry.c','    *snapshot=s_baseline40;\n    taskEXIT_CRITICAL();\n}\n#endif','''    *snapshot=s_baseline40;
    taskEXIT_CRITICAL();
}
void AppOdometry_ReadNavigation40(AppOdometryBaseline40 *snapshot)
{
#if APP_LEFT_NAV_RESIDUAL_ENABLE
    taskENTER_CRITICAL();
    *snapshot=s_navigation40;
    taskEXIT_CRITICAL();
#else
    AppOdometry_ReadBaseline40(snapshot);
#endif
}
#endif''')
ch('app_odometry.c','    memset(&s_baseline40,0,sizeof(s_baseline40));\n','''    memset(&s_baseline40,0,sizeof(s_baseline40));
#if APP_LEFT_NAV_RESIDUAL_ENABLE
    memset(&s_navigation40,0,sizeof(s_navigation40));
    s_navigation_left_scope=0U;
#endif
''')
ch('app_odometry.c','    ChassisOdometrySamples samples;\n','''    ChassisOdometrySamples samples;
#if APP_LEFT_NAV_RESIDUAL_ENABLE
    float navigation_x,navigation_y;
#endif
''')
ch('app_odometry.c','            if(ChassisOdometry_SetPose(&s_odometry,x,y,heading,source.yaw_deg))\n                g_odometry_pose_result=1U;','''            if(ChassisOdometry_SetPose(&s_odometry,x,y,heading,source.yaw_deg))
            {
                g_odometry_pose_result=1U;
#if APP_LEFT_NAV_RESIDUAL_ENABLE
                s_navigation_left_scope=0U;
#endif
            }''')
ch('app_odometry.c','            s_baseline40.anchor_valid=1U;\n','''            s_baseline40.anchor_valid=1U;
#if APP_LEFT_NAV_RESIDUAL_ENABLE
            s_navigation_left_scope=AppOdometry_LeftNavigationScope(inputs);
#endif
''')
ch('app_odometry.c','    ChassisOdometry_Update(&s_odometry,now);\n    taskENTER_CRITICAL();\n    g_chassis_odometry=s_odometry.status;\n#if APP_BASELINE40_ACTIVE\n    s_baseline40.pose=s_odometry.status;\n#endif','''    ChassisOdometry_Update(&s_odometry,now);
#if APP_LEFT_NAV_RESIDUAL_ENABLE
    navigation_x=s_odometry.status.x_mm;navigation_y=s_odometry.status.y_mm;
    if(s_navigation_left_scope)
    {
        float angle=s_baseline40.path_heading_deg*0.01745329251994329577f;
        float cs=cosf(angle),sn=sinf(angle);
        float left=-sn*(navigation_x-s_baseline40.origin_x_mm)+
            cs*(navigation_y-s_baseline40.origin_y_mm);
        float correction=APP_LEFT_NAV_RESIDUAL_WEIGHT*APP_LEFT_NAV_RESIDUAL_COEFFICIENT*
            ((left>0.0f)?left:0.0f);
        navigation_x+=cs*correction;navigation_y+=sn*correction;
    }
#endif
    taskENTER_CRITICAL();
#if APP_BASELINE40_ACTIVE
    s_baseline40.pose=s_odometry.status;
#endif
#if APP_LEFT_NAV_RESIDUAL_ENABLE
    s_navigation40=s_baseline40;
    s_navigation40.pose.x_mm=navigation_x;s_navigation40.pose.y_mm=navigation_y;
    g_chassis_odometry=s_navigation40.pose;
#else
    g_chassis_odometry=s_odometry.status;
#endif''')
ch('app_odometry.h','void AppOdometry_ReadBaseline40(AppOdometryBaseline40 *snapshot);','void AppOdometry_ReadBaseline40(AppOdometryBaseline40 *snapshot);\nvoid AppOdometry_ReadNavigation40(AppOdometryBaseline40 *snapshot);')
ch('app_icm_observer.c','    AppOdometry_ReadBaseline40(&wheel);','    AppOdometry_ReadNavigation40(&wheel);')
ch('app_odometry.c','\n    s_baseline40.anchor_valid=0U;\n','\n    s_baseline40.anchor_valid=0U;\n#if APP_LEFT_NAV_RESIDUAL_ENABLE\n    s_navigation40.anchor_valid=0U;\n#endif\n')
ch('app_odometry.c','\n        s_baseline40.anchor_valid=0U;\n','\n        s_baseline40.anchor_valid=0U;\n#if APP_LEFT_NAV_RESIDUAL_ENABLE\n        s_navigation40.anchor_valid=0U;\n#endif\n')
report=[]
for name in files:
 b=(U/name).read_bytes();old=(B/name).read_bytes();assert b.count(b'\n')==b.count(b'\r\n');assert b.decode('cp936').encode('cp936')==b
 report.append(dict(file=name,before_sha256=hashlib.sha256(old).hexdigest(),after_sha256=hashlib.sha256(b).hexdigest(),bytes=len(b),crlf=b.count(b'\r\n'),encoding='GBK preserved, ASCII edits'))
(E/'product_source_manifest.json').write_text(json.dumps(report,indent=2),'utf8');print(json.dumps(report,indent=2))