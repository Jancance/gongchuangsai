from pathlib import Path
import json,hashlib,shutil
R=Path.cwd();U=R/'3.stm32f407工程/CHASSIS/USER';E=Path(__file__).parent
files=['app_config.h','motion_trial40.h','chassis_kinematics.h','chassis_kinematics.c','chassis_auto_test.c'];texts={};before={}
(E/'source_before').mkdir(exist_ok=True)
for f in files:
 b=(U/f).read_bytes();assert b'\r\n' in b and b'\n' not in b.replace(b'\r\n',b'');before[f]=hashlib.sha256(b).hexdigest();shutil.copy2(U/f,E/'source_before'/f);texts[f]=b.decode('gbk')
def rep(f,a,b):
 assert texts[f].count(a)==1,(f,a,texts[f].count(a));texts[f]=texts[f].replace(a,b)
rep('app_config.h','#define APP_BASELINE40_LEFT_RPM 102U','#define APP_BASELINE40_LEFT_RPM 102U\r\n#ifndef APP_LEFT40_DRIFT_TRIM_ENABLE\r\n#define APP_LEFT40_DRIFT_TRIM_ENABLE 0U\r\n#endif\r\n#if APP_LEFT40_DRIFT_TRIM_ENABLE > 1U\r\n#error Left40_drift_trim_enable_invalid\r\n#endif\r\n#define APP_LEFT40_DRIFT_RATIO 0.04f')
rep('motion_trial40.h','    return MotionTrial40Allowed(direction,rpm,mm) ? APP_BASELINE40_ACCEL_MM_S2 : APP_DISTANCE_SCURVE_ACCEL_MM_S2;','    if (APP_LEFT40_DRIFT_TRIM_ENABLE && direction==1U && rpm==APP_BASELINE40_LEFT_RPM &&\r\n        MotionTrial40Allowed(direction,rpm,mm)) return APP_BASELINE40_ACCEL_MM_S2/(1.0f+APP_LEFT40_DRIFT_RATIO);\r\n    return MotionTrial40Allowed(direction,rpm,mm) ? APP_BASELINE40_ACCEL_MM_S2 : APP_DISTANCE_SCURVE_ACCEL_MM_S2;')
rep('motion_trial40.h','    return MotionTrial40Allowed(direction,rpm,mm) ? APP_BASELINE40_JERK_MM_S3 : APP_DISTANCE_SCURVE_JERK_MM_S3;','    if (APP_LEFT40_DRIFT_TRIM_ENABLE && direction==1U && rpm==APP_BASELINE40_LEFT_RPM &&\r\n        MotionTrial40Allowed(direction,rpm,mm)) return APP_BASELINE40_JERK_MM_S3/(1.0f+APP_LEFT40_DRIFT_RATIO);\r\n    return MotionTrial40Allowed(direction,rpm,mm) ? APP_BASELINE40_JERK_MM_S3 : APP_DISTANCE_SCURVE_JERK_MM_S3;')
rep('chassis_kinematics.h','#if APP_BASELINE40_ACTIVE\r\n/* PLAN8-only','#if APP_BASELINE40_ACTIVE\r\nuint32_t Chassis_Left40DriftCommands(uint32_t base_rpm,float correction_rpm,\r\n    ChassisMotorCommand commands[CHASSIS_WHEEL_COUNT]);\r\n/* PLAN8-only')
rep('chassis_kinematics.c','#if APP_BASELINE40_ACTIVE\r\nuint32_t Chassis_Baseline40Budget','''#if APP_BASELINE40_ACTIVE\r
uint32_t Chassis_Left40DriftCommands(uint32_t base_rpm,float correction_rpm,\r
    ChassisMotorCommand commands[CHASSIS_WHEEL_COUNT])\r
{\r
    ChassisWheelSpeeds logical;float h,b,t;\r
    Chassis_ClearMotorCommands(commands);\r
    if (!commands || base_rpm>APP_BASELINE40_LEFT_RPM ||\r
        !Chassis_ProfileCorrectionRpm(base_rpm,correction_rpm,&h)) return 0U;\r
    b=(float)base_rpm;t=(float)(uint32_t)(APP_LEFT40_DRIFT_RATIO*b+0.5f);\r
    logical.rpm[0]=b-t-h;logical.rpm[1]=-b-t+h;\r
    logical.rpm[2]=-b-t-h;logical.rpm[3]=b-t+h;\r
    return Chassis_EncodeEmm(&logical,APP_BASELINE40_LEFT_RPM+6U,commands);\r
}\r
uint32_t Chassis_Baseline40Budget''')
rep('chassis_auto_test.c','''(s_profile_active ? ((s_route_trial && s_translation_mode==1U && APP_LEFT_TRIM_ACTIVE) ?''','''(s_profile_active ? ((APP_LEFT40_DRIFT_TRIM_ENABLE && g_debug_plan_id==DEBUG_PLAN_LEFT210_40 &&\r
                s_translation_mode==1U && !s_route_trial &&\r
                AppDebugPlan40Allowed(g_debug_plan_id,s_translation_mode,s_distance_max_rpm,s_distance_target_mm)) ?\r
                Chassis_Left40DriftCommands(s_translation_base_rpm,effective_heading,s_commands[2]) :\r
                (s_route_trial && s_translation_mode==1U && APP_LEFT_TRIM_ACTIVE) ?''')
after={}
for f,s in texts.items():
 b=s.encode('gbk');assert b'\n' not in b.replace(b'\r\n',b'');(U/f).write_bytes(b);after[f]=hashlib.sha256(b).hexdigest()
(E/'source_changes.json').write_text(json.dumps(dict(before=before,after=after,encoding='GBK',newline='CRLF'),indent=2),encoding='utf8')
print('5 surgical files changed')
