from pathlib import Path
import hashlib,json
R=Path('D:/Desktop/工创赛');U=R/'3.stm32f407工程/CHASSIS/USER';E=R/'工创赛/evidence/navigation-planning/2026-11-17/right1-forward-trim-prep/medium'
def change(name,old,new):
 p=U/name;b=p.read_bytes();o=old.replace('\n','\r\n').encode('ascii');n=new.replace('\n','\r\n').encode('ascii')
 assert b.count(o)==1,(name,b.count(o));p.write_bytes(b.replace(o,n,1))
change('app_config.h','#define APP_LEFT40_DRIFT_RATIO 0.02f\n','#define APP_LEFT40_DRIFT_RATIO 0.02f\n#ifndef APP_RIGHT40_FORWARD_TRIM_ENABLE\n#define APP_RIGHT40_FORWARD_TRIM_ENABLE 0U\n#endif\n#if APP_RIGHT40_FORWARD_TRIM_ENABLE > 1U\n#error Right40_forward_trim_enable_invalid\n#endif\n#if APP_RIGHT40_FORWARD_TRIM_ENABLE && !APP_BASELINE40_ACTIVE\n#error Right40_forward_trim_requires_baseline40_debug\n#endif\n#define APP_RIGHT40_FORWARD_RATIO 0.01f\n')
change('chassis_kinematics.c','uint32_t Chassis_Baseline40Budget(uint32_t base_rpm, float heading_rpm,','uint32_t Chassis_Right40ForwardCommands(uint32_t base_rpm,float correction_rpm,\n    ChassisMotorCommand commands[CHASSIS_WHEEL_COUNT])\n{\n    ChassisWheelSpeeds logical;float h,b,t;\n    Chassis_ClearMotorCommands(commands);\n    if (!commands || base_rpm>APP_BASELINE40_LEFT_RPM ||\n        !Chassis_ProfileCorrectionRpm(base_rpm,correction_rpm,&h)) return 0U;\n    b=(float)base_rpm;t=APP_RIGHT40_FORWARD_RATIO*b;\n    logical.rpm[0]=-b+t-h;logical.rpm[1]=b+t+h;\n    logical.rpm[2]=b+t-h;logical.rpm[3]=-b+t+h;\n    return Chassis_EncodeEmm(&logical,APP_BASELINE40_LEFT_RPM+4U,commands);\n}\nuint32_t Chassis_Baseline40Budget(uint32_t base_rpm, float heading_rpm,')
change('chassis_kinematics.h','uint32_t Chassis_Left40DriftCommands(uint32_t base_rpm,float correction_rpm,\n    ChassisMotorCommand commands[CHASSIS_WHEEL_COUNT]);','uint32_t Chassis_Left40DriftCommands(uint32_t base_rpm,float correction_rpm,\n    ChassisMotorCommand commands[CHASSIS_WHEEL_COUNT]);\nuint32_t Chassis_Right40ForwardCommands(uint32_t base_rpm,float correction_rpm,\n    ChassisMotorCommand commands[CHASSIS_WHEEL_COUNT]);')
change('chassis_auto_test.c','#if APP_BASELINE40_ACTIVE && APP_LEFT40_DRIFT_TRIM_ENABLE\n                ((g_debug_plan_id==DEBUG_PLAN_LEFT210_40','#if APP_BASELINE40_ACTIVE && APP_RIGHT40_FORWARD_TRIM_ENABLE\n                ((g_debug_plan_id==DEBUG_PLAN_RIGHT210_40 || g_debug_plan_id==DEBUG_PLAN_RIGHT150_40 || g_debug_plan_id==DEBUG_PLAN_RIGHT30_40) &&\n                s_translation_mode==2U && !s_route_trial &&\n                AppDebugPlan40Allowed(g_debug_plan_id,s_translation_mode,s_distance_max_rpm,s_distance_target_mm)) ?\n                Chassis_Right40ForwardCommands(s_translation_base_rpm,effective_heading,s_commands[2]) :\n#endif\n#if APP_BASELINE40_ACTIVE && APP_LEFT40_DRIFT_TRIM_ENABLE\n                ((g_debug_plan_id==DEBUG_PLAN_LEFT210_40')
for kind,limit in [('Acceleration','ACCEL_MM_S2'),('Jerk','JERK_MM_S3')]:
 old=f'#endif\n    return MotionTrial40Allowed(direction,rpm,mm) ? APP_BASELINE40_{limit}'
 new=f'#endif\n#if APP_RIGHT40_FORWARD_TRIM_ENABLE\n    if (direction==2U && rpm==APP_BASELINE40_LEFT_RPM &&\n        MotionTrial40Allowed(direction,rpm,mm)) return APP_BASELINE40_{limit}/(1.0f+APP_RIGHT40_FORWARD_RATIO);\n#endif\n    return MotionTrial40Allowed(direction,rpm,mm) ? APP_BASELINE40_{limit}'
 change('motion_trial40.h',old,new)
files=['app_config.h','chassis_kinematics.c','chassis_kinematics.h','chassis_auto_test.c','motion_trial40.h'];manifest=[]
for name in files:
 b=(U/name).read_bytes();old=(E/'source_before'/name).read_bytes()
 assert b.count(b'\n')==b.count(b'\r\n'),name
 assert old.decode('cp936').encode('cp936')==old,name
 assert b.decode('cp936').encode('cp936')==b,name
 manifest.append(dict(file=name,before_sha256=hashlib.sha256(old).hexdigest(),after_sha256=hashlib.sha256(b).hexdigest(),bytes=len(b),crlf=b.count(b'\r\n'),encoding='GBK raw bytes preserved except ASCII insertions'))
(E/'source_after_manifest.json').write_text(json.dumps(manifest,indent=2),encoding='utf8');print(json.dumps(manifest,indent=2))