"""Apply the written 40cm/s / 60deg/s plan to the uninstalled stage."""
from pathlib import Path
import hashlib,json,re
R=Path(__file__).resolve().parents[1];E=R/'evidence/integration/2026-11-03/stage2-grid-route'
m=json.loads((E/'manifest.json').read_text(encoding='utf-8'));S=Path(m['stage']);A=Path(m['actual'])
assert not m['installed'] and (R/'docs/STAGE2_SPEED_PLAN_2026-11-03.md').exists()
def edit(rel,fn):
 p=S/rel;b=p.read_bytes()
 try:s=b.decode('utf-8');enc='utf-8'
 except UnicodeDecodeError:s=b.decode('gbk');enc='gbk'
 if rel not in m['original']:
  original=(A/rel).read_bytes();m['original'][rel]=hashlib.sha256(original).hexdigest()
  q=E/'before-install'/rel;q.parent.mkdir(parents=True,exist_ok=True);q.write_bytes(original)
 p.write_bytes(fn(s.replace('\r\n','\n')).replace('\n','\r\n').encode(enc))
def params(s):
 s=s.replace('#define VEHICLE_STAGE2_FORWARD_CM_S 10.0f','#define VEHICLE_STAGE2_FORWARD_CM_S 40.0f')
 s=s.replace('#define VEHICLE_STAGE2_LATERAL_CM_S 10.0f','#define VEHICLE_STAGE2_LATERAL_CM_S 40.0f')
 mark='#define VEHICLE_ALIGNMENT_POSITION_EXECUTOR_ENABLE 0U'
 s=s.replace(mark,mark+'\n#if VEHICLE_ALIGNMENT_POSITION_EXECUTOR_ENABLE\n#error Metric_vision_and_position_executor_are_not_integrated\n#endif')
 lo=s.index(' /* 2 扫码区');hi=s.index('\n};',lo)
 s=s[:lo]+re.sub(r'(V[FLRB]\([^,]+,)(?:10\.0f|5\.0f)(\))',r'\g<1>40.0f\2',s[lo:hi])+s[hi:]
 s=s.replace('#define VEHICLE_ROUTE_JERK_MM_S3 2000.0f','#define VEHICLE_ROUTE_JERK_MM_S3 2000.0f\n/* 底盘旋转上限；与DM机械臂300deg/s分开。 */\n#define VEHICLE_CHASSIS_TURN_DEG_S 60U\n#define VEHICLE_TURN_ACCEL_DEG_S2 90.0f\n#define VEHICLE_TURN_JERK_DEG_S3 360.0f')
 return s
edit('USER/vehicle_parameters.h',params)
def config(s):
 s=s.replace('#define APP_TURN_BRAKE_TRIAL_DPS 30U','#define APP_TURN_BRAKE_TRIAL_DPS VEHICLE_CHASSIS_TURN_DEG_S')
 s=s.replace('APP_TURN_BRAKE_TRIAL_DPS != 15U && APP_TURN_BRAKE_TRIAL_DPS != 30U','APP_TURN_BRAKE_TRIAL_DPS != 15U && APP_TURN_BRAKE_TRIAL_DPS != 30U && APP_TURN_BRAKE_TRIAL_DPS != 60U')
 s=s.replace('((APP_TURN_BRAKE_TRIAL_DPS == 30U) ? 27U : 13U)','((APP_TURN_BRAKE_TRIAL_DPS == 60U) ? 54U : ((APP_TURN_BRAKE_TRIAL_DPS == 30U) ? 27U : 13U))')
 s=s.replace('((APP_TURN_BRAKE_TRIAL_DPS == 30U) ? 40U : 30U)','((APP_TURN_BRAKE_TRIAL_DPS == 60U) ? 80U : ((APP_TURN_BRAKE_TRIAL_DPS == 30U) ? 40U : 30U))')
 return s
edit('CHASSIS/USER/app_config.h',config)
edit('CHASSIS/USER/route_plan.h',lambda s:s.replace('maximum=(APP_TURN_BRAKE_TRIAL_DPS==30U)?27.2f:13.6f;','maximum=(float)APP_TURN_BRAKE_TRIAL_DPS/factor;'))
edit('CHASSIS/USER/turn_brake.h',lambda s:s.replace('envelope=sqrtf(144.0f+30.0f*distance)-12.0f;','envelope=sqrtf(144.0f+2.0f*VEHICLE_TURN_ACCEL_DEG_S2*distance)-12.0f;').replace('TurnBrake_Clip((want-s->speed)*5.0f,15.0f)','TurnBrake_Clip((want-s->speed)*5.0f,VEHICLE_TURN_ACCEL_DEG_S2)').replace('TurnBrake_Clip(desired-s->acceleration,60.0f*dt)','TurnBrake_Clip(desired-s->acceleration,VEHICLE_TURN_JERK_DEG_S3*dt)'))
edit('CHASSIS/USER/vehicle_mission.inc',lambda s:s.replace('VehicleMission_Stopped() && StartupAxes_Holding()){','VehicleMission_Stopped() && StartupAxes_Holding() &&\n       !g_rtos_fault && !g_safety_fault && !g_can_ctrl_fault_latched){'))
m['changed']={rel:hashlib.sha256((S/rel).read_bytes()).hexdigest() for rel in m['original']}
m['speed_plan']='docs/STAGE2_SPEED_PLAN_2026-11-03.md'
(E/'manifest.json').write_text(json.dumps(m,ensure_ascii=False,indent=2),encoding='utf-8')
print('Updated uninstalled stage; seven-file plan including node and alignment interface')
