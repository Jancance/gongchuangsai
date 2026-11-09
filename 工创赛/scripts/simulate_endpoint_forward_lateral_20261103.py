"""完整实际底盘后端两段模型；Emm整数RPM反馈，末端单帧/持续故障。"""
from pathlib import Path
import json
import subprocess

R = Path(__file__).resolve().parents[1]
S = R / 'tmp/endpoint_integer_feedback_20261103'
E = R / 'evidence/integration/2026-11-03/no-motion/endpoint'
base = (E.parent / 'handoff-backend-unit.c').read_text(encoding='utf-8')
base = base[:base.index('int main(void)')]
anchor = 'uint32_t StartupAxes_Holding(void)'
prelude = r'''
static float plant_pos[4],plant_velocity[4],plant_target[4],plant_gain=1.0f;
static uint32_t plant_enabled[4],injection,injected;
static uint32_t PlantEndpointAllowed(uint32_t wheel);
'''
base = base.replace(anchor, prelude + anchor, 1)


def replace_function(text, name, body):
    start = text.index('uint32_t ' + name + '(')
    end = text.index('{', start) + 1
    depth = 1
    while depth:
        depth += (text[end] == '{') - (text[end] == '}')
        end += 1
    return text[:start] + body + text[end:]


base = replace_function(base, 'CAN1_ReadStatus', r'''
uint32_t CAN1_ReadStatus(uint8_t a,CanRxMsg *p){hardware_calls++;ck(!test_depth);p->Data[1]=plant_enabled[a-1];return 1;}
''')
base = replace_function(base, 'CAN1_ReadPosition', r'''
uint32_t CAN1_ReadPosition(uint8_t a,CanRxMsg *p,uint8_t *sign,uint32_t *raw){
 (void)p;hardware_calls++;ck(!test_depth);
 float driver=plant_pos[a-1]*(g_chassis_forward_direction[a-1]?-1:1);
 *sign=driver<0;*raw=(uint32_t)(fabsf(driver)*65536.0f/(3.14159265358979323846f*77.0f)+0.5f);return 1;
}
''')
base = replace_function(base, 'CAN1_ReadEmmSystemSnapshot', r'''
uint32_t CAN1_ReadEmmSystemSnapshot(uint8_t a,CanRxMsg *p,float *rpm){
 (void)p;hardware_calls++;ck(!test_depth);
 float logical=roundf(plant_velocity[a-1]);*rpm=g_chassis_forward_direction[a-1]?-logical:logical;
 if(injection && a==1 && ((injected && injection==2) || (!injected && PlantEndpointAllowed(a-1)))){
  *rpm=(s_direction_guard[a-1].target_deci>0?-1.0f:1.0f)*(injection==3?3.0f:2.0f);injected++;
 }
 return 1;
}
''')
base = replace_function(base, 'CAN1_SetMotorEnable', r'''
uint32_t CAN1_SetMotorEnable(uint8_t a,uint8_t v,CanRxMsg *p){
 (void)p;hardware_calls++;ck(!test_depth);plant_enabled[a-1]=v;if(!v)plant_target[a-1]=0;return 1;
}
''')
base = replace_function(base, 'CAN1_SetEmmSpeedRaw', r'''
uint32_t CAN1_SetEmmSpeedRaw(uint8_t a,uint8_t d,uint16_t r,uint8_t ac,CanRxMsg *p){
 (void)p;hardware_calls++;ck(!test_depth && r<=1040 && ac==236);
 plant_target[a-1]=(d==g_chassis_forward_direction[a-1]?1.0f:-1.0f)*(float)r*0.1f;return 1;
}
''')
base = replace_function(base, 'CAN1_StopMotor', r'''
uint32_t CAN1_StopMotor(uint8_t a,CanRxMsg *p){(void)p;hardware_calls++;ck(!test_depth);plant_target[a-1]=0;return 1;}
''')
base += r'''
static uint32_t PlantEndpointAllowed(uint32_t wheel){
 return s_forward_hold && s_distance_mode && s_route_trial && s_profile_active &&
  !s_finish_requested && !s_stop_due && !s_distance_reached && s_translation_base_rpm<=2 &&
  s_distance_progress_mm>=s_distance_target_mm-10 && s_distance_progress_mm<=s_distance_target_mm &&
  s_distance_reference.acceleration_mm_s2<=0 && s_direction_guard[wheel].target_deci &&
  abs(s_direction_guard[wheel].target_deci)<=20 && test_tick-s_direction_guard[wheel].direction_ms>=500;
}
static void stage(uint32_t direction,float distance,float speed){
 ChassisAutoPreparedTranslation p;MotionProfile profile;uint32_t cap,ms,steps;
 ck(MotionRoute_Build(direction,distance,speed,&profile,&cap,&ms));
 ck(ChassisAuto_PrepareRouteTranslation(direction,cap,ms,distance,speed,&p));
 ck(ChassisAuto_BeginPreparedTranslation(&p,g_chassis_auto_session));
 for(steps=0;steps<4500 && g_chassis_auto_active;steps++){
  test_tick+=10;
  for(unsigned i=0;i<4;i++){
   float demand=plant_enabled[i]?plant_target[i]*plant_gain:0;
   plant_velocity[i]+=(demand-plant_velocity[i])*(0.01f/0.03f);
   plant_pos[i]+=plant_velocity[i]*(3.14159265358979323846f*77.0f/60)*0.01f;
  }
  (void)ChassisAuto_UpdateForwardHold(0.0f);
  ChassisAuto_SafetyUpdate(1);ChassisAuto_CanStep();ck(!test_depth);
 }
 printf("STAGE,%u,%.0f,%.0f,%u,%u,%u,%u,%.3f,%u\n",direction,distance,speed,steps*10,g_chassis_auto_result,g_chassis_auto_cause,g_chassis_auto_fault_diagnostic.reason,s_distance_progress_mm,injected);
 ck(steps<4500);
 if(injection>=2){ck(g_chassis_auto_result==2 && g_chassis_auto_cut_vm && !s_distance_reached);}
 else ck(g_chassis_auto_result==1 && s_distance_reached && g_chassis_auto_stop_mask[2]==15);
}
int main(void){
 for(injection=0;injection<=3;injection++){
  reset();injected=0;memset(plant_pos,0,sizeof(plant_pos));memset(plant_velocity,0,sizeof(plant_velocity));
  memset(plant_target,0,sizeof(plant_target));memset(plant_enabled,0,sizeof(plant_enabled));
  ck(App_MotionAccept());stage(0,150,200);
  if(injection)ck(injected>0);
  if(injection<=1){
   ck(ChassisAuto_ReleaseCompleted(g_chassis_auto_session));
   injection=0;plant_gain=1.0f;stage(1,1050,400); /* no second physical test, model only */
   injection=(unsigned)(injected?1:0);
  }
 }
 printf("DYNAMIC checks=%u failures=%u\n",checks,failures);return failures?1:0;
}
'''
# Keep the scenario selector separate from the injection state used by stage 2.
base = base.replace('for(injection=0;injection<=3;injection++){', 'for(unsigned scenario=0;scenario<=3;scenario++){\n  injection=scenario;plant_gain=scenario?0.97f:1.0f;')
base = base.replace('injection=(unsigned)(injected?1:0);', '')
path = E / 'endpoint-dynamic-unit.c'
path.write_text(base, encoding='utf-8')
exe = E / 'endpoint-dynamic.exe'
p = subprocess.run(['C:/mingw/mingw64/bin/gcc.exe', '-std=c99', '-include', 'stdlib.h', '-DAPP_MOTOR_OFFLINE_ONLY=0',
                    '-I' + str(E.parent / 'backend-stubs'), '-I' + str(S / 'CHASSIS/USER'), str(path),
                    str(S / 'CHASSIS/USER/chassis_kinematics.c'), str(S / 'CHASSIS/USER/motion_profile.c'), '-lm', '-o', str(exe)],
                   capture_output=True, text=True, errors='replace')
assert p.returncode == 0, p.stderr
p = subprocess.run([str(exe)], capture_output=True, text=True)
print(p.stdout)
(E / 'dynamic-model.log').write_text(p.stdout + p.stderr, encoding='utf-8')
assert p.returncode == 0, p.stdout
(E / 'dynamic-check.json').write_text(json.dumps(dict(passed=True, actual_complete_backend=True,
    CAN_RTOS_platform_and_wheel_response_are_models=True, integer_rpm_feedback=True,
    hardware_accessed=False, output=p.stdout), indent=2), encoding='utf-8')
