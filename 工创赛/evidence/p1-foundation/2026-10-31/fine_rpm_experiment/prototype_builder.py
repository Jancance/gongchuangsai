from pathlib import Path
import hashlib,json,re,shutil
R=Path(__file__).resolve().parents[2]
B=R/'工创赛/tmp/perimeter_heading_ref_candidate_20261031'
C=R/'工创赛/tmp/fine_rpm_experiment_20261031'
E=R/'工创赛/evidence/p1-foundation/2026-10-31/fine_rpm_experiment'
assert not C.exists()
E.mkdir(parents=True,exist_ok=True)
protected={str(p.relative_to(R)):hashlib.sha256(p.read_bytes()).hexdigest() for d in [B/'USER',R/'3.stm32f407工程/USER'] for p in d.glob('*') if p.is_file()}
protected[str((B/'Objects/gongxun-f407.axf').relative_to(R))]=hashlib.sha256((B/'Objects/gongxun-f407.axf').read_bytes()).hexdigest()
shutil.copytree(B,C)
def edit(name,fn):
 p=C/'USER'/name;b=p.read_bytes()
 try:s=b.decode('utf8');enc='utf8'
 except UnicodeDecodeError:s=b.decode('gbk');enc='gbk'
 s=fn(s);p.write_bytes(s.encode(enc))
def sub(s,a,b):
 assert s.count(a)==1,(a[:90],s.count(a));return s.replace(a,b)
def types(s):
 # Only command/correction data are physical float RPM. Encoder and Emm feedback stay integer.
 for name in ['command_rpm','correction_rpm','requested','effective']:
  s=re.sub(r'\bint32_t(\s+\*?\s*)'+name+r'\b',r'float\1'+name,s)
 return s
for n in ['heading_controller.c','heading_controller.h','heading_session.h','turn_brake.h',
          'chassis_kinematics.c','chassis_kinematics.h','chassis_auto_test.c','chassis_auto_test.h','main.c']:
 edit(n,types)
edit('app_config.h',lambda s:s+'\r\n/* Offline experiment. Not a vehicle-ready firmware or persistent motor configuration. */\r\n#define APP_FINE_RPM_OFFLINE_ONLY 1U\r\n')
edit('main.c',lambda s:sub(s,'static uint32_t App_MotionAccept(void)\r\n{',
 'static uint32_t App_MotionAccept(void)\r\n{\r\n    if(APP_FINE_RPM_OFFLINE_ONLY) return 0U; /* Explicitly disallow real launch. */'))
edit('main.c',lambda s:sub(s,'    int32_t correction = 0;','    float correction = 0;'))
edit('heading_controller.h',lambda s:sub(s,'int32_t quantized_rpm;','float quantized_rpm;'))
def hold(s):
 a=s.index('    /* 0.65RPM');b=s.index('    hold->filtered_rate_dps',a)
 return s[:a]+'''    /* 0.1RPM resolution, proportional hysteresis; bounded updates preserve slew. */
    {
        uint32_t quantize_step;
        for(quantize_step=0U;quantize_step<40U;quantize_step++) {
            if(hold->command_rpm-hold->quantized_rpm>0.065f)
                hold->quantized_rpm+=0.1f;
            else if(hold->command_rpm-hold->quantized_rpm < -0.065f)
                hold->quantized_rpm-=0.1f;
            else break;
        }
        if(hold->quantized_rpm>2.0f)hold->quantized_rpm=2.0f;
        if(hold->quantized_rpm < -2.0f)hold->quantized_rpm=-2.0f;
    }
'''.replace('\n','\r\n')+s[b:]
edit('heading_controller.c',hold)
def brake(s):
 s=sub(s,'int32_t *command','float *command')
 s=sub(s,'*command=(int32_t)(rpm+0.5f);','*command=floorf(rpm*10.0f+0.5f)*0.1f;')
 s=sub(s,'*command=(int32_t)(s->direction*(float)*command);','*command=s->direction*(*command);')
 return s
edit('turn_brake.h',brake)
def kine(s):
 s=sub(s,'raw = (uint16_t)(magnitude + 0.5f);','raw = (uint16_t)(magnitude*10.0f + 0.5f);')
 s=sub(s,'if ((raw > max_raw) ||\r\n            (Chassis_Abs(magnitude - (float)raw) > CHASSIS_INTEGER_RPM_TOLERANCE))',
  'if ((raw > (uint32_t)max_raw*10U) || raw>3000U ||\r\n            (Chassis_Abs(magnitude - (float)raw*0.1f) > 0.0501f))')
 # No float-to-int conversion before wheel mixing or profile correction clamp.
 s=sub(s,'    int32_t limit;','    float limit;')
 s=sub(s,'(correction_rpm < -2) || (correction_rpm > 2)',
  '!(correction_rpm >= -2.0f && correction_rpm <= 2.0f)')
 s=sub(s,'requested < -2 || requested > 2','!(requested >= -2.0f && requested <= 2.0f)')
 s=sub(s,'(command_rpm < -5) || (command_rpm > 5)',
  '!(command_rpm >= -5.0f && command_rpm <= 5.0f)')
 s=sub(s,'(command_rpm < -(int32_t)cap) || (command_rpm > (int32_t)cap)',
  '!(command_rpm >= -(float)cap && command_rpm <= (float)cap)')
 return s
edit('chassis_kinematics.c',kine)
def backend(s):
 s=sub(s,'static int32_t s_heading_initial_rpm = 0;','static float s_heading_initial_rpm = 0;')
 s=sub(s,'static int32_t s_heading_pending_rpm = 0, s_heading_applied_rpm = 0;',
  'static float s_heading_pending_rpm = 0, s_heading_applied_rpm = 0;')
 s=sub(s,'    int32_t logical;','    float logical;')
 s=sub(s,'magnitude=(uint32_t)(p->command_rpm<0 ? -p->command_rpm : p->command_rpm);',
  'magnitude=(uint32_t)((p->command_rpm<0 ? -p->command_rpm : p->command_rpm)*10.0f+0.5f);')
 s=sub(s,'commands[i][j].speed_raw = (uint16_t)CHASSIS_AUTO_TARGET_RPM;',
  'commands[i][j].speed_raw = (uint16_t)(10U*CHASSIS_AUTO_TARGET_RPM);')
 a='if ((command_rpm < -(int32_t)(s_turn_brake_trial ? APP_TURN_BRAKE_MAX_RPM : 5U)) ||\r\n            (command_rpm > (int32_t)(s_turn_brake_trial ? APP_TURN_BRAKE_MAX_RPM : 5U)) ||'
 b='if (!(command_rpm >= -(float)(s_turn_brake_trial ? APP_TURN_BRAKE_MAX_RPM : 5U) &&\r\n              command_rpm <= (float)(s_turn_brake_trial ? APP_TURN_BRAKE_MAX_RPM : 5U)) ||'
 s=sub(s,a,b)
 s=sub(s,'(p->command_rpm==0 || p->command_rpm < -5 || p->command_rpm >5)',
  '(p->command_rpm==0 || !(p->command_rpm>=-5.0f && p->command_rpm<=5.0f))')
 return s
edit('chassis_auto_test.c',backend)
m=dict(date='2026-10-31',purpose='OFFLINE ONLY fine heading RPM experiment, not adopted',base=str(B.relative_to(R)),candidate=str(C.relative_to(R)),hardware_accessed=False,vehicle_launch_disabled=True,motor_configuration_changed=False,raw_speed_scale=10,feedback_scale=1,translation_profile_base_quantization_rpm=1,turn_and_hold_quantization_rpm=.1,protected_sha256=protected)
(E/'manifest.json').write_text(json.dumps(m,ensure_ascii=False,indent=2),encoding='utf8')
print(C)
