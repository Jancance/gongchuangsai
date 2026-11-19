"""Actual owner and DM driver checks with mocked I/O. No device access."""
from pathlib import Path
import subprocess,json
R=Path(__file__).resolve().parents[1];P=R/'tmp/pickup_half_descent_20261110/project'
E=R/'evidence/integration/2026-11-10/half-descent';D=E/'stubs';D.mkdir(exist_ok=True)
(D/'FreeRTOS.h').write_text('#include <stdint.h>\ntypedef uint32_t TickType_t;\n#define pdMS_TO_TICKS(x) (x)\n#define portTICK_PERIOD_MS 1U\n')
(D/'task.h').write_text('uint32_t xTaskGetTickCount(void);\n')

def run(name,unit,include):
    src=E/(name+'.c');exe=E/(name+'.exe');src.write_text(unit,encoding='utf-8')
    cmd=['gcc','-std=c99','-Wno-unused-function','-Wno-unused-variable','-I'+str(D),'-I'+str(include),
         '-DVEHICLE_STARTUP_GROUP_BEFORE_ROUTE=0','-DVEHICLE_STARTUP_GROUP_ONLY=0',
         '-DVEHICLE_EXTENSION_HOME_ONLY=0','-DVEHICLE_DM_STARTUP_ONLY=0',str(src),'-lm','-o',str(exe)]
    p=subprocess.run(cmd,capture_output=True,text=True);log=p.stdout+p.stderr
    if p.returncode==0:
        p=subprocess.run([str(exe)],capture_output=True,text=True,timeout=15);log+=p.stdout+p.stderr
    (E/(name+'.log')).write_text(log,encoding='utf-8');print(log,flush=True);assert p.returncode==0

src=(R/'scripts/check_integration13_immediate_20261109.py').read_text(encoding='utf-8')
src=src[:src.index("src=OUT/")];ns={'__file__':str(R/'scripts/check_integration13_immediate_20261109.py')};exec(src,ns);unit=ns['unit']
unit=unit.replace('static uint32_t platform=1,','volatile uint32_t g_pick_trial_profile,g_pick_trial_requested,g_pick_trial_scurve;\nstatic uint32_t fixture_tick;\nstatic uint32_t xTaskGetTickCount(void){return fixture_tick;}\nstatic uint32_t platform=1,')
unit=unit.replace(' uint32_t request=g_it_request;', ' fixture_tick=now+7U;\n uint32_t request=g_it_request;')
unit=unit.replace('now+2199','now+2056').replace('now+2200','now+2057')
unit=unit.replace('g_it_auto_release_ms[item]==now+2000-1000','g_it_auto_release_ms[item]==now+2007-1000')
unit=unit.replace('assert(g_it_request==3);','assert(g_it_request==3);assert(g_it_auto_next_request_ms[item]-g_it_auto_release_ms[item]==50);')
unit=unit.replace('waits199/200ms boundary','waits49/50ms from actual PWM timestamp boundary')
run('owner-release50',unit,P/'CHASSIS/USER')

unit=(R/'evidence/integration/2026-11-03/early-scan-startup/dm-driver-unit.c').read_text(encoding='utf-8')
unit=unit.replace('static float actual,last_target,last_speed;','static float actual,last_target,last_speed,fixture_velocity,fixture_bias;')
unit=unit.replace('f->position_rad=actual;return 1;','f->position_rad=actual+fixture_bias;f->velocity_rad_s=fixture_velocity;return 1;')
extra=r'''
 reset(180);g_dm_parallel_group_active=g_dm_parallel_group_go=g_dm_integrated_owner=1;
 g_dm_posvel_speed_deg_s=300;g_dm_place_target_deg=37;g_dm_jog_request=5;
 for(uint32_t j=0;j<100 && g_dm_jog_state!=4;j++)steps(1);
 C(g_dm_jog_state==4);fixture_bias=2.0f*JOG_RAD_PER_DEG;steps(4);
 C(g_dm_jog_state==4 && s_settled==0); /* tighter1deg, no premature completion */
 fixture_bias=0;fixture_velocity=.036f;steps(4);C(g_dm_jog_state==4 && s_settled==0);
 fixture_velocity=0;steps(1);C(s_settled==1 && g_dm_jog_state==4);
 pending=0;steps(1);C(s_settled==1 && g_dm_jog_state==4); /* no fresh reply */
 pending=1;steps(1);C(s_settled==2 && g_dm_jog_state==4);
 steps(1);C(s_settled==3 && g_dm_jog_state==7 && g_dm_posvel_holding);
 g_dm_jog_request=3;steps(3);C(g_dm_jog_stop_result==1 && !g_dm_posvel_holding);
'''
unit=unit[:unit.index('int main(void)')]+r'int main(void){'+extra+r'''
 reset(180);g_dm_parallel_group_active=g_dm_parallel_group_go=0;g_dm_integrated_owner=1;
 g_dm_posvel_speed_deg_s=300;g_dm_place_target_deg=37;g_dm_jog_request=5;
 for(uint32_t j=0;j<100 && g_dm_jog_state!=4;j++)steps(1);
 C(g_dm_jog_state==4);s_settled=0;steps(14);C(g_dm_jog_state==4 && s_settled==14);
 steps(1);C(g_dm_jog_state==7 && s_settled==15); /* nonparallel mode retains15 */
 printf("actual scoped DM driver checks=%u failures=%u\n",checks,failures);return failures?1:0;}
'''
run('dm-tight-three-feedback',unit,P/'HARDWARE/DM_CAN2')
(E/'owner-driver-checks.json').write_text(json.dumps({'passed':True,'actual_owner':True,'actual_dm_driver':True,'mocked_io':True,'hardware_access':False},indent=2)+'\n')
