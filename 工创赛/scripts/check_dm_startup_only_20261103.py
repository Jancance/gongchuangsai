"""实际独立DM编排器：按键前不动、无CAN1伸缩交易、保持/取消/故障。"""
from pathlib import Path
import json
import subprocess

R = Path(__file__).resolve().parents[1]
S = R / 'tmp/dm_startup_only_20261103'
E = R / 'evidence/integration/2026-11-03/dm-startup-only'
OLD = R / 'evidence/integration/2026-11-03/early-scan-startup'
unit = (OLD / 'axes-unit.c').read_text(encoding='utf-8')
unit = unit[:unit.index('int main(void)')]
unit = unit.replace('static uint32_t tick=100,', 'static uint32_t can1_calls;\nstatic uint32_t tick=100,')
unit = unit.replace(' CanRxMsg *p=out;', ' can1_calls++;CanRxMsg *p=out;')
unit = unit.replace(' tick=100;power_on=1;', ' can1_calls=0;tick=100;power_on=1;')
unit += r'''
int main(void){
#if APP_MOTOR_OFFLINE_ONLY
 C(!StartupAxes_Begin());StartupAxes_Service(1,1);C(!can1_calls && !dm_calls && !g_startup_axes_vm_request);
#else
 reset();for(unsigned i=0;i<10;i++)step();C(!dm_calls && !can1_calls && !g_startup_axes_vm_request);
 g_startup_extension_reference_valid=0;fail_query=1;C(StartupAxes_Begin());C(!StartupAxes_Begin());
 power_on=0;step();C(!dm_calls && !can1_calls);power_on=1;
 step();C(g_startup_axes_state==2 && g_dm_posvel_speed_deg_s==150.0f && !g_dm_startup_go);
 for(unsigned i=0;i<5;i++)step();C(!g_dm_startup_go && !can1_calls);
 dm_prepared=1;step();C(g_startup_axes_state==3 && g_dm_startup_go && !can1_calls);
 step();C(!StartupAxes_Holding());dm_done=1;step();
 C(StartupAxes_Holding() && g_startup_axes_state==4 && g_dm_posvel_speed_deg_s==300.0f && !can1_calls);
 for(unsigned i=0;i<20;i++){step();StartupAxes_Safety(1);}C(StartupAxes_Holding() && !can1_calls);
 StartupAxes_Cancel();step();C(g_startup_axes_state==6 && !g_startup_axes_vm_request && !g_dm_posvel_holding && !can1_calls);
 C(!StartupAxes_Begin());
 reset();C(StartupAxes_Begin());step();g_dm_jog_state=8;step();C(g_startup_axes_state==5 && g_startup_axes_cause==7);step();C(!can1_calls && !g_startup_axes_vm_request);
 reset();C(StartupAxes_Begin());step();tick+=151;StartupAxes_Safety(1);C(g_startup_axes_state==5 && !g_startup_axes_vm_request);step();C(!can1_calls);
 reset();C(StartupAxes_Begin());step();StartupAxes_Safety(0);C(g_startup_axes_state==5 && !g_startup_axes_vm_request);step();C(!can1_calls);
 reset();C(StartupAxes_Begin());step();g_dm_jog_first_fault=12;StartupAxes_Safety(1);C(g_startup_axes_cause==7 && !g_startup_axes_vm_request);step();C(!can1_calls);
 reset();C(StartupAxes_Begin());for(unsigned i=0;i<760;i++){step();StartupAxes_Safety(1);}C(g_startup_axes_state==5 && g_startup_axes_cause==3 && !can1_calls);
#endif
 printf("DM-only checks=%u failures=%u\n",checks,failed);return failed?1:0;
}
'''
(E / 'dm-only-unit.c').write_text(unit, encoding='utf-8')
reports = []
for offline in [0, 1]:
    exe = E / f'dm-only-{offline}.exe'
    p = subprocess.run(['C:/mingw/mingw64/bin/gcc.exe', '-std=c99',
                        f'-DAPP_MOTOR_OFFLINE_ONLY={offline}',
                        '-I' + str(OLD / 'axis-stubs'), '-I' + str(S / 'CHASSIS/USER'),
                        str(E / 'dm-only-unit.c'), '-lm', '-o', str(exe)],
                       capture_output=True, text=True, errors='replace')
    assert p.returncode == 0, p.stderr
    p = subprocess.run([str(exe)], capture_output=True, text=True)
    print(offline, p.stdout, flush=True)
    assert p.returncode == 0, p.stdout
    reports.append(dict(offline=offline, passed=True, output=p.stdout))
(E / 'dm-only-check.json').write_text(json.dumps(dict(actual_c=True,
    hardware_accessed=False, peripherals_mocked=True, cases=reports), indent=2), encoding='utf-8')
