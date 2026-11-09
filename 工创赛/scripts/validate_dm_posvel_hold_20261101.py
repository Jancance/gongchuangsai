from pathlib import Path
import json, subprocess, shutil
R = Path(__file__).resolve().parents[1]
S = R / 'tmp/dm_posvel_20261101/project'
H = R / 'tmp/dm_posvel_20261101/host'
E = R / 'evidence/p2-arm/2026-11-01/dm-posvel'
old = H / 'motion-before-hold.c'
if not old.exists():
    shutil.copyfile(H / 'motion.c', old)
t = old.read_text(encoding='utf-8')
t = t.replace('ab(target-original)<.0001f','ab(target-position)<.0001f')
t = t.replace('is("negative")?-45.0f:45.0f','(is("negative")||is("next_from_hold"))?-45.0f:45.0f')
t = t.replace('is("already_mode2")||is("limit")', 'is("already_mode2")||is("limit")||is("holding_with_other_axis")||is("next_from_hold")||is("hold_stop")')
t = t.replace('is("stale")?4:is("txfail")?5:(is("noreply")||is("stoplost"))?6:', 'is("stale")?4:is("txfail")?5:is("noreply")?6:is("stoplost")?2:')
t = t.replace('  if(g_dm_jog_state==7 || g_dm_jog_state==8)break;', '''  if(is("stoplost")&&g_dm_jog_state==7){g_dm_jog_request=3;continue;}
  if(g_dm_jog_state==7 || g_dm_jog_state==8)break;''')
t = t.replace('assert(g_dm_jog_state==7 && g_dm_jog_stop_result==1);', 'assert(g_dm_jog_state==7 && g_dm_jog_stop_result==0 && g_dm_posvel_holding==1 && g_dm_can2_motor_state==1);')
t = t.replace('assert(enable_count==1 && disable_count==1);', 'assert(enable_count==1 && disable_count==0);')
marker = '  if(point_case){\n   float expected_raw='
addition = '''  if(is("holding_with_other_axis")){
   for(i=0;i<100;i++){now+=12;DM_CAN2_ModuleService(1,1);}
   assert(g_dm_jog_state==7&&g_dm_posvel_holding&&disable_count==0&&g_dm_can2_motor_state==1);
  }
  if(is("next_from_hold")){
   g_dm_jog_request=2;
   for(i=0;i<200;i++){now+=12;DM_CAN2_ModuleService(1,0);if(g_dm_jog_state==7&&enable_count==2)break;}
   assert(enable_count==2&&disable_count==1&&g_dm_posvel_holding&&g_dm_jog_state==7);
  }
  g_dm_jog_request=3;
  for(i=0;i<50;i++){now+=12;DM_CAN2_ModuleService(1,0);if(g_dm_jog_stop_result==1)break;}
  assert(g_dm_jog_state==8&&g_dm_jog_result==2&&g_dm_jog_stop_result==1&&!g_dm_posvel_holding&&g_dm_can2_motor_state==0);
'''
assert marker in t
t = t.replace(marker, addition + marker, 1)
(H / 'motion-hold.c').write_text(t, encoding='utf-8')
mod = S / 'HARDWARE/DM_CAN2'
gcc = 'C:/mingw/mingw64/bin/gcc.exe'
z = subprocess.run([gcc, '-std=c99', '-Wall', '-Wextra', '-Werror', '-I'+str(H), '-I'+str(mod), str(mod/'dm_can2_protocol.c'), str(mod/'dm_can2_motion.c'), str(H/'motion-hold.c'), '-lm', '-o', str(H/'motion-hold.exe')], capture_output=True, text=True)
assert z.returncode == 0, z.stderr
m = json.loads((E/'manifest.json').read_text(encoding='utf-8'))
cases = m['motion_cases'] + ['holding_with_other_axis', 'next_from_hold', 'hold_stop']
logs = []
for case in cases:
    print(case, flush=True)
    z = subprocess.run([str(H/'motion-hold.exe'), case], capture_output=True, text=True, timeout=5)
    assert z.returncode == 0, (case, z.stdout, z.stderr)
    logs.append(z.stdout + z.stderr)
(E/'hold-host-check.log').write_text('\n'.join(logs), encoding='utf-8')
m.update(hold_motion_cases=cases, host_pass=True, arrival_holds_enabled=True, explicit_stop_disables=True)
(E/'manifest.json').write_text(json.dumps(m, ensure_ascii=False, indent=2), encoding='utf-8')
print(str(len(cases))+' hold+motion scenarios passed')
