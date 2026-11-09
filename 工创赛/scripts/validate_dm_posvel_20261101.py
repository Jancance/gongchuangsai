from pathlib import Path
import json,shutil,subprocess
R=Path(__file__).resolve().parents[1];S=R/'tmp/dm_posvel_20261101/project';H=R/'tmp/dm_posvel_20261101/host';E=R/'evidence/p2-arm/2026-11-01/dm-posvel'
shutil.copytree(R/'tmp/dm_can2_points_20261029/host-check',H,dirs_exist_ok=True,ignore=shutil.ignore_patterns('*.exe'))
p=H/'motion.c';t=p.read_text(encoding='utf-8')
def once(t,a,b):assert t.count(a)==1,a[:120];return t.replace(a,b,1)
t=once(t,'static uint32_t point_case;','static uint32_t point_case,mode_writes;')
t=t.replace('if(is("badprofile"))g_dm_can2_ctrl_mode=2;','if(is("badprofile"))g_dm_can2_ctrl_mode=3;')
t=once(t,' txcount++;failed=0;',""" txcount++;failed=0;
 if(t->StdId==0x7ff){
  assert(t->DLC==8&&t->Data[0]==1&&t->Data[1]==0&&t->Data[2]==0x55&&t->Data[3]==0x0a&&t->Data[4]==2);
  assert(!enabled);mode_writes++;
  if(!is("mode_ignored"))g_dm_can2_ctrl_mode=2;
  return 0;
 }""")
t=t.replace('assert(t->StdId==1 && t->IDE==0 && t->RTR==0 && t->DLC==8);','assert((t->StdId==0x101||t->StdId==0x7ff) && t->IDE==0 && t->RTR==0 && t->DLC==8);')
a=t.index('  float target;');b=t.index('  if(enabled && is("noreply"))',a)
t=t[:a]+"""  float target,speed;
  memcpy(&target,t->Data,4);memcpy(&speed,t->Data+4,4);
  assert(ab(target-original)<(point_case?183.0f:45.1f)*0.01745329252f);
  if(!enabled){assert(speed==0.0f);assert(ab(target-original)<.0001f);}
  else {assert(ab(speed-300.0f*0.01745329252f)<.00001f);position=target;}
  if(enabled && (is("tracking")||is("wait_forever")))position=original;
  if(enabled && is("bounds"))position=original-0.1f;
  if(enabled && is("limit"))torque=2350;
"""+t[b:]
# Initial mode can already be2; no unnecessary write in that case.
t=once(t,' assert(argc==2);scenario=argv[1];',' assert(argc==2);scenario=argv[1];\n if(is("already_mode2"))g_dm_can2_ctrl_mode=2;')
t=t.replace(' && i>80',' && i>10')
# Deliberate stalled motor keeps returning fresh valid feedback: action wait is unbounded.
t=once(t,' assert(i<2600);',""" if(is("wait_forever")||is("tracking")){
  assert(i==2600 && g_dm_jog_state==4 && g_dm_jog_result==0);
  assert(enable_count==1&&disable_count==0);
  g_dm_jog_request=3;
  for(i=0;i<30;i++){now+=10;DM_CAN2_ModuleService(1,0);if(g_dm_jog_stop_result==1)break;}
  assert(g_dm_jog_result==2&&g_dm_jog_stop_result==1);puts("PASS infinite wait >26s, explicit stop");return 0;
 }
 if(is("mode_ignored")){
  assert(mode_writes==1&&enable_count==0&&g_dm_jog_result==15&&g_dm_mode_change_state==4);puts("PASS mode verification rejects failed switch");return 0;
 }
 assert(i<2600);""")
t=t.replace('is("positive")||is("negative")||is("repeat")','is("positive")||is("negative")||is("repeat")||is("already_mode2")||is("limit")')
t=t.replace('is("tracking")?8:is("limit")?9:', 'is("bounds")?8:')
# One optional mode write is permitted before rejecting a point coordinate; no enable is allowed.
t=t.replace('))assert(txcount==0);','))assert(enable_count==0);')
t=t.replace(' assert(g_dm_jog_result==expected);',' assert(g_dm_jog_result==expected);\n if(expected==1)assert(mode_writes==(is("already_mode2")?0U:1U));')
p.write_text(t,encoding='utf-8')
mod=S/'HARDWARE/DM_CAN2';gcc='C:/mingw/mingw64/bin/gcc.exe';base=[gcc,'-std=c99','-Wall','-Wextra','-Werror','-I'+str(H),'-I'+str(mod)]
z=subprocess.run(base+[str(mod/'dm_can2_protocol.c'),str(mod/'dm_can2_motion.c'),str(p),'-lm','-o',str(H/'motion.exe')],capture_output=True,text=True);assert z.returncode==0,z.stderr
cases=['positive','negative','stale','txfail','noreply','fault','disabled','tracking','bounds','limit','busy','abort','repeat','gap','badprofile','stoplost','wait_forever','already_mode2','mode_ignored']+['point_'+n for n in ['pick','place','minus_turn','edit','gated','gate_drop','badtarget','nantarget','outside','wrong_direction','abort','repeat','stale','noreply','fault','gap','stoplost']]
logs=[]
for case in cases:
 z=subprocess.run([str(H/'motion.exe'),case],capture_output=True,text=True);assert z.returncode==0,(case,z.stdout,z.stderr);logs.append(z.stdout+z.stderr)
(E/'host-check.log').write_text('\n'.join(logs),encoding='utf-8');m=json.loads((E/'manifest.json').read_text(encoding='utf-8'));m['host_pass']=True;m['motion_cases']=cases;(E/'manifest.json').write_text(json.dumps(m,ensure_ascii=False,indent=2),encoding='utf-8');print(str(len(cases))+' real C motion scenarios passed')
