from pathlib import Path
import ast,hashlib,json,subprocess,sys
sys.dont_write_bytecode=True
R=Path.cwd();E=R/'工创赛/evidence/navigation-planning/2026-11-17/turn90-entry-fix';V=E/'independent-review';U=R/'3.stm32f407工程/CHASSIS/USER'
def sha(b):return hashlib.sha256(b).hexdigest()
def extract(s,sig):
 b=s.index(sig);e=s.index('{',b)+1;d=1
 while d:
  d+=(s[e]=='{')-(s[e]=='}');e+=1
 return s[b:e]
sources={n:(U/n).read_bytes() for n in ['app_motion_runtime.c','app_serial_runtime.c','app_heading_runtime.c','heading_session.c']}
motion=sources['app_motion_runtime.c'].decode('gbk');serial=sources['app_serial_runtime.c'].decode('gbk');heading=sources['app_heading_runtime.c'].decode('gbk');unit=(E/'actual-c/unit.c').read_bytes().decode('gbk');checks={}
for sig,new in [('static uint32_t App_CombinedLoadAction(',None),('uint32_t TurnStart(','static uint32_t RealTurnStart('),('static uint32_t App_PerimeterTurnStart(','static uint32_t RealPerimeterTurnStart(')]:
 a=extract(motion,sig);expected=a if new is None else a.replace(sig,new,1);assert unit.count(expected)==1,sig;checks[sig]=sha(a.encode('gbk'))
tele=serial[serial.index('    values[56] = (int32_t)(('):serial.index('    if (g_motion_plan_valid != 1U)')]
owner=heading[heading.index('            prepared_ok=(*inputs->route_turn_active) ?'):heading.index('            if(prepared_ok)prepared_ok=ChassisAuto_PrepareHeading')]
assert unit.count(tele)==unit.count(owner)==1
checks.update(telemetry_slice=sha(tele.encode('gbk')),owner_expression=sha(owner.encode('gbk')))
runner=(E/'check_integration.py').read_text('utf-8-sig')
runner=runner.replace("E=Path(__file__).resolve().parent","E=R/'工创赛/evidence/navigation-planning/2026-11-17/turn90-entry-fix'",1)
runner=runner.replace("T=E/('actual-c' if variant=='fixed' else 'actual-c-'+variant)","T=E/'independent-review'/('actual-c' if variant=='fixed' else 'actual-c-'+variant)",1)
runner=runner.replace("(E/'integration_checks.json').write_text","(E/'independent-review'/'integration_checks.json').write_text",1)
extra=r'''
 for(id=20;id<=21;id++){
  uint32_t k,rpms[6]={0,5,13,27,81,83};
  for(k=0;k<6;k++){
   reset_case(id,37);CK(AppDebugPlan_Load(id,s_motion_actions,&s_motion_action_count));s_motion_actions[0].base_rpm=rpms[k];CK(!App_CombinedLoadAction(0));CK(!turn_calls && !perimeter_calls && !move_calls);
  }
  reset_case(id,37);CK(AppDebugPlan_Load(id,s_motion_actions,&s_motion_action_count));s_motion_action_count=0;CK(!App_CombinedLoadAction(0));s_motion_action_count=9;CK(!App_CombinedLoadAction(0));
  reset_case(id,37);CK(AppDebugPlan_Load(id,s_motion_actions,&s_motion_action_count));s_motion_actions[0].relative_deg+=(id==20?.001f:-.001f);CK(!App_CombinedLoadAction(0));
  reset_case(id,37);CK(AppDebugPlan_Load(id,s_motion_actions,&s_motion_action_count));g_heading_exec_phase=HR_RUN;CK(!App_CombinedLoadAction(0));CK(!s_motion_has_action);
  reset_case(id,37);CK(AppDebugPlan_Load(id,s_motion_actions,&s_motion_action_count));s_combined_turn=1;s_motion_has_action=1;g_combined_test.state=1;make_telemetry(values);CK(values[56]==3 && values[57]==(-2147483647L-1L));
 }
 reset_case(22,37);s_motion_action_count=1;s_motion_actions[0].kind=MOTION_TURN;s_motion_actions[0].base_rpm=82;s_motion_actions[0].relative_deg=90;CK(!App_CombinedLoadAction(0));
'''
needle="unit=prefix+turn+'\\n'+perimeter+'\\n'+wrappers+load+'\\n'+milli+tele+test"
assert runner.count(needle)==1
runner=runner.replace(needle,"test=test.replace(' return 0;\\n}\\n',"+repr(extra)+"+' return 0;\\n}\\n');\n"+needle)
ast.parse(runner);(V/'rerun_integration.py').write_text(runner,encoding='utf8')
cp=subprocess.run([sys.executable,'-B','-X','utf8',str(V/'rerun_integration.py')],cwd=R,capture_output=True,text=True,encoding='utf8',timeout=120)
(V/'rerun.log').write_text(cp.stdout+cp.stderr,encoding='utf8');assert cp.returncode==0,cp.stdout+cp.stderr
rr=json.loads((V/'integration_checks.json').read_text('utf8'));assert rr['c_assertions']>10482 and rr['session_cases']==36
for n,b in sources.items():assert (U/n).read_bytes()==b and rr['source_sha256'][n]==sha(b),n
for n in ['app_motion_runtime.c','app_serial_runtime.c']:assert sources[n].count(b'\n')==sources[n].count(b'\r\n'),n
out=dict(status='PASS',hardware_accessed=False,product_files_written=False,source_sha256={n:sha(b) for n,b in sources.items()},exact_extraction_checks=checks,c_assertions=rr['c_assertions'],session_cases=36,client_runs=rr['client_runs'],extra_checks='Exact RPM/angle/count/id negatives, busy heading rejection and active telemetry.',limitations=rr['limitations'])
(V/'review_verification.json').write_text(json.dumps(out,ensure_ascii=False,indent=2),encoding='utf8');print(json.dumps(out,ensure_ascii=False))

