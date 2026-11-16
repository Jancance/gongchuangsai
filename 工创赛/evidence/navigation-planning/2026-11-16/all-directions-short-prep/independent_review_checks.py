from pathlib import Path
import json,hashlib,importlib.util,ast,sys
R=Path.cwd();E=Path(__file__).resolve().parent;U=R/'3.stm32f407工程/CHASSIS/USER';sha=lambda p:hashlib.sha256(Path(p).read_bytes()).hexdigest();checks=0
def ck(x):
 global checks
 checks+=1;assert x
before=json.loads((E/'inputs_before.json').read_text('utf8'));delta=[p for p,h in before.items() if sha(R/p)!=h]
expected=['app_entry.c','app_entry.h','app_motion_runtime.c','app_serial_runtime.c','motion_trial40.h','chassis_auto_test.c'];ck(set(Path(p).name for p in delta)==set(expected));ck(len(delta)==6)
formats={}
for n in expected:
 a=(E/'source_before'/n).read_bytes();b=(U/n).read_bytes();a.decode('gbk');b.decode('gbk');ck(not b.startswith(b'\xef\xbb\xbf'));ck(a.count(b'\n')==a.count(b'\r\n'));ck(b.count(b'\n')==b.count(b'\r\n'));ck(a.count(b'\r\r\n')==b.count(b'\r\r\n')==0);formats[n]=dict(original_CRLF=a.count(b'\r\n'),current_CRLF=b.count(b'\r\n'),GBK_decodable=True,current_sha256=sha(U/n))
a=(E/'source_before/chassis_auto_test.c').read_bytes().decode('gbk');b=(U/'chassis_auto_test.c').read_bytes().decode('gbk')
a=a.replace('(g_debug_plan_id==DEBUG_PLAN_LEFT210_40 &&','((g_debug_plan_id==DEBUG_PLAN_LEFT210_40 || g_debug_plan_id==DEBUG_PLAN_LEFT150_40 || g_debug_plan_id==DEBUG_PLAN_LEFT30_40) &&').replace('s_lateral_enabled=g_debug_plan_id==DEBUG_PLAN_FORWARD210_40 &&','s_lateral_enabled=(g_debug_plan_id==DEBUG_PLAN_FORWARD210_40 || g_debug_plan_id==DEBUG_PLAN_FORWARD150_40 || g_debug_plan_id==DEBUG_PLAN_FORWARD30_40) &&').replace('MotionTrial40Allowed(prepared->motion_mode,prepared->base_rpm,prepared->distance_mm) &&','AppDebugPlan40Allowed(g_debug_plan_id,prepared->motion_mode,prepared->base_rpm,prepared->distance_mm) &&');ck(a==b)
p=E/'p1_serial_motion_powered_done.py';spec=importlib.util.spec_from_file_location('independent_client',p);m=importlib.util.module_from_spec(spec);spec.loader.exec_module(m)
expected_specs={8:(100,4,2100),9:(102,5,2100),10:(102,6,2100),11:(100,7,2100),12:(102,6,1500),13:(102,6,300),14:(100,4,1500),15:(100,4,300),16:(100,7,1500),17:(100,7,300),18:(102,5,1500),19:(102,5,300)};ck(m.BASELINE40_SPECS==expected_specs);ck(m.DEBUG_PLAN_IDS['right30']==3)
for plan,(rpm,kind,mm) in expected_specs.items():
 ck(m.baseline40_config(plan)==[plan,rpm,mm,400,150,300,130,12779]);c=m.Controller('run-once',3,plan,expected_complete_powered=True)
 for observed in expected_specs:
  orpm,okind,omm=expected_specs[observed];r=dict.fromkeys(m.FIELDS,0);r.update(debug_plan_id=observed,sequence_count=1,target_theoretical_milli_cm_s=40000,base_rpm=orpm,feedback_limit_rpm=130,tick_ms=1000,action_kind=okind,motion_mode=3,source_valid=1,selftest=1,protocol_selftest=1);c.baseline_config=m.baseline40_config(observed);c.baseline_tick=900;ck(c._speed_profile_healthy(r)==(observed==plan or observed==8))
 c.plan_selected=True
 for observed in expected_specs:
  orpm,okind,omm=expected_specs[observed];r=dict.fromkeys(m.FIELDS,0);r.update(debug_plan_id=observed,sequence_count=1,target_theoretical_milli_cm_s=40000,base_rpm=orpm,feedback_limit_rpm=130,tick_ms=1000,action_kind=okind,motion_mode=3,source_valid=1,selftest=1,protocol_selftest=1);c.baseline_config=m.baseline40_config(observed);c.baseline_tick=900;ck(c._speed_profile_healthy(r)==(observed==plan))
for verb in ['PLAN20','PLAN014','PLAN14x','PLAN150','PLAN019','PLAN19x','PLAN-1','PLAN+14','PLAN']:
 try:m.command(verb)
 except ValueError:ck(True)
 else:ck(False)
mat=json.loads((E/'actual_matlab_receipt.json').read_text('utf8'));prod=json.loads((E/'product_write_receipt.json').read_text('utf8'));ck(mat['returncode']==0);ck(mat['finished_at']<prod['recorded_at'])
gates=json.loads((E/'independent_gate_checks.json').read_text('utf8'));ck(all(v['status']=='PASS' for v in gates['cases'].values()))
c=json.loads((E/'actual_c_checks.json').read_text('utf8'));ck(all(v['status']=='PASS' for v in c['cases'].values()));ck(all(sha(U/n)==h for n,h in c['source_sha256'].items()))
h=json.loads((E/'host_checks.json').read_text('utf8'));ck(h['status']=='PASS');ck(h['client_sha256']==sha(p));ck(set(x['plan'] for x in h['simulations'])==set(range(8,20)));ck(all(x['simulated_START_frames']==1 for x in h['simulations']))
build=json.loads((E/'build_receipt.json').read_text('utf8'));ck(build['success'] and build['returncode']==0);ck(sha(build['axf'])==build['axf_sha256']);ck('-f' not in build['command']);ck(build['target']=='Vehicle_Integrated')
sys.path.insert(0,str(R/'工创赛/tmp/turn_probe_deps'));from elftools.elf.elffile import ELFFile
with Path(build['axf']).open('rb') as f:
 elf=ELFFile(f);flash=[dict(address=int(v['p_paddr']),size=int(v['p_filesz']),sha256=hashlib.sha256(v.data()).hexdigest()) for v in elf.iter_segments() if v['p_type']=='PT_LOAD' and int(v['p_filesz'])]
old=E.parent/'right30-and150-prep/frozen/Vehicle_Integrated.axf';ck(sha(old)=='3f8fe3bf732637ebe1e9765b5ebf686c7042d536bcfd1898dc163033d260b3cd')
report=dict(status='PASS_WITH_LIMITATIONS',reviewer='current model independent reviewer; not6.1-sol high',checks=checks,inputs=len(before),changed_sources=delta,formats=formats,client_sha256=sha(p),gate_source_sha256=sha(U/'chassis_auto_test.c'),independent_gate_cases=gates['cases'],new_axf_sha256=build['axf_sha256'],old_axf_sha256=sha(old),flash_sections=flash,hardware_accessed=False,serial_opened=False,serial_TX=0,git_mutation=False)
(E/'independent_review_checks.json').write_text(json.dumps(report,ensure_ascii=False,indent=2),'utf8');print(json.dumps(report,ensure_ascii=False))
