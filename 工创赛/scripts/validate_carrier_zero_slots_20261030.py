"""复用已验证的实际状态机/调度场景，检查新0..240配置及三个候选点。"""
from pathlib import Path
import ast
import hashlib
import json
import subprocess

ROOT=Path(__file__).resolve().parents[1]
BASE=ROOT/'tmp/carrier_zero_slots_20261030'
STAGE=BASE/'project'
EV=ROOT/'evidence/p2-arm/2026-10-30/carrier-zero-slots'
HOST=BASE/'host-check'
old=ast.parse((ROOT/'scripts/validate_carrier_pb1_20261030.py').read_text(encoding='utf-8'))
model=next(ast.literal_eval(n.value) for n in old.body if isinstance(n,ast.Assign)
           and any(isinstance(t,ast.Name) and t.id=='MODEL' for t in n.targets))
model=model.replace(' Carrier_Init();',''' Carrier_Init();
 assert(g_carrier_debug_angle==0.0f);
 assert(CARRIER_SLOT_1_DEG==0.0f&&CARRIER_SLOT_2_DEG==120.0f&&CARRIER_SLOT_3_DEG==240.0f);
 g_carrier_debug_angle=135.0f;''',1)
model=model.replace('119.99f:150.01f','-0.01f:240.01f')
model=model.replace('is(s,"min")?120:150','is(s,"min")?0:240')
model=model.replace('is(s,"min")?1389:1611','is(s,"min")?500:2278')
model=model.replace(' if(is(s,"boot"))',''' if(is(s,"slot0")||is(s,"slot120")||is(s,"slot240")){
  g_carrier_debug_angle=is(s,"slot0")?0:is(s,"slot120")?120:240;
  start(0);assert(writes==1&&g_carrier_state==1);
  assert(last_pulse==(is(s,"slot0")?500:is(s,"slot120")?1389:2278));
  for(unsigned t=100;t<=1500;t+=100)Carrier_Service(1,0,t);
  assert(g_carrier_state==2&&g_carrier_result==1&&!g_carrier_signal_active);
 }
 else if(is(s,"beyond240")||is(s,"wrap360")){
  g_carrier_debug_angle=is(s,"beyond240")?241:360;
  start(0);assert(writes==0&&g_carrier_state==9&&g_carrier_result==3);
 }
 else if(is(s,"boot"))''',1)
t=(STAGE/'USER/main.c').read_bytes()
a=t.index(b'static void App_CarrierStep(void)')
b=t.index(b'static void App_CanTask(void *argument)',a)
HOST.mkdir(parents=True,exist_ok=True)
(HOST/'model.c').write_text(model.replace('@@WRAPPER@@',t[a:b].decode('ascii')),encoding='utf-8')
def run(args):
    r=subprocess.run(args,capture_output=True,text=True,timeout=30)
    assert r.returncode==0,(args,r.stdout,r.stderr)
    return r.stdout.strip()
exe=HOST/'carrier-zero-check.exe'
run(['C:/mingw/mingw64/bin/gcc.exe','-std=c99','-Wall','-Wextra',
     '-I',str(STAGE/'HARDWARE/CARRIER'),str(HOST/'model.c'),
     str(STAGE/'HARDWARE/CARRIER/carrier.c'),'-o',str(exe)])
cases=['boot','disabled','init_fail','not_ready','nan','inf','low','high','min','max',
       'write_fail','unknown','window','busy','no_repeat','post_window_pause','gap',
       'power_loss','disable','conflict','abort','stop_idle','wrap','rearm',
       'mode','safety','vm','identity','guard_starts','guard_stops']
cases+=['block_'+s for s in ['dm','lhome','ehome','emove','lmove','dmreq','lmreq',
                           'lhreq','emreq','ehreq','lfault','lhfault','efault','ehfault']]
cases+=['slot0','slot120','slot240','beyond240','wrap360']
results=[run([str(exe),c]) for c in cases]
m=json.loads((EV/'manifest.json').read_text(encoding='utf-8'))
for rel,h in m['protected'].items():assert hashlib.sha256((STAGE/rel).read_bytes()).hexdigest()==h,rel
h=hashlib.sha256((STAGE/m['file']).read_bytes()).hexdigest()
assert h==m['after']
report={'passed':True,'runtime_cases':len(cases),'cases':cases,'configuration_sha256':h,
        'protected_source_count':len(m['protected']),'actual_state_machine_and_main_wrapper_tested':True,
        'physical_zero_or_slot_alignment_verified':False}
(EV/'host-review.json').write_text(json.dumps(report,ensure_ascii=False,indent=2)+'\n',encoding='utf-8')
(EV/'host-check.log').write_text('\n'.join(results)+'\n',encoding='utf-8')
print('PASS:',len(cases),'runtime cases; default zero, 0/120/240 mapping, invalid angles and all prior interlocks.')
