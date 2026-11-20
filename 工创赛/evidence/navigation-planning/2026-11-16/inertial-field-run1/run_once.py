import importlib.util,json,os,subprocess,sys
from run_binding import E,R,CLIENT,validate,start
binding=validate('run')
ready=json.loads((E/'readonly_ready_receipt.json').read_text('utf8'));assert ready['accepted'] and ready['candidate_sha256']==__import__('run_binding').EXPECTED and ready['initialization_policy']=='download_initialized_no_extra_reset'
assert ready['wireless_port']==binding['wireless_port'] and ready['probe_uid']==binding['probe_uid']
monitor=json.loads((E/'postdownload_monitor_receipt.json').read_text('utf8'));assert monitor['returncode']==0
files=list((E/'postdownload_preflight').glob('*.summary.json'));assert len(files)==1
pre=json.loads(files[0].read_text('utf8'));assert pre['status']=='monitor_received' and pre['valid_frames']>0
spec=importlib.util.spec_from_file_location('field_client',CLIENT);m=importlib.util.module_from_spec(spec);spec.loader.exec_module(m)
last=pre['last'];assert m.Controller('run-once',3,8)._healthy(last) and last['debug_plan_id']==8
# Actual client opens a fresh stream and applies its own current-frame gates before START.
cmd=[sys.executable,str(CLIENT),'--port',binding['wireless_port'],'--mode','3','--action','run-once','--plan','forward210_40','--expect-powered-done','--seconds','30','--output',str(E/'serial')]
start('run',cmd)
env=os.environ.copy();env['PYTHONPATH']=str(R/'工创赛/tmp/serial_capture_deps')+os.pathsep+env.get('PYTHONPATH','')
with (E/'run_client.log').open('xb') as f:p=subprocess.run(cmd,stdout=f,stderr=subprocess.STDOUT,timeout=65,env=env)
files=list((E/'serial').glob('*.summary.json'));assert len(files)==1
result=json.loads(files[0].read_text('utf8'));result['returncode']=p.returncode
with (E/'result_summary.json').open('x',encoding='utf8') as f:json.dump(result,f,ensure_ascii=False,indent=2)
print(json.dumps(result,ensure_ascii=True));raise SystemExit(p.returncode)
