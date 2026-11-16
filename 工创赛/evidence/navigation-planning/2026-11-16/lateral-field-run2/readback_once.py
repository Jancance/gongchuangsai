import json,os,subprocess,sys
from run_binding import E,R,S,IMAGE,EXPECTED,validate,start
b=validate('readback');assert isinstance(b.get('expected_session'),int) and b['expected_session']>0
summary=json.loads((E/'result_summary.json').read_text('utf8'))
position=summary.get('last_position') or {}
assert position.get('session')==b['expected_session'] and position['session']>0, 'Session must match this run PM1, not historical binding'
cmd=[sys.executable,str(S/'read_capture_once.py'),'--axf',str(IMAGE),'--read-hardware','--expected-sha256',EXPECTED,'--expected-session',str(b['expected_session']),'--probe-uid',b['probe_uid'],'--output-dir',str(E/'capture')]
start('readback',cmd)
env=os.environ.copy();env['PYTHONPATH']=str(R/'工创赛/tmp/turn_probe_deps')+os.pathsep+env.get('PYTHONPATH','')
with (E/'capture_read_process.log').open('xb') as f:p=subprocess.run(cmd,stdout=f,stderr=subprocess.STDOUT,timeout=120,env=env)
with (E/'readback_process_receipt.json').open('x',encoding='utf8') as f:json.dump(dict(returncode=p.returncode),f)
raise SystemExit(p.returncode)
