import json,os,subprocess,sys
from run_binding import E,R,CLIENT,validate,start
binding=validate('postreset_monitor')
sys.path.insert(0,str(R/'工创赛/tmp/serial_capture_deps'))
cmd=[sys.executable,str(CLIENT),'--port',binding['wireless_port'],'--mode','3','--action','monitor','--seconds','3','--output',str(E/'postreset_preflight')]
start('postreset_monitor',cmd)
env=os.environ.copy();env['PYTHONPATH']=str(R/'工创赛/tmp/serial_capture_deps')+os.pathsep+env.get('PYTHONPATH','')
with (E/'postreset_monitor_client.log').open('xb') as f:p=subprocess.run(cmd,stdout=f,stderr=subprocess.STDOUT,timeout=12,env=env)
with (E/'postreset_monitor_receipt.json').open('x',encoding='utf8') as f:json.dump(dict(returncode=p.returncode,monitor_only=True),f)
raise SystemExit(p.returncode)
