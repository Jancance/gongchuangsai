from pathlib import Path
import json,hashlib,datetime,subprocess
E=Path(__file__).parent;b=json.loads((E/'execution_binding.json').read_text('utf8'));sha=lambda p:hashlib.sha256(Path(p).read_bytes()).hexdigest()
assert b['root_release'] and b['execution_permitted'] and b['plan_id']==20
assert sha(b['image'])==b['candidate_sha256'] and sha(b['project'])==b['project_sha256']
r={'candidate_sha256':b['candidate_sha256'],'attempts':1,'automatic_retry':False,'download_initializes_MCU':True,'separate_uniform_reset_follows':1,'serial_TX_bytes':0,'START_commands':0,'started_at':datetime.datetime.now().astimezone().isoformat()}
with (E/'download_once_started.json').open('x',encoding='utf8') as f:json.dump(r,f,indent=2)
cmd=['D:/Keil_v5/UV4/UV4.exe','-f',b['project'],'-t','Vehicle_Integrated','-o',str(E/'download.log')];r['command']=cmd
try:
 p=subprocess.run(cmd,capture_output=True,timeout=45,creationflags=subprocess.CREATE_NO_WINDOW);r['returncode']=p.returncode;(E/'download_process.log').write_bytes(p.stdout+p.stderr)
except Exception as e:r['error']=repr(e)
log=(E/'download.log').read_bytes().decode('gbk',errors='replace') if (E/'download.log').exists() else ''
r['markers']={m:m in log for m in ['Erase Done.','Programming Done.','Verify OK.','Application running ...']};r['candidate_sha256_after']=sha(b['image']);r['success']=r.get('returncode')==0 and all(r['markers'].values()) and r['candidate_sha256_after']==b['candidate_sha256'];r['finished_at']=datetime.datetime.now().astimezone().isoformat()
(E/'download_once_receipt.json').write_text(json.dumps(r,indent=2),'utf8');print(json.dumps(r,indent=2));print(log[-2000:]);assert r['success'],'First download failed; no retry/reset/START'
b['download_receipt_sha256']=sha(E/'download_once_receipt.json');(E/'execution_binding.json').write_text(json.dumps(b,indent=2),'utf8')
