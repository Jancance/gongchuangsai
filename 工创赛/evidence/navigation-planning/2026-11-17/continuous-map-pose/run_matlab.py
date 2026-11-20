from pathlib import Path
import json,hashlib,datetime,subprocess
R=Path.cwd();E=Path(__file__).resolve().parent
script=E/'verify_map_lifecycle.m';receipt={'started_at':datetime.datetime.now().astimezone().isoformat(),'script_sha256':hashlib.sha256(script.read_bytes()).hexdigest(),'hardware_accessed':False,'product_before':json.loads((E/'preparation.json').read_text('utf-8'))['product_before']}
command=['E:/Program Files/Polyspace/R2020a/bin/matlab.exe','-batch',"addpath('"+E.as_posix()+"'); verify_map_lifecycle"];receipt['command']=command
with (E/'matlab.log').open('wb') as f:p=subprocess.run(command,stdout=f,stderr=subprocess.STDOUT,timeout=180,creationflags=subprocess.CREATE_NO_WINDOW)
receipt.update(returncode=p.returncode,finished_at=datetime.datetime.now().astimezone().isoformat())
(E/'matlab_receipt.json').write_text(json.dumps(receipt,ensure_ascii=False,indent=2),'utf-8');print(json.dumps(receipt,ensure_ascii=False));assert p.returncode==0
