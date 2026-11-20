from pathlib import Path
import subprocess,hashlib,json,datetime
E=Path(__file__).resolve().parent;m=E/'verify_fusion_reference.m';U=Path.cwd()/'3.stm32f407工程/CHASSIS/USER';command=['E:/Program Files/Polyspace/R2020a/bin/matlab.exe','-batch',"addpath('"+E.as_posix()+"'); verify_fusion_reference"]
r={'started_at':datetime.datetime.now().isoformat(),'script_sha256':hashlib.sha256(m.read_bytes()).hexdigest(),'command':command,'hardware_accessed':False,'product_sha_before':{p.name:hashlib.sha256(p.read_bytes()).hexdigest() for p in [U/'fusion_reference.c',U/'fusion_reference.h'] if p.exists()}}
with (E/'matlab.log').open('w',encoding='utf-8') as f:p=subprocess.run(command,stdout=f,stderr=subprocess.STDOUT)
r.update(returncode=p.returncode,finished_at=datetime.datetime.now().isoformat());(E/'matlab_receipt.json').write_text(json.dumps(r,indent=2),encoding='utf-8');print(json.dumps(r));print((E/'matlab.log').read_text(encoding='utf-8',errors='replace')[-1300:]);raise SystemExit(p.returncode)
