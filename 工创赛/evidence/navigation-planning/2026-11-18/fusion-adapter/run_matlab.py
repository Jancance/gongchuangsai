from pathlib import Path
import subprocess,hashlib,json,datetime
E=Path(__file__).resolve().parent;m=E/'verify_fusion_adapter.m';U=Path.cwd()/'3.stm32f407工程/CHASSIS/USER';cmd=['E:/Program Files/Polyspace/R2020a/bin/matlab.exe','-batch',"addpath('"+E.as_posix()+"');verify_fusion_adapter"]
d={'started_at':datetime.datetime.now().isoformat(),'script_sha256':hashlib.sha256(m.read_bytes()).hexdigest(),'command':cmd,'product_adapter_written':(U/'fusion_adapter.c').exists(),'hardware_accessed':False}
with (E/'matlab.log').open('wb') as f:p=subprocess.run(cmd,stdout=f,stderr=subprocess.STDOUT)
d.update(returncode=p.returncode,finished_at=datetime.datetime.now().isoformat());(E/'matlab_receipt.json').write_text(json.dumps(d,indent=2),encoding='utf-8');print(json.dumps(d));print((E/'matlab.log').read_text(encoding='gbk',errors='replace')[-1300:]);raise SystemExit(p.returncode)
