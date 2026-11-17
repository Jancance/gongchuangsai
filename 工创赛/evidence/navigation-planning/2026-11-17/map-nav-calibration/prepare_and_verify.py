from pathlib import Path
import datetime,hashlib,json,subprocess
R=Path.cwd();E=Path(__file__).resolve().parent;U=R/'3.stm32f407工程/CHASSIS/USER'
files=['app_config.h','app_odometry.c','app_odometry.h','app_motion_runtime.c']
receipt={'started_at':datetime.datetime.now().astimezone().isoformat(),'hardware_accessed':False,'product_before':{}}
for name in files:
 p=U/name;b=p.read_bytes();d=E/'before'/name;d.parent.mkdir(exist_ok=True);d.write_bytes(b)
 receipt['product_before'][name]={'sha256':hashlib.sha256(b).hexdigest(),'crlf':b.count(b'\r\n'),'bare_lf':b.count(b'\n')-b.count(b'\r\n'),'nonascii_sha256':hashlib.sha256(bytes(x for x in b if x>=128)).hexdigest()}
(E/'preparation.json').write_text(json.dumps(receipt,indent=2),'utf8')
script=E/'verify_map_nav.m';receipt['script_sha256']=hashlib.sha256(script.read_bytes()).hexdigest()
command=['E:/Program Files/Polyspace/R2020a/bin/matlab.exe','-batch',"addpath('"+E.as_posix()+"'); verify_map_nav"]
receipt['command']=command
with (E/'matlab.log').open('wb') as f:
 p=subprocess.run(command,stdout=f,stderr=subprocess.STDOUT,timeout=180,creationflags=subprocess.CREATE_NO_WINDOW)
receipt.update(returncode=p.returncode,finished_at=datetime.datetime.now().astimezone().isoformat(),product_unchanged=all(hashlib.sha256((U/n).read_bytes()).hexdigest()==receipt['product_before'][n]['sha256'] for n in files))
(E/'matlab_receipt.json').write_text(json.dumps(receipt,indent=2),'utf8')
print(json.dumps(receipt,indent=2));assert p.returncode==0 and receipt['product_unchanged']
