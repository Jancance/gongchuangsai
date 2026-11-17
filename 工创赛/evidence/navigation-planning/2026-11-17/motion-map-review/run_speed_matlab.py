from pathlib import Path
import subprocess,hashlib,json,datetime
R=Path.cwd();E=Path(__file__).resolve().parent
source=R/'3.stm32f407工程/CHASSIS/USER/motion_profile.c'
script=E/'verify_speed_feasibility.m'
receipt={'started_at':datetime.datetime.now().astimezone().isoformat(),'hardware_accessed':False,'actual_c_executed':False,'source_sha256':hashlib.sha256(source.read_bytes()).hexdigest(),'script_sha256':hashlib.sha256(script.read_bytes()).hexdigest()}
command=['E:/Program Files/Polyspace/R2020a/bin/matlab.exe','-batch',"addpath('"+E.as_posix()+"'); verify_speed_feasibility"]
receipt['command']=command
with (E/'speed_matlab.log').open('wb') as f:
 p=subprocess.run(command,stdout=f,stderr=subprocess.STDOUT,timeout=180,creationflags=subprocess.CREATE_NO_WINDOW)
receipt.update(returncode=p.returncode,finished_at=datetime.datetime.now().astimezone().isoformat())
(E/'speed_matlab_receipt.json').write_text(json.dumps(receipt,ensure_ascii=False,indent=2),'utf-8')
print(json.dumps(receipt,ensure_ascii=False));assert p.returncode==0
