from pathlib import Path
import datetime,hashlib,json,re,subprocess
R=Path.cwd();E=Path(__file__).resolve().parent
P=R/'3.stm32f407工程/gongxun-f407.uvprojx'
T=R/'工创赛/tmp/right-short40-build-20261116';name='right150_40'
T.mkdir(exist_ok=True);(T/name).mkdir(exist_ok=True)
defines='APP_VM_CALIBRATION_ONLY_ENABLE=0,APP_VM_ONLY_ENABLE=0,APP_WHEEL_ENABLE_HOLD_ENABLE=0,APP_LEFT40_DRIFT_TRIM_ENABLE=1,APP_BASELINE40_ENABLE=1,VEHICLE_INTEGRATED_ENABLE=0,VEHICLE_STAGE1_ENABLE=0,APP_FUSION_CAPTURE_STATIC_TEST=0,APP_FUSION_CAPTURE_ENABLE=1'
text=P.read_text('utf-8').replace('.\\',str(P.parent)+'\\')
for tag in ['OutputDirectory','ListingPath']:
    text=re.sub('<'+tag+'>.*?</'+tag+'>',lambda m:'<'+tag+'>'+name+'\\</'+tag+'>',text)
text=text.replace('<Define>','<Define>'+defines+',')
project=T/(name+'.uvprojx');project.write_text(text,'utf-8')
command=['D:/Keil_v5/UV4/UV4.exe','-r',str(project),'-t','Vehicle_Integrated','-o',str(E/'rebuild.log')]
assert '-f' not in command
receipt={'started_at':datetime.datetime.now().astimezone().isoformat(),'command':command,'hardware_accessed':False,'recipe_only_before_execution':False}
cp=subprocess.run(command,timeout=300,creationflags=subprocess.CREATE_NO_WINDOW)
log=(E/'rebuild.log').read_bytes().decode('cp936',errors='replace')
receipt.update(returncode=cp.returncode,success=cp.returncode==0 and '0 Error(s), 0 Warning(s)' in log)
image=T/name/'Vehicle_Integrated.axf'
if image.exists():receipt.update(axf=str(image),axf_sha256=hashlib.sha256(image.read_bytes()).hexdigest())
(E/'rebuild_receipt.json').write_text(json.dumps(receipt,indent=2),'utf-8')
assert receipt['success'],log[-3000:]
print(json.dumps(receipt))
