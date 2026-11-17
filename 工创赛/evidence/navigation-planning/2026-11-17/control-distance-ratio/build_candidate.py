from pathlib import Path
import datetime,hashlib,json,re,subprocess,sys
R=Path.cwd();E=Path(__file__).resolve().parent
P=R/'3.stm32f407工程/gongxun-f407.uvprojx'
T=R/'工创赛/tmp/control-ratio-build-20261117';name='control_ratio_off' if '--nav-off' in sys.argv else 'control_ratio'
T.mkdir(exist_ok=True);(T/name).mkdir(exist_ok=True)
defines=('APP_LEFT_NAV_RESIDUAL_ENABLE=0,APP_DIRECTION_NAV_RESIDUAL_ENABLE=0,APP_MAIN_NAV_RESIDUAL_ENABLE=0,APP_CONTROL_DISTANCE_RATIO_ENABLE=0,' if '--nav-off' in sys.argv else 'APP_LEFT_NAV_RESIDUAL_ENABLE=1,APP_DIRECTION_NAV_RESIDUAL_ENABLE=1,APP_MAIN_NAV_RESIDUAL_ENABLE=1,APP_CONTROL_DISTANCE_RATIO_ENABLE=1,')+'APP_RIGHT40_FORWARD_TRIM_ENABLE=1,APP_TURN90_90_ENABLE=1,APP_TURN_BRAKE_TRIAL_DPS=90,APP_VM_CALIBRATION_ONLY_ENABLE=0,APP_VM_ONLY_ENABLE=0,APP_WHEEL_ENABLE_HOLD_ENABLE=0,APP_LEFT40_DRIFT_TRIM_ENABLE=1,APP_BASELINE40_ENABLE=1,VEHICLE_INTEGRATED_ENABLE=0,VEHICLE_STAGE1_ENABLE=0,APP_FUSION_CAPTURE_STATIC_TEST=0,APP_FUSION_CAPTURE_ENABLE=1'
text=P.read_text('utf-8').replace('.\\',str(P.parent)+'\\')
for tag in ['OutputDirectory','ListingPath']:
    text=re.sub('<'+tag+'>.*?</'+tag+'>',lambda m:'<'+tag+'>'+name+'\\</'+tag+'>',text)
text=text.replace('<Define>','<Define>'+defines+',')
project=T/(name+'.uvprojx');project.write_text(text,'utf-8')
command=['D:/Keil_v5/UV4/UV4.exe','-r',str(project),'-t','Vehicle_Integrated','-o',str(E/('build_off.log' if '--nav-off' in sys.argv else 'build.log'))]
assert '-f' not in command
receipt={'started_at':datetime.datetime.now().astimezone().isoformat(),'command':command,'hardware_accessed':False,'candidate_project':str(project),'target':'Vehicle_Integrated','defines':defines,'source_checkpoint':'30750bd9'}
(E/('build_off_receipt.json' if '--nav-off' in sys.argv else 'build_receipt.json')).write_text(json.dumps(receipt,indent=2),'utf-8')
cp=subprocess.run(command,timeout=300,creationflags=subprocess.CREATE_NO_WINDOW)
log=(E/('build_off.log' if '--nav-off' in sys.argv else 'build.log')).read_bytes().decode('cp936',errors='replace')
receipt.update(finished_at=datetime.datetime.now().astimezone().isoformat(),returncode=cp.returncode,success=cp.returncode==0 and '0 Error(s), 0 Warning(s)' in log)
image=T/name/'Vehicle_Integrated.axf'
if image.exists():receipt.update(axf=str(image),axf_sha256=hashlib.sha256(image.read_bytes()).hexdigest())
(E/('build_off_receipt.json' if '--nav-off' in sys.argv else 'build_receipt.json')).write_text(json.dumps(receipt,indent=2),'utf-8')
assert receipt['success'],log[-3000:]
print(json.dumps(receipt))
