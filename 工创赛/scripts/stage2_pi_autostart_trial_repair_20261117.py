"""Explicit reviewed Stage2 test autostart selection and current trial restore."""
from pathlib import Path
import ast,base64,json,sys
from stage23_pi_install_20261116 import remote,ROOT
E=ROOT/'evidence/integration/2026-11-17/stage2-pi-preflight'
folder=ROOT/'output/stage2_debug215_pi_trial_20261117'
if '--repair' not in sys.argv:
 print(json.dumps({'local_only':True,'hardware_access':False,'explicit_repair_required':True}));raise SystemExit(0)
tree=ast.parse((ROOT/'scripts/stage2_pi_trial_switch_20261117.py').read_text(encoding='utf-8'))
switch=next(ast.literal_eval(n.value) for n in tree.body if isinstance(n,ast.Assign) and any(isinstance(x,ast.Name) and x.id=='SCRIPT' for x in n.targets))
install=r'''
from pathlib import Path
import json,datetime,hashlib,shutil,base64
root=Path('/home/orangepi/vision_tools/stage23_desktop_20261116');desktop=Path('/home/orangepi/.config/autostart/source-rimpark.desktop')
config=json.loads((root/'stage3_candidate/config/stage3.json').read_text());assert config['calibration_verified'] is False
stage3sha=hashlib.sha256((root/'stage3_candidate/config/stage3.json').read_bytes()).hexdigest();assert stage3sha=='24cd70aa7da70c58eb53c063f1c5d1432f27e8a002a6d3b0eb3c8eb82c2760e8'
backup=Path('/home/orangepi/.config/stage23-autostart-backups')/('stage2-trial-'+datetime.datetime.now().strftime('%Y%m%d-%H%M%S'));backup.mkdir(parents=True)
before=desktop.read_bytes();shutil.copy2(desktop,backup/'source-rimpark.desktop')
for n in ['start.sh','start-trial.sh','start-stage2-debug215-trial.sh']:
 if (root/n).is_file():shutil.copy2(root/n,backup/n)
oldHashes={n:hashlib.sha256((root/n).read_bytes()).hexdigest() for n in ['start.sh','start-trial.sh']}
wrapper=base64.b64decode(WRAPPER64);desired=base64.b64decode(DESKTOP64)
(root/'start-stage2-debug215-trial.sh').write_bytes(wrapper);(root/'start-stage2-debug215-trial.sh').chmod(0o755)
desktop.write_bytes(desired)
assert all(hashlib.sha256((root/n).read_bytes()).hexdigest()==h for n,h in oldHashes.items())
print('AUTOSTART_JSON='+json.dumps({'backup':str(backup),'before':before.decode(),'after':desired.decode(),'wrapper':wrapper.decode(),'unchanged_wrappers':oldHashes,'stage3_config_sha256':stage3sha,'stage3_calibration_verified':config['calibration_verified'],'selection':'current user-authorized Stage2 debug215 trial; not a permanent region filter'},ensure_ascii=False))
'''
install=install.replace('WRAPPER64',repr(base64.b64encode((folder/'start-stage2-debug215-trial.sh').read_bytes()).decode())).replace('DESKTOP64',repr(base64.b64encode((folder/'source-rimpark.desktop').read_bytes()).decode()))
switch=switch.replace("str(root/'start-trial.sh')","str(root/'start-stage2-debug215-trial.sh')")
result=remote(ROOT/'tmp/integration22_ssh.py',install+'\n'+switch)
(E/'autostart-repair-output.txt').write_text(result,encoding='utf-8')
records={}
for label in ['AUTOSTART_JSON','TRIAL_JSON']:
 lines=[x.split(label+'=',1)[1] for x in result.splitlines() if x.startswith(label+'=')]
 if lines:records[label]=json.loads(lines[-1])
if 'TRIAL_JSON' not in records:raise RuntimeError('Repair output saved; no verified trial result; do not blindly retry')
image=records['TRIAL_JSON'].pop('desktop_png_base64',None)
if image:(E/'desktop-autostart-trial.png').write_bytes(base64.b64decode(image))
(E/'autostart-repair.json').write_text(json.dumps(records,ensure_ascii=False,indent=2),encoding='utf-8')
r=records['TRIAL_JSON'];print(json.dumps({'autostart':records.get('AUTOSTART_JSON'),'switched_local':r['switched_local'],'after':r['after'],'owners':r['owners'],'config':r['config'],'session':r['session']},ensure_ascii=False))
