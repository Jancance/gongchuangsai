from pathlib import Path
import hashlib,json
R=Path('D:/Desktop/工创赛');E=R/'工创赛/evidence/navigation-planning/2026-11-17/left-nav-residual-prep/medium';base=R/'工创赛/evidence/navigation-planning/2026-11-17'
items=[];manifest={}
for group,stem in [('train','localization-left210-field-run'),('heldout','localization-left150-crossdistance-run')]:
 for run in range(1,4):
  p=base/(stem+str(run));rec=dict(group=group,run=run)
  for key,name in [('measurement','operator_measurement.json'),('prediction','holdout_prediction.json'),('trace','holdout_decomposition.csv')]:
   f=p/name;assert f.is_file();rec[key]=str(f);manifest[str(f.relative_to(R))]=hashlib.sha256(f.read_bytes()).hexdigest()
  items.append(rec)
for name in ['app_odometry.c','app_odometry.h','app_icm_observer.c','app_icm_observer.h','app_fusion_capture.c','chassis_auto_test.c']:
 f=R/'3.stm32f407工程/CHASSIS/USER'/name;manifest[str(f.relative_to(R))]=hashlib.sha256(f.read_bytes()).hexdigest()
(E/'inputs.json').write_text(json.dumps(dict(records=items,fit_definition='mean_i((groundF_i-modelF_i)/modelL_i)',training_count=3,heldout_refit=False,lambda_sensitivity=[0,.5,1],hardware_accessed=False),indent=2),'utf8')
(E/'input_sha256_before.json').write_text(json.dumps(manifest,indent=2),'utf8');print('Prepared18 evidence inputs and6 current source hashes; no product edits')