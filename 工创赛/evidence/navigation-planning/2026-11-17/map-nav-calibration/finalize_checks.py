from pathlib import Path
import hashlib,json,datetime,subprocess
R=Path.cwd();E=Path(__file__).resolve().parent;U=R/'3.stm32f407工程/CHASSIS/USER'
prep=json.loads((E/'preparation.json').read_text('utf8'))
manifest=json.loads((E.parent/'turn90-90-prep/original_dirty_sha256.json').read_text('utf8'))
result={'time':datetime.datetime.now().astimezone().isoformat(),'hardware_accessed':False,'format_checks':{},'protected_dirty_unchanged':{}}
for n in prep['product_before']:
 b=(U/n).read_bytes();old=(E/'before'/n).read_bytes()
 result['format_checks'][n]={'sha256':hashlib.sha256(b).hexdigest(),'crlf_only':b.count(b'\n')==b.count(b'\r\n'),'non_ascii_unchanged':bytes(x for x in b if x>=128)==bytes(x for x in old if x>=128)}
for n,h in manifest.items():result['protected_dirty_unchanged'][n]=hashlib.sha256((R/n).read_bytes()).hexdigest()==h
cp=subprocess.run(['git','-c','core.whitespace=cr-at-eol','diff','--check','--','3.stm32f407工程/CHASSIS/USER/app_config.h','3.stm32f407工程/CHASSIS/USER/app_odometry.c','3.stm32f407工程/CHASSIS/USER/app_odometry.h','3.stm32f407工程/CHASSIS/USER/app_motion_runtime.c','3.stm32f407工程/CHASSIS/USER/route_heading_reference.h'],capture_output=True,text=True)
result['git_diff_check']={'returncode':cp.returncode,'output':cp.stdout+cp.stderr}
(E/'final_checks.json').write_text(json.dumps(result,ensure_ascii=False,indent=2),'utf8')
assert all(result['protected_dirty_unchanged'].values())
assert all(v['crlf_only'] and v['non_ascii_unchanged'] for v in result['format_checks'].values())
assert cp.returncode==0
print(json.dumps(result,ensure_ascii=False,indent=2))
