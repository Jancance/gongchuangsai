"""Update the existing third-stage viewer at stopped state7 using verified files."""
import argparse
import base64
from datetime import datetime
import hashlib
import json
from pathlib import Path
import time
import zlib
from deploy_stage3_row_bridge_20261118 import SCRIPT, E, ROOT, P
from stage23_pi_install_20261116 import remote


def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--deploy',action='store_true')
    parser.add_argument('--mcu-proof',type=Path)
    parser.add_argument('--pi-proof',type=Path)
    args=parser.parse_args()
    files={'region_desktop.py':P/'region_desktop.py',
           'ring_row_alignment.py':P/'ring_row_alignment.py',
           'stage3_candidate/config/stage3.json':E/'vehicle-edge-live-config.json'}
    payload={name:base64.b64encode(path.read_bytes()).decode() for name,path in files.items()}
    if not args.deploy:
        print(json.dumps({'local_only':True,'sha256':{n:hashlib.sha256(p.read_bytes()).hexdigest() for n,p in files.items()}}));return
    assert args.mcu_proof and args.pi_proof
    assert all(time.time()-p.stat().st_mtime<180 for p in (args.mcu_proof,args.pi_proof))
    m=json.loads(args.mcu_proof.read_text(encoding='utf-8'))
    pi=json.loads(args.pi_proof.read_text(encoding='utf-8'))
    assert m['image']['sha256']=='492ed1a0c4504e9e31a699c042bd4cb7b1e9bafb327bc90946c666a76441b620'
    assert m['verified_flash_bytes']==198308 and m['first_error'] is None
    s=m['after']
    assert s['g_stage3_state']==7 and not s['g_stage3_config_ready'] and not s['g_stage3_manual_state']
    assert s['g_low_wheel_hold_ready'] and s['wheel_hold_results']==[1,1,1,1]
    for n in ('g_stage3_fault','g_it_request','g_it_cause','g_it_rejected','g_safety_fault','g_rtos_fault','g_can_ctrl_fault_latched'):assert s[n]==0,n
    assert len(pi['apps'])==1 and not pi['manifest_mismatches']
    app=pi['apps'][0];status=app['latest']['mcu_status']
    assert status['state']==7 and status['align_done']==1 and not status['config_ready']
    assert status['pose']==s['g_stage3_pose'] and status['run']==s['g_stage3_run']
    expected={'region_desktop.py':'b450f65eaf82a0dcc99b518f1f8c38cd399cc489c8c4d30635e4c804c8cdf240',
              'ring_row_alignment.py':'d185eb3c13fe8d3b7e5699f35d1220d38b092b91da7d5240b731df0e19cc0b11',
              'stage3_candidate/config/stage3.json':pi['stage3_config_sha256']}
    prior=[json.loads(p.read_text(encoding='utf-8')) for p in sorted(E.glob('vehicle-edge-deploy-*.json'))]
    confirmed=[r for r in prior if r.get('status')=='DEPLOYED']
    if confirmed:
        expected.update(confirmed[-1]['changed_sha256'])
    binding={'pid':app['pid'],'session':'captures/'+app['session'].split('/')[-1],
             'run':status['run'],'pose':status['pose']}
    script=SCRIPT.replace("s['state']==2 and s['config_ready']==0 and s['align_done']==0", "s['state']==7 and s['config_ready']==0 and s['align_done']==1",1)
    script=script.replace(" if 'ring_row_alignment.py' in payload:assert not (root/'ring_row_alignment.py').exists()",'')
    script=script.replace('payload=PAYLOAD','payload='+repr(payload),1).replace('expected=EXPECTED','expected='+repr(expected),1).replace('binding=BINDING','binding='+repr(binding),1)
    encoded=base64.b64encode(zlib.compress(script.encode(),9)).decode()
    assert len(encoded)<20000
    output=remote(ROOT/'tmp/integration_mirror_ssh_20261117.py',"import base64,zlib;exec(zlib.decompress(base64.b64decode("+repr(encoded)+")))")
    stamp=datetime.now().strftime('%Y%m%d-%H%M%S')
    (E/('vehicle-edge-deploy-'+stamp+'.txt')).write_text(output,encoding='utf-8')
    rows=[x.split('ROW_DEPLOY_JSON=',1)[1] for x in output.splitlines() if x.startswith('ROW_DEPLOY_JSON=')]
    assert len(rows)==1
    result=json.loads(rows[0]);(E/('vehicle-edge-deploy-'+stamp+'.json')).write_text(json.dumps(result,indent=2),encoding='utf-8')
    print(json.dumps(result));assert result['status']=='DEPLOYED'

if __name__=='__main__':main()
