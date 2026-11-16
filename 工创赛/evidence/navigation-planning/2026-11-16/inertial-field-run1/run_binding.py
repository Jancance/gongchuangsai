from pathlib import Path
import datetime,hashlib,json
E=Path(__file__).resolve().parent
R=E.parents[4]
PREP=E.parent/'inertial-field-run1-prep'
S=E.parents[1]/'2026-11-15/fusion-b-capture-prep'
EXPECTED='d7d66e1dd78fcfc70dceb7d4dc7569c987a87be5687abcf69f42f9b8d9a5d609'
IMAGE=R/'工创赛/tmp/inertial-field-run1-build-20261116/motion_static0_integrated/Vehicle_Integrated.axf'
CLIENT=R/'工创赛/evidence/navigation-planning/2026-11-15/baseline40-field/fusion-capture-run1/p1_serial_motion_powered_done.py'
def sha(p):return hashlib.sha256(p.read_bytes()).hexdigest()
def validate(stage):
    assert sha(IMAGE)==EXPECTED
    for p,h in json.loads((PREP/'input_sha256.json').read_text('utf8')).items():assert sha(R/p)==h,p
    assert sha(CLIENT)=='c8d775e8d91904bb9767a16c44e7b6fae33e3abb6453184f753252d999fbfcb1'
    b=json.loads((E/'device_binding.json').read_text('utf8'))
    key='wireless_port' if stage in ('monitor','postdownload_monitor','run') else 'probe_uid'
    assert b.get(key), 'Actual device binding is unconfirmed'
    go=json.loads((E/(stage+'_go.json')).read_text('utf8'))
    assert go['stage']==stage and go['go'] is True and go['candidate_sha256']==EXPECTED
    assert go[key]==b[key] and go['issued_by']=='root' and go['automatic_retry'] is False
    if stage=='run':assert go['site_ready'] is True and go['starts_authorized']==1
    return b
def start(stage,command=None):
    with (E/(stage+'_once_started.json')).open('x',encoding='utf8') as f:
        json.dump(dict(started_at=datetime.datetime.now().astimezone().isoformat(),candidate_sha256=EXPECTED,stage=stage,command=command,attempts=1,automatic_retry=False),f,indent=2)
