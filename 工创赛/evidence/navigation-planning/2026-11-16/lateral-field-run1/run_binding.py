from pathlib import Path
import datetime, hashlib, json
E=Path(__file__).resolve().parent
R=E.parents[4]
PREP=E.parent/'lateral-field-run1-prep'
S=E
CLIENT=E/'p1_serial_motion_powered_done.py'
def sha(p):return hashlib.sha256(p.read_bytes()).hexdigest()
def candidate():
    c=json.loads((E/'candidate_binding.json').read_text('utf8'))
    assert c['plan_id']==9 and c['rpm_budget']==102 and c['action_kind']==5 and c['target_mm']==2100 and c['reference_mm_s']==400
    assert sha(Path(c['image']))==c['candidate_sha256']
    assert sha(CLIENT)==c['client_sha256']
    for path,digest in c['input_sha256'].items():assert sha(R/path)==digest,path
    return c
# Candidate is intentionally absent until construction and independent review.
_c=json.loads((E/'candidate_binding.json').read_text('utf8'))
EXPECTED=_c['candidate_sha256']
IMAGE=Path(_c['image'])
def validate(stage):
    c=candidate();b=json.loads((E/'device_binding.json').read_text('utf8'))
    key='wireless_port' if stage in ('monitor','postdownload_monitor','run') else 'probe_uid'
    assert b.get(key)
    go=json.loads((E/(stage+'_go.json')).read_text('utf8'))
    assert go['stage']==stage and go['go'] is True and go['candidate_sha256']==EXPECTED
    assert go['candidate_binding_sha256']==sha(E/'candidate_binding.json')
    assert go[key]==b[key] and go['issued_by']=='root' and go['automatic_retry'] is False
    assert go['wrapper_sha256']==sha(E/(stage+'_once.py'))
    assert go['binding_script_sha256']==sha(E/'run_binding.py')
    if stage=='run':assert go['site_ready'] is True and go['starts_authorized']==1
    return b

def start(stage,command=None):
    with (E/(stage+'_once_started.json')).open('x',encoding='utf8') as f:
        json.dump(dict(started_at=datetime.datetime.now().astimezone().isoformat(),candidate_sha256=EXPECTED,stage=stage,command=command,attempts=1,automatic_retry=False),f,indent=2)
