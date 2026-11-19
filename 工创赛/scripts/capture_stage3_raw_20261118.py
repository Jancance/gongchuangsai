"""Local by default. Explicit exact-bound normal viewer stop/restart, no MCU commands."""
import argparse, base64, datetime, hashlib, json, re, zlib
from pathlib import Path
from stage23_pi_install_20261116 import ROOT, remote

SCRIPT = r'''
from pathlib import Path
import base64,hashlib,json,os,signal,struct,subprocess,time
root=Path('/home/orangepi/vision_tools/stage23_desktop_20261116')
binding=BINDING
result=dict(binding=binding,term_sent=False,restart_attempted=False,software_or_config_replaced=False,serial_start_stop_sent=False,mcu_reset=False,same_stopped_pose_pair=True)
def sha(p):return hashlib.sha256(p.read_bytes()).hexdigest()
def procs():
 rows=[]
 for p in Path('/proc').iterdir():
  if not p.name.isdigit():continue
  try:
   a=[x.decode() for x in (p/'cmdline').read_bytes().split(b'\0') if x]
   if any(Path(x).name=='source_fast_cycle_view_20261115.py' for x in a):rows.append(dict(pid=int(p.name),argv=a,cwd=os.readlink(p/'cwd')))
  except OSError:pass
 return rows
def owners():
 rows=[]
 for p in Path('/proc').iterdir():
  if not p.name.isdigit():continue
  try:
   parent=int(next(x.split()[1] for x in (p/'status').read_text().splitlines() if x.startswith('PPid:')))
   for f in (p/'fd').iterdir():
    try:
     d=os.readlink(f)
     if d.startswith(('/dev/video','/dev/ttyUSB','/dev/ttyACM')):rows.append(dict(pid=int(p.name),ppid=parent,device=d))
    except OSError:pass
  except OSError:pass
 return [dict(t) for t in {tuple(sorted(x.items())) for x in rows}]
def bound(p,session=None):
 assert p['cwd']==str(root) and '--preview-only' not in p['argv'],p
 a=p['argv'];s=a[a.index('--session-dir')+1]
 assert not Path(s).is_absolute() and '..' not in Path(s).parts and s.startswith('captures/run-'),s
 if session is not None:assert s==session,(s,session)
 return root/s
def check(o):
 m=o['mcu_status'];assert o['region']=='stage3' and m['state']==binding['state'] and m['pose']==binding['pose'] and m['config_ready']==0,o
 if binding['state']==7:assert m['align_done']==1,o
 assert o['calibration_verified'] is False and not o['replay'],o
 return o
def latest(s):return json.loads((s/'observations.jsonl').read_text().splitlines()[-1])
def owncheck(p):
 rows=owners();assert len(rows)==2 and {x['device'] for x in rows}=={'/dev/video0','/dev/ttyUSB0'},rows
 assert all(x['pid']==p['pid'] or x['ppid']==p['pid'] for x in rows),rows
 assert next(x for x in rows if x['device']=='/dev/ttyUSB0')['pid']==p['pid'],rows
 return rows
def image(p):
 b=p.read_bytes();assert b[:8]==b'\x89PNG\r\n\x1a\n' and struct.unpack('>II',b[16:24])==(1920,1080)
 return dict(source=str(p),sha256=hashlib.sha256(b).hexdigest(),bytes=len(b),size=[1920,1080],base64=base64.b64encode(b).decode())
try:
 manifest=json.loads((root/'manifest.json').read_text());manifestHash=sha(root/'manifest.json')
 assert manifest and all(not Path(n).is_absolute() and '..' not in Path(n).parts and sha(root/n)==h for n,h in manifest.items()),'Manifest mismatch'
 preserved={str(p):sha(p) for p in root.glob('*.json')}
 external=Path('/home/orangepi/vision_tools/material_pickup_top65_20261114.json');preserved[str(external)]=sha(external)
 rows=procs();assert len(rows)==1 and rows[0]['pid']==binding['pid'],rows
 p=rows[0];s=bound(p,binding['session']);owncheck(p)
 seq=latest(s)['frame_sequence'];deadline=time.monotonic()+3
 while time.monotonic()<deadline:
  o=latest(s)
  if o['frame_sequence']>seq:break
  time.sleep(.1)
 assert o['frame_sequence']>seq,'No fresh advancing observation'
 result['before']=check(o);assert procs()==[p],'Exact process changed'
 result['term_sent']=True;os.kill(p['pid'],signal.SIGTERM)
 deadline=time.monotonic()+12
 while time.monotonic()<deadline and (procs() or owners()):time.sleep(.2)
 assert not procs() and not owners(),'Owners remain; no restart'
 saved=json.loads((s/'session.json').read_text());result['stopped_last']=check(saved['last']);result['saved_frames']=saved['frames']
 assert not saved.get('error') and 'Saved:' in s.with_suffix('.log').read_text(),'Normal Saved completion missing'
 result['images']={'before':image(s/'last-raw.png')}
 result['restart_attempted']=True
 subprocess.Popen(['bash',str(root/'start-stage2-debug215-trial.sh')],cwd=root,stdin=subprocess.DEVNULL,stdout=subprocess.DEVNULL,stderr=subprocess.DEVNULL,start_new_session=True)
 deadline=time.monotonic()+12;new=None
 while time.monotonic()<deadline:
  rows=procs()
  if len(rows)==1 and rows[0]['pid']!=binding['pid']:
   candidate=bound(rows[0])
   if candidate!=s and (candidate/'first-raw.png').is_file() and (candidate/'observations.jsonl').is_file():new=rows[0];break
  time.sleep(.2)
 assert new is not None,'Restart unconfirmed; inspect without retry'
 ns=bound(new);result['after_observation']=check(latest(ns));result['owners']=owncheck(new)
 first=json.loads((ns/'observations.jsonl').read_text().splitlines()[0]);check(first);assert first['frame_sequence']==1
 result['images']['after']=image(ns/'first-raw.png');result['after']=new;result['new_session']=str(ns)
 assert sha(root/'manifest.json')==manifestHash and all(sha(root/n)==h for n,h in manifest.items())
 assert all(sha(Path(p))==h for p,h in preserved.items()),'Configuration changed'
 result.update(status='CAPTURED_SAME_STOPPED_POSE',manifest_sha256=manifestHash,preserved_hashes=preserved)
except Exception as e:result.update(status='UNCERTAIN_INSPECT_BEFORE_ANY_RETRY',error=repr(e))
print('RAW_CAPTURE_JSON='+json.dumps(result))
'''

def main():
    ap=argparse.ArgumentParser(description=__doc__)
    ap.add_argument('--capture',action='store_true');ap.add_argument('--pid',type=int)
    ap.add_argument('--session');ap.add_argument('--pose',type=int);ap.add_argument('--label')
    ap.add_argument('--state',type=int,choices=(2,7),default=2)
    a=ap.parse_args()
    if not a.capture:
        print(json.dumps(dict(local_only=True,hardware_access=False)));return
    assert a.pid and a.pid>1 and a.pose is not None
    assert a.session and re.fullmatch(r'captures/run-[0-9-]+',a.session)
    assert a.label and re.fullmatch(r'[A-Za-z0-9_-]+',a.label)
    stamp=datetime.datetime.now().strftime('%Y%m%d-%H%M%S-%f')
    out=ROOT/'evidence/integration/2026-11-18/stage3-resume';out.mkdir(parents=True,exist_ok=True)
    path=out/f'{a.label}-capture-{stamp}.json'
    report=dict(status='UNCERTAIN_REMOTE_RESULT_NOT_RECEIVED',binding=vars(a),retry_permitted=False)
    path.write_text(json.dumps(report,indent=2),encoding='utf-8')
    script=SCRIPT.replace('binding=BINDING','binding='+repr(dict(pid=a.pid,session=a.session,pose=a.pose,state=a.state,label=a.label)),1)
    encoded=base64.b64encode(zlib.compress(script.encode(),9)).decode()
    try:
        output=remote(ROOT/'tmp/integration_mirror_ssh_20261117.py',"import zlib,base64;exec(zlib.decompress(base64.b64decode("+repr(encoded)+")))")
        lines=[x.split('RAW_CAPTURE_JSON=',1)[1] for x in output.splitlines() if x.startswith('RAW_CAPTURE_JSON=')]
        assert len(lines)==1,'Remote result missing; do not retry'
        report=json.loads(lines[0])
        for name,item in report.get('images',{}).items():
            data=base64.b64decode(item.pop('base64'));assert hashlib.sha256(data).hexdigest()==item['sha256']
            target=out/f'{a.label}-{name}-{stamp}.png'
            with target.open('xb') as f:f.write(data)
            item['local']=str(target)
    except Exception as exc:report.update(status='UNCERTAIN_INSPECT_BEFORE_ANY_RETRY',local_error_type=type(exc).__name__)
    path.write_text(json.dumps(report,indent=2),encoding='utf-8');print(json.dumps(dict(evidence=str(path),status=report['status'])))

if __name__=='__main__':main()
