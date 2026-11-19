"""Default local; main alone may --deploy after fresh safe MCU proof. No MCU commands."""
from pathlib import Path
import argparse,base64,zlib,json,hashlib,datetime,time,subprocess,sys,shlex
from prepare_pickup60_ui_20261118 import R,P,E
from stage23_pi_install_20261116 import remote
SCRIPT=r'''
from pathlib import Path
import os,time,json,hashlib,stat,signal,subprocess,datetime
root=Path('/home/orangepi/vision_tools/stage23_desktop_20261116')
c=CANDIDATE
payload=PAYLOAD
audit=AUDIT
def sha(p):return hashlib.sha256(Path(p).read_bytes()).hexdigest()
def fsync_file(p):
 with Path(p).open('rb') as f:os.fsync(f.fileno())
def fsync_dir(p):
 fd=os.open(str(p),os.O_RDONLY|os.O_DIRECTORY)
 try:os.fsync(fd)
 finally:os.close(fd)
def atomic(p,data,mode):
 temp=p.with_name('.'+p.name+'.pickup60-new');assert not temp.exists()
 with temp.open('xb') as f:f.write(data);f.flush();os.fsync(f.fileno())
 temp.chmod(mode);os.replace(temp,p);fsync_file(p);fsync_dir(p.parent)
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
   devices=[]
   for fd in (p/'fd').iterdir():
    try:
     v=os.readlink(fd)
     if v.startswith(('/dev/video','/dev/ttyUSB','/dev/ttyACM')):devices.append(v)
    except OSError:pass
   if devices:
    ppid=int(next(x.split()[1] for x in (p/'status').read_text().splitlines() if x.startswith('PPid:')))
    rows.append(dict(pid=int(p.name),ppid=ppid,devices=sorted(set(devices))))
  except (OSError,StopIteration):pass
 return rows
def assert_owners(main):
 rows=owners();assert all(x['pid']==main or x['ppid']==main for x in rows),rows
 assert all(sum(d in x['devices'] for x in rows)==1 for d in ('/dev/video0','/dev/ttyUSB0')),rows
 return rows
def last_observation(proc):
 a=proc['argv'];v=Path(a[a.index('--session-dir')+1]);v=v if v.is_absolute() else Path(proc['cwd'])/v
 assert root.resolve() in v.resolve().parents
 path=v/'observations.jsonl'
 with path.open('rb') as f:f.seek(max(0,path.stat().st_size-262144));lines=f.read().splitlines()
 for line in reversed(lines):
  try:return json.loads(line),v
  except ValueError:pass
 raise AssertionError('No complete current observation')
current=procs();assert len(current)==1,current;before=current[0]
assert before['cwd']==str(root) and '--preview-only' not in before['argv'] and '--continuous-pick' in before['argv'],before
owned=assert_owners(before['pid']);observation,oldsession=last_observation(before)
assert observation.get('mcu_status',{}).get('state') in (0,8) and not observation['mcu_status'].get('scheduled'),observation.get('mcu_status')
assert observation.get('pickup_window',{}).get('reason')!='status_inactive','MCU status expired despite fresh camera frame'
assert time.monotonic()-observation['host_read_completed_monotonic_s']<5,'Stale camera/session proof'
for n,h in c['pi_base_hashes'].items():assert sha(root/n)==h,('base drift',n)
for n,h in c['pi_unchanged_dependencies'].items():assert sha(root/n)==h,('dependency drift',n)
for n,data in payload.items():
 raw=__import__('base64').b64decode(data);assert hashlib.sha256(raw).hexdigest()==c['pi_changes'][n];compile(raw,str(root/n),'exec')
config_paths={p for p in root.glob('*.json') if p.name!='manifest.json'}
config_paths.update(Path(p) for p in audit['configs'])
entry_paths={p for p in root.glob('start*.sh') if p.is_file()};entry_paths.update(Path(p) for p in audit['launch_entries'])
preserved={str(p):sha(p) for p in sorted(config_paths|entry_paths) if p.is_file()}
for p in audit['configs']:
 if Path(p).name in ('body-profile.json','material_pickup_top65_20261114.json','material_color_thresholds.json'):assert preserved[p]==audit['configs'][p]['sha256'],('critical current config drift',p)
manifest_path=root/'manifest.json';manifest=json.loads(manifest_path.read_text())
backup=root.parent/('pickup60-ui-backup-'+datetime.datetime.now().strftime('%Y%m%d-%H%M%S'));backup.mkdir()
for n in c['pi_changes']:(backup/n).write_bytes((root/n).read_bytes());fsync_file(backup/n)
(backup/'manifest.json').write_bytes(manifest_path.read_bytes());fsync_file(backup/'manifest.json')
(backup/'preserved-sha.json').write_text(json.dumps(preserved,indent=2)+'\n');fsync_file(backup/'preserved-sha.json');fsync_dir(backup);fsync_dir(backup.parent)
os.sync()
# Existing app alone receives TERM; no UART control or other hardware owner.
os.kill(before['pid'],signal.SIGTERM);deadline=time.monotonic()+15
while time.monotonic()<deadline and (procs() or owners()):time.sleep(.2)
assert not procs() and not owners(),'Owner did not release; no replacement/launch attempted'
for n,data in payload.items():
 p=root/n;mode=stat.S_IMODE(p.stat().st_mode);atomic(p,__import__('base64').b64decode(data),mode)
updated=dict(manifest);updated.update(c['pi_changes'])
atomic(manifest_path,(json.dumps(updated,indent=2)+'\n').encode(),stat.S_IMODE(manifest_path.stat().st_mode))
assert all(updated[n]==h for n,h in manifest.items() if n not in c['pi_changes'])
assert all(sha(Path(p))==h for p,h in preserved.items()),'Preserved file drift'
# Persist every effective config and launch entry too, without rewriting bytes.
for p in preserved:fsync_file(Path(p))
for p in {Path(p).parent for p in preserved}|{root,root.parent}:fsync_dir(p)
os.sync()
assert all(sha(root/n)==h for n,h in c['pi_changes'].items())
assert all(sha(Path(p))==h for p,h in preserved.items())
subprocess.Popen(['bash',str(root/'start-stage2-debug215-trial.sh')],cwd=root,stdin=subprocess.DEVNULL,stdout=subprocess.DEVNULL,stderr=subprocess.DEVNULL,start_new_session=True)
after=config=None;deadline=time.monotonic()+20
while time.monotonic()<deadline:
 rows=procs()
 if len(rows)==1 and rows[0]['pid']!=before['pid']:
  row=rows[0]
  try:
   obs,session=last_observation(row);log=Path(str(session)+'.log')
   for line in log.read_text(errors='replace').splitlines()[:20]:
    if line.startswith('CONFIG '):config=json.loads(line[7:])
   status=obs.get('mcu_status') or {}
   actual_rx=status.get('state') in (0,8) and not status.get('scheduled') and obs.get('pickup_window',{}).get('reason')!='status_inactive' and time.monotonic()-obs['host_read_completed_monotonic_s']<5
   if actual_rx and config and config.get('source_first_rotation_required_deg')==c['first_pick_rotation_deg'] and config.get('pickup_required_frames')==3 and config.get('pickup_ui_policy')=='actual_window_trigger_v1' and config.get('observation_tx_enabled') and config.get('boot_handshake_tx_enabled') and config.get('mcu_status_rx_enabled'):
    assert row['cwd']==str(root) and '--preview-only' not in row['argv'];assert_owners(row['pid']);after=row;break
  except (OSError,ValueError,KeyError,AssertionError):pass
 time.sleep(.2)
assert after is not None,'New owner/CONFIG not verified; inspect state before retry'
assert all(sha(Path(p))==h for p,h in preserved.items())
assert all(sha(root/n)==h for n,h in c['pi_changes'].items())
print('PICKUP60_DEPLOY_JSON='+json.dumps(dict(before=before,after=after,backup=str(backup),config=config,session=str(session),new_actual_mcu_status=status,new_actual_mcu_rx_verified=True,owners=owners(),changed_sha256={n:sha(root/n) for n in c['pi_changes']},manifest_updated_only_keys=list(c['pi_changes']),preserved_sha256=preserved,file_and_parent_fsync=True,os_sync=True,serial_start_stop_sent=False,MCU_reset=False,Pi_reboot=False,cold_boot_tested=False,completed_local=datetime.datetime.now().isoformat())))
'''
def prepare_command(c,audit,payload):
 # Transmit only fields actually read remotely. Full release/MCU proof stays local.
 rc={k:c[k] for k in ('pi_base_hashes','pi_unchanged_dependencies','pi_changes','first_pick_rotation_deg')}
 ra=dict(configs={p:dict(sha256=v['sha256']) for p,v in audit['configs'].items()},launch_entries=list(audit['launch_entries']))
 script=SCRIPT.replace('c=CANDIDATE','c='+repr(rc),1).replace('payload=PAYLOAD','payload='+repr(payload),1).replace('audit=AUDIT','audit='+repr(ra),1)
 packed=zlib.compress(script.encode(),9);encoded=base64.b64encode(packed).decode()
 bootstrap="import zlib,base64;exec(zlib.decompress(base64.b64decode("+repr(encoded)+")))"
 ssh_command='python3 -c '+shlex.quote(bootstrap)
 argv=[sys.executable,str(R/'tmp/integration_mirror_ssh_20261117.py'),'command',ssh_command]
 windows_commandline=subprocess.list2cmdline(argv);units=len(windows_commandline.encode('utf-16-le'))//2
 assert units<28000,('Windows transport too long; SSH not started',units)
 unpacked=zlib.decompress(base64.b64decode(encoded)).decode();assert unpacked==script
 # Check actual decompressed top-level literal assignments, not only input maps.
 import ast
 tree=ast.parse(unpacked);values={n.targets[0].id:ast.literal_eval(n.value) for n in tree.body if isinstance(n,ast.Assign) and len(n.targets)==1 and isinstance(n.targets[0],ast.Name) and n.targets[0].id in ('c','audit','payload')}
 needed={name:{n.slice.value for n in ast.walk(tree) if isinstance(n,ast.Subscript) and isinstance(n.value,ast.Name) and n.value.id==name and isinstance(n.slice,ast.Constant) and isinstance(n.slice.value,str)} for name in ('c','audit')}
 assert needed['c']<=set(rc) and needed['audit']<=set(ra),('Remote field omitted',needed)
 assert values['c']==rc and values['payload']==payload and values['audit']==ra
 assert set(ra['configs'])==set(audit['configs']) and set(ra['launch_entries'])==set(audit['launch_entries'])
 report=dict(windows_commandline_characters=len(windows_commandline),windows_utf16_code_units=units,limit=28000,ssh_started=False,remote_literal_fields_equivalent=True,all_config_and_launcher_paths_retained=True,candidate_fields=list(rc),audit_fields=list(ra),required_fields={k:sorted(v) for k,v in needed.items()},uncompressed_bytes=len(script.encode()),compressed_bytes=len(packed))
 return bootstrap,report

def main():
 ap=argparse.ArgumentParser(description=__doc__);ap.add_argument('--deploy',action='store_true');ap.add_argument('--mcu-proof',type=Path);a=ap.parse_args()
 c=json.loads((E/'candidate.json').read_text())
 audit=json.loads((R/'evidence/integration/2026-11-18/pi-persistence/audit-20261118-040246.json').read_text())
 payload={n:base64.b64encode((P/n).read_bytes()).decode() for n in c['pi_changes']};assert all(hashlib.sha256((P/n).read_bytes()).hexdigest()==h for n,h in c['pi_changes'].items())
 command,transport=prepare_command(c,audit,payload);(E/'transport-check.json').write_text(json.dumps(transport,indent=2)+'\n',encoding='utf-8')
 if not a.deploy:print(json.dumps(dict(local_only=True,ssh_executed=False,files=list(c['pi_changes']),transport=transport)));return
 assert c['review_passed'];assert a.mcu_proof and time.time()-a.mcu_proof.stat().st_mtime<180,'Provide fresh read-only MCU proof <=180s'
 proof=json.loads(a.mcu_proof.read_text());s=proof.get('snapshot',proof.get('final',{}))
 assert proof.get('image_sha256')==c['paired_firmware']['sha256'] and proof.get('verified_flash_bytes')==197368
 assert s.get('g_source_cycle_state') in (0,8)
 for k in ('g_source_cycle_fault','g_dm_jog_first_fault','g_low_fault','g_source_cycle_request','g_low_request','g_vehicle_arm_stop_pending','g_source_parallel_stop_mask'):assert s.get(k)==0,('unsafe or absent MCU proof',k)
 out=remote(R/'tmp/integration_mirror_ssh_20261117.py',command)
 (E/'pi-deploy-output.txt').write_text(out,encoding='utf-8');rows=[x.split('PICKUP60_DEPLOY_JSON=',1)[1] for x in out.splitlines() if x.startswith('PICKUP60_DEPLOY_JSON=')];assert len(rows)==1
 report=json.loads(rows[0]);(E/'pi-deploy.json').write_text(json.dumps(report,indent=2)+'\n',encoding='utf-8');print(json.dumps(report,ensure_ascii=False))
if __name__=='__main__':main()
