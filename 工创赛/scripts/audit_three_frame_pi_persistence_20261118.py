"""Default local-only; --audit reads this application's Pi deployment, never changes it."""
from pathlib import Path
import argparse,base64,datetime,hashlib,json,zlib
from stage23_pi_install_20261116 import remote
R=Path(__file__).resolve().parents[1]
E=R/'evidence/integration/2026-11-18/pi-persistence'

SCRIPT=r'''
from pathlib import Path
import os,json,hashlib,time,subprocess,stat,re
root=Path('/home/orangepi/vision_tools/stage23_desktop_20261116')
expected=EXPECTED
needles=('source_fast_cycle','source_cycle_dashboard','source-rimpark','stage23_desktop','start-stage2','start-trial','vision_tools')
def related(s):return any(x in s for x in needles)
def read(p,limit=16384):
 try:
  with Path(p).open('rb') as f:b=f.read(limit)
  return b.decode('utf-8','replace')
 except OSError:return None
def clean(s):
 if s is None:return None
 return '\n'.join('[sensitive line omitted]' if re.search(r'(password|passwd|secret|credential|api[_-]?key|token)\s*[=:]',v,re.I) else v for v in s.splitlines())
def clean_argv(argv):
 out=[];hide=False
 for x in argv:
  if hide:out.append('[redacted]');hide=False;continue
  if x.startswith('-') and re.search(r'password|passwd|secret|token|api[_-]?key|credential',x,re.I):
   out.append(x.split('=',1)[0]+'=[redacted]' if '=' in x else x);hide='=' not in x
  else:out.append(x)
 return out
def info(p,hashfile=False):
 p=Path(p);out={'path':str(p)}
 try:
  z=p.lstat();out.update(exists=True,symlink=p.is_symlink(),realpath=str(p.resolve()),mode=oct(stat.S_IMODE(z.st_mode)),uid=z.st_uid,gid=z.st_gid,mtime=z.st_mtime,size=z.st_size)
  if p.is_symlink():out['link_target']=os.readlink(p)
  if hashfile and p.is_file():out['sha256']=hashlib.sha256(p.read_bytes()).hexdigest()
 except OSError as e:out.update(exists=False,error=type(e).__name__)
 return out
def command(argv):
 try:
  x=subprocess.run(argv,capture_output=True,text=True,timeout=8)
  return dict(exit_code=x.returncode,stdout=clean(x.stdout[:12000]),stderr=clean(x.stderr[:2000]))
 except (OSError,subprocess.TimeoutExpired) as e:return dict(error=type(e).__name__)
btime=next((float(x.split()[1]) for x in read('/proc/stat',65536).splitlines() if x.startswith('btime ')),0)
hz=os.sysconf('SC_CLK_TCK');allproc={};deviceowners=[]
for p in Path('/proc').iterdir():
 if not p.name.isdigit():continue
 try:
  argv=[x.decode('utf-8','replace') for x in (p/'cmdline').read_bytes().split(b'\0') if x]
  if not argv:continue
  status=read(p/'status',8192) or '';ppid=int(next((x.split()[1] for x in status.splitlines() if x.startswith('PPid:')),'0'))
  st=(p/'stat').read_text();parts=st[st.rfind(')')+2:].split();started=btime+int(parts[19])/hz
  row=dict(pid=int(p.name),ppid=ppid,argv=clean_argv(argv),cwd=os.readlink(p/'cwd'),started_epoch=started)
  allproc[int(p.name)]=row
  devices=[];fds={}
  for fd in (p/'fd').iterdir():
   try:
    dest=os.readlink(fd)
    if dest.startswith(('/dev/video','/dev/ttyUSB','/dev/ttyACM')):devices.append(dest)
    if fd.name in ('1','2','9'):fds[fd.name]=dest
   except OSError:pass
  if devices:deviceowners.append(dict(pid=int(p.name),ppid=ppid,devices=sorted(set(devices))))
  if any(related(a) for a in argv):row['selected_fds']=fds
 except (OSError,ValueError,StopIteration):pass
mains=[x for x in allproc.values() if any(Path(a).name=='source_fast_cycle_view_20261115.py' for a in x['argv'])]
mainids={x['pid'] for x in mains}
procs=[x for x in allproc.values() if any(related(a) for a in x['argv']) or x['ppid'] in mainids]
owner_single=len(mains)==1 and all(x['pid'] in mainids or x['ppid'] in mainids for x in deviceowners)
owner_counts={d:sum(d in x['devices'] for x in deviceowners) for d in ('/dev/video0','/dev/ttyUSB0')}
unique_camera_serial=owner_single and all(n==1 for n in owner_counts.values())
manifest_path=root/'manifest.json'
try:manifest=json.loads(manifest_path.read_text())
except (OSError,ValueError):manifest={}
files={}
for n,h in expected['code_and_dependencies'].items():
 z=info(root/n,True);z.update(expected_sha256=h,matches_expected=z.get('sha256')==h,manifest_entry=manifest.get(n),manifest_matches_file=manifest.get(n)==z.get('sha256'))
 z['newer_than_main_pids']=[x['pid'] for x in mains if z.get('mtime',0)>x['started_epoch']]
 files[n]=z
configs={}
for p in sorted(root.glob('*.json')):
 if p.name!='manifest.json':configs[str(p)]=info(p,True)
for p in [Path('/home/orangepi/vision_tools/material_pickup_top65_20261114.json'),Path('/home/orangepi/p2-source-observer-20261106/vision_app/code/config/material_color_thresholds.json')]:configs[str(p)]=info(p,True)
logs=[]
for proc in mains:
 argv=proc['argv'];session=None
 if '--session-dir' in argv:
  i=argv.index('--session-dir');session=Path(argv[i+1]) if i+1<len(argv) else None
 if session is None:continue
 if not session.is_absolute():session=Path(proc['cwd'])/session
 log=Path(str(session)+'.log');entry=dict(pid=proc['pid'],session_dir=str(session),log=info(log),source='exact running argv --session-dir',config=None,recent_state=[],recent_handshake=[])
 try:
  # Never open FIFOs/devices or arbitrary process files as logs.
  if root.resolve() not in log.resolve().parents or not log.is_file():raise OSError('unexpected log path')
  with log.open('rb') as f:
   first=f.read(65536);f.seek(max(0,log.stat().st_size-196608));tail=f.read(196608)
  for line in first.decode('utf-8','replace').splitlines():
   if line.startswith('CONFIG '):
    try:entry['config']=json.loads(line[7:])
    except ValueError:entry['config_parse_error']=True
  lines=tail.decode('utf-8','replace').splitlines()
  entry['recent_state']=[clean(x[:2400]) for x in lines if x.startswith('MCU_STATUS ')][-3:]
  entry['recent_handshake']=[clean(x[:1600]) for x in lines if x.startswith(('BOOT_READY ','BOOT_HANDSHAKE ','BOOT_LINK '))][-3:]
  entry['log_exists_after_process_start']=log.stat().st_mtime>=proc['started_epoch']
 except OSError as e:entry['error']=type(e).__name__
 logs.append(entry)
launch={}
def launchfile(p,require_match=False,matching_lines_only=False):
 p=Path(p)
 if not p.is_file():return
 text=read(p,24576)
 if text is None or (require_match and not related(text)):return
 if matching_lines_only:text='\n'.join(x for x in text.splitlines() if related(x))
 launch[str(p)]={**info(p,True),'text':clean(text),'matching_lines_only':matching_lines_only}
for n in ('start.sh','start-trial.sh','start-preview.sh','start-stage2-debug215-trial.sh'):launchfile(root/n)
for folder in [Path('/home/orangepi/.config/autostart'),Path('/etc/xdg/autostart'),Path('/home/orangepi/Desktop'),Path('/home/orangepi/桌面')]:
 if folder.is_dir():
  for p in sorted(folder.glob('*.desktop'))[:80]:launchfile(p,True)
for folder in [Path('/home/orangepi/.config/systemd/user'),Path('/etc/systemd/system')]:
 if folder.is_dir():
  for p in sorted(folder.glob('*.service'))[:120]:launchfile(p,True)
sessions=[];folder=Path('/home/orangepi/.cache/sessions')
if folder.is_dir():
 for p in sorted(folder.iterdir())[:80]:
  if not p.is_file():continue
  content=read(p,131072) or '';lines=[clean(x[:2400]) for x in content.splitlines() if related(x)]
  if lines:sessions.append({**info(p),'matching_lines':lines[:20]})
cron=command(['crontab','-l']);cron['stdout']='\n'.join(clean(x) for x in cron.get('stdout','').splitlines() if related(x));cron['stderr']=clean(cron.get('stderr',''))
launchfile('/etc/rc.local',True,True)
units={}
for user in (False,True):
 base=['systemctl']+(['--user'] if user else [])
 listed=command(base+['list-units','--all','--plain','--no-legend','--no-pager'])
 relevant=[x.split()[0] for x in listed.get('stdout','').splitlines() if related(x.lower()) and x.split()]
 for unit in relevant[:12]:
  shown=command(base+['show',unit,'--no-pager','--property=Id,User,WorkingDirectory,ExecStart,FragmentPath,ActiveState,MainPID'])
  if related(shown.get('stdout','')):units[('user:' if user else 'system:')+unit]=shown
mounts={}
for name,path in [('root','/'),('home','/home/orangepi'),('application',str(root))]:mounts[name]=command(['findmnt','-T',path,'-n','-o','TARGET,SOURCE,FSTYPE,OPTIONS'])
fstab=read('/etc/fstab',16384) or ''
fstab_lines=[x for x in fstab.splitlines() if x.strip() and not x.lstrip().startswith('#') and (len(x.split())>1 and x.split()[1] in ('/','/home','/home/orangepi') or 'overlay' in x)]
paths=[info(p) for p in ['/', '/home','/home/orangepi','/home/orangepi/vision_tools',str(root)]]
disk={}
try:
 z=os.statvfs(root);disk=dict(available_bytes=z.f_bavail*z.f_frsize,total_bytes=z.f_blocks*z.f_frsize)
except OSError:pass
print('PI_PERSIST_AUDIT_JSON='+json.dumps(dict(read_only=True,remote_files_written=False,processes_changed=False,serial_commands_sent=False,
 hostname=os.uname().nodename,boot_id=(read('/proc/sys/kernel/random/boot_id') or '').strip(),uptime_seconds=float((read('/proc/uptime') or '0').split()[0]),remote_time_epoch=time.time(),root=str(root),processes=procs,main_processes=mains,device_owners=deviceowners,single_main_and_same_owner_tree=owner_single,device_owner_counts=owner_counts,unique_camera_and_serial_owners=unique_camera_serial,
 files=files,manifest=info(manifest_path,True),configs=configs,current_session_logs=logs,launch_entries=launch,related_crontab=cron,related_systemd=units,related_xfce_session_lines=sessions,mounts=mounts,fstab_relevant_lines=[clean(x) for x in fstab_lines],path_chain=paths,disk_space=disk,
 loaded_source_limit='CONFIG comes from the exact running argv session; file hashes are disk bytes, not direct Python-memory source hashes. Newer-than-start flags identify possible disk-new/process-old.',expected_config_hints=expected['config_hints']),ensure_ascii=False))
'''

def main():
 ap=argparse.ArgumentParser(description=__doc__);ap.add_argument('--audit',action='store_true');ap.add_argument('--ssh-helper',type=Path,default=R/'tmp/integration_mirror_ssh_20261117.py');a=ap.parse_args()
 c=json.loads((R/'evidence/integration/2026-11-18/three-frame-warm/candidate.json').read_text())
 boot=json.loads((R/'evidence/integration/2026-11-17/boot-ready-beep/candidate.json').read_text())
 code=dict(c['pi_unchanged_dependencies']);code.update(c['pi_changes'])
 code['boot_ready_handshake.py']=boot['pi_changes']['boot_ready_handshake.py']
 expected=dict(code_and_dependencies=code,config_hints=dict(body_profile_prefix='3aee1fdd',material_pickup_prefix='15a93e31',current_hsv_prefix='e77186cc',alignment_settle='300ms/5frames/2px; preserve actual current settings'))
 if not a.audit:
  print(json.dumps(dict(local_only=True,ssh_executed=False,expected=expected,read_only_audit_ready=True),ensure_ascii=False));return
 helper=a.ssh_helper.read_text(encoding='utf-8');bridge=(R/'tmp/integration18_ssh_bridge.py').read_text(encoding='utf-8')
 assert '10.99.242.211' in helper and '-hostkey' in bridge and 'SHA256:Gw+SS2OxPidCOUuL6uS3h78rkniG+gW05qeQZRCLpRA' in bridge,'Unexpected SSH target/pin; stop before connection'
 script=SCRIPT.replace('expected=EXPECTED','expected='+repr(expected),1)
 encoded=base64.b64encode(zlib.compress(script.encode('utf-8'),9)).decode('ascii')
 out=remote(a.ssh_helper,"import zlib,base64;exec(zlib.decompress(base64.b64decode("+repr(encoded)+")))")
 rows=[x.split('PI_PERSIST_AUDIT_JSON=',1)[1] for x in out.splitlines() if x.startswith('PI_PERSIST_AUDIT_JSON=')]
 assert len(rows)==1,'No structured audit; no application processes were changed'
 report=json.loads(rows[0]);E.mkdir(parents=True,exist_ok=True);p=E/('audit-'+datetime.datetime.now().strftime('%Y%m%d-%H%M%S')+'.json');p.write_text(json.dumps(report,indent=2)+'\n',encoding='utf-8')
 print(json.dumps(dict(report=str(p),hostname=report['hostname'],mains=report['main_processes'],single_owner=report['single_main_and_same_owner_tree'],code_hashes_match=all(v['matches_expected'] for v in report['files'].values()),read_only=True),ensure_ascii=False))
if __name__=='__main__':main()
