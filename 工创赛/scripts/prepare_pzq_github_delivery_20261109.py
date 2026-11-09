"""Review/stage an additive, byte-preserving delivery against fetched main.

Uses a separate index and codex branch in the existing remote bare mirror;
never touches the uncommitted root worktree, deletes remote files or force pushes.
Default only plans. --stage writes Git objects/index, --commit creates a commit,
and --push requires that commit and a fresh unchanged remote main.
"""
from pathlib import Path
import argparse,collections,hashlib,json,os,re,subprocess

R=Path(__file__).resolve().parents[1]
G=R/'tmp/team_review_20261106.git'
D=R/'tmp/pzq_github_delivery_20261109'
D.mkdir(parents=True,exist_ok=True)
SCOPE=R/'docs/PZQ_UPLOAD_SCOPE_2026-11-09.json'
BRANCH='refs/heads/codex/pzq-delivery-20261109'

def git(*args,data=None,env=None):
 p=subprocess.run(['git','--git-dir='+str(G),'-c','core.quotePath=false',*args],input=data,
                  capture_output=True,env=env)
 if p.returncode:raise RuntimeError(p.stderr.decode('utf-8',errors='replace')[-3000:])
 return p.stdout

def tree(ref):
 result={}
 for row in git('ls-tree','-rz',ref).split(b'\0'):
  if row:
   head,name=row.split(b'\t',1);mode,kind,oid=head.decode().split()
   if kind=='blob':result[name.decode('utf-8')]=(mode,oid)
 return result

def oid(data):return hashlib.sha1(b'blob '+str(len(data)).encode()+b'\0'+data).hexdigest()
def write(path,data):path.parent.mkdir(parents=True,exist_ok=True);path.write_bytes(data)

def main():
 parser=argparse.ArgumentParser();parser.add_argument('--stage',action='store_true')
 parser.add_argument('--commit',action='store_true');parser.add_argument('--push',action='store_true')
 args=parser.parse_args()
 base=git('rev-parse','FETCH_HEAD').decode().strip();T=tree(base)
 assert base==git('ls-remote','origin','refs/heads/main').decode().split()[0],'Fetch main again before preparing'
 candidates={};excluded=[]
 def add(dest,path):assert path.is_file();candidates[dest]=path
 def omit(path,reason):excluded.append({'path':path.relative_to(R).as_posix(),'bytes':path.stat().st_size,'reason':reason})
 binary_artifacts={'.exe','.dll','.axf','.o','.obj','.bin','.pyc','.pyo','.zip','.gz','.7z','.rar',
                   '.mp4','.avi','.mkv','.rknn','.onnx','.mat','.fig','.slx'}
 cacheparts={'__pycache__','.pytest_cache','.git','.venv','node_modules','Objects','Listings'}
 folders=['docs','scripts','software_interfaces','config','calibration','src','tests','vision_app','systemd','data','evidence']
 for folder in folders:
  for p in (R/folder).rglob('*'):
   if not p.is_file():continue
   rel=p.relative_to(R).as_posix()
   if any(x in cacheparts for x in p.relative_to(R).parts):continue
   if p==SCOPE:continue
   if folder=='data' and rel!='data/README.md':omit(p,'local raw/training/replay data');continue
   if p.suffix.lower() in binary_artifacts:omit(p,'archive/generated model/compiler artifact; final pzq AXFs included explicitly');continue
   if '.uvguix.' in p.name or p.suffix.lower() in {'.tenderness','.lenovo','.uvoptx'}:
    omit(p,'personal debug layout or historical generated project options');continue
   # The independent original logs and review results are included. Thousands
   # of repeat image renderings are not required to reproduce the new control.
   if folder=='evidence' and p.suffix.lower() in {'.png','.jpg','.jpeg','.html'}:
    representative=('first-frame-raw' in p.name or 'source-vision-new-pose' in rel or
                    '/paper48-' in rel or '/finalstop-' in rel or 'calibration' in rel)
    if not representative and '工创赛/'+rel not in T:
     omit(p,'repeated diagnostic image/report render; raw numeric logs and summaries included');continue
   if p.stat().st_size>45_000_000:
    omit(p,'large individual artifact; retained locally, summarized in numeric evidence');continue
   add('工创赛/'+rel,p)
 for folder in ['pzq','integration13_final_20261109','stage2_pick_20261108',
                'p1_2026-10-15','p2_fixed_cycle_20261102','arm_fixed_replay_20261101',
                'arm_return_home_20261102','arm_source_pick_20261102']:
  root=R/'handoff'/folder
  if not root.exists():continue
  for p in root.rglob('*'):
   if not p.is_file() or any(x in cacheparts for x in p.relative_to(root).parts):continue
   if '.uvguix.' in p.name or p.suffix.lower() in {'.pyc','.exe','.o','.obj'}:continue
   if folder not in {'pzq','integration13_final_20261109'} and p.suffix.lower() in binary_artifacts:
    omit(p,'historical build/archive; completed source and manifest retained');continue
   add('工创赛/'+p.relative_to(R).as_posix(),p)
 for name in ['README.md','AGENTS.md','pyproject.toml']:
  add('工创赛/'+name,R/name)
 for name in ['P1电控组交接_2026-10-15.md','新对话接续_2026-10-31.md','队友接续_2026-11-03.md']:
  if (R/name).exists():add(name,R/name)
 # Root shared AGENTS: preserve the confirmed full clauses, adapt only the
 # documented repo-layout prefix. Keep the previous remote bytes recoverable.
 old_agents=git('show',base+':AGENTS.md');write(D/'remote-AGENTS-before.md',old_agents)
 write(R/'docs/archive/TEAM_REMOTE_AGENTS_BEFORE_PZQ_2026-11-09.md',old_agents)
 add('工创赛/docs/archive/TEAM_REMOTE_AGENTS_BEFORE_PZQ_2026-11-09.md',R/'docs/archive/TEAM_REMOTE_AGENTS_BEFORE_PZQ_2026-11-09.md')
 root_agents=(R/'AGENTS.md').read_bytes().replace(b'`docs/competition_requirements_2027.md`',
  '`工创赛/docs/competition_requirements_2027.md`'.encode())
 write(D/'root-AGENTS.md',root_agents);add('AGENTS.md',D/'root-AGENTS.md')
 old_readme=git('show',base+':README.md');write(D/'remote-README-before.md',old_readme)
 intro='''> 2026-11-09最新：[pzq抓取动作基线](工创赛/handoff/pzq/README.md)已命名并完整交付。左156mm、右160mm、放料37°、松爪后200ms；17.987秒第三次松爪命令、19.185秒回0°，用户确认三槽放稳/无碰擦/停稳。底盘队友先读[原料区与底盘接续交接](工创赛/docs/PZQ_CHASSIS_TEAM_HANDOFF_2026-11-09.md)和[根交接](P1电控组交接_2026-10-15.md)。pzq完整工程在`工创赛/handoff/pzq/project/`，仓库根`3.stm32f407工程/`保留既有底盘版本。本次没有把视觉毫米纠偏、颜色实例自主调用或第三段行驶写成已完成。下方2026-11-03说明为历史。

'''.encode('utf-8')
 split=old_readme.index(b'\n')+1;write(D/'root-README.md',old_readme[:split]+b'\n'+intro+old_readme[split:])
 add('README.md',D/'root-README.md')
 attrs=git('show',base+':.gitattributes')
 attrs+=b'\n# pzq snapshot/source and raw evidence are byte-verified; preserve checkout bytes.\n'
 attrs+='工创赛/handoff/pzq/** -text\n工创赛/handoff/integration13_final_20261109/** -text\n工创赛/evidence/** -text\n'.encode()
 write(D/'root-gitattributes',attrs);add('.gitattributes',D/'root-gitattributes')
 # Root and sub-project docs/evidence share the same current state.
 add('工创赛/P1电控组交接_2026-10-15.md',R/'P1电控组交接_2026-10-15.md')
 suspicious=[];changes={};unchanged=0
 secrets=re.compile(rb'(?:gh[pousr]_[A-Za-z0-9]{30,}|github_pat_[A-Za-z0-9_]{30,}|sk-proj-[A-Za-z0-9_-]{20,}|-----BEGIN (?:RSA |OPENSSH |EC )?PRIVATE KEY-----)')
 assignments=re.compile(rb'(?:password|passwd|api_key|access_token|secret_key)\s*[:=]\s*[\x22\x27]([^\x22\x27\r\n]{4,})',re.I)
 credential_review=[]
 for dest,p in candidates.items():
  data=p.read_bytes();h=oid(data)
  if T.get(dest,('',None))[1]==h:unchanged+=1;continue
  if secrets.search(data):suspicious.append(dest)
  for match in assignments.finditer(data):
   value=match.group(1)
   if value.lower() not in {b'test',b'password',b'changeme',b'your_password',b'none',b'false',b'xxxx'}:
    credential_review.append({'path':dest,'line':data[:match.start()].count(b'\n')+1})
  changes[dest]={'local':str(p),'oid':h,'sha256':hashlib.sha256(data).hexdigest(),
                 'bytes':len(data),'mode':T.get(dest,('100644',''))[0]}
 assert not suspicious,'Credential-like data found; paths only: '+repr(suspicious)
 # Credentials must be reviewed before --stage/commit/push. Reports never print
 # or publish the matching value. Project descriptions with field names alone
 # are not a credential. See the local redacted review for any literals.
 (D/'credential-review-redacted.json').write_text(json.dumps(credential_review,indent=2)+'\n',encoding='utf-8')
 counts=collections.Counter(p.split('/')[1] if p.startswith('工创赛/') else '<repo-root>' for p in changes)
 scope={'date':'2026-11-09','base_main':base,'name':'pzq','target_remote_branch':'main',
  'changed_files_before_scope_file':len(changes),'changed_bytes_before_scope_file':sum(x['bytes'] for x in changes.values()),
  'unchanged_selected_files':unchanged,'changed_by_category':dict(counts),
  'remote_root_firmware_preserved':True,'remote_deletions':[],
  'kept_local_categories':['tmp candidates and third-party debug dependencies','old duplicate AXFs/debug layouts',
   'root ZIP/TAR archives and videos','repeated diagnostic renders and generated executables',
   'output deployment archives; authoritative source/config/evidence uploaded'],
  'excluded_files':excluded,'credential_literals_pending_review':len(credential_review),
  'meaning':'All current completed software/doc/config/interface/test and numeric evidence deliverables; not every disk cache/archive.',
  'changed_paths':sorted(changes),'new_credential_tokens_found':False}
 SCOPE.write_text(json.dumps(scope,ensure_ascii=False,indent=2)+'\n',encoding='utf-8')
 data=SCOPE.read_bytes();changes['工创赛/docs/'+SCOPE.name]={'local':str(SCOPE),'oid':oid(data),
  'sha256':hashlib.sha256(data).hexdigest(),'bytes':len(data),'mode':'100644'}
 plan={'base':base,'changes':changes,'scope':str(SCOPE),'tree':None,'branch':BRANCH,
       'remote_root_firmware_preserved':True,'deletions':[]}
 (D/'upload-plan.json').write_text(json.dumps(plan,ensure_ascii=False,indent=2)+'\n',encoding='utf-8')
 print(json.dumps({'base':base,'files':len(changes),'MB':round(sum(x['bytes'] for x in changes.values())/1e6,2),
                  'unchanged_selected':unchanged,'excluded':len(excluded),'credential_literal_review':len(credential_review),
                  'categories':dict(counts)},indent=2),flush=True)
 if not (args.stage or args.commit or args.push):return
 assert not credential_review,'Review credential literals and exclude any actual secrets before staging'
 env=os.environ.copy();env['GIT_INDEX_FILE']=str(D/'delivery.index')
 git('read-tree',base,env=env)
 paths=[Path(x['local']).resolve().as_posix() for x in changes.values()]
 hashes=git('hash-object','--no-filters','-w','--stdin-paths',data=('\n'.join(paths)+'\n').encode('utf-8')).decode().split()
 assert len(hashes)==len(changes)
 records=[]
 for (dest,item),h in zip(changes.items(),hashes):
  assert h==item['oid'],dest
  records.append((item['mode']+' '+h+'\t'+dest).encode()+b'\0')
 git('update-index','-z','--index-info',data=b''.join(records),env=env)
 staged_tree=git('write-tree',env=env).decode().strip();N=tree(staged_tree)
 assert not set(T)-set(N),'Unexpected remote deletion'
 assert all(N[k]==v for k,v in T.items() if k.startswith('3.stm32f407工程/')),'Root chassis changed'
 assert all(N[k]==v for k,v in T.items() if k not in changes),'Unrelated remote data changed'
 plan['tree']=staged_tree
 (D/'upload-plan.json').write_text(json.dumps(plan,ensure_ascii=False,indent=2)+'\n',encoding='utf-8')
 (D/'diff-stat.txt').write_bytes(git('diff','--stat','--no-renames',base,staged_tree))
 print('Staged additive tree',staged_tree,'remote chassis and all unselected paths preserved',flush=True)
 if not (args.commit or args.push):return
 git('fetch','origin','main');fresh=git('rev-parse','FETCH_HEAD').decode().strip()
 assert fresh==base,'Remote moved; rebuild reviewed tree against latest main'
 message='''Freeze verified pzq pickup baseline and hand off chassis integration

Name the accepted three-piece motion pzq without altering its firmware bytes.
Deliver complete source/AXFs,17.987s third-release and19.185s zero-return
evidence, accumulated local software/config/validation, and a chassis handoff.
Document vision-alignment-before-pickup policy and remaining autonomous/route3
work. Preserve the shared root chassis tree, all remote-only files and the
full confirmed AGENTS workflow. No hardware operations in this publication.
'''
 commit=git('commit-tree',staged_tree,'-p',base,data=message.encode()).decode().strip()
 git('update-ref',BRANCH,commit)
 plan['commit']=commit;(D/'upload-plan.json').write_text(json.dumps(plan,ensure_ascii=False,indent=2)+'\n',encoding='utf-8')
 print('Created',commit,BRANCH,flush=True)
 if args.push:
  git('push','origin',BRANCH+':refs/heads/main')
  remote=git('ls-remote','origin','refs/heads/main').decode().split()[0]
  assert remote==commit,'Remote verification failed'
  plan['pushed']=True;plan['verified_remote_commit']=remote
  (D/'upload-result.json').write_text(json.dumps(plan,ensure_ascii=False,indent=2)+'\n',encoding='utf-8')
  print('VERIFIED origin/main',remote,flush=True)

if __name__=='__main__':main()
