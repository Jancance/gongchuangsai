from pathlib import Path
import sys,json,os,subprocess
R=Path(__file__).resolve().parents[1];sys.path.insert(0,str(R/'scripts'))
from sync_team_handoff_20261102 import git,tree,D,G
p=json.loads((D/'upload-plan.json').read_text(encoding='utf-8'));base=p['base'];new=p['tree']
git('fetch','--no-auto-maintenance','origin')
current=git('rev-parse','origin/main').decode().strip()
assert current==base,'Upstream advanced: integrate before commit, never force push.'
before=tree(base);after=tree(new)
assert all(after.get(k)==v for k,v in before.items() if not k.startswith('工创赛/') and k!='P1电控组交接_2026-10-15.md')
assert all(k in after for k in before),'No upstream path may be deleted'
for k,v in p['changes'].items():assert after[k][1]==v['oid'],k
message='交接135度固定抓放完成状态并汇合最新电控基线\n\n记录用户确认的固定单循环、104000/27000预升回零及整车联调入口；交付机械臂独立源码快照、近期业务代码配置和验证记录。\n保留54bb617电控根工程与团队历史，导入远端新文件并归档本地差异；原始归档与重复采样图列明留本地。\n验证：135度实际Keil构建0错误0警告，快照SHA与68项工程引用核对；实物范围以用户口述为限，整车/动态视觉/库存尚待联调。\n'
commit=git('commit-tree',new,'-p',base,data=message.encode('utf-8')).decode().strip()
branch='refs/heads/codex/handoff-20261102';git('update-ref',branch,commit)
(D/'pending-commit.txt').write_text(commit+'\n',encoding='utf-8')
print('Commit',commit,'prepared; pushing main without force',flush=True)
q=subprocess.run(['git','-c','http.version=HTTP/1.1','-C',str(G),'push','origin',branch+':refs/heads/main'])
assert q.returncode==0,'Push failed; inspect latest remote and preserve local commit.'
remote=git('ls-remote','origin','refs/heads/main').decode().split()[0];assert remote==commit,(remote,commit)
git('fetch','--no-auto-maintenance','origin')
assert git('rev-parse','origin/main').decode().strip()==commit
env=os.environ.copy();env['GIT_INDEX_FILE']=str((D/'upload.index').resolve())
assert not git('diff','--cached','--name-only',commit,env=env)
result=dict(commit=commit,remote=remote,branch='main',parent=base,changed_paths=len(p['changes']),deleted_paths=0,independent_index_clean=True,
            project_root_is_legacy_import=True,local_updates=json.loads((D/'download-result.json').read_text(encoding='utf-8'))['local_changes'],
            excluded_artifacts=len(p['excluded']),root_firmware_preserved=True)
(D/'publish-result.json').write_text(json.dumps(result,ensure_ascii=False,indent=2),encoding='utf-8')
print('Verified remote main:',remote,'paths:',len(p['changes']),'no upstream deletions',flush=True)
