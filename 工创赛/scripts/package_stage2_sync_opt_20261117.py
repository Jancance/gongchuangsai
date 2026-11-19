"""Minimal reviewed overlay and compiled artifacts; never deploys."""
import hashlib,json,zipfile
from pathlib import Path
from prepare_stage2_sync_opt_20261117 import R,BASE,P,E
def main():
 c=json.loads((E/'candidate.json').read_text());assert c['build_passed'] and c['source_rebuild_byte_identical']
 h=c['images']['Vehicle_LiftRuler']['sha256'];dest=R/('output/stage2_sync_opt_20261117_'+h[:8]+'.zip')
 assert not dest.exists(),'Preserve delivery'
 files={}
 for n,digest in c['changes'].items():
  p=P/n;assert hashlib.sha256(p.read_bytes()).hexdigest()==digest;files['project/'+n]=p
 for target,item in c['images'].items():
  p=Path(item['axf']);assert hashlib.sha256(p.read_bytes()).hexdigest()==item['sha256'];files['project/Objects/'+target+'/'+p.name]=p
 for p in E.iterdir():
  if p.is_file() and p.suffix!='.exe' and p.name!='package.json':files['evidence/'+p.name]=p
 for name in ['prepare_stage2_sync_opt_20261117.py','check_stage2_sync_opt_20261117.py','build_stage2_sync_opt_20261117.py','package_stage2_sync_opt_20261117.py','validate_stage2_sync_opt_20261117.m']:
  files['scripts/'+name]=R/'scripts'/name
 files['docs/STAGE2_SYNC_OPT_2026-11-17.md']=R/'docs/STAGE2_SYNC_OPT_2026-11-17.md'
 readme='''第二段缓存同步与100ms释放，本地最小增量包。
基线必须是 tmp/stage3_parallel_preclose_20261116/project（已烧97744c76），不是完整工程。
project/仅四个源码覆盖和双AXF。按专题审查后另行决定部署；本轮未烧录/实测，不自动探针/SSH/复位/发车。
默认PC13仍第三段调试，不是刷后直接测第二段；既有region1入口见专题。
统一放行是CAN1广播同调用DMgo，实际DM目标沿原DM服务发送，不代表物理零差。
MATLAB初400检查00:56:51的工具stdout是初前置证据，当前JSON是420检查01:20:44补STOP后的结果。
evidence内保留生成的实际C夹具、对应日志与完整源码manifest，可在基线上用gcc独立编译；scripts还依赖本工作区已有历史helper/fixture。
当前夹持/供电未知，第三段等队友机械模型后重做；实物闭环仍开放。禁止把软件通过记成实车成功。
'''
 with zipfile.ZipFile(dest,'w',zipfile.ZIP_DEFLATED,compresslevel=6) as z:
  for n,p in sorted(files.items()):z.write(p,n)
  z.writestr('README_使用边界.txt',readme.encode('utf-8'))
 with zipfile.ZipFile(dest) as z:assert z.testzip() is None
 report={'package':str(dest),'sha256':hashlib.sha256(dest.read_bytes()).hexdigest(),'files':len(files)+1,'bytes':dest.stat().st_size,'baseline_required':str(BASE),'flashed':False,'hardware_access':False}
 (E/'package.json').write_text(json.dumps(report,indent=2)+'\n');print(json.dumps(report,ensure_ascii=False))
if __name__=='__main__':main()
