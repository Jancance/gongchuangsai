"""Package only the explicit debug delta and its actual software evidence."""
import json,hashlib,zipfile
from pathlib import Path
from prepare_stage2_debug215_20261117 import R,BASE,P,E
def main():
 c=json.loads((E/'candidate.json').read_text());assert c['build_passed'] and c['review_passed']
 dest=R/'output/stage2_debug215_20261117_6a2752d9.zip';assert not dest.exists(),'Preserve previous package'
 files={}
 for n,h in c['changes'].items():
  p=P/n;assert hashlib.sha256(p.read_bytes()).hexdigest()==h;files['project/'+n]=p
 for target,item in c['images'].items():
  p=Path(item['axf']);assert hashlib.sha256(p.read_bytes()).hexdigest()==item['sha256'];files['project/Objects/'+target+'/'+p.name]=p
 evidence=['candidate.json','source-manifest.json','checks.json','matlab-result.json','Vehicle_LiftRuler-build.log','Vehicle_LiftRuler_Offline-build.log','actual-debug215.c','actual-debug215.log','actual-formal-qr.c','actual-formal-qr.log','base-commit-fixture.c','tested-commit-owner.inc','tested-region-dispatch.inc','tested-source-cycle.inc','tested-key-branch.inc','tested-saoma.c']
 for n in evidence:files['evidence/'+n]=E/n
 for p in E.glob('*.h'):files['evidence/'+p.name]=p
 for n in ['prepare_stage2_debug215_20261117.py','build_stage2_debug215_20261117.py','package_stage2_debug215_20261117.py','validate_stage2_debug215_20261117.m']:
  files['scripts/'+n]=R/'scripts'/n
 files['docs/STAGE2_DEBUG215_2026-11-17.md']=R/'docs/STAGE2_DEBUG215_2026-11-17.md'
 text='''第二段独立黄→红→黑215调试增量包，基于8865源码。
不是完整工程。只project四文件delta及双AXF。在线6a2752d9用于后续显式部署；Offline仅构建验证。
公共宏默认0，本独立uvproj两target显式SOURCE_CYCLE_TEST_215=1，初始region1，必须用户PC13启动。
不伪造扫码/全QR/扫码锁定；原正式宏0分支保留。固定215不计入正式扫码或全码显示验收。
原8865与已烧9774源码未覆盖。此包自身不连接硬件、不烧录、不发动作，不代用户按键。
本轮旧Pi地址172.22.13.211 timeout，当前Pi owner/config/TX尚待新IP实证；默认preview不发观测。
源码及AXF高审通过，不代表已下载或实车三件通过；后续部署日志由主代理独立记录。
证据中的实际C可基于工程头文件用gcc运行；脚本还需本工作区既有helper/fixture。
'''
 with zipfile.ZipFile(dest,'w',zipfile.ZIP_DEFLATED,compresslevel=6) as z:
  for n,p in sorted(files.items()):z.write(p,n)
  z.writestr('README_使用边界.txt',text.encode('utf-8'))
 with zipfile.ZipFile(dest) as z:assert z.testzip() is None
 report={'package':str(dest),'sha256':hashlib.sha256(dest.read_bytes()).hexdigest(),'files':len(files)+1,'bytes':dest.stat().st_size,'baseline_required':str(BASE),'flashed_by_this_package':False,'hardware_access':False}
 (E/'package.json').write_text(json.dumps(report,indent=2)+'\n');print(json.dumps(report,ensure_ascii=False))
if __name__=='__main__':main()
