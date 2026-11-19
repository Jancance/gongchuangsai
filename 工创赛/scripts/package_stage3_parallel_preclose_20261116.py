"""Package overlays, compiled artifacts and offline evidence; no deployment."""
from pathlib import Path
import json,hashlib,zipfile
from prepare_stage3_parallel_preclose_20261116 import R,BASE,P,E
def main():
 c=json.loads((E/'candidate.json').read_text());checks=json.loads((E/'checks.json').read_text());assert checks['passed'] and c['build_passed']
 assert c['images']['Vehicle_LiftRuler']['sha256'].startswith('97744c768b7f')
 destination=R/'output/stage3_parallel_preclose_return600_20261116_97744c76_metadata1.zip'
 assert not destination.exists(),'Preserve previous package'
 files={}
 for n,h in c['changes'].items():
  p=P/n;assert hashlib.sha256(p.read_bytes()).hexdigest()==h;files['project/'+n]=p
 for target,item in c['images'].items():
  p=Path(item['axf']);assert hashlib.sha256(p.read_bytes()).hexdigest()==item['sha256'];files['project/Objects/'+target+'/'+p.name]=p
 for p in E.iterdir():
  if p.is_file() and p.suffix!='.exe' and p.name!='package.json':files['evidence/'+p.name]=p
 for n in ['scripts/prepare_stage3_parallel_preclose_20261116.py','scripts/build_stage3_parallel_preclose_20261116.py','scripts/validate_stage3_parallel_preclose_20261116.m','scripts/package_stage3_parallel_preclose_20261116.py','tests/stage3_return600_20261116.c','docs/STAGE3_PARALLEL_PRECLOSE_RETURN600_2026-11-16.md']:
  files[n]=R/n
 readme='''第三段并行160预收 + 返回600p，独立最小差异包

本包必须基于工作区 tmp/stage3_offline_ready_20261116/project 的967c已烧基线；不是完整工程或Pi安装包。
重建请复制该基线到独立目录，再覆盖project/中3个源文件，保留既有驱动/参数/工程。
双AXF仅供审查和显式部署决策，软件高审通过不代表已烧录或实车通过。
不得据本包自动连接探针、SSH、烧录、复位或发动作。现场持料状态由主代理/用户处理。
scripts验证依赖该工作区既有build_stage3_*辅助脚本和tests历史fixture；evidence包含本轮生成的实际C fixture、日志及SHA256。
MATLAB初始837与补充961通过顺序详见专题；软件计时不是实际夹爪角度反馈。
高审通过AXF：97744c768b7f7d23afeb626300f700ccb9fba0490d473077db3c7e0f8267ee11。
用户最新线断说明使旧30mm/剩25mm输入作废，不构成本包清除故障或实测通过证明。
'''
 with zipfile.ZipFile(destination,'w',compression=zipfile.ZIP_DEFLATED,compresslevel=6) as z:
  for n,p in sorted(files.items()):z.write(p,n)
  z.writestr('README_使用边界.txt',readme.encode('utf-8'))
 with zipfile.ZipFile(destination) as z:assert z.testzip() is None
 report={'package':str(destination),'sha256':hashlib.sha256(destination.read_bytes()).hexdigest(),'files':len(files)+1,'bytes':destination.stat().st_size,'baseline_required':str(BASE),'review_passed':True,'flashed':False,'hardware_access':False}
 (E/'package.json').write_text(json.dumps(report,indent=2)+'\n');print(json.dumps(report,ensure_ascii=False))
if __name__=='__main__':main()
