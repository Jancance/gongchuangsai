"""Minimal one-source overlay, dual artifacts, verified checks and metadata."""
import json,hashlib,zipfile
from pathlib import Path
from prepare_stage2_step20_20261117 import R,BASE,P,E
def main():
 c=json.loads((E/'candidate.json').read_text());assert c['review_passed'] and c['build_passed'] and c['exact_one_line_delta']
 dest=R/'output/stage2_debug215_step20_20261117_6203f1c0.zip';assert not dest.exists(),'Preserve prior delivery'
 files={}
 for n,h in c['changes'].items():
  p=P/n;assert hashlib.sha256(p.read_bytes()).hexdigest()==h;files['project/'+n]=p
 for t,item in c['images'].items():
  p=Path(item['axf']);assert hashlib.sha256(p.read_bytes()).hexdigest()==item['sha256'];files['project/Objects/'+t+'/'+p.name]=p
 for p in E.iterdir():
  if p.is_file() and p.name!='package.json' and p.suffix!='.exe':files['evidence/'+p.name]=p
 for n in ['prepare_stage2_step20_20261117.py','build_stage2_step20_20261117.py','package_stage2_step20_20261117.py','validate_stage2_step20_20261117.m']:files['scripts/'+n]=R/'scripts'/n
 files['docs/STAGE2_DEBUG215_STEP20_2026-11-17.md']=R/'docs/STAGE2_DEBUG215_STEP20_2026-11-17.md'
 readme='''第二段debug215最大20mm纠偏，基于已烧6a27源码的单文件增量包，非完整工程。
project/source_mcu_cycle.inc仅state2单次距离上限30→20，原min1/round/符号/5mm结束不变。
第二段同步、固定215debug/正式QR边界、S3及全部保护/速度/超时不变。
双AXF在线6203f1c0供主代理显式下载，Offline只构建验证。本包自身不连接硬件、不清错/复位/动作。
MATLAB2050检查02:18:39先于实现，actualC17987，双0错0警，高审通过。
仅软件就绪，不代表当前首故障已恢复或实车纠偏通过；部署/物料/承托/单轮现场由主代理新实证记录。
'''
 with zipfile.ZipFile(dest,'w',zipfile.ZIP_DEFLATED,compresslevel=6) as z:
  for n,p in sorted(files.items()):z.write(p,n)
  z.writestr('README_使用边界.txt',readme.encode('utf-8'))
 with zipfile.ZipFile(dest) as z:assert z.testzip() is None
 report={'package':str(dest),'sha256':hashlib.sha256(dest.read_bytes()).hexdigest(),'files':len(files)+1,'bytes':dest.stat().st_size,'baseline_required':str(BASE),'hardware_access':False,'flashed_by_this_package':False}
 (E/'package.json').write_text(json.dumps(report,indent=2)+'\n');print(json.dumps(report,ensure_ascii=False))
if __name__=='__main__':main()
