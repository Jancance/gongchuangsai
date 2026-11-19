"""Minimal firmware/Pi overlays and review evidence; never deploys."""
from pathlib import Path
import json,zipfile,hashlib
from prepare_stage2_settle_ring_20261117 import R,BASE,P,E,PIBASE,PI
def main():
 c=json.loads((E/'candidate.json').read_text());assert c['review_passed'] and c['build_passed']
 dest=R/'output/stage2_settle_ring_20261117_18cc11ee.zip';assert not dest.exists(),'Preserve delivery'
 files={}
 for n,h in c['changes'].items():
  p=P/n;assert hashlib.sha256(p.read_bytes()).hexdigest()==h;files['project/'+n]=p
 for n,h in c['pi_changes'].items():
  p=PI/n;assert hashlib.sha256(p.read_bytes()).hexdigest()==h;files['pi-overlay/'+n]=p
 files['pi-overlay/manifest.json']=PI/'manifest.json'
 for t,image in c['images'].items():
  p=Path(image['axf']);assert hashlib.sha256(p.read_bytes()).hexdigest()==image['sha256'];files['project/Objects/'+t+'/'+p.name]=p
 names=['candidate.json','checks.json','source-manifest.json','pi-manifest.json','matlab-result.json','c-checks.json','pi-checks.json','Vehicle_LiftRuler-build.log','Vehicle_LiftRuler_Offline-build.log','actual-chassis-ring.c','actual-chassis-ring.log','actual-legacy-distance-ring.c','actual-legacy-distance-ring.log','actual-settle-wire-consumer.c','actual-settle-wire-consumer.log','pi-actual-process-trace.json','pi-actual-tx-wires.json']
 for n in names:files['evidence/'+n]=E/n
 for n in ['prepare_stage2_settle_ring_20261117.py','source_alignment_settle_impl_20261117.py','check_stage2_settle_ring_c_20261117.py','check_stage2_settle_pi_20261117.py','finalize_stage2_settle_ring_20261117.py','package_stage2_settle_ring_20261117.py','validate_stage2_settle_ring_20261117.m']:files['scripts/'+n]=R/'scripts'/n
 files['docs/STAGE2_SETTLE_RING_2026-11-17.md']=R/'docs/STAGE2_SETTLE_RING_2026-11-17.md'
 text='''第二段300ms后5新图最大pair距离2px + 底盘诊断最近8条循环，独立本地候选。
固件基线必须是已烧6203的tmp/stage2_debug215_step20_20261117/project，project仅两源码overlay+双AXF。
Pi基线必须原stage23_desktop_20261116的64manifest包；pi-overlay仅3文件delta+新66项完整manifest。
不是完整工程/自动安装包。原S3/抓取120与5帧/HSV/J/20mm不改。此包不连接探针/SSH/相机/串口，不清错/复位/运动。
host完成时刻不是曝光时刻；软件通过不等于实车稳图或纠偏收敛。当前fault不得因包就绪自动恢复。
MATLAB初34302:44:55工具stdout先于初实现，当前JSON为352补充，详见专题。
双Keil0错0警在线18cc11ee/Offlinebb7c2ac4，高审/源码与Pi清单见证据；只能在明确现场部署授权下由主代理安装。
scripts依赖工作区既有辅助夹具，evidence保存本轮实际生成夹具及日志，源码完整清单用于核基线与overlay。
'''
 with zipfile.ZipFile(dest,'w',zipfile.ZIP_DEFLATED,compresslevel=6) as z:
  for n,p in sorted(files.items()):z.write(p,n)
  z.writestr('README_使用边界.txt',text.encode('utf-8'))
 with zipfile.ZipFile(dest) as z:assert z.testzip() is None
 report={'package':str(dest),'sha256':hashlib.sha256(dest.read_bytes()).hexdigest(),'bytes':dest.stat().st_size,'files':len(files)+1,'baseline_required':str(BASE),'pi_baseline_required':str(PIBASE),'hardware_access':False,'flashed':False}
 (E/'package.json').write_text(json.dumps(report,indent=2)+'\n');print(json.dumps(report,ensure_ascii=False))
if __name__=='__main__':main()
