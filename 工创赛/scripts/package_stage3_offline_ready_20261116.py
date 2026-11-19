"""Package reviewed local artifacts only. No hardware/network/process control."""
from pathlib import Path
import hashlib,json,zipfile
R=Path(__file__).resolve().parents[1]
E=R/'evidence/integration/2026-11-16/stage3-offline-ready'
def main():
 c=json.loads((E/'candidate.json').read_text());assert c['build_passed'] and not c['flashed'] and not c['hardware_access']
 files=[R/'docs/STAGE3_OFFLINE_READY_2026-11-16.md',R/'output/stage23_desktop_20261116.zip',R/'output/stage23_desktop_20261116/OFFLINE_READY.md']
 files += [R/'scripts'/n for n in ['stage3_live_20261116.py','stage3_distance_capture_20261116.py','stage3_teach_readonly_diag_20261116.py','stage3_pi_install_20261116.py']]
 files += [R/'tests/test_stage3_live_20261116.py']
 files += [R/'scripts'/n for n in ['prepare_stage3_offline_ready_20261116.py','finish_stage3_offline_ready_20261116.py','build_stage3_offline_ready_20261116.py','package_stage3_offline_ready_20261116.py','validate_stage3_offline_ready_20261116.m']]
 files += [R/'src/stage3_offline_ready_20261116'/n for n in c['changes']]
 files += [R/c['project']/n for n in {**c['changes'],**c['unchanged_sha256']}]
 files += [R/c['project']/'gongxun-f407.uvprojx']
 files += [R/v['axf'] for v in c['images'].values()]
 files += [E/n for n in ['candidate.json','checks.json','rebuild-check.json','matlab-result.json','matlab-taskmap-console.log','matlab-publication-console.log','matlab-geometry-console.log','calibration-config.json','calibration-provenance.json','Vehicle_LiftRuler-build.log','Vehicle_LiftRuler_Offline-build.log','actual-offline-owner.log','actual-loaded-task.log','actual-preclose-it.log','actual-deep-z.log','actual-owner-lrc.log','actual-home-exit.log','actual-source-group.log','host-tests.log','host-local-manifest.json','host-actual-elf-layout.json']]
 files += [R/'evidence/integration/2026-11-16/stage3-preclose160/teach-readonly-20261116-065232-pick-config.json']
 out=R/'output/stage3_offline_ready_20261116.zip'
 payload={p.relative_to(R).as_posix():p.read_bytes() for p in files}
 pi='output/stage23_desktop_20261116.zip'
 pi_sha=hashlib.sha256(payload[pi]).hexdigest()
 assert pi_sha=='e1a41fc7596f9fba98a11bcbbb144cbe23b57068cc3733ed9f0ef6b68bd2fe3f'
 for name,sha in {**c['changes'],**c['unchanged_sha256']}.items():assert hashlib.sha256(payload[c['project']+'/'+name]).hexdigest()==sha,name
 for image in c['images'].values():assert hashlib.sha256(payload[image['axf']]).hexdigest()==image['sha256']
 assert len(json.loads((E/'calibration-config.json').read_text()))==5
 assert len(json.loads((E/'matlab-result.json').read_text())['checks'])==8
 payload['README_OFFLINE_DELIVERY.md']=('''# 第三段离线交付

固件967c9140、host22测试、Pi64项e1a41fc7最终高档审查均已通过；软件证据不代表现场通过。本包未部署/烧录/安装，无硬件访问。
总入口：docs/STAGE3_OFFLINE_READY_2026-11-16.md。Pi包：output/stage23_desktop_20261116.zip，桌面指南：output/stage23_desktop_20261116/OFFLINE_READY.md。
这是增量包，不是完整独立工程。重建依赖保留的tmp/stage3_preclose160_20261116/project完整基线、原仓库tests/构建帮助脚本和工具链。host显式现场动作也依赖原仓库模块与既有环境，不保证本包独立开箱。不要以包内局部工程覆盖完整工程。
prepare→finish→build脚本重建要求全新候选目录；八组MATLAB增量时序以总指南/candidate为准，不宣称全部模型先于全部源码。
仅本地hash核验（解压根目录）：
python scripts/stage3_live_20261116.py --manifest evidence/integration/2026-11-16/stage3-offline-ready/candidate.json
必须显式选择本manifest，避免默认选旧候选；此命令无动作参数，不访问硬件。包生成脚本也仅处理本地文件。不运行installer，不自动恢复配置，不切换自启。
内部MANIFEST.sha256.json校验其他全部条目，排除自身以免循环；ZIP使用固定时间、权限与排序。同输入同运行时可重现，外部delivery-report.json记录两个ZIP的SHA256。
五字段纯配置与provenance分开；不恢复历史运行参考。原料参数与算法保持。
单件未实测、B3/TCP未验，两侧几何/真实放高/带料净空/完整三件仍开放。SourceCycle Low未产生strict commit，真实扫码跨区自动handoff未接；只提供独立Stage3_BindMission API，不能宣称整场完成。
R核RUN-01/02、FIELD-06、PWR-01、MECH-01/02；E核CAL-02/03/05、VIS-09/12/14/15、RUN-06；Q保持待技术答疑确认。成果仅本地，未推送GitHub。
''').encode('utf-8')
 manifest={name:dict(sha256=hashlib.sha256(data).hexdigest(),bytes=len(data)) for name,data in sorted(payload.items())}
 payload['MANIFEST.sha256.json']=(json.dumps(manifest,ensure_ascii=False,indent=2)+'\n').encode('utf-8')
 with zipfile.ZipFile(out,'w',zipfile.ZIP_DEFLATED,compresslevel=9) as z:
  for name,data in sorted(payload.items()):
   info=zipfile.ZipInfo(name,(2026,11,16,0,0,0));info.compress_type=zipfile.ZIP_DEFLATED;info.create_system=3;info.external_attr=0o100644<<16
   z.writestr(info,data,compress_type=zipfile.ZIP_DEFLATED,compresslevel=9)
 with zipfile.ZipFile(out) as z:
  assert z.testzip() is None and len(z.namelist())==len(payload)
  for name,item in manifest.items():assert hashlib.sha256(z.read(name)).hexdigest()==item['sha256']
 report=dict(path=out.relative_to(R).as_posix(),sha256=hashlib.sha256(out.read_bytes()).hexdigest(),bytes=out.stat().st_size,files=len(payload),manifest='MANIFEST.sha256.json',pi_zip=dict(path=pi,sha256=pi_sha,bytes=len(payload[pi])),entry='docs/STAGE3_OFFLINE_READY_2026-11-16.md',hardware_access=False,flashed=False,deployed=False,pushed=False,incremental_requires_preserved_base=True,final_review='firmware, host and Pi passed; session evidence; no physical acceptance',zip_verified=True)
 for name in ['package-report.json','delivery-report.json']:(E/name).write_text(json.dumps(report,indent=2)+'\n',encoding='utf-8')
 print(json.dumps(report))
if __name__=='__main__':main()
