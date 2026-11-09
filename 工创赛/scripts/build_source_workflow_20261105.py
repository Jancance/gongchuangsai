"""Return the preview entry to upstream turntable/material modes; no hardware I/O."""
from pathlib import Path
import hashlib
import json
import shutil
import zipfile

ROOT = Path(__file__).resolve().parents[1]
source = ROOT/'output/auto-camera-trial-20261105/vision_app'
base = ROOT/'output/source-workflow-20261105'
app = base/'vision_app'
if base.exists():
    raise RuntimeError('Output already exists; refusing overwrite')
shutil.copytree(source, app, ignore=shutil.ignore_patterns('__pycache__', 'diagnostics'))
main = app/'code/main.py'
text = main.read_text(encoding='utf-8')
assert text.count('    current_mode = Mode.FOLLOW\n') == 1
text = text.replace('    current_mode = Mode.FOLLOW\n', '    current_mode = Mode.TURNTABLE\n', 1)
needle = '"serial":"DISABLED","features"'
assert text.count(needle) == 1
text = text.replace(needle, '"serial":"DISABLED","initial_mode":"TURNTABLE","material_color_config":str(args.material_color_config.resolve()),"features"', 1)
main.write_text(text, encoding='utf-8')
(base/'现场操作.md').write_text('''# 原料区定位与颜色设置入口

仅在相机固定、4310/伸缩到位后的实际取料姿态设置。旧桌面图片不作该姿态标定。
使用同一vision_app，先退出预览再开设置工具，保存后重启主程序。

1. python3 vision_app/run.py --feature turntable-settings
   按A自动诊断；确认拟合的是原料盘外边缘而非物料圆面，S保存，Q退出。
2. python3 vision_app/run.py --feature color
   1～6选色，物料内部拖框采样，滑条调整，S保存，Q退出。
3. python3 vision_app/run.py --feature pickup-regions
   R重画，依次活动区、左/右/下抓取区；S保存，Q退出。
4. python3 vision_app/run.py
   默认A原料盘定位；M物料抓取识别。按Q退出。

code/config/material_color_thresholds.json为同目录工具及物料识别共用HSV文件。
code/config/turntable_detector.json为原料盘参数，抓取区域在material_pickup_detector.json。
相机沿用AE3/持续AWB候选，启动有控制读回；现场参数应在同一相机设置下调整。
目录内现有HSV来自上一轮；不得将其解释为本次到位姿态已调好。

本包是接入前现场配置入口：串口关闭，无底盘或机械臂命令。
尚未接入机构到位触发、盘心到车辆修正、位置保持、两色推第三色、自动抓放或第三段放行。
原项目M模式含先确认转动、再接近停稳的判定；静止测试时颜色框不等于ready。
后续会按本车原料盘运行方式接续，不通过伪造运动/ready绕过。
''', encoding='utf-8')
changed = sorted(p.relative_to(source).as_posix() for p in source.rglob('*')
                 if p.is_file() and '__pycache__' not in p.parts and p.read_bytes() != (app/p.relative_to(source)).read_bytes())
assert changed == ['code/main.py']
out = ROOT/'output/source-workflow-delivery'
out.mkdir(exist_ok=True)
archive = out/'p2_source_workflow_20261105.zip'
hashes = {p.relative_to(base).as_posix(): hashlib.sha256(p.read_bytes()).hexdigest()
          for p in base.rglob('*') if p.is_file() and '__pycache__' not in p.parts}
with zipfile.ZipFile(archive, 'w', zipfile.ZIP_DEFLATED) as z:
    for name in hashes:
        z.write(base/name, name)
    z.writestr('source-workflow-manifest.json', json.dumps(hashes, indent=2))
with zipfile.ZipFile(archive) as z:
    assert z.testzip() is None
report = dict(file=archive.name, bytes=archive.stat().st_size,
              sha256=hashlib.sha256(archive.read_bytes()).hexdigest(),
              changes_from_auto_candidate=changed, camera_opened=False, serial_opened=False,
              firmware_changed=False, deployed=False, initial_mode='TURNTABLE')
(out/'delivery.json').write_text(json.dumps(report, indent=2), encoding='utf-8')
print(json.dumps(report, indent=2))
