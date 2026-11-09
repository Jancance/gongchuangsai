"""Build a reversible observation delivery. Does not operate SBC or robot."""
from pathlib import Path
import hashlib
import json
import shutil
import zipfile

ROOT = Path(__file__).resolve().parents[1]
BASE = ROOT / 'output/source-workflow-20261105/vision_app'
OUT = ROOT / 'output/source-observer-20261106'


def main():
    proof = json.loads((ROOT / 'evidence/integration/2026-11-06/source-observer/matlab-result.json').read_text(encoding='utf-8'))
    assert proof['passed']
    assert not OUT.exists(), 'Preserve any previous delivery'
    app = OUT / 'vision_app'
    shutil.copytree(BASE, app, ignore=shutil.ignore_patterns('__pycache__', 'diagnostics', '*.before-*'))
    overlays = ['code/source_observe.py', 'code/source_metric_preview.py', 'code/source_metric_bridge.py', 'code/metric_alignment_protocol.py',
                'code/config/source_metric.unconfigured.json']
    for name in overlays:
        p = app / name
        p.parent.mkdir(parents=True, exist_ok=True)
        shutil.copy2(ROOT / 'vision_app' / name, p)
    p = app / 'run.py'
    text = p.read_text(encoding='utf-8')
    assert "'source-observe'" not in text
    p.write_text(text.replace("'main':'main.py',", "'main':'main.py',\n    'source-observe':'source_observe.py',"), encoding='utf-8')
    # Existing main/HSV/region detector code remains byte-identical to the
    # current source-workflow bundle. No surprise default A/M changes.
    for name in ['code/main.py', 'code/turntable_detector.py', 'code/material_pickup_detector.py',
                 'code/颜色标定.py', 'code/物料抓取区域标定.py']:
        assert (BASE / name).read_bytes() == (app / name).read_bytes()
    shutil.copy2(ROOT / 'scripts/install_source_observer_20261106.py', OUT / 'install.py')
    hashes = {p.relative_to(app).as_posix(): hashlib.sha256(p.read_bytes()).hexdigest()
              for p in sorted(app.rglob('*')) if p.is_file()}
    (OUT / 'payload-manifest.json').write_text(json.dumps({'sha256': hashes}, ensure_ascii=False, indent=2), encoding='utf-8')
    (OUT / '晚点接续.md').write_text('''# 充电后先恢复观察

先确认电量和香橙派网络，回报hostname -I。不要直接按STM32 PC13；原板仍是升降往返测试版，重上电会丢失旧参考。

在香橙派确认原vision_app实际位置，退出占用同一摄像头的预览，再安装到新目录：

    python3 install.py --base /实际旧目录/vision_app --destination /新目录/vision_app

安装会校验包，继承旧目录code/config下的现有JSON/YAML，原目录保持，目标已存在会拒绝。不要用包内占位配置覆盖现场HSV/盘面参数。

先检查（不打开相机/串口）：

    python3 /新目录/vision_app/run.py --feature source-observe --check

再采集30秒A模式，只打开相机，不打开串口/电机：

    python3 /新目录/vision_app/run.py --feature source-observe --mode A --duration 30

需要物料观察时改--mode M，仍不抓放。原主程序A/M及原调参工具都保留。

每次生成新diagnostics/source-observe日期目录，含observations.jsonl、first-raw.png、last-raw.png、last-overlay.png和session.json；原始帧不带叠加。--session-dir可显式指定一个不存在的新目录，结果不覆盖旧证据。主机读帧时间不当成曝光时刻。

毫米预览可加--metric-profile与--context，两者必须对应实际同姿态/分辨率/平面；包内未配置模板不含已验标志，不生成车体动作。盘面映射不套物料顶面，不能用拟合图像圆心直接算物理盘心。

本包不发送串口、不改摄像头策略/HSV，不安装或烧录STM32。需要真实车道/三位置/净距和毫米标定证据后，另接执行器；当前只准备观察。
''', encoding='utf-8')
    print(json.dumps({'bundle': str(OUT), 'payload_files': len(hashes), 'deployed': False}, ensure_ascii=False))


if __name__ == '__main__':
    main()
