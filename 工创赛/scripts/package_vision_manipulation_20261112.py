"""Package the vision module only; never ship old HSV or a live motor backend."""
from pathlib import Path
import hashlib,json,shutil,subprocess,sys,zipfile
R=Path(__file__).resolve().parents[1]
E=R/'evidence/integration/2026-11-12/vision-modules'
P=R/'output/vision_manipulation_module_20261112'

def main():
    model=json.loads((E/'matlab-result.json').read_text(encoding='utf-8'));assert model['passed']
    assert not P.exists(),'Preserve an existing delivery'
    P.mkdir()
    files=[R/'src/smart_carrier/__init__.py',R/'src/smart_carrier/task_code.py',R/'src/smart_carrier/task_materials.py',
           *sorted((R/'src/smart_carrier/vision_manipulation').glob('*.py')),
           R/'config/vision_manipulation_20261112.json',R/'config/task_materials_preliminary_2027.json',
           R/'tests/test_vision_manipulation.py',R/'docs/VISION_MANIPULATION_MODULES_2026-11-12.md',
           E/'matlab-result.json',R/'evidence/integration/2026-11-11/source-view-entry/pose-current-20261112/matlab-result.json']
    for f in files:
        dest=P/f.relative_to(R);dest.parent.mkdir(parents=True,exist_ok=True);shutil.copy2(f,dest)
    (P/'start.py').write_text('from pathlib import Path\nimport sys\nsys.path.insert(0,str(Path(__file__).resolve().parent/"src"))\n\nif __name__=="__main__":\n    from smart_carrier.vision_manipulation.workbench import main\n    raise SystemExit(main())\n',encoding='utf-8')
    (P/'README.md').write_text('# 视觉模块离线交付\n\n默认只打印计划，不开相机/串口/执行器。见docs/VISION_MANIPULATION_MODULES_2026-11-12.md。\n\n`python3 start.py`\n\n`python3 start.py --simulate`仅合成回放。`--preview --zone rough`仅VNC观察。现场配置需先补示教，不自动覆盖现有HSV/ROI。\n',encoding='utf-8')
    # Test exactly the delivered module tree, not only the workspace source.
    run=subprocess.run([sys.executable,str(P/'tests/test_vision_manipulation.py')],capture_output=True,text=True)
    (E/'delivery-tests.log').write_text(run.stdout+run.stderr,encoding='utf-8');assert run.returncode==0,run.stdout+run.stderr
    run=subprocess.run([sys.executable,str(P/'start.py'),'--simulate'],capture_output=True,text=True)
    assert run.returncode==0,run.stdout+run.stderr
    simulation=json.loads(run.stdout);assert simulation['synthetic'] and not simulation['hardware_access']
    (E/'simulation-summary.json').write_text(json.dumps(simulation,ensure_ascii=False,indent=2),encoding='utf-8')
    manifest={f.relative_to(P).as_posix():hashlib.sha256(f.read_bytes()).hexdigest() for f in P.rglob('*') if f.is_file() and '__pycache__' not in f.parts}
    (P/'manifest.json').write_text(json.dumps(dict(files=manifest,actuator_backend_registered=False,hardware_enabled=False),indent=2),encoding='utf-8')
    z=P.with_suffix('.zip');assert not z.exists()
    with zipfile.ZipFile(z,'w',zipfile.ZIP_DEFLATED) as archive:
        for rel in [*manifest,'manifest.json']:archive.write(P/rel,rel)
    with zipfile.ZipFile(z) as archive:
        assert archive.testzip() is None
        for rel,digest in manifest.items():assert hashlib.sha256(archive.read(rel)).hexdigest()==digest
    result=dict(passed=True,matlab_checks=model['check_count'],package_tests_passed=True,
                synthetic_jobs=len(simulation['jobs']),archive=z.relative_to(R).as_posix(),archive_sha256=hashlib.sha256(z.read_bytes()).hexdigest(),
                file_count=len(manifest)+1,hardware_access=False,uploaded=False)
    (E/'delivery.json').write_text(json.dumps(result,indent=2),encoding='utf-8');print(json.dumps(result,indent=2))

if __name__=='__main__':main()
