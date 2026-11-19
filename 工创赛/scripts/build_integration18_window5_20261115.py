"""Verify/build an isolated5mm candidate. No devices, deployment, reset or retry."""
from pathlib import Path
import json
import subprocess
import sys
import check_integration18_ext150_20261115 as check
import build_integration18_ext150_20261115 as build

R=Path(__file__).resolve().parents[1]
P=R/'tmp/source_top_window5_pause_20261115/project'
E=R/'evidence/integration/2026-11-15/integration18/window5-pause'


def main():
    assert json.loads((E/'matlab-result.json').read_text())['passed']
    runner="""import sys,unittest
sys.path[:0]=['tests','scripts','src']
import test_integration18_recognition_pause_20261115 as current
import test_committed_workflow_20261114 as previous
previous.trial=current.trial
previous.stream=current.stream
suite=unittest.TestSuite([unittest.defaultTestLoader.loadTestsFromTestCase(current.RecognitionPauseTests),unittest.defaultTestLoader.loadTestsFromTestCase(previous.CommitTests)])
result=unittest.TextTestRunner(verbosity=2).run(suite)
raise SystemExit(not result.wasSuccessful())
"""
    result=subprocess.run([sys.executable,'-X','utf8','-c',runner],cwd=R,capture_output=True,text=True)
    (E/'recognition-pause-tests.log').write_text(result.stdout+result.stderr,encoding='utf-8')
    print(result.stdout+result.stderr,flush=True);assert result.returncode==0
    check.P=P;check.E=E;check.EXPECTED_CORRIDOR_DEG=5*315.5*360/3200;check.main()
    build.PROJECT=P;build.E=E;build.BASE=R/'tmp/source_top_extension150_20261115/project';build.main()


if __name__=='__main__':main()
