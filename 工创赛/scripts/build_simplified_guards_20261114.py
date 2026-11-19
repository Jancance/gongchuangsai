"""Local-only simplified release build. No attach/reset/deployment."""
from pathlib import Path
import json,subprocess
import build_overlap200_commit_20261114 as build
R=Path(__file__).resolve().parents[1]
P=R/'tmp/source_top_simplified_20261114/project'
E=R/'evidence/integration/2026-11-14/overlap200-simplified'

def main():
    assert json.loads((E/'simplification-matlab.json').read_text())['passed']
    subprocess.run(['gcc','-std=c99','-Wno-misleading-indentation','-I'+str(P/'CHASSIS/USER'),
      str(R/'tests/simplified_parallel_group_20261114.c'),str(P/'CHASSIS/USER/source_parallel_flow.c'),
      '-o',str(E/'simplified-group.exe')],check=True)
    result=subprocess.run([str(E/'simplified-group.exe')],check=True,capture_output=True,text=True)
    print(result.stdout,flush=True);(E/'simplified-group.log').write_text(result.stdout,encoding='utf-8')
    build.P=P;build.E=E;build.ALLOW_READ_RETRY_CHANGE=True
    build.run()

if __name__=='__main__':main()
