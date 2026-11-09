from pathlib import Path
R=Path(__file__).resolve().parents[1]
s=(R/'scripts/install_stage12_chain_20261104.py').read_text(encoding='utf-8')
s=s.replace("E=R/'evidence/integration/2026-11-04/stage12-chain'","E=R/'evidence/integration/2026-11-04/restore-integrated'")
s=s.replace("E/'whole/result.json'","E/'after/result.json'").replace("test['cases']==40","test['cases']==80")
exec(compile(s,str(R/'scripts/install_stage12_chain_20261104.py'),'exec'))
