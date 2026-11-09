"""Install final admission candidate with source/build evidence checks; never flash."""
from pathlib import Path
R=Path(__file__).resolve().parents[1]
source=(R/'scripts/install_stage12_chain_20261104.py').read_text(encoding='utf-8')
source=source.replace("E=R/'evidence/integration/2026-11-04/stage12-chain'", "E=R/'evidence/integration/2026-11-04/stage12-admission'")
source=source.replace("E/'whole/result.json'", "E/'after/result.json'").replace("test['cases']==40", "test['cases']==60")
exec(compile(source,str(R/'scripts/install_stage12_chain_20261104.py'),'exec'))
