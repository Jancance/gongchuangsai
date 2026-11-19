"""Explicit read/preparation tools for10mm candidate; default local check."""
from pathlib import Path
import integration18_candidate_tools_20261115 as tool
R=Path(__file__).resolve().parents[1]
tool.E=R/'evidence/integration/2026-11-15/integration20-yaw165-window10'
tool.P=R/'tmp/source_top_window10_yaw165_20261115/project'
if __name__=='__main__':tool.main()
