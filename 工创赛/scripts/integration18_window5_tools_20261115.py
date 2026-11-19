"""5mm candidate tools; default check only. Never flash or reset automatically."""
from pathlib import Path
import integration18_candidate_tools_20261115 as tool

R=Path(__file__).resolve().parents[1]
tool.E=R/'evidence/integration/2026-11-15/integration18/window5-pause'
tool.P=R/'tmp/source_top_window5_pause_20261115/project'

if __name__=='__main__':tool.main()
