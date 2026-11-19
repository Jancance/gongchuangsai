"""Candidate inspection. The real workflow starts withPC13, never this tool."""
from pathlib import Path
import integration18_candidate_tools_20261115 as tool
R=Path(__file__).resolve().parents[1]
tool.E=R/'evidence/integration/2026-11-15/source-continuous'
tool.P=R/'tmp/source_mcu_continuous_20261115/project'
if __name__=='__main__':tool.main()
