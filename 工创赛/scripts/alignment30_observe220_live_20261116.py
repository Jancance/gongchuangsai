"""Read-only verified-image monitor for 30mm/300ms/observation220."""
from pathlib import Path
import stopped5_fastalign_live_20261116
import integration23_live_20261115 as monitor
monitor.E=Path(__file__).resolve().parents[1]/'evidence/integration/2026-11-16/alignment30-release300-observe220'
if __name__=='__main__':monitor.main()
