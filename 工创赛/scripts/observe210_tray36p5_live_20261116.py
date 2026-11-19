"""Read-only verified image and cycle monitor for observation210/tray36.5."""
from pathlib import Path
import stopped5_fastalign_live_20261116
import integration23_live_20261115 as monitor
monitor.E=Path(__file__).resolve().parents[1]/'evidence/integration/2026-11-16/observe210-tray36p5'
if __name__=='__main__':monitor.main()
