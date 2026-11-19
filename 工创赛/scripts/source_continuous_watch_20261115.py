"""Read-only monitor for the continuous-disc candidate, default local check."""
from pathlib import Path
import source_mcu_cycle_watch_20261115 as monitor
monitor.E=Path(__file__).resolve().parents[1]/'evidence/integration/2026-11-15/source-continuous'
if __name__=='__main__':monitor.main()
