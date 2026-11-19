"""Reuse the read-only verified-image monitor for the stopped3 candidate."""
from pathlib import Path
import integration23_live_20261115 as monitor
monitor.E=Path(__file__).resolve().parents[1]/'evidence/integration/2026-11-15/stopped3-grip170-release200'
if __name__=='__main__':monitor.main()
