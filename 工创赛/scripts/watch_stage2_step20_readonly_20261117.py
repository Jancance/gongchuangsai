"""Use the reviewed source watcher with only the step20 manifest/evidence target."""
from pathlib import Path
import watch_stage2_debug215_readonly_20261117 as prior
R=Path(__file__).resolve().parents[1]
if __name__=='__main__':
 prior.E=R/'evidence/integration/2026-11-17/stage2-debug215-step20'
 prior.main()
