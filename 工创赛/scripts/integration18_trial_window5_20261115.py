"""5mm corridor and paused recognition; local check by default, no automatic retry."""
from pathlib import Path
import source_latched_trial_20261115 as latched
import integration18_trial_20261115 as field

E=Path(__file__).resolve().parents[1]/'evidence/integration/2026-11-15/integration18/window5-pause'
field.trial=latched
field.CANDIDATE_EVIDENCE=E
field.FIELD_EVIDENCE=E

if __name__=='__main__':field.main()
