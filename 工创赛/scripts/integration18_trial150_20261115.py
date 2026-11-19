"""Select the150RPM candidate; default local check, no automatic retry."""
from pathlib import Path
import integration18_trial_20261115 as field

field.CANDIDATE_EVIDENCE = Path(__file__).resolve().parents[1] / "evidence/integration/2026-11-15/integration18/ext150-clog3000"
field.FIELD_EVIDENCE = field.CANDIDATE_EVIDENCE

if __name__ == "__main__":
    field.main()
