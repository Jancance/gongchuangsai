"""Integration19 current field geometry; default LOCAL CHECK, explicit motion only.

Reuse the reviewed 5mm extension candidate and latched recognition lifecycle.
Body positioning tolerance is5mm by the user's2026-11-15 decision.
"""
from pathlib import Path

import source_latched_trial_20261115 as latched
import integration18_trial_20261115 as field

ROOT = Path(__file__).resolve().parents[1]
EVIDENCE = ROOT / 'evidence/integration/2026-11-15/integration18/window5-pause'
field.trial = latched
field.CANDIDATE_EVIDENCE = EVIDENCE
field.FIELD_EVIDENCE = EVIDENCE / 'integration19-field'
field.EVIDENCE = field.FIELD_EVIDENCE

if __name__ == '__main__':
    field.main()
