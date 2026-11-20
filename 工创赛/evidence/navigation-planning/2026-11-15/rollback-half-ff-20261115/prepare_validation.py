from pathlib import Path
E=Path(__file__).resolve().parent;R=Path.cwd()
H=R/'工创赛/evidence/navigation-planning/2026-11-14/baseline40-field/field-candidate-conservative-ff'
s=(H/'check_actual_entry.py').read_text('utf8')
s=s.replace("E=R/'工创赛/evidence/navigation-planning/2026-11-14/baseline40-field/field-candidate-conservative-ff';T=R/'工创赛/tmp/field-candidate-conservative-ff-c-check'","E=Path(__file__).resolve().parent;T=R/'工创赛/tmp/rollback-half-ff-c-check-20261115'")
(E/'check_actual_entry.py').write_text(s,'utf8')
b=(E.parent/'float-timing-20261115/build_isolated.py').read_text('utf8')
b=b.replace("E=R/'工创赛/evidence/navigation-planning/2026-11-15/float-timing-20261115'","E=Path(__file__).resolve().parent").replace('工创赛/tmp/float-timing-build-20261115','工创赛/tmp/rollback-half-ff-build-20261115')
(E/'build_isolated.py').write_text(b,'utf8')
