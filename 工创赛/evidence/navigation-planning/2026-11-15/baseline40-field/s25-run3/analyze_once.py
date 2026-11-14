"""Analyze this completed run only; no hardware or motion commands."""
from pathlib import Path
import hashlib
import importlib.util
import json

OUT = Path(__file__).resolve().parent
original = OUT.parent / 'bounded-run4-retry1/analyze_capture.py'
spec = importlib.util.spec_from_file_location('s25_completed_capture_analysis', original)
module = importlib.util.module_from_spec(spec)
spec.loader.exec_module(module)
module.OUT = OUT
result = module.analyze(1)
result['controller'] = dict(M=1.5, Pscale_mm_s=2.5, Ky=0.75)
result['reused_analyzer_sha256'] = hashlib.sha256(original.read_bytes()).hexdigest()
(OUT / 'capture_check.json').write_text(json.dumps(result, ensure_ascii=False, indent=2), encoding='utf8')
print(json.dumps({key: result[key] for key in [
    'middle_sample_peak_abs_mm', 'middle_samples', 'lateral_peak_rpm',
    'lateral_cap30_complete_batches', 'hwt_start_deg', 'hwt_end_deg',
    'hwt_delta_deg', 'pre_stop_left_from_start_deg', 'xd1',
    'post_complete_tx', 'complete_batches',
]}, ensure_ascii=True))
