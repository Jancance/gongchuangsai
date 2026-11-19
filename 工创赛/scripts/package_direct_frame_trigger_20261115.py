"""Prepare the authorized direct2frame Pi trial without prediction code."""
from pathlib import Path
import ast
import hashlib
import json
import shutil
import zipfile

R=Path(__file__).resolve().parents[1]
E=R/'evidence/integration/2026-11-15/direct-frame-trigger'
P=R/'output/source_direct_frame_vehicle_20261115'


def main():
    proof=json.loads((E/'matlab-result.json').read_text());assert proof['passed'] and proof['local_matlab']
    assert not P.exists()
    shutil.copytree(R/'output/source_extension_lead_vehicle_20261115',P,ignore=shutil.ignore_patterns('__pycache__'))
    shutil.copyfile(R/'output/direct_frame_trigger_20261115.py',P/'direct_frame_trigger_20261115.py')
    path=P/'source_fast_cycle_view_20261115.py';s=path.read_text()
    def change(a,b):
        nonlocal s
        assert s.count(a)==1,a
        s=s.replace(a,b,1)
    change('from smart_carrier.vision_manipulation.fast_turntable import FastTurntable','from direct_frame_trigger_20261115 import DirectFrameTrigger')
    change('predictor=FastTurntable(orbit_config) if args.continuous_pick else None','predictor=None\n    direct=DirectFrameTrigger(pickup_config)')
    change('context=key;frame_count=0;sequence=0;gate.reset()','context=key;frame_count=0;sequence=0;gate.reset();direct.reset()')
    change("prediction=predictor.update(value['regions'],observed,time.monotonic(),\n                    status['colors'][status['item']],status['colors'][status['item']:])",
           "prediction=direct.update(value,observed,status['colors'][status['item']],status['colors'][status['item']:])\n                if prediction is not None:colors=prediction['colors']")
    change("delays=predictor.delays_at_send(prediction,time.monotonic(),wire_bytes=0)","delays=prediction['start_delay_ms']")
    change("delays=predictor.delays_at_send(prediction,time.monotonic(),wire_bytes=len(wire))","delays=prediction['start_delay_ms']")
    change("'prediction':", "'prediction':") if False else None
    change('prediction=prediction,continuous_pick=args.continuous_pick,','direct_frame_trigger=prediction,prediction_used=False,continuous_pick=args.continuous_pick,')
    ast.parse(s);path.write_text(s,encoding='utf-8')
    for name in ('start.sh','start-trial.sh'):
        f=P/name;f.write_text(f.read_text().replace('/source_extension_lead_vehicle_20261115','/source_direct_frame_vehicle_20261115'),encoding='utf-8',newline='\n')
    manifest={f.relative_to(P).as_posix():hashlib.sha256(f.read_bytes()).hexdigest() for f in P.rglob('*') if f.is_file() and f.name!='manifest.json' and '__pycache__' not in f.parts}
    (P/'manifest.json').write_text(json.dumps(manifest,indent=2)+'\n')
    with zipfile.ZipFile(R/'output/source_direct_frame_vehicle_20261115.zip','w',zipfile.ZIP_DEFLATED) as z:
        for name in [*manifest,'manifest.json']:z.write(P/name,name)
    print('Direct2frame package prepared; no prediction call, selected region delay0')


if __name__=='__main__':main()
