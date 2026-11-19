"""Adapt existing transport to top view and five-frame stable observations."""
from pathlib import Path
R=Path(__file__).resolve().parents[1]

def main():
    text=(R/'output/low_material_stream_20261114.py').read_text(encoding='utf-8')
    text=text.replace('self.history = deque()','self.history = deque(maxlen=5)')
    text=text.replace('        while self.history and self.history[0][0] < timestamp - .15:\n            self.history.popleft()\n','')
    text=text.replace('len(self.history) < 3 or span < .10','len(self.history) < 5 or span <= 0')
    text=text.replace('settled = active and now - profile_since >= .50','settled = active and len(gate.history) >= 5')
    text=text.replace('if settled and motion_seen and stopped and stop_since is None:',
                      'if settled and stopped and stop_since is None:')
    text=text.replace('stopped = stopped and settled and motion_seen','stopped = stopped and settled')
    text=text.replace('LOW VIEW','TOP VIEW').replace('LOW_POSE_CONFIG','TOP_POSE_CONFIG').replace('LOW_MATERIAL_SENT','TOP_MATERIAL_SENT')
    (R/'output/top_material_stream_20261114.py').write_text(text,encoding='utf-8')
    text=(R/'scripts/source_low_trial_20261114.py').read_text(encoding='utf-8')
    text=text.replace("'evidence/integration/2026-11-14/low-pick'","'evidence/integration/2026-11-14/top-pick-lead8'")
    text=text.replace("'evidence/integration/2026-11-14/source-repark-final-20261114-a/matlab-result.json'",
                      "'evidence/integration/2026-11-14/source-top-repark-20261114/matlab-result.json'")
    start=text.index('                        if model.record_stop(')
    end=text.index('                        candidates =',start)
    text=text[:start]+'''                        # User requests immediate current-region selection after five stable frames.
                        # No learning-cycle requirement or predicted slot substitution.
                        predicted_slots = slots
'''+text[end:]
    text=text.replace("observation.get('stationary') and observation.get('motion_seen') and\n                            observation.get('pause_age_s', 999) <= .25 and",
                      "observation.get('stationary') and")
    text=text.replace("f'source-{region}-low'","f'source-{region}-top'")
    text=text.replace("'observer-low65'","'observer-top65'")
    text=text.replace('material_pickup_focus_20261114.json','material_pickup_top_focus_20261114.json')
    text=text.replace('material_pickup_hover65_20261114.json','material_pickup_top65_20261114.json')
    (R/'scripts/source_top_trial_20261114.py').write_text(text,encoding='utf-8')
    # Initially copied ROI is a candidate. Fresh top images must confirm its spatial coverage.
    import json
    for src,dst in [('material_pickup_near65_20261113.json','material_pickup_top65_20261114.json'),
                    ('material_pickup_focus_20261114.json','material_pickup_top_focus_20261114.json')]:
        data=json.loads((R/'output'/src).read_text(encoding='utf-8'))
        data['motion']['required_low_speed_frames']=5
        (R/'output'/dst).write_text(json.dumps(data,ensure_ascii=False,indent=2),encoding='utf-8')
    print('Prepared top-view transport; ROI confirmation still pending current images.')

if __name__=='__main__':main()
