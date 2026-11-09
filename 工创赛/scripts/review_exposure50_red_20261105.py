from pathlib import Path
import zipfile,json,collections,hashlib
E=Path(__file__).resolve().parents[1]/'evidence/p2-camera/2026-11-05/exposure50-red-audit'
zpath=E/'20261105-141433-687019.zip';z=zipfile.ZipFile(zpath);assert z.testzip() is None
r=[json.loads(s) for s in z.read('observations.jsonl').decode().splitlines()]
fresh=[s for s in r if s['state']!='STALE_RESULT'];stale=[s for s in r if s['state']=='STALE_RESULT']
assert all({'blue','yellow'}<=set(t['color'] for t in s['targets']) for s in fresh)
assert not any(t['color']=='red' for s in r for t in s['targets'])
boxes=[t['bbox_px'] for s in r for t in s['targets'] if t['color']=='lightblue']
report=dict(zip_bytes=zpath.stat().st_size,sha256=hashlib.sha256(zpath.read_bytes()).hexdigest(),frames=len(r),fresh_blue_and_yellow=len(fresh),stale_empty=len(stale),stale_age_ms_range=[min(s['result_host_age_ms'] for s in stale),max(s['result_host_age_ms'] for s in stale)],target_combinations={','.join(k):v for k,v in collections.Counter(tuple(sorted(t['color'] for t in s['targets'])) for s in r).items()},extra_lightblue_bbox_x_range=[min(b[0] for b in boxes),max(b[0] for b in boxes)],red_detected_frames=0,red_lower_border_rejections=sum(1 for s in r for t in s.get('detector_regions',[]) if t['reason']=='TOUCHES_BORDER' and t['bbox_px'][0]>700 and t['bbox_px'][0]<1200 and t['bbox_px'][1]>700),hardware_commands_sent=False)
(E/'review.json').write_text(json.dumps(report,ensure_ascii=False,indent=2),encoding='utf-8');print(json.dumps(report,indent=2))
