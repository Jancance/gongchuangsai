"""Offline summary of actual saved encoder/profile evidence; no hardware imports."""
from pathlib import Path
from datetime import datetime
import argparse
import hashlib
import json
import math
import re
import struct
import sys

ROOT = Path(__file__).resolve().parents[1]
PROJECTION = ((1,1,1,1),(1,-1,-1,1),(-1,1,1,-1),(-1,-1,-1,-1))


def image_geometry(image):
    sys.path.insert(0,str(ROOT/'tmp/gripper_debug_deps'))
    from elftools.elf.elffile import ELFFile
    path = ROOT/image['axf']
    assert hashlib.sha256(path.read_bytes()).hexdigest() == image['sha256']
    with path.open('rb') as stream:
        elf=ELFFile(stream)
        symbols={s.name:s for s in elf.get_section_by_name('.symtab').iter_symbols()
                 if s.name in ('g_chassis_geometry','g_chassis_forward_direction')}
        def data(name):
            symbol=symbols[name];section=elf.get_section(symbol['st_shndx'])
            offset=int(symbol['st_value'])-int(section['sh_addr'])
            return section.data()[offset:offset+int(symbol['st_size'])]
        geometry=struct.unpack('<3f',data('g_chassis_geometry'))
        directions=list(data('g_chassis_forward_direction'))
    project=path.parents[2]
    header=(project/'CHASSIS/USER/chassis_kinematics.h').read_text(encoding='gbk')
    scale=float(re.search(r'#define CHASSIS_FORWARD_DISTANCE_SCALE ([0-9.]+)f',header)[1])
    return dict(wheel_diameter_mm=geometry[0],track_width_mm=geometry[1],wheelbase_mm=geometry[2],
                forward_direction=directions,forward_scale_source_header=scale,
                scope='ELF geometry/directions; existing source forward scale, not independent physical ruler truth')


def endpoints(evidence,geometry):
    words=evidence['raw_words']['s_endpoints'];records=[]
    for i in range(min(evidence['s_endpoint_count'],len(words)//30)):
        row=words[i*30:(i+1)*30]
        record=dict(index=i,session=row[0],outcome=row[1],start_mask=row[2],end_mask=row[3],
                    direction=row[4],distance_mode=row[5],start_sign=row[6:10],start_raw=row[10:14],
                    start_ms=row[14:18],end_sign=row[18:22],end_raw=row[22:26],end_ms=row[26:30])
        if row[2]==15 and row[3]==15 and row[4] in range(4):
            delta=[(-r if s else r)-(-a if b else a) for r,s,a,b in
                   zip(record['end_raw'],record['end_sign'],record['start_raw'],record['start_sign'])]
            wheel=[d*math.pi*geometry['wheel_diameter_mm']/65536*(-1 if reverse else 1)
                   for d,reverse in zip(delta,geometry['forward_direction'])]
            projected=[x*p for x,p in zip(wheel,PROJECTION[row[4]])]
            mean=sum(projected)/4
            scale=geometry['forward_scale_source_header'] if row[4] in (0,3) else (1 if row[4]==2 else None)
            record.update(signed_encoder_delta_counts=delta,wheel_forward_mm=wheel,
                          projected_wheel_mm=projected,encoder_projection_mean_mm=mean,
                          projected_wheel_spread_mm=max(projected)-min(projected),
                          firmware_scaled_endpoint_mean_mm=None if scale is None else mean*scale,
                          note='Four independently timestamped wheel endpoints; body travel not physically measured')
        records.append(record)
    return records


def analyze(path):
    log=json.loads(path.read_text(encoding='utf-8'));geometry=image_geometry(log['image'])
    before=log.get('before',{}).get('body_distance_evidence')
    after=log.get('after',{}).get('body_distance_evidence')
    assert before is not None and after is not None,'Saved same-connection distance evidence is absent'
    evidence=[s['body_distance_evidence'] for s in log.get('samples',[]) if 'body_distance_evidence' in s]
    session=after['g_chassis_auto_session']
    current=[s for s in evidence if s['g_chassis_auto_session']==session]
    records=endpoints(after,geometry)
    record=next((r for r in reversed(records) if r['session']==session),None)
    timeline=[dict(host_time=s['host_time'],tick_before=s['tick_before'],tick_after=s['tick_after'],
                   same_session_during_read=s['same_session_during_read'],active=s['g_chassis_auto_active'],
                   profile_done=s['s_profile_done'],distance_reached=s['s_distance_reached'],
                   target_mm=s['s_distance_target_mm'],progress_mm=s['s_distance_progress_mm'],
                   profile_reference=s['profile_reference'],base_rpm=s['s_translation_base_rpm'],
                   heading_deg=s['g_hwt_yaw_deg']) for s in current]
    intervals=[(datetime.fromisoformat(b['host_time'])-datetime.fromisoformat(a['host_time'])).total_seconds()
               for a,b in zip(evidence,evidence[1:])]
    heading=(after['g_hwt_yaw_deg']-before['g_hwt_yaw_deg']+180)%360-180
    result=dict(input=str(path),action=log.get('action'),image=log['image'],geometry=geometry,
                completed=log.get('completed'),session=session,
                new_session=before['g_chassis_auto_session']!=session,
                heading_change_deg=heading,heading_source='actual saved HWT before/after, not image line angle',
                final_reported_progress_mm=after['s_distance_progress_mm'],
                target_mm=after['s_distance_target_mm'],endpoint=record,
                logged_sample_intervals_s=intervals,timeline=timeline,
                existing_stop_condition='mean >= target, or profile_done and mean >= target - 2mm',
                camera_capture_interval=None,
                limitations=['Encoder projection is wheel travel, not ruler-measured body displacement.',
                             'Four-wheel raw endpoints and snapshots are asynchronous; saved timestamps remain authoritative.',
                             'Sampled progress can precede final endpoint motion; no invented stop trigger time.',
                             'Camera acquisition times are not saved by this MCU log; do not infer frame freshness.'])
    if not result['new_session']:
        result['interpretation']='No new motion session in this record; baseline/layout only.'
    return result


def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('log',type=Path);parser.add_argument('--output',type=Path)
    args=parser.parse_args();report=analyze(args.log)
    text=json.dumps(report,ensure_ascii=False,indent=2)
    if args.output:
        args.output.parent.mkdir(parents=True,exist_ok=True)
        args.output.write_text(text,encoding='utf-8')
    else:print(text)


if __name__=='__main__':main()
