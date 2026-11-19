"""Record one manually placed ring using one fresh DM read and user ruler X.

The diagnostic read can retain an existing IT replay error. It changes no
motion, health, fault, reference or motor-enable state.
"""
import argparse
from datetime import datetime
import json
import math
from pathlib import Path
import struct
import time

import stage3_live_20261116 as live
import stage3_resume_status_20261118 as status

CONFIG=status.ROOT/'calibration/stage3_roughing_ring_teach_20261118.json'
MANIFEST=status.OUT/'teach-return-fix-compatible.json'


def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--read-angle',action='store_true')
    parser.add_argument('--ring',type=int,choices=(1,2,3),required=True)
    parser.add_argument('--extension-mm',type=float,required=True)
    args=parser.parse_args()
    assert math.isfinite(args.extension_mm) and 0<=args.extension_mm<=170
    image=live.manifest(MANIFEST)
    if not args.read_angle:
        print(json.dumps({'local_only':True,'ring':args.ring,'user_extension_mm':args.extension_mm}));return
    data=json.loads(CONFIG.read_text(encoding='utf-8'))
    assert data['ring_points'][str(args.ring)]['dm_yaw_deg'] is None,'Point already recorded'
    import gripper210_attach_read_20261105 as elf
    from source_body_probe_20261111 import attach
    from smart_carrier.vision_manipulation.committed_pick_board import CommittedPickBoard
    from elftools.elf.elffile import ELFFile
    axf=Path(image['axf']);elf.IMAGE=axf;symbols,segments=elf.load_image()
    with axf.open('rb') as stream:
        file=ELFFile(stream)
        objects={s.name:int(s['st_value']) for s in file.get_section_by_name('.symtab').iter_symbols() if s['st_info']['type']=='STT_OBJECT'}
    report={'firmware_sha256':image['sha256'],'ring':args.ring,'extension_mm':args.extension_mm,
            'motion_requests':0,'fault_writes':0,'enable_writes':0,'read_requests':0}
    path=status.OUT/('manual-ring-'+str(args.ring)+'-read-'+datetime.now().strftime('%Y%m%d-%H%M%S-%f')+'.json')
    try:
        with attach() as session:
            target=session.target
            board=CommittedPickBoard(target,symbols,segments)
            report['verified_flash_bytes']=board.verified_flash_bytes
            def sample():
                result=board.snapshot()
                for name in ('g_stage3_manual_state','g_stage3_manual_x','g_it_cause','g_it_state',
                             'g_dm_can2_read_request','g_dm_can2_state','g_dm_can2_result',
                             'g_dm_can2_feedback_valid','g_dm_can2_feedback_tick','g_dm_can2_read_cycle',
                             'g_dm_can2_motor_state','g_dm_posvel_holding'):
                    result[name]=target.read32(objects[name])
                for name in ('g_dm_can2_logical_position_deg','g_dm_can2_position_deg'):
                    result[name]=struct.unpack('<f',struct.pack('<I',target.read32(objects[name])))[0]
                return result
            before=sample();report['before']=before
            assert before['g_stage3_manual_state'] in (3,5) and before['g_stage3_manual_x']==2
            assert before['g_dm_can2_motor_state']==0 and not before['g_dm_posvel_holding']
            assert not before['g_dm_can2_read_request'] and before['g_low_wheel_hold_ready']==1
            assert before['wheel_hold_results']==[1,1,1,1] and not before['g_chassis_auto_active']
            assert not before['g_lift_ruler_stop_pending'] and not before['g_lift_ruler_cause']
            target.write32(objects['g_dm_can2_read_request'],2);report['read_requests']=1
            deadline=time.monotonic()+6
            while True:
                after=sample();report['after']=after
                fresh=after['g_dm_can2_read_cycle']!=before['g_dm_can2_read_cycle']
                if fresh and after['g_dm_can2_state']==5:raise RuntimeError('DM diagnostic read failed')
                if fresh and after['g_dm_can2_state']==4 and after['g_dm_can2_result']==1:
                    assert after['g_dm_can2_feedback_valid']==1
                    assert after['g_dm_can2_feedback_tick']!=before['g_dm_can2_feedback_tick']
                    assert after['g_dm_can2_motor_state']==0 and not after['g_dm_posvel_holding']
                    assert after['g_it_cause']==before['g_it_cause']
                    break
                if time.monotonic()>=deadline:raise TimeoutError('Fresh diagnostic read not completed')
                time.sleep(.15)
            angle=after['g_dm_can2_logical_position_deg']
            assert math.isfinite(angle) and 0<=angle<=270
            report['completed']=True
        path.write_text(json.dumps(report,indent=2),encoding='utf-8')
        data['ring_points'][str(args.ring)]={
            'status':'measured_awaiting_replay','dm_yaw_deg':angle,'extension_mm':args.extension_mm,
            'extension_source':'user ruler from full retraction zero','angle_evidence':str(path),
            'angle_read_cycle':after['g_dm_can2_read_cycle'],
            'raw_dm_yaw_deg':after['g_dm_can2_position_deg'],'measured_at':datetime.now().isoformat(),
            'existing_it_cause_preserved':after['g_it_cause']}
        data['current_ring']=min(args.ring+1,3)
        backup=CONFIG.with_name(CONFIG.stem+'-before-ring'+str(args.ring)+'.json')
        with backup.open('x',encoding='utf-8') as stream:stream.write(CONFIG.read_text(encoding='utf-8'))
        temp=CONFIG.with_suffix('.json.new')
        with temp.open('x',encoding='utf-8') as stream:json.dump(data,stream,ensure_ascii=False,indent=2)
        temp.replace(CONFIG)
        print(json.dumps({'saved':str(CONFIG),'ring':args.ring,'dm_yaw_deg':angle,'extension_mm':args.extension_mm,'evidence':str(path)}))
    except Exception as error:
        report['error']=repr(error)
        if not path.exists():path.write_text(json.dumps(report,indent=2),encoding='utf-8')
        raise


if __name__=='__main__':main()
