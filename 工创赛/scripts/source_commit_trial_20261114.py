"""Reviewed single-commit stage2 entry. Default is a LOCAL CHECK, never hardware.

Execution is separate: --align-only positions first; --once-batch runs3pieces.
After a valid2-frame decision the MCU completes the piece without further vision.
The camera is used again only after return, for body positioning and the next item.
"""
import argparse
import hashlib
import json
import sys
import time
from datetime import datetime
from pathlib import Path

ROOT=Path(__file__).resolve().parents[1]
sys.path.insert(0,str(ROOT/'src'))
from smart_carrier.vision_manipulation.committed_pick_board import CommittedPickBoard,StartOutcomeUnknown
from smart_carrier.vision_manipulation.low_pick_board import LowPickTarget
from smart_carrier.vision_manipulation.source_body_registration import load_profile,next_existing_bounded_step
from smart_carrier.task_materials import resolve_preliminary_task
from source_top_trial_20261114 import CameraPeer

E=ROOT/'evidence/integration/2026-11-14/overlap200-commit'
REMOTE='/home/orangepi/vision_tools/'

class ControllerFault(RuntimeError):
    def __init__(self,message,state):super().__init__(message);self.state=state
class ObservationUnavailable(RuntimeError):pass

def manifest():
    candidate=json.loads((E/'candidate.json').read_text(encoding='utf-8'))
    assert candidate['build_passed']
    for name in ('matlab-result.json','body-registration-matlab.json','wheel-hold-rearm-matlab.json'):
        assert json.loads((E/name).read_text())['passed']
    for name,digest in candidate['changes'].items():
        assert hashlib.sha256((ROOT/candidate['project']/name).read_bytes()).hexdigest()==digest,name
    image=candidate['images']['Vehicle_LiftRuler']
    assert hashlib.sha256((ROOT/image['axf']).read_bytes()).hexdigest()==image['sha256']
    geometry=json.loads((ROOT/'evidence/integration/2026-11-14/top-pick-pb9/parking-current.json').read_text())
    poses=list(zip(geometry['current_arm_angles_deg'],geometry['current_extensions_mm']));poses[2]=(180.0,0.0)
    profile=load_profile(ROOT/'output/source_body_registration_20261114.json')
    return image,poses,profile

def checked_state(board,allow_body=False):
    state=board.snapshot()
    health=dict(state)
    if allow_body:health['g_chassis_auto_active']=0
    if (not board._healthy(health) or state['g_low_state']==9 or
        state['g_chassis_auto_cause'] or state['g_source_body_step_reason']):
        raise ControllerFault('主控/驱动真实故障，保留原STOP处理',state)
    return state

class Vision:
    def __init__(self,port,session,report):self.port=port;self.session=session;self.report=report;self.peer=None
    def exchange(self,state,pose_id,sequence,remaining):
        control=dict(session_id=self.session,state=state,pose_id=pose_id,pose_sequence=sequence,
          remaining_colors=list(remaining),material_pickup_config=REMOTE+'material_pickup_top65_20261114.json')
        try:
            if self.peer is None:self.peer=CameraPeer(self.port)
            value=self.peer.exchange(control)
            if value is not None:self.report['observations'].append(value)
            return value
        except (OSError,ConnectionError,ValueError) as error:
            self.report['camera_link_events'].append(str(error))
            self.close();return None
    def inactive(self,sequence,remaining):
        # Best effort only. Camera loss does not cancel an accepted controller piece.
        self.exchange('MOVING','committed-motion',sequence,remaining)
    def close(self):
        if self.peer is not None:
            try:self.peer.close()
            except OSError:pass
        self.peer=None

def fresh(value,pose,sequence):
    return (value is not None and value.get('pose_active') and value.get('pose_id')==pose and
      value.get('pose_sequence')==sequence and value.get('frame_after_pose_ack') and
      0<=value.get('age_s',999)<=.20)

def ensure_wheel_hold(board,deadline,report):
    board.request_wheel_hold()
    while time.monotonic()<deadline:
        state=checked_state(board)
        if state['g_low_wheel_hold_ready']==1:
            assert state['wheel_hold_results']==[1]*4
            report['wheel_holds'].append(dict(cycle=state['g_low_wheel_hold_cycle'],results=state['wheel_hold_results']))
            return
        if state['g_low_wheel_hold_stage']==99:raise ObservationUnavailable('锁轮请求未成功，尚未启动本件')
        time.sleep(.04)
    raise ObservationUnavailable('调试预算结束，保持等待姿态，尚未启动下一件')

def record_stop_ack(board,report,body=False):
    for _ in range(12):
        try:
            state=board.snapshot();report['after_stop_request']=state
            if body:
                confirmed=not state['g_chassis_auto_active'] and state['body_stop_masks'][2]==15
            else:
                confirmed=(state['g_it_state']==9 and not state['g_it_request'] and
                  not any(state[k] for k in ('g_lift_ruler_stop_pending','g_vehicle_arm_stop_pending','g_source_parallel_stop_mask')))
            if confirmed:report['stop_protocol_confirmed']=True;return
        except BaseException as error:report['stop_read_error']=str(error);return
        time.sleep(.15)

def position_body(board,vision,remaining,profile,deadline,report,save):
    moved=False
    while time.monotonic()<deadline:
        state=checked_state(board)
        if not board.pose_ready(state):time.sleep(.04);continue
        seq=state['g_source_align_command_seq'];pose='observer-align-top65'
        value=vision.exchange('ALIGN',pose,seq,remaining)
        body=value.get('body') if fresh(value,pose,seq) else None
        if not body or not body.get('available') or body.get('profile_id')!=profile['profile_id']:
            time.sleep(.04);continue
        report['body_observations'].append(body)
        step=next_existing_bounded_step(body,profile)
        if step is None:
            if moved or state['g_source_align_chain_active']:
                board.finish_body_alignment()
                while time.monotonic()<deadline:
                    state=checked_state(board)
                    if not state['g_source_align_chain_active'] and not state['g_source_body_step_request']:break
                    time.sleep(.04)
            ensure_wheel_hold(board,deadline,report);save();return
        vision.inactive(seq,remaining)
        old=board.begin_body_step(step['direction'],step['distance_mm'])
        record=dict(**step,before=body);report['body_steps'].append(record);save()
        while time.monotonic()<deadline:
            state=checked_state(board,allow_body=True)
            if (state['g_source_align_command_seq']!=old and state['g_source_body_step_state']==3 and
                not state['g_chassis_auto_active'] and state['g_chassis_auto_result']==1 and
                state['body_stop_masks'][2]==15):
                record['completed_stop_masks']=state['body_stop_masks'];moved=True;break
            time.sleep(.04)
        else:raise ObservationUnavailable('底盘阶段未完成，记录当前真实状态')
    raise ObservationUnavailable('未取得可用固定盘缘位置；未发下一件抓取')

def run(args):
    image,poses,profile=manifest()
    import gripper210_attach_read_20261105 as live
    from source_body_probe_20261111 import attach
    live.IMAGE=ROOT/image['axf'];symbols,segments=live.load_image()
    plan=resolve_preliminary_task(args.task_code,1,ROOT/'config/task_materials_preliminary_2027.json')
    colors=[x.color_code for x in plan.batch_1]
    out=E/('committed-'+datetime.now().strftime('%Y%m%d-%H%M%S')+'.json')
    report=dict(complete=False,align_only=args.align_only,task_code=args.task_code,
      image_sha256=image['sha256'],flashed=False,reset=False,physical_placement_pending=True,
      formal_inventory_updated=False,observations=[],camera_link_events=[],pieces=[],body_observations=[],body_steps=[],wheel_holds=[],trace=[])
    def save():out.write_text(json.dumps(report,ensure_ascii=False,indent=2),encoding='utf-8')
    save();vision=None;board=None;active=False;current=None
    try:
        with attach() as session:
            board=CommittedPickBoard(session.target,symbols,segments)
            report['verified_flash_bytes']=board.verified_flash_bytes
            vision=Vision(args.port,out.stem,report)
            budget=getattr(args,'debug_budget',0)
            deadline=time.monotonic()+budget if budget>0 else float('inf')
            try:
                position_body(board,vision,colors,profile,deadline,report,save)
                if args.align_only:report['complete']=True;return
                for item,color in enumerate(colors):
                    while time.monotonic()<deadline:
                        state=checked_state(board)
                        value=vision.exchange('OBSERVE','observer-commit-top65',state['g_low_done_sequence'],colors[item:])
                        if not fresh(value,'observer-commit-top65',state['g_low_done_sequence']) or not value.get('stationary'):
                            time.sleep(.04);continue
                        regions=[i for i,c in enumerate(value.get('colors',())) if c==color]
                        if len(regions)!=1:time.sleep(.04);continue
                        region=regions[0];yaw,extension=poses[region]
                        sequence=(state['g_low_accepted_sequence']+1)&0xffffffff or 1
                        current=LowPickTarget(sequence,item+1,color,region,yaw,extension,1,1,'observer-commit-top65')
                        try:sent=board.start_from_observation(current,value)
                        except StartOutcomeUnknown as error:
                            report['start_outcome_unknown']=str(error);sent=True
                        if not sent:time.sleep(.04);continue
                        active=True
                        while True:
                            state=checked_state(board)
                            if state['g_low_accepted_sequence']==sequence:break
                            if not state['g_low_request']:
                                active=False;break  # Unstarted rejection only; retry a genuinely fresh decision.
                            time.sleep(.04)
                        if not active:continue
                        entry=dict(target=current.__dict__,decision=value,committed=True)
                        report['pieces'].append(entry);print('COMMITTED',item+1,color,region,flush=True);save()
                        vision.inactive(sequence,colors[item:])
                        # No camera reads/rechecks here. The MCU finishes its own group chain.
                        phase=None
                        while True:
                            state=checked_state(board);report['trace'].append(state)
                            if state['g_low_phase']!=phase:
                                phase=state['g_low_phase'];print('PIECE',item+1,'PHASE',phase,flush=True);save()
                            if state['g_low_state']==2 and state['g_low_done_sequence']==sequence:
                                active=False
                                assert state['g_low_clear_valid']==1 and state['g_low_actual_rise_mm']>65
                                assert state['g_low_release_wait_ms']>=50 and state['g_low_return_overlap_axes']==7
                                entry['command_complete']=state;entry['motion_groups']=board.motion_evidence();save();break
                            time.sleep(.04)
                        break
                    else:raise ObservationUnavailable('预算结束时未提交下一件；保持已完成状态')
                    position_body(board,vision,colors[item+1:],profile,deadline,report,save)
                report['complete']=True
            except KeyboardInterrupt:
                if active:board.cancel()
                else:board.cancel_body()
                report['manual_stop_requested']=True;record_stop_ack(board,report,body=not active);raise
            except ControllerFault as error:
                # These are the actual hardware/controller conditions, not a missing camera.
                report['first_controller_fault']=error.state
                requested=False
                if active:board.cancel();requested=True
                elif board.snapshot().get('g_chassis_auto_active'):board.cancel_body();requested=True
                report['hardware_stop_requested']=requested
                if requested:record_stop_ack(board,report,body=not active)
                raise
            except ObservationUnavailable as error:
                report['awaiting_observation']=str(error)
                report['committed_piece_cancelled_for_vision']=False
            except BaseException:
                # USB/logging/vision failures do not revoke an already accepted piece.
                report['controller_action_may_continue']=active;raise
    except BaseException as error:report['error']=repr(error);raise
    finally:
        if vision is not None:vision.close()
        save();print(out,flush=True)

if __name__=='__main__':
    parser=argparse.ArgumentParser(description=__doc__)
    action=parser.add_mutually_exclusive_group();action.add_argument('--align-only',action='store_true');action.add_argument('--once-batch',action='store_true')
    parser.add_argument('--debug-budget',type=float,default=0,help='0 disables the old total debug deadline')
    parser.add_argument('--task-code',default='215+213+512+321');parser.add_argument('--port',type=int,default=18887)
    args=parser.parse_args()
    if args.align_only or args.once_batch:run(args)
    else:manifest();print('LOCAL CHECK only: no attach, flash/reset, wheel or arm command.')
