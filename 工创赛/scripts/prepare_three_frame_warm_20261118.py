"""MATLAB-gated, isolated three-frame visual warmup and initial XY overlap."""
from pathlib import Path
import json,shutil
R=Path(__file__).resolve().parents[1]
BASE=R/'tmp/home_write_gap_20261118/project'
P=R/'tmp/three_frame_warm_20261118/project'
PI=R/'output/three_frame_warm_overlay_20261118'
PIBASE=R/'output/stage23_settle300_5x2_20261117'
E=R/'evidence/integration/2026-11-18/three-frame-warm'

WINDOW='''"""Three fresh stopped frames, warmed only in a stationary observation window.

State2 -> finish/beep -> state6 carries recognition, never an action command.
Motion, a new item/run, link expiry or a camera worker change clears history.
Metadata proves host read order, not exposure time or zero USB backlog.
"""
import math


class SourcePickupWindow:
    def __init__(self, trigger):
        self.trigger = trigger
        self.status = None
        self.revision = 0
        self.invalidate('not_observing')

    def invalidate(self, reason='inactive'):
        self.context = self.pose = self.epoch = None
        self.generation = self.last_sequence = self.last_stamp = None
        self.consumed_sequence = None
        self.accepted = False
        self.reason = reason
        self.trigger.reset()
        self.revision += 1

    def receive(self, status, received, link_was_expired=False):
        old = self.status
        self.status = dict(status)
        if status['state'] not in (2, 4, 5, 6) or status['item'] >= 3:
            self.invalidate('motion_or_inactive')
            return
        key = (status['run'], status['item'])
        new_station = status['state'] == 2 and (old is None or
            old['state'] != 2 or self.pose != status['pose'])
        if self.context != key or link_was_expired or new_station:
            self.invalidate('new_stationary_window')
            self.context, self.epoch = key, received
        self.pose = status['pose']

    def frame(self, metadata, now):
        self.accepted = False
        if self.context is None:
            return False
        try:
            gen, seq, stamp = metadata['generation'], metadata['sequence'], metadata['completed']
            if (gen is None or type(seq) is not int or seq < 1 or
                not isinstance(stamp, (int, float)) or not math.isfinite(stamp) or
                not math.isfinite(now) or stamp > now or now-stamp > .200):
                raise ValueError('unknown or stale capture')
        except (KeyError, TypeError, ValueError):
            self.trigger.reset(); self.reason = 'unknown_or_stale_capture'
            return False
        if self.generation is not None and gen != self.generation:
            self.trigger.reset()
            self.last_sequence = self.last_stamp = self.consumed_sequence = None
            self.revision += 1
        self.generation = gen
        if stamp <= self.epoch:
            self.reason = 'read_before_stationary_window'
            return False
        if self.last_sequence is None:
            self.last_sequence, self.last_stamp = seq, stamp
            self.reason = 'first_post_window_read_discarded'
            return False
        if seq <= self.last_sequence or stamp <= self.last_stamp:
            self.reason = 'duplicate_or_out_of_order_capture'
            return False
        self.last_sequence, self.last_stamp = seq, stamp
        self.accepted = True; self.reason = 'fresh_stationary_window_frame'
        return True

    def observe(self, detection, timestamp, target, remaining):
        if not self.accepted or self.consumed_sequence == self.last_sequence:
            return None
        self.consumed_sequence = self.last_sequence
        if not isinstance(timestamp, (int, float)) or not math.isfinite(timestamp):
            self.trigger.reset(); return None
        return self.trigger.update(detection, timestamp, target, remaining)

    def diagnostic(self):
        return dict(context=self.context, pose=self.pose, revision=self.revision,
                    accepted=self.accepted, reason=self.reason,
                    required_frames=self.trigger.required_frames,
                    stable_frames=len(self.trigger.history), rotation_gate_enabled=False,
                    action_allowed=self.status is not None and self.status['state'] == 6,
                    exposure_timestamp_verified=False)
'''


def edit(path, old, new):
    b=path.read_bytes();a=old.encode('ascii');z=new.encode('ascii')
    assert b.count(a)==1,(str(path),old[:70],b.count(a));path.write_bytes(b.replace(a,z,1))


def main():
    assert json.loads((E/'matlab-result.json').read_text())['passed']
    assert not P.exists() and not PI.exists(),'Never overwrite a candidate'
    shutil.copytree(BASE,P,ignore=shutil.ignore_patterns('Objects','Listings'))
    p=P/'CHASSIS/USER/source_mcu_cycle.inc'
    edit(p,'static const uint32_t prep[6]={1U,2U,7U,6U,3U,3U};','static const uint32_t prep[5]={1U,2U,7U,6U,3U};')
    edit(p,'if(g_source_cycle_prep>=6U)','if(g_source_cycle_prep>=5U)')
    edit(p,'if(g_source_cycle_prep==4U){g_it_axes=1U;g_it_dm_deg=180.0f;g_it_extension_mm=0.0f;}\r\n  if(g_source_cycle_prep==5U){g_it_axes=2U;g_it_dm_deg=180.0f;g_it_extension_mm=65.0f;}',
           'if(g_source_cycle_prep==4U){g_it_axes=SPF_DM|SPF_EXT;g_it_dm_deg=180.0f;g_it_extension_mm=65.0f;}')
    edit(p,'if(g_source_cycle_state==1U){\r\n  static const uint32_t prep',
           'if(sc_beep_started && !g_source_cycle_beep_done && !Buzzer_IsBusy())g_source_cycle_beep_done=now;\r\n if(g_source_cycle_state==1U){\r\n  static const uint32_t prep')
    edit(p,'g_source_cycle_beeps=2U;g_source_cycle_beep_started=now;return;}\r\n  if(Buzzer_IsBusy())return;\r\n  g_source_cycle_beep_done=now;g_source_cycle_state=6U;SC_Pose();return;',
           'g_source_cycle_beeps=2U;g_source_cycle_beep_started=now;}\r\n  /* First beep releases pickup; remaining beeps continue independently. */\r\n  g_source_cycle_state=6U;SC_Pose();return;')
    p=P/'CHASSIS/USER/integration10_complete.inc'
    edit(p,'if(!VehicleArm_BeginSourceParallel(&it_command,1U,0U)){it_fail(7U);return;}\r\n   it_phase=3U;break;',
           'if(!VehicleArm_BeginSourceParallel(&it_command,1U,0U)){it_fail(7U);return;}\r\n   if(g_source_cycle_state==1U && g_source_cycle_prep==4U &&\r\n      it_command.axes_mask==(SPF_DM|SPF_EXT) && it_command.goal.dm_deg==180.0f &&\r\n      it_command.goal.extension_mm==65.0f){vsp_pick_sync=1U;g_dm_stage2_sync=1U;}\r\n   it_phase=3U;break;')
    p=P/'CHASSIS/USER/vehicle_source_parallel.inc'
    edit(p,'if(vsp_pick_queued==(SPF_EXT|SPF_LIFT)){','if(vsp_pick_queued==(vsp_group.command.axes_mask&(SPF_EXT|SPF_LIFT))){')
    edit(p,'if(address==0U)return vsp_pick_sync && vsp_pick_queued==6U &&',
           'if(address==0U)return vsp_pick_sync && vsp_pick_queued &&\r\n  vsp_pick_queued==(vsp_group.command.axes_mask&(SPF_EXT|SPF_LIFT)) &&')
    PI.mkdir(parents=True)
    entry=PI/'source_fast_cycle_view_20261115.py'
    shutil.copy2(R/'output/stage23_boot_ready_overlay_20261117'/entry.name,entry)
    b=entry.read_bytes();s=b.decode('utf-8');nl='\r\n' if '\r\n' in s else '\n';s=s.replace('\r\n','\n')
    def rep(old,new):
        nonlocal s
        assert s.count(old)==1,(old[:80],s.count(old));s=s.replace(old,new,1)
    rep('from each_observation_rotation import EachObservationRotation','from source_pickup_window import SourcePickupWindow')
    rep('direct=DirectFrameTrigger(pickup_config, required_frames=5)\n    initial_rotation=EachObservationRotation(orbit_config,pickup_config)',
        'direct=DirectFrameTrigger(pickup_config, required_frames=3)\n    pickup_window=SourcePickupWindow(direct)')
    rep('gate.reset();direct.reset();initial_rotation.reset();current_run=None','gate.reset();pickup_window.invalidate(\'stage3_status\');current_run=None')
    rep('alignment_settle.receive(latest,received_at,link_was_expired=status is not None and received_at-received>.60)',
        'alignment_settle.receive(latest,received_at,link_was_expired=status is not None and received_at-received>.60)\n                pickup_window.receive(latest,received_at,link_was_expired=status is not None and received_at-received>.60)')
    rep("frame_epoch.invalidate();alignment_settle.invalidate('status_inactive')", "frame_epoch.invalidate();alignment_settle.invalidate('status_inactive');pickup_window.invalidate('status_inactive')")
    rep("ready_frame=active and status['state']==6 and frame_epoch.frame(metadata,now)\n        if epoch_revision!=frame_epoch.revision:\n            direct.reset();initial_rotation.pause();epoch_revision=frame_epoch.revision",'ready_frame=active and pickup_window.frame(metadata,now)')
    rep("initial_rotation.reset();current_run=status['run']","current_run=status['run']")
    rep('context=key;frame_count=0;sequence=0;gate.reset();direct.reset();initial_rotation.pause()','context=key;frame_count=0;sequence=0;gate.reset()')
    rep("                if valid:initial_rotation.register_rim(body['center_px'])\n",'')
    start=s.index('            if ready_frame and initial_rotation.previous is None')
    end=s.index('            app.draw_material_pickup_detection',start)
    s=s[:start]+'''            detected=app.material_pickup_detector.detect(frame,timestamp=now)
            value=capture.json_value(detected);colors=value['slot_colors']
            target=status['colors'][status['item']] if status['item']<3 else 0
            prediction=pickup_window.observe(value,observed,target,status['colors'][status['item']:]) if ready_frame else None
            stopped=prediction is not None
            if prediction is not None:colors=prediction['colors']
            if status['state']==6:valid=stopped
'''+s[end:]
    rep("if send_phase and observed is not None and time.monotonic()-last_sent>=.10:","if send_phase and observed is not None and (time.monotonic()-last_sent>=.10 or (status['state']==6 and prediction is not None and sequence==0)):")
    # Remove the duplicated packet construction without changing its framing.
    doubled="                        delays=prediction['start_delay_ms']\n                        wire=continuous_observation(status['run'],status['pose'],sequence,True,colors,age,prediction['target_color'],delays)\n"*2
    rep(doubled,doubled[:len(doubled)//2])
    rep("        if not active or status['state']!=6:initial_rotation.pause()\n        learning=False  # Observe120 degrees once; no full-turn learning.","        rotation_info=dict(ready=prediction is not None,rotation_gate_enabled=False,observed_net_deg=0,initial_branch='three_stopped_frames')\n        learning=False")
    s=s.replace('initial_rotation.diagnostic()','rotation_info')
    rep("frame_epoch=frame_epoch.diagnostic(),prearrival_motion='UNKNOWN'", "frame_epoch=frame_epoch.diagnostic(),pickup_window=pickup_window.diagnostic(),prearrival_motion='UNKNOWN'")
    assert 'initial_rotation.' not in s and 'rotation_ready' not in s
    entry.write_bytes(s.replace('\n',nl).encode('utf-8'))
    (PI/'source_pickup_window.py').write_text(WINDOW,encoding='utf-8')
    p=PI/'source_cycle_dashboard_20261115.py';b=(PIBASE/p.name).read_bytes()
    b=b.replace(b"and initial_rotation and not initial_rotation['ready']:",b"and initial_rotation and initial_rotation.get('rotation_gate_enabled',True) and not initial_rotation['ready']:")
    old='物料连续停稳5帧后抓取'.encode();new='物料连续停稳3帧后抓取'.encode();assert b.count(old)==1;b=b.replace(old,new)
    p.write_bytes(b)
    print('Prepared isolated three-frame warmup candidate; no hardware actions.',flush=True)


if __name__=='__main__':main()
