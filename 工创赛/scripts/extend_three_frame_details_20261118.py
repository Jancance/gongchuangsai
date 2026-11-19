"""Apply clarified first-only120, per-pick cue, side Z offset and immediate lift submission."""
from pathlib import Path
import json
from prepare_three_frame_warm_20261118 import P,PI,E,edit


def main():
    assert json.loads((E/'matlab-details.json').read_text())['passed']
    p=P/'CHASSIS/USER/integration10_complete.inc'
    a='   if(g_source_cycle_state==1U && g_source_cycle_prep==4U &&\r\n      it_command.axes_mask==(SPF_DM|SPF_EXT) && it_command.goal.dm_deg==180.0f &&\r\n      it_command.goal.extension_mm==65.0f){vsp_pick_sync=1U;g_dm_stage2_sync=1U;}'
    edit(p,a,'#if !APP_MOTOR_OFFLINE_ONLY\r\n'+a+'\r\n#endif')
    p=P/'CHASSIS/USER/pzq_commit_overlap.inc'
    edit(p,'static float low_yaw,low_extension,low_measured_depth;','static float low_yaw,low_extension,low_measured_depth,low_pick_depth;')
    edit(p,'static uint32_t Low_ReadDepth(float *depth)',
        'static float Low_CommandPickDepth(uint32_t region)\r\n{return (float)SOURCE_PICK_PULSES-(region<2U?10.0f*SOURCE_LIFT_PULSES_PER_MM:0.0f);}\r\nstatic uint32_t Low_ReadDepth(float *depth)')
    edit(p,'if(g_low_actual_rise_mm<=65.0f)return 0U;',
        '/* Keep the prior nominal absolute release height after the side Z offset. */\r\n if(g_low_actual_rise_mm<=65.0f-((float)SOURCE_PICK_PULSES-low_pick_depth)/SOURCE_LIFT_PULSES_PER_MM)return 0U;')
    edit(p,'return Low_Group(4U,g_low_pick_overlap_axes,low_yaw,low_extension,(float)SOURCE_PICK_PULSES);',
        'return Low_Group(4U,g_low_pick_overlap_axes,low_yaw,low_extension,low_pick_depth);')
    edit(p,'low_slot=g_low_slot;low_yaw=g_low_target_yaw;low_extension=g_low_target_extension_mm;',
        'low_slot=g_low_slot;low_yaw=g_low_target_yaw;low_extension=g_low_target_extension_mm;\r\n low_pick_depth=Low_CommandPickDepth(g_low_region);')
    b=p.read_bytes();i=b.index(b' if(g_low_phase==4U && !g_it_lead5_grip_done');j=b.index(b'\r\n#endif',i)
    p.write_bytes(b[:i]+b' /* Close at the completed selected pick pose, not 5mm before it. */'+b[j:])
    edit(p,'if(!g_it_lead5_grip_done && jiazhua(140.0f)!=1U){Low_Fail(7U);return;}\r\n  if(!g_low_grip_command_ms)g_low_grip_command_ms=(uint32_t)xTaskGetTickCount()-low_begin;',
        'if(jiazhua(140.0f)!=1U){Low_Fail(7U);return;}\r\n  g_low_grip_command_ms=(uint32_t)xTaskGetTickCount()-low_begin;g_it_lead5_grip_done=1U;')
    p=P/'CHASSIS/USER/source_mcu_cycle.inc'
    edit(p,'static uint32_t sc_beep_started,sc_last_tx;', '''static uint32_t sc_beep_started,sc_last_tx;
volatile uint32_t g_source_cycle_pick_beeps,g_source_cycle_pick_beep_pending;
volatile uint32_t g_source_cycle_pick_beep_ms[3];
static void SC_PickBeep(uint32_t now)
{
 if(!g_source_cycle_pick_beep_pending || Buzzer_IsBusy())return;
 if(g_source_cycle_pick_beeps<3U)g_source_cycle_pick_beep_ms[g_source_cycle_pick_beeps]=now;
 g_source_cycle_pick_beeps++;g_source_cycle_pick_beep_pending--;
 Buzzer_StartTimed(1U,80U,80U);
}'''.replace('\n','\r\n'))
    edit(p,'if(sc_beep_started && !g_source_cycle_beep_done && !Buzzer_IsBusy())g_source_cycle_beep_done=now;',
        'if(sc_beep_started && !g_source_cycle_beep_done && !Buzzer_IsBusy())g_source_cycle_beep_done=now;\r\n SC_PickBeep(now);')
    edit(p,'g_source_cycle_start_lag_ms=(uint32_t)xTaskGetTickCount()-g_source_cycle_due_tick;return;',
        'if(g_low_state==1U){g_source_cycle_pick_beep_pending++;SC_PickBeep(now);}\r\n   g_source_cycle_start_lag_ms=(uint32_t)xTaskGetTickCount()-g_source_cycle_due_tick;return;')
    p=PI/'source_pickup_window.py';s=p.read_text(encoding='utf-8')
    s=s.replace('import math\n','import math\nfrom initial_rotation import InitialRotation\n',1)
    s=s.replace('def __init__(self, trigger):','def __init__(self, trigger, orbit_config):',1)
    s=s.replace('        self.trigger = trigger\n','        self.trigger = trigger\n        self.rotation = InitialRotation(orbit_config)\n        self.rotation_run = None\n        self.rotation_binding = None\n',1)
    s=s.replace('        self.trigger.reset()\n        self.revision += 1','        self.trigger.reset()\n        self.rotation.previous = None\n        self.rotation_binding = None\n        self.revision += 1',1)
    s=s.replace('        old = self.status\n','        old = self.status\n        if self.rotation_run != status[\'run\']:\n            self.rotation.reset(); self.rotation_run = status[\'run\']\n            self.rotation_binding = None\n',1)
    s=s.replace('            self.revision += 1\n        self.generation = gen','            self.rotation.previous = None; self.rotation_binding = None\n            self.revision += 1\n        self.generation = gen',1)
    s=s.replace('        return self.trigger.update(detection, timestamp, target, remaining)', '''        if self.status['item'] == 0 and self.rotation_binding == self.revision:
            previous = self.rotation.previous
            if previous is not None and timestamp-previous[0] > self.rotation.max_interval:
                self.rotation.previous = None
            measured, opened = self.rotation.progress, self.rotation.opened
            self.rotation.update(detection, timestamp, target)
            # Visibility/camera gaps add no angle. Preserve already witnessed
            # increments, rather than inventing rotation in an unobserved gap.
            if self.rotation.previous is None:
                self.rotation.progress, self.rotation.opened = measured, opened
        candidate = self.trigger.update(detection, timestamp, target, remaining)
        return candidate if self.status['item'] > 0 or self.rotation.opened else None

    def bind_rim(self, center):
        if self.context is None or self.rotation_binding == self.revision:
            return
        measured, opened = self.rotation.progress, self.rotation.opened
        self.rotation.register_rim(center)
        self.rotation.progress, self.rotation.opened = measured, opened
        self.rotation_binding = self.revision

    def rotation_diagnostic(self):
        first = self.status is not None and self.status['item'] == 0
        return dict(ready=self.rotation.opened if first else True,
                    rotation_gate_enabled=first,
                    observed_net_deg=math.degrees(self.rotation.progress),
                    required_deg=120 if first else 0,
                    initial_branch='observed_first120' if first else 'later_pick_no_rotation_wait')''',1)
    s=s.replace('stable_frames=len(self.trigger.history), rotation_gate_enabled=False,','stable_frames=len(self.trigger.history), first_rotation=self.rotation_diagnostic(),',1)
    p.write_text(s,encoding='utf-8')
    p=PI/'source_fast_cycle_view_20261115.py';b=p.read_bytes();s=b.decode('utf-8');nl='\r\n' if '\r\n' in s else '\n';s=s.replace('\r\n','\n')
    s=s.replace('pickup_window=SourcePickupWindow(direct)','pickup_window=SourcePickupWindow(direct,orbit_config)',1)
    s=s.replace("                body['observation_valid']=rim_observation_valid(body,3.0)","                body['observation_valid']=rim_observation_valid(body,3.0)\n                if body['observation_valid']:pickup_window.bind_rim(body['center_px'])",1)
    s=s.replace('            detected=app.material_pickup_detector.detect(frame,timestamp=now)', '''            if status['item']==0 and pickup_window.rotation_binding!=pickup_window.revision:
                try:
                    ready_body=rim.estimate(frame) if rim is not None else estimate(frame,profile)
                    if rim_observation_valid(ready_body,3.0):pickup_window.bind_rim(ready_body['center_px'])
                except (ValueError,ArithmeticError):pass
            detected=app.material_pickup_detector.detect(frame,timestamp=now)''',1)
    s=s.replace("rotation_info=dict(ready=prediction is not None,rotation_gate_enabled=False,observed_net_deg=0,initial_branch='three_stopped_frames')",'rotation_info=pickup_window.rotation_diagnostic()',1)
    p.write_bytes(s.replace('\n',nl).encode('utf-8'))
    print('Applied first-only120, side -10mm, pick cue, same-cycle close/return; no hardware.',flush=True)


if __name__=='__main__':main()
