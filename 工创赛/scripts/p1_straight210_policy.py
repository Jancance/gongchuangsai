"""One forward210cm action policy. Protocol identity is selected by the entry file."""
def make_controller(serial_motion):
 class Straight210Controller(serial_motion.Controller):
  capture_drain_s=10.0
  def __init__(self,action='monitor',expected_mode=1,plan=None,route_trial=False,turn_trial=False):
   if route_trial or turn_trial:raise ValueError('Use straight210 entry without route/turn flags')
   if action=='run-once' and (expected_mode!=3 or plan!=0):raise ValueError('Straight210 requires mode3 and explicit default plan')
   super().__init__(action,expected_mode,plan)
  def _speed_profile_healthy(self,r):
   return r.get('debug_plan_id')==0 and r.get('sequence_count')==1 and r['feedback_limit_rpm']==80 and 3<=r['base_rpm']<=52 and not r.get('distance_mode')
  def _running_shape(self,r):
   return (r.get('debug_plan_id')==0 and r.get('sequence_count')==1 and r.get('sequence_index')==0 and r.get('action_kind')==4 and
    r['feedback_limit_rpm']==80 and 0<=r['base_rpm']<=52 and abs(r['correction_rpm'])<=2 and
    ((r.get('distance_mode')==1 and r.get('distance_target_milli_mm')==2100000) or
     (r['phase']==2 and not r['backend_active'] and not r.get('distance_mode'))))
  def consume(self,r,now):
   if self.start_sent and self.status is None and self.action=='run-once':
    if r['phase'] in {4,5}:return super().consume(r,now)
    fresh=self.last_seq is None or 0<((r['seq']-self.last_seq)&0xffffffff)<0x80000000
    if fresh and not self._running_shape(r):return self.abort('straight210_shape_changed')
    if fresh and r['phase']==3:
     if (not r['trip'] and r['backend_result']==1 and not r['backend_cause'] and r['stop_mask']==15 and
      not r['vm_request'] and not r['pe2_active'] and r.get('sequence_state')==3 and r.get('sequence_completed')==1 and
      r.get('distance_mode')==1 and r.get('distance_reached')==1 and
      not any(r[k] for k in ['rtos_fault','safety_fault','can_fault'])):
      self.last_seq=r['seq'];self.last_rx=now;self.status='software_complete';self.stage='finished';return []
     return self.abort('straight210_incomplete_terminal')
   return super().consume(r,now)
 return Straight210Controller
