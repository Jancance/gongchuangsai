"""Single-commit board adapter. No reset, flash, fault clearing or health writes."""
import math
import struct
from .low_pick_board import LowPickBoard


class StartOutcomeUnknown(RuntimeError):
    pass


class CommittedPickBoard(LowPickBoard):
    def __init__(self,*args):
        super().__init__(*args)
        if self.target.read32(self.symbols['g_low_commit_version'])!=1:
            raise RuntimeError('现板不是单次触发重叠动作版本')
        self._extra_groups=None

    def snapshot(self):
        result=super().snapshot()
        keys=('g_low_commit_version g_low_commit_frame_cycle g_low_release_command_ms '
              'g_low_release_pulse_tick g_low_release_wait_ms g_low_pick_overlap_axes '
              'g_low_lift_retract_axes g_low_return_overlap_axes g_low_wheel_hold_ready '
              'g_low_wheel_hold_stage g_low_wheel_hold_cycle g_low_wheel_hold_request '
              'g_source_body_step_request g_source_body_step_state g_source_body_step_reason '
              'g_source_align_command_seq g_source_align_chain_active '
              'g_chassis_auto_result g_chassis_auto_cause g_lift_compare_top_valid '
              'g_lift_compare_backoff_valid g_lift_compare_top_raw g_lift_ruler_options '
              'g_startup_axes_state g_startup_axes_cause g_extension_home_reference_valid '
              'g_extension_hold_confirmed g_lift_ruler_cause g_dm_jog_first_fault '
              'g_hwt_valid_mask g_hwt_angle_age_ms g_hwt_rate_age_ms').split()
        floats=('g_dm_can2_logical_position_deg','g_startup_extension_current_deg',
                'g_extension_home_reference_deg','g_vehicle_lift_current_deg','g_hwt_yaw_deg')
        if self._extra_groups is None:
            groups=[]
            for address,key in sorted((self.symbols[k],k) for k in (*keys,*floats)):
                if not groups or address-groups[-1][-1][0]>12 or address-groups[-1][0][0]>=128:groups.append([])
                groups[-1].append((address,key))
            self._extra_groups=groups
        for group in self._extra_groups:
            begin=group[0][0];data=self.target.read_memory_block32(begin,(group[-1][0]-begin)//4+1)
            for address,key in group:
                raw=data[(address-begin)//4]
                result[key]=struct.unpack('<f',struct.pack('<I',raw))[0] if key in floats else raw
        result['wheel_hold_results']=list(self.target.read_memory_block32(self.symbols['g_low_wheel_hold_result'],4))
        result['body_stop_masks']=list(self.target.read_memory_block32(self.symbols['g_chassis_auto_stop_mask'],3))
        top=result['g_lift_compare_top_raw'];top=top-(1<<32) if top&(1<<31) else top
        scale=100 if result['g_lift_ruler_options']&0x80 else 10
        result['lift_depth_pulses']=(result['g_vehicle_lift_current_deg']-top/scale)*3200/360
        return result

    @staticmethod
    def pose_ready(value, extension_sign=-1):
        return (CommittedPickBoard._healthy(value) and value['g_low_state'] in (0,2) and
          value['g_it_state']==2 and not value['g_it_request'] and
          value['g_source_body_step_state'] in (0,3) and not value['g_source_body_step_request'] and
          value['g_lift_compare_top_valid']==1 and value['g_lift_compare_backoff_valid']==1 and
          value['g_extension_home_reference_valid']==1 and value['g_extension_hold_confirmed']==1 and
          value['g_startup_axes_state']==4 and
          not any(value[k] for k in ('g_startup_axes_cause','g_lift_ruler_cause','g_dm_jog_first_fault')) and
          abs(value['g_dm_can2_logical_position_deg']-180)<=1 and
          extension_sign in (-1, 1) and
          abs(value['g_startup_extension_current_deg']-value['g_extension_home_reference_deg']-
              extension_sign*65*315.5*360/3200)<=71 and
          abs(value['lift_depth_pulses']-600)<=22.5*3200/360)

    def _write_request(self,request):
        # The MCU owns the complete accepted piece. A lost USB return must not
        # turn a valid two-frame commit into an automatic stop or duplicate start.
        try:self.target.write32(self.symbols['g_low_request'],request)
        except BaseException as error:raise StartOutcomeUnknown(str(error)) from error

    def start_from_observation(self,goal,observation):
        if (not observation.get('pose_active') or not observation.get('stationary') or
            observation.get('pose_id')!='observer-commit-top65' or
            not observation.get('frame_after_pose_ack') or
            not math.isfinite(observation.get('age_s',math.inf)) or not 0<=observation['age_s']<=.20):return False
        value=self.snapshot()
        if (not self.pose_ready(value) or value['g_low_wheel_hold_ready']!=1 or
            value['g_source_align_chain_active']):return False
        if observation.get('pose_sequence')!=value['g_low_done_sequence']:return False
        colors=tuple(value[k] for k in ('g_vis_mat_left','g_vis_mat_right','g_vis_mat_down'))
        if (tuple(observation.get('colors',()))!=colors or colors[goal.region]!=goal.color or
            colors.count(goal.color)!=1 or not value['g_vis_material_valid'] or
            not value['g_vis_mat_cycle'] or
            (value['g_src_now_ms']-value['g_vis_mat_tick'])&0xffffffff>200):return False
        self.target.write32(self.symbols['g_low_confirm_pose_sequence'],goal.sequence)
        self.target.write32(self.symbols['g_low_confirm_frame_cycle'],value['g_vis_mat_cycle'])
        self.prepare(goal)
        return True

    def request_wheel_hold(self):
        value=self.snapshot()
        if not self.pose_ready(value):raise RuntimeError('未处于已完成的固定观察姿态；未发锁轮请求')
        if value['g_low_wheel_hold_ready']!=1:
            self.target.write32(self.symbols['g_low_wheel_hold_request'],1)
        return value['g_low_wheel_hold_cycle']

    def begin_body_step(self,direction,distance_mm):
        value=self.snapshot()
        if not self.pose_ready(value) or value['g_source_body_step_state'] not in (0,3):
            raise RuntimeError('机构未回固定观察姿态；未发底盘指令')
        if direction not in (0,1,2,3) or not 1<=distance_mm<=20:raise ValueError('仅复用已有1..20mm底盘入口')
        self.target.write32(self.symbols['g_source_align_direction'],direction)
        self.target.write32(self.symbols['g_source_align_distance_mm'],struct.unpack('<I',struct.pack('<f',distance_mm))[0])
        self.target.write32(self.symbols['g_source_body_step_request'],2)
        return value['g_source_align_command_seq']

    def finish_body_alignment(self):
        self.target.write32(self.symbols['g_source_body_step_request'],7)

    def cancel_body(self):
        self.target.write32(self.symbols['g_source_body_step_request'],9)

    def motion_evidence(self):
        count=self.target.read32(self.symbols['g_low_group_count'])
        if count>8:raise RuntimeError('动作记录长度无效')
        names=('phase','axes','skew_ms','started_ms','done_ms')
        values={key:list(self.target.read_memory_block32(self.symbols['g_low_group_'+key],count))
                for key in names} if count else {}
        return [{key:values[key][i] for key in names} for i in range(count)]
