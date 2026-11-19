"""Attach-session adapter for the explicit low-pick owner; no device discovery.

Construction verifies the entire supplied Flash image before any write. The
caller owns attachment, camera pose/profile validity and the complete mission.
This adapter exposes prepare/confirm separately and never clears faults.
"""
from dataclasses import dataclass
import math


@dataclass(frozen=True)
class LowPickTarget:
    sequence: int
    slot: int
    color: int
    region: int
    yaw_deg: float
    extension_mm: float
    pose_version: int
    scene_id: int
    view_pose_id: str


class LowPickBoard:
    def __init__(self, target, symbols, flash_segments):
        self.target, self.symbols = target, symbols
        self.verified_flash_bytes = 0
        if target.get_state().name != 'RUNNING':
            raise RuntimeError('主控未运行；不自动复位或恢复运行')
        if not flash_segments:
            raise ValueError('需要完整目标Flash镜像')
        for address, data in flash_segments:
            if not 0x08000000 <= address < address + len(data) <= 0x08100000:
                raise ValueError('目标Flash段地址无效')
            for offset in range(0, len(data), 4096):
                chunk = data[offset:offset + 4096]
                if bytes(target.read_memory_block8(address + offset, len(chunk))) != chunk:
                    raise RuntimeError('板载固件与低位候选不匹配')
                self.verified_flash_bytes += len(chunk)
        self._last_confirmation = None
        self._prepared = None
        self._snapshot_groups = None

    def snapshot(self):
        keys = ('g_low_request g_low_state g_low_phase g_low_fault g_low_rejected '
                'g_low_accepted_sequence g_low_done_sequence g_low_elapsed_ms '
                'g_low_clear_valid g_low_clear_ms g_low_release_ms '
                'g_it_state g_it_request g_it_cause g_it_rejected '
                'g_lift_ruler_stop_pending g_vehicle_arm_stop_pending g_source_parallel_stop_mask '
                'g_safety_ready g_safety_fault g_rtos_fault g_can_ctrl_fault_latched '
                'g_chassis_auto_active g_vis_material_valid g_vis_mat_cycle '
                'g_vis_mat_left g_vis_mat_right g_vis_mat_down g_vis_mat_tick g_src_now_ms').split()
        floats = ('g_low_actual_pick_depth_pulses', 'g_low_actual_rise_mm')
        if self._snapshot_groups is None:
            groups = []
            for address, key in sorted((self.symbols[k], k) for k in (*keys, *floats)):
                if not groups or address - groups[-1][-1][0] > 12 or address - groups[-1][0][0] >= 128:
                    groups.append([])
                groups[-1].append((address, key))
            self._snapshot_groups = groups
        result = {}
        import struct
        for group in self._snapshot_groups:
            start = group[0][0]
            words = self.target.read_memory_block32(start, (group[-1][0] - start) // 4 + 1)
            for address, key in group:
                value = words[(address - start) // 4]
                result[key] = struct.unpack('<f', struct.pack('<I', value))[0] if key in floats else value
        return result

    @staticmethod
    def _healthy(value):
        faults = ('g_low_fault g_it_cause g_it_rejected g_safety_fault g_rtos_fault '
                  'g_can_ctrl_fault_latched g_chassis_auto_active').split()
        return value['g_safety_ready'] == 1 and not any(value[k] for k in faults)

    def _write_request(self, request):
        try:
            self.target.write32(self.symbols['g_low_request'], request)
        except BaseException:
            # A failed transport return cannot prove the motion write was lost.
            try:
                self.target.write32(self.symbols['g_low_request'], 9)
                self.target.write32(self.symbols['g_it_request'], 9)
            except BaseException:
                pass  # Preserve the original failure; caller records disconnect/STOP uncertainty.
            raise

    def prepare(self, goal):
        if (any(type(v) is not int or not 0 < v <= 0xffffffff
                for v in (goal.sequence, goal.pose_version, goal.scene_id)) or
                any(type(v) is not int for v in (goal.slot, goal.color, goal.region)) or
                goal.slot not in (1, 2, 3) or goal.color not in range(1, 7) or goal.region not in (0, 1, 2) or
                not isinstance(goal.view_pose_id, str) or not goal.view_pose_id or
                not math.isfinite(goal.yaw_deg) or not 0 <= goal.yaw_deg <= 270 or
                not math.isfinite(goal.extension_mm) or not 0 <= goal.extension_mm <= 170 or
                (goal.region == 2 and (goal.yaw_deg != 180 or goal.extension_mm != 0))):
            raise ValueError('目标参数无效；不写入主控')
        before = self.snapshot()
        if (not self._healthy(before) or before['g_low_state'] not in (0, 2) or
                before['g_it_state'] != 2 or before['g_low_request'] or before['g_it_request'] or
                goal.sequence == before['g_low_accepted_sequence']):
            raise RuntimeError('当前状态不接纳新目标；不清故障或自动重试')
        fields = dict(g_low_sequence=goal.sequence, g_low_slot=goal.slot, g_low_color=goal.color,
                      g_low_region=goal.region, g_low_pose_version=goal.pose_version, g_low_scene_id=goal.scene_id)
        for key, value in fields.items():
            self.target.write32(self.symbols[key], value)
        import struct
        for key, value in [('g_low_target_yaw', goal.yaw_deg), ('g_low_target_extension_mm', goal.extension_mm)]:
            self.target.write32(self.symbols[key], struct.unpack('<I', struct.pack('<f', value))[0])
        self._write_request(1)
        self._prepared = goal
        return before  # Acceptance/completion is observed separately using this sequence.

    def confirm(self, *, sequence, observation):
        """Consume a fresh pose-matched frontend result plus actual MCU RX cycle."""
        if (self._prepared is None or self._prepared.sequence != sequence or
                not observation.get('pose_active') or not observation.get('stationary') or
                observation.get('pose_id') != self._prepared.view_pose_id or observation.get('pose_sequence') != sequence or
                not observation.get('frame_after_pose_ack') or
                not math.isfinite(observation.get('age_s', math.inf)) or
                not 0 <= observation['age_s'] <= .20):
            return False
        value = self.snapshot()
        colors = tuple(value[k] for k in ('g_vis_mat_left', 'g_vis_mat_right', 'g_vis_mat_down'))
        frame = value['g_vis_mat_cycle']
        if (not self._healthy(value) or value['g_low_state'] != 1 or value['g_low_phase'] != 3 or
                value['g_low_accepted_sequence'] != sequence or value['g_low_request'] or value['g_it_request'] or
                value['g_it_state'] != 2 or value['g_vis_material_valid'] != 1 or frame <= 0 or
                (value['g_src_now_ms'] - value['g_vis_mat_tick']) & 0xffffffff > 200 or
                tuple(observation.get('colors', ())) != colors or self._last_confirmation == (sequence, frame)):
            return False
        self.target.write32(self.symbols['g_low_confirm_pose_sequence'], sequence)
        self.target.write32(self.symbols['g_low_confirm_frame_cycle'], frame)
        self._write_request(2)
        self._last_confirmation = (sequence, frame)
        return True

    def cancel(self):
        self.target.write32(self.symbols['g_low_request'], 9)
        self.target.write32(self.symbols['g_it_request'], 9)
