"""Read existing frozen firmware evidence; this helper performs no writes."""
import struct


def stop_acknowledged(state):
    """A protocol ACK is not proof of physical rest."""
    return (state.get('g_motion_auto_stop_state') == 2 and
            state.get('g_motion_auto_stop_ack_mask') == 15 and
            state.get('g_chassis_auto_active') == 0)


def capture(target, symbols):
    scalar_names = '''g_can_ctrl_fault_latched g_can_ctrl_trace_result g_can_ctrl_trace_phase
g_can_ctrl_trace_address g_can_ctrl_trace_length g_can_ctrl_trace_pe2_start g_can_ctrl_trace_pe2_end
g_can_ctrl_trace_esr g_can_ctrl_trace_rx_count g_can_ctrl_trace_flush_count
g_can_ctrl_trace_tx_ms g_can_ctrl_trace_total_ms g_motion_auto_stop_state
g_motion_auto_stop_ack_mask g_motion_auto_stop_fail_mask g_motion_auto_stop_session
g_motion_auto_stop_cause g_motion_auto_stop_cut_vm g_chassis_auto_active
g_dm_jog_first_fault g_dm_can2_velocity_raw g_dm_can2_motor_state g_eh_first_reason
g_eh_first_tick g_eh_first_command g_eh_first_exchange g_eh_first_flags
g_eh_first_service_gap g_eh_first_phase g_eh_first_step'''.split()
    def rd(name):
        return target.read32(symbols[name])
    before = rd('g_can_ctrl_fault_latched')
    values = {name: rd(name) for name in scalar_names}
    blocks = {}
    for name, size in [('g_can_ctrl_trace_request', 8), ('g_can_ctrl_trace_last_rx', 20),
                       ('g_can_ctrl_trace_context', 92), ('g_motion_auto_stop_result', 16)]:
        blocks[name] = bytes(target.read_memory_block8(symbols[name], size)).hex()
    for name in ['g_dm_can2_velocity_rad_s', 'g_dm_can2_logical_position_deg']:
        values[name] = struct.unpack('<f', struct.pack('<I', rd(name)))[0]
    after = rd('g_can_ctrl_fault_latched')
    request = bytes.fromhex(blocks['g_can_ctrl_trace_request'])
    reply = bytes.fromhex(blocks['g_can_ctrl_trace_last_rx'])
    context = struct.unpack('<23I', bytes.fromhex(blocks['g_can_ctrl_trace_context']))
    return dict(readonly=True, writes=0, can_trace_frozen=before == after == 1,
                values=values, raw_blocks=blocks,
                request_hex=request[:min(8, values['g_can_ctrl_trace_length'])].hex(),
                last_rx=dict(std_id=struct.unpack_from('<I', reply, 0)[0],
                             ext_id=struct.unpack_from('<I', reply, 4)[0],
                             ide=reply[8], rtr=reply[9], dlc=reply[10],
                             data_hex=reply[11:11 + min(8, reply[10])].hex()),
                context=dict(captured=context[0], command_tick=context[1],
                             vm_command_active=context[2], vm_enable_seen=context[3],
                             vm_enable_count=context[4], vm_enable_age_ms=context[5],
                             guard_ready=context[6], poll_result=list(context[7:11]),
                             poll_flags=list(context[11:15]), poll_count=list(context[15:19]),
                             poll_age_ms=list(context[19:23])),
                stop_protocol_acknowledged=stop_acknowledged(values),
                physical_stop_verified=False)
