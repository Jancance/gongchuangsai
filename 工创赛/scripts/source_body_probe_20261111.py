"""Existing synchronous CMSIS-DAP transport, attach only, no USB/core reset."""
import gripper210_attach_read_20261105 as live
from source_stage2_readonly_sync_probe_20261105 import synchronous_interface
from pyocd.core.helpers import ConnectHelper
from pyocd.core.session import Session
def attach():
    options=dict(target_override='cortex_m',connect_mode='attach',frequency=1000000,
        auto_unlock=False,resume_on_disconnect=False,dap_protocol='swd',no_config=True)
    options.update({'cmsis_dap.prefer_v1':False,'cmsis_dap.limit_packets':True})
    settings=Session(None,options=options)
    probes=ConnectHelper.get_all_connected_probes(blocking=False)
    if len(probes)!=1:raise RuntimeError(f'Expected exactly one probe, got{len(probes)}')
    probe=probes[0];interface=probe._link._interface
    if not getattr(interface,'is_bulk',False):raise RuntimeError('Expected existing bulk CMSIS-DAP interface')
    synchronous_interface(interface)
    result=Session(probe,options=options)
    result._source_body_settings=settings
    return result
