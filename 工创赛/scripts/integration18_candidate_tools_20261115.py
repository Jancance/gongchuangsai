"""Explicit150RPM candidate tools. Default local check, no flash or reset.

apply-driver3000 is separate from preparation and pickup. It requires the exact
new image, reads current settings, and changes only normal-motion Clog_Ms.
"""
import argparse
from datetime import datetime
import json
from pathlib import Path
import time

import source_body_step_live_20261111 as tool
import gripper210_attach_read_20261105 as live
from source_body_probe_20261111 import attach

ROOT=Path(__file__).resolve().parents[1]
E=ROOT/'evidence/integration/2026-11-15/integration18/ext150-clog3000'
P=ROOT/'tmp/source_top_extension150_20261115/project'


def apply_driver():
    _,image=tool.manifest();live.IMAGE=ROOT/image['axf'];symbols,segments=live.load_image()
    out=E/('apply-driver3000-'+datetime.now().strftime('%Y%m%d-%H%M%S')+'.json')
    report=dict(complete=False,flashed=False,reset=False,motion_or_fault_clear_commands=False,
                image_sha256=image['sha256'],power_request_written=False)
    try:
        with attach() as session:
            target=session.target;assert target.get_state().name=='RUNNING'
            count=0
            for address,data in segments:
                for offset in range(0,len(data),4096):
                    part=data[offset:offset+4096]
                    assert bytes(target.read_memory_block8(address+offset,len(part)))==part,'wrong image'
                    count+=len(part)
            report['flash_bytes_verified']=count
            keys=('g_safety_fault g_rtos_fault g_can_ctrl_fault_latched g_chassis_auto_active '
                  'g_low_request g_it_request g_source_parallel_stop_mask '
                  'g_vehicle_arm_stop_pending g_lift_ruler_stop_pending').split()
            report['before_state']={k:target.read32(symbols[k]) for k in keys}
            assert not any(report['before_state'].values()),'pending motion/STOP/system fault'
            assert target.read32(symbols['g_low_state']) in (0,2,9)
            assert target.read32(symbols['g_it_state'])!=1
            report['vm_request_before']=target.read32(symbols['g_vm_test_request'])
            report['pe2_before']=(target.read32(0x40021010)>>2)&1
            if not report['pe2_before']:
                raise RuntimeError('VM未由真实准备流程持有；先prepare-once，本工具不改内部供电状态或直接拉高PE2')
            cycle=target.read32(symbols['g_recovery_diag_cycle'])
            target.write32(symbols['g_recovery_diag_request'],2)
            deadline=time.monotonic()+5
            while time.monotonic()<deadline:
                if target.read32(symbols['g_recovery_diag_cycle'])!=cycle:break
                time.sleep(.05)
            report['cycle']=target.read32(symbols['g_recovery_diag_cycle'])
            report['done']=target.read32(symbols['g_recovery_diag_done'])
            report['results']=list(target.read_memory_block32(symbols['g_recovery_diag_result'],5))
            report['raw_queries_hex']=bytes(target.read_memory_block8(symbols['g_recovery_diag_raw'],32)).hex()
            report['config_result']=target.read32(symbols['g_recovery_config_result'])
            report['ack']=target.read32(symbols['g_recovery_config_ack'])
            before=bytes(target.read_memory_block8(symbols['g_recovery_config_before'],36))
            after=bytes(target.read_memory_block8(symbols['g_recovery_config_after'],36))
            report['before_hex']=before.hex();report['after_hex']=after.hex()
            report['changed_byte_indices']=[i for i in range(36) if before[i]!=after[i]]
            assert report['cycle']!=cycle and report['done']==1 and report['results']==[1]*5
            assert report['config_result']==1
            assert before[:3]==after[:3]==bytes([0x42,0x25,0x18]) and before[-1]==after[-1]==0x6b
            assert set(report['changed_byte_indices']).issubset({31,32})
            assert after[26]==before[26]==1 and int.from_bytes(after[31:33],'big')==3000
            already=int.from_bytes(before[31:33],'big')==3000
            assert report['ack']==2 or (already and report['ack']==0)
            report.update(complete=True,clog_ms=3000,current_clog_rpm=int.from_bytes(after[27:29],'big'),
                          current_clog_ma=int.from_bytes(after[29:31],'big'),already_configured=already)
    except BaseException as error:
        report['error']=repr(error);raise
    finally:
        out.write_text(json.dumps(report,ensure_ascii=False,indent=2)+'\n',encoding='utf-8')
        print(json.dumps(report,ensure_ascii=False,indent=2),flush=True);print(out,flush=True)


def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('mode',nargs='?',default='check',choices=['check','read-only','prepare-once','apply-driver3000'])
    args=parser.parse_args();tool.E=E;tool.P=P;tool.GRIPPER_PWM_REGISTER=0x40000840
    if args.mode=='check':
        _,image=tool.manifest();print(json.dumps(dict(local_check=True,hardware_access=False,
            image_sha256=image['sha256'],extension_rpm=150,software_stall_ms=3000,
            driver_parameter_apply_separate=True),ensure_ascii=False))
    elif args.mode=='apply-driver3000':apply_driver()
    else:tool.run(args.mode)


if __name__=='__main__':main()
