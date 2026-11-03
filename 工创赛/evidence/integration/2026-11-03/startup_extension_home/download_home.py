"""用户授权下载：先保存未知旧Flash，再烧录仅回零候选；不发启动/动作。"""
from pathlib import Path
import sys,hashlib,json,time
OUT=Path(__file__).resolve().parent
ROOT=OUT.parents[4]
sys.path.insert(0,str(ROOT/'工创赛/evidence/p1-foundation/2026-11-02/small_ff005_pair'))
from vehicle_probe import parse,ConnectHelper,FileProgrammer

previous=OUT/'download_result.json'
assert not previous.exists() or not json.loads(previous.read_text())['programmed'], '已下载过，不重复复位清除现场'
new=OUT/'firmware_home_only.axf'
sha=hashlib.sha256(new.read_bytes()).hexdigest()
assert sha==json.loads((OUT/'final_review.json').read_text(encoding='utf8'))['image_sha256']
sy,segs=parse(new)
pack=ROOT/'工创赛/evidence/p1-foundation/2026-10-31/motor_menu_power/local_STM32F4xx_DFP.pack'
s=ConnectHelper.session_with_chosen_probe(unique_id='6afaae24',blocking=False,target_override='stm32f407ve',no_config=True,
 options={'pack':str(pack),'frequency':1000000,'resume_on_disconnect':False,'connect_mode':'attach','auto_unlock':False,'cache.enable_memory':False})
assert s is not None
with s:
 t=s.target
 report={'user_authorized_replace_unidentified':True,'programmed':False,'start_command_sent':False,
  'parameters_written':False,'sha256':sha,'before_vm_gpio':bool(t.read32(0x40021014)&4),'before_core_state':str(t.get_state())}
 assert not report['before_vm_gpio'],'VM active: preserve state'
 # 不停核只备份Flash；不从未知符号猜测旧故障状态。
 blob=bytes(t.read_memory_block8(0x08000000,0x80000))
 (OUT/'board_before_download_flash.bin').write_bytes(blob)
 report['before_flash_sha256']=hashlib.sha256(blob).hexdigest()
 previous.write_text(json.dumps(report,indent=2),encoding='utf8')
 assert not(t.read32(0x40021014)&4),'VM became active'
 t.reset_and_halt();FileProgrammer(s,smart_flash=False).program(str(new));report['programmed']=True
 previous.write_text(json.dumps(report,indent=2),encoding='utf8')
 report['flash_bytes_verified']=0
 for addr,b in segs:
  assert bytes(t.read_memory_block8(addr,len(b)))==b,'Flash readback mismatch'
  report['flash_bytes_verified']+=len(b)
 t.reset();t.flush();time.sleep(2)
 names=['g_vm_test_request','g_chassis_auto_active','g_safety_fault','g_rtos_fault','g_can_ctrl_fault_latched',
  'g_reset_iwdg','g_chassis_auto_cause','g_startup_axes_state','g_startup_axes_vm_request',
  'g_startup_extension_home_state','g_startup_extension_home_active','g_vehicle_state','g_vehicle_key',
  'g_dm_jog_request','g_dm_posvel_holding']
 report['after']={n:t.read32(sy[n]) for n in names if n in sy}
 report['after_vm_gpio']=bool(t.read32(0x40021014)&4)
 previous.write_text(json.dumps(report,indent=2),encoding='utf8');print(json.dumps(report,indent=2))
 assert not report['after_vm_gpio'] and all(v==0 for v in report['after'].values()), 'Unexpected startup state; do not press key'
