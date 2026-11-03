"""安装非零工作参考修正版；不发按键、回零或驱动配置命令。"""
from pathlib import Path
import sys,json,time,hashlib,struct
O=Path(__file__).resolve().parent;R=O.parents[4]
sys.path.insert(0,str(R/'工创赛/evidence/p1-foundation/2026-11-02/small_ff005_pair'))
from vehicle_probe import parse,ConnectHelper,FileProgrammer
oldsy,oldsegs=parse(O/'firmware_home_only_failed_zero_assumption.axf')
new=O/'firmware_home_only_reference_fix.axf';sy,segs=parse(new)
review=json.loads((O/'reference_fix_review.json').read_text())
assert hashlib.sha256(new.read_bytes()).hexdigest()==review['image_sha256']
pack=R/'工创赛/evidence/p1-foundation/2026-10-31/motor_menu_power/local_STM32F4xx_DFP.pack'
s=ConnectHelper.session_with_chosen_probe(unique_id='6afaae24',blocking=False,target_override='stm32f407ve',no_config=True,
 options={'pack':str(pack),'frequency':1000000,'resume_on_disconnect':False,'connect_mode':'attach','auto_unlock':False,'cache.enable_memory':False})
with s:
 t=s.target;assert all(bytes(t.read_memory_block8(a,len(b)))==b for a,b in oldsegs)
 names=['g_vm_test_request','g_chassis_auto_active','g_safety_fault','g_rtos_fault','g_can_ctrl_fault_latched','g_reset_iwdg','g_chassis_auto_cause']
 d={'before':{n:t.read32(oldsy[n]) for n in names},'before_home_active':t.read32(oldsy['g_startup_extension_home_active']),
  'before_home_phase':t.read32(oldsy['s_home_phase']),'before_home_flags':t.read32(oldsy['g_startup_extension_home_flags']),
  'before_zero_reading':struct.unpack('<f',bytes(t.read_memory_block8(oldsy['g_startup_extension_current_deg'],4)))[0],
  'user_reported_stopped':True,'programmed':False,'start_sent':False,'driver_parameters_written':False}
 (O/'download_reference_fix_result.json').write_text(json.dumps(d,indent=2),encoding='utf8')
 assert not(t.read32(0x40021014)&4) and all(v==0 for v in d['before'].values())
 # 已保存的已停机软件参考误判，或外部复位后干净闲置，均不启动动作。
 assert (d['before_home_phase']==5 and d['before_home_flags']==3) or (d['before_home_phase']==0 and not d['before_home_flags'] and not d['before_home_active'])
 t.reset_and_halt();FileProgrammer(s,smart_flash=False).program(str(new));d['programmed']=True;d['verified_bytes']=0
 for a,b in segs:
  assert bytes(t.read_memory_block8(a,len(b)))==b;d['verified_bytes']+=len(b)
 t.reset();t.flush();time.sleep(2)
 names+=['g_startup_axes_state','g_startup_axes_vm_request','g_startup_extension_home_state','g_startup_extension_home_active','g_vehicle_state','g_dm_jog_request']
 d['after']={n:t.read32(sy[n]) for n in names};d['vm_gpio']=bool(t.read32(0x40021014)&4);d['image_sha256']=review['image_sha256']
 (O/'download_reference_fix_result.json').write_text(json.dumps(d,indent=2),encoding='utf8');print(json.dumps(d,indent=2))
 assert not d['vm_gpio'] and all(v==0 for v in d['after'].values())
