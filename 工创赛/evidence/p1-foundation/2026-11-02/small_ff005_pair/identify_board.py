"""Read-only identification against local ELF images. Never halt/reset/program."""
import json, subprocess, hashlib, time
from vehicle_probe import ROOT, OUT, parse, ConnectHelper

paths=subprocess.run(['rg','--files','--hidden','--no-ignore','-g','*.axf','-g','!**/.git/**'],cwd=ROOT,capture_output=True,text=True,encoding='utf8').stdout.splitlines()
candidates=[]
for name in paths:
    path=ROOT/name
    try:
        sy,segs=parse(path)
        if segs:candidates.append((path,sy,segs))
    except Exception:pass
pack=ROOT/'工创赛/evidence/p1-foundation/2026-10-31/motor_menu_power/local_STM32F4xx_DFP.pack'
for probe in ConnectHelper.get_all_connected_probes(blocking=False):
    result={'probe_id':probe.unique_id,'description':probe.description,'matches':[], 'reset':False,'halt':False,'programmed':False}
    session=ConnectHelper.session_with_chosen_probe(unique_id=probe.unique_id,blocking=False,target_override='stm32f407ve',no_config=True,options={'pack':str(pack),'frequency':1000000,'resume_on_disconnect':False,'connect_mode':'attach','auto_unlock':False,'cache.enable_memory':False})
    try:
        with session:
            t=session.target; result['state']=str(t.get_state()); result['gpioe_odr']=t.read32(0x40021014)
            cache={}
            for path,sy,segs in candidates:
                addr,blob=segs[0];key=(addr,min(len(blob),128))
                if key not in cache:cache[key]=bytes(t.read_memory_block8(*key))
                if cache[key]!=blob[:key[1]]:continue
                if not all(bytes(t.read_memory_block8(a,len(b)))==b for a,b in segs):continue
                names=['g_vm_test_request','g_chassis_auto_active','g_safety_fault','g_rtos_fault','g_can_ctrl_fault_latched','g_reset_iwdg','g_reset_flags']
                result['matches'].append({'path':str(path),'sha256':hashlib.sha256(path.read_bytes()).hexdigest(),'ram':{n:t.read32(sy[n]) for n in names if n in sy}})
    except Exception as e:result['error']=repr(e)
    (OUT/'vehicle'/('identify_'+probe.unique_id+'_'+str(time.time_ns())+'.json')).write_text(json.dumps(result,indent=2),encoding='utf8')
    print(json.dumps(result,ensure_ascii=False,indent=2),flush=True)
