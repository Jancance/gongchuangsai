"""实际startup_axes及回零C的ARM离线检查，不连接硬件。"""
from pathlib import Path
import json,re,struct,subprocess,sys,shutil

ROOT=Path(__file__).resolve().parents[2]
FW=ROOT/'3.stm32f407工程'
OUT=ROOT/'工创赛/evidence/integration/2026-11-03/startup_extension_home'
OLD=ROOT/'工创赛/evidence/integration/2026-11-03/early-scan-startup'
ARM=Path('D:/Keil_v5/ARM/arm5_compiler/bin')

def build_unit(legacy=False,test_only=False):
    original=(OLD/'axes-unit.c').read_text(encoding='utf8')
    unit=original.split('int main(void)')[0]
    # 保留已有轴/电源桩，用真实新模块执行回零；不替换模块内部逻辑。
    pos=unit.index('uint32_t StartupAxis_Exchange(')
    unit=unit[:pos]+'''static uint32_t home_started,home_running,home_bad_param,home_no_start_ack,home_abort,home_stale,home_immediate,home_default_mode;
uint32_t StartupAxis_ReadHomeParameters(uint8_t out[17]){
 const uint8_t p[17]={0x22,2,0,0,30,0,0,0x27,0x10,1,0x2c,3,0x20,0,60,0,0x6b};
 memcpy(out,p,17);out[1]=home_default_mode;if(home_bad_param)out[4]=60;return fail_query?5:1;
}
'''+unit[pos:]
    needle=' if(q[0]==0x1a)'
    index=unit.index(needle)
    unit=unit[:index]+''' if(q[0]==0x9a){home_started++;p->Data[1]=home_immediate?0x9f:2;if(home_immediate)ext_current=0;return home_no_start_ack?5:1;}
 if(q[0]==0x9c){home_abort++;p->Data[1]=2;return 1;}
 if(q[0]==0x3b){p->Data[1]=(home_started && home_running)?7:3;return 1;}
'''+unit[index:]
    unit=unit.replace('static uint32_t checks,failed;', 'volatile uint32_t checks,failed,first_failure;')
    unit=re.sub(r'#define C\(x\).*?\n', '#define C(x) do{checks++;if(!(x)){failed++;if(!first_failure)first_failure=__LINE__;}}while(0)\n',unit)
    if legacy:
        body=original[original.index('int main(void)'):].replace('int main(void)','void test_entry(void)')
        body=re.sub(r' printf\("startup axes checks=.*?return failed\?1:0;', '',body)
        return unit+body
    if test_only:
        mission=(FW/'CHASSIS/USER/vehicle_mission.inc').read_text(encoding='utf8')
        start=mission.index('    /* HOME_TEST_ONLY_COMPLETE')
        end=mission.index('#endif',start)+len('#endif')
        unit+='\nstatic uint32_t g_vehicle_state,g_vehicle_cause;\nstatic void actual_mission_gate(void){\n'+mission[start:end]+'\n}\n'
        return unit+r'''
void test_entry(void){
#if APP_MOTOR_OFFLINE_ONLY
 C(!StartupAxes_Begin());C(!g_startup_axes_vm_request);StartupAxes_Service(1,1);C(!ext_enable && !dm_calls && !home_started);
#else
 reset();home_started=0;home_running=0;C(StartupAxes_Begin());
 for(uint32_t i=0;i<20 && !home_started;i++)step();C(home_started==1);
 home_running=1;for(uint32_t i=0;i<5;i++)step();home_running=0;ext_current=0;
 for(uint32_t i=0;i<10;i++)step();C(g_startup_axes_state==8 && !g_startup_axes_vm_request);
 C(g_startup_extension_home_state==8 && !g_startup_extension_reference_valid);
 C(!dm_calls && !ext_moves && !g_dm_startup_go);
 g_vehicle_state=1;actual_mission_gate();C(g_vehicle_state==0 && g_vehicle_cause==16);
 for(uint32_t i=0;i<10;i++)step();C(!dm_calls && !ext_moves && home_started==1);
#endif
}
'''
    unit+=r'''
static void home_reset(void){
 reset();home_started=home_running=home_bad_param=home_no_start_ack=home_abort=home_stale=home_immediate=home_default_mode=0;
 g_startup_extension_reference_valid=0;
}
static void until_start(void){for(uint32_t i=0;i<20 && !home_started && g_startup_axes_state!=5;i++)step();}
void test_entry(void){
#if APP_MOTOR_OFFLINE_ONLY
 C(!StartupAxes_Begin());C(!g_startup_axes_vm_request);StartupAxes_Service(1,1);C(!ext_enable && !dm_calls && !home_started);
#else
 home_reset();StartupAxes_Service(1,1);C(!home_started && !ext_enable && !dm_calls);
 for(uint32_t mode=0;mode<=5;mode++){
  home_reset();home_default_mode=mode;C(StartupAxes_Begin());until_start();C(home_started==1 && !dm_calls);
 }
 home_reset();home_default_mode=6;C(StartupAxes_Begin());until_start();C(!home_started && !ext_enable && g_startup_axes_state==5);
 home_reset();home_default_mode=255;C(StartupAxes_Begin());until_start();C(!home_started && !ext_enable && g_startup_axes_state==5);
 home_reset();

 C(StartupAxes_Begin());C(!g_startup_extension_reference_valid);until_start();C(home_started==1 && ext_enable==1 && !dm_calls && !ext_moves);
 home_running=1;for(uint32_t i=0;i<5;i++)step();C(g_startup_extension_home_active && !g_startup_extension_reference_valid);
 home_running=0;ext_current=0;for(uint32_t i=0;i<6;i++)step();C(g_startup_extension_home_state==8 && g_startup_extension_reference_valid);
 C(!g_startup_extension_home_active && !ext_moves);dm_prepared=dm_done=1;
 for(uint32_t i=0;i<45;i++)step();C(StartupAxes_Holding() && !ext_moves && home_started==1);
 StartupAxes_Cancel();C(!g_startup_extension_reference_valid);
 home_reset();C(StartupAxes_Begin());home_bad_param=1;until_start();C(g_startup_axes_state==5 && !home_started && !ext_enable && !g_startup_axes_vm_request);
 home_reset();C(StartupAxes_Begin());home_no_start_ack=1;until_start();C(g_startup_axes_state==5 && !g_startup_extension_reference_valid);step();C(home_abort && !g_startup_extension_home_active);
 home_reset();C(StartupAxes_Begin());until_start();for(uint32_t i=0;i<20;i++)step();C(!g_startup_extension_reference_valid && g_startup_extension_home_active);
 tick+=11000;step();C(g_startup_axes_state==5 && !g_startup_axes_vm_request);step();C(home_abort);
 home_reset();home_immediate=1;C(StartupAxes_Begin());until_start();for(uint32_t i=0;i<8;i++)step();C(g_startup_extension_reference_valid && g_startup_extension_home_state==8);
 home_reset();C(StartupAxes_Begin());until_start();g_startup_extension_home_notice=1;ext_current=0;for(uint32_t i=0;i<8;i++)step();C(g_startup_extension_reference_valid);
 home_reset();C(StartupAxes_Begin());until_start();home_running=1;for(uint32_t i=0;i<5;i++)step();home_running=0;ext_current=10001;
 for(uint32_t i=0;i<7;i++)step();C(g_startup_axes_state==5 && !g_startup_extension_reference_valid && !g_startup_axes_vm_request);
 home_reset();C(StartupAxes_Begin());until_start();home_running=1;for(uint32_t i=0;i<5;i++)step();home_running=0;ext_current=-30.4f;
 for(uint32_t i=0;i<8;i++)step();C(g_startup_extension_home_state==8 && g_startup_extension_reference_valid);
 C(fabsf(g_startup_extension_zero_deg+30.4f)<0.01f && fabsf(g_startup_extension_target_deg+30.4f)<0.01f);
 home_reset();C(StartupAxes_Begin());until_start();StartupAxes_Cancel();step();C(home_abort && !g_startup_axes_vm_request && !g_startup_extension_reference_valid);
 home_reset();C(StartupAxes_Begin());until_start();StartupAxes_Safety(0);C(g_startup_axes_state==5 && !g_startup_axes_vm_request);step();C(home_abort);
 home_reset();C(StartupAxes_Begin());until_start();tick+=160;StartupAxes_Safety(1);C(g_startup_axes_state==5 && !g_startup_axes_vm_request);
#endif
}
'''
    return unit

def main():
    sys.path.insert(0,str(ROOT/'工创赛/tmp/position_alignment_deps'))
    import unicorn as uc
    from unicorn import arm_const as reg
    reports=[]
    for offline,legacy,test_only in [(0,False,False),(1,False,False),(0,True,False),(1,True,False),(0,False,True),(1,False,True)]:
        out=OUT/f'axes_arm_{offline}_legacy{int(legacy)}_only{int(test_only)}';out.mkdir(parents=True,exist_ok=True)
        (out/'unit.c').write_text(build_unit(legacy,test_only),encoding='utf8')
        (out/'unit.sct').write_text('LR_ROM 0x08000000 0x00100000 { ER_ROM 0x08000000 0x00100000 { unit.o (i.test_entry, +First) *(+RO) } RW_RAM 0x20000000 0x00020000 { *(+RW,+ZI) } }')
        inc=[OLD/'axis-stubs',FW/'CHASSIS/USER']
        cmd=[str(ARM/'armcc.exe'),'--cpu','Cortex-M3','--c99','--split_sections','-O1',f'-DAPP_MOTOR_OFFLINE_ONLY={offline}',f'-DSTARTUP_EXTENSION_HOME_ENABLE={int(not legacy)}',f'-DSTARTUP_EXTENSION_HOME_TEST_ONLY={int(test_only)}',*[a for p in inc for a in ('-I',str(p))],'-c','unit.c','-o','unit.o']
        logs=[]
        for command in [cmd,[str(ARM/'armlink.exe'),'--entry','test_entry','--scatter','unit.sct','unit.o','--library_type=microlib','-o','unit.axf']]:
            p=subprocess.run(command,cwd=out,capture_output=True,text=True,errors='replace');logs.append({'command':command,'returncode':p.returncode,'output':p.stdout+p.stderr})
            (out/'build.json').write_text(json.dumps(logs,indent=2),encoding='utf8');assert p.returncode==0,p.stdout+p.stderr
        blob=(out/'unit.axf').read_bytes();h=struct.unpack_from('<16sHHIIIIIHHHHHH',blob)
        m=uc.Uc(uc.UC_ARCH_ARM,uc.UC_MODE_THUMB|uc.UC_MODE_MCLASS);m.mem_map(0x08000000,0x100000);m.mem_map(0x20000000,0x20000)
        for i in range(h[10]):
            kind,offset,address,_,size,_,_,_=struct.unpack_from('<IIIIIIII',blob,h[5]+i*h[9])
            if kind==1 and size:m.mem_write(address,blob[offset:offset+size])
        symbols=subprocess.check_output([str(ARM/'fromelf.exe'),'--text','-s','unit.axf'],cwd=out,text=True,errors='replace')
        def addr(n):return int(re.search(r'\b'+n+r'\s+(0x[0-9a-fA-F]+)',symbols)[1],16)
        stop=0x080ff000;m.reg_write(reg.UC_ARM_REG_SP,0x2001fff0);m.reg_write(reg.UC_ARM_REG_LR,stop|1);m.emu_start(addr('test_entry')|1,stop,count=10000000)
        def val(n):return struct.unpack('<I',m.mem_read(addr(n),4))[0]
        report={'offline':offline,'legacy_manual_reference':legacy,'home_test_only':test_only,'checks':val('checks'),'failures':val('failed'),'first_failure_line':val('first_failure'),'completed':m.reg_read(reg.UC_ARM_REG_PC)==stop,'hardware_accessed':False}
        reports.append(report);(OUT/'axes_arm_report.json').write_text(json.dumps(reports,indent=2),encoding='utf8')
        assert report['completed'] and report['checks']>0 and not report['failures'],report
    print(json.dumps(reports))

if __name__=='__main__':main()
