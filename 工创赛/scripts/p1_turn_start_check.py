"""Actual-C startup admission/fault-injection checks; no hardware access."""
import json,re,shutil,struct,subprocess,sys
from pathlib import Path
from p1_four_wheel_sim import SOURCE_FILES
from p1_turn_fault_audit import function as extract

ROOT=Path(__file__).resolve().parents[2]
USER=ROOT/'3.stm32f407工程/USER'
OUT=ROOT/'工创赛/evidence/p1-foundation/2026-10-29/turn_start_fix/start_checks_final2'


def main():
    OUT.mkdir(parents=True,exist_ok=False);src=OUT/'src';src.mkdir()
    for name in set(SOURCE_FILES+['heading_session.c','heading_session.h','heading_controller.c','heading_controller.h','turn_brake.h','motion_control.h','hwt101.h']):
        shutil.copyfile(USER/name,src/name)
    shutil.copytree(Path(__file__).parent/'p1_offline_tests/stubs',OUT/'stubs')
    p=src/'route_config.h';p.write_bytes(p.read_bytes().replace(b'APP_ROUTE_USE_CUSTOM 0',b'APP_ROUTE_USE_CUSTOM 1'))
    # A test-only hook observes lock depth and injects races during real pure preparation.
    p=src/'chassis_auto_test.c';b=p.read_bytes();marker=b'    uint32_t i, j;\r\n    for (i = 0U; i < 3U; i++)';assert marker in b
    b=b.replace(marker,b'    extern void Test_PrepareHook(void);\r\n    uint32_t i, j;\r\n    Test_PrepareHook();\r\n    for (i = 0U; i < 3U; i++)',1);p.write_bytes(b)
    main_c=(USER/'main.c').read_bytes().decode('gbk')
    prefix=(Path(__file__).parent/'p1_offline_tests/route_speed.c').read_text(encoding='utf-8').split('/* FRONT_TYPE */')[0]
    prelude='''
#include "src/heading_session.h"
#include "src/hwt101.h"
#define HR_IDLE 0U
#define HR_WAIT 1U
#define HR_RUN 2U
#define HR_DONE 3U
#define HR_FAULT 4U
#define HR_CANCELLED 5U
#define HEADING_SOURCE_FRESH_MS 100U
#define App_StageStartVm() 0U
#define App_StageDoneVm() 0U
typedef struct {uint32_t sequence,sample_ms,angle_ms,rate_ms,healthy,valid_mask;float yaw_deg,rate_dps;} AppHeadingSource;
static AppHeadingSource s_heading_source;
static uint32_t s_heading_source_ready,g_heading_exec_request,g_heading_exec_phase,g_heading_exec_trip;
static uint32_t g_heading_exec_state,g_heading_exec_cause,g_heading_exec_action,g_heading_exec_backend_session;
static int32_t g_heading_exec_rpm;
static float g_heading_exec_target_deg,g_heading_exec_error_deg,g_heading_exec_relative_deg;
static float s_heading_exec_relative;
static uint32_t g_vm_test_request,s_heading_exec_wait_ms,s_motion_turn_max_rpm;
volatile uint32_t g_hwt_uart_errors,g_hwt_overflows,g_hwt_checksum_errors,g_hwt_update_tick;
volatile uint32_t g_hwt_version_result,g_hwt_valid_mask,g_hwt_angle_age_ms,g_hwt_rate_age_ms;
volatile uint32_t g_hwt_cal_active,g_hwt_cal_request,g_hwt_cal_end_tx_result,g_hwt_version_state,g_hwt_version_request,g_hwt_a7_request;
volatile float g_hwt_yaw_deg,g_hwt_wz_dps;
static HeadingSession s_heading_exec,s_heading_prepared;
static ChassisAutoPreparedHeading s_heading_backend_prepared;
static uint32_t g_heading_prepare_cycles,g_heading_prepare_max_cycles;
typedef struct {uint32_t captured;} HeadingStopAngleSample;
static struct {HeadingStopAngleSample start,zero_request,reached,completed;uint32_t started,zero_submit_accepted;float target_deg;} g_heading_stop_trace;
static uint32_t mode,prepare_calls,test_can_steps,mock_exclusive,mock_power,mock_clock;
static void App_HeadingSourceObserve(void){}
static void App_HeadingStopObserve(uint32_t now){(void)now;}
static void App_HeadingStopCapture(HeadingStopAngleSample *p,uint32_t now){(void)now;p->captured=1;}
static uint32_t App_HeadingRuntimeExclusive(void){return mock_exclusive;}
static uint32_t App_CanJogReady(void){return g_safety_ready&&g_can_guard_ready;}
static uint32_t App_HeadingPowerReady(uint32_t now){(void)now;return mock_power;}
static uint32_t App_HeadingRuntimeOwns(const ChassisAutoStatus *p){return p->session==g_heading_exec_backend_session&&p->heading_mode==1&&p->segment==2;}
static uint32_t App_HeadingRuntimeSource(uint32_t now);
static void App_MotionFaultCapture(uint32_t cause);
static void App_HeadingRuntimeTrip(uint32_t cause);
#define MOTION_FAULT_DIAG_FIELDS 38U
static volatile uint32_t g_motion_fault_diag[38],g_motion_fault_diag_valid;
static uint32_t s_motion_fault_timing[HWT_TIMING_FIELDS];
static struct {uint32_t phase;} g_forward_hold;
static uint32_t g_heading_fault_phase,g_heading_fault_source_flags,g_heading_fault_sample_age_ms;
static uint32_t g_heading_fault_angle_age_ms,g_heading_fault_rate_age_ms,g_heading_fault_hwt_update_age_ms,g_heading_fault_tick_ms;
void HWT101_ReadDiagnostic(HwtFaultDiagnostic *p){HwtFaultDiagnostic empty={0};*p=empty;p->arm_sequence=4;p->uart_ore=g_hwt_uart_errors;}
void HWT101_ReadTiming(uint32_t *p,uint32_t now){uint32_t i;for(i=0;i<HWT_TIMING_FIELDS;i++)p[i]=now+i;}
uint32_t HWT101_TimingBegin(void){return ++mock_clock;}
void HWT101_TimingEnd(uint32_t scope,uint32_t start){(void)scope;(void)start;CHECK(test_depth>0);}
void Test_PrepareHook(void){
 CHECK(test_depth==0);prepare_calls++;
 if(mode==1)g_heading_exec_request=2;
 if(mode==2)s_heading_source.healthy=0;
 if(mode==3)g_hwt_uart_errors++;
 if(mode==4)test_tick+=101;
 if(mode==5)g_vm_test_request=0;
 if(mode==6)g_chassis_auto_session++;
 if(mode==7)App_HeadingRuntimeTrip(3);
 if(mode==8)mock_exclusive=0;
 if(mode==9)mock_power=0;
}
'''
    helper=main_c[main_c.index('static __inline uint32_t App_HeadingFrameAge'):main_c.index('/* 唯一生产者TaskA')]
    functions=helper+'\n'+extract(main_c,'App_HeadingSourcePublish')+'\n'+extract(main_c,'App_HeadingRuntimeSource')+'\n'+extract(main_c,'App_MotionFaultCapture')+'\n'+extract(main_c,'App_HeadingRuntimeTrip')+'\n'+extract(main_c,'App_HeadingRuntimeStep').replace('ChassisAuto_CanStep();','test_can_steps++;')
    tests='''
static void reset_test(void){
 test_tick=2000;test_depth=0;test_can_steps=prepare_calls=0;mock_exclusive=mock_power=1;
 g_rtos_fault=g_safety_fault=g_motion_diag_fault_latched=g_motion_auto_stop_state=g_motion_auto_stop_cut_vm=0;
 g_wheel_test_active=g_wheel_test_cut_vm=g_can_ctrl_fault_latched=0;g_safety_ready=g_can_guard_ready=1;
 ChassisAuto_Init();ChassisAuto_SafetyUpdate(1);
 g_heading_exec_request=g_heading_exec_trip=0;g_heading_exec_phase=HR_WAIT;g_vm_test_request=1;
 g_motion_fault_diag_valid=0;g_hwt_uart_errors=g_hwt_overflows=g_hwt_checksum_errors=0;
 g_hwt_cal_active=g_hwt_cal_request=g_hwt_cal_end_tx_result=g_hwt_version_state=g_hwt_version_request=g_hwt_a7_request=0;
 g_hwt_version_result=1;g_hwt_valid_mask=3;g_hwt_update_tick=test_tick;
 g_hwt_angle_age_ms=g_hwt_rate_age_ms=0;g_hwt_yaw_deg=177;g_hwt_wz_dps=0;
 App_HeadingSourcePublish();s_heading_source_ready=1;
 s_motion_turn_max_rpm=APP_TURN_BRAKE_MAX_RPM;s_heading_exec_relative=90;s_heading_exec_wait_ms=test_tick;
 HeadingSession_Init(&s_heading_exec);memset(&g_heading_stop_trace,0,sizeof(g_heading_stop_trace));
}
uint32_t test_entry(void){
 uint32_t n,session;ChassisAutoPreparedHeading prepared;const uint32_t causes[10]={0,1,2,2,2,3,5,3,3,3};
 for(mode=0;mode<10;mode++){
  reset_test();session=g_chassis_auto_session;App_HeadingRuntimeStep();
  CHECK(test_depth==0);CHECK(prepare_calls==1);CHECK(g_heading_exec_trip==causes[mode]);
  if(mode==0){CHECK(g_chassis_auto_active==1);CHECK(g_chassis_auto_session==session+1);
   CHECK(s_turn_brake_trial);CHECK(g_heading_exec_phase==HR_RUN);CHECK(test_can_steps==1);
   for(n=0;n<4;n++)CHECK(s_commands[2][n].speed_raw==0);
  }else{CHECK(!g_chassis_auto_active);CHECK(!g_vm_test_request);CHECK(!test_can_steps);
   CHECK(g_motion_fault_diag_valid==1);CHECK(g_motion_fault_diag[2]==causes[mode]);
   App_HeadingRuntimeTrip(8);CHECK(g_motion_fault_diag[2]==causes[mode]);
   CHECK(g_heading_exec_trip==causes[mode]);
   App_HeadingRuntimeStep();CHECK(prepare_calls==1);CHECK(g_heading_exec_trip==causes[mode]);}
 }
 mode=0;reset_test();g_hwt_update_tick=test_tick=22930;
 g_hwt_valid_mask=0;g_hwt_angle_age_ms=g_hwt_rate_age_ms=0xffffffffU;App_HeadingSourcePublish();
 CHECK(!s_heading_source.healthy);CHECK(App_HeadingFrameAge(&s_heading_source,22938,1)==0xffffffffU);
 CHECK(App_HeadingFrameAge(&s_heading_source,22938,2)==0xffffffffU);
 g_hwt_valid_mask=2;g_hwt_rate_age_ms=7;App_HeadingSourcePublish();
 CHECK(App_HeadingFrameAge(&s_heading_source,22938,1)==0xffffffffU);
 CHECK(App_HeadingFrameAge(&s_heading_source,22938,2)==15);
 g_hwt_valid_mask=3;test_tick=g_hwt_update_tick=3;g_hwt_angle_age_ms=g_hwt_rate_age_ms=7;
 App_HeadingSourcePublish();CHECK(s_heading_source.healthy);CHECK(App_HeadingFrameAge(&s_heading_source,3,1)==7);
 reset_test();session=g_chassis_auto_session;
 CHECK(ChassisAuto_PrepareHeading(0,0,1,&prepared));CHECK(!g_chassis_auto_active);
 CHECK(!ChassisAuto_BeginPreparedHeading(&prepared,session+1));
 prepared.commands[2][0].speed_raw=1;CHECK(!ChassisAuto_BeginPreparedHeading(&prepared,session));
 CHECK(!g_chassis_auto_active);
 CHECK(ChassisAuto_PrepareHeading(0,5,0,&prepared));CHECK(ChassisAuto_BeginPreparedHeading(&prepared,session));
 CHECK(!s_turn_brake_trial);CHECK(!ChassisAuto_BeginPreparedHeading(&prepared,session));
 return first_failure ? (0x80000000U|first_failure) : checks;
}
'''
    (OUT/'unit.c').write_bytes((prefix+prelude+functions+tests).encode('gbk'))
    (OUT/'unit.sct').write_text('LR_ROM 0x08000000 0x00100000 { ER_ROM 0x08000000 0x00100000 { unit.o (i.test_entry, +First) *(+RO) } RW_RAM 0x20000000 0x00020000 { *(+RW,+ZI) } }\n')
    arm=Path('D:/Keil_v5/ARM/arm5_compiler/bin');logs=[]
    objects=[('unit.c','unit.o'),('src/chassis_kinematics.c','kinematics.o'),('src/motion_profile.c','profile.o'),('src/heading_controller.c','controller.o'),('src/heading_session.c','session.o')]
    commands=[[str(arm/'armcc.exe'),'--cpu','Cortex-M3','--c99','--split_sections','-O1','-I','src','-I','stubs','-DAPP_TURN_BRAKE_TRIAL_ENABLE=1','-DAPP_TURN_BRAKE_TRIAL_DPS=30','-c',s,'-o',o] for s,o in objects]
    commands.append([str(arm/'armlink.exe'),'--entry','test_entry','--scatter','unit.sct']+[o for _,o in objects]+['--library_type=microlib','-o','unit.axf'])
    for cmd in commands:
        p=subprocess.run(cmd,cwd=OUT,capture_output=True,text=True,errors='replace',timeout=60);logs.append(dict(command=cmd,returncode=p.returncode,stdout=p.stdout,stderr=p.stderr))
        (OUT/'build.json').write_text(json.dumps(logs,indent=2))
        if p.returncode:raise RuntimeError(p.stdout+p.stderr)
    sys.path.insert(0,str(ROOT/'工创赛/tmp/position_alignment_deps'))
    import unicorn as uc
    from unicorn import arm_const as reg
    machine=uc.Uc(uc.UC_ARCH_ARM,uc.UC_MODE_THUMB|uc.UC_MODE_MCLASS);machine.mem_map(0x08000000,0x100000);machine.mem_map(0x20000000,0x20000)
    b=(OUT/'unit.axf').read_bytes();h=struct.unpack_from('<16sHHIIIIIHHHHHH',b)
    for i in range(h[10]):
        kind,off,addr,_,size,_,_,_=struct.unpack_from('<IIIIIIII',b,h[5]+i*h[9])
        if kind==1 and size:machine.mem_write(addr,b[off:off+size])
    stop=0x080ff000;machine.reg_write(reg.UC_ARM_REG_SP,0x2001fff0);machine.reg_write(reg.UC_ARM_REG_LR,stop|1);machine.emu_start(h[4]|1,stop,count=10000000)
    value=machine.reg_read(reg.UC_ARM_REG_R0)
    if value&0x80000000 or machine.reg_read(reg.UC_ARM_REG_PC)!=stop:raise RuntimeError(f'actual C check failed line{value&0x7fffffff}')
    report=dict(hardware_accessed=False,actual_runtime_and_backend_c=True,assertions=value,race_cases=10,preparation_observed_outside_critical=True,invalid_age_and_real_clock_wrap_passed=True,limits='RTOS/CAN/platform are stubs; no MCU timing or physical acceptance.')
    (OUT/'report.json').write_text(json.dumps(report,indent=2));print(json.dumps(report,indent=2))


if __name__=='__main__':main()
