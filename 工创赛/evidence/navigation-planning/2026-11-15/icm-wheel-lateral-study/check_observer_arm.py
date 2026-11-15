from pathlib import Path
import ctypes, hashlib, importlib.util, json, re, struct, subprocess, sys, xml.etree.ElementTree as ET
R=Path.cwd(); U=R/'3.stm32f407工程/CHASSIS/USER'; E=Path(__file__).resolve().parent
T=R/'工创赛/tmp/icm-observer-arm-20261115'; T.mkdir(exist_ok=True)
B=Path('D:/Keil_v5/ARM/arm5_compiler/Bin')
def sha(p): return hashlib.sha256(p.read_bytes()).hexdigest()
def short(p):
    b=ctypes.create_unicode_buffer(32768); assert ctypes.windll.kernel32.GetShortPathNameW(str(p),b,32768); return b.value
def function(s,name):
    m=re.search(r'^(?:static\s+)?(?:void|uint32_t|int32_t|uint16_t)\s+'+name+r'\([^;]*?\)\s*\{',s,re.M); assert m,name
    e=s.index('{',m.start())+1;d=1
    while d:d+=(s[e]=='{')-(s[e]=='}');e+=1
    return s[m.start():e]
source=(U/'app_icm_observer.c').read_text('ascii')
serial=(U/'app_serial_runtime.c').read_bytes().decode('gbk')
prefix=r'''
#include <stdint.h>
#include <math.h>
#include <string.h>
#include <stdio.h>
#include <float.h>
#include "app_icm_observer.h"
#include "app_odometry.h"
#include "chassis_auto_test.h"
#include "heading_source.h"
#include "power.h"
static uint32_t locks,clock_ms,tx_busy,tx_calls,s_serial_tx_tick,s_serial_tx_valid=1U;
#define taskENTER_CRITICAL() (++locks)
#define taskEXIT_CRITICAL() (--locks)
#define APP_SERIAL_STATUS_PERIOD_MS 100U
static AppOdometryBaseline40 tw;
static ChassisAutoStatus tm;
static PowerCommandStatus tp;
static AppHeadingSource ty;
static char wire[4][384];static uint32_t wire_count;
static uint32_t xTaskGetTickCount(void){return clock_ms;}
void AppOdometry_ReadBaseline40(AppOdometryBaseline40 *v){*v=tw;}
uint32_t ChassisAuto_ReadStatus(ChassisAutoStatus *v){*v=tm;return 1U;}
void Power_ReadCommandStatus(PowerCommandStatus *v){*v=tp;}
uint32_t App_HeadingSourceRead(AppHeadingSource *v){*v=ty;return 1U;}
static uint32_t App_DebugSendString(const char *v){tx_calls++;if(tx_busy)return 0U;if(wire_count<4U)strcpy(wire[wire_count++],v);return 1U;}
'''
actual=re.sub(r'^#include[^\n]*','',source,flags=re.M)
telemetry='\n'.join(function(serial,n) for n in ['App_SerialCrc','App_SerialStatusDue','App_SerialDiagnosticWindow','App_ObserverMilli','App_IcmObserverTelemetry'])
tests=r'''
volatile uint32_t cases,assertions;
#define CHECK(x) do{assertions++;if(!(x))return __LINE__;}while(0)
#define ZERO(x) memset(&(x),0,sizeof(x))
static void reset_observer(void){
 ZERO(s_history);s_count=s_head=s_anchors=0U;ZERO(s_anchor_ms);ZERO(s_anchor_p);
 ZERO(s_gravity);ZERO(s_accel_bias);ZERO(s_gyro_bias);ZERO(s_accel_sum);ZERO(s_gyro_sum);
 s_cal_count=s_cal_ready=s_cal_blocked=s_ever_active=0U;
 s_seen=s_last_seq=s_last_edge=s_last_ms=s_last_skipped=s_last_races=s_last_errors=0U;
 s_session=s_reference=s_continuity=s_frame_seen=s_last_predict=s_predicted=s_gaps=s_gravity_gap=0U;
 ZERO(s_yaw);ZERO(s_value);ZERO(s_published);ZERO(tw);ZERO(tm);ZERO(tp);ZERO(ty);
 tw.anchor_valid=tw.pose.valid=1U;ty.healthy=1U;ty.valid_mask=3U;
}
static void sample(IcmRawStatus *raw,uint32_t advance){
 raw->sample_sequence+=advance;raw->edge_sequence+=advance;raw->sample_tick_ms+=5U;
 ty.sample_ms=ty.angle_ms=ty.rate_ms=raw->sample_tick_ms;
 tw.pose.sample_ms=raw->sample_tick_ms;
 AppIcmObserver_Step(raw);
}
static IcmRawStatus raw_status(void){
 IcmRawStatus raw;ZERO(raw);raw.state=ICM_RAW_READY;raw.fresh=1U;
 raw.accel_config=0x38U;raw.gyro_config=0x28U;raw.raw_accel[2]=-8192;
 return raw;
}
static float pfunc(float t,float a,float j){return 80.0f*t+0.5f*a*t*t+j*t*t*t/6.0f;}
static float vfunc(float t,float a,float j){return 80.0f+a*t+0.5f*j*t*t;}
static uint32_t kernel_tests(void){
 uint32_t b,d,h,k,j;const uint32_t deltas[4]={1U,22U,100U,200U},ages[3]={0U,53U,150U};
 const uint32_t bases[2]={1000U,0xFFFFFF00U};float aa,jj,t,t0,t1,now,p0,p1,vbound,pbound,expect;
 for(b=0;b<2;b++)for(d=0;d<4;d++)for(h=0;h<3;h++)for(j=0;j<3;j++) {
  reset_observer();aa=(float)((int)j-1)*150.0f;jj=(float)((int)j-1)*20.0f;
  for(k=0;k<128;k++){t=(float)(k*5U)*0.001f;Obs_Append(bases[b]+k*5U,aa+jj*t,-aa-jj*t);}
  s_anchors=2U;s_anchor_ms[1]=bases[b]+635U-ages[h];s_anchor_ms[0]=s_anchor_ms[1]-deltas[d];
  t0=(float)(s_anchor_ms[0]-bases[b])*0.001f;t1=(float)(s_anchor_ms[1]-bases[b])*0.001f;now=0.635f;
  p0=pfunc(t0,aa,jj);p1=pfunc(t1,aa,jj);
  s_anchor_p[0][0]=p0;s_anchor_p[1][0]=p1;
  s_anchor_p[0][1]=-p0;s_anchor_p[1][1]=-p1;
  CHECK(Obs_Predict(bases[b]+635U)==0U);
  vbound=0.0001f+8.0f*FLT_EPSILON*(fabsf(p0)+fabsf(p1))/((float)deltas[d]*0.001f);
  pbound=0.0001f+(float)ages[h]*0.001f*vbound;
  expect=vfunc(now,aa,jj);CHECK(fabsf(s_value.velocity_f_mm_s-expect)<vbound);
  CHECK(fabsf(s_value.velocity_r_mm_s+expect)<vbound);
  expect=pfunc(now,aa,jj);CHECK(fabsf(s_value.position_f_mm-expect)<pbound);
  CHECK(fabsf(s_value.position_r_mm+expect)<pbound);cases++;
 }
 s_anchor_ms[0]=s_anchor_ms[1];CHECK(Obs_Predict(bases[1]+635U)==ICM_OBS_WINDOW);cases++;
 s_anchor_ms[0]=s_anchor_ms[1]-22U;s_count=1U;CHECK(Obs_Predict(bases[1]+635U)==ICM_OBS_HISTORY);cases++;
 reset_observer();for(k=0;k<128;k++)Obs_Append(k*5U,0.0f,0.0f);
 s_anchors=2U;s_anchor_ms[0]=500U;s_anchor_ms[1]=600U;
 CHECK(Obs_Predict(635U)==0U && s_value.position_f_mm==0.0f && s_value.velocity_r_mm_s==0.0f);cases++;
 s_anchor_ms[0]=185U;s_anchor_ms[1]=435U;
 CHECK(Obs_Predict(635U)==0U);cases++;
 s_anchor_ms[0]=184U;CHECK(Obs_Predict(635U)==ICM_OBS_WINDOW);cases++;
 s_anchor_ms[0]=185U;CHECK(Obs_Predict(636U)==ICM_OBS_WINDOW);cases++;
 return 0U;
}
static uint32_t lifecycle_tests(void){
 IcmRawStatus raw;AppIcmObserverStatus o;uint32_t k,seq,head;
 reset_observer();raw=raw_status();raw.sample_tick_ms=0xFFFFFC00U;
 tw.reference_set_ms=raw.sample_tick_ms;
 for(k=0;k<200;k++)sample(&raw,1U);
 CHECK(s_cal_ready && s_cal_count==200U);CHECK(fabsf(s_accel_bias[2])<0.01f);
 for(k=0;k<12;k++)sample(&raw,1U);
 AppIcmObserver_Read(&o);CHECK(o.valid && o.reasons==0U);CHECK(o.position_f_mm==0.0f && o.position_r_mm==0.0f);
 head=s_head;seq=s_published.sequence;AppIcmObserver_Step(&raw);
 CHECK(head==s_head && seq==s_published.sequence);cases++;
 tm.session++;sample(&raw,1U);CHECK(s_count==1U && s_anchors==1U && !s_published.valid);cases++;
 tw.reference_set_ms=raw.sample_tick_ms+5U;sample(&raw,1U);CHECK(s_count==1U && s_anchors==1U);cases++;
 for(k=0;k<8;k++)sample(&raw,1U);
 raw.skipped_edges++;sample(&raw,2U);CHECK(!s_published.valid && (s_published.reasons&ICM_OBS_GAP));
 CHECK(s_count==1U && s_anchors==1U && s_gravity_gap);cases++;
 for(k=0;k<8;k++)sample(&raw,1U);CHECK(s_published.valid && (s_published.flags&ICM_OBS_GRAVITY_GAP));cases++;
 ty.angle_ms=raw.sample_tick_ms+50U;ty.yaw_deg=90.0f;
 raw.sample_tick_ms+=5U;raw.sample_sequence++;raw.edge_sequence++;tw.pose.sample_ms=raw.sample_tick_ms;
 AppIcmObserver_Step(&raw);CHECK(s_yaw.yaw_deg==0.0f);cases++;
 ty.healthy=0U;raw.sample_tick_ms+=5U;raw.sample_sequence++;raw.edge_sequence++;
 AppIcmObserver_Step(&raw);CHECK(!s_published.valid && s_count==0U && s_anchors==0U && s_published.reasons==ICM_OBS_HWT);cases++;
 ty.healthy=1U;for(k=0;k<8;k++)sample(&raw,1U);CHECK(s_published.valid);cases++;
 raw.fresh=0U;AppIcmObserver_Step(&raw);CHECK(!s_published.valid && s_published.reasons&ICM_OBS_RAW);cases++;
 reset_observer();raw=raw_status();for(k=0;k<10;k++)sample(&raw,1U);
 tm.active=1U;sample(&raw,1U);tm.active=0U;for(k=0;k<210;k++)sample(&raw,1U);
 CHECK(s_cal_blocked && !s_cal_ready && !s_cal_count && s_published.reasons==ICM_OBS_WAIT_CAL);cases++;
 reset_observer();raw=raw_status();raw.raw_accel[2]=-9000;
 raw.raw_gyro[0]=11;raw.raw_gyro[1]=-7;raw.raw_gyro[2]=5;
 for(k=0;k<212;k++)sample(&raw,1U);
 CHECK(s_cal_ready && s_published.valid && s_accel_bias[2]>900.0f);
 CHECK(fabsf(s_published.accel_f_mm_s2)<0.001f && fabsf(s_published.accel_r_mm_s2)<0.001f);
 CHECK(fabsf(s_gravity[0])<0.001f && fabsf(s_gravity[1])<0.001f);cases++;
 tw.pose.continuity_lost++;sample(&raw,1U);CHECK(!s_published.valid && s_published.reasons&ICM_OBS_REFERENCE);cases++;
 for(k=0;k<8;k++)sample(&raw,1U);CHECK(s_published.valid);cases++;
 tm.state=CHASSIS_AUTO_DONE;tm.active=0U;
 for(k=0;k<45;k++){
  raw.sample_tick_ms+=5U;raw.sample_sequence++;raw.edge_sequence++;
  ty.angle_ms=raw.sample_tick_ms;AppIcmObserver_Step(&raw);
 }
 CHECK(!s_published.valid && s_published.reasons&ICM_OBS_WINDOW);cases++;
 tw.pose.valid=0U;sample(&raw,1U);CHECK(!s_count && !s_anchors && !s_published.valid);cases++;

 return 0U;
}
static uint32_t wire_tests(void){
 uint32_t i,before;
 ZERO(s_published);s_published.reasons=ICM_OBS_WAIT_CAL;s_published.flags=ICM_OBS_NO_LEVER;
 s_published.wheel_age_ms=s_published.hwt_age_ms=0xFFFFFFFFU;
 s_serial_tx_tick=0U;clock_ms=40U;tx_busy=1U;before=tx_calls;App_IcmObserverTelemetry();
 for(i=41U;i<=55U;i++){clock_ms=i;App_IcmObserverTelemetry();}
 clock_ms=140U;s_serial_tx_tick=100U;App_IcmObserverTelemetry();
 CHECK(tx_calls==before+1U);cases++;
 tx_busy=0U;clock_ms=540U;s_serial_tx_tick=500U;App_IcmObserverTelemetry();
 CHECK(wire_count==1U);cases++;
 s_published.valid=1U;s_published.reasons=0U;s_published.flags=ICM_OBS_CAL|ICM_OBS_WHEEL|ICM_OBS_ACCEL|ICM_OBS_NO_LEVER;
 s_published.estimate_ms=0xFFFFFFF0U;s_published.t1_ms=0xFFFFFFD0U;s_published.t0_ms=0xFFFFFFBAU;
 s_published.wheel_age_ms=32U;s_published.hwt_age_ms=20U;s_published.cal_samples=200U;s_published.history_samples=128U;
 s_published.wheel_f_mm=s_published.position_f_mm=2100.0f;
 clock_ms=0xFFFFFFF0U;s_serial_tx_tick=clock_ms-40U;App_IcmObserverTelemetry();CHECK(wire_count==2U);cases++;
 s_published.estimate_ms=40U;s_published.t1_ms=8U;s_published.t0_ms=0xFFFFFFF2U;
 clock_ms=540U;s_serial_tx_tick=500U;App_IcmObserverTelemetry();CHECK(wire_count==3U);cases++;
 CHECK(App_ObserverMilli(2100.0f)==2100000L);CHECK(App_ObserverMilli(-2100.0f)==-2100000L);
 CHECK(locks==0U);cases++;
 return 0U;
}
__attribute__((section("i.test_entry"))) uint32_t test_entry(void){
 uint32_t r;r=kernel_tests();if(r)return r;r=lifecycle_tests();if(r)return r;
 return wire_tests();
}
'''
unit=prefix+'\n'+actual+'\n'+telemetry+'\n'+tests
(T/'unit.c').write_bytes(unit.encode('ascii'))
(T/'unit.sct').write_text('LR_ROM 0x08000000 0x100000 { ER_ROM 0x08000000 0x100000 { unit.o (i.test_entry, +First) *(+RO) } RW_RAM 0x20000000 0x20000 { *(+RW,+ZI) } }\n')
v=ET.parse(U.parents[1]/'gongxun-f407.uvprojx').getroot().find('./Targets/Target/TargetOption/TargetArmAds/Cads/VariousControls')
opts=['--cpu','Cortex-M3','--c99','--gnu','--split_sections','-O1','--diag_suppress=177,550','-DAPP_BASELINE40_ENABLE=1','-DVEHICLE_INTEGRATED_ENABLE=0','-DVEHICLE_STAGE1_ENABLE=0']+['-D'+d for d in v.findtext('Define').split(',')]
for p in v.findtext('IncludePath').split(';'):opts+=['-I',short((U.parents[1]/p).resolve())]
commands=[[str(B/'armcc.exe')]+opts+['-c','unit.c','-o','unit.o'],[str(B/'armlink.exe'),'--entry','test_entry','--scatter','unit.sct','unit.o','--library_type=microlib','-o','unit.axf']]
for i,command in enumerate(commands):
    q=subprocess.run(command,cwd=T,capture_output=True,timeout=90);(E/['observer_arm_compile.log','observer_arm_link.log'][i]).write_bytes(q.stdout+q.stderr)
    assert q.returncode==0,(q.stdout+q.stderr).decode('gbk',errors='replace')
sys.path.insert(0,str(R/'工创赛/tmp/position_alignment_deps'));import unicorn as uc
from unicorn import arm_const as reg
blob=(T/'unit.axf').read_bytes();elf=struct.unpack_from('<16sHHIIIIIHHHHHH',blob)
m=uc.Uc(uc.UC_ARCH_ARM,uc.UC_MODE_THUMB|uc.UC_MODE_MCLASS);m.mem_map(0x08000000,0x100000);m.mem_map(0x20000000,0x20000)
for i in range(elf[12]):
    _,kind,flags,address,offset,size,_,_,_,_=struct.unpack_from('<IIIIIIIIII',blob,elf[6]+i*elf[11])
    if flags&2 and size:m.mem_write(address,b'\0'*size if kind==8 else blob[offset:offset+size])
stop=0x080FF000;m.reg_write(reg.UC_ARM_REG_SP,0x2001FFF0);m.reg_write(reg.UC_ARM_REG_LR,stop|1)
m.emu_start(elf[4]|1,stop,timeout=60000000,count=150000000)
assert m.reg_read(reg.UC_ARM_REG_PC)==stop,'execution incomplete'
result=m.reg_read(reg.UC_ARM_REG_R0)
if result:
    sy=subprocess.check_output([str(B/'fromelf.exe'),'--text','-s','unit.axf'],cwd=T).decode(errors='replace')
    for name,size,fmt in [('s_cal_ready',4,'<I'),('s_cal_count',4,'<I'),('s_accel_bias',12,'<3f')]:
        ad=int(re.search(r'\b'+name+r'\s+(0x[0-9a-fA-F]+)',sy)[1],16)
        print(name,struct.unpack(fmt,m.mem_read(ad,size)))
assert result==0,f'actual C assertion failed unit.c line {result}'
symbols=subprocess.check_output([str(B/'fromelf.exe'),'--text','-s','unit.axf'],cwd=T).decode(errors='replace')
def addr(n):return int(re.search(r'\b'+n+r'\s+(0x[0-9a-fA-F]+)',symbols)[1],16)
def value(n):return struct.unpack('<I',m.mem_read(addr(n),4))[0]
sys.path.insert(0,str(R/'工创赛/scripts'));import vehicle_uart_capture as parser
wires=[]
for i in range(value('wire_count')):
    line=bytes(m.mem_read(addr('wire')+384*i,384)).split(b'\0')[0]
    decoded=parser.parse_icm_observer(line);assert decoded is not None
    wires.append({'wire':line.decode('ascii'),'decoded':decoded});assert len(line)<384
assert wires[0]['decoded']['valid']==0 and wires[0]['decoded']['wheel_age_ms']==0xffffffff
assert wires[1]['decoded']['position_f_milli_mm']==2100000
assert wires[2]['decoded']['t0_ms']>wires[2]['decoded']['t1_ms']
for line in [wires[0]['wire'].encode().replace(b'IO1',b'IO2',1),wires[0]['wire'].encode()[:-4]+b'0000']:
    try: val=parser.parse_icm_observer(line)
    except ValueError:pass
    else: assert val is None
report={'status':'PASS_ACTUAL_ARM_C_AND_PARSER','cases':value('cases'),'assertions':value('assertions'),
 'source_sha256':{n:sha(U/n) for n in ['app_icm_observer.c','app_icm_observer.h','app_serial_runtime.c','main.c']},
 'parser_sha256':sha(R/'工创赛/scripts/vehicle_uart_capture.py'),'fixture_sha256':sha(T/'unit.c'),
 'fixture_axf_sha256':sha(T/'unit.axf'),'runner_sha256':sha(Path(__file__)),'commands':commands,
 'wire_cases':wires,'hardware_accessed':False,'limits':'Actual observer and IO1 functions, synthetic read-only getter/TX/clock stubs; Cortex-M3 soft float ARM instructions under Unicorn; not RTOS timing, stack watermark, SPI, UART or ground accuracy.'}
(E/'observer_arm_summary.json').write_text(json.dumps(report,indent=2),'utf8')
(E/'observer_arm_unit.c').write_bytes((T/'unit.c').read_bytes())
print(json.dumps({k:report[k] for k in ['status','cases','assertions','fixture_axf_sha256']}))
