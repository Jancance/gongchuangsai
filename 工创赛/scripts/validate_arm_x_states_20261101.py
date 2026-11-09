from pathlib import Path
import json, subprocess, shutil
R = Path(__file__).resolve().parents[1]
S = R/'tmp/arm_x_20261101/project'
E = R/'evidence/p2-arm/2026-11-01/arm-x'
H = R/'tmp/arm_x_20261101/state-host'
H.mkdir(parents=True, exist_ok=True)
for f in ['stm32f4xx.h', 'stm32f4xx_can.h', 'FreeRTOS.h', 'task.h']:
    shutil.copy2(R/'tmp/lift_down_step_20261030/host-check'/f, H/f)
with (H/'FreeRTOS.h').open('a') as f:
    f.write('\n#define portTICK_PERIOD_MS 1U\n')
can = (R/'tmp/lift_down_step_20261030/host-check/can.h').read_text()
can += '\n' + (R/'tmp/extension_mm_entry_20261101/host-check/can.h').read_text()
can += '\nuint32_t CAN1_ArmXConfigure(uint8_t,uint8_t,uint32_t);\nuint32_t CAN1_ArmXBuildMove(uint8_t,uint8_t,uint32_t,uint16_t,uint16_t,uint16_t,uint8_t*);\n'
(H/'can.h').write_text(can, encoding='utf-8')
model = r"""
#include <assert.h>
#include <stdint.h>
#include <stdio.h>
#include <string.h>
#include "lift.h"
#include "lift_home.h"
#include "lift_config.h"
#include "arm_x.h"
#include "FreeRTOS.h"
CAN_TypeDef fake_can={0};
static unsigned now,moves,stops,enabled=1,moving,options=4,speed_raw,last_angle;
static int32_t pos;
static ArmXEncoding enc;
volatile uint32_t g_lift_home_request,g_lift_home_state,g_lift_home_result;
uint32_t LiftHome_IsBusy(void){return 0;}
void LiftHome_Service(uint32_t ready){(void)ready;}
TickType_t xTaskGetTickCount(void){return now;}
uint32_t CAN1_ReadVersion(uint8_t a,CanRxMsg*r){assert(a==5);memset(r,0,sizeof(*r));return 1;}
uint32_t CAN1_ReadStatus(uint8_t a,CanRxMsg*r){assert(a==5);r->Data[1]=(enabled?1:0)|(moving?0:2);return 1;}
uint32_t CAN1_ReadMotorOptions(uint8_t a,CanRxMsg*r){assert(a==5);r->Data[2]=(uint8_t)options;return 1;}
uint32_t CAN1_ReadSpeed(uint8_t a,CanRxMsg*r,int32_t*v){assert(a==5);(void)r;*v=(int32_t)speed_raw;return 1;}
uint32_t CAN1_ReadPosition(uint8_t a,CanRxMsg*r,uint8_t*sign,uint32_t*value){assert(a==5);(void)r;*sign=pos<0;*value=(uint32_t)(pos<0?-pos:pos);return 1;}
uint32_t CAN1_ReadLiftHomeParameters(uint8_t*p,CanRxMsg*r){(void)p;(void)r;return 5;}
uint32_t CAN1_ArmXConfigure(uint8_t a,uint8_t o,uint32_t ms){assert(a==5);return ArmX_Configure(o,ms,&enc);}
uint32_t CAN1_ArmXBuildMove(uint8_t a,uint8_t d,uint32_t p,uint16_t v,uint16_t ac,uint16_t dc,uint8_t*out){
 assert(a==5);return ArmX_BuildMove(&enc,d,p,d?LIFT_UP_MAX_PULSES:LIFT_DOWN_MAX_PULSES,v,ac,dc,out);
}
uint32_t CAN1_LiftExchange(const uint8_t*q,uint8_t n,CanRxMsg*r){
 memset(r,0,sizeof(*r));r->Data[1]=2;
 if(q[0]==0xFE){assert(n==4);stops++;moving=speed_raw=0;return 1;}
 if(q[0]==0xF3){assert(n==5);enabled=1;return 1;}
 assert(n==15&&q[0]==0xFD&&q[2]==0xAF&&q[3]==0xFF&&q[6]==0x48&&q[7]==0x20&&q[12]==2&&q[14]==0x6B);
 unsigned angle=((unsigned)q[8]<<24)|((unsigned)q[9]<<16)|((unsigned)q[10]<<8)|q[11];
 last_angle=angle;moves++;
 if(options&128)angle/=10;
 pos+=q[1]?-(int32_t)angle:(int32_t)angle;
 return 1;
}
static void tick(unsigned n){while(n--){now+=10;Lift_Service(1);}}
int main(int argc,char**argv){
 (void)argc;const char*s=argv[1];assert(g_lift_x_legacy_mstep==16);
 if(!strcmp(s,"scaled"))options=132;
 if(!strcmp(s,"motor09"))options=5;
 if(!strcmp(s,"wrong_firmware"))options=6;
 if(!strcmp(s,"readonly_emm"))options=6;
 if(!strcmp(s,"missing_steps"))g_lift_x_legacy_mstep=0;
 tick(120);assert(g_lift_state==4&&moves==0);
 if(!strcmp(s,"readonly_x")||!strcmp(s,"readonly_emm")){
  g_lift_read_request=2;tick(1);
  assert(moves==0&&g_lift_state==4&&g_lift_options_result==1);
  assert(g_lift_fw_type==(!strcmp(s,"readonly_emm")?1U:0U));puts(s);return 0;
 }
 if(!strcmp(s,"boot")){tick(200);assert(moves==0);puts(s);return 0;}
 if(!strcmp(s,"enable"))enabled=0;
 unsigned up=!strcmp(s,"up");g_lift_down_pulses=g_lift_up_pulses=32000;
 if(!strcmp(s,"speed_fraction"))speed_raw=5;
 if(!strcmp(s,"stop"))moving=1;
 g_lift_motion_request=up?2:1;tick(80);
 if(!strcmp(s,"wrong_firmware")||!strcmp(s,"missing_steps")){
  assert(moves==0&&g_lift_motion_state==9&&g_lift_motion_result==4);
 }else if(!strcmp(s,"speed_fraction")){
  assert(g_lift_motion_state==6&&g_lift_speed_deci_rpm==5&&g_lift_speed_rpm==0);
  g_lift_motion_request=3;tick(1);assert(g_lift_motion_state==8&&stops==1);
 }else if(!strcmp(s,"stop")){
  g_lift_motion_request=3;tick(1);assert(g_lift_motion_state==8&&stops==1);
 }else{
  assert(g_lift_motion_state==7&&g_lift_motion_result==1&&moves==1);
  unsigned expected=(options&1)?18000:36000;if(options&128)expected*=10;
  assert(last_angle==expected);
  int64_t expected_delta=(int64_t)(expected/((options&128)?10:1));
  assert(g_lift_delta_raw==(up?-expected_delta:expected_delta));
 }
 puts(s);return 0;
}
"""
(H/'lift.c').write_text(model, encoding='utf-8')
gcc = 'C:/mingw/mingw64/bin/gcc.exe'
base = [gcc, '-std=c99', '-Wall', '-Wextra', '-Werror', '-I'+str(H),
        '-I'+str(S/'SYSTEM/CAN'), '-I'+str(S/'HARDWARE/LIFT'), '-I'+str(S/'HARDWARE/EXTENSION')]
z = subprocess.run(base+[str(H/'lift.c'), str(S/'HARDWARE/LIFT/lift.c'),
                   str(S/'HARDWARE/LIFT/lift_protocol.c'), '-o', str(H/'lift.exe')],
                   capture_output=True, text=True)
assert z.returncode == 0, z.stderr
liftcases = ['boot','down','up','scaled','motor09','wrong_firmware','missing_steps','speed_fraction','stop','enable','readonly_x','readonly_emm']
logs = []
for c in liftcases:
    z = subprocess.run([str(H/'lift.exe'), c], capture_output=True, text=True, timeout=5)
    assert z.returncode == 0, (c, z.stderr)
    logs.append('lift '+z.stdout)
model = (R/'tmp/extension_mm_entry_20261101/host-check/mm-model.c').read_text(encoding='utf-8')
model = model.replace('#include "extension.h"', '''#include "extension.h"
#include "extension_config.h"
#include "arm_x.h"
static ArmXEncoding enc;
uint32_t CAN1_ArmXConfigure(uint8_t a,uint8_t o,uint32_t ms){assert(a==6);return ArmX_Configure(o,ms,&enc);}
uint32_t CAN1_ArmXBuildMove(uint8_t a,uint8_t d,uint32_t p,uint16_t v,uint16_t ac,uint16_t dc,uint8_t*out){
 assert(a==6);return ArmX_BuildMove(&enc,d,p,EXTENSION_JOG_PULSES,v,ac,dc,out);
}''')
model = model.replace('is("bad_options")?0:6', 'is("bad_options")?6:4')
a = model.index(' assert(len==12&&b[0]==0xFD')
b = model.index(' if(is("move_fail"))', a)
model = model[:a]+''' assert(len==15&&b[0]==0xFD&&b[1]==((is("negative")||is("mm_extend"))?1:0));
 assert(b[2]==0xAF&&b[3]==0xFF&&b[4]==0xAF&&b[5]==0xFF&&b[6]==4&&b[7]==0x24);
 uint32_t expected_units;assert(ArmX_AngleUnits(&enc,expected_step,&expected_units));
 assert((((uint32_t)b[8]<<24)|((uint32_t)b[9]<<16)|((uint32_t)b[10]<<8)|b[11])==expected_units);
 assert(b[12]==2&&b[13]==0&&b[14]==0x6B);moves++;
'''+model[b:]
model = model.replace('assert(argc==2);scenario=argv[1];', 'assert(argc==2);scenario=argv[1];assert(g_extension_x_legacy_mstep==16);')
model = model.replace('assert(g_extension_state==4&&reads==3&&moves==0);', 'assert(g_extension_state==4&&reads==3&&moves==0&&g_extension_fw_type==0);')
(H/'extension.c').write_text(model, encoding='utf-8')
z = subprocess.run(base+[str(H/'extension.c'), str(S/'HARDWARE/EXTENSION/extension.c'),
                   '-lm', '-o', str(H/'extension.exe')], capture_output=True, text=True)
assert z.returncode == 0, z.stderr
extcases = ['positive','negative','enable','bad_options','ack_only','no_stop','fault','protect',
            'abort','stop_fail','readonly','gap','signed','mm_default','mm_extend','mm_25',
            'mm_max','mm_round','mm_min_round','mm_zero','mm_negative','mm_nan','mm_inf',
            'mm_too_far','mm_too_small','mm_bad_units','mm_busy_edit','mm_busy_invalid_edit',
            'mm_stop_override','mm_fault','mm_protect','mm_timeout','mm_interlock','mm_power_loss']
for c in extcases:
    z = subprocess.run([str(H/'extension.exe'), c], capture_output=True, text=True, timeout=5)
    assert z.returncode == 0, (c, z.stdout, z.stderr)
    logs.append('extension '+z.stdout)
(E/'state-check.log').write_text('\n'.join(logs), encoding='utf-8')
m = json.loads((E/'manifest.json').read_text(encoding='utf-8'))
m.update(lift_state_cases=liftcases, extension_state_cases=extcases, state_checks_pass=True)
(E/'manifest.json').write_text(json.dumps(m, ensure_ascii=False, indent=2), encoding='utf-8')
print(str(len(liftcases)+len(extcases))+' actual lift/extension X-state cases passed')
