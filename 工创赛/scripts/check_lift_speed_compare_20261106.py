"""Actual comparison controller tests with driver/clock stubs, no hardware."""
from pathlib import Path
import hashlib
import json
import subprocess

ROOT = Path(__file__).resolve().parents[1]
STAGE = ROOT / 'tmp/lift_speed_compare_20261106/project'
OUT = ROOT / 'evidence/integration/2026-11-06/lift-speed-compare'

FIXTURE = r'''
#include <stdio.h>
#include <stdlib.h>
#include <stdint.h>
#include <string.h>
#include <math.h>
#include "lift_ruler_calibration.h"
static const char*test;
static uint32_t now,move_at,home_at,home_duration;
static unsigned checks,failures,moves,homes,stops,aborts,forbidden,enabled,engine;
static double motor,origin,goal,move_speed,move_accel,home_origin;
static void ck(unsigned ok,unsigned line){checks++;if(!ok){failures++;printf("FAIL %s line%u\n",test,line);}}
#define CK(x) ck((x),__LINE__)
static uint32_t scale=100;
static void update(void){
 if(engine==1){
  double distance=fabs(goal-origin),v=move_speed*6,a=move_accel*6,ta=v/a,tc,t=(uint32_t)(now-move_at)*.001,x;
  if(distance<v*ta){ta=sqrt(distance/a);v=a*ta;tc=0;}else tc=distance/v-ta;
  if(t<ta)x=.5*a*t*t;
  else if(t<ta+tc)x=.5*a*ta*ta+v*(t-ta);
  else if(t<2*ta+tc){double q=2*ta+tc-t;x=distance-.5*a*q*q;}
  else{x=distance;engine=0;}
  motor=origin+(goal>=origin?x:-x);
 }else if(engine==2){
  double u=(double)(uint32_t)(now-home_at)/home_duration;
  if(u>=1){motor=-12465;engine=0;}else motor=home_origin+(-12465-home_origin)*u;
 }
}
static double rpm_now(void){
 if(engine==2)return 60;
 if(engine==1){
  double distance=fabs(goal-origin),v=move_speed*6,a=move_accel*6,ta=v/a,tc,t=(uint32_t)(now-move_at)*.001;
  if(distance<v*ta){ta=sqrt(distance/a);v=a*ta;tc=0;}else tc=distance/v-ta;
  if(t<ta)return a*t/6;
  if(t<ta+tc)return v/6;
  if(t<2*ta+tc)return a*(2*ta+tc-t)/6;
 }return 0;
}
static void rawpos(uint8_t*r,double p){
 int64_t signed_raw=(int64_t)llround(p*scale);uint32_t v=(uint32_t)llabs(signed_raw);
 r[1]=signed_raw<0;r[2]=v>>24;r[3]=v>>16;r[4]=v>>8;r[5]=v;
}
static void home_params(uint8_t*r){
 const uint8_t value[17]={0x22,0,1,0,60,0,0,0x9c,0x40,1,0x2c,3,0x20,0,60,0,0x6b};
 memcpy(r,value,17);if(!strcmp(test,"wrong_menu"))r[4]=30;
}
static uint32_t bus(void*context,const uint8_t*q,uint32_t n,uint32_t rn,uint8_t*r){
 LiftRulerController*c=context;uint32_t u;double p;update();memset(r,0,rn);r[0]=q[0];r[rn-1]=0x6b;
 if(!strcmp(test,"read_loss") && moves && q[0]!=0xfe && q[0]!=0x9c)return 0;
 if(n==2){
  switch(q[0]){
   case 0x1a:r[2]=scale==100?0x80:0;if(!strcmp(test,"bad_options"))r[2]|=2;break;
   case 0x22:CK(rn==17);home_params(r);break;
   case 0x36:
    p=motor;
    if(!strcmp(test,"position_frozen") && moves && !homes)p=origin;
    if(!strcmp(test,"overshoot") && moves && !homes)p=goal-100;
    rawpos(r,p);
    if(!strcmp(test,"bad_position") || (!strcmp(test,"bad_home_position") && c->phase==26))r[1]=2;
    break;
   case 0x31:
    u=(uint32_t)((12345LL+(int64_t)llround(motor*65536/360))&65535U);
    if(!strcmp(test,"bad_encoder") && moves)u=(u+1000)&65535U;
    r[1]=u>>8;r[2]=u;break;
   case 0x3a:r[1]=(enabled?1:0)|(moves && !engine?2:0);
    if(!strcmp(test,"disabled") && moves)r[1]&=~1U;
    if(!strcmp(test,"protection") && moves)r[1]|=4U;break;
   case 0x35:u=(uint32_t)llround(rpm_now()*10);
    if(!strcmp(test,"initial_moving") && !moves)u=5;
    if(!strcmp(test,"home_speed_nonzero") && homes && !engine)u=5;
    if(!strcmp(test,"overspeed") && moves && !homes)u=5000;
    r[1]=0;r[2]=u>>8;r[3]=u;break;
   case 0x33:rawpos(r,goal+(!strcmp(test,"wrong_target")?10:0));break;
   case 0x3b:r[1]=engine==2?7:3;
    if(!strcmp(test,"home_stale_idle"))r[1]=3;
    if(!strcmp(test,"home_failure") && homes)r[1]|=8;break;
   default:forbidden++;return 0;
  }return 1;
 }
 if(n==5 && q[0]==0xf3){CK(LiftRuler_CommandAllowed(c,q,n));enabled=1;r[1]=2;return 1;}
 if(n==15 && q[0]==0xfd){
  CK(LiftRuler_CommandAllowed(c,q,n));CK(q[12]==2 && !q[13] && q[14]==0x6b);
  u=((uint32_t)q[8]<<24)|((uint32_t)q[9]<<16)|((uint32_t)q[10]<<8)|q[11];
  move_speed=(((unsigned)q[6]<<8)|q[7])*.1;move_accel=((unsigned)q[2]<<8)|q[3];
  CK(move_accel==(((unsigned)q[4]<<8)|q[5]));
  CK((c->operation==0 && move_speed==300 && move_accel==60) ||
     (c->operation>0 && fabs(move_speed-1846.4)<1e-6 && move_accel==45055));
  CK(q[1]==(c->operation==1?0:1));
  origin=motor;goal=motor+(q[1]?-1:1)*(double)u/scale;move_at=now;engine=1;moves++;r[1]=2;
  if(!strcmp(test,"move_ack_lost"))return 0;return 1;
 }
 if(n==4 && q[0]==0x9a){
  CK(LiftRuler_CommandAllowed(c,q,n));CK(q[1]==2 && !q[2] && c->home_params_valid);
  homes++;home_origin=motor;home_duration=c->operation==0?5000:1000;home_at=now;engine=2;r[1]=2;
  if(!strcmp(test,"home_ack_lost"))return 0;return 1;
 }
 if(n==3 && q[0]==0x9c){aborts++;if(!strcmp(test,"abort_ack_missing"))return 0;engine=0;r[1]=2;return 1;}
 if(n==4 && q[0]==0xfe){stops++;if(!strcmp(test,"stop_ack_missing"))return 0;engine=0;r[1]=2;return 1;}
 forbidden++;return 0;
}
static void reset_case(const char*name){test=name;now=0;checks+=0;moves=homes=stops=aborts=forbidden=enabled=engine=0;motor=0;scale=!strcmp(name,"unscaled")?10:100;}
static void run(const char*name){
 LiftRulerController c;unsigned i,k,cancelled,old_moves,old_homes;uint32_t base;reset_case(name);LiftRuler_Init(&c);
 for(i=0;i<10;i++)LiftRuler_Service(&c,i*20,1,1,1,bus,&c);
 CK(c.state==LRC_IDLE && !c.vm_request && !moves && !homes);
 CK(!LiftRuler_Start(&c,now,0));
 for(k=0;k<3;k++){
  base=now;cancelled=0;CK(LiftRuler_Start(&c,now,1));CK(!LiftRuler_Start(&c,now,1));
  for(i=0;i<1700;i++){
   uint32_t power=1,healthy=1;now=base+i*20;
   if(!strcmp(name,"service_gap") && moves && i>60 && i<90)continue;
   if(!strcmp(name,"power_loss") && moves)power=0;
   if(!strcmp(name,"guard_lost") && moves)healthy=0;
   if(!cancelled &&
      ((!strcmp(name,"cancel_pre") || !strcmp(name,"stop_ack_missing")) && moves ||
       (!strcmp(name,"cancel_home") || !strcmp(name,"abort_ack_missing")) && homes)){
    LiftRuler_Stop(&c,now);cancelled=1;
   }
   LiftRuler_Service(&c,now,1,power,healthy,bus,&c);
   if(c.state>=LRC_DONE && !c.stop_pending)break;
  }
  CK(c.vm_request==1 && !forbidden);
  if(!strcmp(name,"normal") || !strcmp(name,"unscaled")){
   CK(c.state==LRC_DONE && c.operation==k+1 && c.samples==k+1 && c.top_valid);
   CK(moves==k+1 && homes==(k==0?1:k==1?1:2));
   if(k!=1)CK(c.home_seen_running && c.home_reported && fabs((double)c.top_position_raw/scale+12465)<.1);
   old_moves=moves;old_homes=homes;
   for(i=0;i<20;i++)LiftRuler_Service(&c,now+i*20,1,1,1,bus,&c);
   CK(moves==old_moves && homes==old_homes && c.vm_request==1);
   now+=500;continue;
  }
  CK(c.state!=LRC_DONE && !c.top_valid && !c.samples && moves<=1 && homes<=1);
  if(!strcmp(name,"home_stale_idle") || !strcmp(name,"home_speed_nonzero")){
   CK(c.state==LRC_MOVING && c.home_started && !c.home_reported);LiftRuler_Stop(&c,now);
   LiftRuler_Service(&c,now+50,1,1,1,bus,&c);CK(c.vm_request==1 && c.stop_confirmed);
  }else{
   CK(c.cause && !LiftRuler_Start(&c,now+20,1));
   if(!strcmp(name,"stop_ack_missing") || !strcmp(name,"abort_ack_missing"))CK(c.stop_pending && c.stop_unconfirmed);
   else CK(c.stop_confirmed && !c.stop_pending);
   if(homes)CK(aborts>0);
  }break;
 }
 if(!strcmp(name,"normal") || !strcmp(name,"unscaled"))CK(!LiftRuler_Start(&c,now+20,1) && c.vm_request==1);
}
int main(void){
 const char*cases[]={"normal","unscaled","wrong_menu","bad_options","bad_position","initial_moving", ...
 "move_ack_lost","wrong_target","position_frozen","overshoot","bad_encoder","disabled","protection", ...
 "overspeed","read_loss","power_loss","guard_lost","service_gap","cancel_pre","cancel_home", ...
 "home_ack_lost","home_failure","home_stale_idle","home_speed_nonzero","bad_home_position", ...
 "stop_ack_missing","abort_ack_missing"};unsigned i;
 for(i=0;i<sizeof(cases)/sizeof(cases[0]);i++)run(cases[i]);
 printf("COMPARE cases=%u checks=%u failures=%u\n",(unsigned)(sizeof(cases)/sizeof(cases[0])),checks,failures);
 return failures?1:0;
}
'''.replace(', ...', ',')


def main():
    assert json.loads((OUT / 'matlab-result.json').read_text(encoding='utf-8'))['passed']
    src, exe = OUT / 'compare-host.c', OUT / 'compare-host.exe'
    src.write_text(FIXTURE, encoding='utf-8')
    command = ['C:/mingw/mingw64/bin/gcc.exe', '-std=c99', '-Wall', '-Wextra', '-Werror',
               '-Wno-misleading-indentation', '-Wno-parentheses', '-I' + str(STAGE / 'CHASSIS/USER'),
               str(src), str(STAGE / 'CHASSIS/USER/lift_ruler_calibration.c'), '-lm', '-o', str(exe)]
    built = subprocess.run(command, capture_output=True, text=True)
    (OUT / 'core-host-build.log').write_text(built.stdout + built.stderr, encoding='utf-8')
    assert built.returncode == 0, built.stderr
    ran = subprocess.run([str(exe)], capture_output=True, text=True)
    (OUT / 'core-host-check.log').write_text(ran.stdout + ran.stderr, encoding='utf-8')
    print(ran.stdout)
    assert ran.returncode == 0, ran.stdout + ran.stderr
    report = {'passed': True, 'host_result': ran.stdout.strip(),
              'scope': 'actual C controller; fake driver, clocks and mechanics',
              'sha256': {n: hashlib.sha256((STAGE / 'CHASSIS/USER' / n).read_bytes()).hexdigest()
                         for n in ['lift_ruler_calibration.c', 'lift_ruler_calibration.h']},
              'hardware_operated': False, 'firmware_downloaded': False}
    (OUT / 'core-checks.json').write_text(json.dumps(report, indent=2), encoding='utf-8')


if __name__ == '__main__':
    main()
