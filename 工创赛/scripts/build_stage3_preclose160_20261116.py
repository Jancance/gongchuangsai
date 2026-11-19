"""Actual C regressions and dual Keil build; no hardware."""
import json
from pathlib import Path
import build_stage3_teach_mode_20261116 as old
from prepare_stage3_preclose160_20261116 import R,BASE,P,E
from build_stopped5_fastalign_20261116 import block

def dispatch_fixture():
 text=(P/'CHASSIS/USER/integration10_complete.inc').read_text(encoding='utf-8')
 admission=next(x.strip() for x in text.splitlines() if 'request>15U' in x)
 dispatch=block(text,'if(request==5U || request==6U || request==17U)')
 finish=text[text.index(' case 5U:case 6U:case 17U:'):text.index(' case 7U:',text.index(' case 5U:case 6U:case 17U:'))]
 return r'''
#include <stdio.h>
#include <stdlib.h>
typedef unsigned uint32_t;
static unsigned checks,request,g_it_rejected,g_it_state,owned,calls,accepted,g_it_elapsed_ms,done;
static float angle;
#define CHECK(x) do{checks++;if(!(x)){printf("FAIL %u %s\n",__LINE__,#x);exit(1);}}while(0)
static unsigned Stage2Fast_IsActive(void){return 0;}
static unsigned SourceStage2_IsActive(void){return 0;}
static unsigned Stage3_Owns(void){return owned;}
static unsigned jiazhua(float a){calls++;angle=a;return accepted;}
static void it_done(void){done++;g_it_state=2;}
static void admission(void){
'''+admission+r'''
}
static void dispatch(void){
'''+dispatch+r'''
}
static void finish(void){switch(request){
'''+finish+r'''
default:break;}}
int main(void){
 for(owned=0;owned<2;owned++)for(request=5;request<=18;request++){
  g_it_state=2;g_it_rejected=0;admission();
  CHECK((g_it_rejected!=0)==(request>15 && !(request==17 && owned)));
 }
 for(unsigned i=0;i<3;i++)for(accepted=0;accepted<2;accepted++){
  request=(unsigned[]){5,6,17}[i];g_it_state=2;g_it_rejected=0;unsigned before=calls;dispatch();
  CHECK(calls==before+1 && angle==(i==0?140:i==1?210:160));
  CHECK(accepted?(g_it_state==1 && !g_it_rejected):(g_it_state==2 && g_it_rejected==7));
  if(accepted){done=0;g_it_elapsed_ms=599;finish();CHECK(!done && g_it_state==1);
   g_it_elapsed_ms=600;finish();CHECK(done==1 && g_it_state==2);}
 }
 printf("PASS actual IT admission/5/6/17: %u checks; source17 rejected, PWM failure7, original600ms\n",checks);return 0;
}
'''

def main():
 proof=json.loads((E/'matlab-result.json').read_text());assert proof['passed']
 fixture=E/'actual-preclose-it.c';fixture.write_text(dispatch_fixture())
 old.P=P;old.E=E
 it_result=old.run('actual-preclose-it',fixture)
 # Retain historical fixtures and adapt only the changed owner phase numbers.
 owner=(R/'tests/stage3_teach_mode_20261116.c').read_text()
 owner=owner.replace('static unsigned motions,commands,grips,releases,slots,dm_moves,x_moves,z_moves;',
  'static unsigned motions,commands,grips,releases,slots,dm_moves,x_moves,z_moves,precloses;')
 owner=owner.replace(' if(g_it_action==5)grips++;',' if(g_it_action==17){CHECK(g_stage3_phase==4 && g_vehicle_lift_current_deg==0 && g_dm_can2_logical_position_deg==36.5f);precloses++;}\n if(g_it_action==5){CHECK(precloses==grips+1);grips++;}')
 owner=owner.replace('commands=motions=grips=releases=slots=dm_moves=x_moves=z_moves=0;', 'commands=motions=grips=releases=slots=dm_moves=x_moves=z_moves=precloses=0;')
 path=E/'stage3-preclose-owner.c';path.write_text(owner)
 lrc=(R/'tests/stage3_deep_z_owner_lrc_20261116.c').read_text().replace('{0,1,6,11}','{0,1,7,12}')
 lp=E/'stage3-preclose-owner-lrc.c';lp.write_text(lrc)
 # Use existing build driver with generated fixtures; firmware includes unchanged.
 driver=Path(old.__file__).read_text()
 driver=driver.replace("from prepare_stage3_teach_mode_20261116 import R,BASE,P,E", "from prepare_stage3_preclose160_20261116 import R,BASE,P,E")
 driver=driver.replace("'tests/stage3_teach_mode_20261116.c'",repr(str(path)))
 driver=driver.replace("'tests/stage3_deep_z_owner_lrc_20261116.c'",repr(str(lp)))
 scope={'__name__':'preclose_build','__file__':str(Path(__file__))};exec(compile(driver,__file__,'exec'),scope);scope['main']()
 path=E/'candidate.json';c=json.loads(path.read_text());c.update(preclose_request=17,preclose_angle=160,software_settle_ms=600,servo_angle_feedback=False)
 path.write_text(json.dumps(c,indent=2)+'\n')
 path=E/'checks.json';checks=json.loads(path.read_text())
 checks['actual_c']['preclose_it']=it_result
 checks['matlab_timeline']={'original_prevalidation':'2026-11-16 05:57:37',
  'evidence':'Original result read in session before candidate inc modification05:58:37; original JSON subsequently overwritten by rerun',
  'rerun':proof['when'],'rerun_console':'matlab-console.log'}
 path.write_text(json.dumps(checks,indent=2)+'\n')

if __name__=='__main__':main()
