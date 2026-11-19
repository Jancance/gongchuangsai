"""Actual state2 branch comparison and exact one-line scope audit; dual Keil."""
from pathlib import Path
import json,hashlib,subprocess,datetime
from prepare_stage2_step20_20261117 import R,BASE,P,E
def sha(p):return hashlib.sha256(p.read_bytes()).hexdigest()
def block(s,signature):
 start=s.index(signature);i=s.index('{',start);depth=1;end=i+1
 while depth:
  if s[end]=='{':depth+=1
  elif s[end]=='}':depth-=1
  end+=1
 return s[start:end]
def test():
 actual=(P/'CHASSIS/USER/source_mcu_cycle.inc').read_text(encoding='utf-8');prior=(BASE/'CHASSIS/USER/source_mcu_cycle.inc').read_text(encoding='utf-8')
 sig=' if(g_source_cycle_state==2U){'
 a=block(actual[actual.index('static void SourceCycle_Service('):],sig)
 b=block(prior[prior.index('static void SourceCycle_Service('):],sig)
 pose=block(actual,'static void SC_Pose(void)')
 prefix=r'''
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <math.h>
static unsigned checks,fresh=1;
#define CHECK(x) do{checks++;if(!(x)){printf("FAIL %u %s\n",__LINE__,#x);exit(1);}}while(0)
static uint32_t g_source_cycle_state,g_source_cycle_error_milli_mm,g_source_align_chain_active;
static uint32_t g_source_body_step_request,g_source_align_direction,sc_step_sequence,g_source_align_command_seq;
static uint32_t g_source_cycle_body_steps,g_source_cycle_pose,sc_valid,sc_frame;
static int32_t g_source_cycle_right_milli_mm,g_source_cycle_forward_milli_mm;
static float g_source_align_distance_mm;
static uint32_t SC_Fresh(uint32_t now){(void)now;return fresh && sc_valid;}
'''+pose+'\nstatic void actual_step20(uint32_t now){'+a+'}\nstatic void baseline_step30(uint32_t now){'+b+'}\n'
 suffix=r'''
static void setup(int32_t r,int32_t f,unsigned error){g_source_cycle_state=2;g_source_cycle_error_milli_mm=error;
 g_source_cycle_right_milli_mm=r;g_source_cycle_forward_milli_mm=f;g_source_body_step_request=0;g_source_align_distance_mm=0;
 g_source_align_direction=99;g_source_cycle_body_steps=0;g_source_cycle_pose=10;sc_valid=sc_frame=1;sc_step_sequence=0;g_source_align_command_seq=9;}
int main(void){
 const int32_t values[]={-2147483647,-30000,-20001,-20000,-19999,-19500,-1500,-1499,-500,-499,-1,0,1,499,500,1499,1500,19500,19999,20000,20001,30000,2147483647};
 const unsigned errors[]={0,4999,5000,5001,38000};unsigned i,j,k,chain;
 for(i=0;i<23;i++)for(j=0;j<23;j++)for(k=0;k<5;k++)for(chain=0;chain<2;chain++){
  unsigned state,request,direction,pose,steps,seq;float dist;
  g_source_align_chain_active=chain;setup(values[i],values[j],errors[k]);baseline_step30(100);
  state=g_source_cycle_state;request=g_source_body_step_request;direction=g_source_align_direction;pose=g_source_cycle_pose;steps=g_source_cycle_body_steps;seq=sc_step_sequence;dist=g_source_align_distance_mm;
  setup(values[i],values[j],errors[k]);actual_step20(100);
  CHECK(g_source_cycle_state==state && g_source_body_step_request==request && g_source_align_direction==direction);
  CHECK(g_source_cycle_pose==pose && g_source_cycle_body_steps==steps && sc_step_sequence==seq);
  CHECK(g_source_align_distance_mm==fminf(dist,20));
  if(errors[k]>5000)CHECK(g_source_align_distance_mm>=1 && g_source_align_distance_mm<=20);
 }
 fresh=0;setup(40000,-30000,38000);actual_step20(100);CHECK(g_source_cycle_state==2 && !g_source_body_step_request && g_source_cycle_pose==10);
 printf("PASS actual state2 branch: %u checks; signs/tie/round/clamp20/min1/fresh/5mm stop/chain identical to real30 except capped distance\n",checks);return 0;
}
'''
 path=E/'actual-step20-state2.c';path.write_text(prefix+suffix,encoding='utf-8');exe=E/'actual-step20-state2.exe'
 subprocess.run(['gcc','-std=c99','-Wall','-Wextra','-Werror',str(path),'-lm','-o',str(exe)],check=True)
 r=subprocess.run([str(exe)],capture_output=True,text=True);(E/'actual-step20-state2.log').write_text(r.stdout+r.stderr);print(r.stdout+r.stderr,flush=True);assert r.returncode==0
 return r.stdout.strip()
def main():
 proof=json.loads((E/'matlab-result.json').read_text());assert proof['passed']
 actual=test();manifest={};changes={}
 for p in P.rglob('*'):
  if not p.is_file() or any(x in ('Objects','Listings') for x in p.relative_to(P).parts):continue
  n=p.relative_to(P).as_posix();manifest[n]=sha(p)
  if p.read_bytes()!=(BASE/n).read_bytes():changes[n]=sha(p)
 assert set(changes)=={'CHASSIS/USER/source_mcu_cycle.inc'}
 b=(BASE/'CHASSIS/USER/source_mcu_cycle.inc').read_bytes();a=(P/'CHASSIS/USER/source_mcu_cycle.inc').read_bytes()
 assert b.replace(b'distance=labs((long)amount)>=30000L?30U:',b'distance=labs((long)amount)>=20000L?20U:',1)==a
 c=json.loads((R/'evidence/integration/2026-11-17/stage2-debug215/candidate.json').read_text());images={}
 for target in ['Vehicle_LiftRuler','Vehicle_LiftRuler_Offline']:
  for folder in ['Objects','Listings']:(P/folder/target).mkdir(parents=True,exist_ok=True)
  log=E/(target+'-build.log');r=subprocess.run(['C:/Keil_v5/UV4/UV4.exe','-r',str(P/'gongxun-f407.uvprojx'),'-t',target,'-j0','-o',str(log)],timeout=240);data=log.read_bytes();print(data.decode('gb18030',errors='replace')[-500:],flush=True)
  assert r.returncode==0 and b'0 Error(s), 0 Warning(s)' in data
  axf=P/'Objects'/target/(target+'.axf');images[target]={'axf':str(axf),'sha256':sha(axf)}
 c.update(base=str(BASE),project=str(P),changes=changes,images=images,matlab_latest=proof,matlab_timeline={'before_step20_implementation':proof['completed_local']},actual_c={'step20_state2':actual},source_step_max_mm=20,source_step_min_mm=1,alignment_stop_mm=5,
  exact_one_line_delta=True,review_passed=False,flashed=False,hardware_access=False,completed_local=datetime.datetime.now().isoformat())
 c['unchanged_sha256']={n:h for n,h in manifest.items() if n not in changes}
 (E/'candidate.json').write_text(json.dumps(c,indent=2)+'\n');(E/'source-manifest.json').write_text(json.dumps(manifest,indent=2)+'\n');(E/'checks.json').write_text(json.dumps({'passed':True,'actual_c':actual,'dual_build':True,'exact_one_line_delta':True,'hardware_access':False,'flashed':False},indent=2)+'\n')
if __name__=='__main__':main()
