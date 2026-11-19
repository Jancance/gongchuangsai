"""Pure gate + actual desktop process/wire and actual MCU parser consumption."""
from pathlib import Path
import ast,sys,types,json,math,subprocess,shutil
from prepare_stage2_settle_ring_20261117 import R,P,E,PI
sys.path.insert(0,str(PI))
from source_alignment_settle import SourceAlignmentSettle
from observation_frame_epoch import ObservationFrameEpoch,camera_metadata
from mcu_cycle_protocol import frame,decode_status
CONFIG=json.loads((PI/'source-alignment-settle.json').read_text(encoding='utf-8'))
checks=0
def ck(value):
 global checks
 checks+=1
 assert value,checks
def meta(seq,stamp,gen=(1,2,3)):return dict(generation=gen,sequence=seq,completed=stamp)
def unit():
 for epoch in [10.,4294967.25]:
  gate=SourceAlignmentSettle(CONFIG);status=dict(run=1,pose=2,item=0,state=2)
  gate.receive(status,epoch);gate.receive(status,epoch+.2);ck(gate.epoch==epoch)
  ck(not gate.frame(meta(1,epoch-.01),epoch+.01,[0,0],True))
  ck(not gate.frame(meta(2,epoch+.01),epoch+.02,[0,0],True))
  for k in range(5):ck(not gate.frame(meta(3+k,epoch+.1+k*.02),epoch+.11+k*.02,[0,0],True))
  ck(not gate.centers)
  for k in range(5):ck(gate.frame(meta(8+k,epoch+.31+k*.02),epoch+.32+k*.02,[k*.1,0],True)==(k==4))
  ck(not gate.frame(meta(12,epoch+.39),epoch+.4,[0,0],True))
  ck(not gate.frame(meta(13,epoch+.42),epoch+.43,[4,0],True))
  ck(not gate.frame(meta(14,epoch+.44),epoch+.45,[0,0],False) and not gate.centers)
  for k in range(5):ck(gate.frame(meta(15+k,epoch+.46+k*.02),epoch+.47+k*.02,[0,0],True)==(k==4))
  ck(not gate.frame(meta(20,epoch+.6),epoch+1,[0,0],True) and not gate.centers)
  ck(not gate.frame(meta(21,epoch+1.1,(2,3,4)),epoch+1.11,[0,0],True) and not gate.centers)
  ck(gate.reason=='host_read_before_pose')
  gate.receive(dict(status,pose=4),epoch+2);ck(gate.epoch==epoch+2 and gate.fence is None)
  gate.receive(status,math.nan);ck(gate.key is None)
  gate.receive(status,epoch+3);ck(not gate.frame(meta(1,epoch+3.1),math.nan,[0,0],True) and gate.key is None)
  gate.receive(status,epoch+4);gate.invalidate('lost');ck(not gate.frame(meta(30,epoch+4.4),epoch+4.41,[0,0],True))
 # Pairwise all-pairs criterion must reject slowly drifting adjacent-good samples.
 gate=SourceAlignmentSettle(CONFIG);gate.receive(dict(run=1,pose=2,item=0,state=2),10)
 gate.frame(meta(1,10.01),10.02,[0,0],True)
 for i in range(5):ck(not gate.frame(meta(i+2,10.31+i*.02),10.32+i*.02,[i*.8,0],True))
 ck(gate.max_pair_seen>2)

def integrated():
 code=(PI/'source_fast_cycle_view_20261115.py').read_text(encoding='utf-8');tree=ast.parse(code)
 main=next(n for n in tree.body if isinstance(n,ast.FunctionDef) and n.name=='main')
 process=next(n for n in main.body if isinstance(n,ast.FunctionDef) and n.name=='process')
 # Only scope binding is adapted; the complete actual frame processing body runs.
 for i,n in enumerate(process.body):
  if isinstance(n,ast.Nonlocal):process.body[i]=ast.Global(names=n.names)
 mod=ast.fix_missing_locations(ast.Module(body=[process],type_ignores=[]))
 class Image:
  shape=(1080,1920,3)
  def copy(self):return self
 class Clock:
  now=10.
  def monotonic(self):return self.now
 class Dummy:
  def __getattr__(self,n):return lambda *a,**k:None
 class Link:
  def __init__(self):self.rx=b'';self.wires=[]
  @property
  def in_waiting(self):return len(self.rx)
  def read(self,n):r=self.rx;self.rx=b'';return r
  def write(self,wire):self.wires.append(wire);return len(wire)
 class Region(Dummy):
  region='source'
  def receive(self,*a):return False
  def source_received(self):self.region='source'
 class Gate(Dummy):
  def diagnostic(self):return {}
 class Rotation(Gate):
  previous=None;branch=None
 class Rim:
  center=[980.,400.]
  def reset(self):pass
  def estimate(self,f):return dict(center_px=self.center,radius_px=600,fit_rms_px=.8,observation_valid=True,right_forward_mm=[8,-3],error_norm_mm=9.)
 clock=Clock();link=Link();camera=types.SimpleNamespace(_worker=types.SimpleNamespace(pid=3),_sequence=0,last_read_completed_time=None)
 cv=Dummy();cv.FONT_HERSHEY_SIMPLEX=0;cv.MARKER_CROSS=0;cv.waitKey=lambda n:255
 epoch=ObservationFrameEpoch();settle=SourceAlignmentSettle(CONFIG)
 env=dict(json=json,time=clock,link=link,region_ui=Region(),frame_epoch=epoch,camera_metadata=camera_metadata,
  app=types.SimpleNamespace(camera=camera,draw_material_pickup_detection=lambda *a:None),args=types.SimpleNamespace(preview_only=False,study_stopped_pose=False,continuous_pick=True,observe_motion=False,session_dir=E),
  gate=Gate(),direct=Gate(),initial_rotation=Rotation(),alignment_settle=settle,rim=Rim(),predictor=None,
  status=None,received=0.,buffer=b'',context=None,frame_count=0,sequence=0,last_sent=0.,packets=0,last_observation_tx=None,
  window_ready=True,current_run=None,epoch_revision=epoch.revision,window='test',cv2=cv,profile={'reference_center_px':[986,400]},
  rim_observation_valid=lambda b,r:True,decode_status=decode_status,STATES={2:'align'},capture=types.SimpleNamespace(json_value=lambda x:x),
  compose=lambda *a,**k:Image())
 from mcu_cycle_protocol import observation,continuous_observation
 env.update(observation=observation,continuous_observation=continuous_observation)
 exec(compile(mod,str(PI/'source_fast_cycle_view_20261115.py'),'exec'),env)
 results=[];trace=[]
 def tick(now,seq,stamp,statuswire=None):
  clock.now=now;camera._sequence=seq;camera.last_read_completed_time=stamp
  link.rx=statuswire if statuswire is not None else frame('SC2',[1,2,2,6,0,2,1,5,0,0,0,0,0])
  result,_=env['process'](Image(),'M');trace.append(result);return result
 statuswire=frame('SC2',[1,2,2,6,0,2,1,5,0,0,0,0,0])
 # Receive the true post-STOP status; current frame predates it and is excluded.
 tick(10,1,9.99,statuswire)
 tick(10.02,2,10.01);tick(10.1,3,10.09);tick(10.2,4,10.19)
 for k in range(4):
  result=tick(10.31+k*.12,5+k,10.305+k*.12);ck(not result['alignment_settle']['ready'])
 result=tick(10.79,9,10.785);ck(result['alignment_settle']['ready'])
 ck(result['last_observation_tx']['wire'].split(',')[4]=='1')
 ck(all(w.decode().split(',')[4]=='0' for w in link.wires[:-1]))
 ck(link.wires[-1].decode().split(',')[4]=='1')
 # Repeat pose does not re-arm; moving/newpose explicitly re-arms and resets.
 tick(11,10,10.99,frame('SC2',[1,3,3,6,0,2,1,5,0,0,0,0,0]));ck(settle.key is None)
 tick(11.1,11,11.09,frame('SC2',[1,2,4,6,0,2,1,5,0,0,0,0,0]));ck(settle.epoch==11.1 and not settle.centers)
 (E/'pi-actual-process-trace.json').write_text(json.dumps(trace,indent=2,default=str),encoding='utf-8')
 (E/'pi-actual-tx-wires.json').write_text(json.dumps([w.decode() for w in link.wires],indent=2),encoding='utf-8')
 return link.wires

def consume(wires):
 # Run complete current source cycle/CRC parser, reusing its existing hardware stubs.
 old=R/'evidence/integration/2026-11-17/stage2-debug215'
 for p in old.iterdir():
  if p.suffix in ('.h','.inc','.c'):shutil.copyfile(p,E/p.name)
 (E/'tested-source-cycle.inc').write_bytes((P/'CHASSIS/USER/source_mcu_cycle.inc').read_bytes())
 s=(old/'actual-debug215.c').read_text(encoding='utf-8');start=s.rindex('int main(void){');s=s[:start]
 literals=',\n'.join(json.dumps(w.decode()) for w in wires if w.startswith(b'OC1'))
 s+=r'''
int main(void){const char* wires[]={'''+literals+r'''};unsigned i,j,accepted=0;
 round_reset();g_source_cycle_run=1;g_source_cycle_pose=2;g_source_cycle_state=2;g_source_cycle_colors[0]=2;g_source_cycle_colors[1]=1;g_source_cycle_colors[2]=5;
 for(i=0;i<sizeof(wires)/sizeof(wires[0]);i++){
  for(j=0;j<strlen(wires[i]);j++)CHECK(SourceCycle_SharedByte((uint8_t)wires[i][j],tick,0));
  SourceCycle_Service(tick);if(g_source_cycle_state==3){accepted++;CHECK(g_source_body_step_request==2 && g_source_align_distance_mm<=20);break;}
  CHECK(g_source_cycle_state==2 && !g_source_body_step_request);
 }
 CHECK(accepted==1 && g_source_cycle_rx==i+1 && !g_source_cycle_rx_bad);
 printf("PASS actual source parser consumes real Pi process wires: %u checks; invalid pre-settle waits, one valid correction with unchanged20mm path\n",checks);return 0;}
'''
 path=E/'actual-settle-wire-consumer.c';path.write_text(s,encoding='utf-8');exe=E/'actual-settle-wire-consumer.exe'
 cmd=['gcc','-std=c99','-Wno-unused-function','-Wno-misleading-indentation','-I'+str(E),'-I'+str(P/'CHASSIS/USER'),'-I'+str(P/'USER'),str(path),str(P/'CHASSIS/USER/source_parallel_flow.c'),'-lm','-o',str(exe)]
 subprocess.run(cmd,check=True);r=subprocess.run([str(exe)],capture_output=True,text=True);(E/'actual-settle-wire-consumer.log').write_text(r.stdout+r.stderr);print(r.stdout+r.stderr);ck(r.returncode==0)
def main():
 unit();wires=integrated();consume(wires)
 result={'passed':True,'checks':checks,'actual_process_wires':len(wires),'scope':'real process() + metadata boundary + OC1 generation + actual MCU CRC/parser/state2; GUI/camera/UART/time mocked; no hardware'}
 (E/'pi-checks.json').write_text(json.dumps(result,indent=2)+'\n');print(json.dumps(result),flush=True)
if __name__=='__main__':main()
