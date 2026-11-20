from pathlib import Path
import re,subprocess,sys,struct,json,hashlib
R=Path.cwd();T=R/'工创赛/tmp/radar_four_turn_20261111/arm_tests';T.mkdir(exist_ok=True);E=R/'工创赛/evidence/navigation-planning/2026-11-11/radar-zone2/four-turn-software';B=Path('D:/Keil_v5/ARM/arm5_compiler/bin');P=R/'3.stm32f407工程/CHASSIS/HARDWARE/RADAR_V6';old=R/'模快资料/雷达资料/radar_stm32例程/F407VET6/HostTests'
for n in ['radar_client.h','radar_uart_port.h','link_protocol.h']:(T/n).write_bytes((P/n).read_bytes())
s=(old/'stub_port.c').read_text('utf8');s=s.replace('static uint16_t receive_position;','static uint16_t receive_position,read_pos;').replace('    receive_position = 0U;','    receive_position = 0U; read_pos=0U;');a=s.index('uint16_t RadarUartPort_GetReceivePosition');b=s.index('bool RadarUartPort_StartTransmit',a);s=s[:a]+'''bool RadarUartPort_ReadByte(uint8_t *b){if(read_pos==receive_position)return false;*b=receive_buffer[read_pos];read_pos=(read_pos+1U)%receive_size;return true;}
'''+s[b:]
tests=(old/'test_main.c').read_text('utf8');tests=re.sub(r'#define CHECK\(value\).*?while \(0\)', '#define CHECK(value) do { assertions++; if(!(value)){ failed=__LINE__;return; } } while(0)',tests);tests=tests.replace('static unsigned callback_count;','volatile unsigned failed,assertions,cases;\nstatic unsigned callback_count;');tests=tests[:tests.index('int main(void)')]
extra=r'''
static unsigned grids,paths;
static void on_grid_test(const radar_client_grid_result_t *g){(void)g;grids++;}
static void on_path_test(const radar_client_path_result_t *p){(void)p;paths++;}
static void ready(void){
 radar_client_callbacks_t c;uint8_t hello[13]={1,35,6,1},persisted[33]={11},ack[2]={LINK_CMD_GET_PERSISTED_STATE,0};
 memset(&c,0,sizeof(c));c.grid=on_grid_test;c.path=on_path_test;c.error=on_error;c.calibration=on_calibration;
 StubPort_Reset();RadarClient_Init(&c,1);RadarClient_Poll();feed_event(1,LINK_EVT_HELLO,hello,13);feed_event(1,LINK_EVT_PERSISTED_STATE,persisted,33);feed_event(1,LINK_EVT_ACK,ack,2);CHECK(RadarClient_IsReady());
}
static void transaction_cases(void){
 uint8_t grid[5]={1,0,0,0,0},locked[5]={4,0,0,14,0},ack[2]={LINK_CMD_SINGLE_MEASURE,0};
 uint8_t p[8]={0,0,0,0,3,1,3,0};unsigned before;uint16_t seq;
 ready();grids=paths=0;CHECK(RadarClient_RequestGridScan()==0);feed_event(1,LINK_EVT_OBSTACLE_GRID,grid,5);CHECK(grids==0);feed_event(1,LINK_EVT_STATUS,locked,5);CHECK(grids==0);feed_event(1,LINK_EVT_ACK,ack,2);CHECK(grids==1&&!RadarClient_IsBusy());cases++;
 CHECK(RadarClient_RequestGridScan()==0);feed_event(1,LINK_EVT_OBSTACLE_GRID,grid,5);feed_event(1,LINK_EVT_STATUS,locked,5);feed_event(1,LINK_EVT_ACK,ack,2);CHECK(grids==1&&RadarClient_IsBusy());/* prior locked status cannot validate overflow */
 RadarClient_RequestStop();ack[0]=LINK_CMD_STOP;feed_event(1,LINK_EVT_ACK,ack,2);cases++;
 before=paths;CHECK(RadarClient_RequestPlanPath(0,3)==0);seq=transaction.request_sequence;p[0]=(uint8_t)(seq-1);p[1]=(uint8_t)((seq-1)>>8);feed_event(1,LINK_EVT_PATH_RESULT,p,7);CHECK(paths==before);p[0]=(uint8_t)seq;p[1]=(uint8_t)(seq>>8);p[3]=1;feed_event(1,LINK_EVT_PATH_RESULT,p,7);CHECK(paths==before);p[3]=0;p[6]=2;feed_event(1,LINK_EVT_PATH_RESULT,p,7);CHECK(paths==before);p[6]=3;feed_event(1,LINK_EVT_PATH_RESULT,p,7);CHECK(paths==before);ack[0]=LINK_CMD_PLAN_PATH;feed_event(1,LINK_EVT_ACK,ack,2);CHECK(paths==before+1);cases++;
 CHECK(RadarClient_RequestPlanPath(3,3)==0);seq=transaction.request_sequence;p[0]=(uint8_t)seq;p[1]=(uint8_t)(seq>>8);p[3]=p[4]=3;p[5]=0;feed_event(1,LINK_EVT_ACK,ack,2);feed_event(1,LINK_EVT_PATH_RESULT,p,6);CHECK(paths==before+2);cases++;
 CHECK(RadarClient_RequestPlanPath(1,2)==0);StubPort_Advance(1501);RadarClient_Poll();CHECK(!RadarClient_IsBusy());cases++;
 StubPort_Advance(5001);RadarClient_Poll();CHECK(!RadarClient_IsReady());cases++;
}

static void calibration_orders(void){
 static const uint8_t orders[6][3]={{0,1,2},{0,2,1},{1,0,2},{1,2,0},{2,0,1},{2,1,0}};
 uint8_t ack[2]={LINK_CMD_CALIBRATE,0},cal[15]={1},persist[33]={11};unsigned k,j,before;
 persist[6]=1;
 for(k=0;k<6;k++){
  ready();before=calibration_count;CHECK(RadarClient_RequestCalibrate(1)==0);
  for(j=0;j<3;j++){
   if(orders[k][j]==0)feed_event(1,LINK_EVT_ACK,ack,2);
   if(orders[k][j]==1)feed_event(1,LINK_EVT_CALIBRATION,cal,15);
   if(orders[k][j]==2)feed_event(1,LINK_EVT_PERSISTED_STATE,persist,33);
   CHECK(calibration_count==before+(j==2));
  }
  feed_event(1,LINK_EVT_CALIBRATION,cal,15);CHECK(calibration_count==before+1);cases++;
 }


 
 ready();before=calibration_count;CHECK(RadarClient_RequestCalibrate(1)==0);persist[11]=0;
 feed_event(1,LINK_EVT_CALIBRATION,cal,15);feed_event(1,LINK_EVT_PERSISTED_STATE,persist,33);
 persist[11]=5;feed_event(1,LINK_EVT_PERSISTED_STATE,persist,33);persist[11]=9;feed_event(1,LINK_EVT_PERSISTED_STATE,persist,33);
 CHECK(calibration_count==before&&RadarClient_IsBusy());feed_event(1,LINK_EVT_ACK,ack,2);CHECK(calibration_count==before+1&&!RadarClient_IsBusy());cases++;

 for(k=0;k<3;k++){
 ready();before=calibration_count;CHECK(RadarClient_RequestCalibrate(1)==0);persist[11]=5;
  if(k==0)feed_event(1,LINK_EVT_ACK,ack,2);
  if(k==1)feed_event(1,LINK_EVT_CALIBRATION,cal,15);
  feed_event(1,LINK_EVT_PERSISTED_STATE,persist,33);
  persist[11]=7;feed_event(1,LINK_EVT_PERSISTED_STATE,persist,33);
  if(k!=1)feed_event(1,LINK_EVT_CALIBRATION,cal,15);
  if(k!=0)feed_event(1,LINK_EVT_ACK,ack,2);
  CHECK(calibration_count==before&&RadarClient_IsBusy());
  persist[11]=0;feed_event(1,LINK_EVT_PERSISTED_STATE,persist,33);CHECK(calibration_count==before+1&&!RadarClient_IsBusy());cases++;
 }
 ready();before=calibration_count;CHECK(RadarClient_RequestCalibrate(1)==0);persist[11]=5;
 feed_event(1,LINK_EVT_ACK,ack,2);feed_event(1,LINK_EVT_CALIBRATION,cal,15);feed_event(1,LINK_EVT_PERSISTED_STATE,persist,33);CHECK(calibration_count==before&&RadarClient_IsBusy());persist[11]=0;feed_event(1,LINK_EVT_PERSISTED_STATE,persist,33);CHECK(calibration_count==before+1&&!RadarClient_IsBusy());cases++;
}
static void grid_failure_cases(void){
 uint8_t ack[2]={LINK_CMD_SINGLE_MEASURE,0},grid[5]={1,0,0,0,0},locked[5]={4,0,0,13,0};unsigned before;
 ready();before=grids;CHECK(RadarClient_RequestGridScan()==0);feed_event(1,LINK_EVT_OBSTACLE_GRID,grid,5);feed_event(1,LINK_EVT_ACK,ack,2);feed_event(1,LINK_EVT_STATUS,locked,5);CHECK(grids==before&&RadarClient_IsBusy());locked[3]=14;locked[4]=1;feed_event(1,LINK_EVT_STATUS,locked,5);CHECK(grids==before&&RadarClient_IsBusy());locked[4]=0;feed_event(1,LINK_EVT_STATUS,locked,5);CHECK(grids==before+1&&!RadarClient_IsBusy());cases++;
}
unsigned test_entry(void){
 test_fixed_vector();cases++;if(failed)return failed;
 test_stream_decoder();cases++;if(failed)return failed;
 test_work_zone_gate();cases++;if(failed)return failed;
 test_work_zone_error();cases++;if(failed)return failed;
 test_deferred_zone_request_retry_is_bounded();cases++;if(failed)return failed;
 test_link_timeout_restarts_handshake();cases++;if(failed)return failed;
 test_calibration_waits_for_zone_persisted_state();cases++;if(failed)return failed;
 transaction_cases();if(failed)return failed;calibration_orders();if(failed)return failed;grid_failure_cases();return failed;
}
'''
source=(P/'radar_client.c').read_text('utf8')+'\n'+s+'\n'+tests+'\n'+extra
(T/'unit.c').write_text(source,'utf8');(T/'unit.sct').write_text('LR_ROM 0x08000000 0x100000 { ER_ROM 0x08000000 0x100000 { unit.o (i.test_entry, +First) *(+RO) } RW_RAM 0x20000000 0x20000 { *(+RW,+ZI) } }')
(T/'link_protocol.c').write_bytes((P/'link_protocol.c').read_bytes())
commands=[[str(B/'armcc.exe'),'--cpu','Cortex-M3','--c99','--gnu','--split_sections','-O1','-I.','-c','link_protocol.c','-o','link.o'],[str(B/'armcc.exe'),'--cpu','Cortex-M3','--c99','--gnu','--split_sections','-O1','-I.','-c','unit.c','-o','unit.o'],[str(B/'armlink.exe'),'--entry','test_entry','--scatter','unit.sct','unit.o','link.o','--library_type=microlib','-o','unit.axf']]
from radar_zone2_arm_runner_20261111 import run
result=run(R,T,B,source,commands)
(E/'client_arm_result.json').write_text(json.dumps({**result,'external_uart_stub':True,'loader':'SHF_ALLOC PROGBITS and NOBITS','hardware':False},indent=2),'utf8')
