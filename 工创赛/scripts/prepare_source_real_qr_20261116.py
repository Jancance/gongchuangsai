"""Minimal real UART5 QR gate in an isolated source-cycle candidate; no hardware IO."""
from pathlib import Path
import hashlib,json,shutil
R=Path(__file__).resolve().parents[1]
BASE=R/'tmp/source_loaded_retract30_20261116/project'
P=R/'tmp/source_real_qr_20261116/project'
E=R/'evidence/integration/2026-11-16/source-real-qr'

SCAN=r'''/* TaskA owns scanner queue/parser and screen. CAN owns task validation and
 * publishes locked_run last; the mailbox stays immutable until consumed. */
extern volatile uint32_t g_scan_display_result,g_scan_display_count;
volatile uint32_t g_source_scan_owner_run,g_source_scan_locked_run;
volatile uint32_t g_source_scan_pending,g_source_scan_accept,g_source_scan_result;
volatile uint32_t g_source_scan_frame_start_ms,g_source_scan_frame_end_ms;
volatile uint32_t g_source_scan_rejected,g_source_scan_displayed_run;
char g_source_scan_code[16];
static uint32_t sc_scan_seen;
void SourceCycle_ScanService(void)
{
 uint32_t run=g_source_cycle_run,state=g_source_cycle_state;
 if(state==1U && g_source_scan_owner_run!=run){
  SAOMA_DisableReceive();SAOMA_EnableReceive();
  taskENTER_CRITICAL();
  sc_scan_seen=g_saoma_accept_count;g_source_scan_pending=0U;
  g_source_scan_result=0U;g_source_scan_code[0]='\0';
  g_source_scan_owner_run=run;
  taskEXIT_CRITICAL();
 }
 if(g_source_scan_locked_run==run && run){
  if(!g_saoma_locked)SAOMA_DisableReceive();
  if(g_source_scan_displayed_run!=run){
   g_scan_display_result=Screen_ShowTaskCode(g_source_scan_code);
   if(g_scan_display_result==SCREEN_OK)g_scan_display_count++;
   g_source_scan_displayed_run=run;
  }
  return;
 }
 if(state!=1U || !run || g_source_scan_owner_run!=run || g_source_scan_pending)return;
 taskENTER_CRITICAL();
 if(g_source_cycle_state==1U && g_source_cycle_run==run &&
    g_source_scan_owner_run==run && g_source_scan_locked_run!=run &&
    !g_source_scan_pending && SAOMA_DataReady && g_saoma_accept_count!=sc_scan_seen){
  memcpy(g_source_scan_code,SAOMA_Data,16U);
  g_source_scan_accept=g_saoma_accept_count;g_source_scan_result=g_saoma_last_result;
  g_source_scan_frame_start_ms=g_saoma_frame_start_ms;
  g_source_scan_frame_end_ms=g_saoma_frame_end_ms;
  sc_scan_seen=g_saoma_accept_count;SAOMA_DataReady=0U;
  g_source_scan_pending=1U;
 }
 taskEXIT_CRITICAL();
}
static uint32_t SC_ScanTask(void)
{
 uint32_t i,accepted;char code[16];
 if(g_source_scan_locked_run==g_source_cycle_run)return 1U;
 if(g_source_scan_owner_run!=g_source_cycle_run || !g_source_scan_pending)return 0U;
 taskENTER_CRITICAL();
 memcpy(code,g_source_scan_code,16U);
 accepted=g_source_scan_result;
 taskEXIT_CRITICAL();
 /* Task mailbox has no actuator access. Full rule semantics stay centralized. */
 if(accepted==1U && Vision_SetTask(code,1U,g_source_cycle_run)){
  for(i=0U;i<3U;i++)g_source_cycle_colors[i]=g_vis_task_colors[i];
  taskENTER_CRITICAL();
  g_source_scan_locked_run=g_source_cycle_run;
  g_source_scan_pending=0U;
  taskEXIT_CRITICAL();
  return 1U;
 }
 g_source_scan_rejected++;
 taskENTER_CRITICAL();g_source_scan_pending=0U;taskEXIT_CRITICAL();
 return 0U;
}
'''

def main():
 proof=json.loads((E/'matlab-result.json').read_text());assert proof['passed'] and proof['local_matlab']
 assert not P.exists(),'Preserve previous candidate'
 shutil.copytree(BASE,P,ignore=shutil.ignore_patterns('Objects','Listings'))
 def edit(name,replacements):
  path=P/name;data=path.read_bytes();assert data.count(b'\n')==data.count(b'\r\n')
  for old,new in replacements:
   old=old.replace('\n','\r\n').encode('ascii');new=new.replace('\n','\r\n').encode('ascii')
   assert data.count(old)==1,(name,old);data=data.replace(old,new,1)
  path.write_bytes(data)
 edit('CHASSIS/USER/source_mcu_cycle.inc',[
  ('#include "Buzzer.h"','#include "Buzzer.h"\n#include "saoma.h"\n#include "screen.h"'),
  ('g_source_cycle_colors[3]={2U,1U,5U}','g_source_cycle_colors[3]={0U,0U,0U}'),
  ('void SourceCycle_KeyEvent(void)',SCAN+'void SourceCycle_KeyEvent(void)'),
  ('  g_source_cycle_started=now;g_source_cycle_state=1U;SC_Pose();','  g_source_cycle_colors[0]=g_source_cycle_colors[1]=g_source_cycle_colors[2]=0U;\n  g_source_cycle_started=now;g_source_cycle_state=1U;SC_Pose();'),
  ('  static const uint32_t prep[6]={1U,2U,7U,6U,3U,3U};','  static const uint32_t prep[6]={1U,2U,7U,6U,3U,3U};\n  if(!SC_ScanTask())return;')])
 edit('CHASSIS/USER/main.c',[('        SAOMA_Update();','        if(SOURCE_MCU_CYCLE_ENABLE)SourceCycle_ScanService();\n        SAOMA_Update();')])
 edit('CHASSIS/USER/vehicle_arm.h',[('void SourceCycle_KeyEvent(void);','void SourceCycle_KeyEvent(void);\nvoid SourceCycle_ScanService(void);')])
 changes={p.relative_to(P).as_posix():hashlib.sha256(p.read_bytes()).hexdigest() for p in P.rglob('*') if p.is_file() and p.read_bytes()!=(BASE/p.relative_to(P)).read_bytes()}
 assert len(changes)==3
 (E/'prepared.json').write_text(json.dumps(dict(changes=changes,flashed=False,hardware_access=False),indent=2)+'\n')
 print('Prepared three-file QR-only delta; all other baseline source bytes preserved')
if __name__=='__main__':main()
