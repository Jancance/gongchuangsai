"""Post-MATLAB teach publication and independent task/inventory binding."""
import json
from prepare_stage3_offline_ready_20261116 import P,E,save
def modify(name,pairs):
 b=(P/name).read_bytes()
 for old,new in pairs:
  old=old.replace('\n','\r\n').encode('ascii');new=new.replace('\n','\r\n').encode('ascii')
  assert b.count(old)==1,(name,old,b.count(old));b=b.replace(old,new)
 save(name,b)
def geometry_finish():
 proof=json.loads((E/'matlab-result.json').read_text());assert proof['passed'] and len(proof['checks'])>=8
 modify('CHASSIS/USER/stage3_cycle.inc',[
 ('uint32_t n=g_stage3_teach_request,needs_dm;','uint32_t n=g_stage3_teach_request,needs_dm,index;'),
 ('if(n>5U || !S3_TeachReady())','if(n>7U || !S3_TeachReady())'),
 ('if(n!=5U && (!g_vehicle_lift_feedback_valid','if(n!=5U && n<6U && (!g_vehicle_lift_feedback_valid'),
 ('if((n==2U || n==4U) && !(g_stage3_teach_measured_x','if((n==2U || n==4U || n==6U || n==7U) && !(g_stage3_teach_measured_x'),
 ('if(n==2U || n==4U)g_stage3_teach_measured_x=-1.0f;','if(n==2U || n==4U || n==6U || n==7U)g_stage3_teach_measured_x=-1.0f;'),
 ('if(n==2U || n==4U || n==5U){','if(n==2U || n==4U || n==5U || n==6U || n==7U){'),
 ('needs_dm=(n==2U || n==4U || n==5U);','needs_dm=(n==2U || n==4U || n==5U || n==6U || n==7U);'),
 (' }else g_stage3_carrier_yaw=g_stage3_actual_yaw;',''' }else if(n==6U || n==7U){
  index=n==6U?0U:2U;g_stage3_target_yaw[index]=g_stage3_actual_yaw;g_stage3_target_x[index]=s3_teach_x;
 }else g_stage3_carrier_yaw=g_stage3_actual_yaw;'''),
 ('if(n!=5U)g_stage3_teach_z_source=1U;','if(n!=5U && n<6U)g_stage3_teach_z_source=1U;')])
def main():
 proof=json.loads((E/'matlab-result.json').read_text());assert proof['passed'] and len(proof['checks'])>=6
 modify('CHASSIS/USER/stage3_cycle.h',[
 ('void Stage3_Arrive(void);','''void Stage3_Arrive(void);
typedef struct {uint32_t run_id,batch_id,colors[3],positions[3];} Stage3_Task;
/* Existing mission/CAN owner only. Bind first, then call Stage3_Arrive. */
uint32_t Stage3_BindMission(const Stage3_Task *task,const uint32_t slot_colors[3]);
extern volatile uint32_t g_stage3_slot_colors[3],g_stage3_task_run,g_stage3_task_batch;
extern volatile uint32_t g_stage3_teach_sequence,g_stage3_teach_saved_sequence;''')])
 modify('CHASSIS/USER/stage3_cycle.inc',[
 ('volatile uint32_t g_stage3_colors[3]={2U,1U,5U},g_stage3_positions[3]={1U,2U,3U};','''volatile uint32_t g_stage3_colors[3]={2U,1U,5U},g_stage3_positions[3]={1U,2U,3U};
/* Explicit bench inventory; normal mission binds committed real inventory. */
volatile uint32_t g_stage3_slot_colors[3]={2U,1U,5U},g_stage3_task_run,g_stage3_task_batch;
volatile uint32_t g_stage3_teach_sequence,g_stage3_teach_saved_sequence;'''),
 ('static uint32_t s3_first,s3_count,s3_colors[3],s3_positions[3];','static uint32_t s3_first,s3_count,s3_colors[3],s3_positions[3],s3_slots[3];'),
 ('void Stage3_Arrive(void){g_stage3_request=1U;}','''void Stage3_Arrive(void){g_stage3_request=1U;}
uint32_t Stage3_BindMission(const Stage3_Task *task,const uint32_t slots[3])
{
 uint32_t i,j,found;
 if(!task || !slots || !task->run_id || task->batch_id<1U || task->batch_id>2U ||
    (g_stage3_state!=0U && g_stage3_state!=6U && g_stage3_state!=7U && g_stage3_state!=11U) ||
    g_it_state==1U || g_it_request || g_stage3_request || s3_pending || s3_teach_pending)return 0U;
 for(i=0U;i<3U;i++){
  if(task->colors[i]<1U || task->colors[i]>6U || task->positions[i]<1U || task->positions[i]>3U || slots[i]<1U || slots[i]>6U)return 0U;
  for(j=0U;j<i;j++)if(task->colors[i]==task->colors[j] || task->positions[i]==task->positions[j] || slots[i]==slots[j])return 0U;
  found=0U;for(j=0U;j<3U;j++)if(slots[j]==task->colors[i])found++;
  if(found!=1U)return 0U;
 }
 for(i=0U;i<3U;i++){g_stage3_colors[i]=task->colors[i];g_stage3_positions[i]=task->positions[i];g_stage3_slot_colors[i]=slots[i];}
 g_stage3_task_run=task->run_id;g_stage3_task_batch=task->batch_id;return 1U;
}'''),
 ('uint32_t i,j,first=g_stage3_test_first_item,count=g_stage3_test_count;','uint32_t i,j,found,first=g_stage3_test_first_item,count=g_stage3_test_count;'),
 ('  for(j=0U;j<i;j++)if(g_stage3_colors[j]==g_stage3_colors[i] || g_stage3_positions[j]==g_stage3_positions[i])return 0U;','''  for(j=0U;j<i;j++)if(g_stage3_colors[j]==g_stage3_colors[i] || g_stage3_positions[j]==g_stage3_positions[i])return 0U;
  found=0U;for(j=0U;j<3U;j++)if(g_stage3_slot_colors[j]==g_stage3_colors[i])found++;
  if(found!=1U)return 0U;'''),
 ('  !g_it_cause && !g_it_rejected && !g_stage3_request;','  !g_it_cause && !g_it_rejected && !g_stage3_request && !SourceBodyStep_Busy() && !g_source_body_step_request;'),
 ('  s3_teach_pending=n;g_stage3_teach_result=2U;','''  s3_teach_pending=n;g_stage3_teach_result=2U;
  if(++g_stage3_teach_sequence==0U)g_stage3_teach_sequence=1U;
  if(n==2U || n==4U)g_stage3_teach_measured_x=-1.0f;'''),
 (' s3_teach_pending=0U;g_stage3_teach_missing=0U;g_stage3_teach_result=1U;',' s3_teach_pending=0U;g_stage3_teach_missing=0U;g_stage3_teach_saved_sequence=g_stage3_teach_sequence;g_stage3_teach_result=1U;'),
 ('  s3_colors[i]=g_stage3_colors[i];s3_positions[i]=g_stage3_positions[i];','  s3_colors[i]=g_stage3_colors[i];s3_positions[i]=g_stage3_positions[i];s3_slots[i]=g_stage3_slot_colors[i];'),
 (' S3_ReadActual();S3_Teach();g_stage3_config_ready=S3_Config();',''' S3_ReadActual();
 if(request){
  if(s3_teach_pending || g_stage3_teach_request){g_stage3_teach_result=3U;g_stage3_teach_missing=1U;}
  s3_teach_pending=0U;g_stage3_teach_request=0U;
 }else S3_Teach();
 g_stage3_config_ready=S3_Config();'''),
 ('  case 0U:g_it_carrier_deg=(float)(g_stage3_item*120U);S3_Command(10U);break;','''  case 0U:{
   uint32_t slot=0U;
   while(slot<3U && s3_slots[slot]!=s3_colors[g_stage3_item])slot++;
   /* Config validation plus locked inventory guarantee a unique color match. */
   g_it_carrier_deg=(float)(slot*120U);S3_Command(10U);break;
  }''')])
 modify('CHASSIS/USER/source_vision_runtime.h',[
 ('uint32_t Vision_ReadLoadedBatch(uint32_t run,uint32_t batch,uint32_t colors[3]);','''uint32_t Vision_ReadLoadedBatch(uint32_t run,uint32_t batch,uint32_t colors[3]);
/* Read-only task/inventory handoff; requires the same complete physical commits. */
uint32_t Vision_ReadLoadedTask(uint32_t run,uint32_t batch,uint32_t colors[3],uint32_t positions[3],uint32_t slot_colors[3]);''')])
 modify('CHASSIS/USER/integration10_complete.inc',[
 ('static uint8_t vis_buf[10];','''uint32_t Vision_ReadLoadedTask(uint32_t run,uint32_t batch,uint32_t colors[3],uint32_t positions[3],uint32_t slots[3])
{
 uint32_t i,offset;
 if(!colors || !positions || !slots || !Vision_ReadLoadedBatch(run,batch,slots))return 0U;
 offset=batch==1U?0U:8U;
 for(i=0U;i<3U;i++){colors[i]=g_vis_task_colors[i];positions[i]=(uint32_t)(vis_task_code[offset+4U+i]-'0');}
 return 1U;
}

static uint8_t vis_buf[10];''')])
 geometry_finish()
 print('Finished task binding/teach publication after MATLAB '+proof['when'])
if __name__=='__main__':main()
