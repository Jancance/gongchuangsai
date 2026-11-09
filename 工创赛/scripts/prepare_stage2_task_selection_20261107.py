"""Mainline: real task code + current vision -> one target; evidence -> next item."""
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
P = ROOT / 'tmp/integration12_stage2_vision_20261107/project/CHASSIS/USER/integration10_complete.inc'

TASK = r'''
/* Task order is independent of region and carrier slot. No actuator access. */
volatile uint32_t g_vis_task_valid=0U,g_vis_task_batch=0U,g_vis_task_item=0U;
volatile uint32_t g_vis_task_colors[3]={0U,0U,0U};
volatile uint32_t g_vis_pending_seq=0U,g_vis_pending_slot=0U,g_vis_pending_grasped=0U;
volatile uint32_t g_vis_loaded_mask=0U,g_vis_batch_complete=0U;
static char vis_task_code[16];

uint32_t Vision_SetTask(const char *code,uint32_t batch)
{
 uint32_t i,j,offset;uint32_t first=0U,second=0U,rings1=0U,rings2=0U;
 if(!code || (batch!=1U && batch!=2U) || g_vis_pending_seq)return 0U;
 for(i=0U;i<15U;i++){
  if(i==3U || i==7U || i==11U){if(code[i]!='+')return 0U;}
  else if(code[i]<'1' || code[i]>'6')return 0U;
 }
 if(code[15]!='\0')return 0U;
 for(i=0U;i<3U;i++){
  first|=1U<<(code[i]-'1');second|=1U<<(code[8U+i]-'1');
  if(code[4U+i]>'3' || code[12U+i]>'3')return 0U;
  rings1|=1U<<(code[4U+i]-'1');rings2|=1U<<(code[12U+i]-'1');
  for(j=0U;j<i;j++)if(code[i]==code[j] || code[8U+i]==code[8U+j])return 0U;
 }
 if(first!=second || rings1!=7U || rings2!=7U)return 0U;
 if(g_vis_task_valid && g_vis_task_batch==batch && !memcmp(vis_task_code,code,16U))return 1U;
 memcpy(vis_task_code,code,16U);offset=batch==1U?0U:8U;
 for(i=0U;i<3U;i++)g_vis_task_colors[i]=(uint32_t)(code[offset+i]-'0');
 g_vis_task_valid=1U;g_vis_task_batch=batch;g_vis_task_item=0U;
 g_vis_loaded_mask=g_vis_batch_complete=0U;g_vis_material_valid=0U;
 return 1U;
}

/* Only the existing mission/CAN owner calls these functions. External screen,
 * sensor or vision providers must supply actual evidence, not motor ACKs. */
uint32_t Vision_ReportGrasp(uint32_t sequence,uint32_t source_clear,uint32_t held,
 uint32_t evidence_ms,uint32_t now)
{
 if(!g_vis_pending_seq || sequence!=g_vis_pending_seq || source_clear!=1U || held!=1U ||
    (uint32_t)(now-evidence_ms)>1000U)return 0U;
 g_vis_pending_grasped=1U;return 1U;
}

uint32_t Vision_ReportPlaced(uint32_t sequence,uint32_t slot,uint32_t released,
 uint32_t present,uint32_t stable,uint32_t evidence_ms,uint32_t now)
{
 if(!g_vis_pending_seq || sequence!=g_vis_pending_seq || slot!=g_vis_pending_slot ||
    !g_vis_pending_grasped || released!=1U || present!=1U || stable!=1U ||
    (uint32_t)(now-evidence_ms)>1000U || g_vis_task_item>=3U)return 0U;
 g_vis_loaded_mask|=1U<<g_vis_task_item;g_vis_task_item++;
 g_vis_pending_seq=g_vis_pending_slot=g_vis_pending_grasped=0U;
 g_vis_material_valid=0U;
 if(g_vis_task_item==3U && g_vis_loaded_mask==7U)g_vis_batch_complete=1U;
 return 1U;
}
'''

SELECT = r'''
static uint32_t Vision_TakeTaskTarget(uint32_t now)
{
 uint32_t colors[3],i,count=0U,region=0U,desired;
 if(!g_vis_task_valid || g_vis_task_item>=3U){g_vis_txn_result=10U;return 0U;}
 if(g_vis_pending_seq){g_vis_txn_result=11U;return 0U;}
 if(!g_vis_material_valid || (uint32_t)(now-g_vis_mat_tick)>3000U){g_vis_txn_result=7U;return 0U;}
 desired=g_vis_task_colors[g_vis_task_item];
 colors[0]=g_vis_mat_left;colors[1]=g_vis_mat_right;colors[2]=g_vis_mat_down;
 for(i=0U;i<3U;i++)if(colors[i]==desired){count++;region=i;}
 if(count!=1U){g_vis_txn_result=count?12U:9U;return 0U;}
 g_vis_pick_region=region;g_src_slot=g_vis_task_item+1U;
 if(!Vision_TakeTarget(now))return 0U;
 g_vis_pending_seq=g_src_txn_seq;g_vis_pending_slot=g_src_slot;
 g_vis_pending_grasped=0U;return 1U;
}
'''


def main():
    raw = P.read_bytes()
    nl = b'\r\n' if b'\r\n' in raw else b'\n'
    s = raw.decode('latin1').replace('\r\n', '\n')
    assert 'Vision_TakeTaskTarget' not in s
    marker = 'static uint8_t vis_buf[10];'
    assert s.count(marker) == 1
    s = s.replace(marker, TASK + '\n' + marker)
    marker = 'static uint32_t it_platform'
    assert s.count(marker) == 1
    s = s.replace(marker, SELECT + '\n' + marker)
    s = s.replace('request>12U', 'request>13U')
    marker = '   }else if(request==12U){'
    assert s.count(marker) == 1
    s = s.replace(marker, '''   }else if(request==13U){
    if(Vision_TakeTaskTarget(now))g_it_state=1U;else g_it_rejected=1U;
''' + marker)
    s = s.replace(' case 12U:\n  it_done();break;', ' case 12U:case 13U:\n  it_done();break;')
    P.write_bytes(s.replace('\n', nl.decode()).encode('latin1'))
    print('Task-ordered target and explicit evidence acceptance added; no motor commands.')


if __name__ == '__main__':
    main()
