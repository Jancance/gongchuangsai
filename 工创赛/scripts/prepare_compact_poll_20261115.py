"""Reduce scheduler gaps only, preserving accepted motion and confirmations."""
from pathlib import Path
import json
import shutil

R=Path(__file__).resolve().parents[1]
E=R/'evidence/integration/2026-11-15/compact-poll'
BASE=R/'tmp/source_extension_lead_pick_20261115/project'
P=R/'tmp/source_compact_poll_20261115/project'

def main():
    d=json.loads((E/'matlab-result.json').read_text());assert d['passed'] and d['local_matlab']
    assert not P.exists(),'Preserve previous candidates'
    shutil.copytree(BASE,P,ignore=shutil.ignore_patterns('Objects','Listings'))
    path=P/'CHASSIS/USER/vehicle_source_parallel.inc';data=path.read_bytes()
    old=b'static uint32_t VSP_Poll(void *context,uint32_t bit)';assert data.count(old)==1
    data=data.replace(old,b'static uint32_t VSP_PollStep(void *context,uint32_t bit)',1)
    anchor=b'static uint32_t VSP_Stop(void *context,uint32_t bit)';assert data.count(anchor)==1
    wrapper='''/* 只合并一轮状态/位置/速度读取，保留跨服务两轮到位确认。
 * 缺包退出本次服务，不重发运动，不增加故障门限。 */
static uint32_t VSP_Poll(void *context,uint32_t bit)
{
 uint32_t i,result,before;VSP_Axis *axis;
 if(bit==SPF_DM || !Low_Busy())return VSP_PollStep(context,bit);
 axis=&vsp_axis[bit==SPF_EXT?0:1];
 for(i=0U;i<3U;i++){
  before=axis->phase;result=VSP_PollStep(context,bit);
  if(result || axis->phase==before || axis->phase==0U)return result;
 }
 return 0U;
}
'''.replace('\n','\r\n').encode('utf-8')
    path.write_bytes(data.replace(anchor,wrapper+anchor,1))
    print('Prepared polling-only candidate, original trajectory/speeds unchanged')

if __name__=='__main__':main()
