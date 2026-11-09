"""Isolate DM parameter transactions from ordinary CAN1 polling; STOP stays active."""
from pathlib import Path
import shutil,json,hashlib
R=Path(__file__).resolve().parents[1];B=R/'tmp/integration10_can2_recovery_20261106/project'
S=R/'tmp/integration10_can2_schedule_20261106/project';O=R/'evidence/integration/2026-11-06/can2-recovery'
def main():
    assert json.loads((O/'schedule-matlab.json').read_text())['passed']
    assert not S.exists()
    shutil.copytree(B,S,ignore=shutil.ignore_patterns('Objects','ObjectsOffline','Listings','*.uvguix.*'))
    changes={}
    def edit(n,fn):
        p=S/n;raw=p.read_bytes()
        try:t=raw.decode('utf-8');enc='utf-8'
        except UnicodeDecodeError:t=raw.decode('gbk');enc='gbk'
        nl='\r\n' if b'\r\n' in raw else '\n';t=fn(t.replace('\r\n','\n'))
        p.write_bytes(t.replace('\n',nl).encode(enc));changes[n]={'encoding':enc,'before':hashlib.sha256(raw).hexdigest(),'after':hashlib.sha256(p.read_bytes()).hexdigest()}
    def sub(t,a,b):assert t.count(a)==1,a;return t.replace(a,b)
    edit('CHASSIS/USER/main.c',lambda t:sub(t,'                    App_ComboIdlePoll();',
     '''                    /* 参数事务期间不启动可重置CAN1共享资源的普通轮询。
                     * 下方已有停止帧分派仍无条件保留。 */
                    if(!(g_dm_can2_read_request || (g_dm_can2_state>=1U && g_dm_can2_state<=3U)))App_ComboIdlePoll();'''))
    def flow(t):
        t=sub(t,' StartupAxes_Service(it_platform(),g_can_guard_ready);',
        ''' /* DM参数读取窗口内暂缓伸缩保持状态查询，取消/失败STOP仍服务。 */
 if(g_startup_axes_state>=5U || !(g_dm_can2_read_request || (g_dm_can2_state>=1U && g_dm_can2_state<=3U)))
  StartupAxes_Service(it_platform(),g_can_guard_ready);''')
        t=sub(t,'  if(it_phase==1U){\n   if(g_dm_can2_state==5U)',
        '  if(it_phase==1U){\n   if(g_dm_can2_read_cycle==it_dm_cycle)break; /* 不消费上一轮失败。 */\n   if(g_dm_can2_state==5U)')
        t=sub(t,' case 7U:\n  if(g_dm_can2_state==5U)',
        ' case 7U:\n  if(it_phase==1U && g_dm_can2_read_cycle==it_dm_cycle)break;\n  if(g_dm_can2_state==5U)')
        return t
    edit('CHASSIS/USER/integration10_complete.inc',flow)
    (O/'schedule-staged.json').write_text(json.dumps({'base':str(B),'candidate':str(S),'changes':changes,'firmware_downloaded':False,'cause_of_external_no_ACK_proven':False,'mechanical_targets_or_speeds_changed':False},indent=2),encoding='utf-8')
    print('Prepared scheduling-only candidate, no hardware action.')
if __name__=='__main__':main()
