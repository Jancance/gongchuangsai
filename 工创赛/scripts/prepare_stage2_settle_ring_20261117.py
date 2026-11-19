"""MATLAB-gated isolated ring repair + exact user-authorized source pose gate."""
from pathlib import Path
import json,shutil
R=Path(__file__).resolve().parents[1]
BASE=R/'tmp/stage2_debug215_step20_20261117/project'
P=R/'tmp/stage2_settle_ring_20261117/project'
E=R/'evidence/integration/2026-11-17/stage2-settle-ring'
PIBASE=R/'output/stage23_desktop_20261116'
PI=R/'output/stage23_settle300_5x2_20261117'
def edit(path,pairs,encoding='ascii'):
 b=path.read_bytes()
 for old,new in pairs:
  line='\r\n' if b.count(b'\r\n')==b.count(b'\n') and b.count(b'\n') else '\n'
  old=old.replace('\n',line).encode(encoding);new=new.replace('\n',line).encode(encoding)
  assert b.count(old)==1,(path,old,b.count(old));b=b.replace(old,new,1)
 path.write_bytes(b)
def main():
 assert json.loads((E/'matlab-result.json').read_text())['passed']
 assert not P.exists() and not PI.exists(),'Preserve candidates'
 shutil.copytree(BASE,P,ignore=shutil.ignore_patterns('Objects','Listings'))
 edit(P/'CHASSIS/USER/chassis_auto_test.c',[
 ('        g_chassis_auto_session++;', '        g_chassis_auto_session++;\n        if(!g_chassis_auto_session)g_chassis_auto_session=1U;'),
 ('if (s_endpoint_index < s_endpoint_count && s_heading_session)\n                    s_endpoints[s_endpoint_index].outcome = 1U;', 'if (s_endpoint_index < s_endpoint_count)\n                    s_endpoints[s_endpoint_index].outcome = 1U;'),
 ('static uint32_t s_endpoint_count, s_endpoint_index;', 'static uint32_t s_endpoint_count, s_endpoint_index, s_endpoint_head;'),
 ('static void Auto_PositionReset(uint32_t session)', '''/* Diagnostic history must never exhaust motion admission. Only terminal records
 * may be recycled; the active encoder reference keeps its physical slot. */
static uint32_t Auto_EndpointAvailable(void)
{
    if(s_endpoint_count < CHASSIS_POSITION_ENDPOINT_CAPACITY) return 1U;
    return s_endpoints[s_endpoint_head].outcome==1U || s_endpoints[s_endpoint_head].outcome==2U;
}
static void Auto_PositionReset(uint32_t session)'''),
 ('''    if (session == 0U) s_endpoint_count = 0U;
    s_endpoint_index = CHASSIS_POSITION_ENDPOINT_CAPACITY;
    if (session && s_endpoint_count < CHASSIS_POSITION_ENDPOINT_CAPACITY)
    {
        ChassisPositionEndpoints endpoint = {0};
        endpoint.session = session;''', '''    if (session == 0U) s_endpoint_count = s_endpoint_head = 0U;
    s_endpoint_index = CHASSIS_POSITION_ENDPOINT_CAPACITY;
    if (session && Auto_EndpointAvailable())
    {
        ChassisPositionEndpoints endpoint = {0};
        uint32_t slot;
        endpoint.session = session;'''),
 ('''        s_endpoint_index = s_endpoint_count++;
        s_endpoints[s_endpoint_index] = endpoint;''', '''        if(s_endpoint_count < CHASSIS_POSITION_ENDPOINT_CAPACITY)
        {
            slot=(s_endpoint_head+s_endpoint_count)%CHASSIS_POSITION_ENDPOINT_CAPACITY;
            s_endpoint_count++;
        }
        else
        {
            slot=s_endpoint_head;
            s_endpoint_head=(s_endpoint_head+1U)%CHASSIS_POSITION_ENDPOINT_CAPACITY;
        }
        s_endpoint_index=slot;
        s_endpoints[slot] = endpoint;'''),
 ('    *record = s_endpoints[index];','    *record = s_endpoints[(s_endpoint_head+index)%CHASSIS_POSITION_ENDPOINT_CAPACITY];'),
 ('''    uint32_t i, j, accepted = 0U;
    if ((dry_run <= 1U) && s_vm_requested''', '''    uint32_t i, j, accepted = 0U;
#if APP_POSITION_DIAGNOSTIC
    if(!Auto_EndpointAvailable()) return 0U;
#endif
    if ((dry_run <= 1U) && s_vm_requested'''),
 ('if (prepared->distance_mm > 0.0f && s_endpoint_count >= CHASSIS_POSITION_ENDPOINT_CAPACITY)', 'if (prepared->distance_mm > 0.0f && !Auto_EndpointAvailable())'),
 ('if (s_endpoint_count >= CHASSIS_POSITION_ENDPOINT_CAPACITY)\n    { taskEXIT_CRITICAL(); return 0U; }','if (!Auto_EndpointAvailable())\n    { taskEXIT_CRITICAL(); return 0U; }')])
 edit(P/'CHASSIS/USER/main.c',[
 ('''    static uint32_t index;
    static char frame[512];
    ChassisPositionEndpoints endpoint;
    uint32_t i, used, phase;''', '''    static uint32_t last_session, session_seen;
    static char frame[512];
    ChassisPositionEndpoints endpoint;
    uint32_t i, used, phase, index, found=0U;'''),
 ('''    if (!ChassisAuto_ReadPositionEndpoints(index, &endpoint)) return;
    if (endpoint.outcome == 2U) { index++; return; }''', '''    /* Getter is logical oldest->newest even after physical ring wrap. */
    for(index=0U;index<CHASSIS_POSITION_ENDPOINT_CAPACITY;index++)
    {
        uint32_t delta;
        if(!ChassisAuto_ReadPositionEndpoints(index,&endpoint)) break;
        delta=endpoint.session-last_session;
        if(!session_seen || (delta>0U && delta<0x80000000U)){found=1U;break;}
    }
    if(!found) return;
    if (endpoint.outcome == 2U || (endpoint.outcome == 1U &&\n        (endpoint.start_mask != 15U || endpoint.end_mask != 15U)))\n    { last_session=endpoint.session;session_seen=1U;return; }'''),
 ('    if (App_DebugSendString(frame) == 1U) index++;','    if (App_DebugSendString(frame) == 1U) {last_session=endpoint.session;session_seen=1U;}')])
 # Separate Pi candidate. Preserve its modules, HSV, J and original start entrypoints.
 shutil.copytree(PIBASE,PI,ignore=shutil.ignore_patterns('captures','records','__pycache__'))
 (PI/'source_alignment_settle.py').write_text((R/'scripts/source_alignment_settle_impl_20261117.py').read_text(encoding='utf-8'),encoding='utf-8',newline='\n')
 (PI/'source-alignment-settle.json').write_text(json.dumps({'format_version':1,'settle_ms':300,'required_frames':5,'maximum_pair_distance_px':2.0,'count_frames_after_dwell':True,'说明':'用户2026-11-17确认：每次停车新pose收到后300ms，然后5张停车后新图盘心最大两两欧氏距离不超过2px；仍晃只等待不新添故障。主机读完成时间不等于曝光时间。'},ensure_ascii=False,indent=2)+'\n',encoding='utf-8')
 edit(PI/'source_fast_cycle_view_20261115.py',[
 ("    from observation_frame_epoch import ObservationFrameEpoch,camera_metadata", "    from observation_frame_epoch import ObservationFrameEpoch,camera_metadata\n    from source_alignment_settle import SourceAlignmentSettle"),
 ("    frame_epoch=ObservationFrameEpoch();epoch_revision=frame_epoch.revision", "    frame_epoch=ObservationFrameEpoch();epoch_revision=frame_epoch.revision\n    settle_config=json.loads((Path(__file__).parent/'source-alignment-settle.json').read_text(encoding='utf-8'))\n    alignment_settle=SourceAlignmentSettle(settle_config)"),
 ("                    gate.reset();direct.reset();initial_rotation.reset();current_run=None", "                    gate.reset();direct.reset();initial_rotation.reset();current_run=None\n                    alignment_settle.invalidate('stage3_status')"),
 ("                region_ui.source_received()\n                status=latest;received=received_at", "                region_ui.source_received()\n                alignment_settle.receive(latest,received_at,link_was_expired=status is not None and received_at-received>.60)\n                status=latest;received=received_at"),
 ("        if not active:frame_epoch.invalidate()", "        if not active:\n            frame_epoch.invalidate();alignment_settle.invalidate('status_inactive')"),
 ("                valid=frame_count>=2 and body['observation_valid']", "                valid=alignment_settle.frame(metadata,now,body['center_px'],body['observation_valid'])"),
 ("            except (ValueError,ArithmeticError):pass\n        study=", "            except (ValueError,ArithmeticError):alignment_settle.reject_fit()\n        study="),
 ("        return dict(mode='M',mcu_status=status,body=body,detection=", "        return dict(mode='M',mcu_status=status,body=body,alignment_settle=alignment_settle.diagnostic(),last_observation_tx=last_observation_tx,detection="),
 ("    status=None;received=0.;buffer=b'';context=None;frame_count=0;sequence=0;last_sent=0.;packets=0", "    status=None;received=0.;buffer=b'';context=None;frame_count=0;sequence=0;last_sent=0.;packets=0;last_observation_tx=None"),
 ("nonlocal status,received,buffer,context,frame_count,sequence,last_sent,packets,window_ready,current_run,epoch_revision", "nonlocal status,received,buffer,context,frame_count,sequence,last_sent,packets,window_ready,current_run,epoch_revision,last_observation_tx"),
 ("                    last_sent=time.monotonic();packets+=1", "                    last_sent=time.monotonic();packets+=1\n                    last_observation_tx=dict(wire=wire.decode('ascii').strip(),sent_monotonic_s=last_sent,capture_metadata=metadata,host_capture_age_ms=age,alignment_gate=alignment_settle.diagnostic())")])
 print('Prepared independent firmware ring + Pi300ms/5frames/2px after MATLAB; no hardware.')
if __name__=='__main__':main()
