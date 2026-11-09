"""Build an isolated integrated-project candidate; never edit/flash actual car."""
from pathlib import Path
import hashlib
import json
import shutil
import xml.etree.ElementTree as ET

ROOT=Path(__file__).resolve().parents[1]
BASE=ROOT/'tmp/source_observer_20261106/project'
OUT=ROOT/'tmp/source_parallel_final_20261106/project'
PROOF=ROOT/'evidence/integration/2026-11-06/source-parallel-plan'
MODULE=ROOT/'software_interfaces/source_parallel_flow'


def main():
    assert json.loads((PROOF/'matlab-result.json').read_text())['passed']
    assert json.loads((PROOF/'flow-result.json').read_text())['passed']
    assert not OUT.exists(),'Preserve previous candidate'
    shutil.copytree(BASE,OUT,ignore=shutil.ignore_patterns('Objects','ObjectsOffline','Listings','*.uvguix.*','__pycache__'))
    changes={}
    def edit(name,fn):
        p=OUT/name;raw=p.read_bytes()
        try:s=raw.decode('utf-8');enc='utf-8'
        except UnicodeDecodeError:s=raw.decode('gbk');enc='gbk'
        newline='\r\n' if b'\r\n' in raw else '\n'
        s=fn(s.replace('\r\n','\n'))
        p.write_bytes(s.replace('\n',newline).encode(enc))
        changes[name]={'before_sha256':hashlib.sha256(raw).hexdigest(),'after_sha256':hashlib.sha256(p.read_bytes()).hexdigest(),'encoding':enc,'newline':repr(newline)}
    for name in ['source_parallel_flow.h','source_parallel_flow.c','vehicle_source_parallel.inc']:
        shutil.copy2(MODULE/name,OUT/'CHASSIS/USER'/name)
    def arm(s):
        marker='uint32_t VehicleArm_Begin(const VehicleArmRow *row,uint32_t item)'
        assert s.count(marker)==1
        s=s.replace(marker,'#include "vehicle_source_parallel.inc"\n'+marker)
        s=s.replace('static void SourcePrep_RecordFault(uint32_t why);','static void SourcePrep_RecordFault(uint32_t why);\nstatic uint32_t VehicleSourceParallel_Owns(void);\nstatic void VehicleSourceParallel_Cancel(void);\nstatic void VehicleSourceParallel_Service(void);')
        s=s.replace('{arm_fail(2U);}','{if(VehicleSourceParallel_Owns()){VehicleSourceParallel_Cancel();return;}arm_fail(2U);}')
        marker='    if(g_vehicle_arm_stop_pending==5U || g_vehicle_arm_stop_pending==6U) {'
        assert s.count(marker)==1
        return s.replace(marker,'    if(VehicleSourceParallel_Owns()){VehicleSourceParallel_Service();return;}\n'+marker)
    edit('CHASSIS/USER/vehicle_arm.c',arm)
    def header(s):
        s=s.replace('#include "../../USER/vehicle_parameters.h"','#include "../../USER/vehicle_parameters.h"\n#include "source_parallel_flow.h"')
        return s.replace('#endif','uint32_t VehicleArm_BeginSourceParallel(const SPF_Command *,uint32_t sweep_verified,uint32_t driving_stow_verified);\nuint32_t VehicleArm_SourceParallelAllowsBody(void);\n#endif')
    edit('CHASSIS/USER/vehicle_arm.h',header)
    def dm(s):
        marker='volatile uint32_t g_dm_source_smooth=0U,g_dm_source_go=0U;'
        assert s.count(marker)==1
        s=s.replace(marker,marker+'\nvolatile uint32_t g_dm_parallel_group_active=0U,g_dm_parallel_group_go=0U;')
        s=s.replace('return g_dm_source_smooth && !g_dm_source_go && g_dm_integrated_owner &&','return ((g_dm_source_smooth && !g_dm_source_go) || (g_dm_parallel_group_active && !g_dm_parallel_group_go)) && g_dm_integrated_owner &&')
        s=s.replace('s_point_goal <= 180.0F','s_point_goal <= (g_dm_parallel_group_active ? 270.0F : 180.0F)')
        s=s.replace('current_work_deg <= 182.0F','current_work_deg <= (g_dm_parallel_group_active ? 272.0F : 182.0F)')
        marker='        if(s_source_smooth && !g_dm_source_go){JogReference(s_start,now);return;}'
        assert s.count(marker)==1
        return s.replace(marker,marker+'\n        if(g_dm_parallel_group_active && !g_dm_parallel_group_go){JogReference(s_start,now);return;}')
    edit('HARDWARE/DM_CAN2/dm_can2_motion.c',dm)
    def main_c(s):
        old='(VehicleArm_Busy() || g_vehicle_arm_stop_pending))VehicleArm_Service();'
        # A group fault needs all-axis STOP retries even after arm state=3.
        new='(VehicleArm_Busy() || g_vehicle_arm_stop_pending || g_source_parallel_stop_mask))VehicleArm_Service();'
        assert s.count(old)==1;s=s.replace(old,new)
        old='&& !VehicleArm_Busy() && (!VEHICLE_INTEGRATED_ENABLE || StartupAxes_Holding())'
        new='&& (!VehicleArm_Busy() || VehicleArm_SourceParallelAllowsBody()) && (!VEHICLE_INTEGRATED_ENABLE || StartupAxes_Holding())'
        assert s.count(old)==1;return s.replace(old,new)
    edit('CHASSIS/USER/main.c',main_c)
    edit('CHASSIS/USER/vehicle_arm.h',lambda s:s.replace('uint32_t VehicleArm_SourceParallelAllowsBody(void);','uint32_t VehicleArm_SourceParallelAllowsBody(void);\nextern volatile uint32_t g_source_parallel_state,g_source_parallel_cause,g_source_parallel_started_mask,g_source_parallel_done_mask,g_source_parallel_stop_mask,g_source_parallel_start_skew_ms;'))
    def params(s):
        marker='/* ---------- 各段机械臂动作：'
        assert marker in s
        comment='''/* 源区并行最终流程候选：保留旧逐行流程作为回退，未接新M目标事务前不自动启用。
 * 运行接口接受一次三轴目标，速度固定为300deg/s、106RPM、1846.4RPM。
 * 三槽盘高度待测，不用旧28000；启动姿态须从本轮StartupAxes取值。
 * 最后一件放妥并松爪后，第三段行驶与机构回初始化同批发布。
 */
#define VEHICLE_SOURCE_FINAL_FLOW_ENABLED 0U
#define VEHICLE_SOURCE_TRANSFER_HEIGHT_VALID 0U
#define VEHICLE_SOURCE_CARRIER_SURFACE_DELTA_MM (-100000.0f)
#define VEHICLE_SOURCE_CARRIER_CLEARANCE_MM 5.0f
#define VEHICLE_SOURCE_PARALLEL_MAX_EXTENSION_MM 170.0f
#define VEHICLE_SOURCE_PARALLEL_MAX_DM_DEG 270.0f

'''
        return s.replace(marker,comment+marker)
    edit('USER/vehicle_parameters.h',params)
    project=OUT/'gongxun-f407.uvprojx';tree=ET.parse(project)
    for target in tree.getroot().findall('./Targets/Target'):
        offline='Offline' in target.findtext('TargetName')
        target.find('TargetName').text='Vehicle_SourceParallel_Offline' if offline else 'Vehicle_SourceParallel'
        name=target.findtext('TargetName');common=target.find('TargetOption/TargetCommonOption')
        for tag,value in [('OutputDirectory',f'.\\Objects\\{name}\\'),('OutputName',name),('ListingPath',f'.\\Listings\\{name}\\')]:
            e=common.find(tag)
            if e is not None:e.text=value
        groups=target.find('Groups');group=ET.SubElement(groups,'Group');ET.SubElement(group,'GroupName').text='SourceParallelFlow'
        files=ET.SubElement(group,'Files');f=ET.SubElement(files,'File')
        for tag,value in [('FileName','source_parallel_flow.c'),('FileType','1'),('FilePath','.\\CHASSIS\\USER\\source_parallel_flow.c')]:ET.SubElement(f,tag).text=value
    tree.write(project,encoding='UTF-8',xml_declaration=True)
    # Protect both the original candidate and actual car source from accidental edits.
    protected=['CHASSIS/USER/vehicle_arm.c','USER/vehicle_parameters.h','CHASSIS/USER/main.c','HARDWARE/DM_CAN2/dm_can2_motion.c']
    report={'candidate':str(OUT),'changes':changes,'base_source_sha256':{n:hashlib.sha256((BASE/n).read_bytes()).hexdigest() for n in protected},
      'actual_project_source_sha256':{n:hashlib.sha256((Path('D:/新建文件夹4/工创赛/代码/stm32/3.stm32f407工程')/n).read_bytes()).hexdigest() for n in protected},
      'runtime_final_flow_auto_enabled':False,'source_target_receiver_bound':False,'chassis_alignment_executor_bound':False,
      'compiled_port_includes_real_CAN_exchange':True,'firmware_downloaded':False,'hardware_accessed':False,'motion_allowed':False}
    (PROOF/'candidate-manifest.json').write_text(json.dumps(report,ensure_ascii=False,indent=2),encoding='utf-8')
    print('Prepared isolated full-project candidate with real parallel CAN port and portable flow; automatic new flow remains disabled pending actual profile and mission bindings.')


if __name__=='__main__':main()
