"""MATLAB通过后制作一次接齐单件示教接口的隔离工程，不操作板卡。"""
from pathlib import Path
import hashlib,json,shutil,xml.etree.ElementTree as ET
ROOT=Path(__file__).resolve().parents[1]
BASE=ROOT/'tmp/integration10_extension_20261106/project'
STAGE=ROOT/'tmp/integration10_complete_20261106/project'
OUT=ROOT/'evidence/integration/2026-11-06/integration10-complete'
MODULE=ROOT/'software_interfaces/source_parallel_flow'

def main():
    assert json.loads((OUT/'matlab-result.json').read_text(encoding='utf-8'))['passed']
    assert not STAGE.exists(),'保留既有候选'
    shutil.copytree(BASE,STAGE,ignore=shutil.ignore_patterns('Objects','ObjectsOffline','Listings','*.uvguix.*'))
    changes={}
    def edit(name,fn):
        p=STAGE/name;raw=p.read_bytes()
        try:t=raw.decode('utf-8');enc='utf-8'
        except UnicodeDecodeError:t=raw.decode('gbk');enc='gbk'
        newline='\r\n' if b'\r\n' in raw else '\n'
        p.write_bytes(fn(t.replace('\r\n','\n')).replace('\n',newline).encode(enc))
        changes[name]={'encoding':enc,'newline':repr(newline),'before':hashlib.sha256(raw).hexdigest(),'after':hashlib.sha256(p.read_bytes()).hexdigest()}
    def sub(t,a,b):
        assert t.count(a)==1,a
        return t.replace(a,b)
    for name in ['source_parallel_flow.c','source_parallel_flow.h','vehicle_source_parallel.inc','integration10_complete.inc']:
        shutil.copy2(MODULE/name,STAGE/'CHASSIS/USER'/name)
    def header(t):
        decl='''
#include "source_parallel_flow.h"
extern volatile uint32_t g_vehicle_lift_reference_valid;
extern volatile uint32_t g_it_request,g_it_state,g_it_cause,g_it_rejected,g_it_action,g_it_sequence;
extern volatile uint32_t g_it_axes,g_it_elapsed_ms,g_it_lift_permit;
extern volatile float g_it_dm_deg,g_it_extension_mm,g_it_lift_pulses;
void Integration10_Service(void);
uint32_t VehicleArm_BeginSourceParallel(const SPF_Command*,uint32_t,uint32_t);
uint32_t VehicleArm_SourceParallelCanAllowed(uint8_t,const uint8_t*,uint32_t);
extern volatile uint32_t g_source_parallel_state,g_source_parallel_cause,g_source_parallel_stop_mask;
'''
        return sub(t,'#endif',decl+'#endif')
    edit('CHASSIS/USER/vehicle_arm.h',header)
    def arm(t):
        t=sub(t,'#include "carrier_config.h"','#include "carrier_config.h"\n#include "lift_ruler_adapter.h"')
        t=sub(t,'static void SourcePrep_RecordFault(uint32_t why);','''static void SourcePrep_RecordFault(uint32_t why);
static uint32_t VehicleSourceParallel_Owns(void);
static void VehicleSourceParallel_Cancel(void);
static void VehicleSourceParallel_Service(void);''')
        t=sub(t,'{arm_fail(2U);}','{if(VehicleSourceParallel_Owns()){VehicleSourceParallel_Cancel();return;}arm_fail(2U);}')
        prefix='''extern volatile uint32_t g_vm_test_request,g_safety_ready,g_safety_fault,g_rtos_fault;
extern volatile uint32_t g_can_test_result,g_can_guard_ready,g_can_ctrl_fault_latched,g_chassis_auto_active;
static uint32_t SingleTeach_ReferencesReady(void)
{return g_lift_ruler_controller.top_valid && !g_lift_ruler_cause && g_lift_ruler_state==4U &&
 g_startup_axes_state==4U && !g_startup_axes_cause && g_extension_home_reference_valid &&
 g_extension_hold_confirmed && g_vm_test_request==1U && !g_safety_fault && !g_rtos_fault && !g_can_ctrl_fault_latched;}
#if !APP_MOTOR_OFFLINE_ONLY
static float SingleTeach_TopDeg(void)
{return (float)g_lift_ruler_controller.top_position_raw/(float)g_lift_ruler_controller.top_scale;}
#endif
#include "vehicle_source_parallel.inc"
#include "integration10_complete.inc"
'''
        t=sub(t,'uint32_t VehicleArm_Begin(const VehicleArmRow *row,uint32_t item)',prefix+'uint32_t VehicleArm_Begin(const VehicleArmRow *row,uint32_t item)')
        return sub(t,'    if(g_vehicle_arm_stop_pending==5U || g_vehicle_arm_stop_pending==6U) {','    if(VehicleSourceParallel_Owns()){VehicleSourceParallel_Service();return;}\n    if(g_vehicle_arm_stop_pending==5U || g_vehicle_arm_stop_pending==6U) {')
    edit('CHASSIS/USER/vehicle_arm.c',arm)
    def port(t):
        t=t.replace('StartupAxes_Holding()','SingleTeach_ReferencesReady()')
        t=sub(t,'VEHICLE_LIFT_TOP_DEG+VEHICLE_LIFT_DOWN_SIGN*c->goal.lift_pulses','SingleTeach_TopDeg()+VEHICLE_LIFT_DOWN_SIGN*c->goal.lift_pulses')
        t=sub(t,'if(address==5){g_vehicle_lift_current_deg=a->current;g_vehicle_lift_feedback_valid=1;}','if(address==5){g_vehicle_lift_current_deg=a->current;g_vehicle_lift_feedback_valid=1;}\n else {g_startup_extension_current_deg=a->current;g_startup_extension_feedback_valid=1;}')
        t=sub(t,'if(!body.active && (body.cause || body.rotation_stop_mask!=15))return 0;','if(!body.active && body.cause)return 0;')
        t+='''
uint32_t VehicleArm_SourceParallelCanAllowed(uint8_t address,const uint8_t *q,uint32_t n)
{
#if APP_MOTOR_OFFLINE_ONLY
 (void)address;(void)q;(void)n;return 0U;
#else
 uint32_t bit;VSP_Axis *a;
 if(!vsp_owned || !q || (address!=5U && address!=6U))return 0U;
 bit=address==6U?SPF_EXT:SPF_LIFT;a=&vsp_axis[address==6U?0:1];
 if(!(vsp_group.command.axes_mask&bit))return 0U;
 if(n==5U && q[0]==0xf3 && q[1]==0xab && q[2]==1 && q[3]==0 && q[4]==0x6b)
  return vsp_group.state==1U && a->phase==3U;
 return n==15U && vsp_group.state==1U && (vsp_group.started&bit) && !memcmp(q,a->move,15U);
#endif
}
'''
        return t
    edit('CHASSIS/USER/vehicle_source_parallel.inc',port)
    def dm(t):
        t=sub(t,'volatile uint32_t g_dm_source_smooth=0U,g_dm_source_go=0U;','volatile uint32_t g_dm_source_smooth=0U,g_dm_source_go=0U;\nvolatile uint32_t g_dm_parallel_group_active=0U,g_dm_parallel_group_go=0U;')
        t=sub(t,'return g_dm_source_smooth && !g_dm_source_go && g_dm_integrated_owner &&','return ((g_dm_source_smooth && !g_dm_source_go) || (g_dm_parallel_group_active && !g_dm_parallel_group_go)) && g_dm_integrated_owner &&')
        t=sub(t,'s_point_goal <= 180.0F','s_point_goal <= (g_dm_parallel_group_active?270.0F:180.0F)')
        t=sub(t,'current_work_deg <= 182.0F','current_work_deg <= (g_dm_parallel_group_active?272.0F:182.0F)')
        marker='        if(s_source_smooth && !g_dm_source_go){JogReference(s_start,now);return;}'
        return sub(t,marker,marker+'\n        if(g_dm_parallel_group_active && !g_dm_parallel_group_go){JogReference(s_start,now);return;}')
    edit('HARDWARE/DM_CAN2/dm_can2_motion.c',dm)
    def adapter(t):
        t=sub(t,'!g_chassis_auto_active && g_vehicle_arm_state==0U && g_startup_axes_state==0U && !g_dm_jog_request && g_dm_jog_state==0U;',
         '''!g_chassis_auto_active && g_it_lift_permit && (g_vehicle_arm_state==0U || g_vehicle_arm_state==2U) &&
  (g_startup_axes_state==0U || (g_startup_axes_state==4U && !g_startup_axes_cause && g_extension_hold_confirmed)) &&
  !g_dm_jog_request && (g_dm_jog_state==0U || (g_dm_jog_state==7U && g_dm_jog_result==1U && g_dm_posvel_holding &&
   (uint32_t)xTaskGetTickCount()-g_dm_can2_feedback_tick<100U));''')
        marker=' if(request==LRC_REQUEST_START){'
        t=sub(t,marker,' if(request==LRC_REQUEST_START || request==3U)g_lift_ruler_rejected=0U;\n'+marker)
        marker=' if(request==LRC_REQUEST_STOP)LiftRuler_Stop(&g_lift_ruler_controller,now);'
        t=sub(t,marker,''' if(request==3U && !LiftRuler_StartReturn(&g_lift_ruler_controller,now,ready))g_lift_ruler_rejected=1U;
'''+marker)
        return sub(t,'return VEHICLE_LIFT_RULER_ONLY && address==5U && LiftRuler_CommandAllowed(&g_lift_ruler_controller,q,n);',
                   'return (VEHICLE_LIFT_RULER_ONLY && address==5U && LiftRuler_CommandAllowed(&g_lift_ruler_controller,q,n)) || VehicleArm_SourceParallelCanAllowed(address,q,n);')
    edit('CHASSIS/USER/lift_ruler_adapter.c',adapter)
    edit('CHASSIS/USER/lift_ruler_calibration.h',lambda t:sub(t,'void LiftRuler_Stop(', 'uint32_t LiftRuler_StartReturn(LiftRulerController*,uint32_t now,uint32_t ready);\nvoid LiftRuler_Stop('))
    def lift(t):
        marker='void LiftRuler_Stop(LiftRulerController*c,uint32_t now)'
        api='''uint32_t LiftRuler_StartReturn(LiftRulerController*c,uint32_t now,uint32_t ready)
{
 uint32_t previous;
 if(!c || !ready || c->state!=LRC_DONE || !c->top_valid || c->cause || c->stop_pending)return 0U;
 previous=c->operation;c->operation=2U;
 if(LiftRuler_Start(c,now,ready))return 1U;
 c->operation=previous;return 0U;
}
'''
        return sub(t,marker,api+marker)
    edit('CHASSIS/USER/lift_ruler_calibration.c',lift)
    def main_c(t):
        t=sub(t,'#include "integration10_extension.inc"','/* 单件完整接口由vehicle_arm.c提供，旧伸缩单项入口不再编译。 */')
        t=sub(t,'if(VEHICLE_LIFT_RULER_ONLY){LiftRuler_DebugService();Integration10_ExtensionService();}',
         '''if(VEHICLE_LIFT_RULER_ONLY){
#if !APP_MOTOR_OFFLINE_ONLY
                 DM_CAN2_ModuleService(g_safety_ready && !g_safety_fault && !g_rtos_fault && g_vm_test_request,0U);
#endif
                 LiftRuler_DebugService();Integration10_Service();
                }''')
        return t
    edit('CHASSIS/USER/main.c',main_c)
    tree=ET.parse(STAGE/'gongxun-f407.uvprojx')
    for target in tree.getroot().findall('./Targets/Target'):
        group=ET.SubElement(target.find('Groups'),'Group');ET.SubElement(group,'GroupName').text='SingleTeachGroup'
        files=ET.SubElement(group,'Files');f=ET.SubElement(files,'File')
        for tag,value in [('FileName','source_parallel_flow.c'),('FileType','1'),('FilePath','.\\CHASSIS\\USER\\source_parallel_flow.c')]:ET.SubElement(f,tag).text=value
    tree.write(STAGE/'gongxun-f407.uvprojx',encoding='UTF-8',xml_declaration=True)
    (OUT/'staged.json').write_text(json.dumps({'candidate':str(STAGE),'changes':changes,'firmware_downloaded':False,'ordinary_axes_available':True,'VM_held_after_normal_done':True},indent=2),encoding='utf-8')
    print('Prepared complete single-teach candidate; no hardware action.')

if __name__=='__main__':main()
