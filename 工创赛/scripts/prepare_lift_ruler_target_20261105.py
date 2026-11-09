"""构建独立升降标定工程副本；不更改实际工程，不烧录或操作硬件。"""
from pathlib import Path
import hashlib
import json
import shutil
import xml.etree.ElementTree as ET
from finish_lift_ruler_candidate_20261105 import finish

ROOT = Path(__file__).resolve().parents[1]
ACTUAL = Path("D:/新建文件夹4/工创赛/代码/stm32/3.stm32f407工程")
STAGE = ROOT / "tmp/lift_ruler_20261105/project"
EVIDENCE = ROOT / "evidence/integration/2026-11-05/lift-ruler-candidate"
MODULE = ROOT / "software_interfaces/lift_ruler_calibration"


def once(text, old, new):
    assert text.count(old) == 1, old[:110]
    return text.replace(old, new)


def main():
    proof = json.loads((EVIDENCE / "checks.json").read_text(encoding="utf-8"))
    assert proof["passed"] and proof["matlab_checks"] == 89
    assert not STAGE.exists(), "Preserve existing candidate; do not silently overwrite"
    shutil.copytree(ACTUAL, STAGE, ignore=shutil.ignore_patterns("Objects", "ObjectsOffline", "Listings", ".git"))
    original = {}
    def edit(relative, function):
        source = ACTUAL / relative
        raw = source.read_bytes()
        original[relative] = hashlib.sha256(raw).hexdigest()
        try:
            text = raw.decode("utf-8")
            encoding = "utf-8"
        except UnicodeDecodeError:
            text = raw.decode("gbk")
            encoding = "gbk"
        text = function(text.replace("\r\n", "\n"))
        (STAGE / relative).write_bytes(text.replace("\n", "\r\n").encode(encoding))
    def params(t):
        t = once(t, "#define VEHICLE_SOURCE_PREP_ENABLE 1U", "#define VEHICLE_LIFT_RULER_ONLY 1U /* isolated debug candidate, one node5 step only */\n#define VEHICLE_SOURCE_PREP_ENABLE 1U")
        return once(t, "#define STARTUP_GRIPPER_ANGLE_DEG 145.0f", "#define STARTUP_GRIPPER_ANGLE_DEG 210.0f /* this isolated candidate preserves user's current open command */")
    edit("USER/vehicle_parameters.h", params)
    def main_c(t):
        t = once(t, '#include "vehicle_arm.h"', '#include "vehicle_arm.h"\n#include "lift_ruler_adapter.h"')
        old = '''                StartupAxes_Service(g_safety_ready && g_can_test_result==1U,g_can_guard_ready);
                if(!VEHICLE_EXTENSION_HOME_ONLY && StartupAxes_Holding())DM_CAN2_ModuleService(1U,0U);
                if(!VEHICLE_EXTENSION_HOME_ONLY && !VEHICLE_DM_STARTUP_ONLY && !VEHICLE_STARTUP_GROUP_ONLY && (VehicleArm_Busy() || g_vehicle_arm_stop_pending))VehicleArm_Service();
                if(!VEHICLE_EXTENSION_HOME_ONLY && StartupAxes_Holding())DM_CAN2_ModuleService(1U,0U);
                if(!VEHICLE_EXTENSION_HOME_ONLY && !VEHICLE_DM_STARTUP_ONLY && !VEHICLE_STARTUP_GROUP_ONLY && !VehicleArm_Busy() && (!VEHICLE_INTEGRATED_ENABLE || StartupAxes_Holding()) && g_startup_axes_state!=1U && g_startup_axes_state!=2U && g_startup_axes_state!=3U) App_CombinedTestStep();'''
        t = once(t, old, "                if(VEHICLE_LIFT_RULER_ONLY)LiftRuler_DebugService();\n                else {\n" + old + "\n                }")
        old = '''        StartupAxes_Safety(!g_rtos_fault && !g_safety_fault && !g_can_ctrl_fault_latched);
        if(g_startup_axes_state==5U && (g_combined_test.state==1U || g_combined_test.state==2U)) MotionStop();
        App_CombinedTestSafety();'''
        t = once(t, old, "        if(VEHICLE_LIFT_RULER_ONLY)LiftRuler_DebugSafety();\n        else {\n" + old + "\n        }")
        # The sole remaining idle-poll DM hook is excluded too; wheel feedback queries remain.
        t = t.replace("if(!VEHICLE_EXTENSION_HOME_ONLY && StartupAxes_Holding())DM_CAN2_ModuleService(1U,0U);",
                      "if(!VEHICLE_LIFT_RULER_ONLY && !VEHICLE_EXTENSION_HOME_ONLY && StartupAxes_Holding())DM_CAN2_ModuleService(1U,0U);")
        old = "        RadarLift_Service((uint32_t)(xTaskGetTickCount()*portTICK_PERIOD_MS),"
        t = once(t, old, "        if(!VEHICLE_LIFT_RULER_ONLY) RadarLift_Service((uint32_t)(xTaskGetTickCount()*portTICK_PERIOD_MS),")
        t = once(t, "        RadarServo_DebugService((g_vm_test_request==0U", "        if(!VEHICLE_LIFT_RULER_ONLY) RadarServo_DebugService((g_vm_test_request==0U")
        return t
    edit("CHASSIS/USER/main.c", main_c)
    def mission(t):
        marker = "    taskENTER_CRITICAL();key=g_vehicle_key;g_vehicle_key=0U;taskEXIT_CRITICAL();"
        t = once(t, marker, marker + "\n    if(VEHICLE_LIFT_RULER_ONLY){if(key)LiftRuler_DebugKey();return;}")
        marker = "    if(g_extension_calibration_only){\n        static uint32_t cal_last;"
        telemetry = '''    if(VEHICLE_LIFT_RULER_ONLY){
        static uint32_t ruler_last;
        if(now-ruler_last<500U)return;
        count=snprintf(frame,sizeof(frame),"LR1,%lu,%lu,%lu,%lu,%lu,5000,300,%lu,%ld,%ld,%ld,%lu,%lu,%lu,%lu,%lu,",
        (unsigned long)now,(unsigned long)g_lift_ruler_state,(unsigned long)g_lift_ruler_phase,
        (unsigned long)g_lift_ruler_cause,(unsigned long)g_lift_ruler_samples,(unsigned long)g_lift_ruler_options,
        (long)g_lift_ruler_before_raw,(long)g_lift_ruler_after_raw,(long)g_lift_ruler_target_raw,
        (unsigned long)g_lift_ruler_encoder_before,(unsigned long)g_lift_ruler_encoder_after,
        (unsigned long)g_lift_ruler_stop_pending,(unsigned long)g_lift_ruler_stop_unconfirmed,(unsigned long)g_lift_ruler_vm_request);
        if(count<=0 || (uint32_t)count>=sizeof(frame)-7U)return;
        crc=App_SerialCrc(frame,(uint32_t)count);snprintf(frame+count,sizeof(frame)-count,"%04X\\r\\n",crc);
        if(App_DebugSendString(frame))ruler_last=now;return;
    }
'''
        return once(t, marker, telemetry + marker)
    edit("CHASSIS/USER/vehicle_mission.inc", mission)
    def can(t):
        t = once(t, "uint32_t VehicleArm_Exchange(uint8_t address", '#include "lift_ruler_adapter.h"\nuint32_t VehicleArm_Exchange(uint8_t address')
        old = '''    if(!g_startup_axes_vm_request || g_startup_axes_state<2U || g_startup_axes_state>4U ||
       g_safety_fault || g_rtos_fault || g_can_ctrl_fault_latched)return 9U;'''
        new = '''    if(g_safety_fault || g_rtos_fault || g_can_ctrl_fault_latched ||
       ((!g_startup_axes_vm_request || g_startup_axes_state<2U || g_startup_axes_state>4U) &&
        !LiftRuler_DebugCanAllowed(address,q,length)))return 9U;'''
        return once(t, old, new)
    edit("CHASSIS/SYSTEM/CAN/can.c", can)
    for name in ["lift_ruler_calibration.c", "lift_ruler_calibration.h", "lift_ruler_adapter.c", "lift_ruler_adapter.h"]:
        relative = "CHASSIS/USER/" + name
        assert not (ACTUAL / relative).exists()
        shutil.copy2(MODULE / name, STAGE / relative)
        original[relative] = None
    project = STAGE / "gongxun-f407.uvprojx"
    original["gongxun-f407.uvprojx"] = hashlib.sha256((ACTUAL / "gongxun-f407.uvprojx").read_bytes()).hexdigest()
    tree = ET.parse(project)
    names = []
    for target in tree.getroot().findall("./Targets/Target"):
        previous = target.findtext("TargetName")
        name = "Vehicle_LiftRuler_Offline" if "Offline" in previous else "Vehicle_LiftRuler"
        target.find("TargetName").text = name
        common = target.find("./TargetOption/TargetCommonOption")
        common.find("OutputName").text = name
        common.find("OutputDirectory").text = ".\\Objects\\" + name + "\\"
        common.find("ListingPath").text = ".\\Listings\\" + name + "\\"
        group = ET.SubElement(target.find("Groups"), "Group")
        ET.SubElement(group, "GroupName").text = "Lift_Ruler_Debug_Only"
        files = ET.SubElement(group, "Files")
        for filename in ["lift_ruler_calibration.c", "lift_ruler_adapter.c"]:
            file = ET.SubElement(files, "File")
            ET.SubElement(file, "FileName").text = filename
            ET.SubElement(file, "FileType").text = "1"
            ET.SubElement(file, "FilePath").text = ".\\CHASSIS\\USER\\" + filename
        names.append(name)
    tree.write(project, encoding="utf-8", xml_declaration=True)
    finish(STAGE)
    manifest = {"actual": str(ACTUAL), "stage": str(STAGE), "targets": names, "source_before_sha256": original,
                "changed_sha256": {n: hashlib.sha256((STAGE / n).read_bytes()).hexdigest() for n in original},
                "installed_in_actual": False, "firmware_downloaded": False, "hardware_operated": False,
                "step": "one node5 downward5000 virtual pulses after PC13", "ordinary_boot_motion": False}
    (EVIDENCE / "staged-manifest.json").write_text(json.dumps(manifest, ensure_ascii=False, indent=2), encoding="utf-8")
    print(json.dumps({"stage": str(STAGE), "targets": names, "changed_files": len(original)}, ensure_ascii=False, indent=2))


if __name__ == "__main__":
    main()
