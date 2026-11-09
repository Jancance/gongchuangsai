"""MATLAB结果→独立C几何/反馈检查候选，只有离线计算，无硬件入口。"""
from pathlib import Path
import hashlib
import json
import subprocess

ROOT = Path(__file__).resolve().parents[1]
BASE = ROOT / "evidence/integration/2026-11-05/source-markers/paper100-transfer/continuation/body270x250"
OUT = BASE / "geometry-candidate"
HEADER = ROOT / "software_interfaces/source_pick_geometry/source_pick_geometry.h"


def main():
    OUT.mkdir(parents=True, exist_ok=True)
    matlab = json.loads((BASE / "matlab-body-pose/result.json").read_text(encoding="utf-8"))
    top = json.loads((BASE / "matlab-top-sweep/result.json").read_text(encoding="utf-8"))
    assert matlab["algorithm_failures"] == 0 and top["abstract_top_motion_checks_passed"]
    points = matlab["targets_initial_body_right_forward_mm"]
    expected = matlab["maximum_margin_within100mm_yaw30"]
    shift = expected["chassis_right_forward_mm"]
    def num(value):
        return f"{float(value):.10e}f"
    rows = []
    for point, extension, angle in zip(points, expected["extension_mm"], expected["logical_deg"]):
        rows.append("{" + ",".join(num(v) for v in (*point, extension, angle)) + "}")
    prefix = "static const float vectors[3][4]={" + ",".join(rows) + "};\n"
    prefix += "static const float shift[2]={" + ",".join(map(num, shift)) + "};\n"
    unit = r'''
#include <stdio.h>
#include "source_pick_geometry.h"
static unsigned checks,failures;
#define CK(x) do{checks++;if(!(x)){failures++;printf("FAIL line %u\n",__LINE__);}}while(0)
int main(void){
 SourcePickGeometry g={165,170,180,0,360,10},bad;
 SourcePickTarget t,saved;
 SourcePickTolerance fine={.2f,.1f,.5f,.5f,2,100};
 unsigned k;float err;
 for(k=0;k<3;k++){
  CK(SourcePick_Solve(&g,vectors[k][0],vectors[k][1],shift[0],shift[1],0,&t)==SPG_MARGIN);
  CK(fabsf(t.extension_mm-vectors[k][2])<.0005f);
  CK(fabsf(t.logical_deg-vectors[k][3])<.0005f);
  CK(t.geometry_in_range==1 && !t.requested_margin_met && !t.motion_allowed);
  CK(!SourcePick_FirmwareSupports(&t,180,50));
  CK(SourcePick_FirmwareSupports(&t,360,170)); /* 比较假设能力，未写入固件。 */
  CK(SourcePick_SampleWithinTolerance(&g,&t,&fine,t.logical_deg,t.extension_mm,0,0,100,80,80,1,&err));
  CK(err<.0001f);
  CK(!SourcePick_SampleWithinTolerance(&g,&t,&fine,t.logical_deg,t.extension_mm,0,0,100,0,80,1,&err));
  CK(!SourcePick_SampleWithinTolerance(&g,&t,&fine,t.logical_deg,t.extension_mm,0,0,100,80,0,1,&err));
  CK(!SourcePick_SampleWithinTolerance(&g,&t,&fine,t.logical_deg,t.extension_mm,0,0,100,80,80,0,&err));
  CK(!SourcePick_SampleWithinTolerance(&g,&t,&fine,t.logical_deg+1.6f,t.extension_mm,0,0,100,80,80,1,&err));
  CK(!SourcePick_SampleWithinTolerance(&g,&t,&fine,t.logical_deg,t.extension_mm+.11f,0,0,100,80,80,1,&err));
  CK(!SourcePick_SampleWithinTolerance(&g,&t,&fine,t.logical_deg,t.extension_mm,.51f,0,100,80,80,1,&err));
  CK(!SourcePick_SampleWithinTolerance(&g,&t,&fine,t.logical_deg,t.extension_mm,0,.51f,100,80,80,1,&err));
  CK(SourcePick_SampleWithinTolerance(&g,&t,&fine,t.logical_deg,t.extension_mm,0,0,9,0xfffffff0U,0xfffffff0U,1,&err));
  CK(!SourcePick_SampleWithinTolerance(&g,&t,&fine,t.logical_deg,t.extension_mm,0,0,100,101,80,1,&err));
 }
 CK(SourcePick_Solve(&g,165,0,0,0,0,&t)==SPG_MARGIN && t.extension_mm==0 && !t.motion_allowed);
 CK(SourcePick_Solve(&g,335,0,0,0,0,&t)==SPG_MARGIN && t.extension_mm==170);
 CK(SourcePick_Solve(&g,164.99f,0,0,0,0,&t)==SPG_STROKE && !t.geometry_in_range);
 CK(SourcePick_Solve(&g,335.01f,0,0,0,0,&t)==SPG_STROKE && !t.geometry_in_range);
 CK(SourcePick_Solve(&g,235,0,0,0,90,&t)==SPG_OK && fabsf(t.logical_deg-90)<.0001f);
 CK(!t.motion_allowed && t.requested_margin_met);
 saved=t;bad=g;bad.max_extension_mm=NAN;
 CK(SourcePick_Solve(&bad,235,0,0,0,0,&t)==SPG_ARGUMENT && !t.geometry_in_range && !t.motion_allowed);
 CK(SourcePick_Solve(&g,NAN,0,0,0,0,&t)==SPG_ARGUMENT);
 CK(SourcePick_Solve(&g,INFINITY,0,0,0,0,&t)==SPG_ARGUMENT);
 CK(SourcePick_Solve(&g,0,0,0,0,0,&t)==SPG_ARGUMENT);
 CK(SourcePick_Solve(&g,235,0,0,0,0,0)==SPG_ARGUMENT);
 CK(!SourcePick_FirmwareSupports(0,360,170));
 saved.logical_deg=-1;CK(!SourcePick_FirmwareSupports(&saved,360,170));
 saved.logical_deg=90;saved.extension_mm=-1;CK(!SourcePick_FirmwareSupports(&saved,360,170));
 saved.extension_mm=70;saved.motion_allowed=1;CK(!SourcePick_FirmwareSupports(&saved,360,170));
 CK(!SourcePick_SampleWithinTolerance(&g,&saved,&fine,90,70,0,0,10,0,0,1,&err));
 CK(fabsf(SPG_AngleError(359,1)+2)<.0001f);
 CK(fabsf(SPG_AngleError(1,359)-2)<.0001f);
 printf("SOURCE_GEOMETRY checks=%u failures=%u\n",checks,failures);return failures?1:0;
}
'''
    source = OUT / "geometry-check.c"
    source.write_text(prefix + unit, encoding="utf-8")
    exe = OUT / "geometry-check.exe"
    compile_cmd = ["C:/mingw/mingw64/bin/gcc.exe", "-std=c99", "-Wall", "-Wextra", "-Werror",
                   "-I" + str(HEADER.parent), str(source), "-o", str(exe), "-lm"]
    built = subprocess.run(compile_cmd, capture_output=True, text=True)
    (OUT / "host-compile.log").write_text(built.stdout + built.stderr, encoding="utf-8")
    assert built.returncode == 0, built.stderr
    ran = subprocess.run([str(exe)], capture_output=True, text=True)
    (OUT / "checks.log").write_text(ran.stdout + ran.stderr, encoding="utf-8")
    assert ran.returncode == 0, ran.stdout + ran.stderr
    arm = Path("C:/Keil_v5/ARM/ARMCC/bin/armcc.exe")
    arm_result = subprocess.run([str(arm), "--cpu", "Cortex-M4.fp.sp", "--c99", "-c", "-I" + str(HEADER.parent),
                                 str(source), "-o", str(OUT / "geometry-check.o")], capture_output=True, text=True)
    (OUT / "armcc-compile.log").write_text(arm_result.stdout + arm_result.stderr, encoding="utf-8")
    assert arm_result.returncode == 0, arm_result.stdout + arm_result.stderr
    report = {"passed": True, "host_output": ran.stdout.strip(), "host_warning_as_error": True,
              "armcc_cortex_m4_compile_exit": arm_result.returncode,
              "matlab_vectors": str((BASE / "matlab-body-pose/result.json").relative_to(ROOT)),
              "matlab_top_model": str((BASE / "matlab-top-sweep/result.json").relative_to(ROOT)),
              "actual_firmware_changed": False, "firmware_downloaded": False, "hardware_operated": False,
              "motion_allowed": False,
              "sha256": {str(p.relative_to(ROOT)): hashlib.sha256(p.read_bytes()).hexdigest()
                         for p in [HEADER, Path(__file__).resolve(), source, BASE / "matlab-body-pose/result.json",
                                   BASE / "matlab-top-sweep/result.json"]}}
    (OUT / "result.json").write_text(json.dumps(report, ensure_ascii=False, indent=2), encoding="utf-8")
    print(json.dumps(report, ensure_ascii=False, indent=2))


if __name__ == "__main__":
    main()
