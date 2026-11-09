"""当前255/94/170尺量的MATLAB结果与独立C几何接口交叉检查，无硬件。"""
from pathlib import Path
import hashlib
import json
import subprocess

ROOT = Path(__file__).resolve().parents[1]
BASE = ROOT / "evidence/integration/2026-11-05/source-markers/current-221545-a3467c481720"
OUT = BASE / "mechanics255-e94/c-cross-check"
HEADER = ROOT / "software_interfaces/source_pick_geometry/source_pick_geometry.h"


def main():
    result_path = BASE / "mechanics255-e94/matlab-result.json"
    result = json.loads(result_path.read_text(encoding="utf-8"))
    scene = json.loads((BASE / "matlab-result.json").read_text(encoding="utf-8"))
    assert result["math_processing_passed"] and result["failure_count"] == 0
    assert result["current_measured_pivot_to_tcp_mm"] == 255 and result["retracted_radius_derived_mm"] == 161
    comparison = result["same_stop_comparison"]
    assert isinstance(comparison, dict) and comparison["max_extension_mm"] == 170
    assert comparison["reconfirmed_for_current_ruler_pair"]
    expected = comparison["best_nearby_single_stop"]
    OUT.mkdir(parents=True, exist_ok=True)
    def f(value):
        return f"{float(value):.10e}f"
    vectors = []
    for offset, extension, angle in zip(scene["marker_offsets_image_right_extension_forward_mm"], expected["extension_mm"], expected["logical_deg"]):
        vectors.append("{" + ",".join(f(v) for v in (255 + offset[1], -offset[0], extension, angle)) + "}")
    prefix = "static const float vectors[3][4]={" + ",".join(vectors) + "};\n"
    prefix += "static const float shift[2]={" + ",".join(map(f, expected["chassis_right_forward_shift_mm"])) + "};\n"
    unit = r'''
#include <stdio.h>
#include "source_pick_geometry.h"
int main(void){
 SourcePickGeometry g={161,170,180,0,360,10};SourcePickTarget target;
 unsigned k,checks=0,failures=0;
 for(k=0;k<3;k++){
  unsigned rc=SourcePick_Solve(&g,vectors[k][0],vectors[k][1],shift[0],shift[1],0,&target);
  checks++;if(rc!=SPG_MARGIN)failures++;
  checks++;if(fabsf(target.extension_mm-vectors[k][2])>.0005f)failures++;
  checks++;if(fabsf(target.logical_deg-vectors[k][3])>.0005f)failures++;
  checks++;if(target.motion_allowed || !target.geometry_in_range || target.requested_margin_met)failures++;
  checks++;if(SourcePick_FirmwareSupports(&target,180,50))failures++;
 }
 printf("CURRENT_PAIR checks=%u failures=%u\n",checks,failures);return failures?1:0;
}
'''
    source = OUT / "current-pair-check.c"
    source.write_text(prefix + unit, encoding="utf-8")
    exe = OUT / "current-pair-check.exe"
    command = ["C:/mingw/mingw64/bin/gcc.exe", "-std=c99", "-Wall", "-Wextra", "-Werror",
               "-I" + str(HEADER.parent), str(source), "-o", str(exe), "-lm"]
    compiled = subprocess.run(command, capture_output=True, text=True)
    (OUT / "compile.log").write_text(compiled.stdout + compiled.stderr, encoding="utf-8")
    assert compiled.returncode == 0, compiled.stderr
    ran = subprocess.run([str(exe)], capture_output=True, text=True)
    (OUT / "checks.log").write_text(ran.stdout + ran.stderr, encoding="utf-8")
    assert ran.returncode == 0, ran.stdout + ran.stderr
    arm = subprocess.run(["C:/Keil_v5/ARM/ARMCC/bin/armcc.exe", "--cpu", "Cortex-M4.fp.sp", "--c99", "-c",
                          "-I" + str(HEADER.parent), str(source), "-o", str(OUT / "current-pair-check.o")], capture_output=True, text=True)
    (OUT / "armcc.log").write_text(arm.stdout + arm.stderr, encoding="utf-8")
    assert arm.returncode == 0, arm.stdout + arm.stderr
    report = {"passed": True, "host_output": ran.stdout.strip(), "host_warnings_as_errors": True,
              "armcc_unit_compile_exit": arm.returncode, "actual_firmware_changed": False,
              "current_limits_relaxed": False, "hardware_operated": False, "motion_allowed": False,
              "sha256": {p.relative_to(ROOT).as_posix(): hashlib.sha256(p.read_bytes()).hexdigest()
                         for p in [HEADER, Path(__file__).resolve(), result_path, source]}}
    (OUT / "result.json").write_text(json.dumps(report, indent=2), encoding="utf-8")
    print(json.dumps(report, indent=2))


if __name__ == "__main__":
    main()
