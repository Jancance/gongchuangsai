function validate_source_pick_clearance_matlab_20261105
% 用户50mm抓取底面候选与45/48/49mm对比；无升降脉冲换算或执行命令。
root=fileparts(fileparts(mfilename('fullpath')));
base=fullfile(root,'evidence','integration','2026-11-05','source-markers','paper100-transfer','continuation');
out=fullfile(base,'body270x250','matlab-pick-clearance');if ~exist(out,'dir'),mkdir(out);end
ref=read_json(fullfile(root,'calibration','source_tcp_center_reference_20261105.json'));
top=ref.scene_measurements.gripper_tip_above_source_plane_mm;
material=ref.scene_measurements.material_height_mm;
assert(ref.scene_measurements.top_tip_height_confirmed && top==119 && material==60);
grips=[50,49,48,45];clearance_goal=10;
rows=struct('gripper_lowest_bottom_at_grip_mm',{},'nominal_descent_mm',{}, ...
 'nominal_material_below_gripper_mm',{},'nominal_loaded_material_bottom_at_top_mm',{}, ...
 'nominal_gap_above_other_60mm_material_mm',{},'meets_nominal10mm_gap',{}, ...
 'declared_height_error_each_mm',{},'sensitivity_combinations',{},'worst_gap_mm',{}, ...
 'passes_all_declared_combinations',{});
for k=1:numel(grips)
 g=grips(k);loaded_bottom=top-g;nominal_gap=loaded_bottom-material;
 worst=Inf;combinations=0;
 for top_error=[-1,0,1]
  for grip_error=[-1,0,1]
   for other_height_error=[-1,0,1]
    worst=min(worst,(top+top_error)-(g+grip_error)-(material+other_height_error));
    combinations=combinations+1;
   end
  end
 end
 rows(k)=struct('gripper_lowest_bottom_at_grip_mm',g,'nominal_descent_mm',top-g, ...
  'nominal_material_below_gripper_mm',g,'nominal_loaded_material_bottom_at_top_mm',loaded_bottom, ...
  'nominal_gap_above_other_60mm_material_mm',nominal_gap,'meets_nominal10mm_gap',nominal_gap>=clearance_goal, ...
  'declared_height_error_each_mm',1,'sensitivity_combinations',combinations,'worst_gap_mm',worst, ...
  'passes_all_declared_combinations',worst>=clearance_goal);
end
checks=[rows(1).nominal_descent_mm==69,rows(1).nominal_gap_above_other_60mm_material_mm==9, ...
 ~rows(1).meets_nominal10mm_gap,rows(4).nominal_descent_mm==74, ...
 rows(4).nominal_gap_above_other_60mm_material_mm==14,rows(4).worst_gap_mm==11, ...
 rows(4).passes_all_declared_combinations];
report=struct;report.engine='local MATLAB';report.version=version;report.release=version('-release');
report.reference=ref;report.user_proposed_grip_height_mm=50;report.comparison_grip_height_mm=45;
report.comparison_selected_by_user=isfield(ref.scene_measurements,'selected_gripper_bottom_grasp_height_mm') && ...
 ref.scene_measurements.selected_gripper_bottom_grasp_height_mm==45;
report.user_selected_grip_height_mm=ref.scene_measurements.selected_gripper_bottom_grasp_height_mm;
report.user_selected_height_meets10mm_gap=false;
for k=1:numel(rows)
 if rows(k).gripper_lowest_bottom_at_grip_mm==report.user_selected_grip_height_mm
  report.user_selected_height_meets10mm_gap=rows(k).meets_nominal10mm_gap;
 end
end
report.minimum_vertical_gap_goal_mm=clearance_goal;
report.height_comparison=rows;report.check_count=numel(checks);report.failure_count=sum(~checks);
report.declared_height_errors_field_verified=false;
report.assumption='rigid secure grip; material bottom rises with gripper by same vertical displacement; no slip';
report.contact_geometry_or_grip_force_verified=false;report.lift_mm_to_encoder_calibration_available=false;
report.actual_stm32_lift_pulse_command_generated=false;report.full_motion_validation_passed=false;
report.hardware_operated=false;report.motion_allowed=false;
report.pending={'choose and field validate grip bottom height','actual contact/secure grip/slip evidence', ...
 'lift mm to feedback/pulses and current reference','carrier plane and arm/cable envelope','whole cycle and fault exits'};
write_json(fullfile(out,'result.json'),report);save(fullfile(out,'simulation.mat'),'report','ref','rows');
fprintf('MATLAB pick clearance: checks=%d failures=%d; user50mm->descent69/gap9; comparison45mm->descent74/gap14/worst11; motion=0\n', ...
 report.check_count,report.failure_count);
assert(all(checks),'Pick-clearance checks failed');
end
function value=read_json(path)
fid=fopen(path,'r','n','UTF-8');assert(fid~=-1);clean=onCleanup(@()fclose(fid)); %#ok<NASGU>
value=jsondecode(fscanf(fid,'%c'));
end
function write_json(path,value)
fid=fopen(path,'w','n','UTF-8');assert(fid~=-1);clean=onCleanup(@()fclose(fid)); %#ok<NASGU>
fprintf(fid,'%s',jsonencode(value));
end
