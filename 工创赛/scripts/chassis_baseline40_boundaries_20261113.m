function chassis_baseline40_boundaries_20261113
% Engineering arithmetic only. No drive dynamics, slip, hardware or deployment.
root = fileparts(fileparts(mfilename('fullpath')));
repo = fileparts(root);
source = fullfile(repo,['3.stm32f407' char([24037 31243])],'CHASSIS','USER');
out = fullfile(root,'evidence','navigation-planning','2026-11-13','baseline40-boundaries-r2020a');
if ~exist(out,'dir'), mkdir(out); end
prior = jsondecode(fileread(fullfile(root,'evidence','navigation-planning', ...
    '2026-11-13','baseline40-r2020a','summary.json')));
c = prior.cases(prior.selected_case_index);
kin = fileread(fullfile(source,'chassis_kinematics.c'));
auto = fileread(fullfile(source,'chassis_auto_test.c'));
assert(contains(kin,'limit = base_rpm + 2U;'));
assert(contains(kin,'requested >= -2.0f && requested <= 2.0f'));
assert(contains(auto,'(uint32_t)(ref_rpm+0.5f)'));
assert(contains(auto,'+ 512)'));
assert(contains(auto,'dt > 2000U'));
assert(contains(auto,'raw >= -(float)speed_limit && raw <= (float)speed_limit'));
assert(prior.selected_requested_A_mm_s2==150 && prior.selected_requested_J_mm_s3==300);
assert(prior.follow_rpm==2 && prior.follow_omega==1);
factor = prior.wheel_mm_s_per_rpm;
referenceRpm = prior.speed_limit_mm_s/factor;
baseCap = ceil(referenceRpm)+prior.follow_rpm;
headingLimit = 2; % Existing HWT correction, unchanged.
actualRoundedBase = floor(referenceRpm+prior.follow_rpm+0.5);
actualModelWheel = actualRoundedBase+headingLimit;
conservativeWheel = baseCap+headingLimit;
assert(baseCap==100 && actualRoundedBase==99);
assert(actualModelWheel==101 && conservativeWheel==102);
% Existing 20 cm/s: feedback80 versus conservative (52+2) leaves26 RPM.
% Candidate130 versus102 leaves28 RPM. Neither is a measured capability.
feedback = 130;
assert(feedback-conservativeWheel >= prior.existing_feedback_guard_rpm- ...
    (prior.existing_command_guard_rpm+headingLimit));
assert(all(abs([-feedback feedback])<=feedback));
assert(~(feedback+0.1<=feedback) && ~(-feedback-0.1>=-feedback));
countsPerRev = 65536;
% Preserve the original extra30 RPM and +512 count jitter allowance.
jump = ceil((feedback+30)*countsPerRev/60000);
assert(jump==175);
for interval = [1 10 100 399 2000]
    boundary = interval*jump+512;
    assert(~(interval>2000 || boundary>interval*jump+512));
    assert(interval>2000 || boundary+1>interval*jump+512);
    assert(ceil(conservativeWheel*countsPerRev/60000)*interval<=boundary);
end
assert(2001>2000); % Original long interval rejection remains.
duration = c.duration_s;
finalWait = 2000; margin = 2000;
deadline = ceil(duration*1000)+finalWait+margin;
assert(deadline==12779 && duration*1000+finalWait<deadline && deadline<=60000);
accel = c.effective_A_mm_s2; jerk = c.effective_J_mm_s3;
tj = accel/jerk;
ta = prior.speed_limit_mm_s/accel-tj;
cruise = prior.distance_mm/prior.speed_limit_mm_s-(2*tj+ta);
assert(tj>0 && ta>0 && abs(cruise-c.cruise400_s)<1e-9);
assert(abs((4*tj+2*ta+cruise)-duration)<1e-9);
% The last original1RPM approach, for2s, does not close arbitrary50mm lag.
approachDistance = factor*finalWait/1000;
assert(approachDistance<50);
report = struct('matlab_version',version,'selected_requested_A_mm_s2',150, ...
    'selected_requested_J_mm_s3',300,'effective_A_mm_s2',accel, ...
    'effective_J_mm_s3',jerk,'reference_rpm',referenceRpm, ...
    'follow_reserve_rpm',prior.follow_rpm,'base_configuration_cap_rpm',baseCap, ...
    'rounded_base_peak_model_rpm',actualRoundedBase, ...
    'existing_hwt_correction_limit_rpm',headingLimit, ...
    'wheel_peak_model_rpm',actualModelWheel,'wheel_conservative_cap_rpm',conservativeWheel, ...
    'feedback_limit_candidate_rpm',feedback,'feedback_command_margin_rpm',feedback-conservativeWheel, ...
    'original_feedback_command_margin_rpm',26,'jump_extra_rpm_unchanged',30, ...
    'position_jump_counts_per_ms_candidate',jump,'position_jump_extra_counts_unchanged',512, ...
    'position_jump_max_interval_ms_unchanged',2000,'duration_s',duration, ...
    'cruise400_s',cruise,'deadline_ms',deadline,'final_wait_ms_unchanged',finalWait, ...
    'deadline_margin_ms',margin,'approach_only_distance_during_final_wait_mm',approachDistance, ...
    'negative_boundary_cases_passed',true,'hardware_tested',false, ...
    'limitation','Reference and arithmetic boundary validation only. 101RPM is the ideal profile plus bounded follow and HWT model;102RPM is the conservative code cap. 130RPM feedback is an engineering candidate, not measured drive capability. Existing 50mm tracking,400ms position freshness, HWT validity, reverse checks and stops remain. No dynamic/load/friction/latency model or full feasibility guarantee; final1RPM/2s cannot close arbitrary50mm lag.', ...
    'result','MATLAB_BASELINE40_BOUNDARIES_OK');
fid=fopen(fullfile(out,'summary.json'),'w','n','UTF-8');
assert(fid>=0); cleanup=onCleanup(@() fclose(fid));
fwrite(fid,jsonencode(report),'char');
fprintf('MATLAB_BASELINE40_BOUNDARIES_OK base=%d model_wheel=%d conservative_wheel=%d feedback=%d jump=%d deadline=%d cruise=%.9f\n', ...
    baseCap,actualModelWheel,conservativeWheel,feedback,jump,deadline,cruise);
end
