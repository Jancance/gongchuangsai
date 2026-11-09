function validate_gripper210_current_matlab_20261105
% 单次145->210 PWM命令验证；仅允许TIM2_CCR1，非实物角度/电流/干涉验收。
root=fileparts(fileparts(mfilename('fullpath')));
out=fullfile(root,'evidence','integration','2026-11-05','gripper210-current-session');
baseline=read_json(fullfile(out,'readonly-sync-report.json'));
assert(baseline.flash_matches && baseline.flash_bytes_compared==148728);
s=baseline.snapshot;
assert(strcmp(s.core_state,'RUNNING') && s.g_vehicle_state==0 && s.g_vehicle_run_id==0 && s.g_vehicle_stage==1);
assert(s.TIM2_PSC==83 && s.TIM2_ARR==19999 && s.TIM2_CCR1==1574);
assert(bitand(s.GPIOA_MODER,3)==2 && bitand(s.GPIOA_AFRL,15)==1);
start_pulse=s.TIM2_CCR1;goal=round(500+210*2000/270);
n=ceil(1.875*abs(goal-start_pulse)/11);u=(1:n)'/n;
smooth=u.^3.*(10-15*u+6*u.^2);
pulses=round(start_pulse+(goal-start_pulse)*smooth);
deltas=diff([start_pulse;pulses]);
checks=[goal==2056,all(pulses>=start_pulse),all(pulses<=2056),all(deltas>=0), ...
 all(deltas<=12),pulses(end)==2056,all(pulses<20000),abs((goal-500)*270/2000-210)<.1];
report=struct('engine','local MATLAB','version',version,'release',version('-release'), ...
 'start_pulse_us',start_pulse,'requested_angle_deg',210,'goal_pulse_us',goal,'pulses_us',pulses', ...
 'step_count',n,'minimum_interval_s',.03,'maximum_step_us',max(deltas), ...
 'sole_target_write_address_hex','0x40000034','check_count',numel(checks),'failure_count',sum(~checks), ...
 'actual_failure_injection',false, ...
 'allowed_signal_plan',all(checks),'physical_angle_verified',false,'physical_sweep_verified',false, ...
 'live_execution_done',false,'baseline_image_sha256',baseline.image_sha256);
write_json(fullfile(out,'matlab-signal-plan.json'),report);
save(fullfile(out,'matlab-signal-plan.mat'),'report','pulses','baseline');
fprintf('MATLAB GRIP210 checks=%d failures=%d steps=%d maxstep=%dus final=%dus\n', ...
 report.check_count,report.failure_count,n,max(deltas),goal);
assert(report.allowed_signal_plan,'Signal plan failed; no execution code or hardware writes');
end
function value=read_json(path)
fid=fopen(path,'r','n','UTF-8');assert(fid~=-1);c=onCleanup(@()fclose(fid)); %#ok<NASGU>
value=jsondecode(fscanf(fid,'%c'));
end
function write_json(path,value)
fid=fopen(path,'w','n','UTF-8');assert(fid~=-1);c=onCleanup(@()fclose(fid)); %#ok<NASGU>
fprintf(fid,'%s',jsonencode(value));
end
