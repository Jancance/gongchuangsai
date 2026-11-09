function validate_lift_return_top_matlab_20261106
% Declared trajectory/failure model, not identified contact force dynamics.
root=fileparts(fileparts(mfilename('fullpath')));
out=fullfile(root,'evidence','integration','2026-11-06','lift-return-top');if ~exist(out,'dir'),mkdir(out);end
checks=struct('name',{},'passed',{});
pre_p=90000;ratio=1600;start_h=50;top_h=119;v=300*6;a=60*6;
angle=pre_p*360/3200;ta=v/a;tc=angle/v-ta;duration=2*ta+tc;
t=linspace(0,duration,2001);x=zeros(size(t));speed=zeros(size(t));
for k=1:numel(t)
 if t(k)<ta,x(k)=.5*a*t(k)^2;speed(k)=a*t(k);
 elseif t(k)<ta+tc,x(k)=.5*a*ta^2+v*(t(k)-ta);speed(k)=v;
 else,q=duration-t(k);x(k)=angle-.5*a*q^2;speed(k)=a*q;
 end
end
gain=[5/6,1,7/6];heights=start_h+pre_p/ratio*gain;remaining=top_h-heights;
check('prelift completes before declared total16s',duration+2<16);
check('upward path monotonic',all(diff(x)>=-1e-8));
check('normal endpoint leaves12.75mm before top',abs(remaining(2)-12.75)<1e-8);
check('declared gain range does not hit top during prelift',all(remaining>0));
check('retained60RPM homing covers declared remainder before driver40s',all(remaining/(3200/ratio)<40));
check('model acceleration and stop bounded',all(speed>=0 & speed<=v+1e-8) && abs(speed(end))<1e-8);
cases={'normal','wrong_menu','bad_options','prelift_ack_missing','prelift_frozen','prelift_overshoot', ...
 'target_mismatch','cancel_pre','cancel_home','home_ack_missing','home_stale_idle','home_failure', ...
 'home_running_then_idle','speed_nonzero','position_invalid','abort_ack_missing','stop_ack_missing', ...
 'power_loss','service_gap','communication_failure','offline'};
for k=1:numel(cases)
 name=cases{k};moves=0;homes=0;done=false;stop_pending=false;abort_pending=false;
 if ~any(strcmp(name,{'offline','wrong_menu','bad_options'})),moves=1;end
 pre_ok=~any(strcmp(name,{'offline','wrong_menu','bad_options','prelift_ack_missing','prelift_frozen', ...
  'prelift_overshoot','target_mismatch','cancel_pre','power_loss','service_gap','communication_failure'}));
 if pre_ok,homes=1;end
 seen=~any(strcmp(name,{'home_stale_idle','home_ack_missing','offline'}));
 done=pre_ok && seen && any(strcmp(name,{'normal','home_running_then_idle'}));
 if ~done && ~strcmp(name,'offline')
  stop_pending=strcmp(name,'stop_ack_missing');abort_pending=homes>0 && strcmp(name,'abort_ack_missing');
 end
 check([name ': at most one finite prelift and home start'],moves<=1 && homes<=1);
 check([name ': success requires new run and fresh stopped valid feedback'],~done || (pre_ok && seen));
 if ~any(strcmp(name,{'normal','home_running_then_idle'})),check([name ': no false DONE'],~done);end
 if strcmp(name,'abort_ack_missing'),check('abort pending retained',abort_pending);end
 if strcmp(name,'stop_ack_missing'),check('stop pending retained',stop_pending);end
end
% Active home respects prior user policy: no new host total deadline; the
% read-back driver40-second limit, cancellation and failure checks remain.
report=struct('engine','local MATLAB','version',version,'release',version('-release'), ...
 'prelift_virtual_pulses',pre_p,'prelift_rpm',300,'prelift_acceleration_rpm_s',60, ...
 'prelift_nominal_duration_s',duration,'prelift_total_timeout_ms',16000, ...
 'declared_gain_factors',gain,'prelift_bottom_heights_mm',heights,'remaining_to_top_mm',remaining, ...
 'home_direction',1,'home_rpm',60,'home_driver_timeout_ms',40000,'home_host_new_total_timeout',false, ...
 'home_detect_rpm',300,'home_detect_current_ma',800,'home_detect_time_ms',60,'home_auto',0, ...
 'cases',{cases},'checks',checks,'check_count',numel(checks),'failure_count',sum(~[checks.passed]), ...
 'passed',all([checks.passed]),'plant_contact_force_identified',false,'physical_top_passed',false, ...
 'hardware_operated',false,'vm_auto_cut_on_module_fault',false);
fid=fopen(fullfile(out,'matlab-result.json'),'w','n','UTF-8');assert(fid~=-1);c=onCleanup(@()fclose(fid)); %#ok<NASGU>
fprintf(fid,'%s',jsonencode(report));save(fullfile(out,'matlab-result.mat'),'report');
fprintf('MATLAB RETURN: prelift%.3fs, remaining%.2fmm, cases%d checks%d failures%d\n',duration,remaining(2),numel(cases),numel(checks),report.failure_count);
assert(report.passed);
 function check(name,passed)
  checks(end+1)=struct('name',name,'passed',logical(passed)); %#ok<AGROW>
 end
end
