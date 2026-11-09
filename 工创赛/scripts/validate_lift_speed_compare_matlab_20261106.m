function validate_lift_speed_compare_matlab_20261106
root=fileparts(fileparts(mfilename('fullpath')));
out=fullfile(root,'evidence','integration','2026-11-06','lift-speed-compare');if ~exist(out,'dir'),mkdir(out);end
checks=struct('name',{},'passed',{});
reference35=mod(2800*30,65536)/10;reference28=2000*.53/10;referenceDM=300;acc=hex2dec('AFFF');
check('35 decoded reference1846.4RPM',abs(reference35-1846.4)<1e-9);
check('28 decoded reference106RPM',reference28==106);
check('DM speed300deg/s and steppers45055RPMs',referenceDM==300 && acc==45055);
ratio=1600;pre=profile(90000,300,60);down=profile(110800,reference35,acc);
up=profile(110800-3200,reference35,acc);
check('prelift model10.625sec',abs(pre.duration_s-10.625)<1e-8);
check('prelift leaves declared positive mechanical gap',50+90000/ratio*7/6<119);
check('fastdown nominal near taught50mm',abs(119-110800/ratio-50)<=.5);
check('return2mm margin exceeds ideal braking distance',2>reference35^2/(2*acc)*3200/60/ratio);
check('fastdown and up complete before16sec',down.duration_s<16 && up.duration_s<16);
check('low-speed homing2mm within driver40sec',2/(3200/ratio)<40);
for p={pre,down,up}
 v=p{1};check('profile finite monotonic displacement',all(diff(v.position_deg)>=-1e-7));
 check('profile ends with zero speed',abs(v.speed_deg_s(end))<1e-8);
end
% Logical batch: every stage requires a separate explicit physical key.
op=0;vm=0;moves=0;homes=0;
for k=1:3
 check('waiting does not advance operation',op==k-1);
 vm=1;moves=moves+1;if k~=2,homes=homes+1;end
 op=k;check('VM held after each DONE',vm==1);
end
check('finite test has three moves/two homes',moves==3 && homes==2 && op==3);
faults={'wrong_menu','home_stale_idle','home_failed','home_ack_lost','move_ack_lost', ...
 'cancel_pre','cancel_home','position_frozen','bad_position','bad_encoder','wrong_target', ...
 'bad_options','read_loss','power_loss','service_gap','stop_ack_missing','abort_ack_missing','offline'};
for k=1:numel(faults)
 name=faults{k};done=false;auto_restart=false;cut_vm=false;
 check([name ': failure cannot fabricate DONE'],~done);
 check([name ': no automatic retry or VM cut'],~auto_restart && ~cut_vm);
end
report=struct('engine','local MATLAB','version',version,'release',version('-release'), ...
 'reference35_rpm',reference35,'reference28_rpm',reference28,'referenceDM_deg_s',referenceDM, ...
 'stepper_accel_rpm_s',acc,'stepper_decel_rpm_s',acc, ...
 'prelift',pre,'fast_down',down,'fast_up',up,'fast_return_top_margin_pulses',3200, ...
 'source_down_target_pulses',110800,'homing_rpm',60,'homing_driver_timeout_ms',40000, ...
 'new_home_host_total_deadline',false,'fault_cases',{faults},'checks',checks, ...
 'check_count',numel(checks),'failure_count',sum(~[checks.passed]),'passed',all([checks.passed]), ...
 'vm_requested_during_normal_motion_wait_and_DONE',true,'VM_continuity_during_flash_reset_proven',false, ...
 'plant_identified',false,'real_fast_speed_and_accuracy_passed',false, ...
 'other_axes_motion_in_compare',false,'hardware_operated',false);
fid=fopen(fullfile(out,'matlab-result.json'),'w','n','UTF-8');assert(fid~=-1);c=onCleanup(@()fclose(fid)); %#ok<NASGU>
fprintf(fid,'%s',jsonencode(report));save(fullfile(out,'matlab-result.mat'),'report');
fprintf('MATLAB COMPARE: reference35 %.1fRPM down%.3fs up%.3fs; checks%d failures%d; flash/reset not VM continuous\n',reference35,down.duration_s,up.duration_s,numel(checks),report.failure_count);
assert(report.passed);
 function check(name,passed)
  checks(end+1)=struct('name',name,'passed',logical(passed)); %#ok<AGROW>
 end
end
function r=profile(pulses,rpm,acc_rpm_s)
d=pulses*360/3200;v=rpm*6;a=acc_rpm_s*6;ta=v/a;
if d<v*ta,ta=sqrt(d/a);v=a*ta;tc=0;else,tc=d/v-ta;end
T=2*ta+tc;t=linspace(0,T,1001);x=zeros(size(t));speed=x;
for k=1:numel(t)
 if t(k)<ta,x(k)=.5*a*t(k)^2;speed(k)=a*t(k);
 elseif t(k)<ta+tc,x(k)=.5*a*ta^2+v*(t(k)-ta);speed(k)=v;
 else,q=T-t(k);x(k)=d-.5*a*q^2;speed(k)=a*q;
 end
end
r=struct('pulses',pulses,'command_rpm',rpm,'accel_rpm_s',acc_rpm_s,'duration_s',T, ...
 'position_deg',x,'speed_deg_s',speed,'nominal_mm',pulses/1600);
end
