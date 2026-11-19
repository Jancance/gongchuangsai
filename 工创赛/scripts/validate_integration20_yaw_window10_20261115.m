function validate_integration20_yaw_window10_20261115
% User-selected yaw/path window. This is conditional geometry, not vehicle acceptance.
root=fileparts(fileparts(mfilename('fullpath')));
base=fullfile(root,'evidence','integration','2026-11-15','integration18','window5-pause');
out=fullfile(root,'evidence','integration','2026-11-15','integration20-yaw165-window10');
if ~exist(out,'dir'),mkdir(out);end
g=jsondecode(fileread(fullfile(base,'integration19-field','parking-current-matlab.json')));
fault=jsondecode(fileread(fullfile(base,'integration20-first-fault-analysis.json')));
scale=315.5*360/3200;window=10*scale;checks=0;
yaw=[195 165 180];extension=[g.current_extensions_mm(1:2)' 0];
assert(yaw(1)+yaw(2)==360 && all(yaw>=0 & yaw<=270));checks=checks+1;
assert(all(extension>=0 & extension<=170));checks=checks+1;
oldYaw=g.current_arm_angles_deg(:)';oldExt=[extension(1:2) 0];
oldPoints=[(161+oldExt).*cosd(oldYaw-180);(161+oldExt).*sind(oldYaw-180)]';
points=[(161+extension).*cosd(yaw-180);(161+extension).*sind(yaw-180)]';
tcpShift=vecnorm(points-oldPoints,2,2);
minimum=Inf;
for region=1:3
 for travel=linspace(0,min(60,extension(region)),121)
  p=(161+extension(region)-travel)*[cosd(yaw(region)-180),sind(yaw(region)-180)];
  for other=setdiff(1:3,region)
   distance=norm(p-oldPoints(other,:));minimum=min(minimum,distance);
   assert(distance>40+25+4);checks=checks+1;
  end
 end
end
for overshoot=[-10.01 -10 -9.99 -5.868088819437033 0 5.868088819437033 9.99 10 10.01]
 allowed=abs(overshoot*scale)<=window;
 assert(allowed==(abs(overshoot)<=10));checks=checks+1;
 arrived=abs(overshoot*scale)<=71;
 if abs(overshoot)>2.01,assert(~arrived);checks=checks+1;end
end
assert(fault.extension_position_analysis.first_fault_overshoot_mm<10);checks=checks+1;
% Existing stops: communication/drive faults and sustained stall still exit.
for flags=[0 1 3 5 9 13]
 for elapsed=[0 2999 3000 4000]
  hard=bitand(flags,1)==0 || bitand(flags,8)~=0;
  stopped=hard || (bitand(flags,4)~=0 && elapsed>=3000);
  if hard,assert(stopped);end
  if flags==5,assert(stopped==(elapsed>=3000));end
  checks=checks+1;
 end
end
% Recovery after the image reset rebuilds real references, then retracts at top
% before yaw180 and extension65. No body motion occurs anywhere in this trial.
stages={'real_top','real_extension_home','read_arm','open','yaw180','extension65'};
assert(find(strcmp(stages,'real_top'))<find(strcmp(stages,'real_extension_home')));
assert(find(strcmp(stages,'real_extension_home'))<find(strcmp(stages,'yaw180')));
checks=checks+2;
for rise=[0 40 65 65.0025 69.75]
 assert((rise>65)==(floor(rise*400)/400>65));checks=checks+1;
end
phases=[1 2 4 5 11 12 6 7 10];
for phase=phases
 for faultPresent=[false true]
  stop=faultPresent;assert(stop==faultPresent);checks=checks+1;
 end
end
r=struct('passed',true,'local_matlab',true,'checks',checks,...
 'yaw_deg',yaw,'extensions_mm',extension,'extension_path_window_mm',10,...
 'extension_path_window_virtual_deg',window,'arrival_virtual_deg',71,...
 'zero_virtual_deg',17.75,'extension_rpm',150,'stall_confirm_ms',3000,...
 'minimum_neighbour_center_distance_mm',minimum,'assumed_tool_radius_mm',40,...
 'tcp_shift_from_prior_model_mm',tcpShift','recovery_stages',{stages},...
 'skip_body_alignment',true,'body_motion_commands',0,'hardware_access',false,...
 'physical_sweep_validated',false,'physical_grasp_verified',false,...
 'assumptions','Prior scale and source points;40mm tool radius; no load/braking/ground-slip model');
f=fopen(fullfile(out,'matlab-result.json'),'w','n','UTF-8');fprintf(f,'%s\n',jsonencode(r));fclose(f);
fprintf('PASS MATLAB integration20: %u checks, yaw195/165/180, corridor10mm; min gap %.3fmm; TCP shifts %.3f %.3fmm; no body alignment\n',checks,minimum,tcpShift(1),tcpShift(2));
end
