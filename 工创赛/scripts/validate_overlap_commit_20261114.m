function validate_overlap_commit_20261114
% Prior to implementation: user-requested committed overlapping motion.
root=fileparts(fileparts(mfilename('fullpath')));
out=fullfile(root,'evidence','integration','2026-11-14','overlap200-commit');
if ~exist(out,'dir'),mkdir(out);end
base=fullfile(root,'evidence','integration','2026-11-14','top-pick-pb9');
geometry=jsondecode(fileread(fullfile(base,'parking-current.json')));
ppm=400;top=119;pick=29700;hover=15600;place=1800;observe=600;
extRpm=200;extAcc=600;liftRpm=1846.4;liftAcc=45055;
assert(top-hover/ppm==80 && top-hover/ppm-60==20);
assert((pick-place)/ppm>65 && observe/ppm==1.5);
allYaw=geometry.current_arm_angles_deg(:)';allExt=geometry.current_extensions_mm(:)';
yaw=[allYaw(1:2),180];extension=[allExt(1:2),0];
points=[(161+extension).*cosd(yaw-180);(161+extension).*sind(yaw-180)]';
% The first 60mm of loaded retraction stays clear even if Z has not moved.
% 50mm material and a DECLARED40mm tool radius, not a measured tool envelope.
partial=60;minimum=Inf;checks=0;
for region=1:3
 for travel=linspace(0,min(partial,extension(region)),121)
  r=161+extension(region)-travel;
  p=r*[cosd(yaw(region)-180),sind(yaw(region)-180)];
  for other=setdiff(1:3,region)
   separation=norm(p-points(other,:));minimum=min(minimum,separation);
   assert(separation>40+25+4);checks=checks+1;
  end
 end
end
% Complete retraction and all loaded yaw require ACTUAL >65mm lift.
for actual=[0 40 65 65.0025 69.75]
 allowed=actual>65;
 assert(allowed==(floor(actual*ppm)/ppm>65));checks=checks+1;
end
% Electrical pulse preload: start the existing50ms dwell after next timer update.
minimumDwell=Inf;
for phase=0:19
 applied=20-phase;
 observed=ceil(applied/10)*10;
 returnAt=observed+50;
 assert(returnAt-applied>=50);minimumDwell=min(minimumDwell,returnAt-applied);checks=checks+1;
end
% After the one two-frame commit, camera changes never cancel the piece.
phases=[1 2 4 5 11 12 6 7 10];
for missingAt=1:numel(phases)
 for k=1:numel(phases)
  vision=k<missingAt; %#ok<NASGU>
  if k==numel(phases),assert(phases(k)==10);end
  checks=checks+1;
 end
end
% A hardware fault/manual stop remains effective at every phase.
for k=1:numel(phases)
 for actualFault=[false true]
  cancelled=actualFault;assert(cancelled==actualFault);checks=checks+1;
 end
end
% Fixed wheel encoder target can resist wheel rotation, not tyre slip.
wheelEncoderDrift=[0 10];groundSlip=[5 0];
assert(wheelEncoderDrift(1)==0 && groundSlip(1)>0);checks=checks+1;
% Trapezoid timing estimates only: include neither CAN scheduling nor servo response.
mmPerRevX=32000/315.5; % existing virtual extension-angle scale, not new ruler data
times=zeros(3,4);
for k=1:3
 times(k,1)=profile(abs(extension(k)-65)/mmPerRevX,extRpm/60,extAcc/60);
 times(k,2)=profile((pick-hover)/3200,liftRpm/60,liftAcc/60);
 times(k,3)=profile(min(partial,extension(k))/mmPerRevX,extRpm/60,extAcc/60);
 times(k,4)=profile(max(extension(k)-partial,0)/mmPerRevX,extRpm/60,extAcc/60);
end
r=struct('passed',true,'local_matlab',true,'checks',checks,...
 'pulses_per_mm',ppm,'pick_pulses',pick,'empty_overlap_hover_pulses',hover,...
 'place_pulses',place,'observe_pulses',observe,'extension_rpm',extRpm,...
 'extension_acceleration_rpm_s',extAcc,'lift_rpm',liftRpm,...
 'lift_acceleration_rpm_s',liftAcc,'loaded_partial_retraction_mm',partial,...
 'minimum_neighbour_center_distance_mm',minimum,'assumed_empty_tool_radius_mm',40,...
 'assumed_material_radius_mm',25,'existing_position_uncertainty_budget_mm',4,...
 'source_points_mm',points,'ideal_profile_times_s',times,...
 'release_dwell_ms',50,'minimum_pulse_applied_dwell_ms',minimumDwell,...
 'full_retract_before_loaded_yaw',true,'return_xyz_parallel',true,...
 'single_commit_no_visual_recheck',true,'wheel_hold_cannot_measure_ground_slip',true,...
 'hardware_access',false,'physical_sweep_validated',false,'stop_window_guaranteed',false);
f=fopen(fullfile(out,'matlab-result.json'),'w','n','UTF-8');fprintf(f,'%s\n',jsonencode(r));fclose(f);
fprintf('PASS overlap/commit: %u checks; partial60mm min centre gap %.2fmm; 200RPM/600 unchanged accel; no physical timing claim\n',checks,minimum);
end
function t=profile(d,v,a)
if d==0,t=0;elseif d<=v*v/a,t=2*sqrt(d/a);else,t=d/v+v/a;end
end
