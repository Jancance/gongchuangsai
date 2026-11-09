function validate_source_finalstop_A_matlab_20261106
% Validate saved current-pose observations, not actuator implementation.
root=fileparts(fileparts(mfilename('fullpath')));
out=fullfile(root,'evidence','integration','2026-11-06','source-vision-new-pose','finalstop-A-review');
d=jsondecode(fileread(fullfile(out,'measurement-input.json')));s=d.summary;
cal=jsondecode(fileread(fullfile(root,d.current_calibration_path)));H=cal.homography_image_to_plane;
prior=jsondecode(fileread(fullfile(root,'evidence','integration','2026-11-06','source-vision-new-pose','finalstop-geometry','matlab-result.json')));
checks=struct('name',{},'passed',{});
check('current board capture complete with serial disabled',s.frames==s.session_frames && s.frames==587 && isempty(s.error) && strcmp(s.serial,'DISABLED') && s.all_frames_motion_disabled);
check('detected all and stable after initial four frame window',s.detected_frames==s.frames && s.stable_frames==s.frames-4 && isequal(s.unstable_frame_sequences(:)',1:4));
check('contiguous sequence and unchanged saved config',s.frame_sequence_contiguous && s.camera_config_hashes_unchanged);
p=d.paper_to_plain_patches;accepted=[p.correlation]>=.9;loc=vertcat(p(accepted).paper_px);shift=vertcat(p(accepted).integer_shift_px);
if size(loc,2)~=2,loc=reshape(loc,2,[])';shift=reshape(shift,2,[])';end
same_view=sum(accepted)>=6 && rank(loc-mean(loc,1))==2 && max(vecnorm(shift,2,2))<=1;
check('independent fixed disk features preserve same view',same_view);
frames=d.frame_boundary_points_px;
if iscell(frames),n=numel(frames);else,n=size(frames,1);end
centers=zeros(n,2);radii=zeros(n,1);rms=zeros(n,1);counts=zeros(n,1);
for k=1:n
 if iscell(frames),q=frames{k};else,q=squeeze(frames(k,:,:));end
 q=project(H,q);keep=true(size(q,1),1);
 for it=1:5
  [c,r,~]=circle(q(keep,:));res=vecnorm(q-c,2,2)-r;med=median(res(keep));mad=median(abs(res(keep)-med));keep=abs(res-med)<=max(1.5,3*1.4826*mad);
 end
 [c,r,res]=circle(q(keep,:));centers(k,:)=c;radii(k)=r;rms(k)=sqrt(mean(res.^2));counts(k)=sum(keep);
end
delta=vecnorm(centers-prior.rim.center_plane_mm(:)',2,2);
check('all frame physical-plane rim fits plausible diagnostic',all(radii>130 & radii<170 & rms<3 & counts>80));
%3mm is the existing diagnostic comparison only, not an accuracy guarantee.
check('new rim agrees with same-view paper diagnostic within3mm',max(delta)<3);
check('official planning obstacle not shrunk to fit',prior.rim.planning_obstacle_radius_mm>=150);
report=struct('engine','local MATLAB','release',version('-release'),'calibration_id',cal.calibration_id, ...
 'frames',n,'same_view_fixed_features_passed',same_view,'fixed_patch_count',sum(accepted), ...
 'fixed_patch_max_integer_shift_px',max(vecnorm(shift,2,2)), ...
 'mapped_rim_center_median_plane_mm',median(centers,1),'mapped_rim_center_peak_to_peak_mm',max(centers)-min(centers), ...
 'mapped_rim_radius_min_median_max_mm',[min(radii),median(radii),max(radii)], ...
 'mapped_rim_fit_rms_min_median_max_mm',[min(rms),median(rms),max(rms)], ...
 'maximum_center_difference_from_paper_rim_mm',max(delta),'planning_obstacle_radius_mm',150, ...
 'checks',checks,'check_count',numel(checks),'failure_count',sum(~[checks.passed]),'passed',all([checks.passed]), ...
 'pixel_stability_accepted',true,'same_view_mapping_diagnostic_only',true,'sensor_exposure_age_verified',false, ...
 'full_plane_accuracy_accepted',false,'material_top_mapping_verified',false,'physical_reach_accepted',false,'motion_allowed',false);
fid=fopen(fullfile(out,'matlab-result.json'),'w','n','UTF-8');assert(fid~=-1);fprintf(fid,'%s',jsonencode(report));fclose(fid);
save(fullfile(out,'matlab-result.mat'),'report','centers','radii','rms','counts');
fprintf('FINALSTOP A MATLAB checks%d failures%d frames%d fixed%d; rim radius median%.3fmm, RMSmedian%.3fmm, paper center difference max%.3fmm\n', ...
 report.check_count,report.failure_count,n,sum(accepted),median(radii),median(rms),max(delta));
assert(report.passed);
 function check(name,passed)
  checks(end+1)=struct('name',name,'passed',logical(passed));
 end
end
function p=project(H,p)
p=[p,ones(size(p,1),1)]*H';assert(all(abs(p(:,3))>1e-10));p=p(:,1:2)./p(:,3);
end
function [c,r,res]=circle(p)
o=mean(p,1);q=p-o;z=[2*q,ones(size(q,1),1)]\sum(q.^2,2);c=z(1:2)'+o;r=sqrt(z(3)+sum(z(1:2).^2));res=vecnorm(p-c,2,2)-r;
end
