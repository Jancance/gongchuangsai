function validate_source_clean_20261111
% 静态空盘诊断。沿用既有TCP中心/161mm候选作条件性可达检查，不当实测。
root=fileparts(fileparts(mfilename('fullpath')));
base=fullfile(root,'evidence','integration','2026-11-11','source-view-entry');
out=fullfile(base,'clean-disk-review');d=jsondecode(fileread(fullfile(out,'measurement-input.json')));
m=jsondecode(fileread(fullfile(base,'paper50-geometry','model-comparison.json')));
v=jsondecode(fileread(fullfile(base,'ruler-review','matlab-result.json')));assert(v.passed);
old=jsondecode(fileread(fullfile(root,'calibration','source_pose_current_20261106.json')));
H=m.models(1).H_image_to_paper_plane;assert(strcmp(m.models(1).model,'similarity'));
checks=struct('name',{},'passed',{});
check('15 real frames all detected;11 stable after warmup',d.summary.frames==15 && d.summary.detected_frames==15 && d.summary.stable_frames==11);
check('observation only',strcmp(d.summary.serial,'DISABLED') && ~d.motion_allowed);
check('fixed surface retains measured1px right shift',all(arrayfun(@(p)p.correlation>.9 && isequal(p.integer_shift_px(:)',[1 0]),d.fixed_surface_patches)));
circles=struct([]);
for k=1:3
 pts=project(H,d.circles(k).boundary_image_px);[c,r,e,keep]=robust_circle(pts);
 a=sort(mod(atan2d(pts(keep,2)-c(2),pts(keep,1)-c(1)),360));coverage=360-max(diff([a;a(1)+360]));
 circles(k).id=d.circles(k).id;circles(k).center_plane_mm=c;
 circles(k).center_image_px=project(inv(H),c);circles(k).radius_mm=r;
 circles(k).fit_rms_mm=sqrt(mean(e.^2));circles(k).coverage_deg=coverage;circles(k).retained_points=sum(keep);
 check(['drawn circle boundary ' circles(k).id],r>20 && r<30 && circles(k).fit_rms_mm<1 && coverage>300 && sum(keep)>280);
end
plane=vertcat(circles.center_plane_mm);pairs=[1 2;2 3;3 1];spacing=zeros(3,1);
for k=1:3,spacing(k)=norm(plane(pairs(k,1),:)-plane(pairs(k,2),:));end
[orbit,orbitRadius,~]=plain_circle(plane);check('nominal layout only qualitatively plausible',all(spacing>160 & spacing<185) && orbitRadius>90 && orbitRadius<110);
rimCenters=zeros(15,2);rimRadius=zeros(15,1);rimRms=zeros(15,1);
for k=1:15
 [rimCenters(k,:),rimRadius(k),e,~]=robust_circle(project(H,d.frame_boundary_points_px{k}));rimRms(k)=sqrt(mean(e.^2));
end
% Paper frame +X is AB, +Y is AD. Source-view frame is separate from chassis:
% +X points along the declared outward/image-up arm direction; +Y image-left.
tcp=project(H,[960 540]);f=project(H,[960 539])-tcp;f=f/norm(f);
right=project(H,[961 540])-tcp;right=right/norm(right);
r0=old.retained_mechanics.retracted_pivot_to_TCP_candidate_mm;ext=65;maximum=old.retained_mechanics.maximum_extension_mm;
T=[f (r0+ext)-dot(tcp,f);-right dot(tcp,right);0 0 1];
source=project(T,plane);lengths=vecnorm(source,2,2)-r0;angles=mod(180+atan2d(source(:,2),source(:,1)),360);
check('explicit retained model values; current65mm not old48mm',r0==161 && maximum==170 && ext==65);
% Candidate pivot shifts in this SOURCE-VIEW frame only, NOT wheel commands.
[xx,yy]=meshgrid(-20:.25:20,-20:.25:20);shift=[xx(:) yy(:)];stroke=zeros(size(shift,1),3);
for k=1:3,stroke(:,k)=vecnorm(source(k,:)-shift,2,2)-r0;end
margin=min([stroke maximum-stroke],[],2);valid=margin>=2;
best=[];
if any(valid)
 ids=find(valid);[~,rank]=sortrows([vecnorm(shift(valid,:),2,2) -margin(valid)],[1 2]);idx=ids(rank(1));
 best=struct('pivot_shift_source_view_frame_mm',shift(idx,:),'extensions_candidate_mm',stroke(idx,:), ...
  'minimum_stroke_margin_mm',margin(idx),'optimization','minimum shift on0.25mm grid with2mm declared nominal stroke margin', ...
  'chassis_axis_mapping_verified',false,'swept_path_verified',false,'is_motion_command',false);
end
report=struct('engine','local MATLAB','release',version('-release'),'image_sha256',d.image_sha256, ...
 'model','similarity derived from user50mm paper and checked by independent ruler strip', ...
 'homography_image_to_old_paper_plane',H,'circles',circles,'target_spacing_mm',spacing', ...
 'target_orbit_center_plane_mm',orbit,'target_orbit_radius_mm',orbitRadius, ...
 'rim_center_plane_median_mm',median(rimCenters,1),'rim_radius_median_mm',median(rimRadius), ...
 'rim_fit_rms_median_mm',median(rimRms),'rim_center_peak_to_peak_mm',max(rimCenters,[],1)-min(rimCenters,[],1), ...
 'source_frame_definition','+X declared outward/image-up at arm180; +Y image-left; pivot origin; NOT verified chassis axes', ...
 'source_frame_T_candidate',T,'retained_retracted_radius_candidate_mm',r0,'current_extension_command_mm',65, ...
 'TCP_center_retained_input_not_new_physical_check',true, ...
 'target_source_frame_candidate_mm',source,'extension_candidate_mm',lengths','arm_angle_candidate_deg',angles', ...
 'all_three_current_strokes_within_0to170',all(lengths>=0 & lengths<=maximum), ...
 'minimal_nominal_shift_candidate',best,'user_approximate_current_gap_mm',75, ...
 'current_gap_model_computed',false,'new_goal_parking_reference_verified',false, ...
 'check_count',numel(checks),'failure_count',sum(~[checks.passed]),'checks',checks,'passed',all([checks.passed]), ...
 'pose_body_axis_binding_verified',false,'full_swept_path_verified',false,'material_instance_verified',false, ...
 'sensor_exposure_age_verified',false,'motion_allowed',false);
fid=fopen(fullfile(out,'matlab-result.json'),'w','n','UTF-8');assert(fid~=-1);fprintf(fid,'%s\n',jsonencode(report));fclose(fid);
fprintf('CLEAN MATLAB%d checks/%d failures; current conditional extensions[%.3f %.3f %.3f]mm angles[%.3f %.3f %.3f]deg\n', ...
 report.check_count,report.failure_count,lengths,angles);
fprintf('Target spacing[%.3f %.3f %.3f]mm; orbit%.3fmm; no body command or verified TCP/axis binding.\n',spacing,orbitRadius);
assert(report.passed);
 function check(name,passed)
  checks(end+1)=struct('name',name,'passed',logical(passed));
 end
end
function p=project(H,p)
p=[p ones(size(p,1),1)]*H';p=p(:,1:2)./p(:,3);
end
function [c,r,e,keep]=robust_circle(p)
keep=true(size(p,1),1);
for k=1:5
 [c,r,~]=plain_circle(p(keep,:));e=vecnorm(p-c,2,2)-r;m=median(e(keep));mad=median(abs(e(keep)-m));keep=abs(e-m)<=max(.5,3*1.4826*mad);
end
[c,r,e]=plain_circle(p(keep,:));
end
function [c,r,e]=plain_circle(p)
o=mean(p,1);q=p-o;z=[2*q ones(size(q,1),1)]\sum(q.^2,2);c=z(1:2)'+o;r=sqrt(z(3)+sum(z(1:2).^2));e=vecnorm(p-c,2,2)-r;
end
