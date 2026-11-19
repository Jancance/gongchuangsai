function validate_source_paper50_20261111
% 当前50mm纸平面诊断；不含底盘坐标变换、独立全盘真值或运动准入。
root=fileparts(fileparts(mfilename('fullpath')));
out=fullfile(root,'evidence','integration','2026-11-11','source-view-entry','paper50-geometry');
d=jsondecode(fileread(fullfile(out,'measurement-input.json')));
p=d.corners_ABCD_px;q=[0 0;50 0;50 50;0 50];
[H,condition]=homography(p,q);Hfirst=homography(d.first_corners_ABCD_px,q);
checks=struct('name',{},'passed',{});
check('current50mm dimensions, not historical100mm',d.paper_width_mm==50 && d.paper_height_mm==50);
check('actual1920x1080 frame',isequal(d.image_size(:)',[1920 1080]));
check('corners and border extraction usable',all(d.border_line_support_counts>80) && all(d.border_line_rms_px<1));
crosses=zeros(4,1);
for k=1:4,a=p(mod(k,4)+1,:)-p(k,:);b=p(mod(k+1,4)+1,:)-p(mod(k,4)+1,:);crosses(k)=a(1)*b(2)-a(2)*b(1);end
check('convex clockwise quad',all(crosses>0));
check('normalized homography well conditioned',isfinite(condition) && condition<100);
check('four-corner numerical interpolation only',max(vecnorm(project(H,p)-q,2,2))<1e-7);
firstLast=vecnorm(p-d.first_corners_ABCD_px,2,2);
check('first to last paper corner repeatability below1px',max(firstLast)<1);
patches=d.fixed_surface_patches;
check('eight fixed surface patches unchanged integer position',numel(patches)==8 && ...
 all(arrayfun(@(x)x.correlation>.9 && all(x.integer_shift_px==0),patches)));
check('serial and all metric motion remained disabled',strcmp(d.summary.serial,'DISABLED') && ~d.motion_allowed);
check('saved log frame sequence complete',d.summary.sequence_contiguous && d.summary.frames==532);

centers=[];radii=[];rms=[];retained=[];centerPixels=[];
for k=1:numel(d.frame_boundary_points_px)
 pts=d.frame_boundary_points_px{k};
 if size(pts,1)<20,continue;end
 points=project(H,pts);[c,r,e,keep]=robust_circle(points);
 centers(end+1,:)=c;radii(end+1,1)=r;rms(end+1,1)=sqrt(mean(e.^2));
 retained(end+1,1)=sum(keep);centerPixels(end+1,:)=project(inv(H),c);
end
rimPoints=d.frame_boundary_points_px{end};
[baseCenter,baseRadius,baseError,~]=robust_circle(project(H,rimPoints));
[firstCenter,firstRadius,~,~]=robust_circle(project(Hfirst,rimPoints));
% Declared +/-1px perturbations: sensitivity, NOT measured corner uncertainty.
cornerDelta=zeros(256,1);radiusPerturbed=zeros(256,1);
for k=0:255
 delta=reshape(2*double(bitget(uint32(k),1:8))-1,4,2);
 Hi=homography(p+delta,q);
 [ci,ri,~,~]=robust_circle(project(Hi,rimPoints));
 cornerDelta(k+1)=norm(ci-baseCenter);radiusPerturbed(k+1)=ri;
end
[gx,gy]=meshgrid(0:160:1920,0:120:1080);den=[gx(:),gy(:),ones(numel(gx),1)]*H(3,:)';
check('projection denominator no zero or sign crossing over image',all(isfinite(den)) && all(den>0));
check('rim finite and qualitatively plausible with nominal150mm',all(isfinite(centers(:))) && all(radii>130 & radii<170));
% Corner plane axes are defined ONLY by the user's square; no vehicle signs.
local=project(H,[960 540;961 540;960 541]);
r=struct('engine','local MATLAB','version',version,'release',version('-release'), ...
 'image_sha256',d.image_sha256,'paper_width_mm',50,'paper_height_mm',50, ...
 'plane_axes','origin A; +X along AB; +Y along AD; millimetres; not chassis axes', ...
 'homography_image_to_paper_plane',H,'normalized_linear_condition',condition, ...
 'numerical_corner_error_mm',vecnorm(project(H,p)-q,2,2)', ...
 'first_last_corner_shift_px',firstLast','frame_count',d.summary.frames, ...
 'detected_frames',d.summary.detected_frames,'stable_frames',d.summary.stable_frames, ...
 'rim_metric_frame_count',numel(radii),'rim_center_plane_median_mm',median(centers,1), ...
 'rim_center_plane_peak_to_peak_mm',max(centers,[],1)-min(centers,[],1), ...
 'rim_center_reprojected_pixel_median',median(centerPixels,1), ...
 'rim_radius_median_mm',median(radii),'rim_radius_range_mm',[min(radii) max(radii)], ...
 'rim_fit_rms_median_mm',median(rms),'rim_fit_rms_max_mm',max(rms), ...
 'rim_last_center_plane_mm',baseCenter,'rim_last_radius_mm',baseRadius, ...
 'rim_last_rms_mm',sqrt(mean(baseError.^2)), ...
 'first_last_homography_rim_center_change_mm',norm(firstCenter-baseCenter), ...
 'first_last_homography_rim_radius_change_mm',firstRadius-baseRadius, ...
 'declared_corner_perturbation_px',1,'corner_perturbation_cases',256, ...
 'declared_corner_sensitivity_center_max_mm',max(cornerDelta), ...
 'declared_corner_sensitivity_radius_range_mm',[min(radiusPerturbed) max(radiusPerturbed)], ...
 'image_center_local_scale_mm_per_px',[norm(local(2,:)-local(1,:)) norm(local(3,:)-local(1,:))], ...
 'check_count',numel(checks),'failure_count',sum(~[checks.passed]),'checks',checks,'passed',all([checks.passed]), ...
 'four_corner_fit_is_not_independent_validation',true,'independent_known_metric_check_points',[], ...
 'paper_dimension_uncertainty_measured',false,'full_plane_accuracy_accepted',false, ...
 'plane_to_chassis_verified',false,'pose_binding_verified',false, ...
 'sensor_exposure_age_verified',false,'material_top_mapping_verified',false,'motion_allowed',false);
fid=fopen(fullfile(out,'matlab-result.json'),'w','n','UTF-8');assert(fid~=-1);fprintf(fid,'%s\n',jsonencode(r));fclose(fid);
save(fullfile(out,'matlab-result.mat'),'r','centers','radii','rms','retained');
fprintf('MATLAB50mm: %d checks, %d failures; %d frames/%d stable; rim r%.3fmm RMS%.3fmm\n', ...
 r.check_count,r.failure_count,r.frame_count,r.stable_frames,r.rim_radius_median_mm,r.rim_fit_rms_median_mm);
fprintf('Declared +/-1px corner sensitivity: center max%.3fmm; radius[%.3f,%.3f]mm. No motion permission.\n', ...
 r.declared_corner_sensitivity_center_max_mm,r.declared_corner_sensitivity_radius_range_mm);
assert(r.passed);
 function check(name,passed)
  checks(end+1)=struct('name',name,'passed',logical(passed));
 end
end
function [H,condition]=homography(p,q)
[pn,P]=normalize(p);[qn,Q]=normalize(q);A=zeros(8,8);b=zeros(8,1);
for k=1:4
 x=pn(k,1);y=pn(k,2);u=qn(k,1);v=qn(k,2);
 A(2*k-1,:)=[x y 1 0 0 0 -u*x -u*y];A(2*k,:)=[0 0 0 x y 1 -v*x -v*y];b(2*k-1:2*k)=[u;v];
end
condition=cond(A);z=A\b;H=Q\[z(1:3)';z(4:6)';z(7:8)' 1]*P;H=H/H(3,3);
end
function [p,T]=normalize(p)
c=mean(p,1);s=sqrt(2)/mean(vecnorm(p-c,2,2));T=[s 0 -s*c(1);0 s -s*c(2);0 0 1];p=project(T,p);
end
function p=project(H,p)
p=[p ones(size(p,1),1)]*H';assert(all(abs(p(:,3))>1e-10));p=p(:,1:2)./p(:,3);
end
function [c,r,res,keep]=robust_circle(p)
keep=true(size(p,1),1);
for k=1:5
 [c,r,~]=circle(p(keep,:));res=vecnorm(p-c,2,2)-r;
 m=median(res(keep));mad=median(abs(res(keep)-m));keep=abs(res-m)<=max(1.5,3*1.4826*mad);
end
[c,r,res]=circle(p(keep,:));
end
function [c,r,res]=circle(p)
o=mean(p,1);q=p-o;z=[2*q ones(size(q,1),1)]\sum(q.^2,2);
c=z(1:2)'+o;r=sqrt(z(3)+sum(z(1:2).^2));res=vecnorm(p-c,2,2)-r;
end
