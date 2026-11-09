function validate_source_finalstop_matlab_20261106
% Static field-image diagnostic. No motion implementation or device access.
root=fileparts(fileparts(mfilename('fullpath')));
out=fullfile(root,'evidence','integration','2026-11-06','source-vision-new-pose','finalstop-geometry');
d=jsondecode(fileread(fullfile(out,'measurement-input.json')));
previous=jsondecode(fileread(fullfile(root,'evidence','integration','2026-11-06','source-vision-new-pose','paper48-geometry','matlab-result.json')));
pose=jsondecode(fileread(fullfile(root,'calibration','source_pose_current_20261106.json')));
p=d.corners_ABCD_px;xy=[0,0;100,0;100,100;0,100];H=homography(p,xy);
checks=struct('name',{},'passed',{});
check('current original size and retained paper dimensions',isequal(d.image_size(:)',[1920,1080]) && d.paper_width_mm==100 && d.paper_height_mm==100);
check('four corners inside image and line support sufficient',all(p(:,1)>0 & p(:,1)<1920 & p(:,2)>0 & p(:,2)<1080) && all(d.border_line_support_counts>200));
crosses=zeros(4,1);for k=1:4,a=p(mod(k,4)+1,:)-p(k,:);b=p(mod(k+1,4)+1,:)-p(mod(k,4)+1,:);crosses(k)=a(1)*b(2)-a(2)*b(1);end
check('paper convex and uncrossed',all(crosses>0)||all(crosses<0));
errors=vecnorm(project(H,p)-xy,2,2);check('four corner fit numerical only',max(errors)<1e-7);
heldout=project(H,d.heldout_MEF_px);heldout_error=vecnorm(heldout-[50,50;25,25;75,25],2,2);
check('independent MEF marks within existing3mm diagnostic',max(heldout_error)<=3);
circles=struct('id',{},'center_plane_mm',{},'center_image_px',{},'radius_mm',{},'rms_mm',{},'coverage_deg',{},'retained_points',{});
for k=1:3
 q=project(H,d.circles(k).boundary_image_px);[c,r,res,keep]=robust_circle(q);
 angles=sort(mod(atan2d(q(keep,2)-c(2),q(keep,1)-c(1)),360));coverage=360-max(diff([angles;angles(1)+360]));
 circles(k)=struct('id',d.circles(k).id,'center_plane_mm',c,'center_image_px',project(inv(H),c),'radius_mm',r,'rms_mm',sqrt(mean(res.^2)),'coverage_deg',coverage,'retained_points',sum(keep));
 check(['drawn target ' d.circles(k).id],coverage>270 && sum(keep)>240 && r>20 && r<30 && sqrt(mean(res.^2))<3);
end
q=project(H,d.rim_boundary_image_px);[disk,radius,res,keep]=robust_circle(q);
rim=struct('center_plane_mm',disk,'center_image_px',project(inv(H),disk),'radius_mm',radius,'rms_mm',sqrt(mean(res.^2)), ...
 'retained_points',sum(keep),'planning_obstacle_radius_mm',max(150,radius));
check('observed rim plausible with official150mm obstacle retained',radius>130 && radius<170 && rim.rms_mm<3 && sum(keep)>80);
tcp_px=[960,540];tcp=project(H,tcp_px);f=project(H,tcp_px+[0,-1])-tcp;f=f/norm(f);right=[-f(2),f(1)];
if dot(right,project(H,tcp_px+[1,0])-tcp)<0,right=-right;end
L=pose.derived_geometry_diagnostic_only.pivot_to_TCP_candidate_mm;
zero=pose.retained_mechanics.retracted_pivot_to_TCP_candidate_mm;maximum=pose.retained_mechanics.maximum_extension_mm;
T=[f(1),f(2),L-dot(tcp,f);-right(1),-right(2),dot(tcp,right);0,0,1];
check('declared installation lengths and rigid transform',L==209 && zero==161 && maximum==170 && norm(T(1:2,1:2)'*T(1:2,1:2)-eye(2),'fro')<1e-9);
centers=vertcat(circles.center_plane_mm);body=project(T,centers);disk_body=project(T,disk);
extension=vecnorm(body,2,2)-zero;angle=mod(180+atan2d(body(:,2),body(:,1)),360);
b=pose.retained_mechanics.chassis_bounds_front_rear_left_right_mm(:)';
gap=rectangle_gap(disk_body,b,rim.planning_obstacle_radius_mm);
margin=min([extension;maximum-extension]);
% Reconstruct manual displacement under unchanged heading assumption, and
% independently inspect inter-target distortion rather than asserting truth.
old=previous.body_target_candidate_mm;delta=old-body;
translation=mean(delta,1);translation_residual=vecnorm(delta-translation,2,2);
pairs=[1,2;2,3;3,1];spacing=zeros(3,1);old_spacing=spacing;
for k=1:3,spacing(k)=norm(body(pairs(k,1),:)-body(pairs(k,2),:));old_spacing(k)=norm(old(pairs(k,1),:)-old(pairs(k,2),:));end
% A margin sweep quantifies a declared bound, not measured whole-plane error.
bounds=[1,2,3,5,10];worst_margin=margin-bounds;
% Small static displacement search, independently bounded to +/-15mm.
[xx,yy]=meshgrid(-15:.1:15,-15:.1:15);shifts=[xx(:),yy(:)];ext=zeros(size(shifts,1),3);
for k=1:3,v=body(k,:)-shifts;ext(:,k)=vecnorm(v,2,2)-zero;end
margins=min([ext,maximum-ext],[],2);gaps=rectangle_gap(disk_body-shifts,b,rim.planning_obstacle_radius_mm);
valid=margins>=0 & gaps>=50;ids=find(valid);
[~,order]=sortrows([-margins(valid),vecnorm(shifts(valid,:),2,2)],[1,2]);i=ids(order(1));
best=struct('right_forward_shift_mm',shifts(i,:),'extension_mm',ext(i,:),'minimum_margin_mm',margins(i),'nominal_gap_mm',gaps(i),'is_diagnostic_not_command',true);
rounded_shift=[-5,5];v=body-rounded_shift;rounded_ext=vecnorm(v,2,2)-zero;
rounded=struct('right_forward_shift_mm',rounded_shift,'extension_mm',rounded_ext','minimum_margin_mm',min([rounded_ext;maximum-rounded_ext]), ...
 'nominal_gap_mm',rectangle_gap(disk_body-rounded_shift,b,rim.planning_obstacle_radius_mm),'is_diagnostic_not_command',true);
report=struct('engine','local MATLAB','release',version('-release'),'version',version, ...
 'image_sha256',d.paper_image_sha256,'calibration_id',['source-finalstop-' d.paper_image_sha256(1:12) '-diagnostic'], ...
 'homography_image_to_plane',H,'heldout_MEF_mapped_mm',heldout,'heldout_MEF_error_mm',heldout_error', ...
 'heldout_rms_mm',sqrt(mean(heldout_error.^2)),'heldout_max_mm',max(heldout_error),'circles',circles,'rim',rim, ...
 'plane_to_chassis_candidate',T,'body_target_candidate_mm',body,'disk_center_body_candidate_mm',disk_body, ...
 'extension_candidate_mm',extension','logical_angle_candidate_deg',angle','minimum_stroke_margin_mm',margin, ...
 'all_three_at_current_stop_within0to170',all(extension>=0 & extension<=maximum),'meets_prior10mm_margin',margin>=10, ...
 'current_generic0to180deg_firmware_accepts_all',all(angle>=0 & angle<=180),'nominal_outermost_gap_mm',gap, ...
 'actual_post_move_gap_measured',false,'translation_from_previous_conditional_mm',translation, ...
 'per_target_translation_mm',delta,'translation_residual_max_mm',max(translation_residual), ...
 'target_pair_distances_mm',spacing','previous_pair_distances_mm',old_spacing', ...
 'declared_radial_error_bounds_mm',bounds,'worst_stroke_margin_with_declared_bound_mm',worst_margin, ...
 'small_shift_best_nominal',best,'small_shift_rounded_trial',rounded, ...
 'check_count',numel(checks),'failure_count',sum(~[checks.passed]),'checks',checks,'passed',all([checks.passed]), ...
 'full_plane_accuracy_accepted',false,'plane_to_chassis_verified',false,'physical_reach_accepted',false, ...
 'joint_reference_verified',false,'material_top_mapping_verified',false,'swept_path_verified',false, ...
 'motion_allowed',false,'actuator_commands',0);
fid=fopen(fullfile(out,'matlab-result.json'),'w','n','UTF-8');assert(fid~=-1);fprintf(fid,'%s',jsonencode(report));fclose(fid);
save(fullfile(out,'matlab-result.mat'),'report');
fprintf('FINALSTOP MATLAB checks%d failures%d MEFrms%.3f/max%.3fmm; extension[%.2f %.2f %.2f] margin%.2f gap%.2f\n', ...
 numel(checks),report.failure_count,report.heldout_rms_mm,report.heldout_max_mm,extension(1),extension(2),extension(3),margin,gap);
fprintf('Conditional displacement right/forward[%.2f %.2f]mm residual%.2f; physical acceptance remains false\n',translation(1),translation(2),max(translation_residual));
assert(report.passed);
 function check(name,passed)
  checks(end+1)=struct('name',name,'passed',logical(passed));
 end
end
function H=homography(p,q)
A=zeros(8,8);b=zeros(8,1);
for k=1:4,x=p(k,1);y=p(k,2);u=q(k,1);v=q(k,2);A(2*k-1,:)=[x,y,1,0,0,0,-u*x,-u*y];A(2*k,:)=[0,0,0,x,y,1,-v*x,-v*y];b(2*k-1:2*k)=[u;v];end
z=A\b;H=[z(1:3)';z(4:6)';z(7:8)',1];
end
function p=project(H,p)
p=[p,ones(size(p,1),1)]*H';assert(all(abs(p(:,3))>1e-10));p=p(:,1:2)./p(:,3);
end
function [c,r,res,keep]=robust_circle(p)
keep=true(size(p,1),1);
for k=1:5
 [c,r,~]=circle(p(keep,:));res=vecnorm(p-c,2,2)-r;med=median(res(keep));mad=median(abs(res(keep)-med));keep=abs(res-med)<=max(1.5,3*1.4826*mad);
end
[c,r,res]=circle(p(keep,:));
end
function [c,r,res]=circle(p)
o=mean(p,1);q=p-o;z=[2*q,ones(size(q,1),1)]\sum(q.^2,2);c=z(1:2)'+o;r=sqrt(z(3)+sum(z(1:2).^2));res=vecnorm(p-c,2,2)-r;
end
function gap=rectangle_gap(d,b,r)
dx=max(max(d(:,1)-b(4),-b(3)-d(:,1)),0);dy=max(max(d(:,2)-b(1),-b(2)-d(:,2)),0);gap=hypot(dx,dy)-r;
end
