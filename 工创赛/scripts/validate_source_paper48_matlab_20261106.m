function validate_source_paper48_matlab_20261106
% Current physical paper and rim observations only. No motor commands.
root=fileparts(fileparts(mfilename('fullpath')));
out=fullfile(root,'evidence','integration','2026-11-06','source-vision-new-pose','paper48-geometry');
d=jsondecode(fileread(fullfile(out,'measurement-input.json')));
pose=jsondecode(fileread(fullfile(root,'calibration','source_pose_current_20261106.json')));
corners=d.corners_ABCD_px;physical=[0,0;100,0;100,100;0,100];H=homography(corners,physical);
checks=struct('name',{},'passed',{});
check('current original image and source size match',isequal(d.image_size(:)',[1920,1080]) && d.paper_width_mm==100 && d.paper_height_mm==100);
check('corners finite and in image',all(isfinite(corners(:))) && all(corners(:,1)>=0 & corners(:,1)<1920 & corners(:,2)>=0 & corners(:,2)<1080));
crosses=zeros(4,1);for k=1:4,a=corners(mod(k,4)+1,:)-corners(k,:);b=corners(mod(k+1,4)+1,:)-corners(mod(k,4)+1,:);crosses(k)=a(1)*b(2)-a(2)*b(1);end
check('paper quad convex without corner crossing',all(crosses>0) || all(crosses<0));
fit_error=sqrt(sum((project(H,corners)-physical).^2,2));
heldout_expected=[50,50;25,25;75,25];heldout=project(H,d.heldout_MEF_px);heldout_error=sqrt(sum((heldout-heldout_expected).^2,2));
check('four corner residual is numerical only',max(fit_error)<1e-7);
check('independent drawnMEF within existing3mm diagnostic',max(heldout_error)<=3);
matches=d.fixed_surface_patches;accepted=[matches.correlation]>=.9;
location=vertcat(matches(accepted).plain_px);shifts=vertcat(matches(accepted).integer_shift_px);
if size(location,2)~=2,location=reshape(location,2,[])';shifts=reshape(shifts,2,[])';end
same_pose=sum(accepted)>=6 && rank(location-mean(location,1))==2 && max(vecnorm(shifts,2,2))<=1;
check('paper insertion preserved observed view',same_pose);
circles=struct('id',{},'center_plane_mm',{},'center_image_px',{},'radius_mm',{},'rms_mm',{},'coverage_deg',{});
for k=1:numel(d.circles)
 q=project(H,d.circles(k).boundary_image_px);[c,r,res]=circle(q);
 angles=sort(mod(atan2d(q(:,2)-c(2),q(:,1)-c(1)),360));coverage=360-max(diff([angles;angles(1)+360]));
 circles(k)=struct('id',d.circles(k).id,'center_plane_mm',c,'center_image_px',project(inv(H),c), ...
  'radius_mm',r,'rms_mm',sqrt(mean(res.^2)),'coverage_deg',coverage);
 check(['current drawn circle ' d.circles(k).id],coverage>300 && r>20 && r<30 && sqrt(mean(res.^2))<3);
end
q=project(H,d.rim_boundary_image_px);keep=true(size(q,1),1);
for k=1:5
 [c,r,~]=circle(q(keep,:));res=sqrt(sum((q-c).^2,2))-r;med=median(res(keep));mad=median(abs(res(keep)-med));keep=abs(res-med)<=max(1.5,3*1.4826*mad);
end
[disk,radius,res]=circle(q(keep,:));
rim=struct('center_plane_mm',disk,'center_image_px',project(inv(H),disk),'radius_mm',radius, ...
 'rms_mm',sqrt(mean(res.^2)),'planning_obstacle_radius_mm',max(150,radius),'retained_points',sum(keep));
check('rim scale checked without overriding official300mm',radius>130 && radius<170 && rim.rms_mm<3);
% User accepted image-centre TCP and image-up = chassis-right. Keep these
% as declared installation inputs, not independently measured transform.
tcp_px=[960,540];tcp=project(H,tcp_px);f=project(H,tcp_px+[0,-1])-tcp;f=f/norm(f);right=[-f(2),f(1)];
if dot(right,project(H,tcp_px+[1,0])-tcp)<0,right=-right;end
L=pose.derived_geometry_diagnostic_only.pivot_to_TCP_candidate_mm;
zero=pose.retained_mechanics.retracted_pivot_to_TCP_candidate_mm;maximum=pose.retained_mechanics.maximum_extension_mm;
T=[f(1),f(2),L-dot(tcp,f);-right(1),-right(2),dot(tcp,right);0,0,1];
centers=vertcat(circles.center_plane_mm);if size(centers,2)~=2,centers=reshape(centers,2,[])';end
body=project(T,centers);disk_body=project(T,disk);extension=hypot(body(:,1),body(:,2))-zero;
angle=mod(180+atan2d(body(:,2),body(:,1)),360);
b=pose.retained_mechanics.chassis_bounds_front_rear_left_right_mm(:)';
gap=rectangle_gap(disk_body,b,rim.planning_obstacle_radius_mm);
check('new stretch and rigid radius arithmetic',L==209 && zero==161 && maximum==170);
check('plane-to-body transform preserves lengths',norm(T(1:2,1:2)'*T(1:2,1:2)-eye(2),'fro')<1e-9);
% Diagnostic-only single-stop search. No controller code or movement release.
[xx,yy]=meshgrid(-100:1:100,-100:1:100);positions=[xx(:),yy(:)];ext=zeros(size(positions,1),3);
for k=1:3,delta=body(k,:)-positions;ext(:,k)=hypot(delta(:,1),delta(:,2))-zero;end
margin=min([ext,maximum-ext],[],2);endgap=rectangle_gap(disk_body-positions,b,rim.planning_obstacle_radius_mm);
valid=margin>=0 & endgap>=50;candidates=find(valid);candidate=[];
if ~isempty(candidates)
 [~,order]=sortrows([-margin(valid),vecnorm(positions(valid,:),2,2)],[1,2]);idx=candidates(order(1));
 seed=positions(idx,:);[x,y]=meshgrid(seed(1)-1:.1:seed(1)+1,seed(2)-1:.1:seed(2)+1);p=[x(:),y(:)];e=zeros(size(p,1),3);a=e;
 for k=1:3,v=body(k,:)-p;e(:,k)=hypot(v(:,1),v(:,2))-zero;a(:,k)=mod(180+atan2d(v(:,2),v(:,1)),360);end
 g=rectangle_gap(disk_body-p,b,rim.planning_obstacle_radius_mm);m=min([e,maximum-e],[],2);ok=m>=0 & g>=50;ids=find(ok);
 [~,o]=sortrows([-m(ok),vecnorm(p(ok,:),2,2)],[1,2]);idx=ids(o(1));shift=p(idx,:);
 candidate=struct('chassis_right_forward_shift_mm',shift,'extension_mm',e(idx,:),'logical_angles_deg',a(idx,:), ...
  'minimum_stroke_margin_mm',m(idx),'nominal_endpoint_gap_mm',g(idx),'meets_prior10mm_margin',m(idx)>=10, ...
  'within_current_generic0to180deg_firmware',all(a(idx,:)>=0 & a(idx,:)<=180), ...
  'current_route_or_lane_verified',false,'is_diagnostic_not_command',true);
end
report=struct('engine','local MATLAB','version',version,'release',version('-release'), ...
 'paper_image_sha256',d.paper_image_sha256,'plain_image_sha256',d.plain_image_sha256, ...
 'image_size',[1920,1080],'calibration_id',['source-paper48-' d.paper_image_sha256(1:12) '-diagnostic'], ...
 'homography_image_to_plane',H,'fit_error_mm',fit_error','heldout_MEF_expected_mm',heldout_expected, ...
 'heldout_MEF_mapped_mm',heldout,'heldout_MEF_error_mm',heldout_error', ...
 'heldout_rms_mm',sqrt(mean(heldout_error.^2)),'heldout_max_mm',max(heldout_error), ...
 'paper_local_check_passed',max(heldout_error)<=3,'fixed_patch_count',sum(accepted), ...
 'max_integer_patch_shift_px',max(vecnorm(shifts,2,2)),'paper_plain_view_consistent',same_pose, ...
 'circles',circles,'rim',rim,'tcp_plane_mm',tcp,'plane_to_chassis_candidate',T, ...
 'body_target_candidate_mm',body,'current_extension_candidate_mm',extension','current_logical_angle_candidate_deg',angle', ...
 'all_three_at_current_stop_within0to170',all(extension>=0 & extension<=maximum), ...
 'current_model_outermost_gap_mm',gap,'user_reported_gap_approx_mm',50,'model_minus_user_gap_mm',gap-50, ...
 'single_stop_diagnostic',candidate,'checks',checks,'check_count',numel(checks),'failure_count',sum(~[checks.passed]), ...
 'passed',all([checks.passed]),'field_calibration_accepted',false,'full_plane_metric_accuracy_accepted',false, ...
 'actual_joint_reference_verified',false,'material_top_mapping_verified',false,'motion_allowed',false);
write_json(fullfile(out,'matlab-result.json'),report);save(fullfile(out,'matlab-result.mat'),'report');
fprintf('PAPER48 MATLAB checks%d failures%d MEFrms%.3f/max%.3fmm; current extensions[%.2f %.2f %.2f] gap%.2fmm; no command\n', ...
 numel(checks),report.failure_count,report.heldout_rms_mm,report.heldout_max_mm,extension(1),extension(2),extension(3),gap);
assert(report.passed);
 function check(name,passed)
  checks(end+1)=struct('name',name,'passed',logical(passed)); %#ok<AGROW>
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
function [c,r,res]=circle(p)
o=mean(p,1);q=p-o;z=[2*q,ones(size(q,1),1)]\sum(q.^2,2);c=z(1:2)'+o;r=sqrt(z(3)+sum(z(1:2).^2));res=sqrt(sum((p-c).^2,2))-r;
end
function gap=rectangle_gap(d,b,r)
dx=max(max(d(:,1)-b(4),-b(3)-d(:,1)),0);dy=max(max(d(:,2)-b(1),-b(2)-d(:,2)),0);gap=hypot(dx,dy)-r;
end
function write_json(p,v)
fid=fopen(p,'w','n','UTF-8');assert(fid~=-1);c=onCleanup(@()fclose(fid));fprintf(fid,'%s',jsonencode(v)); %#ok<NASGU>
end
