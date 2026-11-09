function validate_source_single_pose_matlab_20261105
% 用户选定单停车点：MATLAB加入真实车体包络及盘面外圆的几何检查。
% 只仿真，绝不输出串口/电机命令；车道、抓放Z与真实调度仍需单独验证。
root=fileparts(fileparts(mfilename('fullpath')));
base=fullfile(root,'evidence','integration','2026-11-05','source-markers');
continuation=fullfile(base,'paper100-transfer','continuation');
out=fullfile(continuation,'matlab-single-stop-stroke170');
if ~exist(out,'dir'),mkdir(out);end
reference=read_json(fullfile(root,'calibration','source_tcp_center_reference_20261105.json'));
old=read_json(fullfile(continuation,'matlab','result.json'));
assert(strcmp(reference.workflow.selected,'one_chassis_stop_for_all_three'));
assert(reference.mechanics.extension_max_from_home_mm==170);
q=old.offsets_right_forward_mm; H=old.independent_H;
L=reference.mechanics.current_rotation_pivot_to_tcp_mm;
e=reference.mechanics.source_prepare_nominal_extension_mm;
maximum=reference.mechanics.extension_max_from_home_mm;

% 原图外圆用径向梯度取样，先映射边界再在平面拟合圆。
% 原图缺失的上弧不补点；不把图像拟合圆心直接代入H。
image=double(imread(fullfile(base,'paper100-8fa7a4e70382','source_markers_20261105_185038.png')));
gray=.2989*image(:,:,1)+.5870*image(:,:,2)+.1140*image(:,:,3);
gx=conv2(gray,[-1,0,1]/2,'same'); gy=conv2(gray,[-1;0;1]/2,'same');
gradient=hypot(gx,gy); pixels=[];
for angle=0:359
 radius=575:.5:630; x=985+radius*cosd(angle); y=450+radius*sind(angle);
 valid=x>=2 & x<=size(gray,2)-1 & y>=2 & y<=size(gray,1)-1;
 if ~any(valid),continue;end
 x=x(valid);y=y(valid);strength=interp2(gradient,x,y,'linear');
 [value,j]=max(strength);
 if value>=6,pixels(end+1,:)=[x(j),y(j)];end %#ok<AGROW>
end
mapped=map(H,pixels); keep=true(size(mapped,1),1);
for iteration=1:5
 [center,radius,residual]=circle_fit(mapped(keep,:));
 all_residual=sqrt(sum((mapped-center).^2,2))-radius;
 med=median(all_residual(keep)); mad=median(abs(all_residual(keep)-med));
 keep=abs(all_residual-med)<=max(1.5,3*1.4826*mad);
end
[center,radius,residual]=circle_fit(mapped(keep,:));
tcp=map(H,reference.reference_image_px(:)');
direction=reference.extension_image_direction(:)';direction=direction/norm(direction);
forward=map(H,reference.reference_image_px(:)'+direction)-tcp;forward=forward/norm(forward);
right=[-forward(2),forward(1)];
if dot(right,map(H,reference.reference_image_px(:)'+[1,0])-tcp)<0,right=-right;end
center_local=(center-tcp)*[right',forward'];
center_body=[L+center_local(2),-center_local(1)];
rim=struct('center_paper_mm',center,'center_local_right_forward_mm',center_local, ...
 'center_body_right_forward_from_pivot_mm',center_body,'radius_mm',radius, ...
 'original_samples',size(mapped,1),'retained_samples',sum(keep), ...
 'boundary_rms_mm',sqrt(mean(residual.^2)),'missing_arc_imputed',false, ...
 'absolute_position_physically_verified',false);
data=struct('pixels',pixels,'retained',keep,'rim',rim);
write_json(fullfile(out,'rim-boundary-diagnostic.json'),data);

report=struct;report.engine='local MATLAB';report.version=version;
report.reference=reference;report.target_ids=old.target_ids;report.offsets_right_forward_mm=q;
report.rim=rim;report.current_tip_to_material_top_gap_mm= ...
 reference.scene_measurements.gripper_tip_above_source_plane_mm-reference.scene_measurements.material_height_mm;
% 图像边界半径不能静默覆盖SRC-01的官方名义150mm。
planning_radius=max(150,radius);
report.planning_disk_radius_mm=planning_radius;
report.rim_minus_nominal_radius_mm=radius-150;
report.rim_radius_is_independent_physical_measurement=false;
report.current_static_tip_clears_material_by_10mm=report.current_tip_to_material_top_gap_mm>=10;
report.full_motion_validation_passed=false;report.motion_allowed=false;report.hardware_operated=false;
% 独立验证包络距离与扫掠算法；不使用这些合成尺寸冒充实车尺寸。
test_bounds=struct('left',1,'right',1,'front',1,'rear',1);
algorithm_checks=[abs(clearance([5,0],test_bounds,1)-3)<1e-8, ...
 abs(swept_clearance([5,0],test_bounds,1,[2,0])-1)<1e-8, ...
 swept_clearance([5,0],test_bounds,1,[10,0])<0, ...
 abs(swept_clearance([0,5],test_bounds,1,[10,0])-3)<1e-8];
rng(170);bounded=true;
for sample=1:100
 test_center=randn(1,2)*8;test_shift=randn(1,2)*10;
 sampled=Inf;
 for t=linspace(0,1,41),sampled=min(sampled,clearance(test_center-t*test_shift,test_bounds,1));end
 lower_bound=sampled-norm(test_shift)/80;
 exact=swept_clearance(test_center,test_bounds,1,test_shift);
 bounded=bounded && lower_bound<=exact+1e-8;
end
algorithm_checks(end+1)=bounded;
report.algorithm_check_count=numel(algorithm_checks);
report.algorithm_failure_count=sum(~algorithm_checks);
report.swept_distance_synthetic_cases=100;
assert(all(algorithm_checks),'Swept-volume geometry algorithm validation failed');
bounds=reference.scene_measurements.chassis_bounds_from_dm_axis_mm;
keys={'front','rear','left','right'}; complete=true;
for k=1:4,value=bounds.(keys{k});complete=complete && isnumeric(value) && isscalar(value) && isfinite(value) && value>=0;end
report.body_dimensions_complete=complete;
report.geometry_passed=false;
report.pending={'chassis world-pose and gray-lane envelope','TCP/height/feature-plane acceptance', ...
 'lift/grip/carrier and combined failure exits','firmware admission still 0..180; no execution code change'};
if ~complete
 report.pending=[{'four body edge distances from DM pivot'},report.pending];
 write_json(fullfile(out,'result.json'),report);
 fprintf('Single-stop model: body dimensions missing; rim radius=%.3fmm RMS=%.3fmm; tip margin=%.1fmm; motion_allowed=0\n', ...
 radius,rim.boundary_rms_mm,report.current_tip_to_material_top_gap_mm);
 return;
end

% 同时检查每个目标、车盘50mm净距及直线移车全过程。
% 采样净距减去路径离散的Lipschitz上界，避免采样点之间漏碰撞。
[bx,by]=meshgrid(-300:2:300,-200:2:200);x=bx(:);y=by(:);
xp=q(:,1)'-x;yp=L+q(:,2)'-y;
ext=hypot(xp,yp)-(L-e);angles=180-atan2d(xp,yp);
joint_mask=all(ext>=10 & ext<=maximum-10 & angles>=2 & angles<=358,2);
measured_gap=reference.scene_measurements.current_chassis_to_disk_min_gap_mm;
current_gap=clearance(center_body,bounds,planning_radius);
report.current_body_model_gap_mm=current_gap;
report.current_gap_measurement_mm=measured_gap;
report.current_gap_model_error_mm=current_gap-measured_gap;
report.current_gap_consistency_5mm_diagnostic=abs(current_gap-measured_gap)<=5;
path_gap=Inf(size(x));
for t=linspace(0,1,41)
 body_shift=[t*y,-t*x];
 path_gap=min(path_gap,clearance(center_body-body_shift,bounds,planning_radius));
end
path_gap=path_gap-hypot(x,y)/80;
mask=joint_mask & path_gap>=50;
ids=find(mask);report.search=struct('grid_count',numel(x),'joint_feasible_count',sum(joint_mask), ...
 'body_and_path_feasible_count',numel(ids),'grid_step_mm',2,'stroke_margin_mm',10,'angle_margin_deg',2, ...
 'body_disk_min_gap_mm',50,'path_sample_count',41,'path_sample_bound_subtracted',true);
if ~isempty(ids)
 [distance,k]=min(hypot(x(ids),y(ids)));k=ids(k);shift=[x(k),y(k)];body_shift=[y(k),-x(k)];
 exact_path_gap=swept_clearance(center_body,bounds,planning_radius,body_shift);
 report.candidate=struct('local_right_forward_mm',shift,'chassis_right_forward_mm',body_shift, ...
  'travel_mm',distance,'extension_mm',ext(k,:),'logical_deg',angles(k,:), ...
  'conservative_path_gap_mm',path_gap(k),'exact_swept_rectangle_gap_mm',exact_path_gap);
 report.geometry_passed=report.current_gap_consistency_5mm_diagnostic && exact_path_gap>=50 ...
  && report.current_static_tip_clears_material_by_10mm;
end
report.scope='geometric rectangle/disk/endpoint model only; not lane, cable, dynamics, lift or runtime scheduling validation';
write_json(fullfile(out,'result.json'),report);
save(fullfile(out,'simulation.mat'),'report','reference','q','rim');
fprintf('Single-stop joint=%d body+path=%d current_gap=%.2f measured=%.2f geometry_passed=%d motion_allowed=0\n', ...
 sum(joint_mask),numel(ids),current_gap,measured_gap,report.geometry_passed);
if isfield(report,'candidate'),disp(report.candidate);end
end

function gaps=clearance(center,bounds,radius)
dx=max(max(-bounds.left-center(:,1),center(:,1)-bounds.right),0);
dy=max(max(-bounds.rear-center(:,2),center(:,2)-bounds.front),0);
gaps=hypot(dx,dy)-radius;
end

function gap=swept_clearance(center,bounds,radius,shift)
corners=[-bounds.left,-bounds.rear;bounds.right,-bounds.rear; ...
 bounds.right,bounds.front;-bounds.left,bounds.front];
vertices=[corners;corners+shift];hull=convhull(vertices(:,1),vertices(:,2));poly=vertices(hull,:);
if inpolygon(center(1),center(2),poly(:,1),poly(:,2)),gap=-radius;return;end
nearest=Inf;
for k=1:size(poly,1)-1
 a=poly(k,:);b=poly(k+1,:);ab=b-a;t=max(0,min(1,dot(center-a,ab)/dot(ab,ab)));
 nearest=min(nearest,norm(center-(a+t*ab)));
end
gap=nearest-radius;
end

function mapped=map(H,p)
q=[p,ones(size(p,1),1)]*H';assert(all(isfinite(q(:))) && all(abs(q(:,3))>1e-10));mapped=q(:,1:2)./q(:,3);
end

function [center,radius,residual]=circle_fit(p)
assert(size(p,1)>=12);origin=mean(p,1);local=p-origin;A=[2*local,ones(size(p,1),1)];assert(rank(A)==3);
c=A\sum(local.^2,2);center=c(1:2)'+origin;radius=sqrt(c(3)+sum(c(1:2).^2));
residual=sqrt(sum((p-center).^2,2))-radius;
end

function value=read_json(path)
fid=fopen(path,'r','n','UTF-8');assert(fid~=-1);cleanup=onCleanup(@() fclose(fid)); %#ok<NASGU>
value=jsondecode(fscanf(fid,'%c'));
end

function write_json(path,value)
fid=fopen(path,'w','n','UTF-8');assert(fid~=-1);cleanup=onCleanup(@() fclose(fid)); %#ok<NASGU>
fprintf(fid,'%s',jsonencode(value));
end
