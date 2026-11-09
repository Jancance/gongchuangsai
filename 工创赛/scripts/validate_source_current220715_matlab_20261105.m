function validate_source_current220715_matlab_20261105
% 220715新原图：MATLAB独立纸面H/留出点/可见圆弧/旧场景变化诊断。
% 不生成电机命令；重启后当前关节参考未恢复，不能沿用旧纠偏量。
root=fileparts(fileparts(mfilename('fullpath')));
out=fullfile(root,'evidence','integration','2026-11-05','source-markers','current-220715-a5702160ff7f');
data=read_json(fullfile(out,'measurement-input.json'));
reference=read_json(fullfile(root,'calibration','source_tcp_center_reference_20261105.json'));
assert(isequal(data.image_size(:)',[1920,1080]));
corners=data.corners_ABCD_px;physical=[0,0;100,0;100,100;0,100];
H=homography4(corners,physical);
checks=struct('name',{},'passed',{});
check('all corners finite and in image',all(isfinite(corners(:))) && all(corners(:,1)>=0 & corners(:,1)<1920 & corners(:,2)>=0 & corners(:,2)<1080));
crosses=zeros(4,1);
for k=1:4,a=corners(mod(k,4)+1,:)-corners(k,:);b=corners(mod(k+1,4)+1,:)-corners(mod(k,4)+1,:);crosses(k)=a(1)*b(2)-a(2)*b(1);end
check('four corners form convex noncrossed quad',all(crosses>0) || all(crosses<0));
fit_errors=sqrt(sum((project(H,corners)-physical).^2,2));
held_expected=[50,50;25,25;75,25];held_mapped=project(H,data.held_out_MEF_px);
held_errors=sqrt(sum((held_mapped-held_expected).^2,2));
check('corner interpolation residual small only numerically',max(fit_errors)<1e-7);
check('MEF independent points within pre-existing3mm diagnostic',max(held_errors)<=3);
% 非仿射透视合成数据；验证算法及圆心应在毫米面拟合的处理顺序。
test_plane=[0,0;100,0;100,100;0,100];test_px=[500,300;1300,100;1550,800;800,950];
T=homography4(test_plane,test_px);test_H=homography4(test_px,test_plane);
test_points=[33,44;80,20;10,90];
check('synthetic perspective independent controls recover',max_norm(project(test_H,project(T,test_points))-test_points)<1e-8);
angle=linspace(0,2*pi,361)';curve=[30,40]+25*[cos(angle),sin(angle)];
[c,r,~]=fit_circle(project(test_H,project(T,curve)));
check('synthetic projected circle recovered in plane',norm(c-[30,40])<1e-8 && abs(r-25)<1e-8);

min_occupied_degrees=150;max_circle_rms=3;
circles=struct('id',{},'observed_points',{},'center_plane_mm',{},'radius_mm',{}, ...
 'center_image_px',{},'boundary_rms_mm',{},'angular_span_deg',{},'occupied_10deg_bins',{}, ...
 'occupied_degrees',{},'diagnostic_circle_check_passed',{},'outside_paper_coverage',{});
for k=1:numel(data.circles)
 px=data.circles(k).boundary_image_px;
 q=project(H,px);[center,radius,residual]=fit_circle(q);
 a=mod(atan2d(q(:,2)-center(2),q(:,1)-center(1)),360);
 sorted=sort(a);gaps=diff([sorted;sorted(1)+360]);span=360-max(gaps);
 bins=unique(floor(a/10));occupied=numel(bins)*10;
 in_paper=inpolygon(px(:,1),px(:,2),corners(:,1),corners(:,2));
 circles(k)=struct('id',data.circles(k).id,'observed_points',size(px,1), ...
  'center_plane_mm',center,'radius_mm',radius,'center_image_px',project(inv(H),center), ...
  'boundary_rms_mm',sqrt(mean(residual.^2)),'angular_span_deg',span, ...
  'occupied_10deg_bins',numel(bins),'occupied_degrees',occupied, ...
  'diagnostic_circle_check_passed',occupied>=min_occupied_degrees && sqrt(mean(residual.^2))<=max_circle_rms, ...
  'outside_paper_coverage',any(~in_paper));
end
check('full visible left marker passes declared arc gate',circles(1).diagnostic_circle_check_passed);
% 右上圈是否通过由数据确定，不把它强制设为合格。
rim=project(H,data.rim_boundary_image_px);keep=true(size(rim,1),1);
for iteration=1:5
 [center,radius,~]=fit_circle(rim(keep,:));
 residual=sqrt(sum((rim-center).^2,2))-radius;
 med=median(residual(keep));mad=median(abs(residual(keep)-med));
 keep=abs(residual-med)<=max(1.5,3*1.4826*mad);
end
[center,radius,residual]=fit_circle(rim(keep,:));
rim_report=struct('center_plane_mm',center,'center_image_px',project(inv(H),center), ...
 'radius_mm',radius,'rms_mm',sqrt(mean(residual.^2)),'retained_points',sum(keep), ...
 'total_points',size(rim,1),'radius_difference_from_nominal150_mm',radius-150, ...
 'planning_radius_minimum_mm',max(150,radius),'physical_diameter_measured',false);
check('visible outer rim has plausible scale, diagnostic only',radius>130 && radius<170 && rim_report.rms_mm<=3);

% 两张图中同一张纸的对应关系只作场景注册假设；固定点匹配不参与拟合。
new_to_previous=homography4(corners,data.previous_corners_ABCD_px);
registration=struct('previous_px',{},'current_px',{},'correlation',{},'registration_error_px',{},'accepted_match',{});
for k=1:numel(data.independent_static_matches)
 m=data.independent_static_matches(k);
 err=norm(project(new_to_previous,m.current_px(:)')-m.previous_px(:)');
 registration(k)=struct('previous_px',m.previous_px,'current_px',m.current_px,'correlation',m.correlation, ...
  'registration_error_px',err,'accepted_match',m.correlation>=.8);
end
accepted=[registration.accepted_match];
registration_accepted=sum(accepted)>=3;
if registration_accepted,registration_accepted=all([registration(accepted).registration_error_px]<=3);end
old_M=project(new_to_previous,data.held_out_MEF_px(1,:));
paper_M_delta=data.held_out_MEF_px(1,:)-old_M;
% 当前画面中心映射只用于盘面诊断；缺乏关节参考不生成车体纠偏/抓取。
ref_px=data.reference_pixel_assumption(:)';ref_plane=project(H,ref_px);
v=data.extension_pixel_direction_assumption(:)';v=v/norm(v);
forward=project(H,ref_px+v)-ref_plane;forward=forward/norm(forward);
right=[-forward(2),forward(1)];
if dot(right,project(H,ref_px+[1,0])-ref_plane)<0,right=-right;end
relative=zeros(numel(circles),2);
for k=1:numel(circles),relative(k,:)=(circles(k).center_plane_mm-ref_plane)*[right',forward'];end
rim_relative=(center-ref_plane)*[right',forward'];
ref_inside=inpolygon(ref_px(1),ref_px(2),corners(:,1),corners(:,2));
check('no full physical calibration claimed',~reference.field_calibration_accepted && ~reference.motion_enabled);

report=struct;report.engine='local MATLAB';report.version=version;report.release=version('-release');
report.time_asia_shanghai=char(datetime('now','TimeZone','Asia/Shanghai','Format','yyyy-MM-dd HH:mm:ss Z'));
report.image_sha256=data.image_sha256;report.image_size=[1920,1080];
report.calibration_id=['source-paper-current220715-' data.image_sha256(1:12) '-diagnostic'];
report.coordinate_frame='paper_local_A_AB_AD';report.homography_image_to_paper=H;
report.corner_fit_errors_mm=fit_errors';report.heldout_MEF_errors_mm=held_errors';
report.heldout_rms_mm=sqrt(mean(held_errors.^2));report.heldout_max_mm=max(held_errors);
report.heldout_point_selection_uncertainty_px=3;report.paper_local_check_passed=max(held_errors)<=3;
report.circle_diagnostics=circles;report.circle_min_occupied_degrees=min_occupied_degrees;
report.all_three_marker_diagnostics_passed=all([circles.diagnostic_circle_check_passed]);
report.rim= rim_report;report.new_image_to_previous_paper_registration=new_to_previous;
report.independent_registration_matches=registration;report.registration_independently_accepted=registration_accepted;
report.paper_M_displacement_image_px=paper_M_delta;
report.displacement_not_assigned_to_chassis_motion=true;
report.image_center_plane_mm=ref_plane;report.image_center_inside_paper_hull=ref_inside;
report.diagnostic_marker_offsets_image_right_extension_forward_mm=relative;
report.diagnostic_rim_center_offset_image_right_extension_forward_mm=rim_relative;
report.input_reference_snapshot=reference;
report.checks=checks;report.check_count=numel(checks);report.failure_count=sum(~[checks.passed]);
report.processing_passed=all([checks.passed]);report.reboot_joint_reference_restored=false;
report.field_calibration_accepted=false;report.operational_extrapolation_enabled=false;
report.full_motion_validation_passed=false;report.hardware_operated=false;report.motion_allowed=false;
report.pending={'paper-covered right marker observation','current pose/encoder reference and TCP binding', ...
 'absolute mm/remaining distortion and full working-area validation','lift mm calibration', ...
 'all-three single-stop geometry rebuilt from current view before control implementation'};
write_json(fullfile(out,'matlab-result.json'),report);save(fullfile(out,'matlab-result.mat'),'report','data','H','circles','rim');

im=imread(fullfile(root,data.image_path));fig=figure('Visible','off','Color','w','Position',[50,50,1300,730]);
image(im);axis image;hold on;
plot([corners(:,1);corners(1,1)],[corners(:,2);corners(1,2)],'g-','LineWidth',1.5);
for k=1:4,text(corners(k,1)+8,corners(k,2),char('A'+k-1),'Color','green','FontSize',13,'FontWeight','bold');end
plot(ref_px(1),ref_px(2),'r+','MarkerSize',14,'LineWidth',2);
plot(rim_report.center_image_px(1),rim_report.center_image_px(2),'y+','MarkerSize',14,'LineWidth',2);
for k=1:numel(circles)
 p=circles(k).center_image_px;
 if circles(k).diagnostic_circle_check_passed,col='green';else,col='red';end
 plot(p(1),p(2),'+','Color',col,'MarkerSize',13,'LineWidth',2);
 text(p(1)-80,max(20,p(2)-20),sprintf('%s: %d deg observed bins',circles(k).id,circles(k).occupied_degrees),'Color',col,'FontSize',10);
end
title({'Current original: paper fit and observed arcs only','No motion approval; paper-covered marker not restored from old images'});
print(fig,fullfile(out,'current-diagnostic.png'),'-dpng','-r140');close(fig);
fprintf('MATLAB current220715 checks=%d failures=%d heldout RMS=%.3f max=%.3fmm; all3=%d motion=0\n', ...
 report.check_count,report.failure_count,report.heldout_rms_mm,report.heldout_max_mm,report.all_three_marker_diagnostics_passed);
for k=1:numel(circles),c=circles(k);fprintf('%s: RMS=%.3fmm occupied=%ddeg span=%.1fdeg accepted=%d center_px=[%.1f,%.1f]\n', ...
 c.id,c.boundary_rms_mm,c.occupied_degrees,c.angular_span_deg,c.diagnostic_circle_check_passed,c.center_image_px);end
fprintf('Rim radius=%.3fmm center_px=[%.1f,%.1f], paperM displacement=[%.1f,%.1f]px; registrationAccepted=%d\n', ...
 rim_report.radius_mm,rim_report.center_image_px,paper_M_delta,registration_accepted);
assert(report.processing_passed,'New-image diagnostics failed; do not implement or execute motion');

 function check(name,passed)
  checks(end+1)=struct('name',name,'passed',logical(passed)); %#ok<AGROW>
 end
end

function H=homography4(p,q)
assert(isequal(size(p),[4,2]) && isequal(size(q),[4,2]) && all(isfinite([p(:);q(:)])));
[p,Tp]=normalize(p);[q,Tq]=normalize(q);A=zeros(8,8);b=zeros(8,1);
for k=1:4
 u=p(k,1);v=p(k,2);x=q(k,1);y=q(k,2);
 A(2*k-1,:)=[u,v,1,0,0,0,-x*u,-x*v];b(2*k-1)=x;
 A(2*k,:)=[0,0,0,u,v,1,-y*u,-y*v];b(2*k)=y;
end
assert(rcond(A)>1e-12);h=A\b;H=Tq\[h(1:3)';h(4:6)';h(7:8)',1]*Tp;H=H/H(3,3);
end
function [p,T]=normalize(p)
c=mean(p,1);scale=sqrt(2)/mean(sqrt(sum((p-c).^2,2)));assert(isfinite(scale) && scale>0);
T=[scale,0,-scale*c(1);0,scale,-scale*c(2);0,0,1];p=project(T,p);
end
function q=project(H,p)
v=[p,ones(size(p,1),1)]*H';assert(all(isfinite(v(:))) && all(abs(v(:,3))>1e-10));q=v(:,1:2)./v(:,3);
end
function [center,radius,residual]=fit_circle(p)
assert(size(p,1)>=12);o=mean(p,1);v=p-o;A=[2*v,ones(size(v,1),1)];assert(rank(A)==3);
c=A\sum(v.^2,2);center=c(1:2)'+o;radius=sqrt(c(3)+sum(c(1:2).^2));residual=sqrt(sum((p-center).^2,2))-radius;
end
function d=max_norm(p)
d=max(sqrt(sum(p.^2,2)));
end
function value=read_json(path)
fid=fopen(path,'r','n','UTF-8');assert(fid~=-1);clean=onCleanup(@()fclose(fid)); %#ok<NASGU>
value=jsondecode(fscanf(fid,'%c'));
end
function write_json(path,value)
fid=fopen(path,'w','n','UTF-8');assert(fid~=-1);clean=onCleanup(@()fclose(fid)); %#ok<NASGU>
fprintf(fid,'%s',jsonencode(value));
end
