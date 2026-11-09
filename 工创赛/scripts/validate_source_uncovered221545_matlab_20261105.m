function validate_source_uncovered221545_matlab_20261105
% 去纸新原图：一致性检查、三圈补全、同一停车点几何重新计算。
root=fileparts(fileparts(mfilename('fullpath')));
folder='evidence/integration/2026-11-05/source-markers/current-221545-a3467c481720';
out=fullfile(root,folder);data=read_json(fullfile(out,'measurement-input.json'));
paper=read_json(fullfile(root,'evidence','integration','2026-11-05','source-markers', ...
 'current-220715-a5702160ff7f','matlab-result.json'));
ref=read_json(fullfile(root,'calibration','source_tcp_center_reference_20261105.json'));
assert(strcmp(data.paper_image_sha256,paper.image_sha256));
assert(paper.paper_local_check_passed && isequal(data.image_size(:)',[1920,1080]));
H=paper.homography_image_to_paper;
checks=struct('name',{},'passed',{});
matches=data.fixed_surface_matches;accepted=[matches.correlation]>=.8;
p=zeros(sum(accepted),2);shifts=p;j=0;
for k=find(accepted)
 j=j+1;p(j,:)=matches(k).previous_px(:)';
 shifts(j,:)=matches(k).current_px(:)'-matches(k).previous_px(:)';
end
alignment_ok=size(p,1)>=3 && rank(p-mean(p,1))==2 && max_norm(shifts)<=1;
check('same-view fixed features agree after paper removal',alignment_ok);
check('new image contains no target positions copied from old scene',all([data.circles.observed_points]>=12));
centers=zeros(3,2);
circles=struct('id',{},'center_plane_mm',{},'center_image_px',{},'radius_mm',{}, ...
 'rms_mm',{},'observed_bins_deg',{},'angular_span_deg',{},'diagnostic_passed',{});
for k=1:3
 q=project(H,data.circles(k).boundary_image_px);[c,r,residual]=fit_circle(q);centers(k,:)=c;
 a=sort(mod(atan2d(q(:,2)-c(2),q(:,1)-c(1)),360));span=360-max(diff([a;a(1)+360]));
 occupied=10*numel(unique(floor(a/10)));
 circles(k)=struct('id',data.circles(k).id,'center_plane_mm',c,'center_image_px',project(inv(H),c), ...
  'radius_mm',r,'rms_mm',sqrt(mean(residual.^2)),'observed_bins_deg',occupied,'angular_span_deg',span, ...
  'diagnostic_passed',occupied>=150 && sqrt(mean(residual.^2))<=3);
 check(['visible circle ' data.circles(k).id],circles(k).diagnostic_passed);
end
q=project(H,data.rim_boundary_image_px);mask=true(size(q,1),1);
for iteration=1:5
 [center,radius,~]=fit_circle(q(mask,:));residual=sqrt(sum((q-center).^2,2))-radius;
 med=median(residual(mask));mad=median(abs(residual(mask)-med));mask=abs(residual-med)<=max(1.5,3*1.4826*mad);
end
[disk_center,disk_radius,residual]=fit_circle(q(mask,:));
rim=struct('center_plane_mm',disk_center,'center_image_px',project(inv(H),disk_center), ...
 'radius_mm',disk_radius,'rms_mm',sqrt(mean(residual.^2)),'planning_radius_mm',max(150,disk_radius));
check('outer rim sensible without changing official nominal radius',disk_radius>130 && disk_radius<170 && rim.rms_mm<=3 && rim.planning_radius_mm>=150);

tcp_px=ref.reference_image_px(:)';tcp=project(H,tcp_px);
direction=ref.extension_image_direction(:)';direction=direction/norm(direction);
forward=project(H,tcp_px+direction)-tcp;forward=forward/norm(forward);right=[-forward(2),forward(1)];
if dot(right,project(H,tcp_px+[1,0])-tcp)<0,right=-right;end
offsets=(centers-tcp)*[right',forward'];
rim_offset=(disk_center-tcp)*[right',forward'];
L=ref.mechanics.current_rotation_pivot_to_tcp_mm;e0=ref.mechanics.source_prepare_nominal_extension_mm;
maximum=ref.mechanics.extension_max_from_home_mm;zero_radius=L-e0;
b=ref.scene_measurements.chassis_bounds_from_dm_axis_mm;
targets=[L+offsets(:,2),-offsets(:,1)];disk_body=[L+rim_offset(2),-rim_offset(1)];
current_gap=rectangle_gap(disk_body,b,rim.planning_radius_mm);
initial_ext=hypot(targets(:,1),targets(:,2))-zero_radius;
initial_angles=mod(180+atan2d(targets(:,2),targets(:,1)),360);
% 三个点都处在同一终点；物理尺寸仅沿用用户尺量作为假设，不恢复运行原点。
rmax=zero_radius+maximum;lo=max(targets-rmax,[],1);hi=min(targets+rmax,[],1);
[x,y]=meshgrid(floor(lo(1)):1:ceil(hi(1)),floor(lo(2)):1:ceil(hi(2)));positions=[x(:),y(:)];
ext=zeros(size(positions,1),3);angles=ext;
for k=1:3
 delta=targets(k,:)-positions;ext(:,k)=hypot(delta(:,1),delta(:,2))-zero_radius;
 angles(:,k)=mod(180+atan2d(delta(:,2),delta(:,1)),360);
end
gaps=rectangle_gap(disk_body-positions,b,rim.planning_radius_mm);
margins=min([ext,maximum-ext],[],2);
valid=margins>=0 & gaps>=50 & all(angles>=2 & angles<=358,2);
strict=valid & margins>=10;
near=valid & hypot(positions(:,1),positions(:,2))<=120;
candidate=struct;ids=find(near);
if ~isempty(ids)
 [~,order]=sortrows([-margins(near),hypot(positions(near,1),positions(near,2))],[1,2]);i=ids(order(1));
 seed=positions(i,:);[fx,fy]=meshgrid(seed(1)-1:.1:seed(1)+1,seed(2)-1:.1:seed(2)+1);p=[fx(:),fy(:)];
 fe=zeros(size(p,1),3);fa=fe;
 for k=1:3
  delta=targets(k,:)-p;fe(:,k)=hypot(delta(:,1),delta(:,2))-zero_radius;
  fa(:,k)=mod(180+atan2d(delta(:,2),delta(:,1)),360);
 end
 fg=rectangle_gap(disk_body-p,b,rim.planning_radius_mm);fm=min([fe,maximum-fe],[],2);
 good=fm>=0 & fg>=50 & all(fa>=2 & fa<=358,2);fi=find(good);
 [~,order]=sortrows([-fm(good),hypot(p(good,1),p(good,2))],[1,2]);i=fi(order(1));
 shift=p(i,:);path=[];
 for t=linspace(0,1,101),path(end+1)=rectangle_gap(disk_body-t*shift,b,rim.planning_radius_mm);end %#ok<AGROW>
 path_bound=min(path)-norm(shift)/200;
 candidate=struct('chassis_right_forward_shift_mm',shift,'extension_mm',fe(i,:), ...
  'logical_deg',fa(i,:),'minimum_stroke_margin_mm',fm(i),'nominal_endpoint_disk_gap_mm',fg(i), ...
  'path_min_gap_lower_bound_mm',path_bound,'path_gap_nondecreasing_in_samples',all(diff(path)>=-1e-8), ...
  'whole_path_meets50mm',path_bound>=50,'meets_original10mm_stroke_margin',fm(i)>=10, ...
  'is_diagnostic_not_command',true);
 % 用本轮量值与声明误差重验，目标/停车/尺量误差不冒充实测精度。
 definitions=[0,3,0;1,3,0;1,3,1;1,2,1];sensitivity=struct('ruler_error_mm',{},'target_axis_error_mm',{}, ...
  'base_axis_error_mm',{},'sets',{},'rejected_sets',{},'minimum_stroke_margin_mm',{});
 for scenario=1:size(definitions,1)
  rr=unique([-definitions(scenario,1),0,definitions(scenario,1)]);
  pp=unique([-definitions(scenario,2),0,definitions(scenario,2)]);
  bb=unique([-definitions(scenario,3),0,definitions(scenario,3)]);
  sets=0;rejected=0;worst=Inf;
  for length=[225,235,245]
   for eerr=rr
    for extra=rr
     for dx=pp
      for dy=pp
       for bx=bb
        for by=bb
         pt=[length+offsets(:,2),-offsets(:,1)]+[dx,dy]-(shift+[bx,by]);
         ee=hypot(pt(:,1),pt(:,2))-(length-(70+eerr));maxE=70+eerr+100+extra;
         sets=sets+1;rejected=rejected+~all(ee>=0 & ee<=maxE);worst=min(worst,min([ee;maxE-ee]));
        end
       end
      end
     end
    end
   end
  end
  sensitivity(scenario)=struct('ruler_error_mm',definitions(scenario,1),'target_axis_error_mm',definitions(scenario,2), ...
   'base_axis_error_mm',definitions(scenario,3),'sets',sets,'rejected_sets',rejected,'minimum_stroke_margin_mm',worst);
 end
 candidate.declared_sensitivity=sensitivity;
 % 正逆解只验证求解计算，不验证场景/TCP/机械尺寸。
 reconstructed=[(zero_radius+candidate.extension_mm(:)).*cosd(candidate.logical_deg(:)-180), ...
  (zero_radius+candidate.extension_mm(:)).*sind(candidate.logical_deg(:)-180)]+shift;
 check('candidate inverse/forward consistency',max_norm(reconstructed-targets)<1e-8);
end
check('no current motion permission inferred from camera alignment',~ref.motion_enabled && ~ref.field_calibration_accepted);

report=struct;report.engine='local MATLAB';report.version=version;report.release=version('-release');
report.image_sha256=data.image_sha256;report.paper_image_sha256=data.paper_image_sha256;
report.calibration_id=paper.calibration_id;report.homography_image_to_paper=H;
report.fixed_feature_count=numel(matches);report.fixed_feature_accepted_count=sum(accepted);
report.fixed_feature_max_integer_shift_px=max_norm(shifts);report.same_image_pose_check_passed=alignment_ok;
report.all_three_marker_diagnostics_passed=all([circles.diagnostic_passed]);report.circles=circles;report.rim=rim;
report.reference=ref;report.tcp_image_assumption_px=tcp_px;report.marker_offsets_image_right_extension_forward_mm=offsets;
report.body_kinematic_assumptions=struct('current_pivot_to_tcp_mm',L,'current_extension_mm',e0,'zero_radius_mm',zero_radius, ...
 'current_logical_angle_assumed_deg',180,'pose_encoder_reference_verified',false);
report.current_pose_nominal_extension_mm=initial_ext';report.current_pose_nominal_angle_deg=initial_angles';
report.current_model_disk_gap_mm=current_gap;report.current_disk_gap_physically_verified=false;
report.current_user_measured_outermost_disk_gap_mm=ref.scene_measurements.current_chassis_to_disk_min_gap_mm;
report.current_gap_model_minus_user_mm=current_gap-report.current_user_measured_outermost_disk_gap_mm;
report.current_user_gap_meets50mm=report.current_user_measured_outermost_disk_gap_mm>=50;
report.historical_gap70_outermost_reference_valid=false;
report.search=struct('grid_step_mm',1,'pose_count',size(positions,1),'same_heading_feasible_endpoints',sum(valid), ...
 'same_heading10mm_stroke_margin_endpoints',sum(strict),'search_bound_from_maximum_radius',true);
report.diagnostic_single_stop_candidate=candidate;
report.checks=checks;report.check_count=numel(checks);report.failure_count=sum(~[checks.passed]);
report.processing_passed=all([checks.passed]);report.historical_shift_13_4_14_9_reused=false;
report.joint_reference_restored=false;report.gray_lane_verified=false;report.paper_external_coverage_approved=false;
report.full_motion_validation_passed=false;report.hardware_operated=false;report.motion_allowed=false;
report.pending={'current real chassis-disk gap, old70mm invalid after movement', ...
 'actual kinematic reference after reboot','independent work-area/TCP/lens mm accuracy', ...
 'bounded lift calibration and user50mm-height loaded clearance','complete path/lane/actuator and failure-exit validation'};
write_json(fullfile(out,'matlab-result.json'),report);save(fullfile(out,'matlab-result.mat'),'report','data','H','offsets','targets','disk_body');

im=imread(fullfile(root,data.image_path));fig=figure('Visible','off','Color','w','Position',[50,50,1250,730]);
image(im);axis image;hold on;
for k=1:3
 c=circles(k).center_image_px;
 plot(c(1),c(2),'g+','MarkerSize',14,'LineWidth',2);
 text(c(1)-60,max(20,c(2)-20),sprintf('%s: %d deg observed bins',circles(k).id,circles(k).observed_bins_deg),'Color','green','FontSize',11);
end
plot(960,540,'r+','MarkerSize',14,'LineWidth',2);
plot(rim.center_image_px(1),rim.center_image_px(2),'y+','MarkerSize',14,'LineWidth',2);
title({'Paper removed; observed three plane markers','Image geometry only; not live material IDs or motion approval'});
print(fig,fullfile(out,'current-three-markers.png'),'-dpng','-r130');close(fig);
fprintf('MATLAB uncovered221545 checks=%d failures=%d acceptedFixed=%d maxShift=%.2fpx all3=%d currentGapConditional=%.3fmm motion=0\n', ...
 report.check_count,report.failure_count,sum(accepted),max_norm(shifts),report.all_three_marker_diagnostics_passed,current_gap);
fprintf('Current assumed extensions/mm: ');disp(initial_ext');
fprintf('Current marker offsets local/right-forward mm:\n');disp(offsets);
if isfield(candidate,'extension_mm'),disp(candidate);end
assert(report.processing_passed,'Current uncovered-image checks failed; no implementation or execution');

 function check(name,passed)
  checks(end+1)=struct('name',name,'passed',logical(passed)); %#ok<AGROW>
 end
end
function q=project(H,p)
v=[p,ones(size(p,1),1)]*H';assert(all(isfinite(v(:))) && all(abs(v(:,3))>1e-10));q=v(:,1:2)./v(:,3);
end
function [c,r,res]=fit_circle(p)
assert(size(p,1)>=12);o=mean(p,1);v=p-o;A=[2*v,ones(size(v,1),1)];assert(rank(A)==3);
k=A\sum(v.^2,2);c=k(1:2)'+o;r=sqrt(k(3)+sum(k(1:2).^2));res=sqrt(sum((p-c).^2,2))-r;
end
function gap=rectangle_gap(c,b,r)
dx=max(max(-b.left-c(:,1),c(:,1)-b.right),0);dy=max(max(-b.rear-c(:,2),c(:,2)-b.front),0);gap=hypot(dx,dy)-r;
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
