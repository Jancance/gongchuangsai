function validate_source_reach_matlab_20261105
% MATLAB先验证整合8静态几何；不连接串口/调试器，不生成执行命令。
% 同轴平面旋转+伸缩是假设模型；未包含升降/碰撞/供电/真实调度。
root = fileparts(fileparts(mfilename('fullpath')));
base = fullfile(root, 'evidence', 'integration', '2026-11-05', 'source-markers');
in = fullfile(base, 'paper100-transfer', 'continuation');
out = fullfile(in, 'matlab');
if ~exist(out, 'dir'), mkdir(out); end
reference = read_json(fullfile(root, 'calibration', 'source_tcp_center_reference_20261105.json'));
paper = read_json(fullfile(root, 'calibration', 'source_paper_100mm_20261105.json'));
previous = read_json(fullfile(in, 'reference-calibration.json'));
transfer = read_json(fullfile(base, 'paper100-transfer', 'extracted-full-paper-views.json'));
boundaries = read_json(fullfile(in, 'circle-boundary-inputs.json'));
assert(strcmp(reference.calibration_id, paper.calibration_id));
assert(strcmp(previous.calibration_id, paper.calibration_id));
checks = struct('name', {}, 'passed', {});

% 独立用四角线性方程求H，不依赖Python/OpenCV拟合函数。
corners = zeros(4,2);
for k=1:4, corners(k,:) = paper.markers(k).image_px(:)'; end
known = [0,0; paper.width_mm,0; paper.width_mm,paper.height_mm; 0,paper.height_mm];
A=zeros(8,8); b=zeros(8,1);
for k=1:4
    u=corners(k,1); v=corners(k,2); x=known(k,1); y=known(k,2);
    A(2*k-1,:)=[u,v,1,0,0,0,-x*u,-x*v]; b(2*k-1)=x;
    A(2*k,:)=[0,0,0,u,v,1,-y*u,-y*v]; b(2*k)=y;
end
h=A\b; H=[h(1:3)';h(4:6)';h(7:8)',1];
check('independent H recovers measured corners', max_row_norm(map(H,corners)-known)<1e-7);
held = zeros(3,2);
for k=1:3, held(k,:)=paper.markers(k+4).image_px(:)'; end
w=paper.width_mm; d=paper.height_mm;
held_expected=[w/2,d/2;w/4,d/4;3*w/4,d/4];
held_errors=sqrt(sum((map(H,held)-held_expected).^2,2));
check('M E F holdout within pre-existing 3mm diagnostic limit', max(held_errors)<=paper.limits.validation_max_mm);
lengths=struct('label',{},'side_mm',{},'diagonal_mm',{},'max_error_mm',{});
for k=1:numel(transfer)
    q=map(H,transfer(k).corners_ABCD_px);
    side=sqrt(sum((q([2,3,4,1],:)-q).^2,2));
    diagonal=sqrt(sum((q([3,4],:)-q([1,2],:)).^2,2));
    errors=[side-[w;d;w;d];diagonal-hypot(w,d)];
    check(['transferred rectangle ' transfer(k).label], max(abs(errors))<=paper.limits.validation_max_mm);
    lengths(k)=struct('label',transfer(k).label,'side_mm',side','diagonal_mm',diagonal','max_error_mm',max(abs(errors)));
end

% 边界先变换到物理平面，再独立求圆心。原图椭圆中心不参与计算。
circles=struct('target_id',{},'view',{},'center_plane_mm',{},'radius_mm',{},'boundary_rms_mm',{});
for k=1:numel(boundaries)
    px=boundaries(k).boundary_image_px;
    p=map(H,px);
    check(sprintf('H independent cross-check boundary %d',k), max_row_norm(p-map(previous.homography_image_to_plane,px))<0.001);
    [center,radius,residual]=circle_fit(p);
    check(sprintf('circle boundary diagnostic %d',k), sqrt(mean(residual.^2))<paper.limits.validation_max_mm);
    circles(k)=struct('target_id',boundaries(k).target_id,'view',boundaries(k).view, ...
        'center_plane_mm',center,'radius_mm',radius,'boundary_rms_mm',sqrt(mean(residual.^2)));
end
ids={'upper-left','upper-right','lower'}; physical=zeros(3,2);
for k=1:3
    j=find(strcmp({circles.target_id},ids{k}),1);
    physical(k,:)=circles(j).center_plane_mm;
end
tcp=map(H,reference.reference_image_px(:)');
step=reference.extension_image_direction(:)'; step=step/norm(step);
forward=map(H,reference.reference_image_px(:)'+step)-tcp; forward=forward/norm(forward);
right=[-forward(2),forward(1)];
if dot(right,map(H,reference.reference_image_px(:)'+[1,0])-tcp)<0, right=-right; end
offsets=(physical-tcp)*[right',forward'];
L=reference.mechanics.current_rotation_pivot_to_tcp_mm;
e=reference.mechanics.source_prepare_nominal_extension_mm;
maximum=reference.mechanics.extension_max_from_home_mm;
initial=reach(offsets,L,e,maximum,[0,0]);
candidate=reach(offsets,L,e,maximum,[0,-20]);
check('initial near point is rejected for negative extension', ~initial.in_range(3) && initial.extension_mm(3)<0);
check('20mm reverse shift puts all three points in stroke', all(candidate.in_range));
check('nominal candidate leaves 10mm extension margin', min(candidate.extension_mm)>=10 && max(candidate.extension_mm)<=maximum-10);
check('candidate inverse/forward reconstruction', max_row_norm(candidate.reconstructed_mm-offsets)<1e-8);

% 声明的敏感性扰动网格，不代表已测量的不确定度或动力学证明。
sensitivity_count=0; all_in_range=true; minimum=Inf; largest=-Inf;
for variedL=[225,235,245]
 for variedE=[65,70,75]
  for variedMax=[190,205]
   for shift=[17,20,23]
    for dx=[-3,0,3]
     for dy=[-3,0,3]
      result=reach(offsets+[dx,dy],variedL,variedE,variedMax,[0,-shift]);
      sensitivity_count=sensitivity_count+1;
      all_in_range=all_in_range && all(result.in_range);
      minimum=min(minimum,min(result.extension_mm)); largest=max(largest,max(result.extension_mm));
     end
    end
   end
  end
 end
end
check('declared sensitivity grid all three points stay in stroke',all_in_range);
check('reject nonfinite target', rejects(@() reach([NaN,0],L,e,maximum,[0,0])));
check('reject impossible baseline length', rejects(@() reach(offsets,50,e,maximum,[0,0])));
check('reject max stroke below current extension', rejects(@() reach(offsets,L,e,50,[0,0])));
too_far=reach([500,0],L,e,maximum,[0,0]);
check('out-of-stroke target cannot pass',~all(too_far.in_range));

% 行程内外边界和镜像不变量；完整固件失败出口仍未建模。
inner=reach([0,-e],L,e,maximum,[0,0]);
outer=reach([0,maximum-e],L,e,maximum,[0,0]);
below=reach([0,-e-0.01],L,e,maximum,[0,0]);
above=reach([0,maximum-e+0.01],L,e,maximum,[0,0]);
mirror=reach([-50,30;50,30],L,e,maximum,[0,0]);
check('exact inner stroke boundary maps to zero',inner.in_range && abs(inner.extension_mm)<1e-8);
check('exact outer stroke boundary maps to max',outer.in_range && abs(outer.extension_mm-maximum)<1e-8);
check('point inside inner radius is rejected',~below.in_range);
check('point outside outer radius is rejected',~above.in_range);
check('mirrored targets have equal stroke and opposite angles', ...
    abs(diff(mirror.extension_mm))<1e-8 && abs(sum(mirror.clockwise_delta_deg))<1e-8);

report=struct;
report.engine='local MATLAB'; report.version=version; report.release=version('-release');
report.matlab_root=matlabroot; report.platform=computer;
report.time_asia_shanghai=char(datetime('now','TimeZone','Asia/Shanghai','Format','yyyy-MM-dd HH:mm:ss Z'));
report.calibration_id=paper.calibration_id; report.independent_H=H;
report.holdout_errors_mm=held_errors'; report.transfer=lengths; report.circle_fits=circles;
report.target_ids=ids; report.tcp_reference_plane_mm=tcp; report.offsets_right_forward_mm=offsets;
report.model_assumption='coaxial planar rotation and radial extension; current pivot at (0,-235) relative to TCP';
report.initial=initial; report.candidate=candidate;
report.sensitivity=struct('sets',sensitivity_count,'targets_checked',3*sensitivity_count, ...
    'L_mm',[225,235,245],'current_extension_mm',[65,70,75],'max_extension_mm',[190,205], ...
    'reverse_shift_mm',[17,20,23],'target_axis_perturbation_mm',[-3,0,3], ...
    'minimum_extension_mm',minimum,'maximum_extension_mm',largest,'all_in_range',all_in_range, ...
    'measured_uncertainty_bounds',false);
report.checks=checks; report.check_count=numel(checks); report.failure_count=sum(~[checks.passed]);
report.geometry_passed=all([checks.passed]);
report.full_motion_validation_passed=false; report.hardware_operated=false; report.motion_allowed=false;
report.operational_extrapolation_enabled=false;
report.pending={'physical TCP/feature plane/absolute position', 'positive DM rotation sign and camera-to-chassis directions',
    'current chassis envelope/lane/50mm rim clearance', 'Z lift height, tool/carrier/cable swept volume',
    'concurrent real-source firmware schedules and fault exits', 'fresh observations after motion and automatic pick/carrier transactions'};
write_json(fullfile(out,'result.json'),report);
save(fullfile(out,'simulation.mat'),'report','H','physical','offsets','reference');

fig=figure('Visible','off','Color','w','Position',[50,50,1250,580]);
for panel=1:2
    subplot(1,2,panel); hold on; grid on; axis equal;
    if panel==1, state=initial; shift=[0,0]; text_title='Current chassis pose';
    else, state=candidate; shift=[0,-20]; text_title='Candidate: reverse shift 20 mm'; end
    pivot=[shift(1),shift(2)-L]; t=linspace(0,2*pi,800);
    minimum_radius=L-e; maximum_radius=L-e+maximum;
    plot(pivot(1)+minimum_radius*cos(t),pivot(2)+minimum_radius*sin(t),'k--');
    plot(pivot(1)+maximum_radius*cos(t),pivot(2)+maximum_radius*sin(t),'k--');
    plot(shift(1),shift(2),'kp','MarkerSize',14,'MarkerFaceColor','y');
    quiver(shift(1),shift(2),0,30,0,'k','LineWidth',1.5);
    for k=1:3
        if state.in_range(k), color=[0.1,0.45,0.2]; else, color=[0.8,0.1,0.1]; end
        plot(offsets(k,1),offsets(k,2),'o','Color',color,'MarkerFaceColor',color,'MarkerSize',7);
        text(offsets(k,1)-29,offsets(k,2)+10,sprintf('%s\nE = %.1f mm\nCW = %.2f deg',ids{k},state.extension_mm(k),state.clockwise_delta_deg(k)), ...
            'FontSize',10,'VerticalAlignment','bottom');
    end
    xlim([-160,160]); ylim([-130,150]); xlabel('Right / mm'); ylabel('Extension forward / mm');
    title({text_title,'Geometry diagnostic only; not motion approval'});
end
print(fig,fullfile(out,'reachability.png'),'-dpng','-r140'); close(fig);
fprintf('MATLAB %s: checks=%d, failures=%d; sensitivity=%d sets; geometry=%d; motion_allowed=0\n', ...
    report.release,report.check_count,report.failure_count,sensitivity_count,report.geometry_passed);
disp(table(ids',offsets(:,1),offsets(:,2),initial.extension_mm,candidate.extension_mm,candidate.clockwise_delta_deg,...
    'VariableNames',{'Target','Right_mm','Forward_mm','Initial_extension_mm','Candidate_extension_mm','Clockwise_delta_deg'}));
assert(report.geometry_passed,'MATLAB geometry checks failed; do not implement motion code');

    function check(name,passed)
        checks(end+1)=struct('name',name,'passed',logical(passed)); %#ok<AGROW>
    end
end

function result=reach(offsets,L,e,maximum,shift)
assert(size(offsets,2)==2 && all(isfinite(offsets(:))) && all(isfinite([L,e,maximum,shift])));
assert(L>e && e>=0 && maximum>=e && numel(shift)==2);
relative=offsets-shift+[0,L]; radius=hypot(relative(:,1),relative(:,2));
angle=atan2d(relative(:,1),relative(:,2)); extension=radius-(L-e);
reconstructed=[radius.*sind(angle),radius.*cosd(angle)]-[0,L]+shift;
result=struct('chassis_shift_right_forward_mm',shift,'extension_mm',extension,...
    'clockwise_delta_deg',angle,'in_range',extension>=0 & extension<=maximum,...
    'reconstructed_mm',reconstructed);
end

function mapped=map(H,p)
q=[p,ones(size(p,1),1)]*H'; assert(all(isfinite(q(:))) && all(abs(q(:,3))>1e-10));
mapped=q(:,1:2)./q(:,3);
end

function [center,radius,residual]=circle_fit(p)
origin=mean(p,1); local=p-origin; A=[2*local,ones(size(p,1),1)];
assert(rank(A)==3); c=A\sum(local.^2,2);
center=c(1:2)'+origin; radius=sqrt(c(3)+sum(c(1:2).^2));
residual=sqrt(sum((p-center).^2,2))-radius;
end

function value=max_row_norm(p)
value=max(sqrt(sum(p.^2,2)));
end

function tf=rejects(action)
try, action(); tf=false; catch, tf=true; end
end

function value=read_json(path)
fid=fopen(path,'r','n','UTF-8'); assert(fid~=-1);
cleanup=onCleanup(@() fclose(fid)); %#ok<NASGU>
value=jsondecode(fscanf(fid,'%c'));
end

function write_json(path,value)
fid=fopen(path,'w','n','UTF-8'); assert(fid~=-1);
cleanup=onCleanup(@() fclose(fid)); %#ok<NASGU>
fprintf(fid,'%s',jsonencode(value));
end
