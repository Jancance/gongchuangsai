function validate_source_body_pose_matlab_20261105
% 仅MATLAB几何仿真：270x250真实车体，单停车点含航向搜索。
root=fileparts(fileparts(mfilename('fullpath')));
base=fullfile(root,'evidence','integration','2026-11-05','source-markers','paper100-transfer','continuation');
out=fullfile(base,'body270x250','matlab-body-pose');if ~exist(out,'dir'),mkdir(out);end
ref=read_json(fullfile(root,'calibration','source_tcp_center_reference_20261105.json'));
single=read_json(fullfile(base,'matlab-single-stop-stroke170','result.json'));
old=read_json(fullfile(base,'matlab','result.json'));
b=ref.scene_measurements.chassis_bounds_from_dm_axis_mm;
assert(isequal([b.front,b.rear,b.left,b.right],[137,133,220,30]));
assert(strcmp(ref.workflow.selected,'one_chassis_stop_for_all_three'));
assert(strcmp(ref.reference_id,single.reference.reference_id));
q=old.offsets_right_forward_mm;
L=ref.mechanics.current_rotation_pivot_to_tcp_mm;e=ref.mechanics.source_prepare_nominal_extension_mm;
maxE=ref.mechanics.extension_max_from_home_mm;assert(maxE==170);
targets=[L+q(:,2),-q(:,1)];center=single.rim.center_body_right_forward_from_pivot_mm(:)';
disk_radius=single.planning_disk_radius_mm;
checks=struct('name',{},'passed',{});
% 朝向用车右X/车前Y图上的标准逆时针为正。
check('yaw inverse rotation preserves vector norm',abs(norm(rotate([3,4],-37))-5)<1e-10);
check('90deg body yaw changes relative right into rear',norm(rotate([10,0],-90)-[0,-10])<1e-10);
check('zero-yaw circle rectangle distance',abs(gap(center,b,disk_radius)-single.current_body_model_gap_mm)<1e-9);
check('reported width and length consistent',b.left+b.right==250 && b.front+b.rear==270);

% 由三个最大半径圆相交的包围盒覆盖所有平移终点，避免任意搜索框遗漏盘对面。
rmax=(L-e)+maxE;
lo=max(targets-rmax,[],1);hi=min(targets+rmax,[],1);
[xs,ys]=meshgrid(floor(lo(1)):2:ceil(hi(1)),floor(lo(2)):2:ceil(hi(2)));
shifts=[xs(:),ys(:)];
ranges=zeros(size(shifts,1),3);
for k=1:3,ranges(:,k)=hypot(targets(k,1)-shifts(:,1),targets(k,2)-shifts(:,2))-(L-e);end
scenarios=struct('stroke_margin_mm',{},'min_body_disk_gap_mm',{},'pose_grid_count',{}, ...
 'joint_shift_count',{},'same_heading_endpoint_count',{},'same_heading_max_gap_mm',{}, ...
 'all_yaws_endpoint_count',{},'minimum_travel_candidate',{},'minimum_yaw_candidate',{});
accepted=[];best_balanced=[];
for scenario=1:2
 margin=10;if scenario==2,margin=0;end
 mask=all(ranges>=margin & ranges<=maxE-margin,2);p=shifts(mask,:);ext=ranges(mask,:);
 candidates=[];same_heading_count=0;max_gap_no_yaw=-Inf;
 for yaw=-180:2:178
  relative=rotate(center-p,-yaw);clearances=gap(relative,b,disk_radius);
  angles=zeros(size(p,1),3);
  for k=1:3
   t=rotate(targets(k,:)-p,-yaw);
   angles(:,k)=mod(180+atan2d(t(:,2),t(:,1)),360);
  end
  valid=clearances>=50 & all(angles>=2 & angles<=358,2);
  if yaw==0,same_heading_count=sum(valid);max_gap_no_yaw=max(clearances);end
  indexes=find(valid);
  if ~isempty(indexes)
   candidates=[candidates;[p(indexes,:),repmat(yaw,numel(indexes),1),clearances(indexes), ...
    ext(indexes,:),angles(indexes,:)]]; %#ok<AGROW>
  end
 end
 s=struct('stroke_margin_mm',margin,'min_body_disk_gap_mm',50,'pose_grid_count',numel(xs)*180, ...
  'joint_shift_count',size(p,1),'same_heading_endpoint_count',same_heading_count, ...
  'same_heading_max_gap_mm',max_gap_no_yaw,'all_yaws_endpoint_count',size(candidates,1), ...
  'minimum_travel_candidate',struct,'minimum_yaw_candidate',struct);
 if ~isempty(candidates)
  [~,order]=sortrows([hypot(candidates(:,1),candidates(:,2)),abs(candidates(:,3))],[1,2]);
  s.minimum_travel_candidate=record(candidates(order(1),:));
  [~,order]=sortrows([abs(candidates(:,3)),hypot(candidates(:,1),candidates(:,2))],[1,2]);
  s.minimum_yaw_candidate=record(candidates(order(1),:));
  if scenario==2
   margins=min([candidates(:,5:7),maxE-candidates(:,5:7)],[],2);
   [~,order]=sortrows([-margins,hypot(candidates(:,1),candidates(:,2)),abs(candidates(:,3))],[1,2,3]);
   best_balanced=record(candidates(order(1),:));
   best_balanced.minimum_stroke_margin_mm=margins(order(1));
   % 当前停车点附近的比较候选，避免将绕到盘对面误当很小的现场调整。
   local=hypot(candidates(:,1),candidates(:,2))<=100 & abs(candidates(:,3))<=30;
   local_ids=find(local);
   if ~isempty(local_ids)
    [~,j]=sortrows([-margins(local),hypot(candidates(local,1),candidates(local,2)),abs(candidates(local,3))],[1,2,3]);
    chosen=local_ids(j(1));
    report_local_best=record(candidates(chosen,:));
    report_local_best.minimum_stroke_margin_mm=margins(chosen);
   end
  end
 end
 scenarios(scenario)=s;
 if scenario==1,accepted=candidates;end
end
check('prior same-heading 10mm-margin result reproduced',scenarios(1).same_heading_endpoint_count==0);

% 局部名义候选细化，不将较小余量升级为原10mm要求已通过。
if exist('report_local_best','var')
 seed=report_local_best.chassis_right_forward_mm;
 [fine_x,fine_y]=meshgrid(seed(1)-2:.1:seed(1)+2,seed(2)-2:.1:seed(2)+2);
 fine=[fine_x(:),fine_y(:)];fine_ext=zeros(size(fine,1),3);fine_angles=fine_ext;
 for k=1:3
  delta=targets(k,:)-fine;
  fine_ext(:,k)=hypot(delta(:,1),delta(:,2))-(L-e);
  fine_angles(:,k)=mod(180+atan2d(delta(:,2),delta(:,1)),360);
 end
 fine_gap=gap(center-fine,b,disk_radius);
 fine_margins=min([fine_ext,maxE-fine_ext],[],2);
 valid=fine_gap>=50 & all(fine_angles>=2 & fine_angles<=358,2) & fine_margins>=0;
 ids=find(valid);[~,j]=sortrows([-fine_margins(valid),hypot(fine(valid,1),fine(valid,2))],[1,2]);
 selected=ids(j(1));
 local=record([fine(selected,:),0,fine_gap(selected),fine_ext(selected,:),fine_angles(selected,:)]);
 local.minimum_stroke_margin_mm=fine_margins(selected);
 local.refinement_step_mm=.1;
 local.exact_straight_path_gap_mm=translation_gap(center,b,disk_radius,[0,0],fine(selected,:),0);
 local.meets_original10mm_margin=local.minimum_stroke_margin_mm>=10;
 check('refined local candidate reconstructs all targets',all(local.extension_mm>=0 & local.extension_mm<=maxE));
 check('local straight chassis sweep clears modeled disk by 50mm',local.exact_straight_path_gap_mm>=50);
 shifted=targets-local.chassis_right_forward_mm;
 reconstructed=[((L-e)+local.extension_mm(:)).*cosd(local.logical_deg(:)-180), ...
  ((L-e)+local.extension_mm(:)).*sind(local.logical_deg(:)-180)];
 check('local joint inverse and forward positions agree',max(sqrt(sum((shifted-reconstructed).^2,2)))<1e-8);
 % 声明扰动仅为敏感性，不是已测不确定度边界。
 cases=0;accepted_cases=0;min_ext=Inf;max_ext=-Inf;max_violation=0;
 for variedL=[225,235,245]
  for variedE=[65,70,75]
   for variedMax=[165,170]
    for dx=[-3,0,3]
     for dy=[-3,0,3]
      varied_targets=[variedL+q(:,2),-q(:,1)]+[dx,dy]-local.chassis_right_forward_mm;
      varied_ext=hypot(varied_targets(:,1),varied_targets(:,2))-(variedL-variedE);
      cases=cases+1;accepted_cases=accepted_cases+all(varied_ext>=0 & varied_ext<=variedMax);
      min_ext=min(min_ext,min(varied_ext));max_ext=max(max_ext,max(varied_ext));
      max_violation=max(max_violation,max([0;-varied_ext;varied_ext-variedMax]));
     end
    end
   end
  end
 end
 local.sensitivity=struct('sets',cases,'targets_checked',cases*3,'all_targets_in_stroke_sets',accepted_cases, ...
  'out_of_stroke_sets',cases-accepted_cases,'minimum_extension_mm',min_ext, ...
  'maximum_extension_mm',max_ext,'max_stroke_violation_mm',max_violation, ...
  'current_L_mm',[225,235,245],'current_E_mm',[65,70,75],'maximum_E_mm',[165,170], ...
  'target_axis_perturbation_mm',[-3,0,3],'measured_uncertainty_bounds',false);
 report_local_best=local;
end

% 对有余量的终点候选检查可执行前的几何路径。
% 只模拟先收拢到车体包络内，再绕盘外侧中间点、原地转向、到最终停车点。
% 未有收拢/升降/线缆证据时仍不得据路径结果放行执行。
paths=struct('endpoint',{},'staging_body_right_forward_mm',{},'segment1_min_gap_mm',{}, ...
 'turn_min_gap_bound_mm',{},'segment3_min_gap_mm',{},'path_bound_mm',{});
if ~isempty(accepted)
 [~,order]=sortrows([abs(accepted(:,3)),hypot(accepted(:,1),accepted(:,2))],[1,2]);
 accepted=accepted(order,:);limit=min(1500,size(accepted,1));
 for k=1:limit
  c=accepted(k,:);shift=c(1:2);yaw=c(3);
  % 保持距盘至少当前位置的横向距离，沿车前后绕行，然后朝目标偏移。
  % 左侧安全走廊→盘外前/后绕行→远端回转点→目标；明确避开穿盘直线。
  staging1=[-80,0];
  if shift(2)>=center(2),side=1;else,side=-1;end
  corridor_y=center(2)+side*(disk_radius+hypot(250,270)+80);
  staging2=[-80,corridor_y];
  staging=[shift(1),corridor_y];
  gap1=min([translation_gap(center,b,disk_radius,[0,0],staging1,0), ...
   translation_gap(center,b,disk_radius,staging1,staging2,0), ...
   translation_gap(center,b,disk_radius,staging2,staging,0)]);
  n=max(2,ceil(abs(yaw)/0.5)+1);angles=linspace(0,yaw,n);
  values=zeros(size(angles));
  for j=1:n,values(j)=gap(rotate(center-staging,-angles(j)),b,disk_radius);end
  extent=hypot(max(b.left,b.right),max(b.front,b.rear));
  angle_step=max(abs(diff(angles)))*pi/180;
  gapturn=min(values)-extent*angle_step/2;
  gap3=translation_gap(center,b,disk_radius,staging,shift,yaw);
  worst=min([gap1,gapturn,gap3]);
  if worst>=55
   paths(end+1)=struct('endpoint',record(c),'staging_body_right_forward_mm',staging, ...
    'segment1_min_gap_mm',gap1,'turn_min_gap_bound_mm',gapturn,'segment3_min_gap_mm',gap3, ...
    'path_bound_mm',worst); %#ok<AGROW>
   if numel(paths)>=5,break;end
  end
 end
end
report=struct;report.engine='local MATLAB';report.version=version;report.release=version('-release');
report.time_asia_shanghai=char(datetime('now','TimeZone','Asia/Shanghai','Format','yyyy-MM-dd HH:mm:ss Z'));
report.reference=ref;report.target_ids=old.target_ids;report.targets_initial_body_right_forward_mm=targets;
report.disc_center_initial_body_right_forward_mm=center;report.disc_radius_mm=disk_radius;
report.axis_convention='initial chassis: X right, Y front; yaw positive CCW in top view; physical camera/frame verification pending';
report.search_box_right_mm=[-300,300];report.search_box_front_mm=[-350,350];report.grid_step_mm=2;
report.search_box_right_mm=[floor(lo(1)),ceil(hi(1))];report.search_box_front_mm=[floor(lo(2)),ceil(hi(2))];
report.search_bounds_derived_from_maximum_reach=true;
report.yaw_grid_deg=[-180,178,2];report.scenarios=scenarios;report.path_candidates=paths;
report.maximum_margin_endpoint=best_balanced;
if exist('report_local_best','var'),report.maximum_margin_within100mm_yaw30=report_local_best;end
report.current_gap_model_mm=single.current_body_model_gap_mm;report.current_gap_user_mm=70;
report.gap_disagreement_mm=single.current_gap_model_error_mm;
report.checks=checks;report.algorithm_checks=numel(checks);report.algorithm_failures=sum(~[checks.passed]);
report.geometry_endpoint_found=~isempty(accepted);report.geometry_path_found=~isempty(paths);
report.geometry_endpoint_found_requires10mm_margin=true;
report.full_body_footprint_verified_on_gray_lane=false;
report.full_motion_validation_passed=false;report.motion_allowed=false;report.hardware_operated=false;
report.scope='single chassis stop endpoint/path body rectangle geometry; not live pose, gray lane, deployed arm/cable swept-volume or dynamic validation';
write_json(fullfile(out,'result.json'),report);
save(fullfile(out,'simulation.mat'),'report','accepted','targets','center','ref');
fprintf('MATLAB body pose: checks=%d failures=%d; same yaw endpoint count margins10/0 = %d/%d; all yaw endpoints=%d; paths=%d; motion=0\n', ...
 report.algorithm_checks,report.algorithm_failures,scenarios(1).same_heading_endpoint_count, ...
 scenarios(2).same_heading_endpoint_count,size(accepted,1),numel(paths));
for k=1:2
 fprintf('margin%d same_yaw_max_gap=%.3f endpoints_all_yaw=%d\n',scenarios(k).stroke_margin_mm,scenarios(k).same_heading_max_gap_mm,scenarios(k).all_yaws_endpoint_count);
 if isfield(scenarios(k).minimum_travel_candidate,'yaw_deg'),disp(scenarios(k).minimum_travel_candidate);end
end
if ~isempty(paths),disp(paths(1));end

fig=figure('Visible','off','Color','w','Position',[50,50,1250,650]);
subplot(1,2,1);draw_scene([0,0],0,'Current chassis pose');
if ~isempty(paths),c=paths(1).endpoint;subplot(1,2,2);draw_scene(c.chassis_right_forward_mm,c.yaw_deg,'Candidate with chassis yaw');
elseif ~isempty(accepted),c=scenarios(1).minimum_travel_candidate;subplot(1,2,2);draw_scene(c.chassis_right_forward_mm,c.yaw_deg,'Endpoint only; path not verified');
elseif exist('report_local_best','var'),c=report_local_best;subplot(1,2,2);draw_scene(c.chassis_right_forward_mm,c.yaw_deg,'Best local margin; below required 10mm');end
print(fig,fullfile(out,'body-pose.png'),'-dpng','-r140');close(fig);
assert(report.algorithm_failures==0,'Geometric algorithm checks failed');

 function check(name,passed)
  checks(end+1)=struct('name',name,'passed',logical(passed)); %#ok<AGROW>
 end
 function draw_scene(p,yaw,title_text)
  hold on;axis equal;grid on;
  t=linspace(0,2*pi,361);plot(center(1)+disk_radius*cos(t),center(2)+disk_radius*sin(t),'k-');
  plot(center(1)+(disk_radius+50)*cos(t),center(2)+(disk_radius+50)*sin(t),'k:');
  corners=[-b.left,-b.rear;b.right,-b.rear;b.right,b.front;-b.left,b.front;-b.left,-b.rear];
  body=rotate(corners,yaw)+p;fill(body(:,1),body(:,2),[.7,.85,.95],'FaceAlpha',.6);
  plot(p(1),p(2),'kp','MarkerSize',14,'MarkerFaceColor','y');
  for j=1:3
   plot(targets(j,1),targets(j,2),'ro','MarkerFaceColor','r');
   plot([p(1),targets(j,1)],[p(2),targets(j,2)],'--');
  end
  quiver(p(1),p(2),-40*sind(yaw),40*cosd(yaw),0,'b','LineWidth',1.5);
  xlabel('Initial chassis right / mm');ylabel('Initial chassis front / mm');
  title({title_text,sprintf('Yaw %.1f deg; geometric diagnostic only',yaw)});
 end
end

function r=record(row)
r=struct('chassis_right_forward_mm',row(1:2),'yaw_deg',row(3),'endpoint_gap_mm',row(4), ...
 'extension_mm',row(5:7),'logical_deg',row(8:10),'translation_mm',hypot(row(1),row(2)));
end
function p=rotate(p,degrees)
c=cosd(degrees);s=sind(degrees);p=p*[c,s;-s,c];
end
function d=gap(center,b,r)
dx=max(max(-b.left-center(:,1),center(:,1)-b.right),0);
dy=max(max(-b.rear-center(:,2),center(:,2)-b.front),0);d=hypot(dx,dy)-r;
end
function d=translation_gap(center,b,r,start,finish,yaw)
corners=[-b.left,-b.rear;b.right,-b.rear;b.right,b.front;-b.left,b.front];
points=[rotate(corners,yaw)+start;rotate(corners,yaw)+finish];h=convhull(points(:,1),points(:,2));poly=points(h,:);
if inpolygon(center(1),center(2),poly(:,1),poly(:,2)),d=-r;return;end
d=Inf;
for k=1:size(poly,1)-1
a=poly(k,:);v=poly(k+1,:)-a;t=max(0,min(1,dot(center-a,v)/dot(v,v)));d=min(d,norm(center-a-t*v)-r);
end
end
function value=read_json(path)
fid=fopen(path,'r','n','UTF-8');assert(fid~=-1);clean=onCleanup(@()fclose(fid)); %#ok<NASGU>
value=jsondecode(fscanf(fid,'%c'));
end
function write_json(path,value)
fid=fopen(path,'w','n','UTF-8');assert(fid~=-1);clean=onCleanup(@()fclose(fid)); %#ok<NASGU>
fprintf(fid,'%s',jsonencode(value));
end
