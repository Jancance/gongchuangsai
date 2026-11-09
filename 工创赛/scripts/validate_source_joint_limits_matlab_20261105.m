function validate_source_joint_limits_matlab_20261105
% 170mm新行程、CCW正方向、用户360度机械范围的MATLAB联合诊断。
% 比较固定停车点与分两次停稳候选；无串口/调试器/车辆执行代码。
root=fileparts(fileparts(mfilename('fullpath')));
base=fullfile(root,'evidence','integration','2026-11-05','source-markers','paper100-transfer','continuation');
out=fullfile(base,'matlab-stroke170-joint-limits'); if ~exist(out,'dir'), mkdir(out); end
reference=read_json(fullfile(root,'calibration','source_tcp_center_reference_20261105.json'));
old=read_json(fullfile(base,'matlab','result.json'));
assert(strcmp(reference.calibration_id,old.calibration_id));
assert(reference.mechanics.extension_max_from_home_mm==170);
assert(strcmp(reference.mechanics.positive_rotation_top_view,'counterclockwise'));
assert(reference.mechanics.mechanical_rotation_max_deg==360);
q=old.offsets_right_forward_mm;
L=reference.mechanics.current_rotation_pivot_to_tcp_mm;
e=reference.mechanics.source_prepare_nominal_extension_mm;
maximum=reference.mechanics.extension_max_from_home_mm;
checks=struct('name',{},'passed',{});
original=solve(q,L,e,maximum,[0,0],360);
old_shift=solve(q,L,e,maximum,[0,-20],360);
old_shift_firmware=solve(q,L,e,maximum,[0,-20],180);
check('new original near point rejects negative stroke',~original.in_stroke(3));
check('old 20mm shift rejects upper-right over 170mm',~old_shift.in_stroke(2));
check('current firmware additionally rejects upper-left above 180deg',~old_shift_firmware.in_angle(1));
check('positive logical angle produces CCW/leftward displacement',original.logical_deg(1)>180 && original.logical_deg(2)<180);

% 三个目标同时满足行程/角度与旧10mm名义余量，搜几何停车点。
searches=struct('label',{},'max_angle_deg',{},'pose_count',{},'feasible_count',{}, ...
 'best_local_shift_mm',{},'best_chassis_right_forward_mm',{},'best_travel_mm',{},'best_state',{});
for scenario=1:2
 if scenario==1, angle_max=180; label='current firmware 0..180';
 else, angle_max=360; label='user mechanical 0..360; firmware not extended'; end
 [bx,by]=meshgrid(-200:2:200,-120:2:120); x=bx(:); y=by(:);
 xp=q(:,1)'-x; yp=L+q(:,2)'-y;
 ext=hypot(xp,yp)-(L-e); logical_angles=180-atan2d(xp,yp);
 mask=all(ext>=10 & ext<=maximum-10 & logical_angles>=2 & logical_angles<=angle_max-2,2);
 ids=find(mask); s=struct('label',label,'max_angle_deg',angle_max,'pose_count',numel(x), ...
  'feasible_count',numel(ids),'best_local_shift_mm',[],'best_chassis_right_forward_mm',[], ...
  'best_travel_mm',[],'best_state',struct);
 if ~isempty(ids)
  [distance,k]=min(hypot(x(ids),y(ids))); k=ids(k); shift=[x(k),y(k)];
  s.best_local_shift_mm=shift;
  s.best_chassis_right_forward_mm=[shift(2),-shift(1)];
  s.best_travel_mm=distance; s.best_state=solve(q,L,e,maximum,shift,angle_max);
 end
 searches(scenario)=s;
 check(sprintf('scenario %d finds a nominal geometry pose',scenario),~isempty(ids));
end

% 两个停稳位置按任务目标切换；不得固定先远后近而改变任务码顺序。
two_pose=original;
names={'extension_mm','logical_deg','clockwise_delta_deg','in_stroke','in_angle'};
for k=1:numel(names), name=names{k}; two_pose.(name)(3)=old_shift.(name)(3); end
two_pose.local_shift_by_target_mm=[0,0;0,0;0,-20];
two_pose.chassis_shift_right_forward_by_target_mm=[0,0;0,0;-20,0];
two_pose.requires_park_realign_and_new_observation=true;
check('two-pose all three meet user 170mm/360deg model',all(two_pose.in_stroke & two_pose.in_angle));
check('two-pose nominal stroke endpoints each retain 10mm',min(two_pose.extension_mm)>=10 && max(two_pose.extension_mm)<=maximum-10);
check('first two targets stay at original chassis pose',all(two_pose.local_shift_by_target_mm(1:2,:)==0,'all'));
check('20mm against extension maps to 20mm chassis left',isequal(two_pose.chassis_shift_right_forward_by_target_mm(3,:),[-20,0]));

orders=perms(1:3); order_checks=struct('order',{},'base_changes',{},'base_travel_mm',{}, ...
 'final_chassis_shift_right_forward_mm',{},'requires_carrier_confirmation_before_each_reposition',{});
for k=1:size(orders,1)
 order=orders(k,:);
 poses=two_pose.chassis_shift_right_forward_by_target_mm(order,:);
 step_lengths=sqrt(sum(diff([[0,0];poses],1,1).^2,2));
 order_checks(k)=struct('order',order,'base_changes',sum(step_lengths>0), ...
  'base_travel_mm',sum(step_lengths),'final_chassis_shift_right_forward_mm',poses(end,:), ...
  'requires_carrier_confirmation_before_each_reposition',true);
 check(sprintf('task order permutation %d preserves targets and valid poses',k), ...
  isequal(sort(order),1:3) && all(two_pose.in_stroke(order) & two_pose.in_angle(order)) ...
  && any(abs(sum(step_lengths)-[20,40])<1e-8));
end

% 在声明扰动网格中同时验证全部三点；不冒充测量置信区间。
sets=0; all_pass=true; minimum=Inf; largest=-Inf; angles=[Inf,-Inf];
for variedL=[225,235,245]
 for variedE=[65,70,75]
  for variedMax=[165,170]
   for shift=[17,20,23]
    for dx=[-3,0,3]
     for dy=[-3,0,3]
      changed=q+[dx,dy]; a=solve(changed(1:2,:),variedL,variedE,variedMax,[0,0],360);
      b=solve(changed(3,:),variedL,variedE,variedMax,[0,-shift],360);
      ex=[a.extension_mm;b.extension_mm]; an=[a.logical_deg;b.logical_deg];
      all_pass=all_pass && all([a.in_stroke;b.in_stroke]) && all([a.in_angle;b.in_angle]);
      minimum=min(minimum,min(ex)); largest=max(largest,max(ex));
      angles=[min(angles(1),min(an)),max(angles(2),max(an))]; sets=sets+1;
     end
    end
   end
  end
 end
end
check('two-pose joint limits pass declared perturbation grid',all_pass);
check('nonfinite input rejects',rejects(@() solve([NaN,0],L,e,maximum,[0,0],360)));
check('impossible baseline rejects',rejects(@() solve(q,60,e,maximum,[0,0],360)));
zero_boundary=solve([0,-70],L,e,maximum,[0,0],360);
max_boundary=solve([0,100],L,e,maximum,[0,0],360);
check('zero stroke boundary',abs(zero_boundary.extension_mm)<1e-8 && zero_boundary.in_stroke);
check('170mm boundary',abs(max_boundary.extension_mm-170)<1e-8 && max_boundary.in_stroke);

report=struct; report.engine='local MATLAB'; report.version=version; report.release=version('-release');
report.time_asia_shanghai=char(datetime('now','TimeZone','Asia/Shanghai','Format','yyyy-MM-dd HH:mm:ss Z'));
report.reference=reference; report.target_ids=old.target_ids; report.offsets_right_forward_mm=q;
report.original_pose=original; report.old_20mm_shift=old_shift; report.old_20mm_shift_with_current_firmware=old_shift_firmware;
report.single_pose_search=searches; report.two_pose_candidate=two_pose;
report.task_order_permutations=order_checks;
report.nominal_stroke_margin_mm=min([two_pose.extension_mm;maximum-two_pose.extension_mm]);
report.sensitivity=struct('sets',sets,'targets_checked',3*sets,'all_joint_limits_pass',all_pass, ...
 'current_L_mm',[225,235,245],'current_E_mm',[65,70,75],'max_E_mm',[165,170], ...
 'shift_mm',[17,20,23],'target_axis_perturbation_mm',[-3,0,3], ...
 'minimum_extension_mm',minimum,'maximum_extension_mm',largest,'logical_range_deg',angles, ...
 'measured_uncertainty_bounds',false);
report.checks=checks; report.check_count=numel(checks); report.failure_count=sum(~[checks.passed]);
report.geometry_passed=all([checks.passed]); report.full_motion_validation_passed=false;
report.current_firmware_accepts_two_pose=false;
report.user_has_selected_two_pose_workflow=false;
report.motion_allowed=false; report.hardware_operated=false;
report.pending={'single vs two-pose workflow choice', ...
 'new-pose freshness and updated plane/camera transform', ...
 'actual TCP/height/coverage and chassis pose/clearance', ...
 'all three joint dynamic and swept-volume/fault-exit MATLAB validation', ...
 'firmware angular admission change after applicable MATLAB validation'};
write_json(fullfile(out,'result.json'),report);
save(fullfile(out,'simulation.mat'),'report','q','reference');

fig=figure('Visible','off','Color','w','Position',[50,50,1200,570]);
subplot(1,2,1); hold on; grid on;
bar([original.extension_mm,old_shift.extension_mm,two_pose.extension_mm]);
plot([0.4,3.6],[170,170],'r--','LineWidth',1.5);
plot([0.4,3.6],[10,10],'k:'); plot([0.4,3.6],[160,160],'k:');
set(gca,'XTick',1:3,'XTickLabel',{'Upper-left','Upper-right','Near'});
ylabel('Extension from home / mm'); title('Corrected maximum: 170 mm');
legend('Original','All shifted left 20mm','Two stop poses','170mm limit','Location','southoutside');
subplot(1,2,2); hold on; grid on;
bar(two_pose.logical_deg); plot([0.4,3.6],[180,180],'r--','LineWidth',1.5);
set(gca,'XTick',1:3,'XTickLabel',{'Upper-left','Upper-right','Near'});
ylabel('Logical angle / deg; positive CCW'); ylim([0,220]);
title({'Two-pose geometry candidate','Current firmware still rejects >180 deg'});
print(fig,fullfile(out,'joint-limits.png'),'-dpng','-r130'); close(fig);
fprintf('MATLAB joint limits: checks=%d failures=%d sets=%d; motion_allowed=0\n',report.check_count,report.failure_count,sets);
disp(table(old.target_ids(:),two_pose.extension_mm,two_pose.logical_deg, ...
 'VariableNames',{'Target','TwoPoseExtension_mm','TwoPoseLogical_deg'}));
for k=1:numel(searches)
 fprintf('%s: feasible=%d closest_shift_local=[%.1f,%.1f]mm travel=%.2fmm\n', ...
  searches(k).label,searches(k).feasible_count,searches(k).best_local_shift_mm,searches(k).best_travel_mm);
end
assert(report.geometry_passed,'Joint geometry model has failing checks; no implementation');

 function check(name,passed)
  checks(end+1)=struct('name',name,'passed',logical(passed)); %#ok<AGROW>
 end
end

function result=solve(q,L,e,maximum,shift,angle_max)
assert(size(q,2)==2 && all(isfinite(q(:))) && all(isfinite([L,e,maximum,shift,angle_max])));
assert(L>e && e>=0 && maximum>=e && angle_max>0 && angle_max<=360);
relative=q-shift+[0,L]; radius=hypot(relative(:,1),relative(:,2));
clockwise=atan2d(relative(:,1),relative(:,2)); logical_angles=180-clockwise; ext=radius-(L-e);
result=struct('extension_mm',ext,'logical_deg',logical_angles,'clockwise_delta_deg',clockwise, ...
 'in_stroke',ext>=0 & ext<=maximum,'in_angle',logical_angles>=0 & logical_angles<=angle_max);
end

function tf=rejects(action)
try, action(); tf=false; catch, tf=true; end
end

function value=read_json(path)
fid=fopen(path,'r','n','UTF-8'); assert(fid~=-1); cleanup=onCleanup(@() fclose(fid)); %#ok<NASGU>
value=jsondecode(fscanf(fid,'%c'));
end

function write_json(path,value)
fid=fopen(path,'w','n','UTF-8'); assert(fid~=-1); cleanup=onCleanup(@() fclose(fid)); %#ok<NASGU>
fprintf(fid,'%s',jsonencode(value));
end
