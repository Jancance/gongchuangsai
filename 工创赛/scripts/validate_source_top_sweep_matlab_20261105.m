function validate_source_top_sweep_matlab_20261105
% MATLAB前置：同一停车点的顶部旋转/伸缩轨迹、到位误差与故障退出候选。
% 反馈/动力学为声明模型，不是实车系统辨识；Z下降与载台几何尚不在通过范围。
root=fileparts(fileparts(mfilename('fullpath')));
base=fullfile(root,'evidence','integration','2026-11-05','source-markers','paper100-transfer','continuation');
out=fullfile(base,'body270x250','matlab-top-sweep');if ~exist(out,'dir'),mkdir(out);end
reference=read_json(fullfile(root,'calibration','source_tcp_center_reference_20261105.json'));
body=read_json(fullfile(base,'body270x250','matlab-body-pose','result.json'));
candidate=body.maximum_margin_within100mm_yaw30;
theta=candidate.logical_deg(:);extension=candidate.extension_mm(:);
p=struct('dt',.01,'feedback_period',.05,'freshness',.10,'settle_samples',3, ...
 'dm_v',150,'dm_a',150,'ext_v',50,'ext_a',100,'segment_mm',35, ...
 'dm_tol_deg',.2,'extension_tol_mm',.1,'velocity_dm_deg_s',.5,'velocity_extension_mm_s',.5, ...
 'service_gap_s',.10,'settle_timeout_s',2,'stop_ack_timeout_s',.3, ...
 'physical_zero_radius_mm',165,'extension_max_mm',170,'gripper_hold_loaded_deg',135, ...
 'gripper_hold_empty_deg',210,'gripper_position_feedback_available',false);
checks=struct('name',{},'passed',{});trajectories=struct('order',{},'legs',{},'simulated_top_motion_s',{});
orders=perms(1:3);tau_values=[.04,.10,.20];normal_runs=0;max_joint_error=0;max_duration=0;
for k=1:size(orders,1)
 start=[35.1079407,0];legs=struct('kind',{},'target_index',{},'goal',{},'result',{});total=0;
 for n=1:3
  index=orders(k,n);
  goal=[theta(index),extension(index)];
  for phase=1:2
   if phase==1,kind='to-source-plane-marker';else,goal=[35.1079407,0];kind='return-carrier-reference-candidate';end
   for tau=tau_values
    result=simulate_leg(start,goal,p,tau,'none');normal_runs=normal_runs+1;
    check(sprintf('normal order%d item%d phase%d tau%.2f completes',k,n,phase,tau),strcmp(result.status,'COMPLETE'));
    check(sprintf('normal trajectory%d never leaves 0..170mm',normal_runs),result.minimum_ext_mm>=-1e-8 && result.maximum_ext_mm<=170+1e-8);
    check(sprintf('normal trajectory%d respects max180deg per joint move',normal_runs),abs(goal(1)-start(1))<=180);
    check(sprintf('normal trajectory%d waits for reference completion',normal_runs),result.finish_s>=result.reference_end_s);
    max_joint_error=max(max_joint_error,result.final_joint_error_mm);
   end
   legs(end+1)=struct('kind',kind,'target_index',index,'goal',goal,'result',result); %#ok<AGROW>
   total=total+result.finish_s;start=goal;
  end
 end
 trajectories(k)=struct('order',orders(k,:),'legs',legs,'simulated_top_motion_s',total);
 max_duration=max(max_duration,total);
end

% 直接把真实±3deg判据映射到TCP误差；不是假设电机一定有此误差。
radii=165+extension;
legacy_angle_chord=2*radii.*sind(3/2);
fine_angle_chord=2*radii.*sind(p.dm_tol_deg/2);
check('legacy3deg permits more than2mm endpoint error',max(legacy_angle_chord)>2);
check('candidate0.2deg chord plus0.1mm extension under2mm joint-only',max(fine_angle_chord)+p.extension_tol_mm<2);
% 永久1.6deg反馈偏差，旧判据可以通过；新候选应超时拒绝。
oldp=p;oldp.dm_tol_deg=3;oldp.velocity_dm_deg_s=2;
legacy=simulate_leg([180,70],[theta(1),extension(1)],oldp,.1,'dm_bias');
strict=simulate_leg([180,70],[theta(1),extension(1)],p,.1,'dm_bias');
check('legacy3deg model accepts1.6deg residual',strcmp(legacy.status,'COMPLETE'));
check('fine candidate rejects1.6deg residual',strcmp(strict.status,'FAULT_SETTLE_TIMEOUT'));

failures={'cancel','dm_stale','ext_stale','dm_disabled','reference_lost','pose_changed', ...
 'service_gap','target_readback_mismatch','pwm_write_failed','stop_ack_failed','dm_bias'};
faults=struct('fault',{},'result',{});
for k=1:numel(failures)
 result=simulate_leg([180,70],[theta(1),extension(1)],p,.1,failures{k});
 faults(k)=struct('fault',failures{k},'result',result);
 check(['failure blocks completion: ' failures{k}],~strcmp(result.status,'COMPLETE'));
 check(['failure time bounded: ' failures{k}],result.finish_s<=result.reference_end_s+p.settle_timeout_s+p.stop_ack_timeout_s+.15);
 if strcmp(failures{k},'stop_ack_failed')
  check('unconfirmed STOP remains pending, never reports stopped',result.stop_pending && ~result.stop_confirmed);
 else
  check(['STOP acknowledged in modeled case: ' failures{k}],result.stop_confirmed);
 end
end

% 与本轮MATLAB误差/停车/车盘几何在同一结果中记录，不当动力学辨识完成。
tip=reference.scene_measurements.gripper_tip_above_source_plane_mm;
height=reference.scene_measurements.material_height_mm;clearance=10;
grasp_offsets=[0,10,20,30,40,49,50,60];
loaded_bottom=tip-grasp_offsets;
vertical=struct('tip_top_mm',tip,'material_height_mm',height,'clearance_goal_mm',clearance, ...
 'tested_tip_above_material_bottom_mm',grasp_offsets,'loaded_bottom_at_top_mm',loaded_bottom, ...
 'above_other_material_with_10mm_margin',loaded_bottom>=height+clearance, ...
 'maximum_tip_above_material_bottom_for_clearance_mm',tip-height-clearance, ...
 'user_top_height_confirmed',false,'user_grasp_offset_mm',[],'loaded_clearance_physically_verified',false);
vertical.height_reference='lowest gripper bottom, user clarified; not an independently measured contact-point offset';
if isfield(reference.scene_measurements,'top_tip_height_confirmed')
 vertical.user_top_height_confirmed=reference.scene_measurements.top_tip_height_confirmed;
end
if isfield(reference.scene_measurements,'grasp_tip_above_material_bottom_mm') && ...
 ~isempty(reference.scene_measurements.grasp_tip_above_material_bottom_mm)
 vertical.user_grasp_offset_mm=reference.scene_measurements.grasp_tip_above_material_bottom_mm;
 vertical.user_offset_loaded_bottom_mm=tip-vertical.user_grasp_offset_mm;
 vertical.user_offset_clears_other_material_by10mm=vertical.user_offset_loaded_bottom_mm>=height+clearance;
end
report=struct;report.engine='local MATLAB';report.version=version;report.release=version('-release');
report.reference=reference;report.parameters=p;report.target_ids={'upper-left','upper-right','near'};
report.target_extension_mm=extension';report.target_theta_deg=theta';report.trajectories=trajectories;
report.normal_runs=normal_runs;report.fault_cases=faults;report.legacy_bias_case=legacy;report.strict_bias_case=strict;
report.legacy3deg_chord_error_mm=legacy_angle_chord';report.candidate0_2deg_chord_error_mm=fine_angle_chord';
report.max_normal_final_joint_error_mm=max_joint_error;report.maximum_simulated_top_motion_time_s=max_duration;
report.vertical_clearance=vertical;report.checks=checks;report.check_count=numel(checks);
report.failure_count=sum(~[checks.passed]);report.abstract_top_motion_checks_passed=all([checks.passed]);
report.model_scope='quintic reference + declared first-order plants + feedback samples/age and STOP acknowledgement model';
report.plant_is_identified_from_real_vehicle=false;report.actual_firmware_scheduler_simulated=false;
report.horizontal_arm_and_cable_swept_volume_verified=false;report.gray_lane_verified=false;
report.source_descent_or_carrier_placement_simulated=false;
report.full_motion_validation_passed=false;report.motion_allowed=false;report.hardware_operated=false;
report.pending={'Z/top and grasp-offset field confirmation','tool/cable swept volume and onboard geometry', ...
 'actual target/TCP/base/encoder error measurements','3-axis full cycle and real scheduling/CAN/power failure exits', ...
 'candidate tolerances are design conditions, not achieved accuracy'};
write_json(fullfile(out,'result.json'),report);save(fullfile(out,'simulation.mat'),'report','p','theta','extension');

fig=figure('Visible','off','Color','w','Position',[50,50,1200,550]);
subplot(1,2,1);bar([legacy_angle_chord,fine_angle_chord]);grid on;
set(gca,'XTick',1:3,'XTickLabel',{'Upper-left','Upper-right','Near'});
ylabel('TCP chord error / mm');title('Angular acceptance converted to endpoint error');
legend('Existing 3 deg','Candidate 0.2 deg','Location','southoutside');
subplot(1,2,2);plot(grasp_offsets,loaded_bottom-height,'o-');hold on;grid on;
plot([0,60],[10,10],'r--');xlabel('Tip above held material bottom / mm');ylabel('Loaded bottom clearance over other material / mm');
title({'At tip height 119mm, material height 60mm','Parametric geometry; actual grasp offset required'});
print(fig,fullfile(out,'top-sweep.png'),'-dpng','-r130');close(fig);
fprintf('MATLAB top sweep: normal=%d fault=%d checks=%d failures=%d; final joint error max=%.3fmm; full_motion=0\n', ...
 normal_runs,numel(failures),report.check_count,report.failure_count,max_joint_error);
fprintf('Legacy3deg max endpoint chord=%.3fmm; candidate0.2deg=%.3fmm; required grasp-tip offset<=%.1fmm for10mm vertical margin\n', ...
 max(legacy_angle_chord),max(fine_angle_chord),vertical.maximum_tip_above_material_bottom_for_clearance_mm);
assert(report.abstract_top_motion_checks_passed,'Top motion candidate has failing modeled checks');

 function check(name,passed)
  checks(end+1)=struct('name',name,'passed',logical(passed)); %#ok<AGROW>
 end
end

function result=simulate_leg(start,goal,p,tau,fault)
dt=p.dt;time=0;state=start;vel=[0,0];held=start;heldvel=[0,0];last_feedback=[0,0];
last_service=0;last_sample=-p.feedback_period;stable=0;status='RUNNING';stop_at=NaN;stop_pending=false;stop_confirmed=false;
fault_at=.25;dmT=duration(goal(1)-start(1),p.dm_v,p.dm_a);
delta=goal(2)-start(2);count=max(1,ceil(abs(delta)/p.segment_mm));segments=linspace(start(2),goal(2),count+1);
segment_times=zeros(1,count);
for k=1:count,segment_times(k)=duration(segments(k+1)-segments(k),p.ext_v,p.ext_a)+.15;end
endT=max(dmT,sum(segment_times));minE=state(2);maxE=state(2);max_follow=[0,0];
while time<=endT+p.settle_timeout_s+p.stop_ack_timeout_s+.3
 if strcmp(fault,'service_gap') && time>=fault_at && time<fault_at+.15,time=time+dt;continue;end
 if ~strcmp(status,'RUNNING')
  if ~strcmp(fault,'stop_ack_failed') && time-stop_at>=.05
   stop_confirmed=true;stop_pending=false;break;
  elseif time-stop_at>=p.stop_ack_timeout_s
   status='FAULT_STOP_UNCONFIRMED';break;
  end
 else
  reason='';
  if time-last_service>p.service_gap_s+1e-9,reason='FAULT_SERVICE_GAP';end
  if time>=fault_at
   switch fault
    case {'cancel','stop_ack_failed'},reason='CANCELLED';
    case 'dm_disabled',reason='FAULT_DM_DISABLED';
    case 'reference_lost',reason='FAULT_EXTENSION_REFERENCE';
    case 'pose_changed',reason='FAULT_POSE_VERSION';
    case 'target_readback_mismatch',reason='FAULT_DRIVER_TARGET';
    case 'pwm_write_failed',reason='FAULT_GRIP_COMMAND';
   end
  end
  if any(time-last_feedback>=p.freshness+1e-9),reason='FAULT_FEEDBACK_AGE';end
  if ~isempty(reason),status=reason;stop_at=time;stop_pending=true;time=time+dt;continue;end
  last_service=time;
  ref=[start(1)+(goal(1)-start(1))*smooth(time/dmT),extension_reference(time,segments,segment_times)];
  next=state+(ref-state)*min(1,dt/tau);vel=(next-state)/dt;state=next;
  if time-last_sample>=p.feedback_period-1e-9
   if ~(strcmp(fault,'dm_stale') && time>=fault_at),held(1)=state(1);heldvel(1)=vel(1);last_feedback(1)=time;end
   if ~(strcmp(fault,'ext_stale') && time>=fault_at),held(2)=state(2);heldvel(2)=vel(2);last_feedback(2)=time;end
   if strcmp(fault,'dm_bias'),held(1)=held(1)+1.6;end
   last_sample=time;
   close=abs(held-goal)<=[p.dm_tol_deg,p.extension_tol_mm];
   stopped=abs(heldvel)<=[p.velocity_dm_deg_s,p.velocity_extension_mm_s];
   if all(close & stopped) && time>=endT,stable=stable+1;else,stable=0;end
  end
  minE=min(minE,state(2));maxE=max(maxE,state(2));max_follow=max(max_follow,abs(ref-state));
  if stable>=p.settle_samples,status='COMPLETE';break;end
  if time>=endT+p.settle_timeout_s,status='FAULT_SETTLE_TIMEOUT';stop_at=time;stop_pending=true;end
 end
 time=time+dt;
end
radius=p.physical_zero_radius_mm+held(2);desired_radius=p.physical_zero_radius_mm+goal(2);
observed=[radius*cosd(held(1)-180),radius*sind(held(1)-180)];
desired=[desired_radius*cosd(goal(1)-180),desired_radius*sind(goal(1)-180)];
result=struct('status',status,'finish_s',time,'reference_end_s',endT,'feedback_final',held, ...
 'goal',goal,'final_joint_error_mm',norm(observed-desired),'minimum_ext_mm',minE,'maximum_ext_mm',maxE, ...
 'maximum_dm_reference_tracking_deg',max_follow(1),'maximum_extension_reference_tracking_mm',max_follow(2),'extension_segment_count',count, ...
 'stop_pending',stop_pending,'stop_confirmed',stop_confirmed,'fault',fault);
end

function v=extension_reference(time,segments,times)
v=segments(end);previous=0;
for k=1:numel(times)
 if time<previous+times(k)
  T=times(k)-.15;v=segments(k)+(segments(k+1)-segments(k))*smooth((time-previous)/T);return;
 end
 previous=previous+times(k);
end
end
function T=duration(delta,v,a)
T=max([.5,abs(delta)*1.875/v,sqrt(abs(delta)*5.774/a)]);
end
function value=smooth(u)
u=min(1,max(0,u));value=u.^3.*(10-15*u+6*u.^2);
end
function value=read_json(path)
fid=fopen(path,'r','n','UTF-8');assert(fid~=-1);clean=onCleanup(@()fclose(fid)); %#ok<NASGU>
value=jsondecode(fscanf(fid,'%c'));
end
function write_json(path,value)
fid=fopen(path,'w','n','UTF-8');assert(fid~=-1);clean=onCleanup(@()fclose(fid)); %#ok<NASGU>
fprintf(fid,'%s',jsonencode(value));
end
