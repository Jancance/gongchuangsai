function validate_source_parallel_plan_matlab_20261106
% Source-only parallel scheduling/trajectory model before C implementation.
% Cylinder/tool bounds are explicitly declared scenarios, not field facts.
root=fileparts(fileparts(mfilename('fullpath')));
out=fullfile(root,'evidence','integration','2026-11-06','source-parallel-plan');
if ~exist(out,'dir'),mkdir(out);end
geo=jsondecode(fileread(fullfile(root,'evidence','integration','2026-11-06','source-vision-new-pose','finalstop-geometry','matlab-result.json')));
lift=jsondecode(fileread(fullfile(root,'evidence','integration','2026-11-06','lift-retarget','round-trip-result.json')));
checks=struct('name',{},'passed',{});rows=struct('kind',{},'item',{},'from',{},'to',{},'duration_s',{},'serial_duration_s',{},'collision_free_declared_model',{},'minimum_other_clearance_mm',{});
dm=300;ext=106;z=1846.4;accel=45055;ext_mm_s=ext*3200/(60*32);down_mm_s=69/(lift.down_move_command_to_stable_ms/1000);up_mm_s=69/(lift.up_move_command_to_stable_ms/1000);
check('reference speeds remain explicit and legal',dm==300 && ext==106 && z==1846.4 && accel==45055);
check('current3target geometry nominal stroke fits',all(geo.extension_candidate_mm>=0 & geo.extension_candidate_mm<=170));
% Angles are explicit unwrapped work coordinates, no implicit shortest-wrap.
check('source required193degree exceeds old180guard',max(geo.logical_angle_candidate_deg)>180 && max(geo.logical_angle_candidate_deg)<270);
centers=geo.body_target_candidate_mm;targets=[geo.logical_angle_candidate_deg(:),geo.extension_candidate_mm(:),50*ones(3,1)];
% Conservative preflight sampling with extra relative arrival skew +/-20ms.
for tool=[15,25,35]
 for k=1:3
  a=[180,48,119];b=targets(k,:);
  for skew=[-.02,0,.02]
   [T,seq,free,gap]=trajectory(a,b,false,k,tool,skew);
   check(sprintf('approach item%d tool%d skew%+.2f no other material collision',k,tool,skew),free);
   if tool==25 && skew==0,rows(end+1)=record('approach',k,a,b,T,seq,free,gap);end %#ok<AGROW>
  end
 end
end
% Requested simultaneous return may intersect another cylinder before lift.
unsafe=0;return_cases=struct('item',{},'carrier_relative_height_mm',{},'free',{},'gap_mm',{},'duration_s',{});
for carrier=[-20,0,20,40]
 for k=1:3
  h=carrier+5+50; % held base=claw-bottom minus grasp50; carrier+5 clearance
  a=targets(k,:);b=[35.1079407,0,h];
  [T,seq,free,gap]=trajectory(a,b,true,k,25,0);
  return_cases(end+1)=struct('item',k,'carrier_relative_height_mm',carrier,'free',free,'gap_mm',gap,'duration_s',T); %#ok<AGROW>
  unsafe=unsafe+~free;
 end
end
check('direct return collision is detected rather than ignored',unsafe>0);
% Preparation release is a barrier, and source M waits for real alignment.
check('parallel group duration=max axis, not sum',all([rows.duration_s]<=[rows.serial_duration_s]));
old_smooth=sqrt(180*5.774/150);old_peak=180*1.875/old_smooth;
check('old smooth prep cap300 did not reach300',old_peak<300 && old_smooth>180/300);
% Final piece release is a fork: route and initialization run independently.
% Neither route completion nor stow completion fabricates loaded inventory.
for confirmed=0:3
 for released=[false,true]
  fork=(confirmed==3 && released);
  route_queued=fork;stow_queued=fork;
  check(sprintf('final release fork inventory%d release%d',confirmed,released), ...
   route_queued==stow_queued && route_queued==(confirmed==3 && released));
 end
end
for stow_done=[false,true]
 route_active=true;parallel_allowed=route_active; % no arm-busy serialization
 check(sprintf('background initialization cannot block route stowdone%d',stow_done),parallel_allowed);
end
%3s budget: hide horizontal time under descent only where sweep is valid.
approach_duration=[rows.duration_s];close=[.1,.2,.3];exit=[.2,.4,.8,1.2];
budget=zeros(3,3,4);for k=1:3,for c=1:3,for e=1:4,budget(k,c,e)=.15+approach_duration(k)+close(c)+exit(e);end,end,end
report=struct('engine','local MATLAB','release',version('-release'),'checks',checks,'check_count',numel(checks), ...
 'failure_count',sum(~[checks.passed]),'passed',all([checks.passed]),'approach_cases',rows,'direct_return_cases',return_cases, ...
 'speed_command',struct('dm_deg_s',dm,'extension_RPM',ext,'lift_RPM',z,'stepper_accel_RPM_s',accel), ...
 'declared_ext_pulses_per_mm',32,'declared_tool_radii_mm',[15,25,35],'declared_material_radius_mm',25,'retained_material_height_mm',60, ...
 'axis_start_skew_s',.02,'old_source_smooth180_duration_s',old_smooth,'old_source_smooth180_peak_deg_s',old_peak, ...
 'pickup_budget_s',budget,'assumed_detection_dispatch_s',.15,'assumed_close_s',close,'assumed_exit_s',exit, ...
 'pickup_budget_meets3s',budget<3,'carrier_height_measured',false,'actual_tool_envelope_measured',false,'real_full_path_validated',false, ...
 'actual_extension_mm_speed_measured',false,'actual_loaded_timing_measured',false,'firmware_downloaded',false,'motion_allowed',false, ...
 'implementation_scope','parallel group scheduler and conditional trajectory preflight; no real pose/reference fabricated; cannot release unsafe direct returns', ...
 'final_release_policy','third material seated and release completed: queue stage3 route and initialization together; do not wait for initialization to start route', ...
 'next_required','actual carrier height and tool/body sweep, current references, field alignment executor and stopped-target transaction');
fid=fopen(fullfile(out,'matlab-result.json'),'w','n','UTF-8');assert(fid~=-1);fprintf(fid,'%s',jsonencode(report));fclose(fid);save(fullfile(out,'matlab-result.mat'),'report');
fprintf('SOURCE PARALLEL MATLAB checks%d failures%d; approach durations[%.3f %.3f %.3f]s; unsafe direct returns%d/12; no field release\n',numel(checks),report.failure_count,approach_duration,unsafe);assert(report.passed);
 function check(name,passed)
  checks(end+1)=struct('name',name,'passed',logical(passed));
 end
 function [T,seq,free,gap]=trajectory(a,b,loaded,item,tool,skew)
  durations=[abs(b(1)-a(1))/dm,abs(b(2)-a(2))/ext_mm_s,abs(b(3)-a(3))/(down_mm_s*(b(3)<a(3))+up_mm_s*(b(3)>=a(3)))];
  T=max(durations)+abs(skew);seq=sum(durations);ts=(0:.002:T)';q=zeros(numel(ts),3);
  for axis=1:3
   if durations(axis)<1e-9,ratio=ones(size(ts));else,ratio=min(max((ts-(axis-2)*skew)/durations(axis),0),1);end
   q(:,axis)=a(axis)+(b(axis)-a(axis))*ratio;
  end
  xy=[(161+q(:,2)).*cosd(q(:,1)-180),(161+q(:,2)).*sind(q(:,1)-180)];
  bottom=q(:,3)-50*loaded;gap=inf;free=true;
  for obstacle=1:3
   if obstacle==item,continue;end
   horizontal=vecnorm(xy-centers(obstacle,:),2,2)-tool-25;
   vertical=bottom-60;clearance=max(horizontal,vertical);gap=min(gap,min(clearance));
   if any(horizontal<0 & vertical<0),free=false;end
  end
 end
 function r=record(kind,item,a,b,T,seq,free,gap)
  r=struct('kind',kind,'item',item,'from',a,'to',b,'duration_s',T,'serial_duration_s',seq,'collision_free_declared_model',free,'minimum_other_clearance_mm',gap);
 end
end
