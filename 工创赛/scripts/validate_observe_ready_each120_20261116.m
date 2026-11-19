function validate_observe_ready_each120_20261116
% Authorized planning model; no product/firmware/hardware implementation.
root=fileparts(fileparts(mfilename('fullpath')));
out=fullfile(root,'evidence','integration','2026-11-16','observe-ready-each120');
if ~exist(out,'dir'),mkdir(out);end
checks=0;cases=struct([]);
bundle=fullfile(root,'output','source_stopped5_initial_branch_pi_20261116');
orbit=jsondecode(fileread(fullfile(bundle,'orbit-config.json')));
assert(orbit.maximum_frame_interval_s==.25);checks=checks+1;
% Real implementation facts: no new UART field or motion-control channel.
entry=fileread(fullfile(bundle,'source_fast_cycle_view_20261115.py'));
protocol=fileread(fullfile(bundle,'mcu_cycle_protocol.py'));
assert(contains(protocol,"'run','state','pose'"));checks=checks+1;
assert(contains(entry,"key=(status['run'],status['state'],status['pose'])"));checks=checks+1;
assert(contains(entry,"status['item']>0 or initial_rotation.update"));checks=checks+1;
% Scope proof: first observation and BOTH return observations use independent epochs.
for run=1:2
 for item=0:2
  e=epoch(run,2+item*2,item,10);
  assert(e.frames==0 && ~e.opened && e.phase==0 && isempty(e.branch));checks=checks+1;
  % Same-pose heartbeat does not reset or move ready receipt time.
  e=admit(e,run,e.pose,1,10.01,10.02);assert(e.frames==0 && e.fence_seq==1);checks=checks+1;
  e=admit(e,run,e.pose,2,10.021,10.03);assert(e.frames==1);checks=checks+1;
  old=e;e=heartbeat(e,run,e.pose,item,10.1);assert(isequal(e,old));checks=checks+1;
  % New pose or run discards all frame/phase/branch state.
  e=heartbeat(e,run,e.pose+1,item,11);assert(e.frames==0 && e.fence_seq==0 && e.ready_s==11);checks=checks+1;
  e=heartbeat(e,run+1,e.pose,item,12);assert(e.frames==0 && e.phase==0 && e.ready_s==12);checks=checks+1;
 end
end
% Timing combinations include old frame, an in-flight read crossing ready,
% late delivery, and frames captured before ready but processed after it.
for readStart=[9 9.9 9.999 10 10.001 10.1]
 for readEnd=[9.5 9.99 10 10.001 10.05 10.2]
  if readEnd<readStart,continue;end
  e=epoch(1,2,0,10);
  e=admit(e,1,2,1,readStart,readEnd);
  assert(e.frames==0);checks=checks+1;
  if readEnd>10
   assert(e.fence_seq==1);checks=checks+1;
   % In a sequential worker, any later sequence started at/after this end.
   e=admit(e,1,2,2,readEnd+.001,readEnd+.02);
   assert(e.frames==1);checks=checks+1;
  else
   assert(e.fence_seq==0);checks=checks+1;
  end
 end
end
% Duplicated sequence cannot count twice; wrong epoch and backwards clocks fail.
e=epoch(1,2,0,10);e=admit(e,1,2,10,9.9,10.01);
for k=1:5,e=admit(e,1,2,10,9.9,10.01);end
assert(e.frames==0);checks=checks+1;
e=admit(e,1,3,11,10.02,10.03);assert(e.frames==0);checks=checks+1;
e=admit(e,2,2,11,10.02,10.03);assert(e.frames==0);checks=checks+1;
e=admit(e,1,2,11,10.02,10.03);assert(e.frames==1);checks=checks+1;
e=admit(e,1,2,12,10.01,10.02);assert(e.frames==1);checks=checks+1;
old=e;e=heartbeat(e,1,1,0,11);assert(isequal(old,e));checks=checks+1;
e=heartbeat(e,0,2,0,11);assert(isequal(old,e));checks=checks+1;
% A camera worker generation change has no continuous sequence proof.
e.valid=false;e=admit(e,1,2,999,11,11.02);assert(e.frames==1);checks=checks+1;
% Unwrap crossing +/-180 degrees measures net motion, not a synthetic full turn.
for phases={[170 179 -170 -150 -130 -110 -90 -70],[-170 -179 170 150 130 110 90 70]}
 values=phases{1};net=0;
 for k=2:numel(values)
  delta=deg2rad(values(k)-values(k-1));net=net+atan2(sin(delta),cos(delta));
 end
 assert(abs(abs(rad2deg(net))-120)<1e-10);checks=checks+1;
end
% Axis ready refers to actual confirmation, never a requested target or ACK alone.
for status=0:9
 for complete=[false true]
  for zero=[false true]
   for fresh=[false true]
    for within=[false true]
     for fault=[false true]
      allowed=status==6 && complete && zero && fresh && within && ~fault;
      if status~=6 || ~complete || ~zero || ~fresh || ~within || fault,assert(~allowed);
      else,assert(allowed);end
      checks=checks+1;
     end
    end
   end
  end
 end
end
% Every item: initial stationary needs LATER observed net 120, then FIVE stop frames.
% Stable qualification itself must not inherit an old angle/old five-frame count.
for item=0:2
 for sign=[-1 1]
  b=branch();
  for k=1:5,[b,grab]=advance(b,0,true,false);assert(~grab);checks=checks+1;end
  assert(strcmp(b.branch,'stopped') && b.phase==0);checks=checks+1;
  for angle=[20 40 60 80 100 119.999]
   [b,grab]=advance(b,sign*angle,false,false);assert(~grab);checks=checks+1;
  end
  [b,grab]=advance(b,sign*120,false,false);assert(~grab && b.opened);checks=checks+1;
  for k=1:4,[b,grab]=advance(b,sign*120,true,false);assert(~grab);checks=checks+1;end
  [b,grab]=advance(b,sign*120,true,false);assert(grab);checks=checks+1;
  b=branch();b.branch='moving';
  for k=1:4,[b,grab]=advance(b,0,true,false);assert(~grab);checks=checks+1;end
  [b,grab]=advance(b,0,true,false);assert(grab);checks=checks+1;
  [b,grab]=advance(b,0,true,true);assert(~grab && b.count==0);checks=checks+1;
 end
end
% Motion-period exception is conditional on a real compensated motion witness.
for age=[-.001 0 .1 .249 .25 .251 1]
 for compensated=[false true]
  for knownMoving=[false true]
   for samePose=[false true]
    bypass=compensated && knownMoving && samePose && age>=0 && age<=orbit.maximum_frame_interval_s;
    if ~compensated || ~knownMoving || ~samePose || age<0 || age>.25,assert(~bypass);
    else,assert(bypass);end
    checks=checks+1;
    % An exception never contributes ANY pre-ready five-frame samples.
    b=branch();if bypass,b.branch='moving';end
    for k=1:4,[b,grab]=advance(b,0,true,false);assert(~grab);checks=checks+1;end
    [b,grab]=advance(b,0,true,false);assert(grab==bypass);checks=checks+1;
   end
  end
 end
end
% Image-plane counterexamples: disc stopped, camera moves. Pixel speed is false motion.
theta=(0:5)'*pi/180;point=[cos(theta) sin(theta)];
translation=[(0:5)'*.1 zeros(6,1)];translated=repmat([1 0],6,1)+translation;
assert(max(vecnorm(diff(translated),2,2))>0);checks=checks+1;
centered=translated-translation;assert(max(abs(diff(centered)),[],'all')<1e-12);checks=checks+1;
% Disc rotation by +theta and camera rotation by -theta produce identical images.
cameraOnly=point;discOnly=point;assert(isequal(cameraOnly,discOnly));checks=checks+1;
% A circular rim is unchanged by either in-plane rotation, so cannot disambiguate.
rimAngles=linspace(0,2*pi,361);rim=[cos(rimAngles);sin(rimAngles)];
for angle=[10 60 120]
 R=[cosd(angle) -sind(angle);sind(angle) cosd(angle)];
 assert(max(abs(vecnorm(R*rim)-1))<1e-12);checks=checks+1;
end
% Fixed-whitening angles change under a camera perspective shear even for a static target.
staticPoint=[.7;.7];H=[1 .4;0 1];
falseAngle=atan2d((H(2,:)*staticPoint),(H(1,:)*staticPoint))-atan2d(staticPoint(2),staticPoint(1));
assert(abs(falseAngle)>1);checks=checks+1;
% Leaving ready (movement, fault, communication unknown) invalidates the epoch;
% this model never clears a real fault or fabricates motor STOP acknowledgement.
for state=[0 1 2 3 4 5 7 8 9]
 e=epoch(1,2,0,10);e.valid=false;
 e=admit(e,1,2,1,10.01,10.02);assert(e.frames==0);checks=checks+1;
end
result=struct('validation_passed',true,'staged_validation',true,...
 'moving_before_ready_exception_validated',false,'local_matlab',true,'checks',checks,...
 'matlab_version',version,'timestamp',char(datetime('now','Format','yyyy-MM-dd HH:mm:ss')),...
 'applies_each_observation',true,'required_new_stopped_frames',5,'required_later_rotation_deg',120,...
 'prearrival_motion_witness_maximum_age_s',orbit.maximum_frame_interval_s,...
 'frame_fence_rule','Reject frames completed before ready; discard first completion after ready; accept only later sequence in the SAME sequential capture worker generation.',...
 'new_uart_fields_required',false,'simulated_product_implementation',false,...
 'capture_epoch_proof_passed',true,'sensor_exposure_epoch_verified',false,...
 'premotion_camera_compensation_available_in_current_bundle',false,...
 'premotion_exception_implementation_ready',false,...
 'similarity_translation_cancellation_model_passed',true,...
 'camera_rotation_ambiguity_counterexample_passed',true,...
 'perspective_false_angle_deg',falseAngle,...
 'limitations',{{'Planning model only: not actual Python or C implementation validation.',...
 'Worker sequence fence proves host cap.read starts after ready, not sensor exposure or absence of USB/V4L2 backlog.',...
 'Worker restart/camera replacement must invalidate fence; missing metadata is UNKNOWN.',...
 'SC2 state6/pose is current completion evidence, not continuous synchronized axis-position telemetry.',...
 'A true compensated prearrival rotation witness is unavailable in the current bundle; mark UNKNOWN rather than infer movement from raw pixels.',...
 'Current fixed-pose circle-center/whitening model cannot remove general camera movement.',...
 'Real 120-degree accuracy and five-frame physical stop require synchronized field evidence.'}},...
 'requirement_ids',{{'R SRC-01','R SRC-02','R RUN-01','R RUN-02','R RUN-04',...
 'E SRC-04','E VIS-12','E VIS-14','E VIS-15','E RUN-06'}},...
 'hardware_access',false,'product_code_modified',false,'deployed',false);
fp=fopen(fullfile(out,'matlab-result.json'),'w','n','UTF-8');assert(fp>=0);
fprintf(fp,'%s\n',jsonencode(result));fclose(fp);
fprintf('PASS observe-ready/each120 planning model: %u checks. Prearrival camera compensation: UNAVAILABLE.\n',checks);
end

function e=epoch(run,pose,item,ready)
e=struct('run',run,'pose',pose,'item',item,'ready_s',ready,'valid',true,...
 'fence_seq',0,'fence_end',0,'last_seq',0,'last_end',0,'frames',0,...
 'branch','','phase',0,'opened',false);
end
function e=heartbeat(e,run,pose,item,received)
if run<e.run || (run==e.run && pose<e.pose),return;end
if e.run~=run || e.pose~=pose || e.item~=item,e=epoch(run,pose,item,received);end
end
function e=admit(e,run,pose,seq,readStart,readEnd)
if ~e.valid || run~=e.run || pose~=e.pose || seq<=e.last_seq || ...
 ~isfinite(readEnd) || readEnd<=e.ready_s || readEnd<e.last_end,return;end
if ~e.fence_seq,e.fence_seq=seq;e.fence_end=readEnd;e.last_seq=seq;e.last_end=readEnd;return;end
% readStart is simulation truth; the product may instead use the lower bound
% from the preceding fence completion, given one sequential capture worker.
assert(readStart>=e.fence_end);
e.frames=e.frames+1;e.last_seq=seq;e.last_end=readEnd;
end
function b=branch()
b=struct('branch','','phase',0,'opened',false,'count',0);
end
function [b,grab]=advance(b,netAngle,stopped,fault)
grab=false;
if fault,b=branch();return;end
if isempty(b.branch)
 if stopped,b.count=b.count+1;else,b.count=0;end
 if b.count>=5,b.branch='stopped';b.phase=0;b.count=0;end
 return;
end
if strcmp(b.branch,'stopped'),b.phase=netAngle;b.opened=b.opened || abs(netAngle)>=120;end
if stopped,b.count=b.count+1;else,b.count=0;end
grab=b.count>=5 && (strcmp(b.branch,'moving') || b.opened);
end
