function validate_source_alignment30_20261116
% Offline model only. No firmware edit, CAN operation, reset or deployment.
root=fileparts(fileparts(mfilename('fullpath')));
out=fullfile(root,'evidence','integration','2026-11-16','source-alignment30-release300');
project=fullfile(root,'tmp','source_stopped3_grip170_release200_20261115','project');
if ~exist(out,'dir'),mkdir(out);end
checks=0;
vp=fileread(fullfile(project,'USER','vehicle_parameters.h'));
cfg=fileread(fullfile(project,'CHASSIS','USER','app_config.h'));
kin=fileread(fullfile(project,'CHASSIS','USER','chassis_kinematics.h'));
route=fileread(fullfile(project,'CHASSIS','USER','motion_route_trial.h'));
auto=fileread(fullfile(project,'CHASSIS','USER','chassis_auto_test.c'));
hdr=fileread(fullfile(project,'CHASSIS','USER','chassis_auto_test.h'));
body=fileread(fullfile(project,'CHASSIS','USER','source_body_step.inc'));
profile=fileread(fullfile(project,'CHASSIS','USER','motion_profile.c'));
p=struct('wheel_diameter_mm',macro(vp,'VEHICLE_WHEEL_DIAMETER_MM'),...
 'forward_scale',macro(kin,'CHASSIS_FORWARD_DISTANCE_SCALE'),...
 'left_scale',macro(cfg,'APP_LEFT_DISTANCE_SCALE'),...
 'left_trim_ratio',macro(cfg,'APP_LEFT_BACKWARD_RATIO'),...
 'acceleration_mm_s2',macro(vp,'VEHICLE_ROUTE_ACCEL_MM_S2'),...
 'jerk_mm_s3',macro(vp,'VEHICLE_ROUTE_JERK_MM_S3'),...
 'follow_rpm',macro(route,'ROUTE_TRIAL_FOLLOW_RPM'),...
 'follow_omega',macro(cfg,'APP_DISTANCE_SCURVE_FOLLOW_OMEGA'),...
 'max_base_rpm',macro(vp,'VEHICLE_ROUTE_MAX_BASE_RPM'),...
 'existing_gap_ms',macro(hdr,'CHASSIS_AUTO_GAP_MS'),...
 'existing_fresh_ms',macro(hdr,'CHASSIS_AUTO_FRESH_MS'));
assert(contains(cfg,'#define APP_DISTANCE_SCURVE_ACCEL_MM_S2 VEHICLE_ROUTE_ACCEL_MM_S2'));
assert(contains(cfg,'#define APP_DISTANCE_SCURVE_JERK_MM_S3 VEHICLE_ROUTE_JERK_MM_S3'));
assert(contains(route,'factor*2.0f*ROUTE_TRIAL_FOLLOW_RPM*APP_DISTANCE_SCURVE_FOLLOW_OMEGA'));
assert(contains(route,'factor*6.0f*ROUTE_TRIAL_FOLLOW_RPM*APP_DISTANCE_SCURVE_FOLLOW_OMEGA*APP_DISTANCE_SCURVE_FOLLOW_OMEGA'));
assert(contains(route,'limits->acceleration/=(1.0f+APP_LEFT_BACKWARD_RATIO)'));
assert(contains(profile,'tj = powf(distance_mm / (2.0f * limits->jerk), 1.0f / 3.0f)'));
assert(contains(auto,'Auto_AllSpeedSinceLocked(now) && Auto_ZeroSpeedLocked()'));
assert(contains(body,'s_align_chain && (g_source_body_step_state==1U || g_source_body_step_state==2U)'));
checks=checks+8;
speeds=[15 30 45 60];vectors=plan(1,0,15,p);vectors=vectors([]);index=0;
for direction=0:3
 for distance=1:30
  for speed=speeds
   v=plan(distance,direction,speed,p);
   assert(v.base_rpm<=p.max_base_rpm && v.duration_s>0);
   assert(v.peak_speed<=speed*(1+1e-10));
   assert(abs(v.end_position_mm-distance)<1e-8);
   assert(abs(v.end_speed_mm_s)<1e-8 && abs(v.end_acceleration_mm_s2)<1e-8);
   assert(v.max_acceleration_mm_s2<=v.acceleration_mm_s2*(1+1e-10));
   assert(v.jerk_mm_s3>0 && v.acceleration_mm_s2>0);
   checks=checks+6;index=index+1;vectors(index)=v; %#ok<AGROW>
  end
 end
end
input=jsondecode(fileread(fullfile(out,'empirical-input.json')));
timings=struct([]);n=0;
for dataIndex=1:numel(input.sets)
 data=input.sets(dataIndex);steps=data.steps;
 baseline=zeros(1,numel(steps));residual=zeros(1,numel(steps));
 for k=1:numel(steps)
  v=plan(steps(k).distance_mm,steps(k).direction,15,p);baseline(k)=v.duration_s;
  residual(k)=steps(k).observed_state3_s-p.existing_gap_ms/1000-baseline(k);
 end
 assert(all(residual>=0));checks=checks+1;
 outside=data.observed_state2_to_state6_s-sum([steps.observed_state3_s]);
 assert(outside>=0);checks=checks+1;
 for speed=speeds
  durations=zeros(1,numel(steps));
  for k=1:numel(steps)
   v=plan(steps(k).distance_mm,steps(k).direction,speed,p);durations(k)=v.duration_s;
  end
  candidate=struct('name',data.name,'speed_mm_s',speed,...
   'observed_baseline_s',data.observed_state2_to_state6_s,...
   'observed_state3_s',[steps.observed_state3_s],...
   'baseline_reference_profile_s',baseline,'candidate_reference_profile_s',durations,...
   'retained_step_residual_s',residual,'retained_outside_step_s',outside,...
   'fixed_gap_removed_s',numel(steps)*p.existing_gap_ms/1000,...
   'model_total_s',sum(durations)+sum(residual)+outside,...
   'model_under_3s',sum(durations)+sum(residual)+outside<=3);
  n=n+1;if n==1,timings=candidate;else,timings(n)=candidate;end %#ok<AGROW>
 end
end
% An explicit two-step case uses measured first+second residuals and the full
% measured outside-step overhead. It is hypothetical, never a measured cycle.
data=input.sets(1);steps=data.steps;
for speed=speeds
 durations=zeros(1,2);baseline=zeros(1,2);residual=zeros(1,2);
 for k=1:2
  b=plan(steps(k).distance_mm,steps(k).direction,15,p);baseline(k)=b.duration_s;
  v=plan(steps(k).distance_mm,steps(k).direction,speed,p);durations(k)=v.duration_s;
  residual(k)=steps(k).observed_state3_s-1-baseline(k);
 end
 outside=data.observed_state2_to_state6_s-sum([steps.observed_state3_s]);
 n=n+1;timings(n)=struct('name','hypothetical_first_two_steps_only','speed_mm_s',speed,...
  'observed_baseline_s',NaN,'observed_state3_s',[steps(1:2).observed_state3_s],...
  'baseline_reference_profile_s',baseline,'candidate_reference_profile_s',durations,...
  'retained_step_residual_s',residual,'retained_outside_step_s',outside,...
  'fixed_gap_removed_s',2,'model_total_s',sum(durations)+sum(residual)+outside,...
  'model_under_3s',sum(durations)+sum(residual)+outside<=3); %#ok<AGROW>
end
% Predicate model for the proposed source-only GAP removal. All existing real
% STOP acknowledgements and post-stop fresh zero-speed samples remain required.
gateCases=0;
for kind=1:3
 for chain=[false true]
  for state=0:9
   fast=kind==2 && chain && (state==1 || state==2);
   if kind~=2 || ~chain || ~(state==1 || state==2),assert(~fast);
   else,assert(fast);end
   checks=checks+1;
  end
 end
end
for source=[false true]
 for elapsed=[0 1 50 999 1000 1500]
  for stopMask=0:15
   for freshMask=[0 7 14 15]
    for nonzero=[false true]
     speedsActual=zeros(1,4);if nonzero,speedsActual(4)=0.1;end
     new=settled(source,elapsed,stopMask,freshMask,speedsActual,false,p.existing_gap_ms);
     expected=(stopMask==15 && freshMask==15 && ~nonzero && (source || elapsed>=1000));
     assert(new==expected);checks=checks+1;gateCases=gateCases+1;
    end
   end
  end
 end
end
assert(~settled(true,0,15,15,zeros(1,4),true,1000));checks=checks+1;
assert(~settled(true,0,15,15,[0 0 0 NaN],false,1000));checks=checks+1;
% Four wheel samples must be at or after the actual post-STOP phase entry.
for now=[100 2^32-1 5]
 phase=mod(now-20,2^32);
 for offset=[-1 0 1 19 20]
  sample=mod(phase+offset,2^32);
  fresh=mod(now-sample,2^32)<=p.existing_fresh_ms && ...
   mod(now-sample,2^32)<=mod(now-phase,2^32);
  assert(fresh==(offset>=0));checks=checks+1;
 end
end
assert(~any([timings.model_under_3s]));checks=checks+1;
manifest=struct('path',{},'sha256',{});
paths={'USER/vehicle_parameters.h','CHASSIS/USER/app_config.h',...
 'CHASSIS/USER/chassis_kinematics.h','CHASSIS/USER/chassis_kinematics.c',...
 'CHASSIS/USER/motion_route_trial.h','CHASSIS/USER/motion_profile.c',...
 'CHASSIS/USER/chassis_auto_test.c','CHASSIS/USER/chassis_auto_test.h',...
 'CHASSIS/USER/source_body_step.inc'};
for k=1:numel(paths)
 manifest(k)=struct('path',paths{k},'sha256',sha256(fullfile(project,paths{k})));
end
result=struct('validation_passed',true,'target_3s_passed',false,...
 'local_matlab',true,'matlab_version',version,'timestamp',char(datetime('now','Format','yyyy-MM-dd HH:mm:ss')),...
 'checks',checks,'gate_cases',gateCases,'parameters',p,'speeds_compared_mm_s',speeds,...
 'selected_candidate_speed_mm_s',45,'speed_selection_reason',...
 '45 mm/s reaches the same jerk-limited reference duration as 60 for the recorded 11/11/6 and 9 mm moves; choose the lower ceiling. No actual convergence claim.',...
 'reference_vectors',vectors,'timings',timings,'source_manifest',manifest,...
 'required_rule_ids',{{'R SRC-01','R SRC-02','R RUN-01','R RUN-02','R RUN-04',...
 'E SRC-04','E VIS-14','E VIS-15','E LOC-06','E MOT-02','E MOT-04'}},...
 'limitations',{{'Empirical windows are Pi host first-seen 100 ms telemetry, not exact MCU state transitions.',...
 'Timing transfer model retains measured residual overhead; it is not a calibrated dynamic plant.',...
 'Subtracting all 1 second GAP assumes all of that interval is removable; fresh zero feedback may still take time, so savings can be smaller.',...
 'Reference profile assumes the same correction distances and number of steps; higher speed may change slip and convergence.',...
 'Outside-step overhead includes image wait, finish, wheel hold and beeps and has not been separated causally.',...
 'Five-frame material confirmation and first-piece rotation behavior are separate changes; no added time is assumed away.',...
 'No firmware implementation or actual source-only bypass predicate was tested; only its proposed logical model.',...
 'No hardware access, deployment or physical acceptance.'}},...
 'hardware_access',false,'firmware_modified',false,'deployed',false);
fp=fopen(fullfile(out,'matlab-result.json'),'w','n','UTF-8');
assert(fp>=0);fprintf(fp,'%s\n',jsonencode(result));fclose(fp);
fprintf('PASS model checks=%u; source-only predicate cases=%u. 3 SECOND TARGET: NOT PASSED.\n',checks,gateCases);
for k=1:numel(timings)
 fprintf('%s speed=%g reference_sum=%.6f retained_total=%.6f seconds\n',...
  timings(k).name,timings(k).speed_mm_s,sum(timings(k).candidate_reference_profile_s),timings(k).model_total_s);
end
end

function value=macro(text,name)
token=regexp(text,['(?m)^#define\s+' name '\s+([0-9.]+)[fU]?\s*'],'tokens','once');
assert(~isempty(token),['Missing numeric source macro ' name]);value=str2double(token{1});
end

function v=plan(distance,direction,speed,p)
factor=pi*p.wheel_diameter_mm/60;
if direction==0 || direction==3,factor=factor*p.forward_scale;end
if direction==1,factor=factor*p.left_scale;end
a=p.acceleration_mm_s2-factor*2*p.follow_rpm*p.follow_omega;
j=p.jerk_mm_s3-factor*6*p.follow_rpm*p.follow_omega^2;
if direction==1,a=a/(1+p.left_trim_ratio);j=j/(1+p.left_trim_ratio);end
base=ceil(speed/factor)+2;
peak=speed;tj=min(a/j,sqrt(peak/j));ta=max(0,peak/(j*tj)-tj);
minimum=peak*(2*tj+ta);tv=0;
if distance>=minimum,tv=(distance-minimum)/peak;
else
 tj=a/j;
 if distance>=2*a*tj^2,ta=.5*(sqrt(tj^2+4*distance/a)-3*tj);
 else,tj=(distance/(2*j))^(1/3);ta=0;end
 peak=j*tj*(tj+ta);
end
dts=[tj ta tj tv tj ta tj];signs=[1 0 -1 0 -1 0 1];
x=0;s=0;acc=0;maxa=0;
for k=1:7
 dt=dts(k);jk=signs(k)*j;
 x=x+s*dt+.5*acc*dt^2+jk*dt^3/6;
 s=s+acc*dt+.5*jk*dt^2;acc=acc+jk*dt;maxa=max(maxa,abs(acc));
end
v=struct('direction',direction,'distance_mm',distance,'speed_limit_mm_s',speed,...
 'factor_mm_per_rpm_s',factor,'base_rpm',base,'acceleration_mm_s2',a,...
 'jerk_mm_s3',j,'phase_time_s',dts,'duration_s',sum(dts),'peak_speed',peak,...
 'max_acceleration_mm_s2',maxa,'end_position_mm',x,'end_speed_mm_s',s,...
 'end_acceleration_mm_s2',acc);
end

function yes=settled(source,elapsed,stopMask,freshMask,speeds,fault,gap)
yes=~fault && stopMask==15 && freshMask==15 && all(isfinite(speeds)) && ...
 all(speeds==0) && (source || elapsed>=gap);
end

function h=sha256(path)
fp=fopen(path,'rb');data=fread(fp,Inf,'*uint8');fclose(fp);
md=java.security.MessageDigest.getInstance('SHA-256');md.update(data);
bytes=typecast(md.digest(),'uint8');h=lower(reshape(dec2hex(bytes,2).',1,[]));
end
