function simulate_pickup_three_speeds_20261110
% pzq-calibrated scheduling scenarios, not real high-speed acceptance.
root=fileparts(fileparts(mfilename('fullpath')));
out=fullfile(root,'evidence','integration','2026-11-10','speed-trials');
if ~exist(out,'dir'),mkdir(out);end
baseline=jsondecode(fileread(fullfile(root,'handoff','pzq','verification', ...
 'integration13-three-immediate-once-20261109-213859.json')));
assert(baseline.complete && abs(baseline.three_to_release_command_s-17.987)<1e-9);
trace=baseline.mcu_trace;if iscell(trace),trace=[trace{:}];end
profiles=[1846.4 106;2100 200;2500 300;2900 400];
zacc=45055;xaccs=[600 600 1200 2000];jerkRamp=.020; % engineering trial values
release=[trace(end).release_command_times_ms]/1000;
phaseStart=zeros(3,3);groups=zeros(3,2);dmTime=zeros(1,3);
for item=0:2
 for phase=1:3
  use=trace([trace.g_it_auto_item]==item & [trace.g_it_auto_phase]==phase);
  % phase_ms may be inherited on the transition sample; robust median of the
  % repeated same-phase values yields the owner's actual phase-entry tick.
  values=[use.g_it_auto_elapsed_ms]-[use.g_it_auto_phase_ms];
  phaseStart(item+1,phase)=median(values)/1000;
 end
 groups(item+1,:)=[phaseStart(item+1,3)-phaseStart(item+1,2),release(item+1)-phaseStart(item+1,3)];
 use=trace([trace.g_it_auto_item]==item & [trace.g_it_auto_phase]==3 & [trace.g_it_action]==15);
 yawStart=find(bitand([use.g_source_parallel_started_mask],1)~=0,1);
 assert(~isempty(yawStart));dmTime(item+1)=release(item+1)-double(use(yawStart).g_it_auto_elapsed_ms)/1000;
end
assert(all(groups(:)>0) && all(dmTime>0));

% Node6=0x84 in the captured status: X firmware, input Scale enabled.
% Existing pzq keeps its empirically taught mm -> command field unchanged.
% Feed the REAL command field / input scale into physical-revolution math,
% not the decoder's displayed virtual motor angle. No firmware units changed.
options=132;assert(bitand(options,2)==0 && bitand(options,128)~=0);
inputScale=100;displayToCommand=10;
xMm=[0 156 160];xRev=xMm*315.5/3200*displayToCommand/inputScale;
zDownRev=([118800-2400,118800-7200,118800-7200])/3200;
zUpRev=(118800-7200)/3200;
zFiveMmRev=8000/3200;
shapes=cell(4,1);baseCrit=zeros(3,2);results=struct([]);checks=0;
for k=1:size(profiles,1)
 zrpm=profiles(k,1);xrpm=profiles(k,2);xacc=xaccs(k);s=zeros(3,2);
 for item=1:3
  z=curve(zDownRev(item),zrpm,zacc,jerkRamp);
  x=curve(xRev(item),xrpm,xacc,0); % requested extension acceleration unchanged
  up=curve(zUpRev,zrpm,zacc,jerkRamp);
  retract=curve(xRev(item),xrpm,xacc,0);
  if k==1 % accepted pzq was native trapezoid, not an S-curve
   z=curve(zDownRev(item),zrpm,zacc,0);up=curve(zUpRev,zrpm,zacc,0);
  end
  s(item,1)=max(z.T,x.T);
  clear=at_position(up,zFiveMmRev);
  near=at_position(up,zUpRev-zFiveMmRev);
  if xMm(item)==0,xEnd=0;else,xEnd=clear+retract.T;end
  s(item,2)=max([up.T,xEnd,near+dmTime(item)]);
  for q={z,x,up,retract}
   p=q{1};assert(p.T>=0 && p.vmax<=p.limit+1e-8 && p.amax<=p.accel+1e-8);
   assert(abs(p.position(end)-p.distance)<1e-7 && abs(p.velocity(end))<1e-6);
   assert(all(diff(p.position)>=-1e-10));checks=checks+3;
  end
 end
 shapes{k}=s;
 if k==1,baseCrit=s;end
 % Fit ONLY the accepted baseline's non-kinematic preparation/feedback/settle
 % budget. It is retained in all trials, not treated as physical precision.
 delta=s-baseCrit;
 estimate=baseline.three_to_release_command_s+sum(delta(:));
 estimateZero=estimate+(baseline.three_to_zero_s-baseline.three_to_release_command_s);
 results(k).lift_rpm=zrpm;results(k).extension_rpm=xrpm;
 results(k).extension_accel_rpm_s=xacc;
 results(k).third_release_command_s=estimate;results(k).zero_complete_s=estimateZero;
 results(k).axis_critical_schedule_s=s;results(k).delta_by_group_s=delta;
 results(k).full_stroke_extension_s=[curve(xRev(2),xrpm,xacc,0).T curve(xRev(3),xrpm,xacc,0).T];
 results(k).extension_peak_rpm=[curve(xRev(2),xrpm,xacc,0).vmax curve(xRev(3),xrpm,xacc,0).vmax];
 results(k).lift_last5mm_closing_time_s=curve(zDownRev(2),zrpm,zacc,jerkRamp).T- ...
  at_position(curve(zDownRev(2),zrpm,zacc,jerkRamp),zDownRev(2)-zFiveMmRev);
end
assert(all(diff([results.third_release_command_s])<0));
assert(min(results(3).extension_peak_rpm)>299.9);
assert(min(results(4).extension_peak_rpm)>399.9);
% Smooth out/back has zero velocity at reversal and NO flat dwell. It cannot
% reverse nonzero velocity instantaneously. No servo-closure success inferred.
p=curve(zUpRev,2900,zacc,jerkRamp);
q=[p.position,zUpRev-p.position(2:end)];
assert(q(1)==0 && abs(q(end))<1e-7 && abs(max(q)-zUpRev)<1e-7);
assert(abs(p.velocity(1))<1e-9 && abs(p.velocity(end))<1e-6);
assert(bitand(132,128)~=0);

uncertainty=groups-baseCrit;
report=struct('passed',true,'checks',checks,'engine','local MATLAB', ...
 'baseline_measured_release_command_s',17.987,'baseline_measured_zero_s',19.185, ...
 'measured_group_durations_s',groups,'measured_dm_start_to_group_end_s',dmTime, ...
 'baseline_nonkinematic_residual_s',uncertainty,'results',results, ...
 'lift_accel_rpm_s',zacc,'extension_accel_rpm_s',xaccs,'s_curve_jerk_ramp_s',jerkRamp, ...
 's_curve_jerk_rpm_s2',zacc/jerkRamp,'extension_options_last_captured',options, ...
 'extension_virtual_degrees_are_not_unscaled_physical_angle',true, ...
 'physical_extension_revolutions_from_command_scale',xRev, ...
 'uncertainty_note','One-run timing calibration. Higher-speed torque,servo capture,bus prep andsettling not identified. Each extra100ms per six groups adds0.6s; not a confidence interval.', ...
 'current_driver_limit_freshly_read',false,'current_s_curve_native_supported',false, ...
 'zero_dwell_is_not_zero_velocity_reversal',true,'hardware_access',false,'field_verified',false);
fixtures=struct([]);
for k=1:3
 c=curve(zUpRev,profiles(k+1,1),zacc,jerkRamp,.00005);t=linspace(0,c.T,25);
 fixtures(k).distance_rev=zUpRev;fixtures(k).rpm=profiles(k+1,1);
 fixtures(k).time_s=t;fixtures(k).position_rev=interp1(c.time,c.position,t,'linear','extrap');
 fixtures(k).velocity_rev_s=interp1(c.time,c.velocity,t,'linear','extrap');
 fixtures(k).position_rev([1 end])=[0 zUpRev];fixtures(k).velocity_rev_s([1 end])=[0 0];
end
report.s_curve_fixtures=fixtures;
f=fopen(fullfile(out,'timing-model.json'),'w','n','UTF-8');assert(f>0);fprintf(f,'%s\n',jsonencode(report));fclose(f);
fprintf('pzq baseline:%.3fs release command;%.3fs return0.\n',17.987,19.185);
for k=2:4
 fprintf('Trial%d lift%.0f/ext%.0f(accel%.0f): scenario %.3fs release,%.3fs return0; ext peak[%.1f %.1f]RPM; closing-last5mm %.3fs.\n', ...
  k-1,results(k).lift_rpm,results(k).extension_rpm,results(k).extension_accel_rpm_s,results(k).third_release_command_s,results(k).zero_complete_s, ...
  results(k).extension_peak_rpm,results(k).lift_last5mm_closing_time_s);
end
fprintf('PASS:%d profile checks. Estimates depend on command scaling and retained baseline overhead; no hardware claim.\n',checks);
end

function p=curve(distance,rpm,accel,ramp,resolution)
% Native trapezoid or seven-phase symmetric jerk-limited profile. Units:
% revolution,second; outputs limits inRPM/RPMs for readable checking.
if nargin<5,resolution=.001;end
v=rpm/60;a=accel/60;distance=double(distance);
if distance==0,p=struct('T',0,'position',0,'velocity',0,'vmax',0,'amax',0,'limit',rpm,'accel',accel,'distance',0,'time',0);return;end
if ramp==0
 peak=min(v,sqrt(distance*a));ta=peak/a;tv=max(0,(distance-peak*ta)/peak);
 duration=[ta tv ta];jerk=[0 0 0];acc=[a 0 -a];
else
 j=a/ramp;tj=min(ramp,sqrt(v/j));ta=max(0,v/a-tj);peak=j*tj*(ta+tj);
 dc=peak*(ta+2*tj);
 if distance>=dc,tv=(distance-dc)/peak;
 else
  tv=0;
  if distance>=2*a*ramp^2,tj=ramp;ta=(-3*tj+sqrt(tj^2+4*distance/a))/2;
  else,tj=(distance/(2*j))^(1/3);ta=0;end
 end
 duration=[tj ta tj tv tj ta tj];jerk=[j 0 -j 0 -j 0 j];acc=[];
end
T=sum(duration);time=0;position=0;velocity=0;x=0;vv=0;aa=0;peakV=0;peakA=0;t=0;
for k=1:numel(duration)
 n=max(1,ceil(duration(k)/resolution));dt=duration(k)/n;
 if ramp==0,aa=acc(k);end
 for z=1:n
  jj=jerk(k);x=x+vv*dt+.5*aa*dt^2+jj*dt^3/6;vv=vv+aa*dt+.5*jj*dt^2;aa=aa+jj*dt;t=t+dt;
  position(end+1)=x;velocity(end+1)=vv;time(end+1)=t; %#ok<AGROW>
  peakV=max(peakV,abs(vv));peakA=max(peakA,abs(aa));
 end
end
p=struct('T',T,'time',time,'position',position,'velocity',velocity,'vmax',peakV*60, ...
 'amax',peakA*60,'limit',rpm,'accel',accel,'distance',distance);
end
function t=at_position(p,x)
if x<=0,t=0;elseif x>=p.distance,t=p.T;else,[q,idx]=unique(p.position);t=interp1(q,p.time(idx),x,'linear');end
end
