function stop_distance_study()
% Saved-data diagnosis. No fitted plant and no candidate acceptance by bounds.
out=fileparts(mfilename('fullpath'));
in=jsondecode(fileread(fullfile(out,'matlab_inputs.json'))); s=in.spec;
factor=single(single(pi)*single(s.wheel_diameter_mm)/single(60)*single(s.forward_scale));
v=min(single(s.requested_speed_mm_s),factor*single(s.max_rpm-s.follow_cap_rpm));
a=single(s.requested_acceleration_mm_s2)-factor*single(2*s.follow_cap_rpm*s.follow_omega);
j=single(s.requested_jerk_mm_s3)-factor*single(6*s.follow_cap_rpm*s.follow_omega^2);
p=prepare(single(s.distance_mm),v,a,j);
t=(0:1:ceil(double(p.duration)*1000))'/1000;
[pos,vel,acc,jerk,phase]=sample(p,t);
assert(abs(double(pos(end))-s.distance_mm)<.001);
assert(max(abs(double(acc)))<=double(a)+.001);
assert(max(abs(double(jerk)))<=double(j)+.001);
% Exact critically damped ZOH propagation against analytic constant input.
check=0; maxerr=0;
for dt=[.01 .025 .1]
    state=[0;0]; u=2;
    for k=1:100
        state=follow(state,u,dt,1);
        exact=[u*(1-(1+k*dt)*exp(-k*dt));u*k*dt*exp(-k*dt)];
        maxerr=max(maxerr,max(abs(state-exact)));check=check+1;
    end
end
assert(maxerr<1e-12);
% Common-time compensation cancels current position algebraically.
cancel=0;
for nowpos=[100 1000 2090]
 for obspos=[90 980 2080]
  for observedMean=[100 990 2104]
   cancel=max(cancel,abs((nowpos-(observedMean+nowpos-obspos))-(obspos-observedMean)));
  end
 end
end
assert(cancel<1e-10);
res=struct('schema','stop-distance-study-results-v1','matlab_version',version,...
 'status','B0_FAIL_NOT_IDENTIFIABLE','B1_executed',false,...
 'factor_mm_per_rpm_s',double(factor),'profile_speed_mm_s',double(v),...
 'profile_acceleration_mm_s2',double(a),'profile_jerk_mm_s3',double(j),...
 'profile_duration_s',double(p.duration),'profile_boundary_s',double(p.bound),...
 'profile_endpoint_mm',double(pos(end)),'follow_analytic_checks',check,...
 'follow_analytic_max_error',maxerr,'common_compensation_identity_max_error',cancel,...
 'baseline_reserved_total_acceleration_mm_s2',double(a)+double(factor)*4,...
 'baseline_reserved_total_jerk_mm_s3',double(j)+double(factor)*12);
rows=[]; tail=[];
for n=1:4
 r=in.runs(n); b=r.summary; xd=r.xd1;
 peMean=0;
 for w=1:4
  delta=double(r.pe3.(sprintf('end_raw%d',w)))*(1-2*double(r.pe3.(sprintf('end_sign%d',w))))-double(r.pe3.(sprintf('start_raw%d',w)))*(1-2*double(r.pe3.(sprintf('start_sign%d',w))));
  peMean=peMean+delta*(1-2*mod(w-1,2))*pi*s.wheel_diameter_mm/65536/4*s.forward_scale;
 end
 assert(abs(peMean-b.final_encoder_from_PE_raw_calibrated_mm)<1e-6);
 telemetryFinal=r.tmq(end).distance_progress_milli_mm/1000;
 assert(abs(telemetryFinal-b.software_arrival_progress_mm)<.001);
 batches=readtable(fullfile(out,'..','baseline40-field',sprintf('s25-run%d',n),'command_batches.csv'));
 last=batches(end,:); % incomplete final TX is deliberately not promoted to full batch
 assert(last.F_rpm==b.last_complete_batch_F_RPM);
 firsttx=last.first_ack_ms-1; % verified first TX lookup below, not a generic ACK assumption
 tx=find(strcmp({xd.kind},'target_tx') & [xd.tick_ms]<=last.first_ack_ms & [xd.address]==1,1,'last');
 firsttx=xd(tx).tick_ms;
 lo=batches.last_ack_ms(end-1);hi=firsttx;
 elapsed_lo=(lo-r.run_tick_upper_ms)/1000;
 elapsed_hi=(hi-r.run_tick_lower_ms)/1000;
 [pp,ss,~,~,ph]=sample(p,[elapsed_lo elapsed_hi]);
 reflo=min(double(ss))/double(factor);refhi=max(double(ss))/double(factor);
 % This is an outer reachable set only, never a reconstructed follower state.
 Flo=max(1,floor(reflo-s.follow_cap_rpm+.5));Fhi=min(s.max_rpm,floor(refhi+s.follow_cap_rpm+.5));
 atstop=[(b.first_STOP_ACK_ms-r.run_tick_upper_ms)/1000,(b.first_STOP_ACK_ms-r.run_tick_lower_ms)/1000];
 [stoppos,stopv]=sample(p,atstop);
 stopref=double(stopv)/double(factor);
 movingpm=r.pm1([r.pm1.tick_ms]<=b.first_STOP_ACK_ms & [r.pm1.tick_ms]>=r.run_tick_lower_ms);
 latest=movingpm(end); common=min([latest.sample_ms1 latest.sample_ms2 latest.sample_ms3 latest.sample_ms4]);
 % Mean interpolated using *saved* PM snapshots: demonstrate availability,
 % not the original unrecorded 4-slot firmware interpolation history.
 commonmean=0;ok=true;start=zeros(1,4);wheels=zeros(1,4);
 for w=1:4
  sign0=b.PE_endpoint.(sprintf('start_sign%d',w));raw0=b.PE_endpoint.(sprintf('start_raw%d',w));
  start(w)=double(raw0)*(1-2*double(sign0));
  ts=arrayfun(@(x)double(x.(sprintf('sample_ms%d',w))),movingpm);
  rr=arrayfun(@(x)double(x.(sprintf('raw%d',w)))*(1-2*double(x.(sprintf('sign%d',w)))),movingpm);
  [ts,ii]=unique(ts);rr=rr(ii); vals=(rr-start(w))*(1-2*mod(w-1,2))*pi*s.wheel_diameter_mm/65536;
  if common<min(ts)||common>max(ts);ok=false;else;wheels(w)=interp1(ts,vals,common);end
 end
 if ok;commonmean=mean(wheels)*s.forward_scale;else;commonmean=NaN;end
 [obspos,~]=sample(p,[(common-r.run_tick_upper_ms)/1000,(common-r.run_tick_lower_ms)/1000]);
 q=struct('run',n,'run_tick_lower_ms',r.run_tick_lower_ms,'run_tick_upper_ms',r.run_tick_upper_ms,...
  'run_tick_is_observed',false,'last_complete_F_rpm',last.F_rpm,'last_freeze_time_outer_lower_ms',lo,...
  'last_freeze_time_outer_upper_ms',hi,'last_elapsed_s_lower',elapsed_lo,'last_elapsed_s_upper',elapsed_hi,...
  'last_phase_possible',unique(double(ph))','last_reference_position_mm_bounds',sort(double(pp))',...
  'last_reference_F_rpm_bounds',[reflo refhi],'last_F_outer_bounds_with_unknown_follow',[Flo Fhi],...
  'last_F_fits_outer_bounds',last.F_rpm>=Flo&&last.F_rpm<=Fhi,...
  'last_F_reconstruction_error_rpm',NaN,'last_F_reconstruction_tolerance_pass',false,...
  'internal_follow_state_identifiable',false,'stop_ack_elapsed_s_bounds',atstop,...
  'stop_ack_reference_position_mm_bounds',sort(double(stoppos))',...
  'stop_ack_reference_F_rpm_bounds',sort(stopref)',...
  'arrival_common_lead_lower_bound_mm',b.software_arrival_progress_mm-max(double(stoppos)),...
  'STOP_ACK_before_profile_end_at_least_s',double(p.duration)-max(atstop),...
  'latest_PM_tick_ms',latest.tick_ms,'latest_PM_common_ms',common,...
  'saved_PM_interpolated_common_mean_mm',commonmean,...
  'observed_reference_position_at_saved_PM_common_bounds_mm',sort(double(obspos))',...
  'saved_PM_common_mean_minus_frozen_mm',commonmean-b.software_arrival_progress_mm,...
  'missing_position_updates_lower_bound',r.missing_position_updates_lower_bound,...
  'frozen_arrival_mm',b.software_arrival_progress_mm,'PE_final_calibrated_mm',peMean,...
  'ruler_final_forward_mm',b.ruler_forward_mm,...
  'PE_minus_frozen_mm',b.final_encoder_minus_frozen_progress_mm,...
  'PE_after_first_STOP_ACK_ms',b.PE_end_time_from_firstSTOP_ms,...
  'measured_ground_slip_mm',NaN);
 if isempty(rows);rows=q;else;rows(n)=q;end
 % Export exact time-reference family against TMQ; telemetry elapsed ignored.
 tm=r.tmq;
 for z=1:numel(tm)
  if tm(z).tick_ms<r.run_tick_lower_ms || tm(z).tick_ms>b.first_STOP_ACK_ms;continue;end
  el=[(tm(z).tick_ms-r.run_tick_upper_ms)/1000,(tm(z).tick_ms-r.run_tick_lower_ms)/1000];
  [rp,rv,~,~,ph]=sample(p,max(0,el));
  tail=[tail; n tm(z).tick_ms tm(z).elapsed_ms el min(double(rp)) max(double(rp)) min(double(rv))/double(factor) max(double(rv))/double(factor) tm(z).distance_progress_milli_mm/1000 tm(z).distance_base_rpm min(double(ph))]; %#ok<AGROW>
 end
end
res.runs=rows;
res.reason='No observed internal run_tick/follow_tick/rpm/rate, missing PM history and batch-freeze/progress state; outer-bound consistency is not <=1RPM replay or arrival reconstruction. B1 not run.';
writejson(fullfile(out,'matlab_results.json'),res);
names={'run','tick_ms','external_elapsed_ms_NOT_profile','profile_elapsed_lower_s','profile_elapsed_upper_s','reference_position_lower_mm','reference_position_upper_mm','reference_F_lower_rpm','reference_F_upper_rpm','TMQ_frozen_common_mean_mm','TMQ_base_rpm','profile_phase_min'};
writetable(array2table(tail,'VariableNames',names),fullfile(out,'reference_time_bounds.csv'));
writetable(table(t,double(pos),double(vel),double(acc),double(jerk),double(phase),'VariableNames',{'elapsed_s','position_mm','speed_mm_s','acceleration_mm_s2','jerk_mm_s3','phase'}),fullfile(out,'baseline_profile_1ms.csv'));
fprintf('MATLAB %s\n',version);fprintf('B0_FAIL_NOT_IDENTIFIABLE; B1_executed=0\n');
fprintf('profile duration=%.9f s factor=%.9f mm/(RPM*s), a=%.6f j=%.6f\n',p.duration,factor,a,j);
for n=1:4;q=rows(n);fprintf('run%d F=%g ref=[%.3f %.3f] outerF=[%g %g] missingPM>=%d PMfreezeGap=%.3fmm\n',n,q.last_complete_F_rpm,q.last_reference_F_rpm_bounds,q.last_F_outer_bounds_with_unknown_follow,q.missing_position_updates_lower_bound,q.saved_PM_common_mean_minus_frozen_mm);end
fprintf('applicable mathematical diagnostics passed; product candidate NOT accepted\n');
end

function p=prepare(d,v,a,j)
tj=min(a/j,sqrt(v/j));ta=max(single(0),v/(j*tj)-tj);tv=single(0);
if d>=v*(2*tj+ta);tv=(d-v*(2*tj+ta))/v;
else;tj=a/j;if d>=2*a*tj*tj;ta=single(.5)*(sqrt(tj*tj+4*d/a)-3*tj);else;tj=(d/(2*j))^single(1/3);ta=single(0);end;end
dur=[tj ta tj tv tj ta tj];sg=single([1 0 -1 0 -1 0 1]);
p=struct('bound',zeros(1,8,'single'),'position',zeros(1,7,'single'),'speed',zeros(1,7,'single'),'acc',zeros(1,7,'single'),'jerk',sg*j,'distance',d,'duration',single(0));
x=0;vv=0;aa=0;tt=0;
for k=1:7
 p.bound(k)=single(tt);p.position(k)=single(x);p.speed(k)=single(vv);p.acc(k)=single(aa);
 dt=double(dur(k));jj=double(p.jerk(k));x=x+vv*dt+.5*aa*dt^2+jj*dt^3/6;vv=vv+aa*dt+.5*jj*dt^2;aa=aa+jj*dt;tt=tt+dt;
end
p.duration=single(tt);p.bound(8)=single(tt);
end

function [x,v,a,j,phase]=sample(p,t)
t=single(t(:));x=zeros(size(t),'single');v=x;a=x;j=x;phase=zeros(size(t));
for k=1:numel(t)
 if t(k)>=p.duration;x(k)=p.distance;phase(k)=8;continue;end
 ph=find(t(k)<p.bound(2:end),1);dt=t(k)-p.bound(ph);
 x(k)=p.position(ph)+p.speed(ph)*dt+single(.5)*p.acc(ph)*dt*dt+p.jerk(ph)*dt*dt*dt/single(6);
 v(k)=p.speed(ph)+p.acc(ph)*dt+single(.5)*p.jerk(ph)*dt*dt;
 a(k)=p.acc(ph)+p.jerk(ph)*dt;j(k)=p.jerk(ph);phase(k)=ph;
end
end

function state=follow(state,u,dt,w)
y=state(1)-u;b=state(2)+w*y;decay=exp(-w*dt);
state=[u+(y+b*dt)*decay;(state(2)-w*b*dt)*decay];
end

function writejson(path,value)
f=fopen(path,'w','n','UTF-8');assert(f>=0);c=onCleanup(@()fclose(f));fwrite(f,jsonencode(value),'char');
end
