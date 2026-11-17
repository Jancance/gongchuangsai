function verify_speed_feasibility
% Mathematical transcription of actual MotionProfile_Prepare (double arithmetic).
% Rest-to-rest baseline a=150,j=300; not wheel limiting, trim, RTOS or hardware.
out=fileparts(mfilename('fullpath'));
distances=[150 300 900 960 1050 1500 1920 2100];caps=[100 200 300 400];
a=150;j=300;rows=zeros(32,8);n=0;checks=0;
for d=distances
 for cap=caps
  [dt,peak]=prepare(d,cap,a,j); signs=[1 0 -1 0 -1 0 1];
  p=0;v=0;acc=0;t=0;previous=0;
  for k=1:7
   z=linspace(0,dt(k),101);jerk=signs(k)*j;
   positions=p+v*z+.5*acc*z.^2+jerk*z.^3/6;
   speeds=v+acc*z+.5*jerk*z.^2;accelerations=acc+jerk*z;
   assert(all(isfinite([positions speeds accelerations])));checks=checks+1;
   assert(all(diff(positions)>=-1e-8)&&positions(1)>=previous-1e-8);checks=checks+1;
   assert(all(speeds>=-1e-8)&all(speeds<=cap+1e-8));checks=checks+1;
   assert(all(abs(accelerations)<=a+1e-8));checks=checks+1;
   % Boundary values are integrated forward, so compare both independent forms.
   h=dt(k);pn=p+v*h+.5*acc*h*h+jerk*h*h*h/6;
   vn=v+acc*h+.5*jerk*h*h;an=acc+jerk*h;
   assert(max(abs([positions(end)-pn speeds(end)-vn accelerations(end)-an]))<1e-9);checks=checks+1;
   previous=positions(end);p=pn;v=vn;acc=an;t=t+h;
  end
  assert(abs(p-d)<1e-7&&abs(v)<1e-8&&abs(acc)<1e-8);checks=checks+1;
  assert(abs(t-sum(dt))<1e-10&&all(dt>=0));checks=checks+1;
  assert(abs(peak-j*dt(1)*(dt(1)+dt(2)))<1e-8);checks=checks+1;
  % Ideal braking from steady cap / achieved peak, initial acceleration zero.
  stop_cap=braking(cap,a,j);stop_peak=braking(peak,a,j);
  assert(abs(stop_peak-(d-peak*dt(4))/2)<1e-7);checks=checks+1;
  n=n+1;rows(n,:)=[d cap peak t dt(4) stop_cap stop_peak double(abs(peak-cap)<1e-7)];
 end
end
% Across increasing distance, peak and total duration must be nondecreasing.
for c=1:4
 r=rows(c:4:end,:);assert(all(diff(r(:,3))>=-1e-8)&&all(diff(r(:,4))>0));checks=checks+1;
end
T=array2table(rows,'VariableNames',{'distance_mm','speed_cap_mm_s','peak_mm_s','reference_duration_s','cruise_s','ideal_brake_from_cap_mm','ideal_brake_from_peak_mm','reaches_cap'});
writetable(T,fullfile(out,'speed_feasibility.csv'));
s=struct('status','PASS_MATHEMATICAL_REFERENCE_ONLY','cases',n,'checks',checks,'acceleration_mm_s2',a,'jerk_mm_s3',j,'source','MotionProfile_Prepare seven phase formulas, double transcription','hardware_accessed',false,'actual_c_executed',false,'ground_verified',false,'rest_to_rest',true,'braking_initial_acceleration',0,'braking_final_speed',0,'short_distance_crossover_mm',caps.*(2*min(a/j,sqrt(caps/j))+max(0,caps./(j*min(a/j,sqrt(caps/j)))-min(a/j,sqrt(caps/j)))),'limits','No latency, feedback, slip, load, wheel limiting, trim, obstacle footprint or emergency dynamics modeled. Not a stopping safety proof.');
fid=fopen(fullfile(out,'speed_summary.json'),'w');fprintf(fid,'%s\n',jsonencode(s));fclose(fid);
fprintf('PASS: %d cases, %d assertions\n',n,checks);
end
function [dt,peak]=prepare(d,cap,a,j)
peak=cap;tj=min(a/j,sqrt(peak/j));ta=max(0,peak/(j*tj)-tj);minimum=peak*(2*tj+ta);tv=0;
if d>=minimum
 tv=(d-minimum)/peak;
else
 tj=a/j;
 if d>=2*a*tj*tj
  ta=.5*(sqrt(tj*tj+4*d/a)-3*tj);
 else
  tj=(d/(2*j))^(1/3);ta=0;
 end
 peak=j*tj*(tj+ta);
end
dt=[tj ta tj tv tj ta tj];
end
function distance=braking(v,a,j)
tj=min(a/j,sqrt(v/j));ta=max(0,v/(j*tj)-tj);
distance=.5*v*(2*tj+ta);
end
