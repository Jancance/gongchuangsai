function validate_right30
out=fileparts(mfilename('fullpath')); diary(fullfile(out,'matlab_diary.log'));
started=datestr(now,30); f=pi*77/60; A=150-f*4; J=300-f*12; V=400; cap=102;
assert(A>0 && J>0 && min(f*(cap-2),400)==400);
results=[]; signs=[1 0 -1 0 -1 0 1]; rpm_cases=0; maxwheel=0;
for base=0:cap
 for hreq=[-2 0 2]
  h=max(-min(base,2),min(min(base,2),hreq));w=base*[-1 1 1 -1]+h*[-1 1 -1 1];
  assert(abs(mean(w.*[-1 1 1 -1])-base)<1e-10);
  assert(max(abs(w))<=104 && max(abs(w))<130);
  if base==0,assert(all(w==0));end
  rpm_cases=rpm_cases+1;maxwheel=max(maxwheel,max(abs(w)));
 end
end
for distance=[300 1500 2100]
 % Same branches and equations as actual motion_profile.c:87-107.
 tj=min(A/J,sqrt(V/J));ta=max(0,V/(J*tj)-tj);peak=V;md=peak*(2*tj+ta);tv=0;
 if distance>=md
  tv=(distance-md)/peak;branch='cruise';
 else
  tj=A/J;
  if distance>=2*A*tj*tj
   ta=.5*(sqrt(tj*tj+4*distance/A)-3*tj);branch='no_cruise_with_constant_acceleration';
  else
   tj=(distance/(2*J))^(1/3);ta=0;branch='jerk_only';
  end
  peak=J*tj*(tj+ta);
 end
 d=[tj ta tj tv tj ta tj];b=[0 cumsum(d)];js=J*signs;x=zeros(8,3);
 for k=1:7
  dt=d(k);p=x(k,1);s=x(k,2);a=x(k,3);j=js(k);
  x(k+1,:)=[p+s*dt+.5*a*dt^2+j*dt^3/6,s+a*dt+.5*j*dt^2,a+j*dt];
 end
 assert(all(d>=0) && abs(x(end,1)-distance)<1e-8 && max(abs(x(end,2:3)))<1e-9);
 assert(b(end)+2<12.779 && peak<=400 && max(abs(x(:,3)))<=A+1e-9);
 assert(A+f*4<=150+1e-9 && J+f*12<=300+1e-9);
 t=unique([(0:.002:b(end))';b']);trace=zeros(numel(t),8);
 for n=1:numel(t)
  k=find(t(n)<b(2:end),1);
  if isempty(k),p=distance;s=0;a=0;j=0;else
   dt=t(n)-b(k);j=js(k);p=x(k,1)+x(k,2)*dt+.5*x(k,3)*dt^2+j*dt^3/6;
   s=x(k,2)+x(k,3)*dt+.5*j*dt^2;a=x(k,3)+j*dt;
  end
  nominal=round(s/f);maxbase=nominal+2;maxwheelcase=maxbase+2;
  if s==0,maxbase=0;maxwheelcase=0;end
  trace(n,:)=[t(n) p s a j nominal maxbase maxwheelcase];
 end
 assert(all(diff(trace(:,2))>=-1e-9));
 assert(max(trace(:,3))<=peak+1e-8 && max(abs(trace(:,4)))<=A+1e-8 && max(abs(trace(:,5)))<=J+1e-8);
 assert(max(trace(:,7))<=102 && max(trace(:,8))<=104 && all(trace(end,3:8)==0));
 if distance==300,assert(tv==0 && peak<400 && ta>0);else,assert(tv>0 && abs(peak-400)<1e-9);end
 % Float operations in C branch, then its double phase integration: endpoint tolerance.
 af=single(A);jf=single(J);vf=single(V);df=single(distance);
 tjf=min(af/jf,sqrt(vf/jf));taf=max(single(0),vf/(jf*tjf)-tjf);pf=vf;tvf=single(0);
 if df>=pf*(2*tjf+taf),tvf=(df-pf*(2*tjf+taf))/pf;else
  tjf=af/jf;
  if df>=2*af*tjf*tjf,taf=single(.5)*(sqrt(tjf*tjf+4*df/af)-3*tjf);else,tjf=(df/(2*jf))^(single(1)/single(3));taf=single(0);end
  pf=jf*tjf*(tjf+taf);
 end
 fd=double([tjf taf tjf tvf tjf taf tjf]);fx=[0 0 0];
 for k=1:7
  dt=fd(k);j=double(jf)*signs(k);p=fx(1);s=fx(2);a=fx(3);
  fx=[p+s*dt+.5*a*dt^2+j*dt^3/6,s+a*dt+.5*j*dt^2,a+j*dt];
 end
 assert(abs(fx(1)-distance)<=.002+distance*.00001 && abs(fx(2))<1e-3 && abs(fx(3))<1e-3);
 writematrix(trace,fullfile(out,sprintf('right%d_profile_trace.csv',distance)));
 r=struct('distance_mm',distance,'speed_upper_bound_mm_s',V,'actual_reference_peak_mm_s',peak,...
 'effective_A_mm_s2',A,'effective_J_mm_s3',J,'tj_s',tj,'ta_s',ta,'tv_s',tv,'branch',branch,...
 'duration_s',b(end),'duration_plus_settle_s',b(end)+2,'run_limit_ms',12779,...
 'endpoint_mm',x(end,1),'end_v_mm_s',x(end,2),'end_a_mm_s2',x(end,3),...
 'nominal_reference_peak_rpm',max(trace(:,6)),'max_follow_base_rpm',max(trace(:,7)),...
 'max_follow_plus_heading_rpm',max(trace(:,8)),'discrete_budget_cases',rpm_cases,...
 'single_branch_peak_mm_s',double(pf),'single_branch_duration_s',sum(fd),'single_endpoint_error_mm',fx(1)-distance);
 results=[results r];
end
assert(abs(results(2).duration_s-results(3).duration_s+1.5)<1e-10);
summary=struct('status','PASS_WITH_LIMITATIONS','started',started,'finished',datestr(now,30),...
 'matlab_version',version,'matlab_executable_expected','E:/Program Files/Polyspace/R2020a/bin/matlab.exe',...
 'factor_mm_s_per_rpm',f,'minimum_distance_for_400_mm',md,'results',results,...
 'hardware_accessed',false,'product_written',false,'parameters_fitted',false,...
 'trace_columns',{{'t_s','x_mm','v_mm_s','a_mm_s2','j_mm_s3','nominal_rpm','follow_base_bound_rpm','follow_heading_bound_rpm'}},...
 'scope','Actual generic C branch reproduced;300mm speed upper bound400 but lower mathematical peak. Not equal-speed ground calibration or hardware acceptance.');
fid=fopen(fullfile(out,'matlab_summary.json'),'w');assert(fid>=0);fwrite(fid,jsonencode(summary),'char');fclose(fid);
disp(summary);disp('RIGHT30_GENERIC_PROFILE_MATLAB_PASS_WITH_LIMITATIONS');diary off
end
