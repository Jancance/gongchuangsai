function validate_right_short
out=fileparts(mfilename('fullpath'));diary(fullfile(out,'matlab_diary.log'));
started=datestr(now,30); f=pi*77/60;A=150-f*4;J=300-f*12;cap=102;
v=min(f*(cap-2),400);tj=A/J;ta=v/A-tj;min_distance=v*(2*tj+ta);
assert(A>0 && J>0 && ta>0 && v==400);results=[];cases=0;maxw=0;
for base=0:cap
 for req=[-2 0 2]
  h=max(-min(base,2),min(min(base,2),req));w=base*[-1 1 1 -1]+h*[-1 1 -1 1];
  assert(abs(mean(w.*[-1 1 1 -1])-base)<1e-10);
  assert(all(abs(w)<=cap+2) && all(abs(w)<130));
  if base==0,assert(all(w==0));end
  cases=cases+1;maxw=max(maxw,max(abs(w)));
 end
end
for distance=[2100 1500]
 tv=(distance-min_distance)/v;assert(tv>0);
 d=[tj ta tj tv tj ta tj];b=[0 cumsum(d)];js=J*[1 0 -1 0 -1 0 1];x=zeros(8,3);
 for k=1:7
  dt=d(k);p=x(k,1);s=x(k,2);a=x(k,3);j=js(k);
  x(k+1,:)=[p+s*dt+.5*a*dt^2+j*dt^3/6,s+a*dt+.5*j*dt^2,a+j*dt];
 end
 assert(abs(x(end,1)-distance)<1e-8 && max(abs(x(end,2:3)))<1e-9);
 assert(b(end)+2<12.779 && abs(x(4,2)-400)<1e-9);
 assert(A+f*4<=150+1e-9 && J+f*12<=300+1e-9);
 t=unique([(0:.002:b(end))';b']);trace=zeros(numel(t),6);
 for n=1:numel(t)
  k=find(t(n)<b(2:end),1);
  if isempty(k),p=distance;s=0;a=0;j=0;else
   dt=t(n)-b(k);j=js(k);p=x(k,1)+x(k,2)*dt+.5*x(k,3)*dt^2+j*dt^3/6;
   s=x(k,2)+x(k,3)*dt+.5*j*dt^2;a=x(k,3)+j*dt;
  end
  trace(n,:)=[t(n) p s a j round(s/f)];
 end
 assert(all(diff(trace(:,2))>=-1e-9) && max(trace(:,3))>=400-1e-9);
 assert(max(trace(:,6))<=cap && all(trace(end,3:6)==0));
 writematrix(trace,fullfile(out,sprintf('right%d_profile_trace.csv',distance)));
 results=[results struct('distance_mm',distance,'reference_mm_s',v,'A',A,'J',J,'tj',tj,'ta',ta,...
 'cruise_s',tv,'duration_s',b(end),'duration_plus_settle_s',b(end)+2,'run_limit_s',12.779,...
 'max_wheel_rpm_bound',maxw,'nominal_reference_peak_rpm',max(trace(:,6)),'integer_cases',cases)];
end
assert(abs(results(1).duration_s-results(2).duration_s-1.5)<1e-10);
assert(results(1).A==results(2).A && results(1).J==results(2).J && results(1).tj==results(2).tj && results(1).ta==results(2).ta);
summary=struct('status','PASS_WITH_LIMITATIONS','started',started,'finished',datestr(now,30),'matlab_version',version,...
 'minimum_distance_for_400_mm',min_distance,'results',results,'hardware_accessed',false,'parameters_fitted',false,...
 'scope','Identical reference400/A/J/accel-decel shape;1500cruise only0.230s. Not proof of equal ground speed, slip or distance accuracy.');
fid=fopen(fullfile(out,'matlab_summary.json'),'w');assert(fid>=0);fwrite(fid,jsonencode(summary),'char');fclose(fid);
disp(summary);disp('RIGHT150_REFERENCE_COMPARISON_PASS_WITH_LIMITATIONS');diary off
end
