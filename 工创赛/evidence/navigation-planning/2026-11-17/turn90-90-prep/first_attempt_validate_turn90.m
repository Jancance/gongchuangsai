function validate_turn90
out=fileparts(mfilename('fullpath'));diary(fullfile(out,'matlab_diary.log'));
S=struct('status','RUNNING','matlab_version',version,'hardware_access',false,'angle_deg',90,'reference_peak_deg_s',90,'accel_deg_s2',180,'jerk_deg_s3',720,'cases',{{}});
try
[P,B,X,J]=profile(90,90,180,720);[old,~,~,~]=profile(90,90,15,60);
assert(abs(P.peak-90)<1e-9 && P.cruise_s>0 && old.peak<90);
S.profile=P;S.old_limits_profile=old;S.geometry_nominal_deg_s_per_rpm=6*77/418;S.ideal_peak_rpm=90/S.geometry_nominal_deg_s_per_rpm;S.driver_peak_0p1rpm=floor(S.ideal_peak_rpm*10)/10;S.driver_nominal_peak_deg_s=S.driver_peak_0p1rpm*S.geometry_nominal_deg_s_per_rpm;
t=(0:.001:P.duration)';v=zeros(numel(t),4);for i=1:numel(t),v(i,:)=sample(t(i),P,B,X,J);end
assert(max(abs(v(:,2)))<=90+1e-9 && max(abs(v(:,3)))<=180+1e-9 && max(abs(v(:,4)))<=720+1e-9);
assert(abs(v(end,1)-90)<1e-8 && max(abs(v(end,2:3)))<1e-8);
writematrix([t,v],fullfile(out,'angular_reference.csv'));
idx=0;
for tau=[.04 .08 .15]
for delay=[0 .044 .088]
for gain=[.9 1 1.1]
for direction=[-1 1]
 idx=idx+1;dt=.005;theta=0;rate=0;command=0;stable=0;done=false;failed=false;history=zeros(3001,6);observer_ticks=4;lag_ticks=round(delay/dt);last_quant=0;
 for n=1:3001
  now=(n-1)*dt;obsidx=max(1,n-observer_ticks);oy=history(obsidx,2);ov=history(obsidx,3);
  ref=sample(now,P,B,X,J);error=ref(1)-direction*oy;
  desired=ref(2)+4*(error-.08*direction*ov)+.3*(ref(2)-direction*ov);
  desired=max(0,min(90,desired));
  if now>=P.duration,desired=max(0,min(15,2*(90-direction*oy)-.3*direction*ov));end
  requested=direction*floor(desired/(6*77/418)*10+1e-6)/10;
  if mod(n-1,9)==0,last_quant=requested;end
  command=last_quant;
  history(n,:)=[now theta rate command ref(1) ref(2)];
  if abs(90-direction*oy)<=.8 && abs(ov)<=1 && now>=P.duration,stable=stable+dt;else,stable=0;end
  if stable>=.7,done=true;break;end
  if direction*oy>91 || direction*oy< -2,failed=true;break;end
  src=max(1,n-lag_ticks);target=history(src,4)*(6*77/418)*gain;
  rate=rate+(target-rate)*(1-exp(-dt/tau));theta=theta+rate*dt;
 end
 h=history(1:n,:);r=struct('tau_s',tau,'command_delay_s',delay,'gain',gain,'direction',direction,'done',done,'failed',failed,'time_s',now,'final_error_deg',90-direction*theta,'peak_actual_model_deg_s',max(abs(h(:,3))),'peak_command_nominal_deg_s',max(abs(h(:,4)))*(6*77/418),'max_overshoot_deg',max(direction*h(:,2)-90));
 S.cases{idx}=r;
end
end
end
end
S.completed=sum(cellfun(@(r)r.done,S.cases));S.failed=sum(cellfun(@(r)r.failed,S.cases));
S.status='PASS_WITH_LIMITATIONS';
catch e
 S.status='FAIL';S.error=getReport(e,'extended','hyperlinks','off');savejson(out,S);diary off;rethrow(e);
end
savejson(out,S);disp(S);diary off;
end
function [P,B,X,J]=profile(D,V,A,K)
tj=min(A/K,sqrt(V/K));ta=max(0,V/(K*tj)-tj);peak=V;tv=0;
if D>=peak*(2*tj+ta),tv=(D-peak*(2*tj+ta))/peak;else
 tj=A/K;if D>=2*A*tj*tj,ta=.5*(sqrt(tj*tj+4*D/A)-3*tj);else,tj=(D/(2*K))^(1/3);ta=0;end
 peak=K*tj*(tj+ta);
end
d=[tj ta tj tv tj ta tj];B=[0 cumsum(d)];J=K*[1 0 -1 0 -1 0 1];X=zeros(8,3);
for i=1:7,t=d(i);x=X(i,:);j=J(i);X(i+1,:)=[x(1)+x(2)*t+x(3)*t^2/2+j*t^3/6,x(2)+x(3)*t+j*t^2/2,x(3)+j*t];end
P=struct('distance',D,'peak',peak,'duration',B(end),'cruise_s',tv);
end
function v=sample(t,P,B,X,J)
if t>=P.duration,v=[P.distance 0 0 0];return;end
k=find(t<B(2:end),1);dt=t-B(k);x=X(k,:);j=J(k);v=[x(1)+x(2)*dt+x(3)*dt^2/2+j*dt^3/6,x(2)+x(3)*dt+j*dt^2/2,x(3)+j*dt,j];
end
function savejson(out,S)
fid=fopen(fullfile(out,'matlab_summary.json'),'w','n','UTF-8');fwrite(fid,jsonencode(S),'char');fclose(fid);
end
