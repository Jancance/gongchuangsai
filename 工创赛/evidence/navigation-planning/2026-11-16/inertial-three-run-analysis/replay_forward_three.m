function replay_forward_three
out=fileparts(mfilename('fullpath'));diary(fullfile(out,'matlab_diary.log'));
s=struct('status','RUNNING','started',datestr(now,30),'matlab_version',version);
try
p=jsondecode(fileread(fullfile(out,'..','..','2026-11-15','fusion-b-mahony-comparison','frozen_plan.json')));
C=jsondecode(fileread(fullfile(out,'telemetry_context.json')));s.runs={};rows=[];
f=figure('Visible','off','Color','w','Position',[50 50 1300 950]);
for run=1:3
 c=C(run);d=jsondecode(fileread(fullfile(out,'..',c.name,'capture','decoded.json')));
 assert(d.header.raw_count==numel(d.raw) && d.header.quality==0 && d.header.frozen && ~d.header.truncated);
 assert(all(diff([d.raw.sample_sequence])==1));
 [t,a0,q0]=front(d,p,0);[tm,am,qm]=front(d,p,.5);assert(isequal(t,tm));
 [E,w,wi]=prepare_wheel(d,t,p);
 [x0,fd]=filter_intervals(t,a0,E,p.history_s,p.tau_s,[0 0]);
 [xm,fm]=filter_intervals(t,am,E,p.history_s,p.tau_s,[0 0]);
 assert(fm.too_old==0 && fm.future==0 && fm.duplicate==0 && fm.STOP_zero_observations==0);
 assert(all(isfinite([xm(:);x0(:)])));
 wheelPos=interp1(w.time,w.position(:,1),t,'linear',NaN)*1000;
 strict=find(t<=w.time(end)+1e-10,1,'last');tail=t>=t(end)-.5-1e-10;
 rawEnd=(double(d.raw(end).service_ms)-double(d.header.trigger_ms))/1000;
 hwtObserved=(double(d.hwt(end).observed_ms)-double(d.header.trigger_ms))/1000;
 hwtSource=(double(d.hwt(end).angle_ms)-double(d.header.trigger_ms))/1000;
 stop=(double(d.header.stop_ms)-double(d.header.trigger_ms))/1000;
 ends=zeros(1,4);
 for j=1:4
  ww=d.wheel(strcmp({d.wheel.source},'position') & [d.wheel.valid] & [d.wheel.wheel]==j-1);
  ends(j)=(double(ww(end).source_ms)-double(d.header.trigger_ms))/1000;
 end
 r=struct('name',c.name,'raw_count',numel(d.raw),'grid_count',numel(t),'pre_samples',qm.pre_samples,...
  'stop_request_s',stop,'raw_end_s',rawEnd,'hwt_observed_end_s',hwtObserved,'hwt_source_end_s',hwtSource,...
  'wheel_source_end_s',ends,'wheel_common_bracket_end_s',w.time(end),'replay_end_s',t(end),...
  'serial_DONE_s',c.done_s,'serial_persistent_reported_zero_rpm_s',c.persistent_reported_zero_rpm_s,...
  'BM_end_F_mm',1000*xm(end,1),'B0_end_F_mm',1000*x0(end,1),...
  'BM_end_vF_mm_s',1000*xm(end,3),'B0_end_vF_mm_s',1000*x0(end,3),...
  'BM_tail0p5_dF_mm',1000*(xm(end,1)-xm(find(tail,1),1)),...
  'BM_tail0p5_vF_min_max_mm_s',1000*[min(xm(tail,3)) max(xm(tail,3))],...
  'B0_tail0p5_dF_mm',1000*(x0(end,1)-x0(find(tail,1),1)),...
  'strict_common_grid_end_s',t(strict),'strict_common_BM_F_mm',1000*xm(strict,1),...
  'strict_common_BM_vF_mm_s',1000*xm(strict,3),'strict_common_B0_F_mm',1000*x0(strict,1),...
  'strict_common_wheel_F_mm',wheelPos(strict),'filter_BM',fm,'filter_B0',fd,'wheel',wi,...
  'ground_truth_synchronized',false,'physical_standstill_verified',false,'ruler_mm',c.ruler_mm,...
  'ruler_uncertainty_mm',[],'BM_minus_ruler_apparent_mm',[],'B0_minus_ruler_apparent_mm',[]);
 if ~isempty(c.ruler_mm),r.BM_minus_ruler_apparent_mm=1000*xm(end,1)-c.ruler_mm;r.B0_minus_ruler_apparent_mm=1000*x0(end,1)-c.ruler_mm;end
 % Pure encoder average-forward travel; separate from HWT path and BM.
 points=[t(strict) t(end) rawEnd c.done_s];means=zeros(size(points));endpointSource=zeros(1,4);
 for j=1:4
  P=c.positions(j);tt=P.time_s(:);xx=P.counts(:)*p.motor_forward_sign(j)*pi*p.wheel_diameter_mm/p.counts_per_rev*p.forward_scale;
  assert(all(diff(tt)>0));means=means+interp1(tt,xx,points,'linear',NaN)/4;
  endpointSource(j)=tt(end);
 end
 r.encoder_only=struct('description','Average signed wheel travel with frozen forward scale, bracketed source-time positions; no slip or path-heading guarantee; not BM extension',...
  'points_s',points,'average_forward_absolute_mm',means,...
  'strict_common_to_DONE_delta_mm',means(4)-means(1),'replay_end_to_DONE_delta_mm',means(4)-means(2),...
  'RAW_end_to_DONE_delta_mm',means(4)-means(3),'per_wheel_source_end_s',endpointSource);
 after=t>=stop;r.BM_stop_to_end_dF_mm=1000*(xm(end,1)-xm(find(after,1),1));
 r.BM_stop_to_end_vF_min_max_mm_s=1000*[min(xm(after,3)) max(xm(after,3))];
 r.hwt_yaw_BM_minus_B0_max_rad=max(abs(atan2(sin(qm.yaw_rad-q0.yaw_rad),cos(qm.yaw_rad-q0.yaw_rad))));
 s.runs{run}=r;
 ruler=NaN;apparent=NaN;if ~isempty(c.ruler_mm),ruler=c.ruler_mm;apparent=r.BM_minus_ruler_apparent_mm;end
 rows=[rows;run,numel(d.raw),t(end),r.BM_end_F_mm,r.BM_end_vF_mm_s,ruler,apparent,r.B0_end_F_mm,c.done_s];
 T=array2table([t,1000*xm(:,1),1000*xm(:,3),1000*x0(:,1),1000*x0(:,3),wheelPos,am(:,1),a0(:,1)],...
  'VariableNames',{'time_s','BM_F_mm','BM_vF_mm_s','B0_F_mm','B0_vF_mm_s','wheel_F_mm','BM_aF_m_s2','B0_aF_m_s2'});
 writetable(T,fullfile(out,sprintf('run%d_forward.csv',run)));
 subplot(3,2,2*run-1);plot(t,1000*xm(:,1),t,1000*x0(:,1));hold on;
 if ~isempty(c.ruler_mm),plot([0 t(end)],[c.ruler_mm c.ruler_mm],'k:');end
 xline(stop,'--','STOP request');grid on;title(sprintf('run%d forward published position',run));ylabel('mm');legend('BM','B0');
 subplot(3,2,2*run);plot(t,1000*xm(:,3),t,1000*x0(:,3));xline(stop,'--','STOP request');grid on;title('Forward velocity: RAW/HWT window ends before DONE');ylabel('mm/s');legend('BM','B0');xlabel('s');
 fprintf('run%d raw=%d replay_end=%.3f BM_F=%.9f BM_vF=%.9f B0_F=%.9f DONE=%.3f\n',run,numel(d.raw),t(end),r.BM_end_F_mm,r.BM_end_vF_mm_s,r.B0_end_F_mm,c.done_s);
end
writetable(array2table(rows,'VariableNames',{'run','raw_count','replay_end_s','BM_F_mm','BM_vF_mm_s','ruler_mm','BM_minus_ruler_apparent_mm','B0_F_mm','serial_DONE_s'}),fullfile(out,'summary.csv'));
print(f,fullfile(out,'forward_comparison.png'),'-dpng','-r120');close(f);
s.status='PASS_WITH_LIMITATIONS';s.finished=datestr(now,30);
s.interpretation='Replay completed, not an accuracy PASS. Endpoint ruler comparisons are apparent differences across unverified times. No post-capture inertial extrapolation, no STOP zero velocity, no endpoint fit, no lateral analysis.';
write_json(fullfile(out,'summary.json'),s);fprintf('FORWARD_THREE_MATLAB_PASS_WITH_LIMITATIONS\n');
catch ex
 s.status='FAIL';s.error=getReport(ex,'extended','hyperlinks','off');write_json(fullfile(out,'summary.json'),s);diary off;rethrow(ex);
end
diary off;
end

function [t,acc,diag]=front(d,p,kp,initialError)
if nargin<4,initialError=[0;0;0];end
raw=d.raw;tr=(double([raw.service_ms]')-double(d.header.trigger_ms))/1000;
assert(all(diff(tr)>0 & diff(tr)<=.020));
M=double(p.body_map);assert(norm(M*M'-eye(3))<1e-14 && det(M)>0);
body=double([[raw.ax]' [raw.ay]' [raw.az]'])*M'*p.accel_scale_m_s2_per_count;
w=double([[raw.gx]' [raw.gy]' [raw.gz]'])*M'*p.gyro_scale_deg_s_per_count*pi/180;
pre=tr>=p.calibration_pretrigger_s(1) & tr<=p.calibration_pretrigger_s(2);
assert(sum(pre)>=50);av=mean(body(pre,:),1);wb=mean(w(pre,:),1);
g0=av/norm(av)*9.80665;bias=av-g0;w=w-wb;
% Only backward differences: no future gyro sample used for alpha.
alpha=zeros(size(w));alpha(2:end,:)=diff(w)./diff(tr);
r=double(p.horizontal_lever_body_m(:));
lever=cross(alpha,repmat(r',numel(tr),1),2)+cross(w,cross(w,repmat(r',numel(tr),1),2),2);
unitZ=cross(alpha,repmat([0 0 1],numel(tr),1),2)+cross(w,cross(w,repmat([0 0 1],numel(tr),1),2),2);
observed=(double([d.hwt.observed_ms]')-double(d.header.trigger_ms))/1000;
source=(double([d.hwt.angle_ms]')-double(d.header.trigger_ms))/1000;
yaw=unwrap(double([d.hwt.yaw_deg]')*pi/180);
assert(all([d.hwt.healthy]) && all(diff(observed)>0));
initial=(double(d.header.path_heading_deg)-double(d.header.heading_offset_deg))*pi/180;
yawNow=interp1(observed,yaw,tr,'previous',NaN);yawNow(tr<observed(1))=initial;
% Body-to-level rotation from PRE gravity; yaw supplied separately by HWT.
u=g0'/norm(g0);v=cross(u,[0;0;1]);K=skew(v);
assert(1+u(3)>1e-6);R=eye(3)+K+K*K/(1+u(3));
R=rotz3(-atan2(R(2,1),R(1,1)))*R;
R0=R;R=R*exp_rotation(initialError);
ar=zeros(numel(tr),2);noLever=ar;zSensitivity=ar;tilt=zeros(numel(tr),2);fb=zeros(numel(tr),3);ya=zeros(numel(tr),1);
orth=0;gravityError=0;detError=0;
for k=1:numel(tr)
 if k>1
  f=body(k-1,:)'-bias';assert(all(isfinite(f)) && norm(f)>0);
  correction=kp*cross(f/norm(f),R'*[0;0;1]);fb(k,:)=correction';
  R=R*exp_rotation((w(k-1,:)'+correction)*(tr(k)-tr(k-1)));
 end
 R=rotz3(yawNow(k)-initial-atan2(R(2,1),R(1,1)))*R;
 world=R*(body(k,:)'-bias');center=world-R*lever(k,:)';
 % F/right outputs: body/level coordinates use F/left/up.
 ar(k,:)=[center(1),-center(2)];noLever(k,:)=[world(1),-world(2)];
 tilt(k,:)=[atan2(R(3,2),R(3,3)),asin(-R(3,1))];ya(k)=atan2(R(2,1),R(1,1));
 zz=R*unitZ(k,:)';zSensitivity(k,:)=[zz(1),-zz(2)];
 orth=max(orth,norm(R'*R-eye(3),'fro'));detError=max(detError,abs(det(R)-1));
 gravityError=max(gravityError,abs(norm(R'*[0;0;9.80665])-9.80665));
end
stop=min(tr(end),observed(end));t=(0:.005:floor(stop/.005)*.005)';
acc=interp1(tr,ar,t,'previous');staticacc=interp1(tr,noLever,t,'previous');
zs=interp1(tr,zSensitivity,t,'previous');assert(all(isfinite(acc(:))));
held=interp1(tr,tr,t,'previous');
diag=struct('pre_samples',sum(pre),'pre_accel_mean_body_m_s2',av,'gyro_bias_body_deg_s',wb*180/pi,...
 'fixed_radial_bias_body_m_s2',bias,'R0',R0,'raw_count',numel(tr),'grid_count',numel(t),...
 'grid_first_last_s',[t(1) t(end)],'first_grid_source_s',held(1),...
 'held_RAW_age_min_max_s',[min(t-held) max(t-held)],...
 'determinant_max_error',detError,'orthogonality_max_error',orth,'physical_gyro_lever_peak_m_s2',max(abs(lever)),...
 'tilt_rad',interp1(tr,tilt,t,'previous'),'feedback_rad_s',interp1(tr,fb,t,'previous'),...
 'yaw_rad',interp1(tr,ya,t,'previous'),'raw_time_s',tr,...
 'raw_tilt_rad',tilt,'raw_feedback_rad_s',fb,'raw_horizontal_accel',ar);
assert(all(isfinite([acc(:);diag.tilt_rad(:);diag.feedback_rad_s(:)])) && orth<1e-10 && detError<1e-10);
end



function R=exp_rotation(v)
a=norm(v);K=skew(v);
if a<1e-12,R=eye(3)+K+.5*K*K;else,R=eye(3)+sin(a)/a*K+(1-cos(a))/a^2*K*K;end
end


function R=rotz3(a)
R=[cos(a) -sin(a) 0;sin(a) cos(a) 0;0 0 1];
end


function [E,wheel,info]=prepare_wheel(d,t,p)
 W=d.wheel;trigger=double(d.header.trigger_ms);sgn=double(p.motor_forward_sign(:));
 src=cell(1,4);val=src;obs=src;
 lo=0;hi=t(end);
 for j=1:4
  a=W(strcmp({W.source},'position') & [W.valid] & [W.wheel]==j-1);
  tt=(double([a.source_ms]')-trigger)/1000;
  xx=double([a.value]').*(1-2*double([a.sign]'))*sgn(j)*pi*p.wheel_diameter_mm/p.counts_per_rev/1000;
  oo=(double([a.observed_ms]')-trigger)/1000;
  [tt,idx]=unique(tt,'last');xx=xx(idx);oo=oo(idx);
  assert(numel(tt)>20 && all(diff(tt)>0) && all(diff(tt)<.4));
  src{j}=tt;val{j}=xx;obs{j}=oo;
  lo=max(lo,tt(1));hi=min(hi,tt(end));
 end
 ht=(double([d.hwt.angle_ms]')-trigger)/1000;
 ho=(double([d.hwt.observed_ms]')-trigger)/1000;
 hy=unwrap(double([d.hwt.yaw_deg]')*pi/180);
 lo=max(lo,ht(1));hi=min(hi,ht(end));
 first=ceil((lo-1e-10)/.005)*.005;last=floor((hi+1e-10)/.005)*.005;
 q=(first:.005:last)';assert(abs(q(1))<1e-8,'Start must be bracketed at trigger; no missing initial segment');
 positions=zeros(numel(q),4);available=zeros(numel(q),4);
 for j=1:4
  positions(:,j)=interp1(src{j},val{j},q,'linear');
  for k=1:numel(q)
   r=find(src{j}>=q(k)-1e-10,1,'first');available(k,j)=obs{j}(r);
  end
 end
 yaw=interp1(ht,hy,q,'linear');yaw0=(d.header.path_heading_deg-d.header.heading_offset_deg)*pi/180;
 path=zeros(numel(q),2);yawAvailability=zeros(numel(q),1);
 for k=1:numel(q)
  r=find(ht>=q(k)-1e-10,1,'first');yawAvailability(k)=ho(r);
  if k==1,continue;end
  dw=positions(k,:)-positions(k-1,:);
  forward=sum(dw)*.25*p.forward_scale;left=(dw(1)-dw(2)-dw(3)+dw(4))*.25;
  half=(yaw(k)-yaw(k-1))*.5;ang=(yaw(k)+yaw(k-1))*.5-yaw0;
  if abs(half)<1e-10,sinc=1;else,sinc=sin(half)/half;end
  path(k,:)=path(k-1,:)+sinc*[forward*cos(ang)-left*sin(ang),-forward*sin(ang)-left*cos(ang)];
 end
 availability=max([available,yawAvailability],[],2);
 % Fixed nonoverlapping 200ms intervals, no synthetic high-rate wheel samples.
 E=empty_events();N=floor((q(end)-q(1)+1e-10)/p.interval_s);
 for k=1:N
  begin=(k-1)*p.interval_s;finish=k*p.interval_s;
  l=round((begin-q(1))/.005)+1;r=round((finish-q(1))/.005)+1;
  arrival=max(availability(l:r));arrival=max(arrival,finish);
  if arrival>t(end)+1e-10,continue;end
  e=struct('begin',begin,'finish',finish,'arrival',arrival,...
   'z',(path(r,:)-path(l,:))/(finish-begin),'position',path(r,:));E(end+1)=e; %#ok<AGROW>
 end
 assert(numel(E)>=30);assert(all(diff([E.arrival])>=-1e-10));
 wheel=struct('time',q,'position',path);
 info=struct('interval_s',p.interval_s,'events',numel(E),'source_start_s',q(1),'source_end_s',q(end),...
  'endpoint_delay_median_max_s',[median([E.arrival]-[E.finish]),max([E.arrival]-[E.finish])],...
  'oldest_required_history_max_s',max([E.arrival]-[E.begin]),...
  'unknown_physical_motor_delay_removed',false,'wheel_high_rate_samples_fabricated',false,...
  'position_and_velocity_fused_independently',false,...
  'interpolation','Only bracketed receipt-time wheel positions; linear interpolation assumes within-bracket motion',...
  'yaw','Bracketed source-time HWT yaw for path displacement; event waits for bracket receipt');
end


function E=empty_events
E=struct('begin',{},'finish',{},'arrival',{},'z',{},'position',{});
end


function [X,diagnostic]=filter_intervals(t,a,E,history,tau,v0)
X=zeros(numel(t),4);X(1,3:4)=v0;accepted=E([]);done=false(size(E));
H=X; % Mutable reconstruction history is separate from past published outputs.
diagnostic=struct('accepted',0,'too_old',0,'future',0,'duplicate',0,'position_corrections',0,...
 'STOP_zero_observations',0,'implementation','Offline full retrospective recomputation, not MCU fixed-memory implementation');
for k=2:numel(t)
 h=t(k)-t(k-1);H(k,1:2)=H(k-1,1:2)+h*H(k-1,3:4)+.5*h^2*a(k-1,:);
 H(k,3:4)=H(k-1,3:4)+h*a(k-1,:);
 ix=find(~done & [E.arrival]<=t(k)+1e-10);
 for j=ix
  done(j)=true;e=E(j);
  if e.finish>t(k)+1e-10,diagnostic.future=diagnostic.future+1;continue;end
  if t(k)-e.begin>=history+1e-10,diagnostic.too_old=diagnostic.too_old+1;continue;end
  if any(abs([accepted.finish]-e.finish)<1e-10),diagnostic.duplicate=diagnostic.duplicate+1;continue;end
  accepted(end+1)=e;diagnostic.accepted=diagnostic.accepted+1; %#ok<AGROW>
 end
 if ~isempty(ix),H(1:k,:)=rebuild(t(1:k),a,accepted,tau,v0);end
 X(k,:)=H(k,:);
end
end


function X=rebuild(t,a,E,tau,v0)
X=zeros(numel(t),4);X(1,3:4)=v0;
if ~isempty(E),[~,order]=sort([E.finish]);E=E(order);end
for k=2:numel(t)
 h=t(k)-t(k-1);X(k,1:2)=X(k-1,1:2)+h*X(k-1,3:4)+.5*h*h*a(k-1,:);
 X(k,3:4)=X(k-1,3:4)+h*a(k-1,:);
 for j=find(abs([E.finish]-t(k))<1e-9)
  e=E(j);b=round((e.begin-t(1))/(t(2)-t(1)))+1;
  dt=e.finish-e.begin;pred=(X(k,1:2)-X(b,1:2))/dt;
  X(k,3:4)=X(k,3:4)+(1-exp(-dt/tau))*(e.z-pred);
 end
end
end


function K=skew(v)
K=[0 -v(3) v(2);v(3) 0 -v(1);-v(2) v(1) 0];
end


function write_json(path,value)
fid=fopen(path,'w');assert(fid>=0);closer=onCleanup(@()fclose(fid));
fprintf(fid,'%s\n',jsonencode(value));
end

