function matlab_ff_trial
% R2016b-compatible offline, data-calibrated mathematical model.
% Not compiled vehicle C; no serial, DAQ, external mode or hardware calls.
out=fileparts(mfilename('fullpath'));cd(out);diary('matlab_run.log');
fprintf('START MATLAB_FF %s\n',version);
C=csvread('commands.csv',1,0);S=csvread('samples.csv',1,0);L=csvread('telemetry.csv',1,0);
review=jsondecode(fileread('matlab_metadata.json'));
fits=struct([]);profile=[];validation=[];
taus=[.02 .04 .06 .08 .10 .15 .20 .30 .40 .60];
delays=0:.02:.30;
fig=figure('Visible','off','Color','w','Position',[50 50 1300 850]);
for train=1:2
 test=3-train;
 for w=1:4
  c=C(C(:,1)==train & C(:,2)==w,3:4);
  s=S(S(:,1)==train & S(:,2)==w,:);
  mask=s(:,5)>=3 & (s(:,9)==5 | s(:,9)==6);
  best=Inf;scan=[];
  for tau=taus
   for delay=delays
    X=response_basis(c,s(:,3),tau,delay);
    A=X(mask,:);y=s(mask,4);beta=A\y;
    if beta(1)<.7 || beta(1)>1.3 || abs(beta(2))>5,continue,end
    mse=mean((A*beta-y).^2);
    scan=[scan;train w tau delay beta(1) beta(2) mse]; %#ok<AGROW>
    if mse<best,best=mse;bt=tau;bd=delay;bb=beta;end
   end
  end
  assert(isfinite(best),'No admissible wheel model');
  near=scan(scan(:,7)<=best*1.1+.01,:);
  ct=C(C(:,1)==test & C(:,2)==w,3:4);
  st=S(S(:,1)==test & S(:,2)==w,:);
  mt=st(:,5)>=3 & (st(:,9)==5 | st(:,9)==6);
  pred=response_basis(ct,st(:,3),bt,bd)*bb;
  rmse=sqrt(mean((pred(mt)-st(mt,4)).^2));
  spread=std(st(mt,4));nrmse=100*(1-rmse/max(spread,eps));
  f=struct('train_run',train,'test_run',test,'wheel',w,'gain',bb(1),'bias_rpm',bb(2), ...
      'tau_s',bt,'delay_s',bd,'train_rmse_rpm',sqrt(best),'heldout_rmse_rpm',rmse, ...
      'heldout_nrmse_percent',nrmse,'train_samples',sum(mask),'heldout_samples',sum(mt), ...
      'near_optimal_tau_range_s',[min(near(:,3)) max(near(:,3))], ...
      'near_optimal_delay_range_s',[min(near(:,4)) max(near(:,4))]);
  if isempty(fits),fits=f;else,fits(end+1)=f;end
  profile=[profile;scan]; %#ok<AGROW>
  validation=[validation;[repmat([train test w],numel(pred),1) st(:,3:5) pred]]; %#ok<AGROW>
  subplot(2,4,(train-1)*4+w);
  plot(st(:,3),st(:,5),'k--',st(:,3),st(:,4),'bo',st(:,3),pred,'r-','LineWidth',1);
  grid on;xlabel('Time (s)');ylabel('RPM');title(sprintf('Train %d -> test %d, wheel %d\nRMSE %.2f RPM',train,test,w,rmse));
  if w==1,legend('ACK target','Measured (sparse)','Model','Location','best');end
  fprintf('FIT train%d w%d K%.4f b%.3f tau%.2f delay%.2f testRMSE%.3f\n',train,w,bb(1),bb(2),bt,bd,rmse);
 end
end
print(fig,fullfile(out,'wheel_validation.png'),'-dpng','-r140');savefig(fig,'wheel_validation.fig');close(fig);
writecsv('fit_profile.csv',profile,'train,wheel,tau_s,delay_s,gain,bias_rpm,train_mse');
writecsv('wheel_validation.csv',validation,'train,test,wheel,t_s,measured_rpm,ack_target_rpm,predicted_rpm');

% Open-loop replay checks whether nominal wheel geometry can predict yaw.
replay=struct([]);
for train=1:2
 test=3-train;params=fits([fits.train_run]==train);
 ct=C(C(:,1)==test,:);lt=L(L(:,1)==test,:);
 t=(0:.01:max(ct(:,3)))';v=zeros(numel(t),4);
 for w=1:4
  p=params(w);cc=ct(ct(:,2)==w,3:4);
  v(:,w)=response_basis(cc,t,p.tau_s,p.delay_s)*[p.gain;p.bias_rpm];
 end
 factor=pi*77/60;k=209;
 rate=(-v(:,1)+v(:,2)-v(:,3)+v(:,4))*factor/(4*k)*180/pi;
 yaw=cumtrapz(t,rate);
 measured=wrap(lt(:,3)-lt(:,4));yaw=yaw+measured(1);
 predicted=interp1(t,yaw,lt(:,2),'linear','extrap');
 entry=struct('train_run',train,'test_run',test, ...
   'yaw_rmse_deg',sqrt(mean(wrap(predicted-measured).^2)), ...
   'measured_peak_abs_yaw_deg',max(abs(measured)),'predicted_peak_abs_yaw_deg',max(abs(predicted)), ...
   'note','Nominal 77mm/209mm kinematics; no ground-slip or heading coupling fitted');
 if isempty(replay),replay=entry;else,replay(train)=entry;end
 writecsv(sprintf('yaw_replay_%d.csv',train),[lt(:,2) measured predicted], 't_s,measured_relative_yaw_deg,model_relative_yaw_deg');
end

% Fixed recorded base-speed schedule, closed-loop heading; not distance backend.
metrics=struct([]);ratios=[0 .005 .01 .02];delay_hyp=[0 .04 .08];slip_hyp=[-.005 0 .005];
fig=figure('Visible','off','Color','w','Position',[60 60 1200 850]);
for train=1:2
 test=3-train;params=fits([fits.train_run]==train);
 for direction=[1 -1]
  for ratio=ratios
   for sensor_delay=delay_hyp
    for slip=slip_hyp
     [m,tr]=closed_loop(params,C(C(:,1)==test,:),L(L(:,1)==test,:),direction,ratio,sensor_delay,slip);
     m.train_run=train;m.schedule_run=test;m.direction=direction;m.ratio=ratio;
     m.assumed_extra_sensor_delay_s=sensor_delay;m.assumed_slip_ratio=slip;
     if isempty(metrics),metrics=m;else,metrics(end+1)=m;end
     if sensor_delay==0 && slip==0
      idx=(train-1)*2+(direction==-1)+1;subplot(2,2,idx);hold on;
      plot(tr(:,2),tr(:,3),'LineWidth',1.4,'DisplayName',sprintf('FF %.1f%%',100*ratio));
      title(sprintf('Fit run%d, schedule run%d, direction %+d',train,test,direction));
      xlabel('Nominal forward position (mm)');ylabel('Nominal left position (mm)');grid on;
      writecsv(sprintf('trajectory_fit%d_dir%d_ff%03d.csv',train,direction,round(ratio*1000)),tr, ...
        't_s,x_mm,y_left_mm,yaw_deg,base_rpm,pd_rpm,combined_correction_rpm,v1_rpm,v2_rpm,v3_rpm,v4_rpm');
     end
    end
   end
  end
 end
end
for i=1:4,subplot(2,2,i);legend('show','Location','best');end
print(fig,'ff_trajectories.png','-dpng','-r140');savefig(fig,'ff_trajectories.fig');close(fig);
result=struct('matlab_version',version,'hardware_accessed',false,'compiled_vehicle_C',false, ...
 'fits',fits,'open_loop_yaw_validation',replay,'simulations',metrics,'data_review',review, ...
 'backward_measured',false,'ground_slip_identified',false, ...
 'model','First-order wheel with gain and signed bias, delay; measured ACK schedule; mathematical PD port; nominal mecanum geometry', ...
 'limits',{{'Two forward logs, sparse ~0.5s samples. Tau and delay estimates are exploratory, not precise identification.', ...
 'Backward extrapolates forward models with signed symmetry, not measured backward calibration.', ...
 'Fixed recorded base schedule, no actual distance-following backend or safety state machine; MATLAB abort only for heading error >10deg.', ...
 'Hypothetical extra sensor delays/slip are sensitivity assumptions, not identified vehicle parameters.', ...
 'Endpoint ruler data are comparisons only, not fitted to force lateral agreement. No prediction of actual vehicle millimetres.'}});
save('matlab_results.mat','result');fid=fopen('matlab_results.json','w','n','UTF-8');fprintf(fid,'%s',jsonencode(result));fclose(fid);
fprintf('MATLAB_FF_FINISHED fits%d simulations%d\n',numel(fits),numel(metrics));diary off;
end

function X=response_basis(cmd,t,tau,delay)
% Exact ZOH first-order integration at actual ACK events, not interpolated targets.
tc=cmd(:,1)+delay;u=cmd(:,2);X=zeros(numel(t),2);x=[0 0];last=0;current=[0 0];j=1;
for i=1:numel(t)
 while j<=numel(tc) && tc(j)<=t(i)
  dt=max(0,tc(j)-last);a=exp(-dt/tau);x=a*x+(1-a)*current;
  last=max(last,tc(j));current=[u(j) sign(u(j))];j=j+1;
 end
 dt=max(0,t(i)-last);a=exp(-dt/tau);x=a*x+(1-a)*current;last=t(i);X(i,:)=x;
end
end

function [m,tr]=closed_loop(p,commands,live,direction,ratio,sensor_delay,slip)
dt=.01;t=(0:dt:max(commands(:,3)))';n=numel(t);tr=zeros(n,11);
base=interp1(live(:,2),live(:,7),t,'previous','extrap');base(t<live(1,2))=0;
v=zeros(1,4);demand=zeros(1,4);pending=zeros(1,4);hist=zeros(n,4);yawhist=zeros(n,1);ratehist=zeros(n,1);
ticks=cell(1,4);next=ones(1,4);
for w=1:4,ticks{w}=commands(commands(:,2)==w,3);end
x=0;y=0;yaw=wrap(live(1,3)-live(1,4));filtered=0;holdcmd=0;q=0;aborted=false;
factor=pi*77/60;k=209;sat=0;
for i=1:n
 read=max(1,i-round(sensor_delay/dt));
 if read>=i,yr=yaw;if i>1,rr=ratehist(i-1);else,rr=0;end
 else,yr=yawhist(read);rr=ratehist(read);end
 if i==1,yr=yaw;rr=0;filtered=rr;elapsed=.02;else,elapsed=dt;filtered=filtered+dt/(.08+dt)*(rr-filtered);end
 error=wrap(-yr);
 if abs(error)>10,aborted=true;tr=tr(1:max(1,i-1),:);break,end
 desired=error-.25*filtered;
 if abs(error)<=.3 && abs(filtered)<=.3,desired=0;end
 desired=max(-2,min(2,desired));holdcmd=holdcmd+max(-10*elapsed,min(10*elapsed,desired-holdcmd));
 for j=1:40
  if holdcmd*10-q>.65,q=q+1;elseif holdcmd*10-q<-.65,q=q-1;else,break,end
 end
 q=max(-20,min(20,q));pd=q*.1;b=base(i);limit=min(b,2);
 ff=direction*min(.5,b*ratio*.5);corr=max(-limit,min(limit,max(-limit,min(limit,pd))+ff));
 if abs(pd+ff)>limit && b>0,sat=sat+1;end
 logical=direction*b+[-1 1 -1 1]*corr;
 for w=1:4
  while next(w)<=numel(ticks{w}) && ticks{w}(next(w))<=t(i)+1e-9
   if w==1,pending=sign(logical).*floor(abs(logical)*10+.5)/10;end
   demand(w)=pending(w);next(w)=next(w)+1;
  end
 end
 hist(i,:)=demand;
 for w=1:4
  j=max(1,i-round(p(w).delay_s/dt));u=hist(j,w);
  a=exp(-dt/p(w).tau_s);v(w)=a*v(w)+(1-a)*(p(w).gain*u+p(w).bias_rpm*sign(u));
 end
 vx=sum(v)*factor/4;vy=(v(1)-v(2)-v(3)+v(4))*factor/4+slip*vx;
 rate=(-v(1)+v(2)-v(3)+v(4))*factor/(4*k)*180/pi;
 mid=(yaw+rate*dt*.5)*pi/180;x=x+(vx*cos(mid)-vy*sin(mid))*dt;y=y+(vx*sin(mid)+vy*cos(mid))*dt;
 yaw=wrap(yaw+rate*dt);yawhist(i)=yaw;ratehist(i)=rate;
 tr(i,:)=[t(i) x y yaw b pd corr v];
end
m=struct('aborted',aborted,'final_x_mm',x,'final_y_left_mm',y,'abs_lateral_mm',abs(y), ...
 'peak_abs_yaw_deg',max(abs(tr(:,4))),'final_yaw_deg',yaw,'saturated_steps',sat,'duration_s',tr(end,1));
end

function y=wrap(x)
y=mod(x+180,360)-180;
end

function writecsv(name,data,header)
fid=fopen(name,'w');fprintf(fid,'%s\n',header);fclose(fid);dlmwrite(name,data,'-append','precision','%.10g');
end
