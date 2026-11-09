function simulate_stage1_blend_matlab_20261104(root)
% Independent assumed two-axis response, not a CAN/RTOS or traction model.
out=fullfile(root,'evidence','integration','2026-11-04','stage1-blend-search');
R=readtable(fullfile(out,'blend-reference.csv'));C=readtable(fullfile(out,'blend-c-model.csv'));
assert(max(abs(R.ax_mm_s2)+abs(R.ay_mm_s2))<=1000.1);
assert(max(abs(R.jx_mm_s3)+abs(R.jy_mm_s3))<=2000.1);
assert(max(R.vx_mm_s)<=450.01 && max(R.vy_mm_s)<=200.01);
idx=find(R.time_s>=R.y_duration_s(1),1);
assert(R.vx_mm_s(idx)>0 && R.vy_mm_s(idx)==0,'Forward handoff stopped lateral reference');
results=[];trace=[];cross=[];
for zone=1:2
 for tau=[.03 .05 .1 .2]
  for delay=[0 .08 .16]
   pos=[0 0];vel=[0 0];cmd=[0 0];corr=[0 0];rate=[0 0];stable=0;
   history=zeros(height(R),2);factor=[R.x_factor(1) R.y_factor(1)];result=0;
   for k=1:height(R)-1
    dt=.01;t=k*dt;vel=vel+(cmd.*factor-vel)*dt/(tau+dt);pos=pos+vel*dt;history(k,:)=pos;
    old=max(1,k-round(delay/dt));observed=(old)*dt;
    nowRef=R(min(k+1,height(R)),:);oldRef=R(min(old+1,height(R)),:);
    oldPos=history(old,:);oldGoal=[oldRef.x_mm oldRef.y_mm];oldSpeed=[oldRef.vx_mm_s oldRef.vy_mm_s];
    limit=min(80,max(50,20+abs(oldSpeed)*.1));
    if any(abs(oldGoal-oldPos)>limit) || any(oldPos< -5) || oldPos(1)>1100 || oldPos(2)>200
        result=2;break;
    end
    goals=[1050 150];duration=[R.duration_s(1) R.y_duration_s(1)];
    % R.duration_s is max(x_duration,y_duration); x is the longer axis here.
    desired=max(-2,min(2,(oldGoal-oldPos)./factor));
    if k>1
      z=corr-desired;b=rate+z;decay=exp(-dt);corr=desired+(z+b*dt)*decay;rate=(rate-b*dt)*decay;
    end
    cmd=[nowRef.vx_mm_s nowRef.vy_mm_s]./factor+corr;done=true;
    for a=1:2
      if t<duration(a),cmd(a)=max(0,cmd(a));done=false;
      elseif abs(goals(a)-oldPos(a))<=2,cmd(a)=0;corr(a)=0;rate(a)=0;
      else,cmd(a)=max(-2,min(2,(goals(a)-oldPos(a))/factor(a)));done=false;
      end
    end
    side=cmd(1);if zone==2,side=-side;end
    wheels=[cmd(2)+side cmd(2)-side cmd(2)-side cmd(2)+side];
    assert(max(abs(wheels))<=175+.001);
    if done,result=1;
    elseif t>=R.duration_s(1)+3,result=3;
    end
    if zone==1 && tau==.05 && delay==0
      trace=[trace;t pos vel cmd]; %#ok<AGROW>
      if k<=height(C),cross=[cross;abs(pos-[C.x_mm(k) C.y_mm(k)])];end %#ok<AGROW>
    end
    if result,break;end
   end
   if tau<=.05,assert(result==1,'Nominal blend failed');end
   results=[results;zone tau delay result t pos-goals]; %#ok<AGROW>
  end
 end
end
assert(max(cross(:))<.2,'C/MATLAB nominal blend disagreement');
writetable(array2table(results,'VariableNames',{'zone','tau_s','delay_s','result','time_s','x_error_mm','y_error_mm'}),fullfile(out,'matlab-blend-summary.csv'));
writetable(array2table(trace,'VariableNames',{'time_s','x_mm','y_mm','vx_mm_s','vy_mm_s','x_rpm','y_rpm'}),fullfile(out,'matlab-blend-trace.csv'));
f=figure('Visible','off','Position',[0 0 1200 850]);
subplot(2,2,1);plot(R.time_s,R.vx_mm_s/10,R.time_s,R.vy_mm_s/10,'LineWidth',1.5);xline(R.y_duration_s(1),'--');grid on;legend('Lateral cap45','Forward cap20','Forward reference complete');xlabel('Time(s)');ylabel('Reference cm/s');title('No full stop at forward-to-lateral handoff');
subplot(2,2,2);plot(2250-R.x_mm,150+R.y_mm,'LineWidth',1.5);hold on;plot(150+R.x_mm,150+R.y_mm,'LineWidth',1.5);plot([1200 1050 1350],[300 300 300],'--o');axis equal;axis([0 2400 0 600]);grid on;xlabel('Project X mm');ylabel('Project Y mm');legend('Zone1','Zone2','Zone1 search -150 then +300');title('Nominal route; heading target +Y');
subplot(2,2,3);plot(trace(:,1),trace(:,4:5)/10,'LineWidth',1.3);grid on;xlabel('Time(s)');ylabel('Assumed response cm/s');legend('Lateral','Forward');
subplot(2,2,4);plot(trace(:,1),trace(:,2)-1050,trace(:,1),trace(:,3)-150,'LineWidth',1.3);grid on;xlabel('Time(s)');ylabel('Encoder-model target error mm');legend('Lateral target','Forward target');title('Model coordinates, not measured ground accuracy');
saveas(f,fullfile(out,'matlab-blend-search.png'));close(f);
report=struct('passed',true,'cases',size(results,1),'nominal_completed',sum(results(:,2)<=.05 & results(:,4)==1),...
 'degraded_completed',sum(results(:,2)>.05 & results(:,4)==1),'degraded_guarded_stop',sum(results(:,2)>.05 & results(:,4)~=1),...
 'c_matlab_max_position_difference_mm',max(cross(:)),'y_handoff_time_s',R.y_duration_s(1),...
 'lateral_reference_at_handoff_cm_s',R.vx_mm_s(idx)/10,'assumed_dynamics',true,'hardware_accessed',false);
fid=fopen(fullfile(out,'matlab-check.json'),'w');fprintf(fid,'%s',jsonencode(report));fclose(fid);disp(report);
end
