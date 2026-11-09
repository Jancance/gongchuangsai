function simulate_stage2_matlab_20261104(root)
% MATLAB R2020a, base MATLAB only. Assumed wheel dynamics; no hardware I/O.
out=fullfile(root,'evidence','integration','2026-11-04','stage2-speed');
P=readtable(fullfile(out,'profiles.csv')); C=readtable(fullfile(out,'rotation-summary.csv'));
T=readtable(fullfile(out,'rotation.csv')); factor=pi*77/60; rotFactor=6*77/418;
keys=unique([P.stage P.segment],'rows'); results=[]; traces=[];
for q=1:size(keys,1)
    plan=P(P.stage==keys(q,1) & P.segment==keys(q,2),:);
    assert(max(plan.speed_mm_s)<=400.01 && max(abs(plan.acceleration_mm_s2))<=1000.1);
    assert(abs(plan.position_mm(end)-plan.distance_mm(1))<.01);
    for tau=[.02 .05 .2]
        for delay=[0 .08 .16]
            pos=0;vel=0;cmd=0;corr=0;corrRate=0;hist=zeros(height(plan),1);
            peak=0;maxerr=0;result=0;stop=0;
            for k=1:height(plan)
                t=plan.time_s(k);dt=.01;
                dv=max(-40,min(40,(cmd*factor-vel)*dt/(tau+dt)));
                vel=vel+dv;pos=pos+vel*dt;hist(k)=pos;
                idx=max(1,k-round(delay/dt));
                err=plan.position_mm(idx)-hist(idx);
                if ~stop
                    maxerr=max(maxerr,abs(err));
                    if abs(err)>50
                        result=2;stop=1;
                    elseif pos>=plan.distance_mm(1) || (t>=plan.duration_s(1) && pos>=plan.distance_mm(1)-2)
                        result=1;stop=1;
                    elseif t>=plan.duration_s(1)+2
                        result=3;stop=1;
                    end
                end
                if stop
                    cmd=0;
                else
                    % Independently implement actual C follower's bounded critically damped correction.
                    desired=max(-2,min(2,err/factor));
                    if k>1
                        y=corr-desired;b=corrRate+y;decay=exp(-dt);
                        corr=desired+(y+b*dt)*decay;corrRate=(corrRate-b*dt)*decay;
                    end
                    cmd=max(0,min(102,plan.speed_mm_s(k)/factor+corr));
                    if t>=plan.approach_start_s(1),cmd=max(1,cmd);end
                    cmd=floor(cmd+.5);
                end
                peak=max(peak,abs(vel));
                if q<=2 && tau==.05 && delay==.08
                    traces=[traces; keys(q,:) t plan.position_mm(k) pos vel cmd*factor]; %#ok<AGROW>
                end
                if stop && abs(vel)<.05,break;end
            end
            assert(peak<=411); % 400mm/s reference plus bounded two-RPM follower/quantization reserve.
            if tau<=.05,assert(result==1,'Nominal translation did not arrive');end
            results=[results;keys(q,:) tau delay result pos-plan.distance_mm(1) maxerr peak t]; %#ok<AGROW>
        end
    end
end
translations=array2table(results,'VariableNames',{'stage','segment','tau_s','delay_s','result','stop_error_mm','max_tracking_error_mm','peak_mm_s','time_s'});
writetable(translations,fullfile(out,'matlab-translation-summary.csv'));
writetable(array2table(traces,'VariableNames',{'stage','segment','time_s','reference_mm','position_mm','speed_mm_s','command_mm_s'}),fullfile(out,'matlab-stage2-traces.csv'));

rotResults=[];rotTrace=[];
for q=1:height(C)
    c=C(q,:); ct=T(T.case_id==c.case_id,:);start=ct.start_yaw(1);
    yaw=start;rate=0;cmd=0;peak=0;hy=zeros(3000,1);hr=hy;
    s=struct('speed',0,'accel',0,'angle',0,'rate',0,'stable',0);
    for k=1:3000
        rate=rate+(cmd*rotFactor-rate)*.01/(c.tau+.01);
        yaw=wrap(yaw+rate*.01);hy(k)=yaw;hr(k)=rate;
        idx=max(1,k-round(c.delay_s/.01));
        [s,cmd,status]=turn(s,start,c.angle,hy(idx),hr(idx),.01);
        assert(abs(cmd)<=54.0001 && sign(c.angle)*cmd>=0);
        peak=max(peak,abs(rate));
        if q==1,rotTrace=[rotTrace;k*.01 wrap(yaw-start) rate cmd*rotFactor];end %#ok<AGROW>
        if status~=0,break;end
    end
    err=wrap(start+c.angle-yaw);
    assert(status==1 && abs(err)<=1 && peak<=60.01,'MATLAB turn did not settle');
    assert(abs(err-c.error_deg)<.1 && abs(k*.01-c.duration_s)<.1,'C/MATLAB turn disagreement');
    rotResults=[rotResults;c.case_id status k*.01 err peak]; %#ok<AGROW>
end
writetable(array2table(rotResults,'VariableNames',{'case_id','result','duration_s','error_deg','peak_dps'}),fullfile(out,'matlab-rotation-summary.csv'));

f=figure('Visible','off','Position',[0 0 1200 850]);
subplot(2,2,1);hold on;
for leg=0:1
    v=P(P.stage==2 & P.segment==leg,:);plot(v.time_s,v.speed_mm_s/10,'LineWidth',1.5);
end
grid on;xlabel('Time (s)');ylabel('Reference (cm/s)');title('Stage 2: 40 cm/s cap');legend('Forward 900 mm','Right 960 mm');
subplot(2,2,2);hold on;
for leg=0:1
    v=P(P.stage==2 & P.segment==leg,:);plot(v.time_s,v.acceleration_mm_s2,'LineWidth',1.3);
end
grid on;xlabel('Time (s)');ylabel('Acceleration (mm/s^2)');title('Jerk-limited reference');
subplot(2,2,3);plot(rotTrace(:,1),rotTrace(:,2),'LineWidth',1.3);grid on;xlabel('Time (s)');ylabel('Angle (deg)');title('MATLAB closed-loop -90 deg turn');
subplot(2,2,4);plot(rotTrace(:,1),rotTrace(:,3:4),'LineWidth',1.3);grid on;xlabel('Time (s)');ylabel('Angular speed (deg/s)');legend('Model response','Command');title('60 deg/s cap; short turns may peak lower');
saveas(f,fullfile(out,'matlab-speed-validation.png'));close(f);
f=figure('Visible','off');plot([1200 1200 2160],[300 1200 1200],'-o','LineWidth',2);axis equal;axis([0 2400 0 2400]);grid on;xlabel('Project X (mm)');ylabel('Project Y (mm)');text(1200,300,' 2 / scan');text(1200,1200,' 12');text(2160,1200,' 14 / wait vision');title('Nominal route; body heading +Y; no metric vision correction yet');saveas(f,fullfile(out,'matlab-route.png'));close(f);
report=struct('passed',true,'matlab_version',version,'translation_cases',height(translations),...
    'rotation_cases',size(rotResults,1),'max_turn_error_deg',max(abs(rotResults(:,4))),...
    'max_turn_peak_dps',max(rotResults(:,5)),'model_is_assumed',true,'hardware_accessed',false,...
    'position_correction_executed',false,'c_matlab_crosscheck_passed',true);
nominal=translations.tau_s<=.05;stage2=nominal & translations.stage==2;
report.nominal_translation_cases=sum(nominal);
report.nominal_translation_completed=sum(translations.result(nominal)==1);
report.stress_cases=sum(~nominal);
report.stress_guarded_stops=sum(translations.result(~nominal)~=1);
report.max_nominal_stop_error_mm=max(abs(translations.stop_error_mm(nominal)));
report.max_stage2_nominal_stop_error_mm=max(abs(translations.stop_error_mm(stage2)));
fid=fopen(fullfile(out,'matlab-result.json'),'w');fprintf(fid,'%s',jsonencode(report));fclose(fid);
disp(report);
end
function a=wrap(a)
a=mod(a+180,360)-180;
end
function [s,cmd,result]=turn(s,start,angle,yaw,rate,dt)
sgn=sign(angle);observed=sgn*wrap(yaw-start);cmd=0;result=0;
if observed>abs(angle)+1 || observed< -2,result=2;return;end
alpha=dt/(.08+dt);s.angle=s.angle+(observed-s.angle)*alpha;s.rate=s.rate+(sgn*rate-s.rate)*alpha;
remaining=abs(angle)-s.angle;ahead=remaining-.7*max(0,s.rate);distance=max(0,ahead-.25);
want=min([sqrt(144+180*distance)-12,2*distance,54*(6*77/418)]);
desired=max(-90,min(90,(want-s.speed)*5));s.accel=s.accel+max(-360*dt,min(360*dt,desired-s.accel));
s.speed=s.speed+s.accel*dt;
if s.speed<0,s.speed=0;s.accel=0;end
if s.speed>54*(6*77/418),s.speed=54*(6*77/418);s.accel=0;end
tol=.8;if s.stable>0,tol=.9;end
if abs(remaining)<=tol && abs(s.rate)<=.6 && abs(rate)<=1 && abs(abs(angle)-observed)<=1
    s.stable=s.stable+dt;
else
    s.stable=0;
end
if s.stable>=.7,result=1;return;end
cmd=sgn*min(54,floor(s.speed/(6*77/418)*10+.5)*.1);
end
