function run_speed_advice(folder)
% Recommendation-only sweep. No firmware file changes, downloads or hardware I/O.
P=readtable(fullfile(folder,'sweep-profiles.csv'));plans=readtable(fullfile(folder,'sweep-plans.csv'));
factor=pi*77/60;results=zeros(height(plans)*36,11);row=0;
for q=1:height(plans)
    plan=plans(q,:);v=P(P.distance_mm==plan.distance_mm & P.cap_cm_s==plan.cap_cm_s,:);
    cap=ceil(plan.cap_cm_s*10/factor)+2;
    assert(cap<300); % hypothetical command cap for offline sweep, not applied to firmware
    for tau=[.02 .05 .1 .2]
        for delay=[0 .08 .16]
            for gain=[.9 1 1.05]
                pos=0;vel=0;cmd=0;corr=0;corrRate=0;stop=0;result=0;peak=0;maxerr=0;
                hist=zeros(height(v),1);
                for k=1:height(v)
                    dt=.01;t=v.time_s(k);
                    dv=max(-40,min(40,(cmd*factor*gain-vel)*dt/(tau+dt)));
                    vel=vel+dv;pos=pos+vel*dt;hist(k)=pos;
                    idx=max(1,k-round(delay/dt));observed=hist(idx);
                    err=v.position_mm(idx)-observed;
                    if ~stop
                        maxerr=max(maxerr,abs(err));
                        if abs(err)>50,result=2;stop=1;
                        elseif observed>=plan.distance_mm || (t>=plan.duration_s && observed>=plan.distance_mm-2),result=1;stop=1;
                        elseif t>=plan.duration_s+2,result=3;stop=1;
                        end
                    end
                    if stop,cmd=0;
                    else
                        desired=max(-2,min(2,err/factor));
                        if k>1
                            y=corr-desired;b=corrRate+y;decay=exp(-dt);
                            corr=desired+(y+b*dt)*decay;corrRate=(corrRate-b*dt)*decay;
                        end
                        cmd=max(0,min(cap,v.speed_mm_s(k)/factor+corr));
                        if t>=v.approach_start_s(1),cmd=max(1,cmd);end
                        cmd=floor(cmd+.5);
                    end
                    peak=max(peak,vel);
                    if stop && abs(vel)<.05,break;end
                end
                row=row+1;results(row,:)=[plan.distance_mm plan.cap_cm_s tau delay gain result pos-plan.distance_mm maxerr peak/10 t cap];
            end
        end
    end
end
cases=array2table(results,'VariableNames',{'distance_mm','cap_cm_s','tau_s','delay_s','gain','result','stop_error_mm','tracking_error_mm','peak_cm_s','time_s','hypothetical_rpm_cap'});
writetable(cases,fullfile(folder,'matlab-cases.csv'));
summary=[];
for q=1:height(plans)
    p=plans(q,:);c=cases(cases.distance_mm==p.distance_mm & cases.cap_cm_s==p.cap_cm_s,:);
    nominal=c(c.gain==1 & c.tau_s<=.05,:);moderate=c(c.gain==1 & c.tau_s==.1,:);
    completed=nominal(nominal.result==1,:);
    if isempty(completed),stopError=NaN;else,stopError=max(abs(completed.stop_error_mm));end
    summary=[summary;p.distance_mm p.cap_cm_s p.peak_cm_s p.duration_s p.braking_mm ...
        stopError max(nominal.tracking_error_mm) ...
        sum(moderate.result==1) height(moderate) sum(c.result==1) height(c) height(completed)]; %#ok<AGROW>
end
S=array2table(summary,'VariableNames',{'distance_mm','cap_cm_s','reference_peak_cm_s','reference_time_s','braking_mm','nominal_max_stop_error_mm','nominal_max_tracking_error_mm','tau100_completed','tau100_cases','all_completed','all_cases','nominal_completed_of6'});
writetable(S,fullfile(folder,'matlab-summary.csv'));
f=figure('Visible','off','Position',[0 0 1100 750]);
subplot(2,2,1);hold on;for d=[150 900 1050 1920],t=S(S.distance_mm==d,:);plot(t.cap_cm_s,t.reference_time_s,'-o');end
grid on;xlabel('Configured speed cap (cm/s)');ylabel('Reference duration (s)');legend('150 mm','900 mm','1050 mm','1920 mm');title('Same acceleration and jerk limits');
subplot(2,2,2);hold on;for d=[900 1050 1920],t=S(S.distance_mm==d,:);plot(t.cap_cm_s,t.braking_mm,'-o');end
grid on;xlabel('Speed cap (cm/s)');ylabel('Reference braking distance (mm)');legend('900 mm','1050 mm','1920 mm');
subplot(2,2,3);t=S(S.distance_mm==1050,:);plot(t.cap_cm_s,t.nominal_max_tracking_error_mm,'-o');hold on;yline(50,'--');grid on;xlabel('Speed cap (cm/s)');ylabel('Tracking error (mm)');title('1050 mm; tau 20/50 ms, unity gain');
subplot(2,2,4);plot(t.cap_cm_s,t.tau100_completed,'-o');ylim([0 3.2]);grid on;xlabel('Speed cap (cm/s)');ylabel('Completed cases out of 3');title('1050 mm; tau 100 ms, delays 0/80/160 ms');
saveas(f,fullfile(folder,'matlab-speed-advice.png'));close(f);
fid=fopen(fullfile(folder,'matlab-result.json'),'w');fprintf(fid,'%s',jsonencode(struct('passed',true,'version',version,'plans',height(plans),'cases',height(cases),'firmware_changed',false,'hardware_accessed',false,'dynamics_are_assumed',true,'limits_above_40_are_hypothetical',true)));fclose(fid);
disp(S(S.distance_mm==150 | S.distance_mm==1050,:));
end
