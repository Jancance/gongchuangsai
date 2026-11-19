function validate_pickup_trial1_braking_20261110
% Compare an asymmetric command profile. This is NOT a fitted motor plant.
root=fileparts(fileparts(mfilename('fullpath')));
ev=fullfile(root,'evidence','integration','2026-11-10','speed-trials-braking');
if ~exist(ev,'dir'),mkdir(ev);end
old=fullfile(root,'evidence','integration','2026-11-10','speed-trials');
d=jsondecode(fileread(fullfile(old,'integration13-three-immediate-once-20261110-201728.json')));
a=d.first_fault.vsp_axes(1);
assert(a.node==6 && a.phase==1 && a.flags==1 && a.stall_active==0);
assert(a.current<a.goal-60 && d.first_fault.g_source_parallel_cause==7);
assert(d.after_stop.g_source_parallel_stop_mask==0 && d.last.pe2==1);
checks=4; mm=[156 160]; profiles=struct([]);
for k=1:numel(mm)
 % Preserve the prior empirically taught command scaling; no new calibration.
 rev=mm(k)*315.5/3200/10;
 p=profile(rev,200,600,600);q=profile(rev,200,600,400);
 assert(abs(p.peak_rpm-200)<1e-8 && abs(q.peak_rpm-200)<1e-8);
 assert(q.decel_start_rev<p.decel_start_rev && q.decel_time_s>p.decel_time_s);
 for c={p,q}
  c=c{1};assert(all(diff(c.x)>=-1e-10));assert(abs(c.x(end)-rev)<1e-9);
  assert(c.v(1)==0 && abs(c.v(end))<1e-9 && max(c.v)<=200/60+1e-9);
  checks=checks+3;
 end
 profiles(k).extension_mm=mm(k);profiles(k).old_time_s=p.total_s;
 profiles(k).candidate_time_s=q.total_s;profiles(k).added_s=q.total_s-p.total_s;
 profiles(k).peak_rpm=q.peak_rpm;profiles(k).old_decel_start_fraction=p.decel_start_rev/rev;
 profiles(k).candidate_decel_start_fraction=q.decel_start_rev/rev;
 checks=checks+2;
 if k==1,pLeft=p;qLeft=q;end
end
% The same measured overrun still fails the unchanged control window.
for direction=[-1 1]
 for excess=[59.9 60 60.1 73.375 95.875 670]
  goal=direction*5537.025;pos=goal+direction*excess;
  allowed=(pos>=min(0,goal)-60 && pos<=max(0,goal)+60);
  assert(allowed==(excess<=60));checks=checks+1;
 end
end
f=figure('Visible','off','Position',[100 100 1080 660]);
subplot(2,1,1);plot(pLeft.t,pLeft.x/pLeft.x(end)*156,'--',qLeft.t,qLeft.x/qLeft.x(end)*156,'LineWidth',1.6);
ylabel('Commanded extension (mm)');xlabel('Time (s)');grid on;
title('Trial 1: commanded trajectory only; actual overshoot is unmodelled');
legend('200 RPM, accel/decel 600/600','200 RPM, accel/decel 600/400','Location','southeast');
subplot(2,1,2);plot(pLeft.t,pLeft.v*60,'--',qLeft.t,qLeft.v*60,'LineWidth',1.6);
xlabel('Time (s)');ylabel('Commanded motor speed (RPM)');grid on;
exportgraphics(f,fullfile(ev,'braking-command-model.png'),'Resolution',140);close(f);
r=struct('passed',true,'checks',checks,'engine','local MATLAB', ...
 'trial',1,'lift_rpm',2100,'lift_accel_decel_rpm_s',45055, ...
 'extension_rpm',200,'extension_accel_rpm_s',600,'extension_decel_rpm_s',400, ...
 'profiles',profiles,'measured_overrun_virtual_deg',a.goal-a.current, ...
 'overrun_derived_mm',(a.goal-a.current)/(315.5*360/3200), ...
 'position_window_unchanged_deg',60,'completion_window_unchanged_deg',45, ...
 'actual_overshoot_fixed',false,'plant_identified',false,'hardware_access',false, ...
 'note','Earlier gentler command braking is a trial hypothesis, not proof of physical tracking or timing.');
h=fopen(fullfile(ev,'matlab.json'),'w','n','UTF-8');assert(h>0);fprintf(h,'%s\n',jsonencode(r));fclose(h);
fprintf('PASS: %d checks. Measured left overrun %.3f virtual deg / %.3f derived mm.\n',checks,r.measured_overrun_virtual_deg,r.overrun_derived_mm);
for k=1:numel(profiles)
 fprintf('%.0fmm: old %.6fs, candidate %.6fs, delta %.6fs, peak %.1fRPM.\n',mm(k),profiles(k).old_time_s,profiles(k).candidate_time_s,profiles(k).added_s,profiles(k).peak_rpm);
end
end

function p=profile(distance,rpm,accel,decel)
v=min(rpm/60,sqrt(2*distance/(60/accel+60/decel)));
a=accel/60;b=decel/60;ta=v/a;td=v/b;da=v*v/(2*a);dd=v*v/(2*b);
tc=max(0,(distance-da-dd)/v);T=ta+tc+td;t=unique([linspace(0,T,4001) ta ta+tc T]);
x=zeros(size(t));vel=x;
for i=1:numel(t)
 if t(i)<=ta,x(i)=a*t(i)^2/2;vel(i)=a*t(i);
 elseif t(i)<=ta+tc,x(i)=da+v*(t(i)-ta);vel(i)=v;
 else,u=t(i)-ta-tc;x(i)=da+v*tc+v*u-b*u*u/2;vel(i)=v-b*u;end
end
p=struct('t',t,'x',x,'v',vel,'total_s',T,'peak_rpm',v*60, ...
 'decel_start_rev',da+v*tc,'decel_time_s',td);
end
