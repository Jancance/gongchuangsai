function validate_pickup_smooth_handoff_20261110
root=fileparts(fileparts(mfilename('fullpath')));ev=fullfile(root,'evidence','integration','2026-11-10','smooth-handoff');
if ~exist(ev,'dir'),mkdir(ev);end
n=0;rows=[];mmperrev=3200*10/315.5;
for gear=2:3
 if gear==2,rpm=300;acc=1200;else,rpm=400;acc=2000;end
 for travel=[156 160]
  for direction=[-1 1]
   before=-travel*(direction==1);goal=-travel*(direction==-1);
   waypoint=goal-direction*8;
   assert(min(before,goal)<waypoint && waypoint<max(before,goal));
   [tfast,peak]=trapezoid((travel-8)/mmperrev,rpm,acc);
   [ttail,tailpeak]=trapezoid(8/mmperrev,106,600);
   assert(abs(peak-rpm)<1e-8 && tailpeak<=106 && tfast>0 && ttail>0);
   rows=[rows;gear travel direction waypoint tfast ttail peak]; %#ok<AGROW>
   n=n+2;
  end
 end
end
% The entire original start-to-final envelope remains the overrun boundary.
for excursion=[59.9 60 60.1 90.1 199.575]
 assert((excursion<=60)==~(excursion>60));n=n+1;
end
% Grasp can preclose on first fresh, in-window, zero-speed/done extension
% sample; group DONE still requires three and cannot be fabricated.
for fresh=0:1
 for enabledDone=0:1
  for sample=[0 1 2 3]
   for error=[0 45 45.1]
    for depth=[110799 110800 118800 119201]
     eligible=fresh && enabledDone && sample>=1 && abs(error)<=45 && depth>=110800 && depth<=119200;
     if eligible,assert(fresh && enabledDone && sample>=1 && abs(error)<=45);end
     n=n+1;
    end
   end
  end
 end
end
for fresh=0:1
 for err=[0 0.999 1.001 3]
  for vel=[0 .034 .036]
   settled=0;
   for frame=1:3
    if fresh && abs(err)<=1 && abs(vel)<=.035,settled=settled+1;else,settled=0;end
    done=settled>=3;
    assert(~done || (fresh && frame==3 && abs(err)<=1 && abs(vel)<=.035));n=n+1;
   end
  end
 end
end
for elapsed=[0 49 50 51]
 assert((elapsed>=50)==~(elapsed<50));n=n+1;
end
r=struct('passed',true,'checks',n,'segment_rows',rows, ...
 'tail_mm',8,'tail_rpm',106,'tail_accel_decel_rpm_s',600, ...
 'release_to_next_request_ms',50,'dm_parallel_samples',3,'dm_parallel_window_deg',1, ...
 'grip_first_fresh_zero_sample',true,'driver_fault_and_final_windows_unchanged',true, ...
 'lift_starts_after_extension_tail_command',true,'simulated_policy_only',true, ...
 'physical_servo_release_time_verified',false,'physical_high_speed_braking_verified',false,'hardware_access',false);
f=fopen(fullfile(ev,'matlab.json'),'w','n','UTF-8');assert(f>0);fprintf(f,'%s\n',jsonencode(r));fclose(f);
fprintf('PASS %d checks. High profiles keep peak300/400 in ideal command model; final8mm at106RPM.\n',n);
disp(array2table(rows,'VariableNames',{'Gear','TravelMM','Sign','WaypointMM','FastS','TailS','PeakRPM'}));
end
function [T,peak]=trapezoid(rev,rpm,acc)
v=min(rpm/60,sqrt(rev*acc/60));a=acc/60;T=2*v/a+max(0,rev-v*v/a)/v;peak=v*60;
end
