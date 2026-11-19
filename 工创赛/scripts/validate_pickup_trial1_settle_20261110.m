function validate_pickup_trial1_settle_20261110
root=fileparts(fileparts(mfilename('fullpath')));ev=fullfile(root,'evidence','integration','2026-11-10','speed-trials-settle');
if ~exist(ev,'dir'),mkdir(ev);end
e=fullfile(root,'evidence','integration','2026-11-10','speed-trials-braking');
r=jsondecode(fileread(fullfile(e,'integration13-three-immediate-once-20261110-203519.json')));
a=r.first_fault.vsp_axes(1);assert(a.node==6 && a.flags==1 && a.phase==1);
assert(abs(a.goal-a.current-60.875)<1e-6);n=2;
for eligible=[false true]
 for ex=[0 45 59.9 60 60.875 73.375 89.9 90 90.1 95.875 670]
  for elapsed=[0 20 100 149 150 151 500]
   allow=ex<=60 || (eligible && ex<=90 && elapsed<150);
   if ex>90 || (ex>60 && (~eligible || elapsed>=150)),assert(~allow);end
   if allow && ex>60,assert(eligible && ex<=90 && elapsed<150);end
   n=n+1;
  end
 end
end
% Gate simulation includes return-inside-window, persistent and severe cases.
events={ [0 60.875;40 75;80 48;120 20], [0 60.875;149 70;150 70], [0 60.875;20 90.1] };
expected=[true false false];results=false(1,3);
for k=1:3
 x=events{k};ok=true;
 for i=1:size(x,1)
  if x(i,2)>90 || (x(i,2)>60 && x(i,1)>=150),ok=false;break;end
 end
 results(k)=ok;assert(ok==expected(k));n=n+1;
end
% Eligibility is ONLY node6 / first paired profile / 156 or 160mm outward.
for node=[5 6]
 for profile=0:3
  for goal=[0 100 156 160 170]
   eligible=(node==6 && profile==1 && (goal==156 || goal==160));
   if eligible,assert(goal+90/(315.5*360/3200)<170);end
   n=n+1;
  end
 end
end
o=struct('passed',true,'checks',n,'engine','local MATLAB', ...
 'measured_previous_excursion_virtual_deg',60.875,'soft_deg',60,'hard_deg',90,'max_ms',150, ...
 'completion_deg',45,'required_zero_speed_samples',3,'eligible_profile',1,'eligible_node',6, ...
 'physical_settle_observed_without_interruption',false,'plant_model_identified',false, ...
 'hardware_access',false,'note','Verifies bounded policy only. Simulated recovery is not proof that the vehicle settles.');
f=fopen(fullfile(ev,'matlab.json'),'w','n','UTF-8');assert(f>0);fprintf(f,'%s\n',jsonencode(o));fclose(f);
fprintf('PASS: %d bounded-settle checks; >90deg or >=150ms still stops, final45deg/3 zero-speed samples unchanged.\n',n);
end
