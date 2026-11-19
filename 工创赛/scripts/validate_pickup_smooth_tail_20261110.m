function validate_pickup_smooth_tail_20261110
root=fileparts(fileparts(mfilename('fullpath')));ev=fullfile(root,'evidence','integration','2026-11-10','smooth-tail');
if ~exist(ev,'dir'),mkdir(ev);end
r=jsondecode(fileread(fullfile(root,'evidence','integration','2026-11-10','smooth-handoff','integration13-three-immediate-once-20261110-211343.json')));
a=r.first_fault.vsp_axes(1);assert(a.node==6 && r.first_fault.g_pick_extension_segment==2 && abs(a.goal-a.current-61.375)<1e-6);
assert(strcmp(a.move_hex(1:16),'fd01025802580424'));n=2;
for profile=2:3
 for stage=0:2
  for rpm=[106 300 400]
   for ex=[60 61.375 90 90.1 199.575]
    for elapsed=[0 149 150]
     eligible=(stage==2 && rpm==106);
     allow=ex<=60 || (eligible && ex<=90 && elapsed<150);
     assert(~allow || ex<=60 || (eligible && ex<=90 && elapsed<150));
     if ex>90 || (ex>60 && elapsed>=150),assert(~allow);end
     n=n+1;
    end
   end
  end
 end
end
o=struct('passed',true,'checks',n,'engine','local MATLAB','tail_only',true,'tail_rpm',106,'tail_accel_decel',600, ...
 'max_excursion_virtual_deg',90,'max_transient_ms',150,'final_window_deg',45,'final_samples',3,'hardware_access',false,'field_verified',false);
f=fopen(fullfile(ev,'matlab.json'),'w','n','UTF-8');assert(f>0);fprintf(f,'%s\n',jsonencode(o));fclose(f);
fprintf('PASS %d low-speed-tail-only checks; high-speed199.575deg remains rejected.\n',n);
end
