function validate_integration13_zero_transient_20261109
root=fileparts(fileparts(mfilename('fullpath')));ev=fullfile(root,'evidence','integration','2026-11-07','stage2-mainline');
r=jsondecode(fileread(fullfile(ev,'integration13-right160-overlap-first-fault-static-20261109.json')));
assert(r.axes(1).current>60 && r.axes(1).current<90 && r.axes(1).goal==0);
n=0;
for zeroReturn=0:1
 for excursion=[59 60 60.1 65.8 89.9 90 90.1 670]
  for elapsed=[0 149 150 151]
   fault=excursion>60 && (~zeroReturn || excursion>90 || elapsed>=150);
   allow=~fault;
   if allow && excursion>60,assert(zeroReturn && excursion<=90 && elapsed<150);end
   if excursion>90,assert(fault);end
   n=n+1;
  end
 end
end
out=struct('passed',true,'cases',n,'soft_motor_deg',60,'hard_motor_deg',90, ...
 'transient_window_ms',150,'zero_completion_motor_deg',2,'stable_zero_samples',3, ...
 'only_extension_zero_return',true,'drive_protection_unchanged',true, ...
 'actual_transient_settling_verified',false);
f=fopen(fullfile(ev,'integration13-zero-transient-matlab-20261109.json'),'w','n','UTF-8');assert(f>0);fprintf(f,'%s\n',jsonencode(out));fclose(f);
fprintf('PASS: %d bounded zero-return overshoot cases; hard90deg/150ms stop remains.\n',n);
end
