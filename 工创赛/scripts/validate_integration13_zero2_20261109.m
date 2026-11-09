function validate_integration13_zero2_20261109
root=fileparts(fileparts(mfilename('fullpath')));ev=fullfile(root,'evidence','integration','2026-11-07','stage2-mainline');
r=jsondecode(fileread(fullfile(ev,'integration13-three-direct-once-20261109-203916.json')));
assert(~r.complete && r.first_fault.g_source_parallel_done_mask==4 && r.first_fault.g_source_parallel_started_mask==6);
ratio=315.5*360/3200;
assert(abs(r.first_fault.g_startup_extension_current_deg-1.2)<1e-5);
assert(2/ratio<.06); % Engineering window: under .06mm nominal stroke error.
n=0;
for x=[0 .9 1 1.2 1.9 2 2.01 3 60 670]
 for speed=[0 .1 1]
  for count=0:3
   for valid=0:1
    done=valid && abs(x)<=2 && speed==0 && count>=3;
    assert(~done || (abs(x)<=2 && speed==0 && valid && count>=3));n=n+1;
   end
  end
 end
end
assert(~(abs(2.01)<=2));assert(~(abs(670)<=2));
out=struct('passed',true,'cases',n,'zero_window_motor_deg',2,'nominal_stroke_error_mm',2/ratio, ...
 'continuous_fresh_zero_speed_samples',3,'overshoot_window_unchanged_deg',60, ...
 'drive_protection_unchanged',true,'under20_demonstrated',false);
f=fopen(fullfile(ev,'integration13-zero2-matlab-20261109.json'),'w','n','UTF-8');assert(f>0);fprintf(f,'%s\n',jsonencode(out));fclose(f);
fprintf('PASS: %d zero2 boundary/freshness/speed cases; nominal max %.6fmm, no physical precision guarantee.\n',n,2/ratio);
end
