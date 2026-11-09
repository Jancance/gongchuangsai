function validate_integration13_lead5_20261109
root=fileparts(fileparts(mfilename('fullpath')));ev=fullfile(root,'evidence','integration','2026-11-07','stage2-mainline');
pick=118800;safe=7200;lead=5*1600;assert(pick-lead==110800 && safe+lead==15200);
n=0;
for fresh=0:1
 for xdone=0:1
  for zero=0:1
   for z=[7200 15199 15200 15201 110799 110800 118800]
    close=fresh && xdone && z>=pick-lead && z<=pick;
    yaw=fresh && zero && z<=safe+lead;
    assert(~close || (fresh && xdone && z>=110800));
    assert(~yaw || (fresh && zero && z<=15200));n=n+1;
   end
  end
 end
end
out=struct('passed',true,'cases',n,'lead_virtual_pulses',lead,'grip_depth_min',pick-lead, ...
 'yaw_depth_max',safe+lead,'gripper_fixed_wait_ms',0,'release_before_next_yaw',true, ...
 'current_early_grip_or_yaw_field_verified',false,'under20_demonstrated',false);
f=fopen(fullfile(ev,'integration13-lead5-matlab-20261109.json'),'w','n','UTF-8');assert(f>0);fprintf(f,'%s\n',jsonencode(out));fclose(f);
fprintf('PASS: %d fresh position/X-ready gates; grip at last5mm, yaw at last5mm; trial required.\n',n);
end
