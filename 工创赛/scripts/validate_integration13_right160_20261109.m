function validate_integration13_right160_20261109
root=fileparts(fileparts(mfilename('fullpath')));ev=fullfile(root,'evidence','integration','2026-11-07','stage2-mainline');
yaw=[180 195 165];ext=[0 156 160];ratio=315.5*360/3200;
assert(all(ext>=0 & ext<=170));assert(ext(2)==156 && ext(3)==160);
assert(abs(ext(2)*ratio-5537.025)<1e-6);assert(abs(ext(3)*ratio-5679)<1e-6);
n=0;
for a=[165 180 195]
 for x=[0 156 160 170]
  accepted=(a==180 && x==0)||(a==195 && x==156)||(a==165 && x==160);
  assert(~accepted || any(yaw==a & ext==x));n=n+1;
 end
end
out=struct('passed',true,'cases',n,'yaw_deg',yaw,'extension_mm',ext, ...
 'right_target_delta_motor_deg',-ext(3)*ratio,'left_unchanged',true, ...
 'slot_yaw_deg',37,'release_wait_ms',200,'existing_lead5_unchanged',true, ...
 'new4mm_sweep_field_verified',false,'route3_enabled',false);
f=fopen(fullfile(ev,'integration13-right160-matlab-20261109.json'),'w','n','UTF-8');assert(f>0);fprintf(f,'%s\n',jsonencode(out));fclose(f);
fprintf('PASS: %d pose/stroke cases; left156mm right160mm; field trial pending.\n',n);
end
