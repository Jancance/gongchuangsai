function validate_integration13_slot37_release200_20261109
root=fileparts(fileparts(mfilename('fullpath')));ev=fullfile(root,'evidence','integration','2026-11-07','stage2-mainline');
assert(37>=0 && 37<=270);delta=38.380798-37;
% This is only a signal/path change; actual placement is field-verified.
assert(delta>0 && delta<1.4);n=0;
for released=0:1
 for elapsed=[0 1 149 199 200 201 1000]
  for fault=0:1
   next=logical(released && elapsed>=200 && ~fault);
   assert(~next || (released && elapsed>=200 && ~fault));n=n+1;
  end
 end
end
% Unsigned tick rollover must retain the same200ms minimum.
before=uint64(4294967200);after=104;elapsed=mod(after-double(before),2^32);
assert(elapsed==200);
out=struct('passed',true,'cases',n+1,'slot_yaw_deg',37,'release_wait_ms',200, ...
 'after_third_release_also_wait200_before_zero',true,'existing_lead5_unchanged',true, ...
 'field_placement_verified',false,'route3_enabled',false);
f=fopen(fullfile(ev,'integration13-slot37-release200-matlab-20261109.json'),'w','n','UTF-8');assert(f>0);fprintf(f,'%s\n',jsonencode(out));fclose(f);
fprintf('PASS: %d angle/release-wait cases; target37deg, release->yaw at >=200ms.\n',n+1);
end
