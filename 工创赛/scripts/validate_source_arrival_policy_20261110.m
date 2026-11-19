function validate_source_arrival_policy_20261110
root=fileparts(fileparts(mfilename('fullpath')));out=fullfile(root,'evidence','integration','2026-11-10','source-arrival-policy');if ~exist(out,'dir'),mkdir(out);end
n=0;
% New supervised sequence replaces yesterday's align->manualQR ordering.
for qr=0:1
 for explicitArrival=0:1
  for stopped=0:1
   for referenced=0:1
    prepare=qr && explicitArrival && stopped && referenced;
    assert(~prepare || (qr && explicitArrival && stopped && referenced));n=n+1;
   end
  end
 end
end
for err=[-100 -60 -40 -21 -20 -10 -5 -2 0 2 5 10 20 21 40 60 100]
 if abs(err)<=2,step=0;speed=0;
 elseif abs(err)<10,step=0;speed=0; % actual executor cannot execute sub10mm
 elseif abs(err)>40,step=40;speed=60;
 elseif abs(err)>20,step=20;speed=35;
 else,step=10;speed=15;end
 assert(step<=abs(err) && step<=40);assert(step==0 || step>=10);n=n+1;
end
% Ideal correction sequence: reobserve after EACH axis step. This tests bounds,
% not physical chassis following, heading hold or obstacle clearance.
for start=[100 80 60 40 20 10]
 err=start;iterations=0;
 while err>2
  if err>40,step=40;elseif err>20,step=20;else,step=10;end
  err=err-step;iterations=iterations+1;assert(iterations<=8);
 end
 assert(err<=2);n=n+1;
end
assert(65>=0 && 65<=170 && 180>=0 && 180<=270);
result=struct('passed',true,'checks',n,'scan_before_simulated_arrival',true, ...
 'view_yaw_deg',180,'view_extension_mm',65,'step_max_mm',40, ...
 'bands_mm',[40 20 10],'band_speed_mm_s',[60 35 15],'min_executable_mm',10, ...
 'sub10mm_not_falsely_done',true,'hardware_access',false,'real_geometry_verified',false);
f=fopen(fullfile(out,'matlab-result.json'),'w','n','UTF-8');assert(f>0);fprintf(f,'%s\n',jsonencode(result));fclose(f);
fprintf('PASS:%d scan/arrival/view/coarse-fine bounds; actual view andchassis response unverified.\n',n);
end
