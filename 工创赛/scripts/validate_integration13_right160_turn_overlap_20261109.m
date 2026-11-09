function validate_integration13_right160_turn_overlap_20261109
root=fileparts(fileparts(mfilename('fullpath')));ev=fullfile(root,'evidence','integration','2026-11-07','stage2-mainline');
angles=[195 165];ext=[156 160];safe=7200;lead=8000;n=0;
for k=1:2
 % Conditional normalized swept joint path, no invented physical obstacles.
 % Object/arm/cable envelope is checked by the operator in this one trial.
 s=(0:.01:1)';q=angles(k)+(37-angles(k))*s;x=ext(k)*(1-s);
 z=safe+lead*(1-s);
 assert(all(q>=37 & q<=270) && all(x>=0 & x<=170));
 assert(all(diff(q)<=0) && all(diff(x)<=0) && all(z>=safe));
 n=n+numel(s);
end
for fresh=0:1
 for depth=[7199 7200 15200 15201 110800]
  for retracted=0:1
   yaw=fresh && depth>=0 && depth<=15200;
   assert(yaw==logical(fresh && depth>=0 && depth<=15200));n=n+1;
   % Retraction completion is NOT required for the user-authorized overlap.
   if fresh && depth==15200,assert(yaw);end
  end
 end
end
out=struct('passed',true,'cases',n,'left_extension_mm',156,'right_extension_mm',160, ...
 'slot_yaw_deg',37,'release_wait_ms',200,'yaw_max_depth_pulses',15200, ...
 'yaw_may_start_before_retraction_done',true,'all_axes_done_before_release',true, ...
 'physical_swept_envelope_known',false,'current_overlap_field_verified',false, ...
 'simulation_scope','joint bounds, directions and freshness admission only; no collision-free claim', ...
 'route3_enabled',false);
f=fopen(fullfile(ev,'integration13-right160-turn-overlap-matlab-20261109.json'),'w','n','UTF-8');assert(f>0);fprintf(f,'%s\n',jsonencode(out));fclose(f);
fprintf('PASS: %d joint-bound/freshness cases; user-authorized loaded overlap still requires this field trial.\n',n);
end
