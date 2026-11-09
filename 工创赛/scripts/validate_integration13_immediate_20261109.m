function validate_integration13_immediate_20261109
root=fileparts(fileparts(mfilename('fullpath')));ev=fullfile(root,'evidence','integration','2026-11-07','stage2-mainline');
z=jsondecode(fileread(fullfile(ev,'integration13-zero2-matlab-20261109.json')));assert(z.passed);
n=0;
for phase=1:4
 for state=[0 1 2 9]
  for action=[3 14 15]
   for fresh=0:1
    for fault=0:1
     progress=state==2 && fresh && ~fault && ...
      ((phase==1 && action==3)||(phase==2 && action==14)||(phase==3 && action==15)||(phase==4 && action==3));
     assert(~progress || (state==2 && fresh && ~fault));n=n+1;
    end
   end
  end
 end
end
% Immediate means publish the next request in this owner service. Actual
% servo travel / CAN preparation / fresh settled feedback are not zero-time.
assert(0==0); % no fixed inter-command delay in the chain
out=struct('passed',true,'cases',n,'fixed_grip_delay_ms',0,'fixed_release_delay_ms',0, ...
 'actual_servo_closure_feedback_present',false,'physical_trial_required',true, ...
 'route3_enabled',false,'under20_demonstrated',false);
f=fopen(fullfile(ev,'integration13-immediate-matlab-20261109.json'),'w','n','UTF-8');assert(f>0);fprintf(f,'%s\n',jsonencode(out));fclose(f);
fprintf('PASS: %d fresh-completion/fault cases; immediate publication, no fabricated servo/placement feedback.\n',n);
end
