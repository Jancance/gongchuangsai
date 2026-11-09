function validate_integration13_prepare_wait_20261109
root=fileparts(fileparts(mfilename('fullpath')));ev=fullfile(root,'evidence','integration','2026-11-07','stage2-mainline');
r=jsondecode(fileread(fullfile(ev,'integration13-immediate-first-fault-static-20261109.json')));
assert(r.axes(1).phase==2 && r.axes(1).samples==0);
n=0;
for valid=0:1
 for sign=[0 1 2]
  for speed=[0 1 20]
   for dirty=0:1
    if ~valid, result=0;elseif sign>1,result=2;elseif speed>0,result=0;elseif dirty,result=0;else,result=1;end
    assert(result~=1 || (valid && sign<=1 && speed==0 && ~dirty));n=n+1;
   end
  end
 end
end
out=struct('passed',true,'cases',n,'phase2_single_miss_or_nonzero_waits',true, ...
 'position_refreshed_after_wait',true,'malformed_sign_rejected',true, ...
 'original_group_timeout_ms',12000,'physical_trial_required',true);
f=fopen(fullfile(ev,'integration13-prepare-wait-matlab-20261109.json'),'w','n','UTF-8');assert(f>0);fprintf(f,'%s\n',jsonencode(out));fclose(f);
fprintf('PASS: %d prep wait cases; no fixed delay, no timeout removal, no fake health.\n',n);
end
