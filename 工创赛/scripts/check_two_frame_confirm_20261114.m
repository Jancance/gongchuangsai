function check_two_frame_confirm_20261114
% Host transaction model before fixing write-versus-acceptance handling.
root=fileparts(fileparts(mfilename('fullpath')));
out=fullfile(root,'evidence','integration','2026-11-14','top-pick-pb9');
% 3=waiting; first request rejected, second accepted; actual phase4 is the ACK.
phases=[3 3 3 4 5];pending=[0 1 0 0 0];stable=[2 2 2 2 2];
host='CONFIRM';vision=false(size(phases));requests=0;started=false;
for k=1:numel(phases)
 if strcmp(host,'CONFIRM')
  if phases(k)>=4,host='EXECUTE';started=true;
  elseif stable(k)>=2 && ~pending(k),requests=requests+1;end
 end
 vision(k)=strcmp(host,'CONFIRM');
 if phases(k)==3,assert(vision(k) && ~started);end
end
assert(requests==2 && started && isequal(vision,[true true true false false]));
% Actual two-frame rolling gate, including reset after an invalid detection.
valid=[true false true true true];count=0;ready=false(size(valid));
for k=1:numel(valid)
 if valid(k),count=min(count+1,2);else,count=0;end
 ready(k)=count>=2;
end
assert(isequal(ready,[false false false true true]));
% There is no required moving-before-stop stage or extra fixed wait.
r=struct('passed',true,'local_matlab',true,'required_stable_frames',2,...
 'confirm_request_is_not_descent_acceptance',true,...
 'vision_remains_active_until_actual_phase4',true,...
 'rejected_request_may_retry_on_new_frame_without_duplicate_descent',true,...
 'model_scope','host acceptance state and two-frame rolling gate; no new motion/protection',...
 'hardware_access',false,'physical_three_piece_passed',false);
f=fopen(fullfile(out,'two-frame-matlab.json'),'w','n','UTF-8');fprintf(f,'%s\n',jsonencode(r));fclose(f);
fprintf('PASS two-frame host model: retain vision through rejected write; execute only after actual phase4\n');
end
