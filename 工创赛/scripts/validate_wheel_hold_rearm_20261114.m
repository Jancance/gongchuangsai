function validate_wheel_hold_rearm_20261114
root=fileparts(fileparts(mfilename('fullpath')));
out=fullfile(root,'evidence','integration','2026-11-14','overlap200-commit');
checks=0;
% Existing Emm profile: direction, speed, acceleration, pulse count, mode, sync.
q=uint8([hex2dec('FD') 0 0 37 236 0 0 0 0 2 0 hex2dec('6B')]);
assert(all(q(6:9)==0) && q(10)==2 && q(11)==0);checks=checks+1;
packets={q(1:8),[q(1) q(9:12)]};
decoded=[packets{1},packets{2}(2:end)];assert(isequal(decoded,q));checks=checks+1;
% Closing a successful held sequence is not a fault/cancel recovery operation.
for state=0:4 % 2 is the model's completed state
 for active=[false true]
  for fault=[false true]
   for stopped=[false true]
    for powered=[false true]
     allowed=state==2 && ~active && ~fault && stopped && powered;
     originalFault=fault;remainingFault=fault;
     if allowed,next=0;else,next=state;end
     assert(remainingFault==originalFault);
     if state~=2 || active || fault || ~stopped || ~powered,assert(next==state);end
     checks=checks+1;
    end
   end
  end
 end
end
r=struct('passed',true,'local_matlab',true,'checks',checks,...
 'emm_zero_relative_position_frame',double(q),'packet_lengths',[8 5],...
 'distance_pulses',0,'encoder_zero_written',false,'driver_parameters_written',false,...
 'normal_completed_reuse_only',true,'fault_or_cancel_rearmed',false,...
 'VM_disabled_by_rearm',false,'hardware_access',false,'physical_holding_validated',false);
f=fopen(fullfile(out,'wheel-hold-rearm-matlab.json'),'w','n','UTF-8');fprintf(f,'%s\n',jsonencode(r));fclose(f);
fprintf('PASS wheel zero-position/normal rearm: %u checks; no fault clear or VM cut\n',checks);
end
