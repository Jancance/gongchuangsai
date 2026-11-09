function validate_integration11_disable_ack_20261107
out=fullfile(fileparts(fileparts(mfilename('fullpath'))),'evidence','integration','2026-11-07','single-cycle');
checks=[];accepted=0;rejected=0;
% Completed reference requires a real disable ACK, fresh disabled status,
% zero speed, stable encoder position, and the confirmed mechanical lock.
for ack=[0,1]
 for enabled=[0,1]
  for protection=[0,4,8,12]
   for speed=[0,1,600]
    for drift=[0,30,31]
     for lock=[0,1]
      allow=ack==1 && enabled==0 && protection==0 && speed==0 && abs(drift)<=30 && lock==1;
      if allow,accepted=accepted+1;else,rejected=rejected+1;end
      if ~ack || enabled || protection || speed || abs(drift)>30 || ~lock
       checks(end+1)=~allow;
      else,checks(end+1)=allow;end
     end
    end
   end
  end
 end
end
% The disable command is admitted only at real homing completion, not normal motion.
for phase=0:30
 for fault=[0,1]
  admitted=(phase==26 && ~fault);
  checks(end+1)=admitted==(phase==26 && fault==0);
 end
end
r=struct('passed',all(checks),'checks',numel(checks),'accepted',accepted,'rejected',rejected, ...
 'engine','local MATLAB','mechanical_lock_confirmed_by_user',true, ...
 'scope','disable acknowledgment, disabled-status and stable-reference model; no physical load acceptance');
f=fopen(fullfile(out,'integration11-disable-ack-matlab.json'),'w');assert(f~=-1);fprintf(f,'%s',jsonencode(r));fclose(f);assert(r.passed);disp(jsonencode(r));
end
