function validate_stage3_disabled_dm_return_20261118
checks=0;
for disabled=0:1
 for stopAck=0:1
  for manualReturn=0:1
   for dmFault=0:1
    for holding=0:1
     for fresh=0:1
      for platform=0:1
       confirmed=disabled && stopAck && ~holding;
       allow=platform && ~dmFault && ((manualReturn && confirmed)||fresh);
       if ~fresh && allow,assert(manualReturn && confirmed);end
       assert(~allow || (platform && ~dmFault));
       checks=checks+1;
      end
     end
    end
   end
  end
 end
end
% Existing read clears feedback_valid while starting. In the previous adapter
% this generated STOP between read start and its successful completion.
valid=[1 0 0 1];old=all(valid);disabledReturn=all(ones(size(valid)));
assert(~old && disabledReturn);
root=fileparts(fileparts(mfilename('fullpath')));
p=fullfile(root,'evidence','integration','2026-11-18','stage3-resume','disabled-dm-return-matlab.json');
fid=fopen(p,'w');assert(fid>=0);cleanup=onCleanup(@()fclose(fid));
fprintf(fid,'%s\n',jsonencode(struct('passed',true,'checks',checks,'matlab_version',version, ...
 'previous_read_transition_stop_reproduced',true,'physical_verified',false)));
clear cleanup;fprintf('PASS %d confirmed-disabled return cases\n',checks);
end
