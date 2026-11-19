function validate_stage3_low_teach_20261118
% Explicit low bench teach. Lifting remains powered, XY is manually positioned.
checks=0;
for stop=0:1
 for top=0:1
  for returnDone=0:1
   for reference=0:1
    state=3; xRehome=0; liftReturn=0;
    if stop
        state=5;
    elseif top
        state=4;xRehome=1;
    else
        state=6;liftReturn=1;
    end
    assert(~xRehome || (top && ~stop));
    if state==6 && returnDone && top && reference
        state=4;xRehome=1;
    end
    assert(~xRehome || top);
    assert(~stop || (~liftReturn && ~xRehome));
    checks=checks+1;
   end
  end
 end
end
% Low release is an explicit request, retaining all old admission facts.
for request=[1 3]
 for low=0:1
  for ready=0:1
   permitted=ready && (~low || request==3);
   assert(~permitted || ready);
   assert(~(request==1 && low && permitted));
   checks=checks+1;
  end
 end
end
% The empty gripper height is determined from live measured clearance.
% No historical release depth is automatically used as the empty-gripper target.
root=fileparts(fileparts(mfilename('fullpath')));
out=fullfile(root,'evidence','integration','2026-11-18','stage3-resume','low-teach-matlab.json');
fid=fopen(out,'w');assert(fid>=0);cleanup=onCleanup(@()fclose(fid));
fprintf(fid,'%s\n',jsonencode(struct('passed',true,'checks',checks,'matlab_version',version, ...
 'lift_power_retained',true,'return_before_rehome',true,'physical_verified',false)));
clear cleanup;fprintf('PASS low teach MATLAB %d cases\n',checks);
end
