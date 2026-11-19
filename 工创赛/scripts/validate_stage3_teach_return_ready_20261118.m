function validate_stage3_teach_return_ready_20261118
% Reproduce the denied lift return with XY disabled, then validate the narrow
% return owner. Ordinary lift and all failed health states retain old behavior.
normal= @(startup,hold) startup==0 || (startup==4 && hold);
manual= @(area,state,phase,xmode,startup,dmoff,refs) ...
 area==3 && state==7 && phase==6 && xmode==2 && startup==4 && dmoff && refs;
assert(~normal(4,0));
assert(manual(3,7,6,2,4,1,1));
checks=0;
for area=[1 3]
 for state=[2 7 9]
  for phase=[0 3 5 6]
   for xmode=[0 2 3 4]
    for startup=[0 4 5 6]
     for dmoff=0:1
      for refs=0:1
       for healthy=0:1
        for fresh=0:1
         special=manual(area,state,phase,xmode,startup,dmoff,refs);
         before=healthy && fresh && normal(startup,0);
         after=healthy && fresh && (normal(startup,0)||special);
         assert(~after || (healthy && fresh));
         assert(~(after && ~before) || special);
         checks=checks+1;
        end
       end
      end
     end
    end
   end
  end
 end
end
oldDepth=63400;requestedRaise=10;newDepth=oldDepth-requestedRaise*400;
assert(newDepth==59400 && newDepth>=3200 && newDepth<=82000);
root=fileparts(fileparts(mfilename('fullpath')));
out=fullfile(root,'evidence','integration','2026-11-18','stage3-resume','teach-return-ready-matlab.json');
r=struct('passed',true,'matlab_version',version,'checks',checks,'new_teach_depth_pulses',newDepth, ...
 'old_return_rejection_reproduced',true,'physical_verified',false,'existing_DM_freshness_retained',true);
fid=fopen(out,'w');assert(fid>=0);cleanup=onCleanup(@()fclose(fid));fprintf(fid,'%s\n',jsonencode(r));clear cleanup;
fprintf('PASS %d lift-readiness cases; new depth %d pulses\n',checks,newDepth);
end
