function validate_stage3_manual_xyz_teach_20261118
% Offline contract for the fixed chassis, three-ring manual teach procedure.
% Boolean events stand for the existing live predicates and hardware replies.
cases=0;
for locked=0:1
 for top=0:1
  for idle=0:1
   for dmOff=0:1
    for xOff=0:1
     for fault=0:1
      admitted=locked && top && idle && ~fault;
      canHandPlace=admitted && dmOff && xOff;
      assert(~canHandPlace || all([locked top idle dmOff xOff ~fault]));
      assert(canHandPlace==all([locked top idle dmOff xOff ~fault]));
      normalMotion=~admitted && ~fault;
      assert(~(admitted && normalMotion));
      cases=cases+1;
     end
    end
   end
  end
 end
end
% Only the requested slot changes. Zero and full reach are valid inputs.
targets=nan(3,3); depth=60206;
for ring=[2 1 3]
 before=targets;
 candidate=[180+ring/4, (ring-1)*85, depth];
 assert(validPoint(candidate));
 targets(ring,:)=candidate;
 others=setdiff(1:3,ring);
 assert(isequaln(targets(others,:),before(others,:)));
 cases=cases+1;
end
assert(targets(2,1)~=180 && targets(2,2)~=0);
for candidate={[NaN 1 depth],[180 -1 depth],[180 170.1 depth], ...
        [271 0 depth],[180 0 82001],[180 0 3199]}
 assert(~validPoint(candidate{1}));cases=cases+1;
end
% An exit request removes manual admission. Automatic motion waits for real
% home/hold references and a separately requested next cycle.
for rehomed=0:1
 for hold=0:1
  for start=0:1
   for stop=0:1
    execute=rehomed && hold && start && ~stop;
    assert(~execute || (rehomed && hold && start && ~stop));
    cases=cases+1;
   end
  end
 end
end
result=struct('matlab_version',version,'model_cases',cases,'passed',true, ...
 'measured_release_depth_candidate_pulses',depth,'physical_placement_verified',false, ...
 'implementation_verified',false,'motor_feedback_verified',false, ...
 'scope','Manual ownership, independent disable acknowledgements, three indexed coordinates and exit references');
root=fileparts(fileparts(mfilename('fullpath')));
path=fullfile(root,'evidence','integration','2026-11-18','stage3-resume','manual-xyz-teach-matlab.json');
fid=fopen(path,'w');assert(fid>=0);cleanup=onCleanup(@()fclose(fid));
fprintf(fid,'%s\n',jsonencode(result));clear cleanup;
fprintf('STAGE3 MANUAL TEACH MODEL PASS: %d cases\n',cases);
end

function ok=validPoint(p)
ok=all(isfinite(p)) && all(p>=[0 0 3200]) && all(p<=[270 170 82000]);
end
