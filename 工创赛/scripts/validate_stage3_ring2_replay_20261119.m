function validate_stage3_ring2_replay_20261119(forwardMm)
% Replay ordering model. No claim about chassis travel accuracy or jaw feedback.
steps=[0 7;0 5;0 2;2 2;0 1;2 1;2 1;0 1;2 1];
if nargin==0, forwardMm=16; end
assert(ismember(forwardMm,[11 16]));
steps(1,2)=steps(1,2)+(forwardMm-16);
assert(sum(steps(steps(:,1)==0,2))==forwardMm);
assert(sum(steps(steps(:,1)==2,2))==5);
checks=0;
for stopAt=0:12
 for rejectedStep=0:9
  for staleStep=0:9
   for firstSequence=[0 100 4294967294]
    done=0; sequence=firstSequence;
    for k=1:size(steps,1)
     if k==stopAt || k==rejectedStep, break; end
     received=sequence+1;
     if received>=4294967296, received=1; end
     if k==staleStep, received=sequence; end
     if received==sequence, break; end
     sequence=received; done=done+1;
    end
    ended=done==9 && stopAt~=10;
    relocked=ended && stopAt~=11;
    beep=relocked;
    pickReady=beep && stopAt~=12;
    if stopAt==0 && rejectedStep==0 && staleStep==0
     assert(pickReady && beep && done==9);
    end
    assert(~beep || (done==9 && ended && relocked));
    assert(~pickReady || beep);
    assert(~(done<9 && (beep || pickReady)));
    checks=checks+1;
   end
  end
 end
end
for contact=[false true]
 for jawOpened=[false true]
  open=contact;
  rise=open && jawOpened;
  assert(~rise || (contact && jawOpened));
  assert(~(contact && ~jawOpened && rise));
  checks=checks+1;
 end
end
fprintf('PASS Stage3 ring2 replay compensation and release ordering: %d cases\n',checks);
fprintf('Recipe: forward %d mm, right 5 mm, nine steps\n',forwardMm);
end
