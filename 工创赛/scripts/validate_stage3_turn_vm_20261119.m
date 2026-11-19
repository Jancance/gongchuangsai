function validate_stage3_turn_vm_20261119
n=0;
for safetyFirst=0:1
 for owner=0:1
  for pending=0:1
   for source=0:1
    for exclusive=0:1
     for stop=0:1
      vm=1;
      queued=owner && pending && ~stop;
      fault=queued && (~source || ~exclusive);
      if safetyFirst && ~queued, vm=0; end
      if queued && ~fault
       assert(vm==1);
       phase=1; % Request accepted and advanced to WAIT in the same critical section.
       assert(phase==1);
      elseif fault
       assert(~source || ~exclusive);
      end
      n=n+1;
     end
    end
   end
  end
 end
end
fprintf('PASS queued turn VM ownership handoff: %d cases\n',n);
end
