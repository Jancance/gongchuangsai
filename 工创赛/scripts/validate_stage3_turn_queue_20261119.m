function validate_stage3_turn_queue_20261119
% Explicitly model producer -> Safety -> CAN and STOP interleavings.
checks=0;
for queue=[0 1]
 for powered=[0 1]
  for healthy=[0 1]
   for stopped=[0 1]
    for backendBusy=[0 1]
     vm=powered;
     oldVm=0; % Existing idle safety unconditionally removes the accepted request.
     preserve=queue && powered && healthy && ~stopped && ~backendBusy;
     fixedVm=oldVm;
     if preserve, fixedVm=vm; end
     canStart=queue && powered && healthy && ~stopped && ~backendBusy && fixedVm==powered;
     if preserve, assert(canStart && fixedVm==1); end
     if stopped || ~healthy || backendBusy, assert(~canStart); end
     if ~queue, assert(fixedVm==oldVm); end
     checks=checks+1;
    end
   end
  end
 end
end
% STOP dominates both before queued Safety and after Safety before CAN.
for boundary=1:3
 request=1; vm=1;
 if boundary==1, request=2; vm=0; end
 if request==1 && vm==1, vm=1; end
 if boundary==2, request=2; vm=0; end
 issued=request==1 && vm==1;
 if boundary<3, assert(~issued); end
 checks=checks+1;
end
fprintf('PASS turn queue VM ownership interleavings: %d cases; not vehicle acceptance\n',checks);
end
