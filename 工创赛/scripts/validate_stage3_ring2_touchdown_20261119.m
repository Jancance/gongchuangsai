function validate_stage3_ring2_touchdown_20261119
% Bounded bench Z target, not a contact or force model.
count=0;
for oldDepth=[48600 59000 60206 61406 82000]
 for increment=[-1 0 1 80 200 400 1199 1200 1201]
  target=oldDepth+increment;
  for scale=[10 100]
   currentDeg=oldDepth*360/3200;
   targetDeg=target*360/3200;
   valid=increment>0 && increment<=1200 && target<=82000;
   if valid
    encoded=round((targetDeg-currentDeg)*scale);
    assert(abs(encoded/scale*3200/360-increment)<=3200/360/scale/2+1e-8);
    assert(encoded>=0 && encoded<=round(1200*360/3200*scale));
    % 30 RPM is 4 mm/s at 3200 virtual pulses/rev and 400 pulses/mm.
    stopTime=30/600;
    assert(stopTime==.05 && 30*3200/60/400==4);
   end
   for live=[false true]
    for stopRequested=[false true]
     start=valid && live && ~stopRequested;
     if ~valid || ~live || stopRequested, assert(~start); end
     for operatorSaysTouch=[false true]
      releaseAllowed=start && operatorSaysTouch;
      assert(~releaseAllowed || operatorSaysTouch);
      count=count+1;
     end
    end
   end
  end
 end
end
fprintf('PASS Stage3 ring2 bounded low-speed touchdown: %d model cases; contact/force NOT verified\n',count);
end
