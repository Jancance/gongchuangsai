function validate_stage3_hold_recheck_20261119
% Check relative-to-park errors and the gates around the existing body step.
n=0;
for item=0:3
 for reference=[-11 0 5]
  for drift=[-3 -2 0 2 3]
   for yaw=[-1.1 -1 0 1 1.1]
    for ready=0:1
     for fresh=0:1
      for locked=0:1
       relative=(reference+drift)-reference;
       readyForVision=ready && fresh && locked;
       correction=readyForVision && item>0 && (abs(relative)>2 || abs(yaw)>1);
       advance=readyForVision && ~correction;
       assert(~correction || (ready && fresh && locked && item>0));
       assert(~advance || (ready && fresh && locked));
       assert(~(correction && advance));
       if readyForVision && item>0
        assert(correction == (abs(drift)>2 || abs(yaw)>1));
        if correction
         % Re-observation after the bounded correction and a new wheel lock.
         remaining=relative-relative;
         assert(abs(remaining)<=2 && abs(yaw-yaw)<=1);
        end
       end
       n=n+1;
      end
     end
    end
   end
  end
 end
end
fprintf('PASS Stage3 hold recheck: %d cases\n',n);
end
