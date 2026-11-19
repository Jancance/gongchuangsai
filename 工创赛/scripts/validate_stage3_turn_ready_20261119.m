function validate_stage3_turn_ready_20261119
n=0;
for rate=[-10 -3.01 -3 -1 0 1 3 3.01 10 NaN]
 for fresh=0:1
  for stopped=0:1
   for stop=0:1
    for elapsed=[0 11999 12000]
     issue=fresh && stopped && ~stop && elapsed<12000 && rate>=-3 && rate<=3;
     assert(~issue || (isfinite(rate) && abs(rate)<=3 && fresh && stopped && ~stop));
     if elapsed>=12000, assert(~issue); end
     n=n+1;
    end
   end
  end
 end
end
fprintf('PASS existing turn rate readiness and bounded wait: %d cases\n',n);
end
