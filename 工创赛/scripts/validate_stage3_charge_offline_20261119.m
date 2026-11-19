function validate_stage3_charge_offline_20261119
% Ordering and pulse-domain model only. No claim about physical clearance.
p = perms(1:3);
n = 0;
for i = 1:6
 for j = 1:6
  for k = 1:6
   firstColors = [2 1 5];
   firstRing = p(i,:);
   secondColors = firstColors(p(j,:));
   slots = firstColors(p(k,:));
   [valid, sameColorIndex] = ismember(secondColors, firstColors);
   assert(all(valid));
   stackRing = firstRing(sameColorIndex);
   assert(isequal(sort(stackRing),1:3));
   for region = 1:3
    onRing = zeros(1,3);
    tray = slots;
    if region == 3
     onRing(firstRing) = firstColors;
     colors = secondColors; rings = stackRing;
    else
     colors = firstColors; rings = firstRing;
    end
    initial = onRing;
    items = 1:3;
    if region == 1, items = 2; end
    for a = items
     s = find(tray == colors(a));
     assert(isscalar(s));
     r = rings(a);
     assert(onRing(r)==0 || (region==3 && onRing(r)==tray(s)));
     onRing(r) = tray(s); tray(s) = 0;
    end
    if region == 2
     for a = items
      r = rings(a);
      s = find(slots == colors(a));
      assert(tray(s)==0 && onRing(r)==colors(a));
      tray(s)=onRing(r); onRing(r)=0;
     end
     assert(isequal(tray,slots) && ~any(onRing));
    elseif region == 3
     assert(~any(tray) && isequal(onRing,initial));
    else
     assert(nnz(onRing)==1 && nnz(tray)==2);
    end
    n = n+1;
   end
  end
 end
end

for dmDone = [10 500 1200]
 for precloseStart = [0 7]
  for xDone = [1 40 180]
   for zDone = [1 80 220]
    descend = max(dmDone,precloseStart+600);
    assert(descend>=dmDone && descend-precloseStart>=600);
    if dmDone>=precloseStart+600, assert(descend==dmDone); end
    yawAtRing = descend+5;
    commonStart = yawAtRing+1;
    release = commonStart+max(xDone,zDone);
    assert(release>=commonStart+xDone && release>=commonStart+zDone);
    n = n+1;
   end
  end
 end
end

for zAtRelease = [36206 59400 60206 61400]
 for rise = [0 2000 8000]
  for margin = [-1 0 1]
   for fresh = [false true]
    for top = [false true]
     for stopped = [false true]
      for zeroError = [0 3 3.01 7.6 17.75]
       feedbackDepth = zAtRelease-rise-margin;
       demonstratedClearance = rise>0 && fresh && feedbackDepth<=zAtRelease-rise;
       mayStartRetract = ~stopped && (top || demonstratedClearance);
       mayRotate = ~stopped && top && zeroError<=3;
       assert(~stopped || (~mayStartRetract && ~mayRotate));
       if ~top && (rise==0 || ~fresh || margin<0), assert(~mayStartRetract); end
       if ~top || zeroError>3, assert(~mayRotate); end
       n=n+1;
      end
     end
    end
   end
  end
 end
end
stackRise=60*400;
assert(stackRise==24000 && 60206-stackRise==36206);
for ground = [23999 24000 27199 27200 60206 82000]
 stackTarget=ground-stackRise;
 allowed=stackTarget>=3200 && stackTarget<=82000;
 assert(allowed==(ground>=27200));
 n=n+1;
end
fprintf('PASS Stage3 offline ordering/feedback/stack arithmetic: %d cases; physical clearance NOT verified\n', n);
end
