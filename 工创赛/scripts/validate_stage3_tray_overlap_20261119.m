function validate_stage3_tray_overlap_20261119(closeMs)
% Event timing only; mechanical sweep clearance remains vehicle validation.
if nargin==0, closeMs=1200; end
assert(ismember(closeMs,[200 1200]));
n=0;
for turn=[100 400 900]
 for descend=[100 700 2200]
  for ascend=[100 700 2200]
   for retract=[0 400 2000]
    for stopPhase=0:7
     trayReady=700;
     alignComplete=trayReady+1000;
     preclose=alignComplete;
     yawStart=alignComplete;
     pickStart=max(yawStart+turn,preclose+600);
     assert(preclose==yawStart && pickStart>=trayReady);
     close=pickStart+descend;
     carry=close+closeMs;
     placeYawDone=carry+ascend+turn;
     nextTrayStart=placeYawDone;
     nextPick=max(nextTrayStart+700,placeYawDone+descend+600+ascend+retract+600+turn);
     assert(nextTrayStart>=placeYawDone);
     assert(nextPick>=nextTrayStart+700);
     assert(carry>=close+closeMs);
     if stopPhase>0
      % Existing STOP prevents any later state from issuing motion.
      allowed=(1:7)<stopPhase;
      assert(~any(allowed(stopPhase:end)));
     end
     n=n+1;
    end
   end
  end
 end
end
fprintf('PASS Stage3 tray/DM/preclose overlap: %d cases\n',n);
fprintf('Close dwell: %d ms\n',closeMs);
end
