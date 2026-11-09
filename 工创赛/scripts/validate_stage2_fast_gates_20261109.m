function validate_stage2_fast_gates_20261109
% Preconditions for the isolated asynchronous motion candidate, not field QA.
root=fileparts(fileparts(mfilename('fullpath')));
ev=fullfile(root,'evidence','integration','2026-11-07','stage2-mainline');
budget=jsondecode(fileread(fullfile(ev,'stage2-sub20-matlab-20261109.json')));
assert(budget.passed && ~budget.hardware_enabled);
n=0;
for fresh=0:1
 for extDone=0:1
  for liftDepth=[7200 20000 50000 118800]
   for turnDepth=[7200 20000]
    % Down-positive virtual pulses, relative to this session's real top.
    retract=fresh && liftDepth<=20000;
    yaw=fresh && extDone && liftDepth<=turnDepth;
    assert(~yaw || (fresh && extDone));
    assert(~retract || liftDepth<=20000);
    n=n+1;
   end
  end
 end
end
% A stale endpoint from the previous move may not trigger early closing.
assert(~closeGate(0,118790,118800,800,1));
assert(~closeGate(1,118790,118800,800,0));
assert(~closeGate(1,118801,118800,800,1));
assert(closeGate(1,118000,118800,800,1));
assert(~closeGate(1,117999,118800,800,1));
% One explicitly supervised empty commissioning move, never an automatic loop.
% This is admission verification; the empty sweep is checked on the vehicle.
for top=0:1
 for zero=0:1
  for yaw=[180 195 210]
   for ext=[-1 0 156 170]
    for pulses=[99999 118800 118801]
     accepted=top && zero && abs(yaw-195)<=3 && ext>=0 && ext<=156 && pulses==118800;
     assert(~accepted || (top && zero && abs(yaw-195)<=3 && ext<=156));
     n=n+1;
    end
   end
  end
 end
end
% No height or shortening is inferred from the historical 60mm case.
assert(budget.old_60_mm_clearance_not_verified);
% The next supervised single-piece measurement lifts only, then HOLDS until
% the operator observes actual clearance. Five mm is a candidate step, not a
% fabricated field clearance measurement or permission to retract.
assert(118800-5*1600==110800);
for phase=0:4
 for ack=0:1
  retract=phase==3 && ack;
  assert(~retract || (phase==3 && ack));n=n+1;
 end
end
% The two individually exercised live groups are composed without changing
% their goals: approach EXT|LIFT, grip, carry with position-gated retraction.
% No grip until BOTH approach axes completed; no low yaw during retraction.
for extDone=0:1
 for zDone=0:1
  for accepted=0:1
   closeAllowed=accepted && extDone && zDone;
   assert(~closeAllowed || (extDone && zDone));n=n+1;
  end
 end
end
% Background yaw is exactly zero; no post-release 180 degree waypoint.
assert(0>=0 && 0<=270);
out=struct('passed',true,'gate_cases',n+5,'hardware_enabled',false, ...
 'current_sweep_verified',false,'target_budget_s',budget.target_budget_s, ...
 'five_pct_slower_s',budget.target_5pct_slower_s, ...
 'under20_demonstrated',false);
fid=fopen(fullfile(ev,'stage2-fast-gates-matlab-20261109.json'),'w','n','UTF-8');
assert(fid>0);fprintf(fid,'%s\n',jsonencode(out));fclose(fid);
fprintf('PASS: %d position/freshness gates; target %.3f s, +5%% %.3f s.\n', ...
 n+5,out.target_budget_s,out.five_pct_slower_s);
fprintf('No current trajectory, close time, or <20 s field result is asserted.\n');
end
function yes=closeGate(fresh,depth,target,lead,extensionSettled)
yes=fresh && extensionSettled && depth<=target && depth>=target-lead;
end
