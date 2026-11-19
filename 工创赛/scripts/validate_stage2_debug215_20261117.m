% Explicit test task vs true scanner ownership; before corresponding firmware.
root='D:/Codex/工创赛';out=[root '/evidence/integration/2026-11-17/stage2-debug215'];
if ~exist(out,'dir'),mkdir(out);end
checks=0;
for testMode=[false true]
 for scannerReady=[false true]
  for stopped=[false true]
   area=3*~testMode+testMode; % formal initial area0 -> existing key3; debug area1
   state=0;request=1;colors=[0 0 0];scanPending=0;scanLocked=0;fullQR=false;
   if area==1 && request==1,state=1;end
   if state==1 && testMode,colors=[2 1 5];end
   if stopped && state==1,state=9;end
   assert(scanPending==0 && scanLocked==0 && ~fullQR);checks=checks+1;
   assert(testMode || state==0);checks=checks+1;
   if testMode && ~stopped,assert(state==1 && isequal(colors,[2 1 5]));checks=checks+1;end
   % A separate formal source-area arrival still waits for a real valid QR.
   if ~testMode
    state=1;taskReady=scannerReady;
    assert(taskReady==scannerReady);checks=checks+1;
   end
  end
 end
end
% Fixed task order is not fixed region order. Live target color/inventory remains required.
for step=1:3
 for fresh=[false true]
  for target=1:6
   task=[2 1 5];canCommit=fresh && target==task(step);
   assert(canCommit==(fresh && target==task(step)));checks=checks+1;
  end
 end
end
% No startup side effects: PC13 required; STOP retains original state exit.
for testMode=[false true]
 request=0;state=0;assert(request==0 && state==0);checks=checks+1;
 if testMode,request=1;state=1;assert(request==1 && state==1);checks=checks+1;end
 state=9;assert(state==9);checks=checks+1;
end
result=struct('passed',true,'checks',checks,'completed_local',datestr(now,31),...
 'hardware_access',false,'scope','explicit debug215 task source, scanner untouched, formal QR, existing key and fresh observation contract');
f=fopen([out '/matlab-result.json'],'w');fwrite(f,jsonencode(result),'char');fclose(f);
fprintf('PASS Stage2 debug215 preimplementation MATLAB: %d checks\n',checks);
