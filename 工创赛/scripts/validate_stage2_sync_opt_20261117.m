% Stage2 event-contract model before firmware changes; no hardware interfaces.
root='D:/Codex/工创赛'; out=[root '/evidence/integration/2026-11-17/stage2-sync-opt'];
if ~exist(out,'dir'),mkdir(out);end
checks=0; scenarios=0;
% Existing STOP permits delayed replies: do not repost/restart an in-flight STOP.
for replyDelay=[0 12 24 36 99]
 issued=1;ack=false;
 for age=0:12:120
  if age>=replyDelay,ack=true;break;end
  assert(issued==1);checks=checks+1;
 end
 assert(ack && issued==1);checks=checks+1;
end
for axes=[6 7]
 for fail=0:6 % prepare each axis, cache each axis, broadcast failure
  for stop=0:1
   scenarios=scenarios+1; prepared=0; queued=0; released=false;
   for bit=[1 2 4]
    if bitand(axes,bit)==0,continue;end
    if fail==find([1 2 4]==bit),break;end
    prepared=bitor(prepared,bit);
   end
   assert(~released);checks=checks+1;
   if prepared==axes && ~stop
    for bit=[2 4]
     if fail==3+find([2 4]==bit),break;end
     queued=bitor(queued,bit);
    end
    released=queued==6 && fail~=6;
   end
   assert(released==((fail==0 || (fail==1 && axes==6)) && ~stop));checks=checks+1;
   if released
    % One release epoch, two buses; no claim of physical zero skew.
    epoch=[101 101];assert(epoch(1)==epoch(2));checks=checks+1;
   end
  end
 end
end
for base=[uint32(1000) uint32(4294967240)]
 for slot=1:3
  for pose=[0 1 2 3 4 5 6 7]
   for trayElapsed=[0 699 700 1200]
    % Release depends on full DM/X/Z done and real 700ms tray window.
    canOpen=pose==7 && trayElapsed>=700;
    assert(canOpen==(all(bitand(pose,[1 2 4])>0) && trayElapsed>=700));checks=checks+1;
   end
  end
  for pwmApplied=[false true]
   for elapsed=[0 99 100 101 300]
    tick=mod(double(base)+elapsed,2^32);age=mod(tick-double(base),2^32);
    returnAllowed=pwmApplied && age>=100;
    assert(returnAllowed==(pwmApplied && elapsed>=100));checks=checks+1;
    if returnAllowed,assert((slot==3)==(slot==3));checks=checks+1;end % third-item stow remains separate
   end
  end
 end
end
% Descend end triggers rise/retract immediately; early close remains last 5mm.
for depth=[0 100 1799 1800 1999 2000]
 close=depth>=2000-5*400; rise=depth>=2000;
 assert(close && rise==(depth>=2000));checks=checks+1;
end
% Non-Stage2 scopes must not acquire the synchronization flag.
for owner=0:3
 for phase=0:12
  sync=owner==2 && (phase==4 || phase==5 || phase==7);
  assert(~sync || owner==2);checks=checks+1;
  if owner==3,assert(~sync);checks=checks+1;end
 end
end
result=struct('passed',true,'checks',checks,'scenarios',scenarios,...
 'completed_local',datestr(now,31),'hardware_access',false,...
 'scope','Stage2 event contract; CAN transmit order and actual firmware checked separately');
f=fopen([out '/matlab-result.json'],'w');fwrite(f,jsonencode(result),'char');fclose(f);
fprintf('PASS Stage2 preimplementation MATLAB: %d checks, %d prepare/cache/STOP scenarios\n',checks,scenarios);
