function validate_stage2_extension_mirror_cancel_20261117
% Interleaving model before the narrow owner-check/enqueue commit repair.
out='D:/Codex/工创赛/evidence/integration/2026-11-17/stage2-extension-mirror';checks=0;
for cancelpoint=0:3
 state=2;vm=1;sent=false;
 if cancelpoint==0,state=6;vm=0;end
 % Atomic guard/enqueue: cancellation can occur before or after, not between.
 if state==2 && vm==1,sent=true;end
 if cancelpoint==1 || cancelpoint==2,state=6;vm=0;end
 % Atomic state commit after ACK must not resurrect a canceled owner.
 if state==2 && vm==1,state=3;end
 if cancelpoint==3,state=6;vm=0;end
 assert(state==6 && vm==0);checks=checks+1;
 if cancelpoint==0,assert(~sent);else,assert(sent);end;checks=checks+1;
end
q=uint8([hex2dec('4c') hex2dec('ae') 0 0 1 0 30 0 0 39 16 1 44 3 32 0 60 0 hex2dec('6b')]);
parts={q(2:8),q(9:15),q(16:19)};
for k=1:3
 bus=[q(1) parts{k}];id=hex2dec('600')+k-1;
 assert(id==1536+k-1 && numel(bus)==8-3*(k==3) && bus(1)==hex2dec('4c'));checks=checks+1;
end
r=struct('passed',true,'checks',checks,'completed_local',datestr(now,31),'hardware_access',false,...
 'scope','atomic owner guard/enqueue and postACK state commit; bus frame supplement; not hardware validation');
f=fopen([out '/matlab-cancel-result.json'],'w');fwrite(f,jsonencode(r),'char');fclose(f);
fprintf('PASS extension mirror cancel preimplementation MATLAB supplement: %d checks\n',checks);
end
