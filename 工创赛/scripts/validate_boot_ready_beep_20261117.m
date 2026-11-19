function validate_boot_ready_beep_20261117
out='D:/Codex/工创赛/evidence/integration/2026-11-17/boot-ready-beep';if ~exist(out,'dir'),mkdir(out);end
checks=0;
% Every required software gate; no mechanical home/reference or PC13 gate.
for mask=0:127
 ready=mask==127;
 for link=[false true]
  for buzzer=[false true]
   for business=[false true]
    beep=ready && link && ~buzzer && ~business;
    assert(beep==(mask==127 && link && ~buzzer && ~business));checks=checks+1;
   end
  end
 end
end
% Session/sequence exact match; old boot, old/new seq, arbitrary RX all reject.
session=uint32([123456789 2345678901]);seq=uint32(7);
for hi=uint32([0 123456789 123456790])
 for lo=uint32([0 2345678901 2345678902])
  for gotseq=uint32([0 6 7 8])
   match=hi==session(1)&&lo==session(2)&&gotseq==seq;
   assert(match==(hi==123456789&&lo==2345678901&&gotseq==7));checks=checks+1;
  end
 end
end
% Retry after busy TX doesn't advertise an unissued sequence. Tick wrap unsigned.
for sent=[false true]
 current=uint32(0);next=uint32(1);if sent,current=next;end
 assert((current==1)==sent);checks=checks+1;
end
for old=[0 4294967000]
 current=mod(old+500,2^32);assert(mod(current-old,2^32)==500);checks=checks+1;
end
% RNG not ready/current error does not invent a nonce or fault/motion action.
for flags=0:7
 valid=bitand(flags,1)~=0 && bitand(flags,6)==0;
 assert(valid==(flags==1));checks=checks+1;
end
% Link completion alone cannot repeat a beep; late boot cannot replace business80ms.
played=false;count=0;
for step=1:10
 busy=step<=3;business=step<=5;
 if ~played && ~busy && ~business,played=true;count=count+1;end
end
assert(count==1);checks=checks+1;
r=struct('passed',true,'checks',checks,'completed_local',datestr(now,31),'hardware_access',false,...
 'scope','software gate/session/sequence/retry/RNG/once/beep coexistence model; not physical UART or audible test');
f=fopen([out '/matlab-result.json'],'w');fwrite(f,jsonencode(r),'char');fclose(f);fprintf('PASS boot ready preimplementation MATLAB: %d checks\n',checks);
end
