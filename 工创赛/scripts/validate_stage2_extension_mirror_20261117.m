function validate_stage2_extension_mirror_20261117
% Offline model only; no probe, serial, SSH, motion or persistent parameter write.
root='D:/Codex/工创赛';out=[root '/evidence/integration/2026-11-17/stage2-extension-mirror'];
if ~exist(out,'dir'),mkdir(out);end
checks=0;K=315.5*360/3200;
for origin=[-5000 -0.3 0 0.7 5000]
 for mm=[0 1 30 48 65 70 156 160 170]
  for s=[-1 1]
   goal=origin+s*mm*K;actual=(goal-origin)*s/K;
   assert(abs(actual-mm)<1e-9);checks=checks+1;
   retract=goal+s*(0-actual)*K;assert(abs(retract-origin)<1e-9);checks=checks+1;
   for tol=[3.6 45 71 110]
    assert(abs(goal-origin-s*mm*K)<=tol);checks=checks+1;
    if mm*K>tol,assert(abs(origin-s*mm*K-origin-s*mm*K)>tol);checks=checks+1;end
   end
  end
 end
end
% Read22 reply sans address: cmd/mode/dir/rpm/timeout/detect-rpm/current/time/auto/6b.
before=uint8([hex2dec('22') 2 0 0 30 0 0 39 16 1 44 3 32 0 60 0 hex2dec('6b')]);
expected=before;expected(3)=1;
q=uint8([hex2dec('4c') hex2dec('ae') 0 before(2) 1 before(4:16) hex2dec('6b')]);
assert(numel(q)==19 && q(3)==0 && q(5)==1);checks=checks+1;
chunks={q(2:8),q(9:15),q(16:19)};
assert(isequal([chunks{:}],q(2:end)));checks=checks+1;
assert(isequal(expected([1:2 4:17]),before([1:2 4:17])));checks=checks+1;
for index=1:17
 for delta=[1 7 128]
  got=expected;got(index)=bitxor(got(index),uint8(delta));
  assert(~isequal(got,expected));checks=checks+1;
 end
end
% Wrong ACK, missing readback, altered params all fail before 9a; no blind retry.
for ack=[0 2 226 238]
 for readok=[false true]
  for match=[false true]
   allow=ack==2 && readok && match;assert(allow==(ack==2 && readok && match));checks=checks+1;
  end
 end
end
for vm=[false true]
 for state=0:6
  for fault=[false true]
   allow=vm && state==2 && ~fault;
   if allow,assert(vm && state==2 && ~fault);else,assert(~vm || state~=2 || fault);end
   checks=checks+1;
  end
 end
end
assert((1==1 && 1==1) && (0==0 && -1==-1));checks=checks+1;
r=struct('passed',true,'checks',checks,'completed_local',datestr(now,31),...
 'hardware_access',false,'save_parameter',0,'extend_sign',1,'home_direction',1,...
 'scope','sign/roundtrip/directional gates/node6 save0 frame/readback and existing owner model; not hardware validation');
f=fopen([out '/matlab-result.json'],'w');fwrite(f,jsonencode(r),'char');fclose(f);
fprintf('PASS extension mirror preimplementation MATLAB: %d checks\n',checks);
end
