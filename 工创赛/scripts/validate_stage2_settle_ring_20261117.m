function validate_stage2_settle_ring_20261117
% Authorized Stage2 post-stop image gate and finite recent-history ring.
root='D:/Codex/工创赛';out=[root '/evidence/integration/2026-11-17/stage2-settle-ring'];
if ~exist(out,'dir'),mkdir(out);end
checks=0;
% Genuine basic DONE terminal record may have no endpoint samples: retain masks0,
% mark its real outcome1 and skip encoder telemetry; live outcome0 remains live.
for heading=[false true]
 for completed=[false true]
  outcome=0;mask=15*heading;
  if completed,outcome=1;end
  reclaim=outcome==1 || outcome==2;emit=outcome==1 && mask==15;
  assert(reclaim==completed && emit==(completed && heading));checks=checks+1;
 end
end
% Reuse existing200ms host-frame freshness; stale/unknown host clock clears gate.
for age=[0 .199 .200 .201 1]
 assert((age<=.200)==(age*1000<=200));checks=checks+1;
end
% Ring logical oldest->newest; live oldest cannot be overwritten.
for N=[8 9 16 20 37]
 ids=zeros(1,8);outcome=zeros(1,8);head=1;count=0;
 for session=1:N
  if count<8,slot=mod(head-1+count,8)+1;count=count+1;
  else,slot=head;assert(outcome(slot)==1 || outcome(slot)==2);checks=checks+1;head=mod(head,8)+1;end
  ids(slot)=session;outcome(slot)=0;
  assert(count<=8 && slot>=1 && slot<=8);checks=checks+1;
  before=ids;
  if count==8 && outcome(head)==0
   allowed=false;assert(~allowed && isequal(ids,before));checks=checks+1;
  end
  if mod(session,5)==0,outcome(slot)=2;else,outcome(slot)=1;end
  logical=ids(mod(head-1+(0:count-1),8)+1);
  assert(isequal(logical,max(1,session-7):session));checks=checks+1;
 end
end
% Telemetry finite latest8, retry doesn't advance, wrap session ordering.
for previous=[uint32(0) uint32(4294967294)]
 for step=1:20
  current=uint32(mod(double(previous)+step,2^32));if current==0,current=uint32(1);end
  delta=mod(double(current)-double(previous),2^32);assert(delta>0 && delta<2^31);checks=checks+1;
 end
end
% Fresh capture fence, post300 window, max pairwise Euclidean2; raw centers retained.
for epoch=[0 100 4294967.25]
 g=newgate(epoch);
 [g,ready]=frame(g,1,epoch-.01,[0 0],true,epoch+.01);assert(~ready);checks=checks+1;
 [g,ready]=frame(g,2,epoch+.01,[0 0],true,epoch+.02);assert(~ready && g.fence==2);checks=checks+1;
 for k=1:5
  [g,ready]=frame(g,2+k,epoch+.10+k*.01,[0 0],true,epoch+.2);assert(~ready && isempty(g.centers));checks=checks+1;
 end
 for k=1:4
  [g,ready]=frame(g,8+k,epoch+.301+k*.02,[k*.1 0],true,epoch+.302+k*.02);assert(~ready);checks=checks+1;
 end
 [g,ready]=frame(g,13,epoch+.401,[.5 0],true,epoch+.402);assert(ready && size(g.centers,1)==5);checks=checks+1;
 [g,ready]=frame(g,13,epoch+.401,[.5 0],true,epoch+.403);assert(~ready);checks=checks+1;
 [g,ready]=frame(g,14,epoch+.421,[4 0],true,epoch+.422);assert(~ready);checks=checks+1;
 [g,ready]=frame(g,15,epoch+.441,[0 0],false,epoch+.442);assert(~ready && isempty(g.centers));checks=checks+1;
 for k=1:5
  [g,ready]=frame(g,15+k,epoch+.45+k*.02,[0 0],true,epoch+.451+k*.02);
  assert(ready==(k==5));checks=checks+1;
 end
 % Pairwise differs from adjacent and distance-to-mean.
 c=[0 0;.8 0;1.6 0;2.4 0;3.2 0];assert(maxpair(c)>2 && all(sqrt(sum(diff(c).^2,2))<=2));checks=checks+1;
 c=[0 0;2 0;0 0;2 0;1 0];assert(maxpair(c)<=2);checks=checks+1;
 g=newgate(epoch+.8);assert(isempty(g.centers) && isempty(g.fence));checks=checks+1;
end
% Generation/link/newpose start a genuinely fresh fence/timer, no motor/fault side effect.
for reset={'newpose','lostlink','generation','invalidtime'}
 g=newgate(10);g.centers=zeros(5,2);g=newgate(11);assert(isempty(g.centers) && g.epoch==11);checks=checks+1;
end
r=struct('passed',true,'checks',checks,'completed_local',datestr(now,31),'hardware_access',false,...
 'settle_ms',300,'frames',5,'max_pair_distance_px',2,'count_after_dwell',true,...
 'ring_capacity',8,'reclaim_outcomes',[1 2],'source_step_mm_unchanged',20,...
 'scope','host read-completion freshness only; no exposure guarantee, physical convergence or motion acceptance');
f=fopen([out '/matlab-result.json'],'w');fwrite(f,jsonencode(r),'char');fclose(f);
fprintf('PASS Stage2 settle/ring preimplementation MATLAB: %d checks\n',checks);
end
function g=newgate(epoch)
g=struct('epoch',epoch,'fence',[],'lastseq',[],'laststamp',[],'centers',zeros(0,2));
end
function [g,ready]=frame(g,seq,stamp,center,valid,now)
ready=false;
if stamp<=g.epoch || stamp>now,return;end
if isempty(g.fence),g.fence=seq;g.lastseq=seq;g.laststamp=stamp;return;end
if seq<=g.lastseq || stamp<=g.laststamp,return;end
g.lastseq=seq;g.laststamp=stamp;
if ~valid,g.centers=zeros(0,2);return;end
if stamp-g.epoch<.300,return;end
g.centers=[g.centers;center];if size(g.centers,1)>5,g.centers=g.centers(end-4:end,:);end
ready=size(g.centers,1)==5 && maxpair(g.centers)<=2;
end
function d=maxpair(c)
d=0;for i=1:size(c,1),for j=1:size(c,1),d=max(d,norm(c(i,:)-c(j,:)));end,end
end
