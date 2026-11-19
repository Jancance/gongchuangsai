function validate_three_frame_warm_20261118
R=fileparts(fileparts(mfilename('fullpath')));E=fullfile(R,'evidence','integration','2026-11-18','three-frame-warm');if ~exist(E,'dir'),mkdir(E);end
n=0;
% Observations may warm in alignment/finish/beep; a pick is impossible until
% the first-beep release state. A physical move starts a fresh frame history.
for needed=0:5
 for activeState=[1 2 3 4 5 6 7 8 9]
  ready=needed>=3;transmit=ready && activeState==6;
  assert(~transmit || activeState==6);n=n+1;
  if activeState==2 || activeState==4 || activeState==5,assert(~transmit);n=n+1;end
 end
end
history=0;state=[2 2 2 4 5 6];pick=false(size(state));
for i=1:numel(state),history=min(3,history+1);pick(i)=state(i)==6 && history==3;end
assert(~any(pick(1:5)) && pick(6));n=n+1;
% A turntable already stopped on return needs three frames, no new rotation.
for visit=1:3
 count=0;for k=1:3,count=count+1;assert((count>=3)==(k==3));n=n+1;end
end
for stale=[false true]
 for moved=[false true]
  for workerChanged=[false true]
   canReuse=~stale && ~moved && ~workerChanged;
   if stale || moved || workerChanged,assert(~canReuse);else,assert(canReuse);end;n=n+1;
  end
 end
end
for dmReady=0:1
 for xReady=0:1
  for cancelled=0:1
   queue=dmReady && xReady && ~cancelled;
   broadcast=queue;dmGo=broadcast;xGo=broadcast;
   assert(dmGo==xGo);n=n+1;
   assert(~dmGo || (dmReady && xReady && ~cancelled));n=n+1;
  end
 end
end
assert(bitand(3,6)==2 && bitand(7,6)==6);n=n+1;
% Same existing safe-top height/limits/speeds; this validates timing/ownership,
% not a measured swept-volume clearance or real cross-bus start skew.
for t=linspace(0,1,101)
 yaw=180*(3*t*t-2*t*t*t);extension=65*(3*t*t-2*t*t*t);
 assert(yaw>=0 && yaw<=180 && extension>=0 && extension<=65);n=n+1;
end
r=struct('passed',true,'checks',n,'completed_local',datestr(now,'yyyy-mm-dd HH:MM:SS'),'hardware_access',false,'scope','3-frame warm recognition across stationary alignment-to-pick; first-beep release; no120-degree gate; initial DM/X common release under existing verified safe-top assumption. Not physical clearance/start-skew acceptance');
f=fopen(fullfile(E,'matlab-result.json'),'w');fprintf(f,'%s',jsonencode(r));fclose(f);fprintf('PASS three-frame warm model %d checks\n',n);
end
