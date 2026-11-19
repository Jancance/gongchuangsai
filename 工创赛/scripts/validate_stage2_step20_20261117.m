% User-selected business step cap20; not a new protection threshold.
root='D:/Codex/工创赛';out=[root '/evidence/integration/2026-11-17/stage2-debug215-step20'];
if ~exist(out,'dir'),mkdir(out);end
v=[-2147483647 -30000 -20001 -20000 -19999 -19500 -1500 -1499 -500 -499 -1 0 1 499 500 1499 1500 19500 19999 20000 20001 30000 2147483647];
checks=0;
for right=v
 for forward=v
  axis=abs(right)<abs(forward);amount=right;if axis,amount=forward;end
  distance=max(1,min(20,floor((abs(amount)+500)/1000)));
  if axis,direction=3;if amount>0,direction=0;end
  else,direction=1;if amount>0,direction=2;end,end
  assert(distance>=1 && distance<=20);checks=checks+1;
  assert(axis==(abs(right)<abs(forward)));checks=checks+1;
  assert((axis && (direction==0 || direction==3)) || (~axis && (direction==1 || direction==2)));checks=checks+1;
  if abs(amount)>=19500,assert(distance==20);checks=checks+1;end
  if abs(amount)<500,assert(distance==1);checks=checks+1;end
 end
end
for error=[0 1 4999 5000 5001 38000]
 assert((error<=5000)==(error/1000<=5));checks=checks+1;
end
% Ideal declared response only: remeasure residual each step, no fixed move list.
traces={};
for start=[-81 -39 -20 -6 6 20 39 81]
 residual=start;trace=[];
 for step=1:20
  if abs(residual)<=5,break;end
  distance=max(1,min(20,floor(abs(residual)+.5)));
  trace(end+1)=sign(residual)*distance;residual=residual-sign(residual)*distance;
  assert(abs(trace(end))<=20);checks=checks+1;
 end
 assert(abs(residual)<=5);checks=checks+1;traces{end+1}=trace;
end
result=struct('passed',true,'checks',checks,'completed_local',datestr(now,31),...
 'hardware_access',false,'sequence_model','Ideal exact response only; does not establish live mapping or convergence',...
 'source_step_max_mm',20,'source_step_min_mm',1,'alignment_stop_mm',5,'sequence_traces',{traces});
f=fopen([out '/matlab-result.json'],'w');fwrite(f,jsonencode(result),'char');fclose(f);
fprintf('PASS Stage2 step20 preimplementation MATLAB: %d checks\n',checks);
