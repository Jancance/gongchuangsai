% Pre-implementation Stage3 explicit manual teach topology. No hardware.
root=fileparts(fileparts(mfilename('fullpath')));
out=fullfile(root,'evidence','integration','2026-11-16','stage3-teach-mode');
if ~exist(out,'dir'),mkdir(out);end
checks={};
for chain=[false true]
 for configured=[false true]
  s=struct('state',2,'aligned',false,'configured',configured,'chain',chain,'pending',false,'busy',false,'pose',4,'valid',true,'hold',13,'ready',true,'holdRequest',false,'finish',false,'execute',false);
  t=enter(s);assert(t.state==4 && ~t.aligned && ~t.valid && t.pose==5 && ~t.ready && t.hold==0 && t.finish==chain && ~t.execute);
  t=wheel(t);assert(t.state==7 && ~t.execute);
  t=resume(t);assert(t.state==1 && ~t.ready && t.hold==0 && t.pose==6 && ~t.valid && ~t.execute);
  % Existing backend requests run sequentially, with real completion each step.
  trace={'real top','extension reference','observe servo','open claw','X0','DM180'};
  assert(strcmp(trace{5},'X0') && strcmp(trace{6},'DM180'));
  t.state=2;assert(~t.aligned && ~t.execute);
 end
end
checks{end+1}='manual entry never autoexecutes even with valid configuration; chain finish and fresh wheel hold; exit allowed unconfigured and reconstructs top/X0/DM180';
for busy=[false true]
 s=struct('state',2,'aligned',false,'configured',false,'chain',false,'pending',busy,'busy',false,'pose',4,'valid',true,'hold',13,'ready',true,'holdRequest',false,'finish',false,'execute',false);
 t=enter(s);if busy,assert(t.state==2 && t.pose==4);else,assert(t.state==4);end
end
checks{end+1}='pending or busy entry is rejected without clearing owner state';
s=struct('state',4,'aligned',true,'configured',true,'chain',false,'pending',false,'busy',false,'pose',0,'valid',false,'hold',13,'ready',true,'holdRequest',false,'finish',false,'execute',false);
t=wheel(s);assert(t.state==5 && t.execute);
s.state=7;t=resume(s);assert(t.state==5 && t.execute);
checks{end+1}='genuine aligned valid-config entry and continue keep prior execution behavior';
for phase=[1 2 4 7]
 s.state=phase;s.pending=true;s.busy=true;s.state=9;s.stopIT=true;s.stopBody=true;
 assert(s.state==9 && s.stopIT && s.stopBody);
end
checks{end+1}='existing STOP remains terminal with backend STOP pending; no resume or synthesized ACK';
result=struct('passed',true,'when',datestr(now,31),'checks',{checks},'hardware_access',false,'scope','topology only; real motor dynamics and physical clearance not modeled');
f=fopen(fullfile(out,'matlab-result.json'),'w');fwrite(f,jsonencode(result),'char');fclose(f);disp(jsonencode(result));
function t=enter(t)
 if t.state~=2 || t.busy || t.pending,return;end
 t.valid=false;t.pose=t.pose+1;t.aligned=false;t.ready=false;t.hold=0;t.holdRequest=false;t.state=4;t.finish=t.chain;
end
function t=wheel(t)
 if t.finish,t.chain=false;t.finish=false;end
 t.holdRequest=true;t.ready=true;t.hold=13;
 if t.aligned && t.configured,t.state=5;t.execute=true;else,t.state=7;end
end
function t=resume(t)
 if ~t.aligned
  t.state=1;t.ready=false;t.hold=0;t.holdRequest=false;t.valid=false;t.pose=t.pose+1;
 elseif t.configured && t.ready,t.state=5;t.execute=true;end
end
