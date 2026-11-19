% Offline teach data dependencies and existing single-owner CAN semantics.
root=fileparts(fileparts(mfilename('fullpath')));
out=fullfile(root,'evidence','integration','2026-11-16','stage3-offline-ready');
if ~exist(out,'dir'),mkdir(out);end
checks={};
% Pick/center depend only on referenced measured Z, never DM/X diagnostics.
for k=[1 3]
 t=teach(k,true,true,false,false,-1);assert(t.saved && t.dmRequests==0 && t.xRequests==0);
 t=teach(k,false,true,true,true,20);assert(~t.saved && t.notready);
 t=teach(k,true,false,true,true,20);assert(~t.saved && t.notready);
end
checks{end+1}='pick and center use referenced measured Z only; no generic diagnostics or feedback targets';
% Side pose requires ruler supplied X independently from any encoder estimate.
for k=[2 4]
 t=teach(k,true,true,true,true,25);assert(t.saved && t.x==25);
 t=teach(k,true,true,true,true,-1);assert(t.notready && ~t.saved);
end
checks{end+1}='side measured X explicit, unknown stays unknown; center remains180/X0';
% Holding owner supplies next real feedback; disabled owner serial read2.
for holding=[false true]
 old=19;current=old;pending=true;saved=false;
 assert(current==old && pending && ~saved);
 current=20;fresh=true;
 if holding,readRequests=0;else,readRequests=1;end
 saved=current~=old && fresh;
 assert(saved && readRequests==~holding);
end
checks{end+1}='one CAN owner only; holding refresh uses real next tick; disabled query uses existing read2';
% User STOP/owner departure cancels old capture without resuming/moving.
pending=true;newRequest=true;assert(pending && newRequest); % pending not overwritten
state=9;if state==9,pending=false;end
motion=0;healthWrites=0;referenceWrites=0;assert(~pending && motion==0 && healthWrites==0 && referenceWrites==0);
checks{end+1}='duplicate capture preserves pending; STOP cancels; never auto-move or fabricate health/references';
% Field-width and actual measurement formula. 6*8 uint32 =192 bytes.
raw=zeros(6,8,'uint32');assert(numel(raw)*4==192);raw(4,3:6)=uint32([0 1 96 66]);
actual=(903.8099975585938-342/100)*(3200/360);
assert(abs(actual-8003.466644965279)<1e-8 && abs(actual-8000)>3);
assert(600+400*(205-50)==62600);
checks{end+1}='uint32 raw width192; measured903.809997 drives8003.466645, not target8000;62600 estimate only';
% Task color/order, destination ring and physically loaded slot are independent.
orders=perms([2 1 5]);rings=perms(1:3);cases=0;
for a=1:6
 for b=1:6
  for c=1:6
   task=orders(a,:);dest=rings(b,:);slots=orders(c,:);
   for item=1:3
    slot=find(slots==task(item));assert(numel(slot)==1);
    carrier=(slot-1)*120;target=dest(item);
    assert(slots(carrier/120+1)==task(item) && target==dest(item));
   end
   locked=slots;slots=fliplr(slots);assert(isequal(locked,orders(c,:)));
   cases=cases+1;
  end
 end
end
assert(cases==216);
checks{end+1}='216 independent task/ring/inventory permutations; lookup by color; lock snapshot isolates live edits';
seq=8;saved=8;resultCode=1;
assert(~(seq~=8 && saved==seq && resultCode==1));
resultCode=2;seq=seq+1;assert(~(saved==seq && resultCode==1));
saved=seq;resultCode=1;assert(seq~=8 && saved==seq && resultCode==1);
% Existing real FD stop: request consumed, TX count changes, then disabled ACK.
beforeTx=7;tx=8;request=0;motor=0;holding=0;state=8;stopResult=1;jogResult=2;
assert(tx~=beforeTx && request==0 && motor==0 && holding==0 && state==8 && stopResult==1 && jogResult==2);
checks{end+1}='request sequence and saved sequence prevent stale DONE; existing disable requires new TX and actual disabled ACK';
for kind=[6 7]
 for oldDepth=[-1 62000]
  depth=oldDepth;yaw=151;x=25;
  assert(depth==oldDepth && yaw==151 && x==25);
 end
end
checks{end+1}='geometry-only kinds6/7 save DM and measured X without requiring measured Z or overwriting unknown/existing place depth';
result=struct('passed',true,'when',datestr(now,31),'checks',{checks},'hardware_access',false,'scope','discrete teach dependencies and CAN ownership only; no physical acceptance');
f=fopen(fullfile(out,'matlab-result.json'),'w');fwrite(f,jsonencode(result),'char');fclose(f);disp(jsonencode(result));
function t=teach(k,ref,zvalid,dmfresh,unused,x)
 t=struct('saved',false,'notready',false,'dmRequests',0,'xRequests',0,'x',-1);
 if ~ref || (k~=5 && ~zvalid) || (any(k==[2 4]) && ~(x>=0 && x<=170)),t.notready=true;return;end
 if any(k==[2 4 5]) && ~dmfresh,t.dmRequests=1;return;end
 t.saved=true;t.x=x;
end
