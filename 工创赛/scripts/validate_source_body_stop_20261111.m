function validate_source_body_stop_20261111
% STOP调度状态模型；不模拟电机/总线，也不证明车辆物理停稳。
root=fileparts(fileparts(mfilename('fullpath')));
out=fullfile(root,'evidence','integration','2026-11-11','source-body-stop');
if ~exist(out,'dir'),mkdir(out);end
checks=struct('name',{},'passed',{}); scenarios=struct('name',{},'events',{},'expected',{});
% event: kind(1=update,2=take,3=report),tick,address/VM,result/fault,session offset.
events=[1 0 1 1 0];
for round=0:25
 t=round*500;
 if round>0,events(end+1,:)=[1 t 1 1 0];end
 for address=1:4
  events(end+1,:)=[2 t+address*2 0 0 0];
  events(end+1,:)=[3 t+address*2+1 address 8 0];
 end
 events(end+1,:)=[2 t+20 0 0 0];
end
e=run_case('all four fail beyond old20 retry limit',events);
check('failure never becomes ACKED',all(e(:,1)==1));
check('failure history preserved',e(end,2)==0 && e(end,3)==15);
check('retry counter saturates but STOP remains pending',e(end,6)==20 && e(end,4)==0);
check('all four wheels get a turn in every round',sum(e(:,10))==26*4);

events=[1 0 1 1 0];
for address=1:4
 events=[events;2 address*2 0 0 0;3 address*2+1 address (1+7*(address==1)) 0];
end
events=[events;2 20 0 0 0;1 499 1 1 0;2 499 0 0 0;1 500 1 1 0;2 501 0 0 0;3 502 1 1 0];
e=run_case('failed first wheel does not starve other wheels',events);
check('other three ACKs retained before retry',e(9,2)==14 && e(9,3)==1);
check('no retry transaction before500ms',e(10,10)==0 && e(12,10)==0);
check('only missing wheel retried',e(end-1,11)==1 && e(end-1,10)==1);
check('recovered all ACKs required for ACKED',e(end,1)==2 && e(end,2)==15);
check('recovery does not erase earlier failed wheel',e(end,3)==1);

events=[1 0 1 1 0;2 1 0 0 0;3 2 1 1 -1;3 3 2 1 0;3 4 1 1 0;2 5 0 0 0;3 6 2 1 0;2 7 0 0 0;3 8 3 1 0;2 9 0 0 0;3 10 4 1 0];
e=run_case('stale and wrong-address reports rejected',events);
check('old session and wrong wheel do not ACK',e(3,2)==0 && e(4,2)==0 && e(4,5)==1);
check('matching replies complete normally',e(end,1)==2 && e(end,2)==15);

events=[1 0 1 1 0;2 1 0 0 0;1 500 1 1 0;3 501 1 1 0;2 502 0 0 0;3 503 1 1 0];
e=run_case('timeout requeues unresolved wheel',events);
check('timed out inflight reply without a new take is ignored',e(4,2)==0);
check('timeout preserves pending and later accepts retry',e(end,1)==1 && e(end,2)==1);

events=[1 0 1 1 0;1 10 0 1 0;2 11 0 0 0;1 20 0 0 0;2 21 0 0 0;1 30 1 1 0];
e=run_case('VM off cancels transmission and new fault gets new session',events);
check('no STOP sent while VM request is off',e(3,10)==0 && e(5,10)==0);
check('VM off does not turn cancellation into ACK',e(2,1)==4 && e(4,1)==0);
check('new incident has new session',e(end,9)==2 && e(end,2)==0);

events=[1 4294967200 1 1 0;1 403 1 1 0;1 404 1 1 0];
e=run_case('uint32 tick rollover retains500ms cadence',events);
check('499ms wrap does not retry;500ms does',e(2,6)==0 && e(3,6)==1);
check('STOP retries never request VM cut in keep-power profile',all(arrayfun(@(s)all(s.expected(:,8)==0),scenarios)));
r=struct('engine','local MATLAB','release',version('-release'),'passed',all([checks.passed]), ...
 'check_count',numel(checks),'failure_count',sum(~[checks.passed]),'checks',checks, ...
 'scenarios',scenarios,'physical_stop_verified',false,'hardware_access',false);
f=fopen(fullfile(out,'matlab-result.json'),'w','n','UTF-8');assert(f~=-1);fprintf(f,'%s\n',jsonencode(r));fclose(f);
save(fullfile(out,'matlab-result.mat'),'r');
fprintf('MATLAB STOP model: %d checks, %d failures, %d fixture scenarios. No hardware claim.\n',numel(checks),r.failure_count,numel(scenarios));
assert(r.passed);

 function check(name,passed)
  checks(end+1)=struct('name',name,'passed',logical(passed));
 end
 function expected=run_case(name,ev)
  state=0;ack=0;failed=0;pending=0;inflight=0;retries=0;cause=0;cut=0;session=0;seen=0;vm=0;start=0;
  expected=zeros(size(ev,1),12);
  for n=1:size(ev,1)
   kind=ev(n,1);now=ev(n,2);taken=0;address=0;returned_session=0;
   if kind==1
    vm=ev(n,3);fault=ev(n,4);
    if ~fault
     if ~vm,state=0;cut=0;cause=0;pending=0;inflight=0;seen=0;retries=0;end
    elseif vm~=1
     if state==1,state=4;pending=0;end
    else
     if ~seen
      seen=1;session=session+1;state=1;ack=0;failed=0;cause=0;pending=15;inflight=0;retries=0;start=now;
     end
     if state==1 && mod(now-start,2^32)>=500
      cause=2;retries=min(20,retries+1);pending=bitand(15,bitxor(ack,15));inflight=0;start=now;
      if pending==0,state=2;end
     end
    end
   elseif kind==2
    if state==1 && vm==1 && inflight==0
     for wheel=1:4
      bit=2^(wheel-1);
      if bitand(pending,bit),pending=pending-bit;inflight=bit;taken=1;address=wheel;returned_session=session;break;end
     end
    end
   else
    wheel=ev(n,3);result=ev(n,4);reported_session=session+ev(n,5);
    if wheel>=1 && wheel<=4
     bit=2^(wheel-1);
     if reported_session==session && inflight==bit
      inflight=0;
      if result==1,ack=bitor(ack,bit);else,failed=bitor(failed,bit);cause=1;end
      if ack==15 && state==1,state=2;end
     end
    end
   end
   expected(n,:)=[state ack failed pending inflight retries cause cut session taken address returned_session];
  end
  scenarios(end+1)=struct('name',name,'events',ev,'expected',expected);
 end
end
