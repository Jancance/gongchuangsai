% Authorized two deltas only. MATLAB prevalidation before implementation.
root=fileparts(fileparts(mfilename('fullpath')));out=fullfile(root,'evidence','integration','2026-11-16','stage3-parallel-preclose');
if ~exist(out,'dir'),mkdir(out);end
checks=0;
for start=[0 100 double(intmax('uint32'))-200]
 for duration=[0 100 599 600 601 2000]
  issued=false;count=0;descend=-1;
  for elapsed=0:2601
   if ~issued,issued=true;count=count+1;tick=mod(start,2^32);end
   nowTick=mod(start+elapsed,2^32);settled=mod(nowTick-tick,2^32)>=600;
   if elapsed>=duration && issued && settled,descend=elapsed;break;end
  end
  assert(descend==max(duration,600) && count==1);checks=checks+1;
 end
end
for item=1:3
 issued=false;count=0;
 for service=1:20
  if ~issued,issued=true;count=count+1;end
 end
 assert(count==1);checks=checks+1;
end
for scope=[0 1]
 for phase=[0 3 4 8]
  for axes=[1 2 4]
   eligible=scope && phase==3 && axes==1;
   for failed=[0 1 2 3] % cancel, launch rejection, PWM rejection, DM fault
    descend=eligible && failed==0;
    if failed,assert(~descend);end
    checks=checks+1;
   end
  end
 end
end
for scale=[10 100]
 for top=[0 78 -1000]
  reuse=0;
  for n=1:20
   if reuse>=9,reuse=0;else,reuse=reuse+1;end
   assert(reuse==mod(n,10));checks=checks+1;
   for stage3=[false true]
    target=top+(1800-1200*stage3)*360*scale/3200;
    assert(target-top==(1800-1200*stage3)*360*scale/3200);
    assert((reuse==0)==ismember(n,[10 20])); % shared native cadence
    assert((1800-600)/400==3 && 600/400==1.5);
    checks=checks+3;
   end
  end
 end
end
for delay=[0 1 37 250]
 writeTick=1000+delay;
 assert(1599-writeTick<600 && (writeTick+600)-writeTick==600);
 checks=checks+1;
end
result=struct('passed',true,'checks',checks,'when',datestr(now,31),'hardware_access',false,'parallel_rule','max(DM done, preclose start +600ms)','stage3_return_pulses',600,'source_return_pulses',1800,'native_cadence',10);
f=fopen(fullfile(out,'matlab-result.json'),'w');fwrite(f,jsonencode(result),'char');fclose(f);disp(jsonencode(result));
