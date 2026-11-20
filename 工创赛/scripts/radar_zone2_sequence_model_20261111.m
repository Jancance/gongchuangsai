function radar_zone2_sequence_model_20261111
% Fixture-only finite-state contract. Completion inputs are synthetic events.
checks=0; scenarios=0;
for wrap=0:1
 for mode=0:1
  s=initial(mode,wrap); s=step(s,0,'begin'); assert(s.state==1); checks=checks+1;
  for t=0:4, s=step(s,t,'none'); assert(s.state==1); checks=checks+1; end
  s=step(s,5,'none'); assert(s.state==2); checks=checks+1;
  s=step(s,6,'none'); assert(s.state==2); checks=checks+1;
  s=step(s,7,'done'); assert(s.state==3); checks=checks+1;
  s=step(s,11,'none'); assert(s.state==3); checks=checks+1;
  s=step(s,12,'none'); assert(s.state==4); checks=checks+1;
  s=step(s,13,'old'); assert(s.state==4); checks=checks+1;
  s=step(s,14,'begin'); assert(s.state==4 && s.commands==3); checks=checks+1;
  if mode==0
   s=step(s,15,'scanok');
  else
   orders=perms(1:3);
   for k=1:size(orders,1)
    q=s;
    for j=1:3
     events={'ack','calib','persist'}; q=step(q,14+j,events{orders(k,j)});
     if j<3, assert(q.state==4); else assert(q.state==5); end
     checks=checks+1;
    end
    assert(q.commands==4); checks=checks+1;
   end
   s=q;
  end
  assert(s.state==5); s=step(s,18,'none'); assert(s.state==5); checks=checks+2;
  s=step(s,19,'done'); assert(s.state==6); checks=checks+1;
  s=step(s,23,'none'); assert(s.state==6); checks=checks+1;
  s=step(s,24,'none'); assert(s.state==7); checks=checks+1;
  count=s.commands; s=step(s,25,'begin'); assert(s.commands==count); checks=checks+1;
  scenarios=scenarios+1;
 end
end
for stage=1:6
 for failure=1:4
  s=initial(0,0);s.state=stage;s.entered=0;s.commands=4;
  events={'cancel','unsafe','bad','scanbad'};
  if failure==4 && stage~=4, continue; end
  s=step(s,1,events{failure});assert(s.state==99 && s.commands==4);checks=checks+1;
  s=step(s,2,'done');assert(s.state==99 && s.commands==4);checks=checks+1;
  scenarios=scenarios+1;
 end
 s=initial(0,1);s.state=stage;s.entered=0;
 s=step(s,100,'none');assert(s.state~=99);checks=checks+1;
 s=initial(0,1);s.state=stage;s.entered=0;
 s=step(s,101,'none');assert(s.state==99);checks=checks+1;scenarios=scenarios+1;
end
s=initial(0,0);s.calibrated=false;s=step(s,0,'begin');assert(s.state==0 && s.commands==0);checks=checks+1;scenarios=scenarios+1;
s=initial(0,0);s=step(s,0,'begin');s=step(s,1,'begin');assert(s.state==1 && s.commands==1);checks=checks+1;scenarios=scenarios+1;
out=fullfile(fileparts(fileparts(mfilename('fullpath'))),'evidence','navigation-planning','2026-11-11','radar-zone2');
if ~exist(out,'dir'),mkdir(out);end
fid=fopen(fullfile(out,'matlab_sequence_result.json'),'w');assert(fid>=0);fprintf(fid,'{"passed":true,"version":2,"scenarios":%d,"checks":%d,"fixture_only":true,"physical_acceptance":false}',scenarios,checks);fclose(fid);
save(fullfile(out,'matlab_sequence_result.mat'),'scenarios','checks');
fprintf('RADAR_ZONE2_MATLAB_V2_PASS scenarios=%d checks=%d\n',scenarios,checks);
end
function s=initial(mode,wrap)
s=struct('state',0,'entered',0,'commands',0,'calibrated',true,'mode',mode,'seen',[false false false],'offset',double(wrap)*4294967290);
end
function s=step(s,t,event)
now=mod(s.offset+t,2^32); entered=mod(s.offset+s.entered,2^32);elapsed=mod(now-entered,2^32);
if s.state==0
 if strcmp(event,'begin') && s.calibrated, s.state=1;s.entered=t;s.commands=s.commands+1;end
 return;
end
if s.state>=7,return;end
if any(strcmp(event,{'cancel','unsafe','bad','scanbad'})) || elapsed>100,s.state=99;return;end
old=s.state;
switch s.state
 case 1,if elapsed>=5,s.state=2;s.commands=s.commands+1;end
 case 2,if strcmp(event,'done'),s.state=3;end
 case 3,if elapsed>=5,s.state=4;s.commands=s.commands+1;end
 case 4
  if s.mode==0 && strcmp(event,'scanok'),s.state=5;s.commands=s.commands+1;end
  if s.mode==1
   e=find(strcmp(event,{'ack','calib','persist'}));if ~isempty(e),s.seen(e)=true;end
   if all(s.seen),s.state=5;s.commands=s.commands+1;end
  end
 case 5,if strcmp(event,'done'),s.state=6;s.commands=s.commands+1;end
 case 6,if elapsed>=5,s.state=7;if s.mode==0,s.commands=s.commands+1;end,end
end
if old~=s.state,s.entered=t;end
end
