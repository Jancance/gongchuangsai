function validate_home_direction_diag_20261117
R=fileparts(fileparts(mfilename('fullpath')));
E=fullfile(R,'evidence','integration','2026-11-17','home-direction-diag');if ~exist(E,'dir'),mkdir(E);end
checks=0;scenarios=0;
for direction=[0 1]
 for failure=0:11
  for packet=1:3
   for result=[4 5 6 7 9]
    for frozen=[0 1]
     [a,eventsA]=model(direction,failure,packet,result,false,false);
     [b,eventsB,d]=model(direction,failure,packet,result,true,frozen);
     assert(a==b && isequal(eventsA,eventsB));checks=checks+1;
     if frozen,assert(d.stage==99 && d.result==99 && d.frozen);else,assert(d.result==b && d.frozen==(b~=1));end;checks=checks+1;
     scenarios=scenarios+1;
    end
   end
  end
 end
end
r=struct('passed',true,'checks',checks,'scenarios',scenarios,'completed_local',datestr(now,'yyyy-mm-dd HH:MM:SS'),'hardware_access',false,'scope','Paired original/instrumented ordered control-flow model: exit values, CAN event sequences, write packets, same-direction skip and frozen diagnostics; not physical CAN');
f=fopen(fullfile(E,'matlab-result.json'),'w');fprintf(f,'%s',jsonencode(r));fclose(f);fprintf('PASS diagnostic model: %d scenarios, %d checks\n',scenarios,checks);
end
function [ret,events,d]=model(direction,failure,failedPacket,result,instrumented,frozen)
d=struct('stage',99,'result',99,'frozen',frozen);events=[];ret=1;
for stage=1:11
 if direction==1 && stage>=5 && stage<=7,continue;end
 if instrumented && ~d.frozen,d.stage=stage;end
 if stage==3 || stage==9,events(end+1)=hex2dec('22');end
 if stage==6
  for packet=1:3
   events(end+1)=hex2dec('4c00')+packet;
   if failure==stage && packet==failedPacket,ret=result;break;end
  end
 elseif failure==stage,ret=result;
 end
 if stage==7,events(end+1)=hex2dec('4c');end
 if ret~=1,break;end
end
if instrumented && ~d.frozen,d.result=ret;d.frozen=ret~=1;end
end
