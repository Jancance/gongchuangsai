function verify_heading_inheritance
out=fileparts(mfilename('fullpath'));checks=0;
for origin=[-179 -90 0 17 90 179]
 for first=[-90 90]
  for error=[-3 -1 -.2 0 .2 1 3]
   state=struct('run',0,'valid',false,'target',0);
   [base,state]=begin(state,1,origin);assert(base==origin);checks=checks+1;
   planned=wrap(base+first);actual=wrap(planned+error);
   state=commit(state,1,planned,true);[next,~]=begin(state,1,actual);
   assert(abs(wrap(next-planned))<1e-10);checks=checks+1;
   second=wrap(next+first);assert(abs(wrap(second-wrap(origin+2*first)))<1e-10);checks=checks+1;
   assert(abs(wrap(next-actual)+error)<1e-10);checks=checks+1;
   bad=commit(state,1,wrap(planned+90),false);assert(isequal(bad,state));checks=checks+1;
   [fresh,~]=begin(state,2,actual);assert(fresh==actual);checks=checks+1;
   [unmapped,~]=begin(state,0,actual);assert(unmapped==actual);checks=checks+1;
   for direction=0:3
    % All directions retain one common desired heading, not actual yaw.
    assert(abs(wrap(next-actual)+error)<1e-10);checks=checks+1;
   end
  end
 end
end
summary=struct('status','PASS','checks',checks,'scope','Heading reference lifecycle model only; no vehicle dynamics or precision acceptance');
f=fopen(fullfile(out,'heading_matlab_summary.json'),'w');fprintf(f,'%s',jsonencode(summary));fclose(f);disp(summary);
end
function [base,s]=begin(s,run,yaw)
base=yaw;if run&&s.valid&&s.run==run,base=s.target;end
end
function s=commit(s,run,target,success)
if success,s=struct('run',run,'valid',run~=0,'target',wrap(target));end
end
function x=wrap(x)
x=mod(x+180,360)-180;
end
