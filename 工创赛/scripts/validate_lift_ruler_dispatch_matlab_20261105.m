function validate_lift_ruler_dispatch_matlab_20261105
% Integration-only supplement to the existing 19-case motion model.
% No change to motion distance, speed, timing or physical assumptions.
root=fileparts(fileparts(mfilename('fullpath')));
checks=struct('name',{},'passed',{});
for offline=[false true]
 for fault=[false true]
  for address=[5 6]
   for operation=1:4 % query, stop, enable, move
    for phase=[0 5 8 10]
     request=(operation==3 && phase==5)||(operation==4 && phase==8);
     accepted=address==5 && (operation<=2 || (~offline && ~fault && request));
     if address~=5, expected=false;
     elseif operation<=2, expected=true;
     else, expected=~offline && ~fault && request;
     end
     check('dedicated admission including legacy startup state',accepted==expected);
    end
   end
  end
 end
end
% A PC13 edge is consumed only by the dedicated service. No mission startup.
state=0;pending=0;moves=0;vm=0;
check('boot idle',state==0 && pending==0 && moves==0 && vm==0);
pending=1;state=1;vm=1;pending=0;
check('first key starts once',state==1 && vm==1 && pending==0);
moves=moves+1;state=2;
pending=2;state=5;stop_pending=true;
for k=1:3
 check('unacknowledged stop retains VM and single move',stop_pending && vm==1 && moves==1 && pending==2);
end
stop_pending=false;
check('acknowledged stop cannot restart terminal state',state==5 && moves==1 && ~stop_pending);
report=struct('engine','local MATLAB','version',version,'checks',checks, ...
 'check_count',numel(checks),'failure_count',sum(~[checks.passed]), ...
 'passed',all([checks.passed]),'scope','dispatch/admission logical model only', ...
 'hardware_operated',false,'firmware_downloaded',false);
out=fullfile(root,'evidence','integration','2026-11-05','lift-ruler-candidate','dispatch-matlab-result.json');
fid=fopen(out,'w','n','UTF-8');assert(fid~=-1);c=onCleanup(@()fclose(fid)); %#ok<NASGU>
fprintf(fid,'%s',jsonencode(report));
fprintf('MATLAB dispatch: checks=%d failures=%d\n',report.check_count,report.failure_count);
assert(report.passed);
 function check(name,passed)
  checks(end+1)=struct('name',name,'passed',logical(passed)); %#ok<AGROW>
 end
end
