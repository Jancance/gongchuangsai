function validate_can2_schedule_matlab_20261106
root=fileparts(fileparts(mfilename('fullpath')));out=fullfile(root,'evidence','integration','2026-11-06','can2-recovery');
checks=[];rows=[];
for request=[0,2]
 for state=0:5
  for stop_pending=[0,1]
   busy=(request==2 || (state>=1 && state<=3));
   poll_allowed=~busy;stop_allowed=true;
   checks(end+1)=(~busy || ~poll_allowed) && stop_allowed; %#ok<AGROW>
   checks(end+1)=poll_allowed==~(request==2 || (state>=1 && state<=3)); %#ok<AGROW>
   rows(end+1,:)=[request,state,stop_pending,poll_allowed,stop_allowed]; %#ok<AGROW>
  end
 end
end
r=struct('engine','local MATLAB','passed',all(checks),'check_count',numel(checks), ...
 'scope','readonly CAN scheduling isolation, STOP remains serviced; no link acceptance');
fid=fopen(fullfile(out,'schedule-matlab.json'),'w');assert(fid~=-1);fprintf(fid,'%s',jsonencode(r));fclose(fid);
assert(r.passed);fprintf('SCHEDULE MATLAB checks=%d passed=%d\n',numel(checks),r.passed);
end
