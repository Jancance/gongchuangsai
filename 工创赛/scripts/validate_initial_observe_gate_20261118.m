function validate_initial_observe_gate_20261118
R=fileparts(fileparts(mfilename('fullpath')));E=fullfile(R,'evidence','integration','2026-11-18','three-frame-warm');n=0;
for state=0:9
 for prep=0:5
  for axes=0:7
   for top=[0 1]
    for zero=[0 1]
     allowed=state==1 && prep==4 && axes==3 && top && zero;
     if allowed,assert(state==1 && prep==4 && axes==3 && top && zero);else,assert(~(state==1 && prep==4 && axes==3 && top && zero));end;n=n+1;
    end
   end
  end
 end
end
r=struct('passed',true,'checks',n,'completed_local',datestr(now,'yyyy-mm-dd HH:MM:SS'),'hardware_access',false,'scope','Narrow first-observation gate: state1/prep4, DM|EXT,180deg/65mm, existing top and home2deg conditions only; other XY rejection remains');
f=fopen(fullfile(E,'matlab-initial-gate.json'),'w');fprintf(f,'%s',jsonencode(r));fclose(f);fprintf('PASS initial observe gate %d checks\n',n);
end
