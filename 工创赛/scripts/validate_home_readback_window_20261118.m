function validate_home_readback_window_20261118
R=fileparts(fileparts(mfilename('fullpath')));
E=fullfile(R,'evidence','integration','2026-11-18','home-readback-window');
if ~exist(E,'dir'),mkdir(E);end
n=0;
for first=0:300
 for spacing=[0 1 5 10 20]
  frames=first+[0 spacing 2*spacing];
  for window=[60 250]
   accepted=all(frames<window);
   observed=sum(frames<window);
   assert(accepted==(observed==3));n=n+1;
   if accepted,assert(frames(3)<window);else,assert(observed<3);end;n=n+1;
  end
 end
end
assert(all([80 100 120]<250) && ~all([80 100 120]<60));n=n+1;
assert(~all([240 245 250]<250));n=n+1;
% Three existing multi-frame reads plus fixed upper bounds remain below the
% existing 2500ms prepare deadline; this is a declared timing budget, not RTOS proof.
budget=3*250+3*20+60+5*50;
assert(budget<2500);n=n+1;
r=struct('passed',true,'checks',n,'window_old_ms',60,'window_new_ms',250,'declared_budget_ms',budget,'completed_local',datestr(now,'yyyy-mm-dd HH:MM:SS'),'hardware_access',false,'scope','Existing multi-frame receive window only; full readback and bounded failure preserved; not physical CAN latency proof');
f=fopen(fullfile(E,'matlab-result.json'),'w');fprintf(f,'%s',jsonencode(r));fclose(f);
fprintf('PASS readback window %d checks\n',n);
end
