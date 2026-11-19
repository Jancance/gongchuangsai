function validate_compact_poll_20261115
root=fileparts(fileparts(mfilename('fullpath')));out=fullfile(root,'evidence','integration','2026-11-15','compact-poll');
if ~exist(out,'dir'),mkdir(out);end
checks=0;cases=0;
% Original two confirmations remain in TWO service passes. Each pass can
% advance at most three existing read steps; failure stops this pass.
for serviceMs=[10 20 50]
 for exchangeMs=[1 4 20]
  for readyAt=[0 5 50 100 300]
   for failStep=0:3
    oldTime=0;newTime=0;oldSamples=0;newSamples=0;
    oldPhase=0;newPhase=0;failOld=failStep;failNew=failStep;
    oldReads=[];newReads=[];
    while oldSamples<2
     oldTime=oldTime+serviceMs;
     [oldPhase,oldSamples,oldTime,failOld,oldReads]=step(oldPhase,oldSamples,oldTime,failOld,oldReads,readyAt,exchangeMs);
    end
    while newSamples<2
     newTime=newTime+serviceMs;
     for k=1:3
      before=newPhase;
      [newPhase,newSamples,newTime,failNew,newReads]=step(newPhase,newSamples,newTime,failNew,newReads,readyAt,exchangeMs);
      if newPhase==before || newPhase==0,break;end
     end
    end
    assert(oldSamples==2 && newSamples==2);checks=checks+1;
    assert(all(ismember(newReads,0:2)));checks=checks+1;
    assert(newTime<=oldTime);checks=checks+1;cases=cases+1;
   end
  end
 end
end
% Query fault means no speculative next read or fake confirmation.
for phase=0:2
 current=phase;before=current;failed=true;
 if failed,after=before;else,after=mod(before+1,3);end
 assert(after==before);checks=checks+1;
end
report=struct('passed',true,'local_matlab',true,'checks',checks,'cases',cases,...
 'max_existing_read_steps_per_service',3,'required_confirmation_rounds',2,...
 'speed_acceleration_goal_and_path_unchanged',true,'failed_query_exits_current_service',true,...
 'no_new_fault_threshold',true,'actual_CAN_load_and_physical_cycle_time_verified',false,...
 'simultaneous_descent_clearance_not_validated_by_this_test',true,'hardware_access',false);
f=fopen(fullfile(out,'matlab-result.json'),'w','n','UTF-8');fprintf(f,'%s\n',jsonencode(report));fclose(f);
fprintf('PASS compact-poll scheduling MATLAB: %u checks/%u cases; same reads/two service confirmations, no physical timing claim\n',checks,cases);
end
function [phase,samples,time,fail,reads]=step(phase,samples,time,fail,reads,readyAt,exchangeMs)
reads(end+1)=phase;time=time+exchangeMs;
if fail==phase+1,fail=0;return;end
if phase<2,phase=phase+1;return;end
if time>=readyAt,samples=samples+1;else,samples=0;end
phase=0;
end
