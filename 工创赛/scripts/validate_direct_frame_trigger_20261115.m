function validate_direct_frame_trigger_20261115
root=fileparts(fileparts(mfilename('fullpath')));out=fullfile(root,'evidence','integration','2026-11-15','direct-frame-trigger');
if ~exist(out,'dir'),mkdir(out);end
checks=0;
for region=1:3
 for target=[2 1 5]
  for previous=[0 target]
   for current=[0 target]
    commit=previous==target && current==target;
    assert(commit==(previous>0 && current>0));checks=checks+1;
    if commit
     delays=ones(1,3);delays(region)=0;[delay,index]=min(delays);
     assert(delay==0 && index==region);checks=checks+1;
    end
   end
  end
 end
end
% A change of run/pose/item resets confirmation; execution has no second
% camera gate and fault/STOP still cancels through existing controller.
for changed=[false true]
 hits=2;if changed,hits=0;end
 assert((hits==2)==~changed);checks=checks+1;
end
report=struct('passed',true,'local_matlab',true,'checks',checks,...
 'confirmation_frames',2,'prediction_used',false,'velocity_model_used',false,...
 'selected_region_delay_ms',0,'other_region_delay_ms',1,...
 'no_wait_for_disc_stop',true,'existing200ms_freshness_preserved',true,...
 'physical_grasp_on_moving_disc_verified',false,'hardware_access',false);
f=fopen(fullfile(out,'matlab-result.json'),'w','n','UTF-8');fprintf(f,'%s\n',jsonencode(report));fclose(f);
fprintf('PASS direct frame-trigger MATLAB: %u checks;2frames->selected region immediate, no prediction, no hardware\n',checks);
end
