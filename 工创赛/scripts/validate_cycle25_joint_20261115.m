function validate_cycle25_joint_20261115
% Feasibility before new implementation. No new drive constants or hardware.
root=fileparts(fileparts(mfilename('fullpath')));
out=fullfile(root,'evidence','integration','2026-11-15','cycle25-joint');
if ~exist(out,'dir'),mkdir(out);end
d=jsondecode(fileread(fullfile(root,'evidence','integration','2026-11-15','direct-frame-trigger','pc13-watch-20261115-210111.json')));
f=d.final;piece=f.cycle_piece_ms(:)';mechanical=sum(piece);
lead=jsondecode(fileread(fullfile(root,'evidence','integration','2026-11-15','extension-lead-pick','timing-config.json')));
sync=jsondecode(fileread(fullfile(root,'evidence','integration','2026-11-15','original-speed-overlap','matlab-result.json')));
checks=0;
assert(f.g_source_cycle_state==8 && f.g_source_cycle_fault==0);checks=checks+1;
afterBeep=f.g_source_cycle_done-f.g_source_cycle_beep_done;
assert(afterBeep>=mechanical);checks=checks+1;
% Optimistic counterfactual: subtract EVERY programmed delay even though some
% can already overlap X travel. This is a favourable scenario, not a proven
% achievable minimum or validated new simultaneous-motion duration.
optimistic=mechanical-sum(lead.extension_lead_ms_by_region);
rows={};n=0;
for stops=0:3
 for stopSeconds=[0 1 2 4]
  for acquisitionMs=[100 200]
   for phaseWaitMs=[0 1000 4000]
    n=n+1;external=stops*stopSeconds*1000;
    total=optimistic+external+3*acquisitionMs+phaseWaitMs;
    rows{n}=struct('stop_events',stops,'stop_seconds_each',stopSeconds,...
     'two_frame_acquisition_ms_each',acquisitionMs,'prediction_phase_wait_ms_total',phaseWaitMs,...
     'optimistic_total_ms',total,'additional_saving_needed_ms',max(0,total-25000),...
     'meets25_under_declared_assumptions',total<=25000); %#ok<AGROW>
    assert(total>=optimistic && total>=external);checks=checks+2;
   end
  end
 end
end
% Requirements-based eligibility transitions, before selecting any numeric
% displacement/velocity gate. No new production threshold is approved here.
% columns: initial still, observed movement, observed changed position, still now
events=[1 0 0 1;1 1 0 0;1 1 1 0;1 1 1 1;0 1 1 0];
expected=[0 0 1 1 1];
for k=1:size(events,1)
 seenMotion=events(k,2);changed=events(k,3);
 eligible=seenMotion && changed;assert(eligible==expected(k));checks=checks+1;
 if eligible && events(k,4),mode='direct';elseif eligible,mode='predict';else,mode='observe';end
 if k==4,assert(strcmp(mode,'direct'));checks=checks+1;end
end
% The rejected first prediction is the actual observed pathological input.
bad=jsondecode(fileread(fullfile(root,'evidence','integration','2026-11-15','extension-lead-pick','abnormal-first-prediction.json')));
prediction=bad.first_prediction.prediction;
assert(min(prediction.start_delay_ms)>25000);checks=checks+1;
% Whole-cycle feasibility cannot approve unsafe descent merely for time gain.
assert(~sync.firmware_implementation_gate_passed);checks=checks+1;
conflicts=sync.results;
remaining=[conflicts.extension_remaining_at_clearance_sample_mm];
assert(all(remaining>5));checks=checks+1;
report=struct('diagnostic_completed',true,'local_matlab',true,'checks',checks,...
 'budget_cases',n,'observed_mechanical_ms',mechanical,'observed_after_beep_ms',afterBeep,...
 'observed_visual_wait_and_other_ms',afterBeep-mechanical,...
 'counterfactual_all_lead_delays_removed_ms',optimistic,...
 'counterfactual_is_not_a_validated_lower_bound',true,...
 'one4s_stop_no_acquisition_no_phase_wait_ms',optimistic+4000,...
 'three4s_stops_no_acquisition_no_phase_wait_ms',optimistic+12000,...
 'existing_clearance_x_remaining_range_mm',[min(remaining) max(remaining)],...
 'simultaneous_original_speed_full_descent_verified',false,...
 'cycle25_guaranteed',false,'implementation_gate_passed',false,...
 'eligibility_state_semantics_checked',true,'numeric_prediction_thresholds_not_selected',true,...
 'cases',[rows{:}],'hardware_access',false,...
 'reason','Even optimistic delay removal exceeds25s with one4s stop before observation/phase wait; original-speed simultaneous descent still has an unresolved clearance conflict.');
fp=fopen(fullfile(out,'matlab-result.json'),'w','n','UTF-8');fprintf(fp,'%s\n',jsonencode(report));fclose(fp);
fprintf('JOINT MATLAB diagnostic %u checks/%u cases: measured %.3fs, optimistic removal %.3fs, +one4s %.3fs, +three4s %.3fs. Implementation NOT approved by feasibility.\n',checks,n,mechanical/1000,optimistic/1000,(optimistic+4000)/1000,(optimistic+12000)/1000);
end
