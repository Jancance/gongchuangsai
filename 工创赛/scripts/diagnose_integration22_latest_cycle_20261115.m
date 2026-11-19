function diagnose_integration22_latest_cycle_20261115
% Read-only historical trace diagnostic. No drive access or new controller.
root=fileparts(fileparts(mfilename('fullpath')));
out=fullfile(root,'evidence','integration','2026-11-15','integration22-latest-cycle');
if ~exist(out,'dir'),mkdir(out);end
source=fullfile(root,'evidence','integration','2026-11-15','direct-frame-trigger','pc13-watch-20261115-210111.json');
d=jsondecode(fileread(source));t=d.trace;checks=0;rows={};
assert(d.final.g_source_cycle_state==8 && d.final.g_source_cycle_fault==0);checks=checks+1;
assert(d.host_motion_commands_sent==0);checks=checks+1;
ppm=315.5*360/3200;
for seq=1:3
 a=t([t.g_low_accepted_sequence]==seq & [t.g_low_phase]==4);
 b=t([t.g_low_accepted_sequence]==seq & [t.g_low_phase]==5);
 assert(numel(a)>2 && ~isempty(b));checks=checks+1;
 time=[a.g_low_elapsed_ms];z=[a.lift_depth_pulses];x=[a.g_startup_extension_current_deg];
 % Phase5 already retracts X, so use the settled end of phase4 as target.
 target=median(x(end-2:end));
 assert(abs(abs(target)/ppm-164.07083627876932)<5);checks=checks+1;
 err=abs(x-target)/ppm;
 % Existing5mm X and400-pulse Z windows only; no production thresholds added.
 xStable=find(err>5,1,'last')+1;
 assert(~isempty(xStable) && xStable<=numel(a));checks=checks+1;
 k=find(z>=15600-400,1);assert(k>1);checks=checks+1;
 % Remove the ACTUAL left-region810ms release delay only as a time-shift
 % counterfactual. This is not a new measured trajectory or validated model.
 shifted=[time(k-1) time(k)]-810;
 assert(shifted(1)>=time(1));checks=checks+1;
 bracket=find(time<=shifted(1),1,'last'):find(time>=shifted(2),1);
 shiftedErr=interp1(time,err,shifted,'linear');
 kFinal=find(abs(z-29700)<=400,1);assert(~isempty(kFinal));checks=checks+1;
 rows{seq}=struct('sequence',seq,'observed_region','left',...
  'target_extension_feedback_magnitude_mm',abs(target)/ppm,...
  'phase4_first_sample_ms',time(1),'x_stable_sample_ms',time(xStable),...
  'original_clearance_crossing_sample_bracket_ms',[time(k-1) time(k)],...
  'counterfactual_clearance_bracket_after_delay_removal_ms',shifted,...
  'counterfactual_x_error_interpolated_at_bracket_ends_mm',shiftedErr,...
  'recorded_x_error_range_in_overlapping_samples_mm',[min(err(bracket)) max(err(bracket))],...
  'first_z_within_existing400_pulses_sample_ms',time(kFinal),...
  'first_rise_phase_sample_ms',b(1).g_low_elapsed_ms,...
  'sampled_z_arrival_to_rise_phase_gap_ms',b(1).g_low_elapsed_ms-time(kFinal),...
  'gap_is_not_proven_removable_delay',true);
end
f=d.final;mechanical=sum(f.cycle_piece_ms);
after=f.g_source_cycle_done-f.g_source_cycle_beep_done;
report=struct('diagnostic_completed',true,'local_matlab',true,'checks',checks,...
 'source',source,'image_sha256',d.image_sha256,'rows',[rows{:}],...
 'mechanical_piece_ms',f.cycle_piece_ms,'mechanical_total_ms',mechanical,...
 'after_beep_total_ms',after,'observation_and_other_ms',after-mechanical,...
 'left_only_latest_run',true,'right_and_near_not_covered',true,...
 'time_shift_is_unvalidated_counterfactual',true,...
 'simultaneous_descent_implementation_gate_passed',false,...
 'cycle25_verified',false,'hardware_access',false,'runtime_changes',false);
fp=fopen(fullfile(out,'matlab-result.json'),'w','n','UTF-8');
fprintf(fp,'%s\n',jsonencode(report));fclose(fp);
fprintf('Latest cycle diagnostic: %u checks, %.3fs mechanical, %.3fs after beep.\n',checks,mechanical/1000,after/1000);
for i=1:3
 r=rows{i};fprintf('item%u left: shifted clearance X errors %.2f..%.2fmm; sampled final-depth to rise gap %ums (not proven removable).\n',i,min(r.counterfactual_x_error_interpolated_at_bracket_ends_mm),max(r.counterfactual_x_error_interpolated_at_bracket_ends_mm),r.sampled_z_arrival_to_rise_phase_gap_ms);
end
end
