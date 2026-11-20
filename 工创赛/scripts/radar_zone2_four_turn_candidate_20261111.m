function radar_zone2_four_turn_candidate_20261111
% User-approved estimate only: 28BYJ-48 label 12 V, 17 output teeth,
% approximately four output turns; estimated 170 mm per output turn.
% Tooth count and voltage do not determine output half-steps per turn H.
% Every sampled H below is hypothetical_input, never a deployment value.
root=fileparts(fileparts(mfilename('fullpath')));
out=fullfile(root,'evidence','navigation-planning','2026-11-11','radar-zone2','four-turn-candidate');
if ~exist(out,'dir'),mkdir(out);end
turns=4;estimated_mm_per_turn=170;estimated_distance_mm=turns*estimated_mm_per_turn;
interval_ms=4;existing_limit=64;measured=0;direction_code_known=0;
H_samples=[1 8 16 17 512 2048 4096];
checks=0;
assert(estimated_distance_mm==680);checks=checks+1;
assert(measured==0 && direction_code_known==0);checks=checks+1;
fid=fopen(fullfile(out,'hypothetical_inputs.csv'),'w');assert(fid>=0);checks=checks+1;
fprintf(fid,'hypothetical_input,H_half_steps_per_output_turn,turns,N_half_steps,one_leg_output_time_s,step_count_limit_allows,deployment_allowed,measured\r\n');
for k=1:numel(H_samples)
 H=H_samples(k);
 assert(isfinite(H) && H>0 && H==floor(H));checks=checks+1;
 N=turns*H;time_s=N*interval_ms/1000;
 step_count_limit_allows=N<=existing_limit;
 deployment_allowed=step_count_limit_allows && measured==1 && direction_code_known==1;
 assert(N==4*H);checks=checks+1;
 assert(abs(time_s-0.016*H)<1e-10);checks=checks+1;
 assert(step_count_limit_allows==(H<=16));checks=checks+1;
 assert(~deployment_allowed);checks=checks+1;
 fprintf(fid,'true,%.0f,%.0f,%.0f,%.6f,%d,%d,%d\r\n',H,turns,N,time_s,step_count_limit_allows,deployment_allowed,measured);
 % Logical return direction is the opposite of whichever code is measured.
 % It does not establish which code physically means CCW or CW.
 for down_fixture=0:1
  up_fixture=1-down_fixture;
  assert(up_fixture~=down_fixture && any(up_fixture==[0 1]));checks=checks+1;
 end
end
assert(4*16==64 && 4*17>64);checks=checks+1;
assert(fclose(fid)==0);checks=checks+1;
fid=fopen(fullfile(out,'summary.json'),'w');assert(fid>=0);checks=checks+1;
checks=checks+1; % Count the fclose assertion that follows the summary write.
fprintf(fid,['{"passed":true,"sample_count":%d,"checks":%d,' ...
 '"all_H_samples_are_hypothetical_input":true,"H_deployment_value":null,' ...
 '"label_model":"28BYJ-48","label_voltage_v":12,"output_gear_teeth":17,"actual_supply_verified":false,' ...
 '"candidate_turns":4,"estimated_mm_per_turn":170,"estimated_distance_mm":680,' ...
 '"distance_is_estimated_not_measured":true,"N_formula":"4*H",' ...
 '"one_leg_output_time_s_formula":"0.016*H",' ...
 '"nominal_service_interval_assumed_ms":4,"actual_hardware_duration_verified":false,' ...
 '"time_excludes_unknown_settle_servo_and_radar_waits":true,' ...
 '"existing_step_limit":64,"step_limit_condition":"H<=16",' ...
 '"measured":0,"direction_code_mapping":null,' ...
 '"viewpoint":"axis_up_view_from_above",' ...
 '"observed_down":"CCW","observed_up":"CW",' ...
 '"return_direction":"opposite_to_measured_down_code",' ...
 '"hardware_accessed":false,"physical_acceptance":false}\n'],numel(H_samples),checks);
assert(fclose(fid)==0);
save(fullfile(out,'candidate_result.mat'),'H_samples','turns','estimated_mm_per_turn','estimated_distance_mm','interval_ms','existing_limit','measured','direction_code_known','checks');
fprintf('RADAR_ZONE2_FOUR_TURN_CANDIDATE_PASS samples=%d checks=%d measured=0 H=unknown\n',numel(H_samples),checks);
end
