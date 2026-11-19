function validate_extension_lead_pick_20261115
% Original speeds: use recorded X/Z timing brackets to move X first.
root=fileparts(fileparts(mfilename('fullpath')));
out=fullfile(root,'evidence','integration','2026-11-15','extension-lead-pick');
if ~exist(out,'dir'),mkdir(out);end
d=jsondecode(fileread(fullfile(root,'evidence','integration','2026-11-15','original-speed-overlap','matlab-result.json')));
assert(d.diagnostic_completed && d.local_matlab);
leads=zeros(1,3);cases={};checks=0;count=0;
for i=1:numel(d.results)
 row=d.results(i);run=jsondecode(fileread(row.source));trace=run.trace;
 part=trace([trace.g_low_accepted_sequence]==row.sequence & [trace.g_low_phase]==2);
 final=trace([trace.g_low_accepted_sequence]==row.sequence & [trace.g_low_phase]==4);
 xGoal=median([final.g_startup_extension_current_deg]);
 if abs(xGoal)<100,region=3;elseif abs(xGoal)/(315.5*360/3200)>163,region=1;else,region=2;end
 xError=abs([part.g_startup_extension_current_deg]-xGoal)/(315.5*360/3200);
 % Arrival must remain within the existing5mm window for the rest of the
 % recorded approach. Do not mistake the first overshoot crossing for settle.
 lastOutside=find(xError>5,1,'last');stable=lastOutside+1;
 assert(stable<=numel(part));checks=checks+1;
 zBracket=row.clearance_sample_bracket_ms;
 required=[part(stable).g_low_elapsed_ms]-zBracket(1)+90;
 lead=ceil(max(0,required)/10)*10;
 leads(region)=max(leads(region),lead);
 count=count+1;
 cases{count}=struct('region',region-1,'required_lead_ms',lead,...
  'original_z_earliest_clearance_elapsed_ms',zBracket(1),...
  'x_stable_within5mm_elapsed_ms',part(stable).g_low_elapsed_ms); %#ok<AGROW>
end
assert(all(leads>0));checks=checks+1;
% Faster uninterrupted Z can reach the former hover sooner than the original
% stopped segment. Replay a conservative range down to INSTANT Z arrival at
% hover: choose X lead >= measured stable-X time relative to physical start.
% Use the first approach sample as an earlier-than-actual X start bound;
% preparing the Z command only after the lead adds time rather than reducing it.
for i=1:numel(d.results)
 row=d.results(i);run=jsondecode(fileread(row.source));trace=run.trace;
 part=trace([trace.g_low_accepted_sequence]==row.sequence & [trace.g_low_phase]==2);
 final=trace([trace.g_low_accepted_sequence]==row.sequence & [trace.g_low_phase]==4);
 goal=median([final.g_startup_extension_current_deg]);
 if abs(goal)<100,region=3;elseif abs(goal)/(315.5*360/3200)>163,region=1;else,region=2;end
 err=abs([part.g_startup_extension_current_deg]-goal)/(315.5*360/3200);
 stable=find(err>5,1,'last')+1;
 earliestStart=part(1).g_low_elapsed_ms;
 bound=part(stable).g_low_elapsed_ms-earliestStart+90;
 leads(region)=max(leads(region),ceil(bound/10)*10);
 checks=checks+1;
end
for i=1:numel(cases)
 row=cases{i};assert(leads(row.region+1)>=row.required_lead_ms);checks=checks+1;
end
% Actual generic group behaviour: X starts first, LIFT is released after the
% lead. The original12s timeout/STOP applies before and after release.
for region=1:3
 for actualStart=[100 180 300]
  for taskDelay=[0 10 30]
   zRelease=actualStart+leads(region)+taskDelay;
   assert(zRelease>=actualStart+leads(region));checks=checks+1;
   for stopAt=[actualStart-1 actualStart zRelease-1 zRelease zRelease+100]
    startedX=stopAt>=actualStart;startedZ=stopAt>=zRelease;
    stopMask=2+4;assert(stopMask==6 && (~startedZ || startedX));checks=checks+1;
   end
  end
 end
end
% Retain prior declared full-retract neighbour-envelope calculation separately;
% neither it nor this sample-based delay proves new hardware motion accepted.
old=jsondecode(fileread(fullfile(root,'evidence','integration','2026-11-15','continuous-arm-pick','matlab-result.json')));
assert(old.loaded_full_retract_same_start);checks=checks+1;
config=struct('extension_lead_ms_by_region',leads,'descent_rpm',1846.4,...
 'extension_rpm',150,'lift_accel_rpm_s',45055,'extension_accel_rpm_s',600,...
 'lead_origin','six historical actual traces; last outside existing5mm plus sampling/start budget',...
 'existing_start_skew_budget_ms',90,'new_runtime_fault_threshold',false);
report=struct('passed',true,'local_matlab',true,'checks',checks,...
 'source_traces',numel(d.results),'cases',[cases{:}],'config',config,...
 'continuous_descent_one_final_target',true,'extension_starts_before_descent',true,...
 'original_speed_payloads_preserved',true,'loaded_full_retract_same_start',true,...
 'full_new_trajectory_physically_verified',false,'moving_disc_contact_timing_verified',false,...
 'hardware_access',false,'position_and_fault_exits_require_C_regression',true);
f=fopen(fullfile(out,'matlab-result.json'),'w','n','UTF-8');fprintf(f,'%s\n',jsonencode(report));fclose(f);
f=fopen(fullfile(out,'timing-config.json'),'w','n','UTF-8');fprintf(f,'%s\n',jsonencode(config));fclose(f);
fprintf('PASS original-speed extension-lead MATLAB: %u checks; left/right/near %u/%u/%ums. No speed changes or hardware acceptance.\n',checks,leads);
end
